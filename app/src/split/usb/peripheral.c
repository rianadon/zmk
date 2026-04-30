/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Peripheral half of the USB-based split transport. Registers a
 * USBD-next vendor class on the device-side UDC (typically
 * udc_pio_usb on a Lemon Wired Link USB-C port) with one interface
 * exposing a bulk OUT (commands from the central) and a bulk IN
 * (events to the central).
 *
 * Inbound commands arrive via the class request callback, which hands
 * them to a worker that validates the envelope's magic prefix + CRC
 * and forwards the command to ZMK's split transport layer via
 * zmk_split_transport_peripheral_command_handler.
 *
 * Outbound events come in via report_event from the higher layer; we
 * frame them into an event_envelope and queue a bulk IN transfer
 * through the USBD class buffer API.
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_split_usb_peripheral, CONFIG_ZMK_SPLIT_USB_LOG_LEVEL);

#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>

#include "usb.h"

/* Per-side identifier — single peripheral; multi-peripheral configs
 * would need per-instance IDs. */
#define PERIPHERAL_ID 0

/* Inbound command staging: USBD class request callback runs in a USBD
 * worker thread but should not block, so we pass envelopes through a
 * ring buffer and process them in a dedicated work item. */
RING_BUF_DECLARE(rx_ring, ZMK_SPLIT_USB_CMD_PACKET_SIZE *
				 CONFIG_ZMK_SPLIT_USB_CMD_BUFFER_ITEMS);

/* Outbound event staging: report_event may be called from any thread,
 * we serialize into this ring and a worker drains it onto bulk-IN. */
RING_BUF_DECLARE(tx_ring, ZMK_SPLIT_USB_EVENT_PACKET_SIZE *
				 CONFIG_ZMK_SPLIT_USB_EVENT_BUFFER_ITEMS);

static struct usbd_class_node *zsu_class_node;
static atomic_t bulk_in_busy;

static void process_rx_work(struct k_work *w);
static K_WORK_DEFINE(process_rx, process_rx_work);

static void drain_tx_work(struct k_work *w);
static K_WORK_DEFINE(drain_tx, drain_tx_work);

/* ===== USBD vendor class descriptor =================================
 *
 * Single interface, two bulk endpoints. wMaxPacketSize is left zero so
 * the USBD stack fills it in from the UDC's reported MPS at runtime. */

struct zmk_split_usb_peripheral_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_out_ep;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_desc_header nil_desc;
} __packed;

static struct zmk_split_usb_peripheral_desc zsu_desc = {
	.if0 = {
		.bLength = sizeof(struct usb_if_descriptor),
		.bDescriptorType = USB_DESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_BCC_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	.if0_out_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = ZMK_SPLIT_USB_BULK_OUT_EP,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = 0,
		.bInterval = 0x00,
	},
	.if0_in_ep = {
		.bLength = sizeof(struct usb_ep_descriptor),
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = ZMK_SPLIT_USB_BULK_IN_EP,
		.bmAttributes = USB_EP_TYPE_BULK,
		.wMaxPacketSize = 0,
		.bInterval = 0x00,
	},
	.nil_desc = {
		.bLength = 0,
		.bDescriptorType = 0,
	},
};

/* ===== Helpers ======================================================== */

static bool envelope_is_valid(const uint8_t *buf, size_t len)
{
	if (len < ZMK_SPLIT_USB_MSG_EXTRA_SIZE +
		      sizeof(struct zmk_split_usb_command_payload)) {
		return false;
	}
	const struct zmk_split_usb_msg_prefix *prefix =
		(const struct zmk_split_usb_msg_prefix *)buf;
	if (memcmp(prefix->magic_prefix, ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
		   sizeof(prefix->magic_prefix)) != 0) {
		return false;
	}
	uint32_t expected_crc = crc32_ieee(
		buf, sizeof(*prefix) + prefix->payload_size);
	const struct zmk_split_usb_msg_postfix *post =
		(const struct zmk_split_usb_msg_postfix *)(buf + sizeof(*prefix) +
							   prefix->payload_size);
	return post->crc == expected_crc;
}

/* ===== Inbound (bulk OUT) command path ============================== */

static int post_bulk_out(void); /* forward */

static void process_rx_work(struct k_work *w)
{
	ARG_UNUSED(w);
	uint8_t buf[ZMK_SPLIT_USB_CMD_PACKET_SIZE];

	while (ring_buf_size_get(&rx_ring) >=
	       ZMK_SPLIT_USB_MSG_EXTRA_SIZE +
		       sizeof(struct zmk_split_usb_command_payload)) {
		size_t got = ring_buf_get(&rx_ring, buf, sizeof(buf));
		if (got == 0) {
			break;
		}
		if (!envelope_is_valid(buf, got)) {
			LOG_WRN("Dropping invalid inbound envelope (%zu B)", got);
			continue;
		}
		const struct zmk_split_usb_command_envelope *env =
			(const struct zmk_split_usb_command_envelope *)buf;
		LOG_DBG("rx command type=%d", env->payload.cmd.type);
		extern struct zmk_split_transport_peripheral
			zmk_split_usb_peripheral_inst;
		zmk_split_transport_peripheral_command_handler(
			&zmk_split_usb_peripheral_inst, env->payload.cmd);
	}
	(void)post_bulk_out();
}

/* Hand a fresh empty buffer to the USBD stack so the next bulk OUT
 * packet has somewhere to land. The class request callback returns
 * each filled buffer back here (after we copy it into the ring). */
static int post_bulk_out(void)
{
	if (zsu_class_node == NULL) {
		return -EAGAIN;
	}
	struct net_buf *buf = usbd_ep_buf_alloc(zsu_class_node,
						ZMK_SPLIT_USB_BULK_OUT_EP,
						ZMK_SPLIT_USB_CMD_PACKET_SIZE);
	if (buf == NULL) {
		LOG_WRN("usbd_ep_buf_alloc OUT failed");
		return -ENOMEM;
	}
	int ret = usbd_ep_enqueue(zsu_class_node, buf);
	if (ret) {
		LOG_ERR("usbd_ep_enqueue OUT failed: %d", ret);
		usbd_ep_buf_free(zsu_class_node->data->uds_ctx, buf);
	}
	return ret;
}

/* ===== Outbound (bulk IN) event path ================================ */

static int try_send_one_event(void)
{
	if (atomic_get(&bulk_in_busy)) {
		return 0;
	}
	uint8_t pkt[ZMK_SPLIT_USB_EVENT_PACKET_SIZE];
	size_t len = ring_buf_get(&tx_ring, pkt, sizeof(pkt));
	if (len == 0) {
		return 0;
	}
	if (zsu_class_node == NULL) {
		return -EAGAIN;
	}
	struct net_buf *buf = usbd_ep_buf_alloc(zsu_class_node,
						ZMK_SPLIT_USB_BULK_IN_EP, len);
	if (buf == NULL) {
		return -ENOMEM;
	}
	net_buf_add_mem(buf, pkt, len);
	atomic_set(&bulk_in_busy, 1);
	int ret = usbd_ep_enqueue(zsu_class_node, buf);
	if (ret) {
		LOG_ERR("usbd_ep_enqueue IN failed: %d", ret);
		atomic_clear(&bulk_in_busy);
		usbd_ep_buf_free(zsu_class_node->data->uds_ctx, buf);
	}
	return ret;
}

static void drain_tx_work(struct k_work *w)
{
	ARG_UNUSED(w);
	(void)try_send_one_event();
}

static int report_event(const struct zmk_split_transport_peripheral_event *event)
{
	struct zmk_split_usb_event_envelope env = {
		.prefix = {
			.magic_prefix = ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
			.payload_size = sizeof(env.payload),
		},
		.payload = {
			.source = PERIPHERAL_ID,
			.event = *event,
		},
	};
	uint32_t crc = crc32_ieee((const uint8_t *)&env,
				  sizeof(env.prefix) + env.prefix.payload_size);
	struct zmk_split_usb_msg_postfix post = {.crc = crc};

	uint8_t pkt[ZMK_SPLIT_USB_EVENT_PACKET_SIZE];
	memcpy(pkt, &env, sizeof(env));
	memcpy(pkt + sizeof(env), &post, sizeof(post));

	if (ring_buf_space_get(&tx_ring) < sizeof(pkt)) {
		LOG_WRN("tx ring full, dropping event");
		return -ENOSPC;
	}
	(void)ring_buf_put(&tx_ring, pkt, sizeof(pkt));
	k_work_submit(&drain_tx);
	return 0;
}

/* ===== USBD class API ================================================ */

static void zsu_update(struct usbd_class_node *c_nd, uint8_t iface,
		       uint8_t alternate)
{
	ARG_UNUSED(c_nd);
	LOG_DBG("interface %u alt %u", iface, alternate);
}

static int zsu_control_to_dev(struct usbd_class_node *c_nd,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	ARG_UNUSED(c_nd);
	ARG_UNUSED(buf);
	/* No vendor control requests defined for the split protocol —
	 * everything goes over the bulk pair. Reject gracefully. */
	LOG_DBG("Unhandled vendor control request 0x%x", setup->bRequest);
	errno = -ENOTSUP;
	return 0;
}

static int zsu_control_to_host(struct usbd_class_node *c_nd,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	ARG_UNUSED(c_nd);
	ARG_UNUSED(buf);
	LOG_DBG("Unhandled vendor control request 0x%x", setup->bRequest);
	errno = -ENOTSUP;
	return 0;
}

/* Endpoint completion. Fired for both bulk IN (transmit done) and
 * bulk OUT (data received). */
static int zsu_request(struct usbd_class_node *c_nd, struct net_buf *buf,
		       int err)
{
	struct udc_buf_info *bi = (struct udc_buf_info *)net_buf_user_data(buf);
	uint8_t ep = bi->ep;
	LOG_DBG("ep 0x%02x len %u err %d", ep, buf->len, err);

	if (USB_EP_DIR_IS_OUT(ep)) {
		if (err == 0 && buf->len > 0) {
			if (ring_buf_space_get(&rx_ring) >= buf->len) {
				ring_buf_put(&rx_ring, buf->data, buf->len);
				k_work_submit(&process_rx);
			} else {
				LOG_WRN("rx ring full, dropping inbound packet");
			}
		}
		usbd_ep_buf_free(c_nd->data->uds_ctx, buf);
		/* Re-arm bulk OUT for the next packet. */
		(void)post_bulk_out();
	} else {
		atomic_clear(&bulk_in_busy);
		usbd_ep_buf_free(c_nd->data->uds_ctx, buf);
		k_work_submit(&drain_tx);
	}
	return 0;
}

static int zsu_init(struct usbd_class_node *c_nd)
{
	zsu_class_node = c_nd;
	LOG_INF("Class init complete; arming bulk OUT");
	(void)post_bulk_out();
	return 0;
}

static struct usbd_class_api zsu_api = {
	.update = zsu_update,
	.control_to_dev = zsu_control_to_dev,
	.control_to_host = zsu_control_to_host,
	.request = zsu_request,
	.init = zsu_init,
};

static struct usbd_class_data zsu_class_data = {
	.desc = (struct usb_desc_header *)&zsu_desc,
	.v_reqs = NULL,
};

USBD_DEFINE_CLASS(zmk_split_usb, &zsu_api, &zsu_class_data);

/* ===== ZMK split transport peripheral registration ================== */

static const struct zmk_split_transport_peripheral_api peripheral_api = {
	.report_event = report_event,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(zmk_split_usb_peripheral_inst,
					&peripheral_api);
