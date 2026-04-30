/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Central half of the USB-based split transport. Drives the host-side
 * UHC (typically uhc_pio_usb on a Lemon Wired Link USB-C port) through
 * the bring-up dance for one peripheral half:
 *
 *   1. Register UHC event callback
 *   2. uhc_init / uhc_enable
 *   3. On UHC_EVT_DEV_CONNECTED_FS: bus_reset, then SET_ADDRESS via a
 *      control transfer to give the peripheral a non-zero device
 *      address. We hardcode address 1 (single-peripheral USB split).
 *   4. Open hardcoded bulk endpoints (ZMK_SPLIT_USB_BULK_OUT_EP /
 *      _IN_EP) — the peripheral's class descriptor matches these.
 *   5. Continually arm a bulk IN read; on completion validate the
 *      envelope and dispatch event via
 *      zmk_split_transport_central_peripheral_event_handler.
 *   6. send_command frames the command into a command_envelope and
 *      queues a bulk OUT UHC transfer.
 *
 * No descriptor enumeration: this is a pinned vendor-class pair, so
 * the central skips GET_DESCRIPTOR(DEVICE/CONFIG) and trusts the
 * known shape. If the bus partner isn't a matching peripheral, bulk
 * transfers will fail and the central reports them as such — but no
 * harm done to other devices since we never set address until after
 * connect.
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/uhc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_split_usb_central, CONFIG_ZMK_SPLIT_USB_LOG_LEVEL);

#include <zmk/split/transport/central.h>
#include <zmk/split/transport/types.h>

#include "usb.h"

/* DT compatible: a single zmk,usb-split node with a uhc phandle pointing
 * at the UHC controller (usually raspberrypi,pio-usb-host). */
#define DT_DRV_COMPAT zmk_usb_split

#if !DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#error                                                                                             \
	"USB split transport selected but no devicetree node with compatible 'zmk,usb-split' found"
#endif

#define UHC_PHANDLE DT_INST_PHANDLE(0, uhc)
static const struct device *const uhc_dev = DEVICE_DT_GET(UHC_PHANDLE);

#define PERIPHERAL_DEV_ADDR 1
#define MPS                 64

static atomic_t connected;
static atomic_t enumerated;

NET_BUF_POOL_FIXED_DEFINE(xfer_pool,
			  CONFIG_ZMK_SPLIT_USB_CMD_BUFFER_ITEMS +
				  CONFIG_ZMK_SPLIT_USB_EVENT_BUFFER_ITEMS + 4,
			  ZMK_SPLIT_USB_EVENT_PACKET_SIZE, 0, NULL);

K_FIFO_DEFINE(rx_evt_fifo);

static void central_thread(void *p1, void *p2, void *p3);
K_THREAD_DEFINE(zmk_split_usb_central_thread, 2048, central_thread, NULL, NULL,
		NULL, 7, 0, 0);
static struct k_sem connect_sem;
static struct k_sem xfer_done;

static void event_dispatch_work(struct k_work *w);
K_WORK_DEFINE(event_dispatch, event_dispatch_work);

/* ===== UHC event callback ============================================ */

static int uhc_event(const struct device *dev, const struct uhc_event *const evt)
{
	ARG_UNUSED(dev);
	switch (evt->type) {
	case UHC_EVT_DEV_CONNECTED_FS:
	case UHC_EVT_DEV_CONNECTED_LS:
	case UHC_EVT_DEV_CONNECTED_HS:
		atomic_set(&connected, 1);
		k_sem_give(&connect_sem);
		break;
	case UHC_EVT_DEV_REMOVED:
		atomic_clear(&connected);
		atomic_clear(&enumerated);
		break;
	case UHC_EVT_EP_REQUEST:
		/* Wake the central thread waiting on this transfer. */
		k_sem_give(&xfer_done);
		if (evt->xfer && evt->xfer->buf && evt->xfer->err == 0 &&
		    USB_EP_DIR_IS_IN(evt->xfer->ep) &&
		    (evt->xfer->ep & 0x7F) ==
			    (ZMK_SPLIT_USB_BULK_IN_EP & 0x7F)) {
			/* Hand the completed bulk-IN buffer to the
			 * deferred dispatcher so we don't run the upper
			 * layer in this driver context. */
			net_buf_put(&rx_evt_fifo, evt->xfer->buf);
			k_work_submit(&event_dispatch);
		}
		break;
	default:
		break;
	}
	return 0;
}

/* ===== Helpers ======================================================== */

static int issue_setup(const struct usb_setup_packet *setup,
		       struct net_buf *data_buf)
{
	struct uhc_transfer xfer = {
		.addr = (atomic_get(&enumerated) ? PERIPHERAL_DEV_ADDR : 0),
		.ep = USB_CONTROL_EP_OUT,
		.attrib = USB_EP_TYPE_CONTROL,
		.mps = MPS,
		.timeout = 1000,
		.buf = data_buf,
		.stage = UHC_CONTROL_STAGE_SETUP,
	};
	memcpy(xfer.setup_pkt, setup, sizeof(xfer.setup_pkt));
	int ret = uhc_ep_enqueue(uhc_dev, &xfer);
	if (ret) {
		LOG_ERR("setup enqueue failed: %d", ret);
		return ret;
	}
	if (k_sem_take(&xfer_done,
		       K_MSEC(CONFIG_ZMK_SPLIT_USB_TRANSFER_TIMEOUT_MS))) {
		LOG_WRN("setup timed out");
		(void)uhc_ep_dequeue(uhc_dev, &xfer);
		return -ETIMEDOUT;
	}
	return xfer.err;
}

static int set_address(uint8_t addr)
{
	struct usb_setup_packet setup = {
		.RequestType =
			{
				.recipient = USB_REQTYPE_RECIPIENT_DEVICE,
				.type = USB_REQTYPE_TYPE_STANDARD,
				.direction = USB_REQTYPE_DIR_TO_DEVICE,
			},
		.bRequest = USB_SREQ_SET_ADDRESS,
		.wValue = sys_cpu_to_le16(addr),
		.wIndex = 0,
		.wLength = 0,
	};
	return issue_setup(&setup, NULL);
}

static int set_configuration(uint8_t cfg)
{
	struct usb_setup_packet setup = {
		.RequestType =
			{
				.recipient = USB_REQTYPE_RECIPIENT_DEVICE,
				.type = USB_REQTYPE_TYPE_STANDARD,
				.direction = USB_REQTYPE_DIR_TO_DEVICE,
			},
		.bRequest = USB_SREQ_SET_CONFIGURATION,
		.wValue = sys_cpu_to_le16(cfg),
		.wIndex = 0,
		.wLength = 0,
	};
	return issue_setup(&setup, NULL);
}

static bool envelope_is_valid(const uint8_t *buf, size_t len)
{
	if (len < ZMK_SPLIT_USB_MSG_EXTRA_SIZE +
		      sizeof(struct zmk_split_usb_event_payload)) {
		return false;
	}
	const struct zmk_split_usb_msg_prefix *prefix =
		(const struct zmk_split_usb_msg_prefix *)buf;
	if (memcmp(prefix->magic_prefix, ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
		   sizeof(prefix->magic_prefix)) != 0) {
		return false;
	}
	uint32_t expected = crc32_ieee(buf, sizeof(*prefix) + prefix->payload_size);
	const struct zmk_split_usb_msg_postfix *post =
		(const struct zmk_split_usb_msg_postfix *)(buf + sizeof(*prefix) +
							   prefix->payload_size);
	return post->crc == expected;
}

/* ===== Bulk paths ==================================================== */

extern const struct zmk_split_transport_central zmk_split_usb_central_inst;

static int arm_bulk_in(void)
{
	struct net_buf *buf = net_buf_alloc(&xfer_pool, K_NO_WAIT);
	if (buf == NULL) {
		return -ENOMEM;
	}
	struct uhc_transfer *xfer = k_calloc(1, sizeof(*xfer));
	if (xfer == NULL) {
		net_buf_unref(buf);
		return -ENOMEM;
	}
	xfer->addr = PERIPHERAL_DEV_ADDR;
	xfer->ep = ZMK_SPLIT_USB_BULK_IN_EP;
	xfer->attrib = USB_EP_TYPE_BULK;
	xfer->mps = MPS;
	xfer->buf = buf;
	int ret = uhc_ep_enqueue(uhc_dev, xfer);
	if (ret) {
		LOG_ERR("bulk-in enqueue failed: %d", ret);
		net_buf_unref(buf);
		k_free(xfer);
	}
	return ret;
}

static void event_dispatch_work(struct k_work *w)
{
	ARG_UNUSED(w);
	struct net_buf *buf;
	while ((buf = net_buf_get(&rx_evt_fifo, K_NO_WAIT)) != NULL) {
		if (envelope_is_valid(buf->data, buf->len)) {
			const struct zmk_split_usb_event_envelope *env =
				(const struct zmk_split_usb_event_envelope *)buf->data;
			zmk_split_transport_central_peripheral_event_handler(
				&zmk_split_usb_central_inst,
				env->payload.source, env->payload.event);
		} else {
			LOG_WRN("Dropping invalid event envelope (%u B)", buf->len);
		}
		net_buf_unref(buf);
	}
	/* Always re-arm the bulk-IN read so the peripheral can keep
	 * sending events. */
	if (atomic_get(&enumerated)) {
		(void)arm_bulk_in();
	}
}

static int send_command(uint8_t source,
			struct zmk_split_transport_central_command cmd)
{
	if (!atomic_get(&enumerated)) {
		LOG_WRN("send_command but peripheral not enumerated");
		return -ENOTCONN;
	}
	if (source != 0) {
		return -EINVAL;
	}

	struct zmk_split_usb_command_envelope env = {
		.prefix = {
			.magic_prefix = ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
			.payload_size = sizeof(env.payload),
		},
		.payload = {
			.source = source,
			.cmd = cmd,
		},
	};
	uint32_t crc = crc32_ieee((const uint8_t *)&env,
				  sizeof(env.prefix) + env.prefix.payload_size);
	struct zmk_split_usb_msg_postfix post = {.crc = crc};

	struct net_buf *buf = net_buf_alloc(&xfer_pool, K_NO_WAIT);
	if (buf == NULL) {
		return -ENOMEM;
	}
	net_buf_add_mem(buf, &env, sizeof(env));
	net_buf_add_mem(buf, &post, sizeof(post));

	struct uhc_transfer *xfer = k_calloc(1, sizeof(*xfer));
	if (xfer == NULL) {
		net_buf_unref(buf);
		return -ENOMEM;
	}
	xfer->addr = PERIPHERAL_DEV_ADDR;
	xfer->ep = ZMK_SPLIT_USB_BULK_OUT_EP;
	xfer->attrib = USB_EP_TYPE_BULK;
	xfer->mps = MPS;
	xfer->buf = buf;
	int ret = uhc_ep_enqueue(uhc_dev, xfer);
	if (ret) {
		LOG_ERR("bulk-out enqueue failed: %d", ret);
		net_buf_unref(buf);
		k_free(xfer);
	}
	return ret;
}

static int get_available_source_ids(uint8_t *sources)
{
	sources[0] = 0;
	return 1;
}

static const struct zmk_split_transport_central_api central_api = {
	.send_command = send_command,
	.get_available_source_ids = get_available_source_ids,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(zmk_split_usb_central_inst, &central_api);

/* ===== Bring-up thread ============================================== */

static void bring_up(void)
{
	int ret;

	LOG_INF("Resetting bus");
	ret = uhc_bus_reset(uhc_dev);
	if (ret) {
		LOG_ERR("bus_reset failed: %d", ret);
		return;
	}
	/* USB §9.2.6.3: at least 10ms after reset before talking. */
	k_msleep(50);

	LOG_INF("SET_ADDRESS %u", PERIPHERAL_DEV_ADDR);
	ret = set_address(PERIPHERAL_DEV_ADDR);
	if (ret) {
		LOG_ERR("SET_ADDRESS failed: %d", ret);
		return;
	}
	atomic_set(&enumerated, 1);
	k_msleep(2);

	LOG_INF("SET_CONFIGURATION 1");
	ret = set_configuration(1);
	if (ret) {
		LOG_ERR("SET_CONFIGURATION failed: %d", ret);
		return;
	}

	LOG_INF("Arming bulk-IN read");
	(void)arm_bulk_in();
}

static void central_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	int ret;

	k_sem_init(&connect_sem, 0, 1);
	k_sem_init(&xfer_done, 0, 1);

	if (!device_is_ready(uhc_dev)) {
		LOG_ERR("UHC %s not ready", uhc_dev->name);
		return;
	}

	ret = uhc_init(uhc_dev, uhc_event);
	if (ret) {
		LOG_ERR("uhc_init failed: %d", ret);
		return;
	}
	ret = uhc_enable(uhc_dev);
	if (ret) {
		LOG_ERR("uhc_enable failed: %d", ret);
		return;
	}
	LOG_INF("USB split central up; waiting for peripheral");

	while (1) {
		k_sem_take(&connect_sem, K_FOREVER);
		if (!atomic_get(&connected)) {
			continue;
		}
		bring_up();
	}
}
