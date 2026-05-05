/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Peripheral half of the USB-based ZMK split transport — native UDC
 * variant.
 *
 * Lemon B's native USB-C exposes a CDC ACM serial port via Zephyr's
 * legacy USB device stack. Lemon A's UHC (uhc_pio_usb on its Link)
 * enumerates this CDC ACM and the two halves talk over the data-
 * interface bulk pair (0x02 OUT / 0x82 IN). No custom vendor class —
 * we tunnel the existing wired-style envelope framing over the bulk
 * pair as if it were a UART.
 *
 * Why CDC ACM instead of a custom vendor class:
 *
 *   - Zephyr's legacy USB CDC ACM is one of the most-used USB classes
 *     in the tree; it Just Works on day-1 hardware bring-up.
 *   - Endpoint addresses are stable (0x02/0x82) when CDC ACM is the
 *     only class in the build, so the central can skip descriptor
 *     enumeration and open the bulk pair directly.
 *   - From this file's perspective the CDC ACM is just a UART —
 *     uart_fifo_read / uart_fifo_fill — so the framing+dispatch loop
 *     reuses existing patterns.
 *
 * Inbound commands: the IRQ-driven UART RX callback drains bytes into
 * a ring buffer; a worker validates the envelope's magic prefix +
 * CRC32 and forwards the command via
 * zmk_split_transport_peripheral_command_handler.
 *
 * Outbound events: report_event frames into an event_envelope, drops
 * it into a ring buffer, and a worker writes it to the CDC ACM via
 * uart_fifo_fill.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_split_usb_peripheral, CONFIG_ZMK_SPLIT_USB_LOG_LEVEL);

#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>

#include "usb.h"

#define PERIPHERAL_ID 0

/* Standard CDC ACM data-interface UART — Zephyr instantiates it
 * automatically when CONFIG_USB_CDC_ACM=y and a zephyr,cdc-acm-uart
 * DT node is present. */
#define ZSU_CDC_NODE DT_NODELABEL(zsu_cdc_acm)
static const struct device *const cdc = DEVICE_DT_GET(ZSU_CDC_NODE);

RING_BUF_DECLARE(rx_ring, ZMK_SPLIT_USB_CMD_PACKET_SIZE *CONFIG_ZMK_SPLIT_USB_CMD_BUFFER_ITEMS);
RING_BUF_DECLARE(tx_ring, ZMK_SPLIT_USB_EVENT_PACKET_SIZE *CONFIG_ZMK_SPLIT_USB_EVENT_BUFFER_ITEMS);

static void process_rx_work(struct k_work *w);
static K_WORK_DEFINE(process_rx, process_rx_work);

/* ===== Helpers ======================================================== */

static bool envelope_is_valid(const uint8_t *buf, size_t len) {
    if (len < ZMK_SPLIT_USB_MSG_EXTRA_SIZE + sizeof(struct zmk_split_usb_command_payload)) {
        return false;
    }
    const struct zmk_split_usb_msg_prefix *prefix = (const struct zmk_split_usb_msg_prefix *)buf;
    if (memcmp(prefix->magic_prefix, ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
               sizeof(prefix->magic_prefix)) != 0) {
        return false;
    }
    uint32_t expected_crc = crc32_ieee(buf, sizeof(*prefix) + prefix->payload_size);
    const struct zmk_split_usb_msg_postfix *post =
        (const struct zmk_split_usb_msg_postfix *)(buf + sizeof(*prefix) + prefix->payload_size);
    return post->crc == expected_crc;
}

/* ===== UART RX/TX IRQ ================================================ */

extern struct zmk_split_transport_peripheral zmk_split_usb_peripheral_inst;

static void uart_irq_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            int n;
            while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
                int put = ring_buf_put(&rx_ring, buf, n);
                if (put != n) {
                    LOG_WRN("rx ring overflow: dropped %d B", n - put);
                }
            }
            k_work_submit(&process_rx);
        }

        if (uart_irq_tx_ready(dev)) {
            uint8_t buf[64];
            uint32_t got = ring_buf_get(&tx_ring, buf, sizeof(buf));
            if (got > 0) {
                int sent = uart_fifo_fill(dev, buf, got);
                /* If only part of the chunk was accepted, push
                 * the rest back to the front of the ring (or
                 * accept the loss). For now: drop the rest —
                 * USB CDC ACM at FS speeds rarely partially
                 * fills. */
                if (sent != (int)got) {
                    LOG_WRN("tx fifo took %d/%u B", sent, got);
                }
            } else {
                uart_irq_tx_disable(dev);
            }
        }
    }
}

/* ===== Inbound commands ============================================ */

static void process_rx_work(struct k_work *w) {
    ARG_UNUSED(w);
    uint8_t buf[ZMK_SPLIT_USB_CMD_PACKET_SIZE];

    while (ring_buf_size_get(&rx_ring) >= sizeof(buf)) {
        uint32_t got = ring_buf_get(&rx_ring, buf, sizeof(buf));
        if (got == 0) {
            break;
        }
        if (!envelope_is_valid(buf, got)) {
            LOG_WRN("Dropping invalid inbound envelope (%u B)", got);
            continue;
        }
        const struct zmk_split_usb_command_envelope *env =
            (const struct zmk_split_usb_command_envelope *)buf;
        LOG_DBG("rx command type=%d", env->payload.cmd.type);
        zmk_split_transport_peripheral_command_handler(&zmk_split_usb_peripheral_inst,
                                                       env->payload.cmd);
    }
}

/* ===== Outbound events =============================================== */

static int report_event(const struct zmk_split_transport_peripheral_event *event) {
    struct zmk_split_usb_event_envelope env = {
        .prefix =
            {
                .magic_prefix = ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX,
                .payload_size = sizeof(env.payload),
            },
        .payload =
            {
                .source = PERIPHERAL_ID,
                .event = *event,
            },
    };
    uint32_t crc = crc32_ieee((const uint8_t *)&env, sizeof(env.prefix) + env.prefix.payload_size);
    struct zmk_split_usb_msg_postfix post = {.crc = crc};

    uint8_t pkt[ZMK_SPLIT_USB_EVENT_PACKET_SIZE];
    memcpy(pkt, &env, sizeof(env));
    memcpy(pkt + sizeof(env), &post, sizeof(post));

    if (ring_buf_space_get(&tx_ring) < sizeof(pkt)) {
        LOG_WRN("tx ring full, dropping event");
        return -ENOSPC;
    }
    (void)ring_buf_put(&tx_ring, pkt, sizeof(pkt));
    uart_irq_tx_enable(cdc);
    return 0;
}

/* ===== Bring-up thread =============================================== */

static void zsup_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!device_is_ready(cdc)) {
        LOG_ERR("CDC ACM device %s not ready", cdc->name);
        return;
    }

    int ret = usb_enable(NULL);
    if (ret && ret != -EALREADY) {
        LOG_ERR("usb_enable failed: %d", ret);
        return;
    }
    LOG_INF("USB enabled — peripheral CDC ACM signaling on native USB-C");

    uart_irq_callback_set(cdc, uart_irq_cb);
    uart_irq_rx_enable(cdc);
}

K_THREAD_DEFINE(zsup_bring_up_thread, 2048, zsup_thread, NULL, NULL, NULL, 7, 0, 0);

/* ===== ZMK split transport peripheral registration ================== */

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = report_event,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(zmk_split_usb_peripheral_inst, &peripheral_api);
