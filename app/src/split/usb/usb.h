/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * USB-based ZMK split transport. Mirrors the wired/UART transport's
 * envelope framing — a magic prefix + length, command/event payload,
 * CRC32 postfix — but carries the framed bytes over a USB bulk pair
 * instead of UART:
 *
 *   Central side: USB host (typically uhc_pio_usb on a Lemon Wired
 *     Link USB-C port). Submits bulk OUT transfers carrying
 *     command_envelope's, polls bulk IN transfers for event_envelope's.
 *
 *   Peripheral side: USB device (typically udc_pio_usb on the matching
 *     Link port of the other half). Registers a vendor-class function
 *     with bulk in/out endpoints, receives commands on bulk OUT,
 *     reports events on bulk IN.
 *
 * Central and peripheral compile against the same envelope structs so
 * the magic prefix + CRC verification is symmetric.
 */
#pragma once

#include <stdint.h>
#include <zephyr/sys/util.h>
#include <zmk/split/transport/types.h>

/* Use the same magic prefix as the wired transport — clients can
 * validate either transport with the same bytes. The framing is also
 * identical (prefix, payload_size, payload, CRC32) so a packet capture
 * looks the same. */
#define ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX "ZmKw"

struct zmk_split_usb_msg_prefix {
	uint8_t magic_prefix[sizeof(ZMK_SPLIT_USB_ENVELOPE_MAGIC_PREFIX) - 1];
	uint8_t payload_size;
} __packed;

struct zmk_split_usb_command_payload {
	uint8_t source;
	struct zmk_split_transport_central_command cmd;
} __packed;

struct zmk_split_usb_command_envelope {
	struct zmk_split_usb_msg_prefix prefix;
	struct zmk_split_usb_command_payload payload;
} __packed;

struct zmk_split_usb_event_payload {
	uint8_t source;
	struct zmk_split_transport_peripheral_event event;
} __packed;

struct zmk_split_usb_event_envelope {
	struct zmk_split_usb_msg_prefix prefix;
	struct zmk_split_usb_event_payload payload;
} __packed;

struct zmk_split_usb_msg_postfix {
	uint32_t crc;
} __packed;

#define ZMK_SPLIT_USB_MSG_EXTRA_SIZE \
	(sizeof(struct zmk_split_usb_msg_prefix) + sizeof(struct zmk_split_usb_msg_postfix))

/* Buffer sizing — both sides use the same envelopes so transmit and
 * receive paths are symmetric. Bulk transfers are sized to fit one
 * envelope plus framing overhead. The peripheral additionally batches
 * up to ZMK_SPLIT_USB_EVENT_BUFFER_ITEMS events while waiting for the
 * central to read; the central queues up to
 * ZMK_SPLIT_USB_CMD_BUFFER_ITEMS commands. */
#define ZMK_SPLIT_USB_CMD_PACKET_SIZE                                                              \
	(sizeof(struct zmk_split_usb_command_envelope) + sizeof(struct zmk_split_usb_msg_postfix))
#define ZMK_SPLIT_USB_EVENT_PACKET_SIZE                                                            \
	(sizeof(struct zmk_split_usb_event_envelope) + sizeof(struct zmk_split_usb_msg_postfix))

/* Endpoint addresses used by both sides — hardcoded so the central
 * doesn't need to enumerate the descriptor before opening them. The
 * peripheral's class descriptor matches these. */
#define ZMK_SPLIT_USB_BULK_OUT_EP 0x01
#define ZMK_SPLIT_USB_BULK_IN_EP  0x81

/* Vendor / product IDs the central uses to identify the peripheral.
 * The same magic also goes in the device descriptor's bcdDevice field
 * so a bus-watcher can tell halves apart from any other CDC ACM. */
#define ZMK_SPLIT_USB_VID 0x2FE3
#define ZMK_SPLIT_USB_PID 0x5AC0
