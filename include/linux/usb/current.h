/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_USB_CURRENT_H
#define __LINUX_USB_CURRENT_H

#include <linux/types.h>

/**
 * enum usb_current_reason - provenance of a USB current report
 * @USB_CURRENT_UNKNOWN: Legacy or otherwise unclassified report
 * @USB_CURRENT_RESET: USB bus reset
 * @USB_CURRENT_CONFIGURED: Successful nonzero SET_CONFIGURATION
 * @USB_CURRENT_UNCONFIGURED: Successful zero SET_CONFIGURATION
 * @USB_CURRENT_SUSPEND: USB bus suspend
 * @USB_CURRENT_RESUME: USB bus resume
 * @USB_CURRENT_DISCONNECT: USB bus disconnect
 * @USB_CURRENT_VENDOR: Vendor-specific current request or adjustment
 * @USB_CURRENT_LOST: Reports were lost; the current grant is not known
 *
 * UNKNOWN, VENDOR and LOST are not standard USB host current grants.
 */
enum usb_current_reason {
	USB_CURRENT_UNKNOWN,
	USB_CURRENT_RESET,
	USB_CURRENT_CONFIGURED,
	USB_CURRENT_UNCONFIGURED,
	USB_CURRENT_SUSPEND,
	USB_CURRENT_RESUME,
	USB_CURRENT_DISCONNECT,
	USB_CURRENT_VENDOR,
	USB_CURRENT_LOST,
};

/**
 * struct usb_current_event - immutable current report from a USB producer
 * @session: Nonzero device-session token assigned before enabling the role
 * @sequence: Nonzero monotonically increasing sequence within the session
 * @milliamps: Reported current in milliamperes
 * @reason: Provenance captured when the report was produced
 *
 * The producer captures the session, sequence and reason at the source, not
 * when queued work is delivered. Consumers must not reinterpret an unknown
 * or vendor report as a standard host grant based on its current value.
 */
struct usb_current_event {
	u64 session;
	u64 sequence;
	unsigned int milliamps;
	enum usb_current_reason reason;
};

#endif /* __LINUX_USB_CURRENT_H */
