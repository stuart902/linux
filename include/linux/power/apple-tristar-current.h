/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_APPLE_TRISTAR_CURRENT_H
#define _LINUX_POWER_APPLE_TRISTAR_CURRENT_H

#include <linux/usb/current.h>

struct device;

/* Read one immutable, production-stamped UDC event from the actual bound
 * Tristar owner. Returns 1/event, 0/empty, or an error. A nonzero session is
 * an owned USB role identity, NOT an Apple effective-source epoch. Revocation
 * or missing initialization returns ENODATA; loss is sticky until a new role
 * session. The caller owns a device reference; the function excludes unbind.
 *
 * This is a producer stream, not a ready shutdown-source snapshot. A source
 * consumer must establish full Dock/backend history and serialize publication
 * with owner invalidation. A copied event alone cannot establish that lifetime.
 */
int apple_tristar_take_usb_current(struct device *dev, u64 *session,
				  struct usb_current_event *event);

#endif
