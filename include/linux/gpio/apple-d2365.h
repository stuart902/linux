/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GPIO_APPLE_D2365_H
#define _LINUX_GPIO_APPLE_D2365_H

#include <linux/types.h>

struct device;

/* Caller must hold a managed device link to the bound event provider. */
bool apple_d2365_gpio_events_ready(struct device *dev);
/* Terminal shutdown handoff after readiness validation. Sleepable context
 * only, never the parent IRQ thread or one of its nested handlers. The caller
 * must exclude subsequent suspend/resume and hold the dependency through its
 * last PMIC access. Ownership is not restored; the parent IRQ remains allocated
 * for normal driver-core cleanup until reset/off. Repeated handoff is allowed.
 */
void apple_d2365_gpio_quiesce(struct device *dev);

#endif
