/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GPIO_APPLE_D2365_H
#define _LINUX_GPIO_APPLE_D2365_H

#include <linux/types.h>

struct device;

/* Caller must hold a managed device link to the bound event provider. */
bool apple_d2365_gpio_events_ready(struct device *dev);
void apple_d2365_gpio_quiesce(struct device *dev);

#endif
