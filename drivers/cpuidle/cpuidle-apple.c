// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU idle for Apple SoCs, which have no PSCI: WFI, twice.
 *
 * Without a cpuidle driver the idle loop never reaches the s2idle path, so
 * suspend-to-idle keeps the tick running and does not suspend timekeeping:
 * CLOCK_MONOTONIC counts the sleep, and every service watchdog armed before a
 * long sleep expires on wake. The core only enters a state above index 0 for
 * s2idle, and only one with a non-zero exit latency, hence the second state.
 * It is the same WFI; both keep CPU state and the timers running, so the
 * timer needs no broadcast.
 */

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#include <asm/proc-fns.h>

static __cpuidle int apple_cpuidle_wfi(struct cpuidle_device *dev,
				       struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static struct cpuidle_driver apple_cpuidle_driver = {
	.name = "apple_cpuidle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter = apple_cpuidle_wfi,
			.exit_latency = 1,
			.target_residency = 1,
			.name = "WFI",
			.desc = "WFI",
		},
		{
			.enter = apple_cpuidle_wfi,
			.enter_s2idle = apple_cpuidle_wfi,
			.exit_latency = 1,
			.target_residency = 1000,
			.name = "WFI-S2IDLE",
			.desc = "WFI, for suspend-to-idle",
		},
	},
	.state_count = 2,
};

/* The SoCs whose suspend-to-idle has been tested with this. */
static const char *const apple_cpuidle_socs[] __initconst = {
	"apple,t8010",
	NULL
};

static int __init apple_cpuidle_init(void)
{
	if (!of_machine_compatible_match(apple_cpuidle_socs))
		return -ENODEV;

	return cpuidle_register(&apple_cpuidle_driver, NULL);
}
device_initcall(apple_cpuidle_init);
