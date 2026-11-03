// SPDX-License-Identifier: GPL-2.0-only
/* Default-off adapter for the separate Apple-derived core. No board calls it.
 * Source mapping and provider teardown contracts must be verified first.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/irqflags.h>
#include <asm/processor.h>
#include <linux/module.h>
#include <linux/power/apple-d2365-reference.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/slab.h>

struct d2365_adapter {
	struct device *dev;
	struct regmap *regmap;
	struct d2365_reference_resources resources;
	struct d2365_result result;
};

struct d2365_provider_ref {
	struct device *dev;
	struct module *owner;
};

static void d2365_put_provider(void *data)
{
	struct d2365_provider_ref *ref = data;

	module_put(ref->owner);
	put_device(ref->dev);
}

static int d2365_pin_provider(struct device *consumer, struct device *provider)
{
	struct d2365_provider_ref *ref;
	struct device_link *link;

	if (!provider || provider == consumer)
		return -EINVAL;
	ref = devm_kzalloc(consumer, sizeof(*ref), GFP_KERNEL);
	if (!ref)
		return -ENOMEM;
	/* A parent's probe may populate this consumer while holding its own
	 * lock. Defer instead of recursively taking that lock and deadlocking.
	 */
	if (!device_trylock(provider))
		return -EPROBE_DEFER;
	/* device_is_bound() remains true while an unbinding supplier drops
	 * its lock to detach consumers. Do not add a late dependency there.
	 */
	if (!device_is_bound(provider) ||
	    provider->links.status != DL_DEV_DRIVER_BOUND ||
	    !try_module_get(provider->driver->owner)) {
		device_unlock(provider);
		return -EPROBE_DEFER;
	}
	ref->owner = provider->driver->owner;
	ref->dev = get_device(provider);
	/* The consumer is probing. Keep the supplier bound until its managed
	 * dependency exists; failed-probe cleanup belongs to driver core.
	 */
	link = device_link_add(consumer, provider, DL_FLAG_AUTOREMOVE_CONSUMER);
	device_unlock(provider);
	if (!link) {
		module_put(ref->owner);
		put_device(ref->dev);
		return -EINVAL;
	}
	if (devm_add_action_or_reset(consumer, d2365_put_provider, ref))
		return -ENOMEM;
	return 0;
}

static int d2365_adapter_read(void *context, unsigned int reg, u8 *data,
			       size_t count)
{
	struct d2365_adapter *a = context;

	return regmap_bulk_read(a->regmap, reg, data, count);
}

static int d2365_adapter_write(void *context, unsigned int reg, const u8 *data,
				size_t count)
{
	struct d2365_adapter *a = context;

	return regmap_bulk_write(a->regmap, reg, data, count);
}

static int d2365_adapter_sources(void *context, struct d2365_sources *sources)
{
	struct d2365_adapter *a = context;

	return a->resources.snapshot_sources(a->resources.source_dev, sources);
}

static struct d2365_io d2365_adapter_io(struct d2365_adapter *a)
{
	return (struct d2365_io) {
		.context = a,
		.read = d2365_adapter_read,
		.write = d2365_adapter_write,
		.sources = d2365_adapter_sources,
	};
}

static int d2365_adapter_prepare(struct sys_off_data *off)
{
	struct d2365_adapter *a = off->cb_data;
	struct d2365_io io = d2365_adapter_io(a);
	int error;

	a->result = (struct d2365_result) { .terminal = D2365_UNRESOLVED };
	error = a->resources.quiesce_events(a->resources.event_dev);
	if (error) {
		a->result.first_error = error;
		dev_err(a->dev, "event ownership unavailable: %d; not accessing PMIC\n", error);
		return NOTIFY_DONE;
	}
	a->result = d2365_reference_shutdown(&io, true);
	if (a->result.first_error)
		dev_err(a->dev, "power sequence error %d, terminal %u, standby attempted %u, transport success %u\n",
			a->result.first_error, a->result.terminal,
			a->result.standby_attempted, a->result.standby_acknowledged);
	return NOTIFY_DONE;
}

static int d2365_adapter_restart(struct sys_off_data *off)
{
	struct d2365_adapter *a = off->cb_data;
	struct d2365_io io = d2365_adapter_io(a);
	struct d2365_result result;
	int error;

	error = a->resources.quiesce_events(a->resources.event_dev);
	if (error) {
		/* Do not compete for clear-on-read events. The existing platform
		 * restart remains available, without this preparation.
		 */
		dev_err(a->dev, "restart event ownership unavailable: %d\n", error);
		return NOTIFY_DONE;
	}
	result = d2365_reference_restart(&io);
	if (result.first_error)
		dev_err(a->dev, "restart preparation error: %d\n", result.first_error);
	return NOTIFY_DONE;
}

static int d2365_adapter_final(struct sys_off_data *off)
{
	struct d2365_adapter *a = off->cb_data;

	/* No provider, regmap, I2C or sleeping call in this phase. */
	if (a->result.terminal == D2365_PLATFORM_RESET)
		machine_restart(NULL);
	/* Apple waits indefinitely after the standby command. Unresolved
	 * integration/failed reads are held here too, rather than guessing a
	 * reset or reissuing the command. This is Linux containment, not proof
	 * of Apple's error behavior or that hardware powered off.
	 */
	pr_emerg("D2365 reference: terminal %u; waiting without further PMIC access\n",
		 a->result.terminal);
	local_irq_disable();
	for (;;)
		cpu_relax();
	return NOTIFY_DONE; /* Unreachable; terminal wait never returns. */
}

int devm_d2365_reference_register(struct device *dev, struct regmap *regmap,
				const struct d2365_reference_resources *resources)
{
	struct d2365_adapter *a;
	struct device *bus_dev;
	int error;

	if (!dev || dev->links.status != DL_DEV_PROBING || !regmap || !resources ||
	    !resources->ordinary_path_verified ||
	    !resources->snapshot_sources || !resources->quiesce_events ||
	    !resources->source_dev || !resources->event_dev)
		return -EINVAL;
	bus_dev = regmap_get_device(regmap);
	if (regmap_get_val_bytes(regmap) != 1 || !bus_dev ||
	    resources->event_dev->parent != bus_dev)
		return -EINVAL;
	/* Refuse an existing handler; board integration must also prevent a
	 * later competing registration. This check is not a global lock.
	 */
	if (kernel_can_power_off())
		return -EBUSY;
	if (!devres_open_group(dev, devm_d2365_reference_register, GFP_KERNEL))
		return -ENOMEM;
	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a) {
		error = -ENOMEM;
		goto release;
	}
	a->dev = dev;
	a->regmap = regmap;
	a->resources = *resources;
	a->result.terminal = D2365_UNRESOLVED;
	error = d2365_pin_provider(dev, bus_dev);
	if (error)
		goto release;
	error = d2365_pin_provider(dev, resources->source_dev);
	if (error)
		goto release;
	error = d2365_pin_provider(dev, resources->event_dev);
	if (error)
		goto release;
	error = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
		SYS_OFF_PRIO_HIGH, d2365_adapter_prepare, a);
	if (error)
		goto release;
	error = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART_PREPARE,
		SYS_OFF_PRIO_HIGH, d2365_adapter_restart, a);
	if (error)
		goto release;
	error = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
		SYS_OFF_PRIO_HIGH, d2365_adapter_final, a);
	if (error)
		goto release;
	devres_remove_group(dev, devm_d2365_reference_register);
	return 0;
release:
	devres_release_group(dev, devm_d2365_reference_register);
	return error;
}
EXPORT_SYMBOL_GPL(devm_d2365_reference_register);

MODULE_DESCRIPTION("Unwired Apple-derived J71b power adapter");
MODULE_LICENSE("GPL");
