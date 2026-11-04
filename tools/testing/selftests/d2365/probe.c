/* SPDX-License-Identifier: MIT
 * Offline provider/probe shims. These test the actual probe helpers, not
 * driver-core concurrency or hardware. No device or reboot access.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ENODEV 19
#define ENOMEM 12
#define EINVAL 22
#define EPROBE_DEFER 517
#define GFP_KERNEL 0
#define DL_FLAG_AUTOREMOVE_CONSUMER 1
#define DL_STATE_DORMANT 0
#define DL_STATE_CONSUMER_PROBE 1
#define DL_STATE_SUPPLIER_UNBIND 2
#define READ_ONCE(x) (x)
#define ERR_PTR(x) ((void *)(intptr_t)(x))
#define PTR_ERR(x) ((int)(intptr_t)(x))
#define IS_ERR(x) ((uintptr_t)(x) >= (uintptr_t)-4095)
#define SYS_OFF_MODE_RESTART_PREPARE 0
#define SYS_OFF_MODE_POWER_OFF_PREPARE 1
#define SYS_OFF_MODE_POWER_OFF 2
#define SYS_OFF_PRIO_DEFAULT 0

struct device_node { struct device_node *parent; };
struct device {
	struct device_node *of_node;
	struct device *parent;
	bool locked, bound, unbinding;
	unsigned int refs;
};
struct platform_device { struct device dev; };
struct device_link { int status; };
struct power_supply { struct device dev; };
struct apple_d2365_poweroff {
	struct device *dev, *event_dev;
	void *data, *regmap;
	struct power_supply *supply;
	bool restart;
};
struct action { void (*release)(void *); void *data; };
static struct device_node pmic_node, event_node, supply_node, consumer_node, foreign_node;
static struct device parent, other;
static struct platform_device consumer, event, supply;
static struct device_node *supply_ref;
static struct power_supply psy;
static struct apple_d2365_poweroff state;
static struct device_link event_link, supply_link;
static struct action actions[8];
static unsigned int action_count, supply_refs, handlers, supply_gets, event_checks;
static unsigned int locks, unlocks, node_puts, links, fail_action, action_calls;
static int fail_link, supply_error, fail_handler;
static bool no_event, no_supply, no_event_node, no_regmap, event_ready, no_psy;
static bool foreign_psy, begin_unbind_after_link;

static void release_all(void)
{
	while (action_count) {
		struct action *action = &actions[--action_count];
		action->release(action->data);
	}
	event_link.status = supply_link.status = DL_STATE_DORMANT;
}
static int dev_err_probe(struct device *dev, int error, const char *format, ...)
{
	return error;
}
static void put_device(void *data)
{
	struct device *dev = data;
	assert(dev->refs && !supply_refs && !handlers);
	dev->refs--;
}
static void of_node_put(struct device_node *node)
{
	assert(node);
	node_puts++;
}
static struct device_node *of_get_compatible_child(struct device_node *node,
						   const char *compatible)
{
	assert(node == &pmic_node);
	assert(!strcmp(compatible, "apple,d2365-pmic-gpio"));
	return no_event_node ? NULL : &event_node;
}
static struct platform_device *of_find_device_by_node(struct device_node *node)
{
	assert(node == &event_node);
	if (no_event)
		return NULL;
	event.dev.refs++;
	return &event;
}
static struct device_node *of_parse_phandle(struct device_node *node,
					   const char *property, int index)
{
	assert(node == &consumer_node && !strcmp(property, "power-supplies") && !index);
	return supply_ref;
}
static int device_match_of_node(struct device *dev, const void *node)
{
	return dev->of_node == node;
}
static struct device *device_find_child(struct device *dev, const void *node,
					int (*match)(struct device *, const void *))
{
	assert(dev == &parent && node == &supply_node);
	if (no_supply)
		return NULL;
	assert(match(&supply.dev, node));
	supply.dev.refs++;
	return &supply.dev;
}
static int devm_add_action_or_reset(struct device *dev,
				    void (*release)(void *), void *data)
{
	assert(dev == &consumer.dev);
	if (++action_calls == fail_action) {
		release(data);
		return -ENOMEM;
	}
	assert(action_count < sizeof(actions) / sizeof(actions[0]));
	actions[action_count++] = (struct action){release, data};
	return 0;
}
static struct device_link *device_link_add(struct device *dev,
					   struct device *provider, unsigned int flags)
{
	struct device_link *link = provider == &event.dev ? &event_link : &supply_link;
	assert(dev == &consumer.dev && flags == DL_FLAG_AUTOREMOVE_CONSUMER);
	assert(provider == &event.dev || provider == &supply.dev);
	assert(provider->refs && !provider->locked);
	if (++links == fail_link)
		return NULL;
	link->status = provider->unbinding ? DL_STATE_SUPPLIER_UNBIND :
		       provider->bound ? DL_STATE_CONSUMER_PROBE : DL_STATE_DORMANT;
	/* Established probing links remain busy when unbind begins later. */
	if (begin_unbind_after_link)
		provider->unbinding = true;
	return link;
}
static void device_lock(struct device *dev)
{
	assert(dev != &consumer.dev && dev != &parent && !dev->locked);
	dev->locked = true;
	locks++;
}
static void device_unlock(struct device *dev)
{
	assert(dev->locked);
	dev->locked = false;
	unlocks++;
}
static bool device_is_bound(struct device *dev)
{
	assert(dev->locked);
	return dev->bound;
}
static bool apple_d2365_gpio_events_ready(struct device *dev)
{
	assert(dev == &event.dev && dev->locked);
	assert(event_link.status == DL_STATE_CONSUMER_PROBE);
	event_checks++;
	return event_ready;
}
static void put_supply(void *data)
{
	assert(data == &psy && supply_refs == 1 && !handlers);
	supply_refs--;
}
static struct power_supply *devm_power_supply_get_by_reference(struct device *dev,
							       const char *property)
{
	assert(dev == &consumer.dev && !strcmp(property, "power-supplies"));
	assert(supply.dev.locked && supply.dev.bound);
	assert(supply_link.status == DL_STATE_CONSUMER_PROBE);
	supply_gets++;
	if (supply_error)
		return ERR_PTR(supply_error);
	if (no_psy)
		return NULL;
	supply_refs++;
	actions[action_count++] = (struct action){put_supply, &psy};
	psy.dev.parent = foreign_psy ? &other : &supply.dev;
	return &psy;
}
static void *devm_kzalloc(struct device *dev, unsigned int size, int flags)
{
	assert(dev == &consumer.dev && size == sizeof(state));
	memset(&state, 0, sizeof(state));
	return &state;
}
static void *device_get_match_data(struct device *dev) { return &state; }
static void *dev_get_regmap(struct device *dev, void *name)
{
	assert(dev == &parent);
	return no_regmap ? NULL : &state;
}
static int apple_d2365_restart_prepare(void *off) { return 0; }
static int apple_d2365_poweroff_prepare(void *off) { return 0; }
static int apple_d2365_poweroff_final(void *off) { return 0; }
static void unregister_handler(void *data)
{
	assert(handlers && supply_refs && event.dev.refs && supply.dev.refs);
	assert((intptr_t)data == handlers - 1);
	handlers--;
}
static int devm_register_sys_off_handler(struct device *dev, int mode, int priority,
					  int (*callback)(void *), void *data)
{
	assert(dev == &consumer.dev && data == &state && supply_refs == 1);
	assert(mode == handlers);
	if (mode == fail_handler)
		return -ENOMEM;
	handlers++;
	actions[action_count++] = (struct action){unregister_handler, (void *)(intptr_t)mode};
	return 0;
}

/* DRIVER */

static void reset(void)
{
	assert(!action_count && !supply_refs && !handlers && !event.dev.refs && !supply.dev.refs);
	parent = (struct device){.of_node = &pmic_node, .locked = true, .bound = true};
	consumer_node.parent = event_node.parent = supply_node.parent = &pmic_node;
	foreign_node.parent = NULL;
	consumer.dev = (struct device){.of_node = &consumer_node, .parent = &parent, .locked = true};
	event.dev = (struct device){.of_node = &event_node, .parent = &parent, .bound = true};
	supply.dev = (struct device){.of_node = &supply_node, .parent = &parent, .bound = true};
	supply_ref = &supply_node;
	event_link.status = supply_link.status = DL_STATE_DORMANT;
	locks = unlocks = node_puts = links = action_calls = supply_gets = event_checks = 0;
	fail_action = 0; fail_link = 0; supply_error = 0; fail_handler = -1;
	no_event = no_supply = no_event_node = no_regmap = no_psy = foreign_psy = false;
	begin_unbind_after_link = false; event_ready = true;
}
static void probe_result(int expected)
{
	assert(apple_d2365_poweroff_probe(&consumer) == expected);
	assert(locks == unlocks);
	if (!expected)
		assert(handlers == 3 && supply_refs == 1 && state.supply == &psy &&
		       state.event_dev == &event.dev && state.restart);
	else
		assert(!handlers || fail_handler >= 0);
	/* Models devres release on failed probe or consumer unbind. */
	release_all();
	assert(!handlers && !supply_refs && !event.dev.refs && !supply.dev.refs);
}
static void topology(void)
{
	reset(); supply_ref = NULL; probe_result(-ENODEV); assert(locks == 1 && !supply_gets);
	reset(); supply_ref = &consumer_node; probe_result(-EINVAL); assert(locks == 1 && !supply_gets);
	reset(); supply_ref = &pmic_node; probe_result(-EINVAL); assert(locks == 1 && !supply_gets);
	reset(); supply_ref = &foreign_node; probe_result(-EINVAL); assert(locks == 1 && !supply_gets);
	reset(); no_supply = true; probe_result(-EPROBE_DEFER); assert(locks == 1 && !supply_gets);
}
static void failures(void)
{
	reset(); no_regmap = true; probe_result(-ENODEV);
	reset(); no_event_node = true; probe_result(-ENODEV);
	reset(); no_event = true; probe_result(-EPROBE_DEFER);
	for (unsigned int i = 1; i <= 2; i++) {
		reset(); fail_action = i; probe_result(-ENOMEM);
		reset(); fail_link = i; probe_result(-EINVAL);
	}
	reset(); event_ready = false; probe_result(-ENODEV); assert(!supply_gets);
	reset(); supply_error = -ENOMEM; probe_result(-ENOMEM);
	reset(); no_psy = true; probe_result(-EPROBE_DEFER);
	reset(); foreign_psy = true; probe_result(-ENODEV);
	for (int i = 0; i < 3; i++) {
		reset(); fail_handler = i; probe_result(-ENOMEM);
	}
}
static void lifetime(void)
{
	reset(); event.dev.bound = false; probe_result(-EPROBE_DEFER); assert(!event_checks && !supply_gets);
	reset(); supply.dev.bound = false; probe_result(-EPROBE_DEFER); assert(!supply_gets);
	/* Still bound, but unbind already started before link creation. */
	reset(); event.dev.unbinding = true; probe_result(-EPROBE_DEFER); assert(!event_checks && !supply_gets);
	reset(); supply.dev.unbinding = true; probe_result(-EPROBE_DEFER); assert(!supply_gets);
	reset(); begin_unbind_after_link = true; probe_result(0);
	reset(); probe_result(0);
}
int main(int argc, char **argv)
{
	assert(argc == 2);
	if (!strcmp(argv[1], "topology"))
		topology();
	else if (!strcmp(argv[1], "failures"))
		failures();
	else if (!strcmp(argv[1], "lifetime"))
		lifetime();
	else
		return 2;
	return 0;
}
