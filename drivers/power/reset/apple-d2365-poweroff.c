// SPDX-License-Identifier: GPL-2.0-only
/* Apple D2365 (Angel) direct PMIC shutdown. */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define D2365_EVENTS		0x0140
#define D2365_EVENT_MASK		0x01c0
#define D2365_EVENT_COUNT	30
#define D2365_SOURCE_STATUS	0x018a
#define D2365_STANDBY		0x0400
#define D2365_STANDBY_REQUEST	BIT(0)
#define D2365_RTC_CONTROL	0x0500
#define D2365_RTC_NO_ALARM	0x01
#define D2365_GPIO_PAIRS		0x0900
#define D2365_GPIO_PAIR_COUNT	18
#define D2365_GPIO_COUNT		24
#define D2365_SHUTDOWN_STATE	0x5001
#define D2365_SHUTDOWN_FLAGS	0x5003
#define D2365_BOOT_STATE		0x500f
#define D2365_BOOT_PRESERVE	0x27
#define D2365_BOOT_SOURCE_FLAGS	0x90

struct apple_d2365_poweroff_data {
	u8 event_mask[D2365_EVENT_COUNT];
};

struct apple_d2365_poweroff {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *supply;
	const struct apple_d2365_poweroff_data *data;
	bool restart;
	bool standby_attempted;
	bool standby_acknowledged;
};

static int apple_d2365_boot_state(struct apple_d2365_poweroff *pmic)
{
	unsigned int value;
	int ret;

	ret = regmap_read(pmic->regmap, D2365_BOOT_STATE, &value);
	if (ret)
		return ret;

	value &= D2365_BOOT_PRESERVE;
	if (pmic->restart)
		value |= D2365_BOOT_SOURCE_FLAGS;

	return regmap_write(pmic->regmap, D2365_BOOT_STATE, value);
}

static int apple_d2365_gpio_prepare(struct apple_d2365_poweroff *pmic)
{
	u8 pairs[D2365_GPIO_PAIR_COUNT * 2];
	u8 extra[D2365_GPIO_COUNT - D2365_GPIO_PAIR_COUNT];
	unsigned int value;
	int i, ret;

	ret = regmap_bulk_read(pmic->regmap, D2365_GPIO_PAIRS,
			       pairs, sizeof(pairs));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(extra); i++) {
		ret = regmap_read(pmic->regmap, 0x0929 + 6 * i, &value);
		if (ret)
			return ret;
		extra[i] = value;
	}

	/* Only simple GPIO configurations change; preserve the paired bytes. */
	for (i = 0; i < sizeof(pairs); i += 2)
		if (pairs[i] <= 0x3f)
			pairs[i] &= ~BIT(0);

	for (i = 0; i < ARRAY_SIZE(extra); i++)
		if (extra[i] <= 0x3f)
			extra[i] &= ~BIT(0);

	ret = regmap_bulk_write(pmic->regmap, D2365_GPIO_PAIRS,
				pairs, sizeof(pairs));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(extra); i++) {
		ret = regmap_write(pmic->regmap, 0x0929 + 6 * i, extra[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/* All transactions run before syscore/IRQ teardown, while I2C may sleep. */
static int apple_d2365_shutdown_sequence(struct apple_d2365_poweroff *pmic)
{
	u8 events[D2365_EVENT_COUNT];
	union power_supply_propval online;
	unsigned int status;
	int ret;

	/* Events acknowledge on read, not by writing zeroes. */
	ret = regmap_bulk_read(pmic->regmap, D2365_EVENTS,
			       events, sizeof(events));
	if (ret)
		return ret;

	ret = regmap_bulk_write(pmic->regmap, D2365_EVENT_MASK,
				pmic->data->event_mask, D2365_EVENT_COUNT);
	if (ret)
		return ret;

	/* Normal shutdown, without a scheduled RTC/UPO wake alarm. */
	ret = regmap_write(pmic->regmap, D2365_RTC_CONTROL, D2365_RTC_NO_ALARM);
	if (ret)
		return ret;

	ret = regmap_write(pmic->regmap, D2365_SHUTDOWN_STATE, 0);
	if (ret)
		return ret;

	ret = regmap_write(pmic->regmap, D2365_SHUTDOWN_FLAGS, 0);
	if (ret)
		return ret;

	ret = apple_d2365_boot_state(pmic);
	if (ret || pmic->restart)
		return ret;

	/* Recheck live source status after the shutdown bookkeeping. */
	ret = regmap_read(pmic->regmap, D2365_SOURCE_STATUS, &status);
	if (ret)
		return ret;
	if (status) {
		pmic->restart = true;
		return apple_d2365_boot_state(pmic);
	}

	ret = apple_d2365_gpio_prepare(pmic);
	if (ret)
		return ret;

	ret = regmap_bulk_read(pmic->regmap, D2365_EVENTS,
			       events, sizeof(events));
	if (ret)
		return ret;

	/* A source may have attached while the GPIOs were being prepared. */
	ret = power_supply_get_property(pmic->supply, POWER_SUPPLY_PROP_ONLINE,
					&online);
	if (ret)
		return ret;
	ret = regmap_read(pmic->regmap, D2365_SOURCE_STATUS, &status);
	if (ret)
		return ret;
	if (online.intval || status) {
		pmic->restart = true;
		return apple_d2365_boot_state(pmic);
	}

	/* Force the command write even if a previous request bit remains set. */
	/* A failed transfer can still reach the PMIC; never retry the request. */
	pmic->standby_attempted = true;
	ret = regmap_write_bits(pmic->regmap, D2365_STANDBY,
				D2365_STANDBY_REQUEST, D2365_STANDBY_REQUEST);
	if (!ret)
		pmic->standby_acknowledged = true;

	return ret;
}

static int apple_d2365_poweroff_prepare(struct sys_off_data *off)
{
	struct apple_d2365_poweroff *pmic = off->cb_data;
	union power_supply_propval online;
	unsigned int status;
	int ret;

	/* Unknown source state is never permission to enter standby. */
	pmic->restart = true;
	pmic->standby_attempted = false;
	pmic->standby_acknowledged = false;
	ret = power_supply_get_property(pmic->supply, POWER_SUPPLY_PROP_ONLINE,
					&online);
	if (ret)
		goto fail;

	/* The supply's cached state alone can miss a just-attached cable. */
	ret = regmap_read(pmic->regmap, D2365_SOURCE_STATUS, &status);
	if (ret)
		goto fail;
	pmic->restart = online.intval != 0 || status != 0;

	ret = apple_d2365_shutdown_sequence(pmic);
	if (ret)
		goto fail;

	if (pmic->restart)
		dev_info(pmic->dev, "external power present; restarting instead of standby\n");
	else
		/* Bounded grace period, not a hardware timing requirement. */
		msleep(1000);

	return NOTIFY_DONE;

fail:
	pmic->restart = true;
	dev_err(pmic->dev, "shutdown preparation failed: %d; standby request %s; restarting\n",
		ret, pmic->standby_attempted ? "outcome unknown" : "not attempted");
	return NOTIFY_DONE;
}

static int apple_d2365_poweroff_final(struct sys_off_data *off)
{
	struct apple_d2365_poweroff *pmic = off->cb_data;

	/* No regmap, I2C, or sleeping operations are legal in this callback. */
	if (pmic->restart) {
		dev_emerg(pmic->dev, "requesting restart instead of PMIC standby\n");
		machine_restart(NULL);
	} else {
		dev_emerg(pmic->dev, "power off failed (standby request %s)\n",
			  pmic->standby_acknowledged ? "acknowledged" :
			  pmic->standby_attempted ? "outcome unknown" : "not attempted");
	}

	return NOTIFY_DONE;
}

/* D2365 shutdown event masks; these are not an all-masked bitmap. */
static const struct apple_d2365_poweroff_data apple_d2365_data = {
	.event_mask = {
		0xff, 0xff, 0xfe, 0xff, 0xff, 0xf0, 0xff, 0xff,
		0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	},
};

static const struct of_device_id apple_d2365_poweroff_of_match[] = {
	{ .compatible = "apple,d2365-poweroff", .data = &apple_d2365_data },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_d2365_poweroff_of_match);

static int apple_d2365_poweroff_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_d2365_poweroff *pmic;
	int ret;

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->dev = dev;
	pmic->data = device_get_match_data(dev);
	pmic->regmap = dev_get_regmap(dev->parent, NULL);
	if (!pmic->regmap)
		return dev_err_probe(dev, -ENODEV, "parent has no register map\n");

	pmic->supply = devm_power_supply_get_by_reference(dev, "power-supplies");
	if (IS_ERR(pmic->supply))
		return dev_err_probe(dev, PTR_ERR(pmic->supply),
				     "cannot get external power supply\n");
	if (!pmic->supply)
		return dev_err_probe(dev, -EPROBE_DEFER, "external power supply not ready\n");

	pmic->restart = true;
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					    SYS_OFF_PRIO_DEFAULT,
					    apple_d2365_poweroff_prepare, pmic);
	if (ret)
		return ret;

	/* Also required for kernel_can_power_off() to recognize this driver. */
	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					     SYS_OFF_PRIO_DEFAULT,
					     apple_d2365_poweroff_final, pmic);
}

static struct platform_driver apple_d2365_poweroff_driver = {
	.probe = apple_d2365_poweroff_probe,
	.driver = {
		.name = "apple-d2365-poweroff",
		.of_match_table = apple_d2365_poweroff_of_match,
	},
};
module_platform_driver(apple_d2365_poweroff_driver);

MODULE_DESCRIPTION("Apple D2365 PMIC direct power-off driver");
MODULE_AUTHOR("Codex <codex@openai.com>");
MODULE_LICENSE("GPL");
