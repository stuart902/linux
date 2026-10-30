// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple PMIC touchscreen supply regulators
 *
 * Copyright (C) 2026 Paul Praschl
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

static const struct regulator_ops apple_pmic_touch_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc apple_d2186_touch_core = {
	.name = "d2186-touch-core",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x85,
	.enable_mask = 0x02,
	.enable_val = 0x02,
	.disable_val = 0,
};

static const struct regulator_desc apple_adelyn_touch_core = {
	.name = "adelyn-touch-core",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x31f,
	.enable_mask = 0x01,
	.enable_val = 0x01,
	.disable_val = 0,
	.enable_time = 1000,
	.off_on_delay = 1000,
};

/*
 * Angel (D2365, iPad 5) pmuL(0x21f): LDO index 31, enable-only, through
 * register 0x31c bit 0 (iOS 0xfffffff006bbe488, 0xfffffff006bbe388,
 * 0xfffffff006bbcdd8). The ADT names only the function, so the register
 * and bit are not yet confirmed by a second source.
 */
static const struct regulator_desc apple_angel_touch_analog = {
	.name = "angel-touch-analog",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x31c,
	.enable_mask = 0x01,
	.enable_val = 0x01,
	.disable_val = 0,
};

/*
 * D2207 (Arabela, iPad Air 2) pmuL(0x20e): LDO index 14, enable-only, at
 * register 0x84 bit 2 (iOS LDO table 0xfffffff005a98774, setLDO
 * 0xfffffff006cec138). LDOs 14 to 16 share bits 0 and 1 of the register,
 * which iOS sets with any of them; they stay set when the touchscreen
 * supply turns off, as other users may need them.
 */
#define APPLE_D2207_LDO14_SHARED	0x03

static int apple_d2207_touch_analog_enable(struct regulator_dev *rdev)
{
	return regmap_update_bits(rdev_get_regmap(rdev), rdev->desc->enable_reg,
				  rdev->desc->enable_mask | APPLE_D2207_LDO14_SHARED,
				  rdev->desc->enable_val | APPLE_D2207_LDO14_SHARED);
}

static const struct regulator_ops apple_d2207_touch_analog_ops = {
	.enable = apple_d2207_touch_analog_enable,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc apple_d2207_touch_analog = {
	.name = "d2207-touch-analog",
	.ops = &apple_d2207_touch_analog_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x84,
	.enable_mask = 0x04,
	.enable_val = 0x04,
	.disable_val = 0,
};

static const struct regulator_desc apple_chestnut_touch_hv = {
	.name = "chestnut-touch-hv",
	.ops = &apple_pmic_touch_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = 0x05,
	.enable_mask = 0x10,
	.enable_val = 0x10,
	.disable_val = 0,
	.enable_time = 1000,
	.off_on_delay = 1000,
};

static int apple_pmic_touch_regulator_probe(struct platform_device *pdev)
{
	const struct regulator_desc *desc;
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct regmap *regmap;

	desc = device_get_match_data(dev);
	if (!desc)
		return dev_err_probe(dev, -EINVAL, "missing regulator match data\n");

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return dev_err_probe(dev, -ENODEV,
				     "parent PMIC has no regmap\n");

	config.dev = dev;
	config.of_node = dev->of_node;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node, desc);
	config.regmap = regmap;

	rdev = devm_regulator_register(dev, desc, &config);
	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id apple_pmic_touch_regulator_of_match[] = {
	{
		.compatible = "apple,d2186-touch-core-regulator",
		.data = &apple_d2186_touch_core,
	},
	{
		.compatible = "apple,adelyn-touch-core-regulator",
		.data = &apple_adelyn_touch_core,
	},
	{
		.compatible = "apple,angel-touch-analog-regulator",
		.data = &apple_angel_touch_analog,
	},
	{
		.compatible = "apple,d2207-touch-analog-regulator",
		.data = &apple_d2207_touch_analog,
	},
	{
		.compatible = "apple,chestnut-touch-hv-regulator",
		.data = &apple_chestnut_touch_hv,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pmic_touch_regulator_of_match);

static struct platform_driver apple_pmic_touch_regulator_driver = {
	.probe = apple_pmic_touch_regulator_probe,
	.driver = {
		.name = "apple-pmic-touch-regulator",
		.of_match_table = apple_pmic_touch_regulator_of_match,
	},
};
module_platform_driver(apple_pmic_touch_regulator_driver);

MODULE_AUTHOR("Paul Praschl <praschlpaul@g-p.at>");
MODULE_DESCRIPTION("Apple PMIC touchscreen supply regulators");
MODULE_LICENSE("GPL");
