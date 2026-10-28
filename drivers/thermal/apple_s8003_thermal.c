// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple S8003 (A9) on-die temperature sensors.
 *
 * The register interface follows iOS's AppleT700XTempSensor ("tempsensor,
 * s8000"): AppleT700XTempSensor_start at 0xfffffff005f1a424 and
 * AppleT700XTempSensor_readQ6 at 0xfffffff005f1b938 in the J71t 20H350
 * kernelcache. The hardware converts with the calibration the boot chain
 * programs; this driver never writes the trim parameters, CTL1/CTL2, the
 * alarm registers or the SoCHOT thresholds.
 *
 * Each sensor accumulates samples between a "start" (CTL0 set, bit 3) and a
 * "freeze" (CTL0 clear, bit 3). A read freezes the accumulator, reads the
 * result word (BK: bit 31 valid, signed Q6 degrees Celsius in the low bits)
 * and the sample count, and starts the next interval, as iOS's timer
 * callback does.
 */

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "thermal_hwmon.h"

#define S8003_CTL0_ENABLE	BIT(0)
#define S8003_CTL0_ACCUMULATE	BIT(3)
#define S8003_BK_VALID		BIT(31)
#define S8003_COUNT_OVERFLOW	BIT(31)
/* Shorter intervals may not have accumulated a sample yet. */
#define S8003_MIN_INTERVAL_MS	100

enum s8003_page {
	S8003_PAGE_SOC,		/* PMGR sensors */
	S8003_PAGE_TCC,		/* ACC sensors */
	S8003_PAGES,
};

/**
 * struct s8003_sensor_desc - one sensor channel
 * @page: register page
 * @ctl_set: CTL0 set register
 * @ctl_clear: CTL0 clear register
 * @bk: result register
 * @count: accumulated sample count
 * @sign_bit: sign bit of the Q6 value (14 for sensor-accuracy 2, else 15)
 */
struct s8003_sensor_desc {
	enum s8003_page page;
	u16 ctl_set;
	u16 ctl_clear;
	u16 bk;
	u16 count;
	u8 sign_bit;
};

/*
 * Sensor IDs are the hardware channels: SOC0..SOC2, then TCC0..TCC2.
 * Only SOC0, SOC1 and TCC2 have been checked against live readings on
 * J71t; the other three are not exposed until their mapping is.
 */
static const struct s8003_sensor_desc s8003_sensors[] = {
	[0] = { S8003_PAGE_SOC, 0x000, 0x004, 0x018, 0x024, 14 },
	[1] = { S8003_PAGE_SOC, 0x100, 0x104, 0x118, 0x124, 14 },
	[5] = { S8003_PAGE_TCC, 0x200, 0x468, 0x220, 0x238, 15 },
};

struct s8003_thermal;

struct s8003_sensor {
	struct s8003_thermal *priv;
	const struct s8003_sensor_desc *desc;
	struct mutex lock;	/* serializes freeze, read and restart */
	unsigned long sampled;	/* jiffies of the last valid sample */
	int temp;		/* millidegrees Celsius */
	bool valid;
};

struct s8003_thermal {
	void __iomem *page[S8003_PAGES];
	struct s8003_sensor sensor[ARRAY_SIZE(s8003_sensors)];
};

static int s8003_sample(struct s8003_sensor *s)
{
	const struct s8003_sensor_desc *d = s->desc;
	void __iomem *base = s->priv->page[d->page];
	u32 ctl, bk, count;
	int temp;

	writel(S8003_CTL0_ACCUMULATE, base + d->ctl_clear);
	ctl = readl(base + d->ctl_set);
	bk = readl(base + d->bk);
	count = readl(base + d->count);
	writel(S8003_CTL0_ACCUMULATE, base + d->ctl_set);

	if (!(ctl & S8003_CTL0_ENABLE) || !(bk & S8003_BK_VALID) ||
	    !count || (count & S8003_COUNT_OVERFLOW))
		return -EAGAIN;

	temp = sign_extend32(bk, d->sign_bit) * 1000 / 64;
	/* Reject impossible codes; iOS substitutes 30 C instead. */
	if (temp < -40000 || temp > 150000)
		return -ERANGE;

	s->temp = temp;
	s->sampled = jiffies;
	s->valid = true;
	return 0;
}

static int s8003_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct s8003_sensor *s = thermal_zone_device_priv(tz);
	int ret = 0;

	mutex_lock(&s->lock);
	if (!s->valid ||
	    time_after_eq(jiffies, s->sampled +
			  msecs_to_jiffies(S8003_MIN_INTERVAL_MS)))
		ret = s8003_sample(s);
	/* A too-short interval keeps the previous sample if there is one. */
	if (ret == -EAGAIN && s->valid &&
	    time_before(jiffies, s->sampled +
			2 * msecs_to_jiffies(S8003_MIN_INTERVAL_MS)))
		ret = 0;
	if (!ret)
		*temp = s->temp;
	mutex_unlock(&s->lock);

	return ret;
}

static const struct thermal_zone_device_ops s8003_thermal_ops = {
	.get_temp = s8003_get_temp,
};

static int s8003_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s8003_thermal *priv;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->page[S8003_PAGE_SOC] = devm_platform_ioremap_resource_byname(pdev, "soc");
	if (IS_ERR(priv->page[S8003_PAGE_SOC]))
		return PTR_ERR(priv->page[S8003_PAGE_SOC]);
	priv->page[S8003_PAGE_TCC] = devm_platform_ioremap_resource_byname(pdev, "tcc");
	if (IS_ERR(priv->page[S8003_PAGE_TCC]))
		return PTR_ERR(priv->page[S8003_PAGE_TCC]);

	for (i = 0; i < ARRAY_SIZE(s8003_sensors); i++) {
		const struct s8003_sensor_desc *d = &s8003_sensors[i];
		struct s8003_sensor *s = &priv->sensor[i];
		struct thermal_zone_device *tz;

		if (!d->bk)
			continue;

		s->priv = priv;
		s->desc = d;
		ret = devm_mutex_init(dev, &s->lock);
		if (ret)
			return ret;

		/* Enable conversion and start accumulating, as iOS does. */
		writel(S8003_CTL0_ENABLE | S8003_CTL0_ACCUMULATE,
		       priv->page[d->page] + d->ctl_set);

		tz = devm_thermal_of_zone_register(dev, i, s, &s8003_thermal_ops);
		if (IS_ERR(tz)) {
			if (PTR_ERR(tz) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tz),
					     "failed to register sensor %u\n", i);
		}

		ret = devm_thermal_add_hwmon_sysfs(dev, tz);
		if (ret)
			dev_warn(dev, "failed to add hwmon for sensor %u: %d\n",
				 i, ret);
	}

	return 0;
}

static const struct of_device_id s8003_thermal_of_match[] = {
	{ .compatible = "apple,s8003-thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, s8003_thermal_of_match);

static struct platform_driver s8003_thermal_driver = {
	.probe = s8003_thermal_probe,
	.driver = {
		.name = "apple-s8003-thermal",
		.of_match_table = s8003_thermal_of_match,
	},
};
module_platform_driver(s8003_thermal_driver);

MODULE_DESCRIPTION("Apple S8003 on-die temperature sensors");
MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_LICENSE("GPL");
