// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Barometric pressure sensor served by the Apple Always-On Processor (AOP)
 *
 * The AOP firmware of the A9 and A10 samples the pressure sensor itself. Once
 * it has the unit's factory calibration and its batch, raw and report
 * intervals (EPIC set-property tags 0x16, 0x5c and 0), it sends one report
 * (EPIC subtype 0xc4) per report interval:
 *
 *   0  u16 sequence
 *   2  u32 pressure in 1/65536 kPa
 *   6  s32 temperature (unit unknown, not exported)
 */

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/aop.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#define AOP_PROP_REPORT_INTERVAL	0x00
#define AOP_PROP_BATCH_INTERVAL		0x16
#define AOP_PROP_RAW_INTERVAL		0x5c

#define AOP_REPORT_PRESSURE		0xc4
#define AOP_PRESSURE_LEN		10
#define AOP_PRESSURE_VALUE		2

#define AOP_PRESSURE_DEFAULT_US		100000	/* 10 Hz */
#define AOP_PRESSURE_MIN_US		10000	/* 100 Hz */
#define AOP_PRESSURE_MAX_US		1000000	/* 1 Hz */

struct apple_aop_pressure {
	struct device *dev;
	const struct apple_aop_service *service;

	struct mutex lock; /* interval */
	u32 interval_us;

	spinlock_t sample_lock; /* sample, have_sample */
	u32 sample;
	bool have_sample;

	IIO_DECLARE_BUFFER_WITH_TS(u32, scan, 1);
};

static int apple_aop_pressure_set_u32(struct apple_aop_pressure *pressure, u32 key, u32 value)
{
	__le32 le = cpu_to_le32(value);

	return apple_aop_set_property(pressure->service, key, &le, sizeof(le));
}

/* No separate raw sample stream: raw interval 0. */
static int apple_aop_pressure_set_interval(struct apple_aop_pressure *pressure, u32 interval_us)
{
	int ret;

	ret = apple_aop_pressure_set_u32(pressure, AOP_PROP_BATCH_INTERVAL, interval_us);
	if (!ret)
		ret = apple_aop_pressure_set_u32(pressure, AOP_PROP_RAW_INTERVAL, 0);
	if (!ret)
		ret = apple_aop_pressure_set_u32(pressure, AOP_PROP_REPORT_INTERVAL, interval_us);
	if (!ret)
		pressure->interval_us = interval_us;
	return ret;
}

static void apple_aop_pressure_report(void *cookie, u16 subtype, const void *data, size_t len)
{
	struct iio_dev *indio_dev = cookie;
	struct apple_aop_pressure *pressure = iio_priv(indio_dev);
	unsigned long flags;

	if (subtype != AOP_REPORT_PRESSURE || len < AOP_PRESSURE_LEN)
		return;

	spin_lock_irqsave(&pressure->sample_lock, flags);
	pressure->sample = get_unaligned_le32(data + AOP_PRESSURE_VALUE);
	pressure->have_sample = true;
	spin_unlock_irqrestore(&pressure->sample_lock, flags);

	if (iio_buffer_enabled(indio_dev)) {
		pressure->scan[0] = pressure->sample;
		iio_push_to_buffers_with_ts(indio_dev, pressure->scan, sizeof(pressure->scan),
					    iio_get_time_ns(indio_dev));
	}
}

static int apple_aop_pressure_read_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int *val, int *val2, long mask)
{
	struct apple_aop_pressure *pressure = iio_priv(indio_dev);
	unsigned long flags;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		spin_lock_irqsave(&pressure->sample_lock, flags);
		*val = pressure->sample;
		ret = pressure->have_sample ? IIO_VAL_INT : -EAGAIN;
		spin_unlock_irqrestore(&pressure->sample_lock, flags);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		/* kPa */
		*val = 1;
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&pressure->lock);
		*val = USEC_PER_SEC / pressure->interval_us;
		*val2 = (u64)(USEC_PER_SEC % pressure->interval_us) * USEC_PER_SEC /
			pressure->interval_us;
		mutex_unlock(&pressure->lock);
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int apple_aop_pressure_write_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int val, int val2, long mask)
{
	struct apple_aop_pressure *pressure = iio_priv(indio_dev);
	u64 micro_hz;
	u32 interval;
	int ret;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val < 0 || val2 < 0)
		return -EINVAL;

	micro_hz = (u64)val * USEC_PER_SEC + val2;
	if (!micro_hz)
		return -EINVAL;
	interval = clamp_t(u64, div64_u64((u64)USEC_PER_SEC * USEC_PER_SEC, micro_hz),
			   AOP_PRESSURE_MIN_US, AOP_PRESSURE_MAX_US);

	mutex_lock(&pressure->lock);
	ret = apple_aop_pressure_set_interval(pressure, interval);
	mutex_unlock(&pressure->lock);
	return ret;
}

static const struct iio_chan_spec apple_aop_pressure_channels[] = {
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static const struct iio_info apple_aop_pressure_info = {
	.read_raw = apple_aop_pressure_read_raw,
	.write_raw = apple_aop_pressure_write_raw,
};

/* The offset or the acceleration sensitivity makes it complete */
static const struct apple_aop_calibration apple_aop_pressure_calibration[] = {
	{ "apple,pressure-offset-calibration", 0x0b, 1 },
	{ "apple,pressure-accel-sensitivity", 0xd3, 1 },
	{ "apple,pressure-heater-resistance", 0xd4, 0 },
};

static void apple_aop_pressure_stop(void *data)
{
	struct apple_aop_pressure *pressure = data;

	apple_aop_pressure_set_u32(pressure, AOP_PROP_REPORT_INTERVAL, 0);
	apple_aop_unlisten(pressure->service);
}

static int apple_aop_pressure_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct apple_aop_service *service = dev_get_platdata(dev);
	struct apple_aop_pressure *pressure;
	struct iio_dev *indio_dev;
	int ret;

	if (!service)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*pressure));
	if (!indio_dev)
		return -ENOMEM;
	pressure = iio_priv(indio_dev);
	pressure->dev = dev;
	pressure->service = service;
	mutex_init(&pressure->lock);
	spin_lock_init(&pressure->sample_lock);

	indio_dev->name = "apple-aop-pressure";
	indio_dev->info = &apple_aop_pressure_info;
	indio_dev->channels = apple_aop_pressure_channels;
	indio_dev->num_channels = ARRAY_SIZE(apple_aop_pressure_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev, NULL);
	if (ret)
		return ret;

	ret = apple_aop_listen(service, apple_aop_pressure_report, indio_dev);
	if (ret)
		return ret;

	ret = apple_aop_calibrate(service, dev, apple_aop_pressure_calibration,
				  ARRAY_SIZE(apple_aop_pressure_calibration));
	if (!ret)
		ret = apple_aop_pressure_set_interval(pressure, AOP_PRESSURE_DEFAULT_US);
	if (ret) {
		apple_aop_unlisten(service);
		return dev_err_probe(dev, ret, "failed to start reports\n");
	}

	ret = devm_add_action_or_reset(dev, apple_aop_pressure_stop, pressure);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id apple_aop_pressure_of_match[] = {
	{ .compatible = "apple,s8000-aop-pressure" },
	{ .compatible = "apple,t8010-aop-pressure" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_aop_pressure_of_match);

static struct platform_driver apple_aop_pressure_driver = {
	.driver = {
		.name = "apple-aop-pressure",
		.of_match_table = apple_aop_pressure_of_match,
	},
	.probe = apple_aop_pressure_probe,
};
module_platform_driver(apple_aop_pressure_driver);

MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_DESCRIPTION("Apple AOP barometric pressure sensor driver");
MODULE_LICENSE("Dual MIT/GPL");
