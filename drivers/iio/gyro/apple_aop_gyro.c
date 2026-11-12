// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Gyroscope served by the Apple Always-On Processor (AOP)
 *
 * The AOP firmware of the A9 and A10 samples the gyroscope itself. Once it
 * has the unit's factory calibration and its batch, raw and report intervals
 * (EPIC set-property tags 0x16, 0x5c and 0), it sends one motion report (EPIC
 * subtype 0xc1) per report interval, laid out like the accelerometer's:
 *
 *   0  u16 sequence
 *   2  u32 status
 *   6  s32 x, y, z in 1/65536 degree per second
 *  18  s32 temperature (unit unknown, not exported)
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

#define AOP_REPORT_GYRO			0xc1
#define AOP_GYRO_LEN			22
#define AOP_GYRO_XYZ			6

#define AOP_GYRO_DEFAULT_US		100000	/* 10 Hz */
#define AOP_GYRO_MIN_US			10000	/* 100 Hz */
#define AOP_GYRO_MAX_US			1000000	/* 1 Hz */

/* pi / 180 rad/s / 65536 = 1171270 / 2^42 rad/s */
#define AOP_GYRO_SCALE			1171270
#define AOP_GYRO_SCALE_LOG2		42

struct apple_aop_gyro {
	struct device *dev;
	const struct apple_aop_service *service;
	struct iio_mount_matrix orientation;

	struct mutex lock; /* interval */
	u32 interval_us;

	spinlock_t sample_lock; /* sample, have_sample */
	s32 sample[3];
	bool have_sample;

	IIO_DECLARE_BUFFER_WITH_TS(s32, scan, 3);
};

static int apple_aop_gyro_set_u32(struct apple_aop_gyro *gyro, u32 key, u32 value)
{
	__le32 le = cpu_to_le32(value);

	return apple_aop_set_property(gyro->service, key, &le, sizeof(le));
}

/* No separate raw sample stream: raw interval 0. */
static int apple_aop_gyro_set_interval(struct apple_aop_gyro *gyro, u32 interval_us)
{
	int ret;

	ret = apple_aop_gyro_set_u32(gyro, AOP_PROP_BATCH_INTERVAL, interval_us);
	if (!ret)
		ret = apple_aop_gyro_set_u32(gyro, AOP_PROP_RAW_INTERVAL, 0);
	if (!ret)
		ret = apple_aop_gyro_set_u32(gyro, AOP_PROP_REPORT_INTERVAL, interval_us);
	if (!ret)
		gyro->interval_us = interval_us;
	return ret;
}

static void apple_aop_gyro_report(void *cookie, u16 subtype, const void *data, size_t len)
{
	struct iio_dev *indio_dev = cookie;
	struct apple_aop_gyro *gyro = iio_priv(indio_dev);
	unsigned long flags;
	int i;

	if (subtype != AOP_REPORT_GYRO || len < AOP_GYRO_LEN)
		return;

	spin_lock_irqsave(&gyro->sample_lock, flags);
	for (i = 0; i < 3; i++)
		gyro->sample[i] = get_unaligned_le32(data + AOP_GYRO_XYZ + 4 * i);
	gyro->have_sample = true;
	spin_unlock_irqrestore(&gyro->sample_lock, flags);

	if (iio_buffer_enabled(indio_dev)) {
		memcpy(gyro->scan, gyro->sample, sizeof(gyro->sample));
		iio_push_to_buffers_with_ts(indio_dev, gyro->scan, sizeof(gyro->scan),
					    iio_get_time_ns(indio_dev));
	}
}

static int apple_aop_gyro_read_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan,
				   int *val, int *val2, long mask)
{
	struct apple_aop_gyro *gyro = iio_priv(indio_dev);
	unsigned long flags;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		spin_lock_irqsave(&gyro->sample_lock, flags);
		*val = gyro->sample[chan->scan_index];
		ret = gyro->have_sample ? IIO_VAL_INT : -EAGAIN;
		spin_unlock_irqrestore(&gyro->sample_lock, flags);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		*val = AOP_GYRO_SCALE;
		*val2 = AOP_GYRO_SCALE_LOG2;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&gyro->lock);
		*val = USEC_PER_SEC / gyro->interval_us;
		*val2 = (u64)(USEC_PER_SEC % gyro->interval_us) * USEC_PER_SEC /
			gyro->interval_us;
		mutex_unlock(&gyro->lock);
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int apple_aop_gyro_write_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int val, int val2, long mask)
{
	struct apple_aop_gyro *gyro = iio_priv(indio_dev);
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
			   AOP_GYRO_MIN_US, AOP_GYRO_MAX_US);

	mutex_lock(&gyro->lock);
	ret = apple_aop_gyro_set_interval(gyro, interval);
	mutex_unlock(&gyro->lock);
	return ret;
}

static const struct iio_mount_matrix *
apple_aop_gyro_get_mount_matrix(const struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct apple_aop_gyro *gyro = iio_priv(indio_dev);

	return &gyro->orientation;
}

static const struct iio_chan_spec_ext_info apple_aop_gyro_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, apple_aop_gyro_get_mount_matrix),
	{ }
};

#define APPLE_AOP_GYRO_CHANNEL(axis, index) {				\
	.type = IIO_ANGL_VEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_CPU,					\
	},								\
	.ext_info = apple_aop_gyro_ext_info,				\
}

static const struct iio_chan_spec apple_aop_gyro_channels[] = {
	APPLE_AOP_GYRO_CHANNEL(X, 0),
	APPLE_AOP_GYRO_CHANNEL(Y, 1),
	APPLE_AOP_GYRO_CHANNEL(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_info apple_aop_gyro_info = {
	.read_raw = apple_aop_gyro_read_raw,
	.write_raw = apple_aop_gyro_write_raw,
};

/* Sensitivity and interrupt calibration make it complete */
static const struct apple_aop_calibration apple_aop_gyro_calibration[] = {
	{ "apple,gyro-sensitivity-calibration", 0x0b, 1 },
	{ "apple,gyro-interrupt-calibration", 0x19, 2 },
	{ "apple,gyro-range-sensitivity-cal", 0x27, 0 },
	{ "apple,gyro-trim-calibration", 0x20, 0 },
};

static void apple_aop_gyro_stop(void *data)
{
	struct apple_aop_gyro *gyro = data;

	apple_aop_gyro_set_u32(gyro, AOP_PROP_REPORT_INTERVAL, 0);
	apple_aop_unlisten(gyro->service);
}

static int apple_aop_gyro_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct apple_aop_service *service = dev_get_platdata(dev);
	struct apple_aop_gyro *gyro;
	struct iio_dev *indio_dev;
	int ret;

	if (!service)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*gyro));
	if (!indio_dev)
		return -ENOMEM;
	gyro = iio_priv(indio_dev);
	gyro->dev = dev;
	gyro->service = service;
	mutex_init(&gyro->lock);
	spin_lock_init(&gyro->sample_lock);

	ret = iio_read_mount_matrix(dev, &gyro->orientation);
	if (ret)
		return ret;

	indio_dev->name = "apple-aop-gyro";
	indio_dev->info = &apple_aop_gyro_info;
	indio_dev->channels = apple_aop_gyro_channels;
	indio_dev->num_channels = ARRAY_SIZE(apple_aop_gyro_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev, NULL);
	if (ret)
		return ret;

	ret = apple_aop_listen(service, apple_aop_gyro_report, indio_dev);
	if (ret)
		return ret;

	ret = apple_aop_calibrate(service, dev, apple_aop_gyro_calibration,
				  ARRAY_SIZE(apple_aop_gyro_calibration));
	if (!ret)
		ret = apple_aop_gyro_set_interval(gyro, AOP_GYRO_DEFAULT_US);
	if (ret) {
		apple_aop_unlisten(service);
		return dev_err_probe(dev, ret, "failed to start reports\n");
	}

	ret = devm_add_action_or_reset(dev, apple_aop_gyro_stop, gyro);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id apple_aop_gyro_of_match[] = {
	{ .compatible = "apple,s8000-aop-gyro" },
	{ .compatible = "apple,t8010-aop-gyro" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_aop_gyro_of_match);

static struct platform_driver apple_aop_gyro_driver = {
	.driver = {
		.name = "apple-aop-gyro",
		.of_match_table = apple_aop_gyro_of_match,
	},
	.probe = apple_aop_gyro_probe,
};
module_platform_driver(apple_aop_gyro_driver);

MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_DESCRIPTION("Apple AOP gyroscope driver");
MODULE_LICENSE("Dual MIT/GPL");
