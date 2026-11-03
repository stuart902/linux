// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/unaligned.h>
#include "t8010-pdm.h"

static struct pdm_filter pdm_test_filter(void)
{
	/* Synthetic metadata only; these are not device filter coefficients. */
	return (struct pdm_filter) {
		.rate = 44100, .frequency = 2823530, .latency = 7,
		.ratios = { 8, 4, 2 }, .lengths = { 1, 2, 3 },
	};
}

static void pdm_valid_filter_test(struct kunit *test)
{
	struct pdm_filter f = pdm_test_filter();

	KUNIT_ASSERT_EQ(test, pdm_filter_validate(&f, 0x1030, 48), 0);
	KUNIT_EXPECT_EQ(test, f.nbytes, (size_t)48);
	KUNIT_EXPECT_EQ(test, f.ratio_reg, 63U | 8U << 10 | 4U << 16 | 2U << 22);
	KUNIT_EXPECT_EQ(test, f.length_reg, 0x030201U);
	/* Clock data must not be replaced with rate * ratios. */
	KUNIT_EXPECT_EQ(test, f.frequency, 2823530U);
}

static void pdm_bad_bounds_test(struct kunit *test)
{
	struct pdm_filter f = pdm_test_filter();

	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x102f, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0xfff, 48), -EINVAL);
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 47), -EINVAL);
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 49), -EINVAL);
	f.lengths[0] = 0;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 40), -EINVAL);
	f = pdm_test_filter();
	f.ratios[0] = 0;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
	f.ratios[0] = 64;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
	f.ratios[0] = 33; /* 264: individual fields fit, product does not. */
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
	f = pdm_test_filter();
	f.rate = 0;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
	f.rate = 192001;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
	f.rate = 48000;
	f.frequency = 0;
	KUNIT_EXPECT_EQ(test, pdm_filter_validate(&f, 0x4000, 48), -EINVAL);
}

static void pdm_priming_test(struct kunit *test)
{
	size_t bytes;

	KUNIT_ASSERT_EQ(test, pdm_discard_size(44100, 50, &bytes), 0);
	KUNIT_EXPECT_EQ(test, bytes, (size_t)8820);
	KUNIT_ASSERT_EQ(test, pdm_discard_size(44100, 0, &bytes), 0);
	KUNIT_EXPECT_EQ(test, bytes, (size_t)884); /* ceil(220.5) stereo frames */
	KUNIT_ASSERT_EQ(test, pdm_discard_size(192000, 1000, &bytes), 0);
	KUNIT_EXPECT_EQ(test, bytes, (size_t)768000);
	KUNIT_EXPECT_EQ(test, pdm_discard_size(48000, 1001, &bytes), -EINVAL);
	KUNIT_EXPECT_EQ(test, pdm_discard_size(0, 50, &bytes), -EINVAL);
}

static void pdm_coefficient_endian_test(struct kunit *test)
{
	static const u8 synthetic[] = { 0xff, 0x78, 0x56, 0x34, 0x12 };

	KUNIT_EXPECT_EQ(test, get_unaligned_le32(synthetic + 1), 0x12345678U);
}

static void pdm_coalesced_position_test(struct kunit *test)
{
	size_t position;

	/* One callback may cover three completed periods. */
	KUNIT_ASSERT_EQ(test, pdm_position_bytes(4096, 4096 - 3 * 512, &position), 0);
	KUNIT_EXPECT_EQ(test, position, (size_t)1536);
	KUNIT_ASSERT_EQ(test, pdm_position_bytes(4096, 4096, &position), 0);
	KUNIT_EXPECT_EQ(test, position, (size_t)0);
	KUNIT_ASSERT_EQ(test, pdm_position_bytes(4096, 0, &position), 0);
	KUNIT_EXPECT_EQ(test, position, (size_t)0);
	KUNIT_EXPECT_EQ(test, pdm_position_bytes(4096, 4097, &position), -EINVAL);
	KUNIT_EXPECT_EQ(test, pdm_position_bytes(0, 0, &position), -EINVAL);
}

static struct kunit_case pdm_cases[] = {
	KUNIT_CASE(pdm_valid_filter_test),
	KUNIT_CASE(pdm_bad_bounds_test),
	KUNIT_CASE(pdm_priming_test),
	KUNIT_CASE(pdm_coefficient_endian_test),
	KUNIT_CASE(pdm_coalesced_position_test),
	{}
};

static struct kunit_suite pdm_suite = {
	.name = "apple-t8010-pdm",
	.test_cases = pdm_cases,
};

kunit_test_suite(pdm_suite);
MODULE_DESCRIPTION("Apple T8010 PDM calibration and PCM arithmetic tests");
MODULE_LICENSE("GPL");
