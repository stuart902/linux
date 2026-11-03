// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/unaligned.h>
#include "t8010-pdm.h"
#include "t8010-pdm-adt.h"

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

static struct pdm_adt pdm_test_adt(struct kunit *test, const u8 *bytes, size_t size)
{
	return (struct pdm_adt) {
		.data = bytes,
		.size = size,
		.nodes = kunit_kcalloc(test, PDM_ADT_MAX_NODES,
				       sizeof(struct pdm_adt_node), GFP_KERNEL),
	};
}

static void pdm_adt_bounds_test(struct kunit *test)
{
	u8 bytes[8] = {};
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	KUNIT_ASSERT_EQ(test, pdm_adt_parse(&tree), 0);
	KUNIT_EXPECT_EQ(test, tree.count, 1U);
	tree.size = 7;
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
	tree.size = PDM_ADT_MAX_BYTES + 1;
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
	tree.size = sizeof(bytes);
	put_unaligned_le32(PDM_ADT_MAX_PROPERTIES + 1, bytes);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
}

static void pdm_adt_property_bounds_test(struct kunit *test)
{
	u8 bytes[44] = {};
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	put_unaligned_le32(1, bytes);
	memcpy(bytes + 8, "name", 5);
	KUNIT_ASSERT_EQ(test, pdm_adt_parse(&tree), 0);
	put_unaligned_le32(1, bytes + 40);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
	put_unaligned_le32(0xffffff, bytes + 40);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
	put_unaligned_le32(0, bytes + 40);
	memset(bytes + 8, 'x', 32);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
}

static void pdm_adt_duplicate_property_test(struct kunit *test)
{
	u8 bytes[80] = {};
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	put_unaligned_le32(2, bytes);
	memcpy(bytes + 8, "name", 5);
	memcpy(bytes + 44, "name", 5);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
}

static void pdm_adt_rate_records_test(struct kunit *test)
{
	u8 bytes[60] = {};
	u32 rates[2];
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	put_unaligned_le32(1, bytes);
	memcpy(bytes + 8, "rates", 6);
	put_unaligned_le32(16, bytes + 40);
	put_unaligned_le32(48000, bytes + 48);
	put_unaligned_le32(44100, bytes + 56);
	KUNIT_ASSERT_EQ(test, pdm_adt_parse(&tree), 0);
	KUNIT_ASSERT_EQ(test, pdm_adt_rates(&tree, 0, "rates", rates, 2), 2);
	KUNIT_EXPECT_EQ(test, rates[0], 48000U);
	KUNIT_EXPECT_EQ(test, rates[1], 44100U);
	KUNIT_EXPECT_EQ(test, pdm_adt_rates(&tree, 0, "rates", rates, 1), -E2BIG);
	put_unaligned_le32(48000, bytes + 56);
	KUNIT_EXPECT_EQ(test, pdm_adt_rates(&tree, 0, "rates", rates, 2), -EINVAL);
	put_unaligned_le32(1, bytes + 44);
	KUNIT_EXPECT_EQ(test, pdm_adt_rates(&tree, 0, "rates", rates, 2), -EINVAL);
}

static void pdm_adt_duplicate_phandle_test(struct kunit *test)
{
	u8 bytes[104] = {};
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	put_unaligned_le32(2, bytes + 4);
	put_unaligned_le32(1, bytes + 8);
	memcpy(bytes + 16, "AAPL,phandle", 13);
	put_unaligned_le32(4, bytes + 48);
	put_unaligned_le32(149, bytes + 52);
	put_unaligned_le32(1, bytes + 56);
	memcpy(bytes + 64, "AAPL,phandle", 13);
	put_unaligned_le32(4, bytes + 96);
	put_unaligned_le32(149, bytes + 100);
	KUNIT_ASSERT_EQ(test, pdm_adt_parse(&tree), 0);
	KUNIT_EXPECT_EQ(test, pdm_adt_phandle(&tree, 149), -EINVAL);
	put_unaligned_le32(150, bytes + 100);
	KUNIT_EXPECT_EQ(test, pdm_adt_phandle(&tree, 149), 1);
	KUNIT_EXPECT_EQ(test, pdm_adt_phandle(&tree, 150), 2);
}

static void pdm_adt_depth_test(struct kunit *test)
{
	u8 bytes[(PDM_ADT_MAX_DEPTH + 2) * 8] = {};
	struct pdm_adt tree = pdm_test_adt(test, bytes, sizeof(bytes));
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, tree.nodes);
	for (i = 0; i < PDM_ADT_MAX_DEPTH + 1; i++)
		put_unaligned_le32(1, bytes + i * 8 + 4);
	KUNIT_EXPECT_EQ(test, pdm_adt_parse(&tree), -EINVAL);
}

static struct kunit_case pdm_cases[] = {
	KUNIT_CASE(pdm_adt_bounds_test),
	KUNIT_CASE(pdm_adt_property_bounds_test),
	KUNIT_CASE(pdm_adt_duplicate_property_test),
	KUNIT_CASE(pdm_adt_rate_records_test),
	KUNIT_CASE(pdm_adt_duplicate_phandle_test),
	KUNIT_CASE(pdm_adt_depth_test),

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
