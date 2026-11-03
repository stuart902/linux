// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/module.h>

#include "t8010-mca-reference.h"

static void t8010_reference_slots_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, t8010_mca_reference_slots(0xf, 0, 8, 16));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_slots(3, 0, 2, 16));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_slots(0xf, 1, 8, 16));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_slots(0xf, 0, 8, 32));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_slots(0xff, 0, 8, 16));
}

static void t8010_reference_params_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, t8010_mca_reference_params(4, 16, 16, 48000));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_params(2, 16, 16, 48000));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_params(4, 16, 32, 48000));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_params(4, 32, 32, 48000));
	KUNIT_EXPECT_FALSE(test, t8010_mca_reference_params(4, 16, 16, 44100));
}

static void t8010_reference_encoding_test(struct kunit *test)
{
	/* Field derivation is independent of the register constants. */
	u32 frame = (1U << 20) | ((8U - 1) << 12) | 1;
	u32 tx = (1U << 8) | 0x33;

	KUNIT_EXPECT_EQ(test, T8010_MCA_REFERENCE_FRAME, frame);
	KUNIT_EXPECT_EQ(test, T8010_MCA_REFERENCE_TX, tx);
	KUNIT_EXPECT_EQ(test, T8010_MCA_REFERENCE_RX, (tx | 4) & ~1U);
	KUNIT_EXPECT_EQ(test, 48000U * 8 * 16, 6144000U);
}

static struct kunit_case t8010_reference_cases[] = {
	KUNIT_CASE(t8010_reference_slots_test),
	KUNIT_CASE(t8010_reference_params_test),
	KUNIT_CASE(t8010_reference_encoding_test),
	{}
};

static struct kunit_suite t8010_reference_suite = {
	.name = "t8010-mca-reference",
	.test_cases = t8010_reference_cases,
};

kunit_test_suite(t8010_reference_suite);

MODULE_LICENSE("Dual MIT/GPL");
