// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>

#include "aop-audio-protocol.h"

static void aop_audio_set_test(struct kunit *test)
{
	u8 request[AOP_AUDIO_CLOCK_SET_SIZE];
	u32 checksum;

	KUNIT_EXPECT_EQ(test, aop_audio_clock_request(request, true, 3072000),
			(size_t)AOP_AUDIO_CLOCK_SET_SIZE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request), 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 4), 0xffffffffU);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 8), 0xc3000005U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 32), 56U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 36), 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 40), 0x70646d30U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 44), 0xc9U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 48), 8U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 52), 0x78693020U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 56), 3072000U);
	checksum = 0x70646d30U ^ 0xc9U ^ 8U ^ 0x78693020U ^ 3072000U;
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 28), checksum);
}

static void aop_audio_off_test(struct kunit *test)
{
	u8 request[AOP_AUDIO_CLOCK_SET_SIZE];

	aop_audio_clock_request(request, true, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 52), 0x6f666620U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 56), 0U);
}

static void aop_audio_get_test(struct kunit *test)
{
	u8 request[AOP_AUDIO_CLOCK_SET_SIZE];

	memset(request, 0xa5, sizeof(request));
	KUNIT_EXPECT_EQ(test, aop_audio_clock_request(request, false, 0),
			(size_t)AOP_AUDIO_CLOCK_GET_SIZE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 8), 0xc3000004U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 32), 48U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 44), 0xcbU);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 48), 1U);
	KUNIT_EXPECT_EQ(test, request[AOP_AUDIO_CLOCK_GET_SIZE], (u8)0xa5);
}

static void aop_audio_domain_test(struct kunit *test)
{
	u8 reply[12] = {};
	u32 domain = 0x12345678;

	put_unaligned_le32(4, reply);
	put_unaligned_le32(0xdeadbeef, reply + 4);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_domain(reply, 7, &domain), -EPROTO);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_domain(reply, 12, &domain), -EPROTO);
	KUNIT_EXPECT_EQ(test, domain, 0x12345678U);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_domain(reply, 8, &domain), 0);
	KUNIT_EXPECT_EQ(test, domain, 0xdeadbeefU);
	put_unaligned_le32(8, reply);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_domain(reply, 8, &domain), -EPROTO);
}

static struct kunit_case aop_audio_cases[] = {
	KUNIT_CASE(aop_audio_set_test),
	KUNIT_CASE(aop_audio_off_test),
	KUNIT_CASE(aop_audio_get_test),
	KUNIT_CASE(aop_audio_domain_test),
	{}
};

static struct kunit_suite aop_audio_suite = {
	.name = "apple-aop-audio",
	.test_cases = aop_audio_cases,
};
kunit_test_suite(aop_audio_suite);

MODULE_LICENSE("GPL");
