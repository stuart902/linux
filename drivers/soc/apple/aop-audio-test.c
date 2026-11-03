// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>

#include "aop-audio-clock.h"

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

struct aop_audio_mock {
	unsigned int calls;
	unsigned int fail_call;
	int error;
	size_t set_reply_len;
};

static int aop_audio_mock_exchange(void *cookie, const void *request,
				   size_t request_len, void *reply,
				   size_t capacity, size_t requested_len, size_t *actual)
{
	struct aop_audio_mock *mock = cookie;

	mock->calls++;
	if (mock->calls == mock->fail_call)
		return mock->error ?: -EIO;
	if (request_len < 12)
		return -EINVAL;
	if (get_unaligned_le32(request + 8) == 0xc3000004) {
		if (capacity != AOP_AUDIO_CLOCK_REPLY_SIZE || requested_len != capacity)
			return -EINVAL;
		put_unaligned_le32(4, reply);
		put_unaligned_le32(0x1234, reply + 4);
		*actual = AOP_AUDIO_CLOCK_REPLY_SIZE;
	} else if (get_unaligned_le32(request + 8) == 0xc3000005) {
		if (requested_len || capacity != AOP_AUDIO_INLINE_MAX)
			return -EINVAL;
		*actual = mock->set_reply_len;
		if (*actual > capacity)
			return -EMSGSIZE;
		memset(reply, 0xa5, *actual);
	} else {
		return -EINVAL;
	}
	return 0;
}

static void aop_audio_clock_initial_test(struct kunit *test)
{
	struct aop_audio_clock clock = {};
	struct aop_audio_mock mock = {};

	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), -EBUSY);
	KUNIT_EXPECT_EQ(test, mock.calls, 0U);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), 0);
	KUNIT_EXPECT_EQ(test, mock.calls, 2U);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_OFF);
}

static void aop_audio_clock_cycle_test(struct kunit *test)
{
	struct aop_audio_clock clock = { .state = AOP_AUDIO_CLOCK_OFF };
	struct aop_audio_mock mock = {};

	KUNIT_ASSERT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), 0);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_ON);
	KUNIT_EXPECT_EQ(test, clock.requested_rate, 3072000U);
	KUNIT_EXPECT_EQ(test, clock.domain, 0x1234U);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 2400000,
			 aop_audio_mock_exchange, &mock), -EBUSY);
	KUNIT_EXPECT_EQ(test, mock.calls, 2U);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), 0);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_OFF);
	KUNIT_EXPECT_EQ(test, mock.calls, 4U);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), 0);
	KUNIT_EXPECT_EQ(test, mock.calls, 4U);
}

static void aop_audio_clock_start_failure_test(struct kunit *test)
{
	struct aop_audio_clock clock = { .state = AOP_AUDIO_CLOCK_OFF };
	struct aop_audio_mock mock = { .fail_call = 2 };

	/* SET succeeds, GET fails: START must not be unwound as known OFF. */
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), -EIO);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_FAULTED);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), -EIO);
	KUNIT_EXPECT_EQ(test, mock.calls, 2U);
}

static void aop_audio_clock_stop_failure_test(struct kunit *test)
{
	struct aop_audio_clock clock = { .state = AOP_AUDIO_CLOCK_ON };
	struct aop_audio_mock mock = { .fail_call = 1 };

	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), -EIO);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_FAULTED);
	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), -EIO);
	KUNIT_EXPECT_EQ(test, mock.calls, 1U);
}

static void aop_audio_clock_rejected_test(struct kunit *test)
{
	struct aop_audio_clock clock = { .state = AOP_AUDIO_CLOCK_OFF };
	struct aop_audio_mock mock = { .fail_call = 1, .error = 7 };

	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), -EREMOTEIO);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_FAULTED);
	KUNIT_EXPECT_EQ(test, mock.calls, 1U);
}

static void aop_audio_clock_set_reply_test(struct kunit *test)
{
	struct aop_audio_clock clock = { .state = AOP_AUDIO_CLOCK_OFF };
	struct aop_audio_mock mock = { .set_reply_len = AOP_AUDIO_INLINE_MAX + 1 };

	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 3072000,
			 aop_audio_mock_exchange, &mock), -EMSGSIZE);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_FAULTED);
	KUNIT_EXPECT_EQ(test, mock.calls, 1U);
}

static void aop_audio_clock_opaque_set_test(struct kunit *test)
{
	struct aop_audio_clock clock = {};
	struct aop_audio_mock mock = { .set_reply_len = AOP_AUDIO_INLINE_MAX };

	KUNIT_EXPECT_EQ(test, aop_audio_clock_program(&clock, 0,
			 aop_audio_mock_exchange, &mock), 0);
	KUNIT_EXPECT_EQ(test, clock.state, AOP_AUDIO_CLOCK_OFF);
	KUNIT_EXPECT_EQ(test, mock.calls, 2U);
}

static struct kunit_case aop_audio_cases[] = {
	KUNIT_CASE(aop_audio_set_test),
	KUNIT_CASE(aop_audio_off_test),
	KUNIT_CASE(aop_audio_get_test),
	KUNIT_CASE(aop_audio_domain_test),
	KUNIT_CASE(aop_audio_clock_initial_test),
	KUNIT_CASE(aop_audio_clock_cycle_test),
	KUNIT_CASE(aop_audio_clock_start_failure_test),
	KUNIT_CASE(aop_audio_clock_stop_failure_test),
	KUNIT_CASE(aop_audio_clock_rejected_test),
	KUNIT_CASE(aop_audio_clock_set_reply_test),
	KUNIT_CASE(aop_audio_clock_opaque_set_test),
	{}
};

static struct kunit_suite aop_audio_suite = {
	.name = "apple-aop-audio",
	.test_cases = aop_audio_cases,
};
kunit_test_suite(aop_audio_suite);

MODULE_LICENSE("GPL");
