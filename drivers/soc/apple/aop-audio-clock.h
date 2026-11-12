/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_AOP_AUDIO_CLOCK_H
#define _APPLE_AOP_AUDIO_CLOCK_H

#include "aop-audio-protocol.h"

enum aop_audio_clock_state {
	AOP_AUDIO_CLOCK_UNKNOWN,
	AOP_AUDIO_CLOCK_OFF,
	AOP_AUDIO_CLOCK_ON,
	AOP_AUDIO_CLOCK_FAULTED,
};

struct aop_audio_clock {
	enum aop_audio_clock_state state;
	u32 requested_rate;
	u32 domain;
};

typedef int (*aop_audio_exchange_fn)(void *cookie, const void *request,
				   size_t request_len, void *reply,
				   size_t reply_capacity, size_t requested_len,
				   size_t *reply_len);

/*
 * Caller is the exclusive Linux host requester and serializes all transitions.
 * A zero rate is an explicit OFF command, including initial UNKNOWN -> OFF.
 * SET acknowledgement establishes host request state; GET validates the property
 * exchange, not measured frequency or hardware state. Any failed exchange
 * makes ownership uncertain and prohibits retries until a verified reboot.
 */
static inline int aop_audio_clock_program(struct aop_audio_clock *clock, u32 rate,
					 aop_audio_exchange_fn exchange, void *cookie)
{
	u8 request[AOP_AUDIO_CLOCK_SET_SIZE];
	u8 reply[AOP_AUDIO_INLINE_MAX];
	size_t actual, len;
	u32 domain;
	int ret;

	if (clock->state == AOP_AUDIO_CLOCK_FAULTED)
		return -EIO;
	if (rate && clock->state != AOP_AUDIO_CLOCK_OFF)
		return -EBUSY;
	if (!rate && clock->state == AOP_AUDIO_CLOCK_OFF)
		return 0;

	/* Even a failed START can have enabled the firmware clock. */
	clock->state = AOP_AUDIO_CLOCK_FAULTED;
	len = aop_audio_clock_request(request, true, rate);
	actual = 0;
	/*
	 * SET has no result fields, but firmware can return its opaque inline
	 * scratch buffer. Keep the advertised result length zero, as on Apple,
	 * while bounding local storage independently. Never interpret or log it.
	 */
	ret = exchange(cookie, request, len, reply, sizeof(reply), 0, &actual);
	if (ret)
		return ret < 0 ? ret : -EREMOTEIO;
	if (actual > sizeof(reply))
		return -EPROTO;
	len = aop_audio_clock_request(request, false, 0);
	actual = 0;
	ret = exchange(cookie, request, len, reply, AOP_AUDIO_CLOCK_REPLY_SIZE,
		       AOP_AUDIO_CLOCK_REPLY_SIZE, &actual);
	if (ret)
		return ret < 0 ? ret : -EREMOTEIO;
	ret = aop_audio_clock_domain(reply, actual, &domain);
	if (ret)
		return ret;
	clock->domain = domain;
	clock->requested_rate = rate;
	clock->state = rate ? AOP_AUDIO_CLOCK_ON : AOP_AUDIO_CLOCK_OFF;
	return 0;
}

#endif
