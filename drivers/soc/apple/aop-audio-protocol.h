/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_AOP_AUDIO_PROTOCOL_H
#define _APPLE_AOP_AUDIO_PROTOCOL_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define AOP_AUDIO_SUBTYPE 0x20
#define AOP_AUDIO_CLOCK_SET_SIZE 60
#define AOP_AUDIO_CLOCK_GET_SIZE 52
#define AOP_AUDIO_CLOCK_REPLY_SIZE 8

/*
 * Inline audio requests contain a status word, a 36-byte wrapped-call header,
 * then a property body. The checksum is XOR of the body's little-endian words.
 * Deliberately zero-initialize reserved fields; do not use them as cookies
 * or a high length word.
 */
static inline size_t aop_audio_clock_request(u8 *request, bool set, u32 rate)
{
	u8 *wrapper = request + 4;
	u8 *body = wrapper + 36;
	size_t body_len = set ? 20 : 12;
	size_t len = 4 + 36 + body_len;
	u32 checksum = 0;
	size_t i;

	memset(request, 0, len);
	put_unaligned_le32(0xffffffff, wrapper);
	put_unaligned_le32(set ? 0xc3000005 : 0xc3000004, wrapper + 4);
	put_unaligned_le32(36 + body_len, wrapper + 28);
	put_unaligned_le32(0x70646d30, body); /* pdm0 */
	put_unaligned_le32(set ? 0xc9 : 0xcb, body + 4);
	put_unaligned_le32(set ? 8 : 1, body + 8);
	if (set) {
		put_unaligned_le32(rate ? 0x78693020 : 0x6f666620, body + 12);
		put_unaligned_le32(rate, body + 16);
	}
	for (i = 0; i < body_len; i += sizeof(u32))
		checksum ^= get_unaligned_le32(body + i);
	put_unaligned_le32(checksum, wrapper + 24);
	return len;
}

/* The returned domain is opaque: it is neither a rate nor an enable flag. */
static inline int aop_audio_clock_domain(const void *reply, size_t len, u32 *domain)
{
	if (len != AOP_AUDIO_CLOCK_REPLY_SIZE || get_unaligned_le32(reply) != 4)
		return -EPROTO;
	*domain = get_unaligned_le32(reply + 4);
	return 0;
}

#endif
