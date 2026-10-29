/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Generation-1 calibration and protocol framing adapted from
 * Corellium Project Sandcastle.
 * Copyright (C) 2020 Corellium LLC
 */
#ifndef _APPLE_Z2_N61_H
#define _APPLE_Z2_N61_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define APPLE_Z2_N61_TOUCH_MAX 1024
#define APPLE_Z2_N61_PROX_MAX 128
#define APPLE_Z2_N61_CHUNK 16384
#define APPLE_Z2_N61_MAX_CHUNKS 4
#define APPLE_Z2_N61_MAX_BLOB (APPLE_Z2_N61_CHUNK * APPLE_Z2_N61_MAX_CHUNKS)
#define APPLE_Z2_N61_READY_ATTEMPTS 50

/* N1 EB includes phase 1 in the sum, then puts that sum at the wire tail. */
static inline int apple_z2_n61_interrupt_request(u8 *out, size_t size, u8 tag)
{
	if (size < 16 || size > 8192 || (tag != 1 && tag != 2))
		return -EINVAL;
	memset(out, 0, size);
	out[0] = 0xeb;
	out[1] = tag;
	out[2] = 1;
	put_unaligned_le16(0xeb + tag + 1, out + size - 2);
	return 0;
}

/* Provider IDs are the stable Z2FW calibration ABI; append new providers. */
static inline bool apple_z2_n61_calibration_target(u32 address, u32 maximum,
						  u32 provider)
{
	if (!maximum)
		return false;
	if (provider == 0)
		return address == 0x10009000 && maximum <= APPLE_Z2_N61_TOUCH_MAX;
	if (provider == 9)
		return address == 0x10009600 && maximum <= APPLE_Z2_N61_PROX_MAX;
	return false;
}

static inline bool apple_z2_n61_init_valid(const u8 *data, size_t size)
{
	static const u8 execute[] = { 0x1d, 0x53, 0x34, 0, 0x10, 0,
				      0, 1, 0, 0, 0, 0x45 };
	size_t i;

	if (size == 2)
		return data[0] == 0x19 && data[1] == 0xc1;
	if (size == sizeof(execute))
		return !memcmp(data, execute, sizeof(execute));
	if (size != 16 || data[0] != 0x1a || data[1] != 0xa1)
		return false;
	for (i = 2; i < size; i += 2)
		if (data[i] != 0x18 || data[i + 1] != 0xe1)
			return false;
	return true;
}

/* Apple N1 retains the phase-0 checksum when it sends the phase-1 request. */
static inline void apple_z2_n61_long_read_request(u8 *out, u8 report, u16 size)
{
	u16 checksum = 0xe7 + report + (size & 0xff) + (size >> 8);

	memset(out, 0, size + 5);
	out[0] = 0xe7;
	out[1] = report;
	out[2] = 1;
	put_unaligned_le16(size, out + 3);
	put_unaligned_le16(checksum, out + size + 3);
}

static inline void apple_z2_n61_short_read_request(u8 *out, u8 report, u16 size)
{
	u16 checksum = 0xe6 + report + (size & 0xff) + (size >> 8);

	memset(out, 0, 16);
	out[0] = 0xe6;
	out[1] = report;
	out[2] = 1;
	put_unaligned_le16(size, out + 3);
	put_unaligned_le16(checksum, out + 14);
}

static inline bool apple_z2_n61_rom_word_valid(u16 word)
{
	switch (word) {
	case 0x18e1:
	case 0x1aa1:
	case 0x1f01:
	case 0x4879:
	case 0x4969:
	case 0x4ad1:
	case 0x4bc1:
		return true;
	default:
		return false;
	}
}

static inline bool apple_z2_n61_rom_ready(const u8 *data, size_t size)
{
	return size >= 4 &&
		apple_z2_n61_rom_word_valid(get_unaligned_be16(data)) &&
		apple_z2_n61_rom_word_valid(get_unaligned_be16(data + 2));
}

/* Legacy HBPP carries a full word count and swaps bytes within payload pairs. */
static inline int apple_z2_n61_calibration_packet(u8 *out, size_t capacity,
						 const u8 *cal, size_t size,
						 u32 address)
{
	size_t padded = (size + 3) & ~(size_t)3;
	u32 checksum = 0;
	u16 header_checksum = 0;
	size_t i;

	if (!size || size > APPLE_Z2_N61_TOUCH_MAX ||
	    (address == 0x10009600 && size > APPLE_Z2_N61_PROX_MAX) ||
	    (address != 0x10009000 && address != 0x10009600))
		return -EINVAL;
	if (capacity < 16 + padded)
		return -EMSGSIZE;
	memset(out, 0, 16 + padded);
	put_unaligned_be16(0x18e1, out);
	put_unaligned_be16(0x3001, out + 2);
	put_unaligned_be16(padded / 4, out + 4);
	put_unaligned_be16(address, out + 6);
	put_unaligned_be16(address >> 16, out + 8);
	for (i = 4; i < 10; i++)
		header_checksum += out[i];
	put_unaligned_be16(header_checksum, out + 10);
	for (i = 0; i < size; i++) {
		out[12 + (i ^ 1)] = cal[i];
		checksum += cal[i];
	}
	put_unaligned_be16(checksum, out + 12 + padded);
	put_unaligned_be16(checksum >> 16, out + 14 + padded);
	return 16 + padded;
}
#endif
