/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __APPLE_T8010_PDM_H
#define __APPLE_T8010_PDM_H

#include <linux/errno.h>
#include <linux/math.h>
#include <linux/types.h>

#define PDM_DECIMATORS 2
#define PDM_MAX_RATES 16
#define PDM_COEFF_BASE 0x1000
#define PDM_MAX_DELAY_MS 1000

struct pdm_filter {
	u32 rate, frequency, latency;
	u8 ratios[3], lengths[3];
	u32 ratio_reg, length_reg;
	size_t nbytes;
	const u8 *coefficients;
};

/* Called before allocation, upload or clock programming. */
static inline int pdm_filter_validate(struct pdm_filter *f, u64 resource_bytes,
				      size_t coefficient_bytes)
{
	u32 product = 1, sum = 0;
	unsigned int i;

	if (!f->rate || f->rate > 192000 || !f->frequency)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (!f->ratios[i] || f->ratios[i] > 63 || !f->lengths[i])
			return -EINVAL;
		product *= f->ratios[i];
		sum += f->lengths[i];
	}
	/* Three u8 operands cannot overflow these u32 calculations. */
	if (product > 256)
		return -EINVAL;
	f->nbytes = 8 * sum;
	if (coefficient_bytes != f->nbytes || resource_bytes < PDM_COEFF_BASE ||
	    f->nbytes > resource_bytes - PDM_COEFF_BASE)
		return -EINVAL;
	f->ratio_reg = (product - 1) | f->ratios[0] << 10 |
		      f->ratios[1] << 16 | f->ratios[2] << 22;
	f->length_reg = f->lengths[0] | f->lengths[1] << 8 |
		       f->lengths[2] << 16;
	return 0;
}

static inline int pdm_discard_size(u32 rate, u32 delay_ms, size_t *bytes)
{
	if (!rate || rate > 192000 || delay_ms > PDM_MAX_DELAY_MS)
		return -EINVAL;
	/* Full >=delay interval; stereo S16, rounded upwards to a full frame. */
	*bytes = DIV_ROUND_UP_ULL((u64)rate * max(delay_ms, 5U), 1000) *
		 PDM_DECIMATORS * 2;
	return 0;
}

/* Residue comes from the DMA provider's completed-period count, not callbacks. */
static inline int pdm_position_bytes(size_t buffer, size_t residue, size_t *position)
{
	if (!buffer || residue > buffer)
		return -EINVAL;
	*position = residue ? buffer - residue : 0;
	return 0;
}
#endif
