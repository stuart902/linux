/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __T8010_MCA_REFERENCE_H
#define __T8010_MCA_REFERENCE_H

#include <linux/types.h>

/*
 * J71b speaker profile recovered from its Apple transport configuration:
 * four active TX slots in an eight-slot, 128-bit frame; 16-bit words,
 * one-bit frame pulse and one-bit data delay. These are register values,
 * not an assignment of unverified ALSA electrical polarity names.
 */
#define T8010_MCA_REFERENCE_FRAME	0x00107001U
#define T8010_MCA_REFERENCE_TX		0x00000133U
#define T8010_MCA_REFERENCE_RX		0x00000136U

static inline bool t8010_mca_reference_slots(unsigned int tx, unsigned int rx,
					     int slots, int width)
{
	return tx == 0xf && rx == 0 && slots == 8 && width == 16;
}

static inline bool t8010_mca_reference_params(unsigned int channels,
					      unsigned int bits,
					      unsigned int physical_bits,
					      unsigned int rate)
{
	/* Keep the existing T8010 clock family restriction. */
	return channels == 4 && bits == 16 && physical_bits == 16 && rate == 48000;
}
#endif
