/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple VXD - HEVC (H.265) command-stream builder
 *
 * Pure code: no MMIO, no DMA API, no locking, so it also builds on the host
 * (scripts/vdec/host/vdec-hevc-host.c) and its words can be compared with
 * the reference generator.
 */
#ifndef __APPLE_VXD_HEVC_CMD_H__
#define __APPLE_VXD_HEVC_CMD_H__

#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/v4l2-controls.h>

#include "apple-vxd-cmd.h"

#define APPLE_VXD_HEVC_MAX_SLICES	127	/* + the kind-4 packet <= 128 */
#define APPLE_VXD_HEVC_MAX_REFS		15	/* hardware reference slots */
#define APPLE_VXD_HEVC_SIDE_SIZE	SZ_256K	/* slice structure buffer */
#define APPLE_VXD_HEVC_SPDEST_SIZE	SZ_1M	/* slice parameter buffer */
#define APPLE_VXD_HEVC_INTRA_SIZE	SZ_64K
#define APPLE_VXD_HEVC_GENC_BUFS	4

/* Per-SoC HEVC parameters (match data). */
struct apple_vxd_hevc_caps {
	u8 buf_mult;		/* 1, or 2 from vxd-version 9 on */
	bool tail_7474;		/* two 0x7474 writes after every slice structure */
	u16 cache_base;		/* 0x701c line split base */
};

/*
 * What a decoded picture keeps for later use as the co-located picture
 * (the kext's 0x1c-byte picture record): its 0x5438 and 0x5440..0x544c
 * words and its temporal MV buffer.  It lives in the CAPTURE buffer's
 * side buffer, so it follows the picture.
 */
struct apple_vxd_hevc_record {
	u32 refs;		/* 0x5438 */
	u32 poc_delta[4];	/* 0x5440..0x544c */
	u32 mv;			/* 0x5434 */
	u32 valid;
};

/* Per-CAPTURE side buffer layout (AllocatePictureMemory sizes). */
struct apple_vxd_hevc_aux_layout {
	u32 mv, frag, above, record, size;
};

struct apple_vxd_hevc_ref {
	u32 luma;
	u32 chroma;
	const struct apple_vxd_hevc_record *rec;
};

/* One located slice NAL in the OUTPUT buffer. */
struct apple_vxd_hevc_nal {
	u32 offset;		/* first NAL header byte */
	u32 size;		/* through the last byte before the next start code */
	u32 header_bits;	/* RBSP bits from the NAL header to slice_data() */
};

struct apple_vxd_hevc_frame {
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	const struct v4l2_ctrl_hevc_decode_params *dp;
	const struct v4l2_ctrl_hevc_scaling_matrix *sm;	/* NULL: none set */
	const struct v4l2_ctrl_hevc_slice_params *slices;
	const struct apple_vxd_hevc_nal *nals;
	unsigned int nslices;
	const struct apple_vxd_hevc_caps *caps;

	/* Device addresses. */
	u32 src;				/* OUTPUT buffer */
	u32 dst_luma, dst_chroma, dst_stride, dst_cstride;
	u32 mv, frag, above;			/* this picture's side buffer */
	struct apple_vxd_hevc_ref refs[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u32 genc[APPLE_VXD_HEVC_GENC_BUFS];
	u32 genc_size;
	u32 side;				/* slice structures, CPU-written */
	u8 *side_cpu;
	size_t side_size;
	u32 spdest;				/* their transfer destination */
	u32 intra, auxline, stream_state;

	/* Session state (C[13c469], C[136254]); the caller updates both. */
	bool first_slice_of_session;
	bool first_picture_of_session;

	/* Output: this picture's record (in its side buffer). */
	struct apple_vxd_hevc_record *record;
};

u32 apple_vxd_hevc_genc_size(unsigned int width, unsigned int height);
void apple_vxd_hevc_aux_layout(const struct v4l2_ctrl_hevc_sps *sps,
			       struct apple_vxd_hevc_aux_layout *l);
u32 apple_vxd_hevc_aux_size(unsigned int width, unsigned int height);
int apple_vxd_hevc_validate(const struct v4l2_ctrl_hevc_sps *sps,
			    const struct v4l2_ctrl_hevc_pps *pps,
			    const char **why);
int apple_vxd_hevc_find_nals(const u8 *src, size_t len,
			     const struct v4l2_ctrl_hevc_slice_params *slices,
			     unsigned int nslices, struct apple_vxd_hevc_nal *nals,
			     const char **why);
int apple_vxd_hevc_build(struct apple_vxd_cmdbuf *cb,
			 const struct apple_vxd_hevc_frame *f,
			 const char **why);

#endif /* __APPLE_VXD_HEVC_CMD_H__ */
