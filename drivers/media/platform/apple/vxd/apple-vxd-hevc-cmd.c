// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - HEVC (H.265) command-stream builder
 *
 * The HEVC path of the kext (picture path 0xfffffff0062960a0; per slice
 * 0x62b37d4, per picture 0x62b7ab0; docs/periph/vdec-hevc-20261110.md in
 * the hardware bring-up records has the word-level map).  Per picture:
 *
 *   per slice segment, one chunk and one kind-1 packet (mailbox 0x81):
 *     0x90000000 + 9 words      slice descriptor
 *     0xb + 0xa                 the whole slice NAL from its header byte;
 *                               the core skips the header (RBSP bits)
 *     0x1 register runs         CMD block 0x30xx / 0x34xx
 *     0x3090 / 0x3094           slice structure destination / lines,
 *                               written after the structure is built
 *     0xf                       slice structure transfer (independent
 *                               segments): built in a CPU buffer, copied
 *                               by the core into the slice parameter buffer
 *     0x60000000
 *     then zero padding to the next 64 bytes, not counted
 *   then one chunk of single register writes (0x000f0000 form) and one
 *   kind-4 packet (mailbox 0x88): entropy registers 0x50xx/0x54xx, the
 *   scaling lists 0x5800.., and the render registers 0x70xx/0x71xx with
 *   the output and reference surfaces.  The 0x88 message ends the job.
 *
 * Reference slots: the hardware has 15, and every list in the stream
 * (0x7100 surfaces, 0x5438 long-term bits, 0x5440 POC deltas, the slice
 * structures' reference indices) names them by number.  Apple numbers them
 * in its own DPB order; this driver uses the V4L2 DPB index.
 *
 * Supported: 8-bit 4:2:0 (Main, Main Still Picture), I/P/B, multiple
 * slices and dependent slice segments, tiles, WPP, long-term references,
 * weighted prediction, PCM, scaling lists.  Refused: other chroma formats
 * and bit depths (the output layout is not established), range extensions,
 * more than 15 references.
 */
#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/string.h>

#include "apple-vxd-hevc-cmd.h"

/* Slice-level CMD block (MSVDX command registers). */
#define HEVC_CMD_SP_DEST		0x3090	/* slice structure address */
#define HEVC_CMD_SP_LINES		0x3094	/* its size in 64-byte lines - 1 */
#define HEVC_CMD_FRAGMENT		0x3098
#define HEVC_CMD_FORMAT			0x30a0
#define HEVC_CMD_SIZE			0x30a8
#define HEVC_CMD_SLICE_POS		0x30b0
#define HEVC_CMD_RESET			0x3080
#define HEVC_CMD_RESET_ZERO		0x3120	/* x4 */
#define HEVC_CMD_GENC_SIZE		0x3100	/* x4, interleaved with: */
#define HEVC_CMD_GENC_BASE		0x3110
#define HEVC_CMD_SPS0			0x3400
#define HEVC_CMD_SPS1			0x3404
#define HEVC_CMD_PPS0			0x3408
#define HEVC_CMD_SEGMENT		0x3410
#define HEVC_CMD_SLICE0			0x3414
#define HEVC_CMD_TILE_COLS		0x3440	/* x12 */
#define HEVC_CMD_TILE_ROWS		0x3480	/* x12 */
/* Entropy block. */
#define HEVC_ENT_CONTROL		0x5000
#define HEVC_ENT_SIZE8			0x5010
#define HEVC_ENT_FRAGMENT		0x5030
#define HEVC_ENT_GENC_SIZE		0x5040
#define HEVC_ENT_GENC_BASE		0x5050
#define HEVC_ENT_ABOVE			0x50c0
#define HEVC_ENT_SPS0			0x5400	/* .. 0x540c */
#define HEVC_ENT_COL_MV			0x5430
#define HEVC_ENT_CUR_MV			0x5434
#define HEVC_ENT_REFS			0x5438
#define HEVC_ENT_COL_REFS		0x543c
#define HEVC_ENT_POC_DELTA		0x5440	/* x4 */
#define HEVC_ENT_COL_POC_DELTA		0x5450	/* x4 */
#define HEVC_ENT_TILE_COLS		0x5480	/* x12 */
#define HEVC_ENT_TILE_ROWS		0x54c0	/* x12 */
#define HEVC_ENT_SCALING		0x5800
/* Render block. */
#define HEVC_REN_DST			0x700c	/* Y, UV */
#define HEVC_REN_INTRA			0x7018
#define HEVC_REN_CACHE			0x701c
#define HEVC_REN_WEIGHT_DENOM		0x7024
#define HEVC_REN_LAYOUT			0x703c
#define HEVC_REN_STRIDE			0x7040
#define HEVC_REN_PIC_CONFIG		0x70a0
#define HEVC_REN_CHROMA_QP		0x70a4
#define HEVC_REN_SIZE			0x70b0
#define HEVC_REN_SIZE8			0x70b4
#define HEVC_REN_REFS			0x7100	/* Y, UV per slot */
#define HEVC_REN_CSTRIDE		0x71ac
#define HEVC_REN_FORMAT			0x71b4
#define HEVC_REN_AUXLINE		0x71ec
#define HEVC_REN_WEIGHTS(l)		(0x7200 + 0x100 * (l))
#define HEVC_REN_OFFSETS(l)		(0x7280 + 0x100 * (l))
#define HEVC_REN_7474			0x7474

#define HEVC_F_ADDR			0x10	/* 0x000f0000 flag on addresses */

/* Largest slice structure: B header, 1 + 2 x 2 x 15 weight pairs, 0x7474 x 2. */
#define HEVC_STRUCT_MAX			(48 + 8 * 63 + 0x38)

/* The linear-output layout word (0x62b8b0c with tiling off, aux line on). */
#define HEVC_LAYOUT_LINEAR		0x400

enum {
	HEVC_B = V4L2_HEVC_SLICE_TYPE_B,
	HEVC_P = V4L2_HEVC_SLICE_TYPE_P,
	HEVC_I = V4L2_HEVC_SLICE_TYPE_I,
};

/* The per-slice parameter block (the kext's P[]), V4L2 names. */
struct hevc_slice {
	bool dependent;
	u32 type;		/* B 0, P 1, I 2 */
	u32 address;
	bool sao_luma, sao_chroma, temporal_mvp, mvd_l1_zero, cabac_init;
	bool col_from_l0, dbk_disabled, lf_across_slices;
	u32 l0, l1;		/* num_ref_idx_lX_active_minus1 */
	u32 col_ref_idx;
	u32 merge_cand;		/* MaxNumMergeCand */
	int qp;			/* SliceQpY */
	int cb_qp, cr_qp, beta, tc;
	u8 slot[2][15];		/* hardware slot of RefPicListX[i], else 0 */
	bool weighted;
};

/* Picture-level state carried across the slices of one picture. */
struct hevc_pic {
	const struct apple_vxd_hevc_frame *f;
	u32 ctb_mb;		/* CtbSizeY / 16 */
	u32 w_ctb, h_ctb;	/* PicWidth/HeightInCtbsY */
	u32 w, h;		/* luma size rounded to the minimum CB */
	u32 qp_bd;		/* QpBdOffsetY */
	u32 tile_cols[12], tile_rows[12];
	u32 seg_x, seg_y;	/* last independent segment, in MBs */
	u32 min_type;		/* minimum slice type so far (C[13c470]) */
	int min_type_q;		/* over independent segments, -1 none */
	u32 cache_div;		/* Q+0x84 */
	u32 side_used, transfers, dest;
	u32 prev_dest, prev_lines;
	int col_slot;		/* -1 none */
	bool col_invalid;
	size_t last_desc;	/* word index of the last slice descriptor */
};

#define SPS_F(sps, n)	(!!((sps)->flags & V4L2_HEVC_SPS_FLAG_##n))
#define PPS_F(pps, n)	(!!((pps)->flags & V4L2_HEVC_PPS_FLAG_##n))
#define SLICE_F(s, n)	(!!((s)->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_##n))

u32 apple_vxd_hevc_genc_size(unsigned int width, unsigned int height)
{
	/* (W/16) * 1152.0 * (H/16) * 0.25, page aligned, capped (0x62b18f8) */
	u32 v = DIV_ROUND_UP(width, 16) * 288 * DIV_ROUND_UP(height, 16);

	return min_t(u32, ALIGN(v, 0x1000), 0x6000000);
}

static void hevc_aux_sizes(u32 w, u32 h, struct apple_vxd_hevc_aux_layout *l)
{
	/* CHEVCDecoder::AllocatePictureMemory (0x62b87dc) */
	u32 mv = (DIV_ROUND_UP(w, 64) * 0x80 + DIV_ROUND_UP(w, 128) * 0x40) *
		 DIV_ROUND_UP(h, 64);
	u32 frag = DIV_ROUND_UP(h, 64) * 0x600 + 0x3fc0;
	u32 above = DIV_ROUND_UP(w, 64) * 0xc0;

	l->mv = 0;
	l->frag = ALIGN(mv, 64);
	l->above = ALIGN(l->frag + frag, 64);
	l->record = ALIGN(l->above + above, 64);
	l->size = l->record + ALIGN(sizeof(struct apple_vxd_hevc_record), 64);
}

static void hevc_coded_size(const struct v4l2_ctrl_hevc_sps *sps, u32 *w, u32 *h)
{
	u32 mcb = sps->log2_min_luma_coding_block_size_minus3 + 3;

	*w = (sps->pic_width_in_luma_samples >> mcb) << mcb;
	*h = (sps->pic_height_in_luma_samples >> mcb) << mcb;
}

void apple_vxd_hevc_aux_layout(const struct v4l2_ctrl_hevc_sps *sps,
			       struct apple_vxd_hevc_aux_layout *l)
{
	u32 w, h;

	hevc_coded_size(sps, &w, &h);
	hevc_aux_sizes(w, h, l);
}

/* An upper bound for any stream that fits a @width x @height CAPTURE. */
u32 apple_vxd_hevc_aux_size(unsigned int width, unsigned int height)
{
	struct apple_vxd_hevc_aux_layout l;

	hevc_aux_sizes(width, height, &l);
	return l.size;
}

int apple_vxd_hevc_validate(const struct v4l2_ctrl_hevc_sps *sps,
			    const struct v4l2_ctrl_hevc_pps *pps,
			    const char **why)
{
	u32 ctb_log2 = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		       sps->log2_diff_max_min_luma_coding_block_size;

	if (sps->chroma_format_idc != 1)
		*why = "chroma format other than 4:2:0";
	else if (sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8)
		*why = "bit depth above 8 (Main 10 output layout not established)";
	else if (SPS_F(sps, SEPARATE_COLOUR_PLANE))
		*why = "separate colour planes";
	else if (ctb_log2 < 4 || ctb_log2 > 6 ||
		 sps->log2_min_luma_coding_block_size_minus3 > 3 ||
		 sps->log2_min_luma_transform_block_size_minus2 > 3 ||
		 sps->log2_diff_max_min_luma_transform_block_size > 3 ||
		 sps->max_transform_hierarchy_depth_inter > 7 ||
		 sps->max_transform_hierarchy_depth_intra > 7)
		*why = "coding or transform block sizes outside the hardware fields";
	else if (!sps->pic_width_in_luma_samples || !sps->pic_height_in_luma_samples ||
		 sps->pic_width_in_luma_samples > 4096 ||
		 sps->pic_height_in_luma_samples > 4096)
		*why = "picture size outside the hardware fields";
	else if (PPS_F(pps, TILES_ENABLED) &&
		 (pps->num_tile_columns_minus1 > 19 || pps->num_tile_rows_minus1 > 21))
		*why = "tile counts out of range";
	else if (pps->diff_cu_qp_delta_depth > 3 || pps->log2_parallel_merge_level_minus2 > 7)
		*why = "PPS depth fields out of range";
	else
		return 0;
	return -EINVAL;
}

/* --- locating the slices --------------------------------------------- */

static bool hevc_is_sc(const u8 *p, size_t i, size_t len)
{
	return i + 3 <= len && !p[i] && !p[i + 1] && p[i + 2] == 1;
}

/*
 * Frame-based Annex-B: the OUTPUT buffer holds the picture's slice NALs,
 * each behind a start code, in slice order.  The i-th slice NAL is the
 * i-th VCL NAL found.  data_byte_offset counts RBSP header bytes, and
 * clients disagree on its base: FFmpeg and libva-v4l2-request count from
 * the buffer start, GStreamer from the slice's start code.  Either way the
 * header ends inside this NAL, which tells them apart.
 */
int apple_vxd_hevc_find_nals(const u8 *src, size_t len,
			     const struct v4l2_ctrl_hevc_slice_params *slices,
			     unsigned int nslices, struct apple_vxd_hevc_nal *nals,
			     const char **why)
{
	size_t i = 0, sc, start, end;
	unsigned int n = 0;

	while (n < nslices) {
		u32 hdr;

		while (i < len && !hevc_is_sc(src, i, len))
			i++;
		if (i >= len) {
			*why = "fewer slice NALs in the OUTPUT buffer than SLICE_PARAMS";
			return -EINVAL;
		}
		sc = i;
		start = i + 3;
		for (end = start; end < len && !hevc_is_sc(src, end, len); end++)
			;
		i = end;
		while (end > start && !src[end - 1])
			end--;
		if (end - start < 3 || ((src[start] >> 1) & 0x3f) > 31)
			continue;	/* not a VCL NAL */

		if (slices[n].data_byte_offset > start &&
		    slices[n].data_byte_offset < end) {
			hdr = slices[n].data_byte_offset - start;
		} else if (slices[n].data_byte_offset >= start - sc + 2 &&
			   slices[n].data_byte_offset < end - sc) {
			hdr = slices[n].data_byte_offset - (start - sc);
		} else {
			*why = "data_byte_offset outside its slice NAL";
			return -EINVAL;
		}
		if (hdr < 3) {
			*why = "slice header shorter than the NAL header";
			return -EINVAL;
		}
		nals[n].offset = start;
		nals[n].size = end - start;
		nals[n].header_bits = 8 * hdr;
		n++;
	}
	return 0;
}

/* --- per-picture words (0x62b2bb8 / 0x62b38d0 / 0x62b7a18) ------------ */

static u32 hevc_sps0(const struct v4l2_ctrl_hevc_sps *sps)
{
	return sps->max_transform_hierarchy_depth_intra << 28 |
	       sps->max_transform_hierarchy_depth_inter << 24 |
	       (sps->log2_diff_max_min_luma_transform_block_size & 3) << 20 |
	       (sps->log2_min_luma_transform_block_size_minus2 & 3) << 16 |
	       (sps->log2_diff_max_min_luma_coding_block_size & 3) << 12 |
	       (sps->log2_min_luma_coding_block_size_minus3 & 3) << 8 |
	       (sps->bit_depth_chroma_minus8 & 7) << 4 |
	       (sps->bit_depth_luma_minus8 & 7);
}

/* PCM fields are zero without PCM, which makes the depths read 1. */
static u32 hevc_sps1(const struct v4l2_ctrl_hevc_sps *sps)
{
	bool pcm = SPS_F(sps, PCM_ENABLED);
	u32 v;

	v = ((pcm ? sps->pcm_sample_bit_depth_chroma_minus1 & 15 : 0) + 1) << 12 |
	    ((pcm ? sps->pcm_sample_bit_depth_luma_minus1 & 15 : 0) + 1) << 8;
	if (pcm)
		v |= (sps->log2_diff_max_min_pcm_luma_coding_block_size & 3) << 6 |
		     (sps->log2_min_pcm_luma_coding_block_size_minus3 & 3) << 4 |
		     SPS_F(sps, PCM_LOOP_FILTER_DISABLED) << 2;
	return v | pcm << 1 | SPS_F(sps, AMP_ENABLED);
}

static u32 hevc_tiles(const struct v4l2_ctrl_hevc_pps *pps)
{
	bool tiles = PPS_F(pps, TILES_ENABLED);

	return (tiles ? pps->num_tile_rows_minus1 : 0) << 24 |
	       (tiles ? pps->num_tile_columns_minus1 : 0) << 16 |
	       PPS_F(pps, ENTROPY_CODING_SYNC_ENABLED) << 10 |
	       tiles << 9 | PPS_F(pps, TRANSQUANT_BYPASS_ENABLED) << 8;
}

static u32 hevc_pic_config(const struct hevc_pic *p)
{
	const struct v4l2_ctrl_hevc_sps *sps = p->f->sps;

	return (p->min_type == HEVC_I) << 30 |
	       (sps->log2_min_luma_coding_block_size_minus3 +
		sps->log2_diff_max_min_luma_coding_block_size) << 28 |
	       0x1c0000 | (sps->bit_depth_chroma_minus8 & 7) << 8 |
	       (sps->bit_depth_luma_minus8 & 7) << 4 | sps->chroma_format_idc;
}

/*
 * Tile boundaries in MBs, two 16-bit halves per word: cumulative explicit
 * sizes, or the kext's float32 accumulation of PicSizeInCtbs / (n + 1),
 * truncated per step.  The integer form below is the H.265 6.5.1 one, which
 * the float form equals whenever it is exact (shift by CtbLog2 - 4).
 */
static void hevc_tile_bounds(const struct v4l2_ctrl_hevc_pps *pps, u32 shift,
			     u32 w_ctb, u32 h_ctb, u32 *cols, u32 *rows)
{
	unsigned int k, n;
	u32 acc;

	memset(cols, 0, 12 * sizeof(*cols));
	memset(rows, 0, 12 * sizeof(*rows));
	if (!PPS_F(pps, TILES_ENABLED))
		return;

	n = pps->num_tile_columns_minus1;
	for (k = 0, acc = 0; k < n; k++) {
		if (PPS_F(pps, UNIFORM_SPACING))
			acc = (k + 1) * w_ctb / (n + 1);
		else
			acc += pps->column_width_minus1[k] + 1;
		cols[k / 2] |= (acc << shift) << (16 * (k & 1));
	}
	n = pps->num_tile_rows_minus1;
	for (k = 0, acc = 0; k < n; k++) {
		if (PPS_F(pps, UNIFORM_SPACING))
			acc = (k + 1) * h_ctb / (n + 1);
		else
			acc += pps->row_height_minus1[k] + 1;
		rows[k / 2] |= (acc << shift) << (16 * (k & 1));
	}
}

/* --- per slice -------------------------------------------------------- */

static int hevc_slice_params(const struct apple_vxd_hevc_frame *f,
			     unsigned int idx, struct hevc_slice *s,
			     const char **why)
{
	const struct v4l2_ctrl_hevc_slice_params *sp = &f->slices[idx];
	const struct v4l2_ctrl_hevc_pps *pps = f->pps;
	unsigned int i, lst;

	memset(s, 0, sizeof(*s));
	s->address = sp->slice_segment_addr;
	s->dependent = SLICE_F(sp, DEPENDENT_SLICE_SEGMENT);
	if (s->dependent) {
		/*
		 * The kext parses only the address of a dependent segment and
		 * builds its words from a zeroed record (slice type B, no
		 * references, MaxNumMergeCand 5).  The core takes the rest
		 * from the independent segment.
		 */
		s->merge_cand = 5;
		s->qp = 26 + pps->init_qp_minus26;
		return 0;
	}

	s->type = sp->slice_type;
	if (s->type > HEVC_I) {
		*why = "invalid slice type";
		return -EINVAL;
	}
	s->sao_luma = SLICE_F(sp, SLICE_SAO_LUMA);
	s->sao_chroma = SLICE_F(sp, SLICE_SAO_CHROMA);
	s->temporal_mvp = SLICE_F(sp, SLICE_TEMPORAL_MVP_ENABLED);
	s->mvd_l1_zero = SLICE_F(sp, MVD_L1_ZERO);
	s->cabac_init = SLICE_F(sp, CABAC_INIT);
	/* Only B slices code it; the kext's default is 1. */
	s->col_from_l0 = s->type != HEVC_B || SLICE_F(sp, COLLOCATED_FROM_L0);
	s->dbk_disabled = SLICE_F(sp, SLICE_DEBLOCKING_FILTER_DISABLED);
	s->lf_across_slices = SLICE_F(sp, SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED);
	/* I slices carry the PPS defaults, P slices the PPS L1 default. */
	s->l0 = s->type == HEVC_I ? pps->num_ref_idx_l0_default_active_minus1 :
				    sp->num_ref_idx_l0_active_minus1;
	s->l1 = s->type == HEVC_B ? sp->num_ref_idx_l1_active_minus1 :
				    pps->num_ref_idx_l1_default_active_minus1;
	if (s->l0 > 14 || s->l1 > 14) {
		*why = "more than 15 active references";
		return -EINVAL;
	}
	s->col_ref_idx = s->temporal_mvp ? sp->collocated_ref_idx : 0;
	s->merge_cand = 5 - (s->type == HEVC_I ? 0 : sp->five_minus_max_num_merge_cand);
	s->qp = 26 + pps->init_qp_minus26 + sp->slice_qp_delta;
	s->cb_qp = sp->slice_cb_qp_offset;
	s->cr_qp = sp->slice_cr_qp_offset;
	s->beta = sp->slice_beta_offset_div2;
	s->tc = sp->slice_tc_offset_div2;
	s->weighted = (PPS_F(pps, WEIGHTED_PRED) && s->type == HEVC_P) ||
		      (PPS_F(pps, WEIGHTED_BIPRED) && s->type == HEVC_B);

	for (lst = 0; s->type != HEVC_I && lst < (s->type == HEVC_B ? 2 : 1); lst++) {
		const u8 *ref = lst ? sp->ref_idx_l1 : sp->ref_idx_l0;
		u32 n = lst ? s->l1 : s->l0;

		for (i = 0; i <= n; i++) {
			if (ref[i] >= f->dp->num_active_dpb_entries) {
				*why = "a reference index names no DPB entry";
				return -EINVAL;
			}
			s->slot[lst][i] = ref[i];
		}
	}
	if (s->type != HEVC_I && s->col_ref_idx > (s->col_from_l0 ? s->l0 : s->l1)) {
		*why = "collocated_ref_idx out of range";
		return -EINVAL;
	}
	return 0;
}

/* The slice structure and its 0xf transfer (0x62b6010). */
static void hevc_slice_struct(struct apple_vxd_cmdbuf *cb, struct hevc_pic *p,
			      const struct hevc_slice *s,
			      const struct v4l2_ctrl_hevc_slice_params *sp,
			      size_t w3090, size_t w3094)
{
	const struct apple_vxd_hevc_frame *f = p->f;
	const struct v4l2_ctrl_hevc_sps *sps = f->sps;
	const struct v4l2_ctrl_hevc_pps *pps = f->pps;
	const struct v4l2_hevc_pred_weight_table *pw = &sp->pred_weight_table;
	u32 hdr = s->type == HEVC_B ? 48 : s->type == HEVC_P ? 32 : 16;
	u32 *w = (u32 *)(f->side_cpu + p->side_used);
	u32 *pairs = w + hdr / 4;
	u32 npairs = 0, pad, unpadded, transfer, lines, c320, c321;
	unsigned int i, lst;

	memset(w, 0, hdr);
#define PAIR(r, v)						\
	do {								\
		pairs[2 * npairs] = VXD_CMD_REG1 | (r);			\
		pairs[2 * npairs + 1] = (v);				\
		npairs++;						\
	} while (0)
	if (s->weighted) {
		u32 cden = pw->luma_log2_weight_denom + pw->delta_chroma_log2_weight_denom;

		PAIR(HEVC_REN_WEIGHT_DENOM, pw->luma_log2_weight_denom | cden << 4);
		for (lst = 0; lst < (s->type == HEVC_B ? 2 : 1); lst++) {
			const s8 *dlw = lst ? pw->delta_luma_weight_l1 : pw->delta_luma_weight_l0;
			const s8 *lo = lst ? pw->luma_offset_l1 : pw->luma_offset_l0;
			const s8 (*dcw)[2] = lst ? pw->delta_chroma_weight_l1 :
						   pw->delta_chroma_weight_l0;
			const s8 (*co)[2] = lst ? pw->chroma_offset_l1 : pw->chroma_offset_l0;

			for (i = 0; i <= (lst ? s->l1 : s->l0); i++) {
				PAIR(HEVC_REN_WEIGHTS(lst) + 4 * i,
				     ((u32)dlw[i] & 0x1ff) | ((u32)dcw[i][0] & 0x1ff) << 9 |
				     ((u32)dcw[i][1] & 0x1ff) << 18);
				PAIR(HEVC_REN_OFFSETS(lst) + 4 * i,
				     (u8)lo[i] | (u32)(u8)co[i][0] << 8 |
				     (u32)(u8)co[i][1] << 16);
			}
		}
	}
	if (f->caps->tail_7474) {
		PAIR(HEVC_REN_7474, 0);
		PAIR(HEVC_REN_7474, 0);
	}
#undef PAIR

	c320 = PPS_F(pps, CONSTRAINED_INTRA_PRED) << 7 |
	       SPS_F(sps, STRONG_INTRA_SMOOTHING_ENABLED) << 6 |
	       SPS_F(sps, SAMPLE_ADAPTIVE_OFFSET) << 5 | s->sao_chroma << 3 |
	       s->sao_luma << 2 |
	       (PPS_F(pps, WEIGHTED_BIPRED) && s->type == HEVC_B) << 1 |
	       (PPS_F(pps, WEIGHTED_PRED) && s->type == HEVC_P);
	c321 = PPS_F(pps, LOOP_FILTER_ACROSS_TILES_ENABLED) << 2 |
	       s->lf_across_slices << 1 | s->dbk_disabled;

	w[0] = s->type | s->sao_luma << 2 | s->sao_chroma << 3 | s->temporal_mvp << 4 |
	       s->l0 << 8 | s->l1 << 12 | s->mvd_l1_zero << 16 | s->col_from_l0 << 18 |
	       s->merge_cand << 20;
	w[1] = s->lf_across_slices << 24 | ((p->qp_bd + s->qp) & 0x7f) |
	       ((u32)s->cb_qp & 31) << 8 | ((u32)s->cr_qp & 31) << 16;
	w[2] = p->seg_y << 10 | p->seg_x | npairs << 20;
	w[3] = c320 << 16 | c321 << 12 | ((u32)s->tc & 15) << 8 |
	       ((u32)s->beta & 15) << 4 | 8 | (2 - s->type);
	for (lst = 0; s->type != HEVC_I && lst < (s->type == HEVC_B ? 2 : 1); lst++)
		for (i = 0; i < 15; i++)
			w[4 + 4 * lst + i / 4] |= (u32)s->slot[lst][i] << (8 * (i % 4));

	pad = (npairs * 0x38 - hdr) & 0x38;
	unpadded = hdr + 8 * npairs;
	memset((u8 *)w + unpadded, 0, pad);
	transfer = unpadded + pad;
	lines = unpadded - 1 > 0x3bf ? 15 : (unpadded - 1) >> 6;

	/* Q+0x7c (minimum independent slice type) and Q+0x84 (cache hint). */
	if (!s->address || p->min_type_q < 0 || (int)s->type < p->min_type_q) {
		u32 d = 0;

		if (s->type == HEVC_P && pps->num_ref_idx_l0_default_active_minus1)
			d = abs((int)s->slot[0][0] - (int)s->slot[0][1]);
		else if (s->type == HEVC_B)
			d = abs((int)s->slot[0][0] - (int)s->slot[1][0]);
		p->cache_div = max(p->cache_div, d);
		p->min_type_q = s->type;
	}

	if (!p->transfers)
		p->dest = f->spdest;
	apple_vxd_cmd_table_xfer(cb, transfer, f->side + p->side_used, p->dest);
	cb->vaddr[w3090] = p->dest;
	cb->vaddr[w3094] = lines;
	p->prev_dest = p->dest;
	p->prev_lines = lines;
	p->dest += transfer;
	p->transfers++;
	p->side_used += transfer;

	/* The co-located picture: the first P/B slice decides (L28666). */
	if (SPS_F(sps, SPS_TEMPORAL_MVP_ENABLED) && p->col_slot < 0 &&
	    !p->col_invalid && s->type != HEVC_I) {
		u8 slot = s->slot[s->col_from_l0 ? 0 : 1][s->col_ref_idx];

		if (f->refs[slot].rec && f->refs[slot].rec->valid)
			p->col_slot = slot;
		else
			p->col_invalid = true;
	}
}

static int hevc_emit_slice(struct apple_vxd_cmdbuf *cb, struct hevc_pic *p,
			   unsigned int idx, const char **why)
{
	const struct apple_vxd_hevc_frame *f = p->f;
	const struct v4l2_ctrl_hevc_sps *sps = f->sps;
	const struct v4l2_ctrl_hevc_pps *pps = f->pps;
	const struct v4l2_ctrl_hevc_slice_params *sp = &f->slices[idx];
	const struct apple_vxd_hevc_nal *nal = &f->nals[idx];
	u32 hdr[VXD_CMD_SLICE_BLOCK_WORDS - 1] = { };
	u32 row, col_mb, row_mb, w3410, w3414;
	size_t w3090, w3094;
	struct hevc_slice s;
	unsigned int k;
	int ret;

	ret = hevc_slice_params(f, idx, &s, why);
	if (ret)
		return ret;
	if (!s.dependent && p->side_used + HEVC_STRUCT_MAX > f->side_size) {
		*why = "slice structure buffer full";
		return -ENOSPC;
	}

	p->min_type = idx ? min(p->min_type, s.type) : s.type;

	/* Segment position in MBs (0x62b55f0). */
	row = p->w_ctb ? s.address / p->w_ctb : 0;
	col_mb = (s.address - row * p->w_ctb) * p->ctb_mb;
	row_mb = row * p->ctb_mb;
	if (!s.dependent) {
		p->seg_x = col_mb;
		p->seg_y = row_mb;
	}
	w3410 = s.dependent << 31 | (p->seg_y & 0xfff) << 16 | (p->seg_x & 0xfff);
	w3414 = ((p->qp_bd + s.qp) & 0x7f) << 24 | s.merge_cand << 20 |
		s.cabac_init << 17 | s.mvd_l1_zero << 16 | s.l1 << 12 | s.l0 << 8 |
		s.sao_chroma << 3 | s.sao_luma << 2 | s.type;

	p->last_desc = cb->used / 4;
	hdr[3] = row_mb << 16 | col_mb;
	hdr[4] = (DIV_ROUND_UP(p->w, 16) - 1) | (DIV_ROUND_UP(p->h, 16) - 1) << 16;
	apple_vxd_cmd_ctrl_alloc(cb, hdr);

	apple_vxd_cmd_slice_desc(cb, nal->header_bits, nal->size);
	apple_vxd_cmd_load(cb, f->src + nal->offset, nal->size);

	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, HEVC_CMD_FRAGMENT, f->frag & ~63U);
	apple_vxd_cmd_reg(cb, HEVC_CMD_FORMAT, sps->chroma_format_idc << 16 | 0x10c);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SIZE, (p->h - 1) << 16 | (p->w - 1));
	apple_vxd_cmd_group_close(cb);
	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SLICE_POS, row_mb << 16 | col_mb);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SEGMENT, w3410);
	apple_vxd_cmd_group_close(cb);
	if (f->first_slice_of_session && !idx) {
		apple_vxd_cmd_group_open(cb, 0);
		for (k = 0; k < 4; k++)
			apple_vxd_cmd_reg(cb, HEVC_CMD_RESET_ZERO + 4 * k, 0);
		apple_vxd_cmd_reg(cb, HEVC_CMD_RESET, 1);
		apple_vxd_cmd_group_close(cb);
	}
	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SPS0, hevc_sps0(sps));
	apple_vxd_cmd_reg(cb, HEVC_CMD_SPS1, hevc_sps1(sps));
	apple_vxd_cmd_reg(cb, HEVC_CMD_PPS0, hevc_tiles(pps) |
			  (pps->diff_cu_qp_delta_depth & 3) << 4 |
			  PPS_F(pps, CU_QP_DELTA_ENABLED) << 3 |
			  PPS_F(pps, TRANSFORM_SKIP_ENABLED) << 2 |
			  PPS_F(pps, SIGN_DATA_HIDING_ENABLED));
	apple_vxd_cmd_group_close(cb);
	apple_vxd_cmd_group_open(cb, 0);
	for (k = 0; k < 12; k++)
		apple_vxd_cmd_reg(cb, HEVC_CMD_TILE_COLS + 4 * k, p->tile_cols[k]);
	for (k = 0; k < 12; k++)
		apple_vxd_cmd_reg(cb, HEVC_CMD_TILE_ROWS + 4 * k, p->tile_rows[k]);
	apple_vxd_cmd_group_close(cb);
	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SLICE0, w3414);
	apple_vxd_cmd_group_close(cb);
	apple_vxd_cmd_group_open(cb, 0);
	for (k = 0; k < APPLE_VXD_HEVC_GENC_BUFS; k++) {
		apple_vxd_cmd_reg(cb, HEVC_CMD_GENC_SIZE + 4 * k, f->genc_size & ~0xfffU);
		apple_vxd_cmd_reg(cb, HEVC_CMD_GENC_BASE + 4 * k, f->genc[k] & ~0xfffU);
	}
	apple_vxd_cmd_group_close(cb);

	/* 0x3090/0x3094 first, filled in once the structure exists. */
	apple_vxd_cmd_group_open(cb, 0);
	apple_vxd_cmd_reg(cb, HEVC_CMD_SP_DEST, 0);
	w3090 = cb->used / 4 - 1;
	apple_vxd_cmd_reg(cb, HEVC_CMD_SP_LINES, 0);
	w3094 = cb->used / 4 - 1;
	apple_vxd_cmd_group_close(cb);
	if (cb->error)
		return -ENOSPC;
	if (s.dependent) {
		cb->vaddr[w3090] = p->prev_dest;
		cb->vaddr[w3094] = p->prev_lines;
	} else {
		hevc_slice_struct(cb, p, &s, sp, w3090, w3094);
	}
	apple_vxd_cmd_end_slice(cb);
	return cb->error ? -ENOSPC : 0;
}

/* --- per picture: the kind-4 chunk (0x62b7ab0, 0x62b859c) -------------- */

/*
 * Scaling lists: the 8x8 base lists one byte per coefficient, four per
 * register; register i takes coefficients 4i..4i+3 of a de-interleaved
 * order in which the even index bits give the row and the odd bits the
 * column (the kext's gather tables at 0xfffffff005595040/0x5595050).
 */
static u32 hevc_scl_word(const u8 *raster, unsigned int n, unsigned int i)
{
	unsigned int k, l, row, col, side = n == 16 ? 8 : 4;
	u32 v = 0;

	for (k = 0; k < 4; k++) {
		l = 4 * i + k;
		row = (l & 1) | (l >> 1 & 2) | (l >> 2 & 4);
		col = (l >> 1 & 1) | (l >> 2 & 2) | (l >> 3 & 4);
		if (side == 4) {
			row &= 3;
			col &= 3;
		}
		v |= (u32)raster[row * side + col] << (8 * k);
	}
	return v;
}

static void hevc_scl_list(struct apple_vxd_cmdbuf *cb, u32 reg, const u8 *raster,
			  unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		apple_vxd_cmd_reg1(cb, reg + 4 * i, hevc_scl_word(raster, n, i), 0);
}

static void hevc_emit_scaling(struct apple_vxd_cmdbuf *cb,
			      const struct v4l2_ctrl_hevc_scaling_matrix *m)
{
	unsigned int i;

	hevc_scl_list(cb, HEVC_ENT_SCALING + 0x000, m->scaling_list_32x32[0], 16);
	hevc_scl_list(cb, HEVC_ENT_SCALING + 0x040, m->scaling_list_32x32[1], 16);
	for (i = 0; i < 6; i++)
		hevc_scl_list(cb, HEVC_ENT_SCALING + 0x080 + 0x40 * i,
			      m->scaling_list_16x16[i], 16);
	for (i = 0; i < 6; i++)
		hevc_scl_list(cb, HEVC_ENT_SCALING + 0x200 + 0x40 * i,
			      m->scaling_list_8x8[i], 16);
	for (i = 0; i < 6; i++)
		hevc_scl_list(cb, HEVC_ENT_SCALING + 0x380 + 0x10 * i,
			      m->scaling_list_4x4[i], 4);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SCALING + 0x3e0, m->scaling_list_dc_coef_32x32[0], 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SCALING + 0x3e4, m->scaling_list_dc_coef_32x32[1], 0);
	for (i = 0; i < 6; i++)
		apple_vxd_cmd_reg1(cb, HEVC_ENT_SCALING + 0x3e8 + 4 * i,
				   m->scaling_list_dc_coef_16x16[i], 0);
}

/* 0x701c: Apple's line-buffer split for 4:2:0 (13d684 set at create). */
static u32 hevc_cache_policy(const struct hevc_pic *p)
{
	u32 t = (p->f->caps->cache_base * 0x15556) >> 17;

	t = (t * 15) >> 5;
	if (p->cache_div > 1)
		t /= p->cache_div;
	return 1 << 24 | ALIGN(t, 8) << 12 | 12;
}

static void hevc_emit_picture(struct apple_vxd_cmdbuf *cb, struct hevc_pic *p)
{
	const struct apple_vxd_hevc_frame *f = p->f;
	const struct v4l2_ctrl_hevc_sps *sps = f->sps;
	const struct v4l2_ctrl_hevc_pps *pps = f->pps;
	const struct v4l2_ctrl_hevc_decode_params *dp = f->dp;
	u32 min_type = p->min_type_q < 0 ? 0 : p->min_type_q;
	u32 lt = 0, delta[4] = { }, size8, w5400;
	unsigned int k;

	size8 = ((ALIGN(p->h, 8) - 1) << 16) | (ALIGN(p->w, 8) - 1);
	w5400 = hevc_sps0(sps) | SPS_F(sps, SCALING_LIST_ENABLED) << 31;

	apple_vxd_cmd_reg1(cb, HEVC_ENT_ABOVE, f->above & ~63U, HEVC_F_ADDR);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SIZE8, size8, 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SPS0, w5400, 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SPS0 + 4, hevc_sps1(sps) |
			   SPS_F(sps, SPS_TEMPORAL_MVP_ENABLED) << 16, 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SPS0 + 8, hevc_tiles(pps) |
			   PPS_F(pps, TRANSFORM_SKIP_ENABLED) << 2, 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_SPS0 + 12,
			   (pps->log2_parallel_merge_level_minus2 & 7) << 16 |
			   ((u32)pps->pps_cr_qp_offset & 31) << 8 |
			   ((u32)pps->pps_cb_qp_offset & 31), 0);
	for (k = 0; k < APPLE_VXD_HEVC_GENC_BUFS; k++) {
		apple_vxd_cmd_reg1(cb, HEVC_ENT_GENC_SIZE + 4 * k, f->genc_size & ~0xfffU, 0);
		apple_vxd_cmd_reg1(cb, HEVC_ENT_GENC_BASE + 4 * k, f->genc[k] & ~0xfffU,
				   HEVC_F_ADDR);
	}
	apple_vxd_cmd_reg1(cb, HEVC_ENT_FRAGMENT, f->frag & ~63U, HEVC_F_ADDR);
	for (k = 0; k < 12; k++)
		apple_vxd_cmd_reg1(cb, HEVC_ENT_TILE_COLS + 4 * k, p->tile_cols[k], 0);
	for (k = 0; k < 12; k++)
		apple_vxd_cmd_reg1(cb, HEVC_ENT_TILE_ROWS + 4 * k, p->tile_rows[k], 0);

	for (k = 0; k < dp->num_active_dpb_entries; k++) {
		const struct v4l2_hevc_dpb_entry *e = &dp->dpb[k];
		s32 d = clamp_t(s32, dp->pic_order_cnt_val - e->pic_order_cnt_val, -128, 127);

		if (e->flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE)
			lt |= BIT(k);
		delta[k / 4] |= (u32)(u8)d << (8 * (k % 4));
	}
	f->record->refs = lt | (2 - min_type) << 16;
	memcpy(f->record->poc_delta, delta, sizeof(delta));
	f->record->mv = f->mv;
	f->record->valid = 1;
	apple_vxd_cmd_reg1(cb, HEVC_ENT_REFS, f->record->refs, 0);
	for (k = 0; k < 4; k++)
		apple_vxd_cmd_reg1(cb, HEVC_ENT_POC_DELTA + 4 * k, delta[k], 0);
	apple_vxd_cmd_reg1(cb, HEVC_ENT_CUR_MV, f->mv, HEVC_F_ADDR);
	if (p->col_slot < 0) {
		apple_vxd_cmd_reg1(cb, HEVC_ENT_COL_REFS, 0, 0);
	} else {
		const struct apple_vxd_hevc_record *r = f->refs[p->col_slot].rec;

		apple_vxd_cmd_reg1(cb, HEVC_ENT_COL_REFS, r->refs, 0);
		for (k = 0; k < 4; k++)
			apple_vxd_cmd_reg1(cb, HEVC_ENT_COL_POC_DELTA + 4 * k,
					   r->poc_delta[k], 0);
		apple_vxd_cmd_reg1(cb, HEVC_ENT_COL_MV, r->mv, HEVC_F_ADDR);
	}
	apple_vxd_cmd_reg1(cb, HEVC_ENT_CONTROL,
			   sps->chroma_format_idc << 28 | 0x10c0000, 0);
	if (SPS_F(sps, SCALING_LIST_ENABLED))
		hevc_emit_scaling(cb, f->sm);

	apple_vxd_cmd_reg1(cb, HEVC_REN_PIC_CONFIG, hevc_pic_config(p), 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_CHROMA_QP, ((u32)pps->pps_cb_qp_offset & 31) |
			   ((u32)pps->pps_cr_qp_offset & 31) << 8, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_SIZE, (p->h - 1) << 16 | (p->w - 1), 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_SIZE8, size8, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_AUXLINE, f->auxline, HEVC_F_ADDR);
	apple_vxd_cmd_reg1(cb, HEVC_REN_CACHE, hevc_cache_policy(p), 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_STRIDE, f->dst_stride & ~63U, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_FORMAT, (sps->bit_depth_chroma_minus8 & 7) << 12 |
			   (sps->bit_depth_luma_minus8 & 7) << 8 | sps->chroma_format_idc, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_LAYOUT, HEVC_LAYOUT_LINEAR, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_CSTRIDE, f->dst_cstride | f->dst_cstride << 16, 0);
	apple_vxd_cmd_reg1(cb, HEVC_REN_DST, f->dst_luma, HEVC_F_ADDR);
	apple_vxd_cmd_reg1(cb, HEVC_REN_DST + 4, f->dst_chroma, HEVC_F_ADDR);
	if (min_type < HEVC_I) {
		for (k = 0; k < dp->num_active_dpb_entries; k++) {
			apple_vxd_cmd_reg1(cb, HEVC_REN_REFS + 8 * k, f->refs[k].luma,
					   HEVC_F_ADDR);
			apple_vxd_cmd_reg1(cb, HEVC_REN_REFS + 8 * k + 4, f->refs[k].chroma,
					   HEVC_F_ADDR);
		}
	}
	apple_vxd_cmd_reg1(cb, HEVC_REN_INTRA, f->intra, HEVC_F_ADDR);

	/* The last slice descriptor learns the strides and the layout. */
	cb->vaddr[p->last_desc + 3] = f->dst_cstride | f->dst_cstride << 16;
	cb->vaddr[p->last_desc + 6] = 0;
	cb->vaddr[p->last_desc + 7] = 0;
	cb->vaddr[p->last_desc + 8] = HEVC_LAYOUT_LINEAR;
}

/*
 * One picture: the slice chunks with their kind-1 packets, then the kind-4
 * chunk and packet.  Returns 0, or -EINVAL / -ENOSPC with @why set.
 */
int apple_vxd_hevc_build(struct apple_vxd_cmdbuf *cb,
			 const struct apple_vxd_hevc_frame *f,
			 const char **why)
{
	const struct v4l2_ctrl_hevc_sps *sps = f->sps;
	struct hevc_pic p = { .f = f, .col_slot = -1, .min_type_q = -1 };
	u32 ctb_log2, mcb, off, bytes, *pkt;
	unsigned int i, k;
	int ret;

	*why = NULL;
	ret = apple_vxd_hevc_validate(sps, f->pps, why);
	if (ret)
		return ret;
	if (!f->nslices || f->nslices > APPLE_VXD_HEVC_MAX_SLICES) {
		*why = "no slices, or more than a job holds";
		return -EINVAL;
	}
	if (f->dp->num_active_dpb_entries > APPLE_VXD_HEVC_MAX_REFS) {
		*why = "more than 15 reference pictures";
		return -EINVAL;
	}
	for (k = 0; k < f->dp->num_active_dpb_entries; k++) {
		if (!f->refs[k].luma) {
			*why = "a DPB entry has no CAPTURE buffer";
			return -EINVAL;
		}
	}
	if (SPS_F(sps, SCALING_LIST_ENABLED) && (!f->sm || !f->sm->scaling_list_4x4[0][0])) {
		*why = "scaling lists enabled but the SCALING_MATRIX control is not set";
		return -EINVAL;
	}

	mcb = sps->log2_min_luma_coding_block_size_minus3 + 3;
	ctb_log2 = mcb + sps->log2_diff_max_min_luma_coding_block_size;
	hevc_coded_size(sps, &p.w, &p.h);
	p.ctb_mb = 1 << (ctb_log2 - 4);
	p.w_ctb = DIV_ROUND_UP(p.w, 1 << ctb_log2);
	p.h_ctb = DIV_ROUND_UP(p.h, 1 << ctb_log2);
	p.qp_bd = 6 * sps->bit_depth_luma_minus8;
	hevc_tile_bounds(f->pps, ctb_log2 - 4, p.w_ctb, p.h_ctb, p.tile_cols, p.tile_rows);

	for (i = 0; i < f->nslices; i++) {
		if (!i && SLICE_F(&f->slices[0], DEPENDENT_SLICE_SEGMENT)) {
			*why = "picture starts with a dependent slice segment";
			return -EINVAL;
		}
		ret = hevc_emit_slice(cb, &p, i, why);
		if (ret) {
			if (!*why)
				*why = "command buffer full";
			return ret;
		}
		ret = apple_vxd_cmdbuf_chunk_end(cb, &off, &bytes);
		if (ret) {
			*why = "command buffer full";
			return ret;
		}
		pkt = apple_vxd_cmdbuf_add_packet(cb, VXD_PKT_MAGIC0, VXD_PKT_H264_PAYLOAD);
		if (!pkt) {
			*why = "packet list full";
			return -ENOSPC;
		}
		pkt[VXD_PKT_FLAGS / 4] = (i ? 0 : 1) |
					 (!i && f->first_picture_of_session ? 0x10 : 0);
		pkt[VXD_PKT_W4 / 4] = hevc_pic_config(&p);
		pkt[VXD_PKT_CMD_BYTES / 4] = bytes;
		pkt[VXD_PKT_W2 / 4] = cb->va + off;
		pkt[VXD_PKT_W5 / 4] = f->stream_state;
		pkt[VXD_PKT_KIND / 4] = VXD_PKT_KIND_PICTURE;
	}

	hevc_emit_picture(cb, &p);
	ret = apple_vxd_cmdbuf_pair_chunk_end(cb, &off, &bytes);
	if (ret) {
		*why = "command buffer full";
		return ret;
	}
	pkt = apple_vxd_cmdbuf_add_packet(cb, VXD_PKT_MAGIC0, VXD_PKT_H264_PAYLOAD);
	if (!pkt) {
		*why = "packet list full";
		return -ENOSPC;
	}
	pkt[VXD_PKT_W4 / 4] = hevc_pic_config(&p);
	pkt[VXD_PKT_CMD_BYTES / 4] = bytes;
	pkt[VXD_PKT_W2 / 4] = cb->va + off;
	pkt[VXD_PKT_W5 / 4] = f->stream_state;
	pkt[VXD_PKT_KIND / 4] = VXD_PKT_KIND_RENDER_TARGET;
	return 0;
}
