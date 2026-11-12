// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - HEVC (H.265) backend (V4L2 stateless, frame based)
 *
 * Userspace parses the parameter sets, the slice headers and the DPB
 * (V4L2 stateless contract); the OUTPUT buffer carries the picture's slice
 * NALs behind Annex-B start codes, and SLICE_PARAMS one entry per slice.
 * Per picture this file
 *
 *   1. validates SPS/PPS against what the command recipe covers,
 *   2. locates the slice NALs (apple_vxd_hevc_find_nals()),
 *   3. resolves the DPB to CAPTURE buffers by timestamp, with their side
 *      buffers (temporal MVs and the co-located picture record),
 *   4. builds the command stream (apple-vxd-hevc-cmd.c) and submits one
 *      0x81 message per slice and the 0x88 message that ends the job.
 *
 * The context owns the buffers CHEVCDecoder::VACommandBufferInit allocates
 * per decoder (0xfffffff0062b18f8): four Genc buffers sized from the
 * picture, the intra and aux-line buffers, the stream state, and the slice
 * structure buffer.  Each CAPTURE buffer carries what AllocatePictureMemory
 * allocates per picture: temporal MVs, the fragment pointer table and the
 * above-parameter buffer.  The kext also gives every picture its own 1 MiB
 * slice parameter buffer; it is only read while that picture decodes, so
 * one per context serves here, decodes being serialized.
 */
#include <linux/slab.h>
#include <linux/sizes.h>

#include "apple-vxd.h"
#include "apple-vxd-hevc-cmd.h"

/* AppleD5500 BRN50577: S8000 A0/A1 cores need a dummy picture first. */
#define APPLE_VXD_CORE_REV_BRN50577	0x70600

struct apple_vxd_hevc_ctx {
	struct apple_vxd_dev_buf genc[APPLE_VXD_HEVC_GENC_BUFS];
	u32 genc_size;
	struct apple_vxd_dev_buf side;
	struct apple_vxd_dev_buf spdest;
	struct apple_vxd_dev_buf intra;
	struct apple_vxd_dev_buf auxline;
	struct apple_vxd_dev_buf stream_state;
	bool first_slice;
	bool first_picture;
	struct apple_vxd_hevc_nal nals[APPLE_VXD_HEVC_MAX_SLICES];
};

static const void *apple_vxd_hevc_ctrl(struct apple_vxd_ctx *ctx, u32 id,
				       unsigned int *elems)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (ctrl && elems)
		*elems = ctrl->elems;
	return ctrl ? ctrl->p_cur.p : NULL;
}

static int apple_vxd_hevc_try_ctrl(struct apple_vxd_ctx *ctx,
				   struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_hevc_sps *sps = ctrl->p_new.p_hevc_sps;

	if (ctrl->id != V4L2_CID_STATELESS_HEVC_SPS)
		return 0;
	/* The default (all zero, before userspace sets an SPS) must pass. */
	if (!sps->pic_width_in_luma_samples)
		return 0;
	if (sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 ||
	    sps->bit_depth_chroma_minus8) {
		dev_dbg(ctx->vxd->dev, "hevc sps: only 8-bit 4:2:0 is supported\n");
		return -EINVAL;
	}
	if (sps->pic_width_in_luma_samples > APPLE_VXD_MAX_PIC_W ||
	    sps->pic_height_in_luma_samples > APPLE_VXD_MAX_PIC_H) {
		dev_dbg(ctx->vxd->dev, "hevc sps: %ux%u too large\n",
			sps->pic_width_in_luma_samples, sps->pic_height_in_luma_samples);
		return -EINVAL;
	}
	return 0;
}

static void apple_vxd_hevc_free(struct apple_vxd_dev *vxd,
				struct apple_vxd_hevc_ctx *h)
{
	unsigned int i;

	for (i = 0; i < APPLE_VXD_HEVC_GENC_BUFS; i++)
		apple_vxd_dev_buf_free(vxd, &h->genc[i]);
	apple_vxd_dev_buf_free(vxd, &h->side);
	apple_vxd_dev_buf_free(vxd, &h->spdest);
	apple_vxd_dev_buf_free(vxd, &h->intra);
	apple_vxd_dev_buf_free(vxd, &h->auxline);
	apple_vxd_dev_buf_free(vxd, &h->stream_state);
}

static int apple_vxd_hevc_start(struct apple_vxd_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;
	struct apple_vxd_dev *vxd = ctx->vxd;
	const struct apple_vxd_hevc_caps *caps = &vxd->variant->hevc;
	struct apple_vxd_hevc_ctx *h;
	unsigned int i;
	int ret;

	h = kvzalloc_obj(*h);
	if (!h)
		return -ENOMEM;

	/* VACommandBufferInit sizes; M is the per-SoC buffer multiplier. */
	h->genc_size = apple_vxd_hevc_genc_size(pix->width, pix->height);
	for (i = 0; i < APPLE_VXD_HEVC_GENC_BUFS; i++) {
		ret = apple_vxd_dev_buf_alloc(vxd, h->genc_size, &h->genc[i]);
		if (ret)
			goto err;
	}
	ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_HEVC_SIDE_SIZE, &h->side);
	if (!ret)
		ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_HEVC_SPDEST_SIZE, &h->spdest);
	if (!ret)
		ret = apple_vxd_dev_buf_alloc(vxd, APPLE_VXD_HEVC_INTRA_SIZE, &h->intra);
	if (!ret)
		ret = apple_vxd_dev_buf_alloc(vxd, caps->buf_mult << 21, &h->auxline);
	if (!ret)
		ret = apple_vxd_dev_buf_alloc(vxd, caps->buf_mult << 12, &h->stream_state);
	if (ret)
		goto err;

	h->first_slice = true;
	h->first_picture = true;
	ctx->priv = h;
	dev_dbg(vxd->dev,
		"hevc: context buffers: genc 4 x %u bytes, aux line %u, stream state %u\n",
		h->genc_size, caps->buf_mult << 21, caps->buf_mult << 12);
	return 0;

err:
	apple_vxd_hevc_free(vxd, h);
	kvfree(h);
	return ret;
}

static void apple_vxd_hevc_stop(struct apple_vxd_ctx *ctx)
{
	struct apple_vxd_hevc_ctx *h = ctx->priv;

	if (!h)
		return;
	apple_vxd_hevc_free(ctx->vxd, h);
	kvfree(h);
	ctx->priv = NULL;
}

static enum apple_vxd_image_fmt
apple_vxd_hevc_get_image_fmt(struct apple_vxd_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id != V4L2_CID_STATELESS_HEVC_SPS)
		return APPLE_VXD_IMG_FMT_ANY;

	return APPLE_VXD_IMG_FMT_420_8BIT;
}

static size_t apple_vxd_hevc_capture_aux_size(struct apple_vxd_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;

	return apple_vxd_hevc_aux_size(pix->width, pix->height);
}

/* A CAPTURE buffer's side-buffer device and CPU addresses for @l. */
static bool apple_vxd_hevc_aux(struct vb2_v4l2_buffer *vbuf,
			       const struct apple_vxd_hevc_aux_layout *l,
			       u32 *va, struct apple_vxd_hevc_record **rec)
{
	u8 *cpu = apple_vxd_buf_aux_cpu(vbuf);

	if (!cpu || apple_vxd_buf_aux_size(vbuf) < l->size)
		return false;
	*va = apple_vxd_buf_aux_va(vbuf);
	*rec = (struct apple_vxd_hevc_record *)(cpu + l->record);
	return true;
}

static int apple_vxd_hevc_run(struct apple_vxd_ctx *ctx)
{
	const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;
	struct apple_vxd_hevc_ctx *h = ctx->priv;
	struct apple_vxd_dev *vxd = ctx->vxd;
	struct apple_vxd_hevc_frame f = { };
	struct apple_vxd_hevc_aux_layout l;
	struct apple_vxd_cmdbuf *cb = NULL;
	struct apple_vxd_hevc_record *rec;
	struct apple_vxd_run run;
	const char *why = NULL;
	struct vb2_queue *cap_q;
	unsigned int i, nslices = 0;
	u32 stride, aux;
	const u8 *src;
	int ret;

	apple_vxd_run_preamble(ctx, &run);

	if (!vxd->hw_ready || vxd->fw_panicked) {
		dev_dbg(vxd->dev, "decode job with no usable firmware\n");
		ret = -EAGAIN;
		goto out;
	}
	if (!h) {
		ret = -EINVAL;
		goto out;
	}

	f.sps = apple_vxd_hevc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SPS, NULL);
	f.pps = apple_vxd_hevc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_PPS, NULL);
	f.dp = apple_vxd_hevc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS, NULL);
	f.sm = apple_vxd_hevc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX, NULL);
	f.slices = apple_vxd_hevc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SLICE_PARAMS, &nslices);
	if (!f.sps || !f.pps || !f.dp || !f.slices) {
		ret = -EINVAL;
		goto out;
	}
	ret = apple_vxd_hevc_validate(f.sps, f.pps, &why);
	if (ret)
		goto out_why;
	if (f.sps->pic_width_in_luma_samples > pix->width ||
	    f.sps->pic_height_in_luma_samples > pix->height) {
		why = "picture larger than the CAPTURE format";
		ret = -EINVAL;
		goto out_why;
	}

	src = vb2_plane_vaddr(&run.src->vb2_buf, 0);
	if (!src) {
		why = "OUTPUT buffer has no kernel mapping";
		ret = -EINVAL;
		goto out_why;
	}
	ret = apple_vxd_hevc_find_nals(src, vb2_get_plane_payload(&run.src->vb2_buf, 0),
				       f.slices, min(nslices, APPLE_VXD_HEVC_MAX_SLICES),
				       h->nals, &why);
	if (ret)
		goto out_why;

	apple_vxd_hevc_aux_layout(f.sps, &l);
	stride = pix->plane_fmt[0].bytesperline;
	f.nals = h->nals;
	f.nslices = nslices;
	f.caps = &vxd->variant->hevc;
	f.src = apple_vxd_buf_va(run.src);
	f.dst_luma = apple_vxd_buf_va(run.dst);
	f.dst_chroma = f.dst_luma + stride * pix->height;
	f.dst_stride = stride;
	f.dst_cstride = stride;
	if (!apple_vxd_hevc_aux(run.dst, &l, &aux, &rec)) {
		why = "CAPTURE side buffer missing or too small";
		ret = -EINVAL;
		goto out_why;
	}
	f.mv = aux + l.mv;
	f.frag = aux + l.frag;
	f.above = aux + l.above;
	f.record = rec;
	rec->valid = 0;

	cap_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	for (i = 0; i < f.dp->num_active_dpb_entries && i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++) {
		struct vb2_buffer *ref = vb2_find_buffer(cap_q, f.dp->dpb[i].timestamp);
		struct apple_vxd_hevc_record *rrec;
		u32 raux;

		if (!ref || !apple_vxd_hevc_aux(to_vb2_v4l2_buffer(ref), &l, &raux, &rrec)) {
			dev_dbg(vxd->dev, "hevc: dpb[%u] ts %llu has no CAPTURE buffer\n",
				i, f.dp->dpb[i].timestamp);
			continue;
		}
		f.refs[i].luma = apple_vxd_buf_va(to_vb2_v4l2_buffer(ref));
		f.refs[i].chroma = f.refs[i].luma + stride * pix->height;
		f.refs[i].rec = rrec;
	}
	for (i = 0; i < APPLE_VXD_HEVC_GENC_BUFS; i++)
		f.genc[i] = h->genc[i].va;
	f.genc_size = h->genc_size;
	f.side = h->side.va;
	f.side_cpu = h->side.mem.vaddr;
	f.side_size = APPLE_VXD_HEVC_SIDE_SIZE;
	f.spdest = h->spdest.va;
	f.intra = h->intra.va;
	f.auxline = h->auxline.va;
	f.stream_state = h->stream_state.va;
	f.first_slice_of_session = h->first_slice;
	f.first_picture_of_session = h->first_picture;

	cb = apple_vxd_cmdbuf_get(vxd);
	if (!cb) {
		why = "no free command buffer";
		ret = -EBUSY;
		goto out_why;
	}
	ret = apple_vxd_hevc_build(cb, &f, &why);
	if (ret)
		goto out_why;

	dev_dbg(vxd->dev,
		"hevc decode: src %u va %#x, %u slices (first NAL %#x+%u, header %u bits), dst %u Y %#x UV %#x stride %u, %zu command bytes in buf %u (va %#x), %u packets\n",
		run.src->vb2_buf.index, f.src, nslices, h->nals[0].offset,
		h->nals[0].size, h->nals[0].header_bits, run.dst->vb2_buf.index,
		f.dst_luma, f.dst_chroma, stride, cb->used, cb->index, cb->va,
		cb->npackets);

	/* Controls were only read while building; the job may finish any time now. */
	apple_vxd_run_postamble(ctx, &run);
	ret = apple_vxd_submit(ctx, cb);
	if (ret) {
		dev_warn_ratelimited(vxd->dev, "hevc decode: submit failed: %d\n", ret);
		if (cb->state == APPLE_VXD_CMDBUF_HOST)
			apple_vxd_cmdbuf_put(vxd, cb);
		return ret;
	}
	h->first_slice = false;
	h->first_picture = false;
	return 0;

out_why:
	dev_warn_ratelimited(vxd->dev, "hevc decode: picture refused (%d): %s\n",
			     ret, why ? why : "?");
out:
	if (cb)
		apple_vxd_cmdbuf_put(vxd, cb);
	apple_vxd_run_postamble(ctx, &run);
	return ret;
}

static const struct apple_vxd_ctrl_desc apple_vxd_hevc_ctrl_descs[] = {
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SPS,
		.cfg.ops = &apple_vxd_ctrl_ops,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_PPS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
		.cfg.flags = V4L2_CTRL_FLAG_DYNAMIC_ARRAY,
		.cfg.dims = { APPLE_VXD_HEVC_MAX_SLICES },
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.cfg.min = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.max = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
		.cfg.def = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED,
	},
	{
		.cfg.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.cfg.min = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.max = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
		.cfg.def = V4L2_STATELESS_HEVC_START_CODE_ANNEX_B,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE,
		.cfg.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.cfg.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.cfg.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.cfg.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1,
		.cfg.def = V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1,
	},
};

static const struct apple_vxd_ctrls apple_vxd_hevc_ctrls = {
	.ctrls = apple_vxd_hevc_ctrl_descs,
	.num_ctrls = ARRAY_SIZE(apple_vxd_hevc_ctrl_descs),
};

static const struct apple_vxd_decoded_fmt_desc apple_vxd_hevc_decoded_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.image_fmt = APPLE_VXD_IMG_FMT_420_8BIT,
	},
};

static const struct apple_vxd_coded_fmt_desc apple_vxd_hevc_fmt = {
	.fourcc = V4L2_PIX_FMT_HEVC_SLICE,
	.frmsize = {
		.min_width = APPLE_VXD_MIN_PIC_W,
		.max_width = APPLE_VXD_MAX_PIC_W,
		.step_width = 16,
		.min_height = APPLE_VXD_MIN_PIC_H,
		.max_height = APPLE_VXD_MAX_PIC_H,
		.step_height = 16,
	},
	.ctrls = &apple_vxd_hevc_ctrls,
	.ops = &(const struct apple_vxd_coded_fmt_ops){
		.start = apple_vxd_hevc_start,
		.stop = apple_vxd_hevc_stop,
		.run = apple_vxd_hevc_run,
		.try_ctrl = apple_vxd_hevc_try_ctrl,
		.get_image_fmt = apple_vxd_hevc_get_image_fmt,
		.capture_aux_size = apple_vxd_hevc_capture_aux_size,
	},
	.num_decoded_fmts = ARRAY_SIZE(apple_vxd_hevc_decoded_fmts),
	.decoded_fmts = apple_vxd_hevc_decoded_fmts,
};

/*
 * Adds HEVC to the device's coded formats when the SoC's match data has
 * it: AppleD5500 advertises HEVCSupported for vxd-version 7, 9 and 10.
 */
int apple_vxd_hevc_register(struct apple_vxd_dev *vxd)
{
	const struct apple_vxd_variant *v = vxd->variant;

	if (!v->hevc_supported || !v->hevc.buf_mult)
		return 0;
	if (vxd->core_rev == APPLE_VXD_CORE_REV_BRN50577) {
		dev_info(vxd->dev, "hevc: core revision %#x needs BRN50577, not implemented\n",
			 vxd->core_rev);
		return 0;
	}
	return apple_vxd_register_coded_fmt(vxd, &apple_vxd_hevc_fmt);
}
