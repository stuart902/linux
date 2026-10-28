// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD (Imagination MSVDX) video decoder driver - core
 *
 * Copyright (c) 2026 Berkay Kösebay <brkosebay@users.noreply.github.com>
 *
 * Probe powers the vdec domain, maps the 1 MiB register window, reads the
 * core identity and registers the V4L2 decoder. The DEVA firmware boots
 * on the first open of the video node (apple-vxd-fw.c).
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "apple-vxd.h"

/*
 * Take the active job if @last_signalled says it is done (the completion
 * path) or unconditionally (the watchdog).  job_lock makes the two paths
 * exclusive: exactly one of them finishes a job.
 */
static bool vxd_job_take(struct apple_vxd_dev *vxd, bool need_signal,
			 struct apple_vxd_job *job)
{
	unsigned long flags;
	bool taken = false;

	spin_lock_irqsave(&vxd->job_lock, flags);
	if (vxd->job_active && vxd->job.nfences &&
	    (!need_signal ||
	     READ_ONCE(vxd->job.fences[vxd->job.nfences - 1]->state) ==
			APPLE_VXD_FENCE_SIGNALLED)) {
		*job = vxd->job;
		vxd->job_active = false;
		taken = true;
	}
	spin_unlock_irqrestore(&vxd->job_lock, flags);
	return taken;
}

/*
 * Every fence of the job goes back to the pool.  A packet whose 0xc0 did
 * not arrive (the firmware answers only the 0x4004 message) is simply
 * released; a fault on any armed packet that did answer fails the picture.
 */
static void vxd_job_complete(struct apple_vxd_dev *vxd,
			     const struct apple_vxd_job *job, bool timed_out)
{
	struct apple_vxd_fence *last = job->fences[job->nfences - 1];
	unsigned int i, answered = 0;
	bool error = timed_out;
	u32 status = 0;

	for (i = 0; i < job->nfences; i++) {
		struct apple_vxd_fence *f = job->fences[i];

		if (READ_ONCE(f->state) == APPLE_VXD_FENCE_SIGNALLED) {
			answered++;
			error |= f->error;
			status |= f->status;
		}
	}
	dev_dbg(vxd->dev,
		"decode: %s: fence %#x (seq %#x), %u of %u packets answered, status %#x, %lld us -> %s\n",
		timed_out ? "TIMEOUT" : "completion", last->id, job->seq,
		answered, job->npackets, status,
		ktime_us_delta(ktime_get(), job->submitted),
		error ? "ERROR" : "DONE");
	for (i = 0; i < job->nfences; i++)
		apple_vxd_fence_release(vxd, job->fences[i]);
	apple_vxd_job_finish(job->ctx, error ? VB2_BUF_STATE_ERROR :
						 VB2_BUF_STATE_DONE);
}

void apple_vxd_job_done(struct work_struct *work)
{
	struct apple_vxd_dev *vxd = container_of(work, struct apple_vxd_dev,
						 job_done_work);
	struct apple_vxd_job job;

	if (!vxd_job_take(vxd, true, &job))
		return;
	cancel_delayed_work(&vxd->watchdog_work);
	vxd_job_complete(vxd, &job, false);
}

/*
 * No 0xc0 for the last packet in APPLE_VXD_JOB_TIMEOUT_MS: fail the
 * picture.  The command buffer stays quarantined (the core may still
 * read it) until the firmware is stopped.
 */
void apple_vxd_watchdog(struct work_struct *work)
{
	struct apple_vxd_dev *vxd = container_of(work, struct apple_vxd_dev,
						 watchdog_work.work);
	struct apple_vxd_job job;

	if (!vxd_job_take(vxd, false, &job))
		return;

	dev_warn(vxd->dev,
		 "decode job timed out: no completion for fence %#x in %u ms (TO_MTX rd %u wr %u, TO_HOST rd %u wr %u, 0x200c %#x, irq status %#x)\n",
		 job.fences[job.nfences - 1]->id, APPLE_VXD_JOB_TIMEOUT_MS,
		 vxd_read(vxd, VXD_REG_TO_MTX_RD_INDEX),
		 vxd_read(vxd, VXD_REG_TO_MTX_WR_INDEX),
		 vxd_read(vxd, VXD_REG_TO_HOST_RD_INDEX),
		 vxd_read(vxd, VXD_REG_TO_HOST_WR_INDEX),
		 vxd_read(vxd, VXD_REG_FW_PC), vxd_read(vxd, VXD_REG_IRQ_STATUS));
	vxd_job_complete(vxd, &job, true);
}

/* Power on and read the identity and feature registers.  Read-only. */
static int apple_vxd_probe_hw(struct apple_vxd_dev *vxd)
{
	u32 memcfg, id, rev;
	int ret;

	ret = pm_runtime_resume_and_get(vxd->dev);
	if (ret) {
		dev_err_probe(vxd->dev, ret, "failed to power the vdec domain\n");
		return ret;
	}

	rev = vxd_read(vxd, VXD_REG_CORE_REV);
	id = vxd_read(vxd, VXD_REG_CORE_ID);
	vxd->core_rev = rev;
	vxd->core_id = id;

	/*
	 * AppleD5500::GetCoreRev rejects MSVDX cores whose coreId high half
	 * is <= 6 ("MSVDX is unsupported").  Note the register semantics:
	 * the gate value is the high u16 of 0x630 (on this A9: 0x0303 = 771,
	 * well above 6), while the ADT's "vxd-version 7" most plausibly
	 * matches the 0x7 nibble of coreRev 0x70b00.  AppleD5500 itself
	 * never parses coreRev further; feature gating in iOS uses the ADT
	 * vxd-version property.  (j71t device run 1, 2026-10-23.)
	 */
	if ((id >> VXD_CORE_ID_MAJOR_SH) <= 6) {
		dev_err(vxd->dev, "MSVDX is unsupported (coreId %#x)\n", id);
		pm_runtime_put_autosuspend(vxd->dev);
		return -ENODEV;
	}

	vxd->num_entropy_pipes = id & VXD_CORE_ID_ENT_PIPES;
	vxd->num_pixel_pipes = FIELD_GET(VXD_CORE_ID_PIX_PIPES, id);

	memcfg = vxd_read(vxd, VXD_REG_MTX_MEM_CFG);
	vxd->mtx_mem_size = (u32)((FIELD_GET(VXD_MTX_MEM_BANK_COUNT, memcfg) - 1) *
				  (4 << FIELD_GET(VXD_MTX_MEM_BANK_SIZE, memcfg))) +
			    (4 << FIELD_GET(VXD_MTX_MEM_LAST_BANK, memcfg));

	vxd->mmu_40bit = ((vxd_read(vxd, VXD_REG_MTX_MMU_CFG0) & 0xf0) ==
			  VXD_MTX_MMU_CFG0_40BIT);
	vxd->mmu_tiling_ctx = !!(vxd_read(vxd, VXD_REG_MTX_MMU_CFG1) &
				 VXD_MTX_MMU_CFG1_TILING_CTX);

	/* Raw pipe status; per-pipe selection (0x460) needs a write. */
	vxd->pipe_status = vxd_read(vxd, VXD_REG_PIPE_STATUS);

	dev_dbg(vxd->dev,
		"coreId %#x, coreRev %#x, entropy pipes %u, pixel pipes %u, mtx mem %u KiB, mmu40 %u, tiling-ctx %u, pipe status %#x\n",
		id, rev, vxd->num_entropy_pipes, vxd->num_pixel_pipes,
		vxd->mtx_mem_size, vxd->mmu_40bit, vxd->mmu_tiling_ctx,
		vxd->pipe_status);

	pm_runtime_put_autosuspend(vxd->dev);
	return 0;
}

/* Job plumbing shared by the m2m device and the watchdog. */
void apple_vxd_job_finish_no_pm(struct apple_vxd_ctx *ctx,
				enum vb2_buffer_state result)
{
	v4l2_m2m_buf_done_and_job_finish(ctx->vxd->m2m_dev, ctx->fh.m2m_ctx,
					 result);
}

void apple_vxd_job_finish(struct apple_vxd_ctx *ctx,
			  enum vb2_buffer_state result)
{
	struct apple_vxd_dev *vxd = ctx ? ctx->vxd : NULL;

	if (vxd) {
		unsigned long flags;

		spin_lock_irqsave(&vxd->job_lock, flags);
		vxd->job_active = false;
		spin_unlock_irqrestore(&vxd->job_lock, flags);
		pm_runtime_put_autosuspend(vxd->dev);
	}

	if (ctx)
		apple_vxd_job_finish_no_pm(ctx, result);
}

void apple_vxd_run_preamble(struct apple_vxd_ctx *ctx,
			    struct apple_vxd_run *run)
{
	struct media_request *src_req;

	memset(run, 0, sizeof(*run));

	run->src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	run->dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	/* Apply request(s) controls if needed. */
	src_req = run->src->vb2_buf.req_obj.req;
	if (src_req)
		v4l2_ctrl_request_setup(src_req, &ctx->ctrl_hdl);

	v4l2_m2m_buf_copy_metadata(run->src, run->dst);
}

void apple_vxd_run_postamble(struct apple_vxd_ctx *ctx,
			     struct apple_vxd_run *run)
{
	struct media_request *src_req = run->src->vb2_buf.req_obj.req;

	if (src_req)
		v4l2_ctrl_request_complete(src_req, &ctx->ctrl_hdl);
}

/* Runtime PM stubs: the genpd (ps_vdec) does all the work. */
static int apple_vxd_runtime_resume(struct device *dev)
{
	return 0;
}

static int apple_vxd_runtime_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops apple_vxd_pm_ops = {
	RUNTIME_PM_OPS(apple_vxd_runtime_suspend, apple_vxd_runtime_resume, NULL)
};

static const struct apple_vxd_variant s8000_vxd_variant = {
	.vxd_version	= 7,
	.num_gart_pts	= 512,		/* "setting 2GB mapping" [E] */
	/* HEVCSupported = ((rev - 7) < 4) && ((0xD >> (rev - 7)) & 1) [E] */
	.hevc_supported	= true,
};

static const struct of_device_id apple_vxd_of_match[] = {
	{ .compatible = "apple,s8000-vxd", .data = &s8000_vxd_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_vxd_of_match);

static int apple_vxd_probe(struct platform_device *pdev)
{
	const struct apple_vxd_variant *variant;
	struct apple_vxd_dev *vxd;
	struct resource *res;
	unsigned int i;
	int ret, irq;

	variant = of_device_get_match_data(&pdev->dev);
	if (!variant)
		return -EINVAL;

	vxd = devm_kzalloc(&pdev->dev, sizeof(*vxd), GFP_KERNEL);
	if (!vxd)
		return -ENOMEM;

	vxd->dev = &pdev->dev;
	vxd->variant = variant;
	platform_set_drvdata(pdev, vxd);

	mutex_init(&vxd->vdev_lock);
	mutex_init(&vxd->power_lock);
	spin_lock_init(&vxd->job_lock);
	spin_lock_init(&vxd->mbox_lock);
	for (i = 0; i < APPLE_VXD_NUM_FENCES; i++)
		init_completion(&vxd->fences[i].done);
	INIT_DELAYED_WORK(&vxd->watchdog_work, apple_vxd_watchdog);
	INIT_WORK(&vxd->job_done_work, apple_vxd_job_done);

	vxd->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(vxd->regs))
		return dev_err_probe(&pdev->dev, PTR_ERR(vxd->regs),
				     "failed to map the vxd registers\n");
	vxd->regs_size = resource_size(res);

	/* The block has no DART; DMA addresses are physical and the core's
	 * own GART translates the low 32 bits into device address space.
	 * GART entries hold PA >> 8 in 32 bits: 40-bit physical addresses.
	 */
	ret = dma_set_mask_and_coherent(&pdev->dev,
					DMA_BIT_MASK(VXD_GART_PA_BITS));
	if (ret) {
		dev_err_probe(&pdev->dev, ret,
			      "failed to set the DMA mask\n");
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	vxd->irq = irq;

	/*
	 * Level interrupt, enabled only once the firmware has taken its init
	 * message (apple_vxd_mbox_start()).
	 */
	ret = devm_request_irq(&pdev->dev, irq, apple_vxd_irq, IRQF_NO_AUTOEN,
			       dev_name(&pdev->dev), vxd);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to request vxd IRQ\n");
		return ret;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, 100);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = apple_vxd_probe_hw(vxd);
	if (ret)
		goto err_disable_runtime_pm;

	ret = apple_vxd_gart_init(vxd);
	if (ret)
		goto err_disable_runtime_pm;

	ret = apple_vxd_h264_register(vxd);
	if (!ret)
		ret = apple_vxd_v4l2_init(vxd);
	if (ret)
		goto err_gart_fini;

	return 0;

err_gart_fini:
	apple_vxd_gart_fini(vxd);
err_disable_runtime_pm:
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static void apple_vxd_remove(struct platform_device *pdev)
{
	struct apple_vxd_dev *vxd = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&vxd->watchdog_work);
	apple_vxd_v4l2_cleanup(vxd);
	apple_vxd_fw_cleanup(vxd);
	/* fw_cleanup failed every fence: let a pending job finish first. */
	flush_work(&vxd->job_done_work);
	cancel_delayed_work_sync(&vxd->watchdog_work);
	apple_vxd_gart_fini(vxd);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static struct platform_driver apple_vxd_driver = {
	.probe = apple_vxd_probe,
	.remove = apple_vxd_remove,
	.driver = {
		.name = APPLE_VXD_DRV_NAME,
		.pm = &apple_vxd_pm_ops,
		.of_match_table = apple_vxd_of_match,
	},
};
module_platform_driver(apple_vxd_driver);

MODULE_DESCRIPTION("Apple VXD (PowerVR MSVDX) video decoder driver");
MODULE_FIRMWARE(APPLE_VXD_FW_NAME);
MODULE_AUTHOR("Berkay Kösebay <brkosebay@users.noreply.github.com>");
MODULE_LICENSE("GPL");
