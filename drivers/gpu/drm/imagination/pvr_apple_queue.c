// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A10 (T8010) job queues.
 *
 * Jobs go to Apple's GFX firmware through pvr_apple_submit_compute_job() rather
 * than IMG's CCCB and KCCB, so an Apple queue is a plain drm_sched instance:
 * run_job translates the firmware command powervr decoded from userspace
 * (struct rogue_fwif_cmd_compute, or the Apple geometry and fragment
 * commands) and returns the fence of the Apple job.
 */

#include "pvr_apple.h"
#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_job.h"
#include "pvr_power.h"
#include "pvr_queue.h"
#include "pvr_rogue_fwif_client.h"

#include <drm/drm_print.h>
#include <drm/gpu_scheduler.h>
#include <linux/slab.h>

#define APPLE_QUEUE_TIMEOUT_MS	5000

static struct dma_fence *pvr_apple_submit(struct pvr_job *job)
{
	const struct rogue_fwif_cmd_compute *cmd = job->cmd;

	if (job->type == DRM_PVR_JOB_TYPE_GEOMETRY || job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		return pvr_apple_submit_render_job(job);
	if (job->type != DRM_PVR_JOB_TYPE_COMPUTE)
		return ERR_PTR(-EOPNOTSUPP);

	return pvr_apple_submit_compute_job(job->pvr_dev, job->ctx->vm_ctx,
					    cmd->regs.cdm_ctrl_stream_base,
					    cmd->regs.cdm_context_state_base_addr);
}

/*
 * The GPU stays powered while the firmware has the job: the reference taken
 * here is dropped when the job's fence leaves the pending list (signalled or
 * cancelled), or right away if the job never reached the firmware.
 */
static struct dma_fence *pvr_apple_run_job(struct drm_sched_job *sched_job)
{
	struct pvr_job *job = container_of(sched_job, struct pvr_job, base);
	struct dma_fence *fence;
	int err;

	err = pvr_power_get(job->pvr_dev);
	if (err)
		return ERR_PTR(err);
	fence = pvr_apple_submit(job);
	if (IS_ERR(fence))
		pvr_power_put(job->pvr_dev);
	return fence;
}

static enum drm_gpu_sched_stat pvr_apple_timedout_job(struct drm_sched_job *sched_job)
{
	struct pvr_job *job = container_of(sched_job, struct pvr_job, base);
	struct drm_gpu_scheduler *sched = sched_job->sched;
	int err;

	/*
	 * Report the hang, restart the GPU (every job in flight ends with
	 * -ETIMEDOUT) and accept no further jobs on the guilty context: its
	 * owner sees the device lost, every other client goes on.
	 *
	 * The scheduler took the job off its pending list; drm_sched_stop()
	 * puts it back, and drm_sched_start() completes it and the others
	 * from their cancelled fences, so that free_job() runs for each.
	 */
	drm_err(from_pvr_device(job->pvr_dev), "Apple GPU job timed out\n");
	pvr_apple_report_hang(job->pvr_dev, sched_job->s_fence->parent);
	atomic_set(&job->ctx->faulty, 1);
	drm_sched_stop(sched, sched_job);
	err = pvr_apple_reset(job->pvr_dev);
	drm_sched_start(sched, 0);
	return err ? DRM_GPU_SCHED_STAT_ENODEV : DRM_GPU_SCHED_STAT_RESET;
}

static void pvr_apple_free_job(struct drm_sched_job *sched_job)
{
	struct pvr_job *job = container_of(sched_job, struct pvr_job, base);

	drm_sched_job_cleanup(sched_job);
	pvr_job_put(job);
}

static const struct drm_sched_backend_ops pvr_apple_sched_ops = {
	.run_job = pvr_apple_run_job,
	.timedout_job = pvr_apple_timedout_job,
	.free_job = pvr_apple_free_job,
};

struct pvr_queue *pvr_apple_queue_create(struct pvr_context *ctx, enum drm_pvr_job_type type)
{
	struct pvr_device *pvr_dev = ctx->pvr_dev;
	const struct drm_sched_init_args sched_args = {
		.ops = &pvr_apple_sched_ops,
		.submit_wq = pvr_dev->sched_wq,
		.num_rqs = 1,
		.credit_limit = 64,
		.hang_limit = 1,
		.timeout = msecs_to_jiffies(APPLE_QUEUE_TIMEOUT_MS),
		.timeout_wq = pvr_dev->sched_wq,
		.name = "pvr-apple-queue",
		.dev = pvr_dev->base.dev,
	};
	struct drm_gpu_scheduler *sched;
	struct pvr_queue *queue;
	int err;

	if (type != DRM_PVR_JOB_TYPE_COMPUTE && type != DRM_PVR_JOB_TYPE_GEOMETRY &&
	    type != DRM_PVR_JOB_TYPE_FRAGMENT)
		return ERR_PTR(-EOPNOTSUPP);

	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return ERR_PTR(-ENOMEM);
	queue->type = type;
	queue->ctx = ctx;
	INIT_LIST_HEAD(&queue->node);
	sched = &queue->scheduler;

	err = drm_sched_init(sched, &sched_args);
	if (err)
		goto err_free;
	err = drm_sched_entity_init(&queue->entity, DRM_SCHED_PRIORITY_KERNEL, &sched, 1,
				    &ctx->faulty);
	if (err)
		goto err_sched_fini;
	return queue;

err_sched_fini:
	drm_sched_fini(sched);
err_free:
	kfree(queue);
	return ERR_PTR(err);
}

void pvr_apple_queue_destroy(struct pvr_queue *queue, bool cleanup_queue_entity)
{
	drm_sched_fini(&queue->scheduler);
	if (cleanup_queue_entity)
		drm_sched_entity_fini(&queue->entity);
	dma_fence_put(queue->last_queued_job_scheduled_fence);
	kfree(queue);
}

int pvr_apple_queue_job_init(struct pvr_job *job, u64 drm_client_id)
{
	struct pvr_queue *queue;

	if (atomic_read(&job->ctx->faulty))
		return -EIO;
	queue = pvr_context_get_queue_for_job(job->ctx, job->type);
	if (!queue)
		return -EINVAL;
	return drm_sched_job_init(&job->base, &queue->entity, 1, THIS_MODULE, drm_client_id);
}

void pvr_apple_queue_job_cleanup(struct pvr_job *job)
{
	if (job->base.s_fence)
		drm_sched_job_cleanup(&job->base);
}
