/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Apple A10 (T8010): Rogue hardware behind Apple's GFX firmware. */

#ifndef PVR_APPLE_H
#define PVR_APPLE_H

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <uapi/drm/pvr_drm.h>

#include "pvr_rogue_fwif_client.h"

struct dma_fence;
struct pvr_context;
struct pvr_device;
struct pvr_job;
struct pvr_queue;
struct pvr_power_sequence_ops;
struct pvr_vm_context;

extern const struct pvr_power_sequence_ops pvr_power_sequence_ops_apple;

/*
 * The A10's GPU resolves 38-bit data addresses (a triangle renders with its
 * buffers at 0x20_xxxx_xxxx, not at 0x40_xxxx_xxxx), below powervr's Rogue
 * heaps; Apple devices get the same heaps below 2^38. The firmware's PDS and
 * USC code bases are the code heaps' bases.
 */
#define PVR_APPLE_GENERAL_HEAP_BASE		0x0800000000ull
#define PVR_APPLE_PDSCODEDATA_HEAP_BASE		0x2a00000000ull
#define PVR_APPLE_USCCODE_HEAP_BASE		0x2c00000000ull
#define PVR_APPLE_RGNHDR_HEAP_BASE		0x2ef0000000ull
#define PVR_APPLE_VISTEST_HEAP_BASE		0x2f00000000ull
#define PVR_APPLE_TRANSFER_FRAG_HEAP_BASE	0x3000000000ull

/*
 * Geometry registers of Apple's StartTA that the Rogue command, the HWRT
 * dataset and the free list do not determine, as raw register values
 * (stream extension PVR_STREAM_EXTHDR_TYPE_GEOM_APPLE).
 */
struct pvr_apple_geom_regs {
	u64 pds_ctrl;			/* 0xc90 */
	u64 ctx_state_base[2];		/* 0x428, 0xcc0 */
	u64 vdm_ctx_store_task[3];	/* 0x438, 0x440, 0x448 */
	u64 vdm_ctx_resume_task[3];	/* 0x450, 0x458, 0x460 */
	u64 vdm_ctx_state;		/* 0x470, PDS-relative */
	u64 vdm_call_stack;		/* 0x418 */
	u64 pds_state[6];		/* 0xf058, 0xf060, 0xf068, 0xf070, 0xf038, 0xf040 */
	u64 cluster[6];			/* 0x11030, per cluster */
	u32 r_c78;			/* 0xc78 */
	u32 padding;
};

struct pvr_apple_cmd_geom {
	struct rogue_fwif_cmd_geom cmd;
	struct pvr_apple_geom_regs apple;
};

/*
 * Fragment registers of Apple's Start3D beyond the Rogue command
 * (PVR_STREAM_EXTHDR_TYPE_FRAG_APPLE). The background program takes four
 * registers (PDS_BGRND0..3) where the Rogue command has three.
 */
struct pvr_apple_frag_regs {
	u64 isp_sample_pos;		/* 0xfa8 */
	u64 pds_bgnd[4];		/* 0x6a0, 0x6a8, 0x6b0, 0x6b8 */
	u64 cluster0[6];		/* 0x11020, per cluster */
	u64 cluster1[6];		/* 0x11080, per cluster */
	u64 r_fd0;			/* 0xfd0 */
	u32 event_pixel_pds_code;	/* 0x618 */
	u32 isp_rgn;			/* 0xf28 */
	u32 r_6d0;			/* 0x6d0 */
	u32 r_3e00;			/* 0x3e00 */
	u32 r_fd8;			/* 0xfd8 */
	u32 r_4070;			/* 0x4070 */
	u32 line[6];			/* 0x11d0 ... 0x11f8: line rasterization */
};

struct pvr_apple_cmd_frag {
	struct rogue_fwif_cmd_frag cmd;
	struct pvr_apple_frag_regs apple;
};

#if IS_ENABLED(CONFIG_DRM_POWERVR_APPLE)
int pvr_apple_fw_init(struct pvr_device *pvr_dev);
void pvr_apple_fw_fini(struct pvr_device *pvr_dev);
struct pvr_gem_object *pvr_apple_pm_pages(struct pvr_device *pvr_dev, size_t size);
int pvr_apple_reset(struct pvr_device *pvr_dev);
int pvr_apple_suspend(struct pvr_device *pvr_dev);
int pvr_apple_resume(struct pvr_device *pvr_dev);
void pvr_apple_device_info_init(struct pvr_device *pvr_dev);
struct dma_fence *pvr_apple_submit_compute_job(struct pvr_device *pvr_dev,
					       struct pvr_vm_context *vm_ctx, u64 stream,
					       u64 ctx_state);
struct dma_fence *pvr_apple_submit_render_job(struct pvr_job *job);
void pvr_apple_report_hang(struct pvr_device *pvr_dev, struct dma_fence *fence);
void pvr_apple_wait_file_jobs(struct pvr_device *pvr_dev);
int pvr_apple_job_add_deps(struct pvr_job *job);
void pvr_apple_job_armed(struct pvr_job *job, struct dma_fence *finished);
int pvr_apple_vm_init(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx);
void pvr_apple_mmu_flush(struct pvr_device *pvr_dev);
void pvr_apple_vm_fini(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx);
void pvr_apple_quiesce(struct pvr_device *pvr_dev);

struct pvr_queue *pvr_apple_queue_create(struct pvr_context *ctx, enum drm_pvr_job_type type);
void pvr_apple_queue_destroy(struct pvr_queue *queue, bool cleanup_queue_entity);
int pvr_apple_queue_job_init(struct pvr_job *job, u64 drm_client_id);
void pvr_apple_queue_job_cleanup(struct pvr_job *job);
#else
static inline int pvr_apple_fw_init(struct pvr_device *pvr_dev) { return -ENODEV; }
static inline void pvr_apple_fw_fini(struct pvr_device *pvr_dev) {}
static inline struct pvr_gem_object *pvr_apple_pm_pages(struct pvr_device *pvr_dev, size_t size)
{
	return ERR_PTR(-E2BIG);
}
static inline int pvr_apple_reset(struct pvr_device *pvr_dev) { return -ENODEV; }
static inline int pvr_apple_suspend(struct pvr_device *pvr_dev) { return 0; }
static inline int pvr_apple_resume(struct pvr_device *pvr_dev) { return 0; }
static inline void pvr_apple_device_info_init(struct pvr_device *pvr_dev) {}
static inline int pvr_apple_vm_init(struct pvr_device *pvr_dev, struct pvr_vm_context *vm_ctx)
{
	return -ENODEV;
}
static inline struct pvr_queue *pvr_apple_queue_create(struct pvr_context *ctx,
							enum drm_pvr_job_type type)
{
	return ERR_PTR(-ENODEV);
}
static inline void pvr_apple_queue_destroy(struct pvr_queue *queue, bool cleanup_queue_entity) {}
static inline int pvr_apple_queue_job_init(struct pvr_job *job, u64 drm_client_id)
{
	return -ENODEV;
}
static inline void pvr_apple_queue_job_cleanup(struct pvr_job *job) {}
static inline int pvr_apple_job_add_deps(struct pvr_job *job) { return 0; }
static inline void pvr_apple_job_armed(struct pvr_job *job, struct dma_fence *finished) {}
static inline void pvr_apple_mmu_flush(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_quiesce(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_wait_file_jobs(struct pvr_device *pvr_dev) {}
static inline void pvr_apple_report_hang(struct pvr_device *pvr_dev, struct dma_fence *fence) {}
static inline void pvr_apple_vm_fini(struct pvr_device *pvr_dev,
				     struct pvr_vm_context *vm_ctx) {}
#endif

#endif /* PVR_APPLE_H */
