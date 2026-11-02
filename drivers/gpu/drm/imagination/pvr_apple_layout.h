/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Apple A10 (T8010) GPU firmware memory layout; see pvr_apple_layout.c. */

#ifndef PVR_APPLE_LAYOUT_H
#define PVR_APPLE_LAYOUT_H

#include <linux/types.h>

#define PVR_APPLE_WINDOW_SIZE	0x100000u	/* IOP window: image, RTKit, init */
#define PVR_APPLE_FW_RUNTIME	0x8f000u	/* image + firmware runtime data */
#define PVR_APPLE_HEAP_SIZE	0x400000u

/*
 * gpu_root: physical address of the context-0 GPU page-table root.
 * pds_base, usc_base: GPU addresses PDS and USC code offsets are relative to.
 */
void pvr_apple_layout_build(u8 *win, u64 win_pa, u8 *heap, u64 heap_pa, u64 gpu_root,
			    u64 pds_base, u64 usc_base);

/* GPU virtual addresses the compute command and its CDM arguments use */
#define PVR_APPLE_GPU_DATA	0x200000ull	/* stream at +0, PDS HALT at +0x1000 */
void pvr_apple_layout_write_compute_job(u8 *heap, u32 seq, u64 stream, u64 ctx_state);
/* A geometry (TA) job; see pvr_apple_layout_write_ta_job() */
struct pvr_apple_ta_job {
	u32 regs32[8];		/* RGX 0xc78, 0xc88, 0xc98, 0xc18, 0xc08, 0xc10, 0xc20, 0xc28 */
	u64 regs64[28];		/* RGX 0xc90 ... 0x408, 0xf058 ... 0xf040, 0x11030 x 6 */
	u64 free_list;		/* GPU address of the free stack (u32 page numbers) */
	u32 free_list_pages;
	u64 mlist, mtile_array, vheap_table;
	u32 skip;		/* non-zero: complete without kicking the hardware */
};

void pvr_apple_layout_build_ta(u8 *heap);
void pvr_apple_layout_write_ta_job(u8 *heap, u32 seq, const struct pvr_apple_ta_job *job);
u32 pvr_apple_layout_ta_producer(void);
u32 pvr_apple_layout_ta_stamp_offset(void);
u32 pvr_apple_layout_ta_units_offset(void);
u32 pvr_apple_layout_ta_pm_slot_offset(void);
u32 pvr_apple_layout_ta_free_list_offset(void);
u32 pvr_apple_layout_ta_pm_state_offset(void);
/* A fragment (3D) job; see pvr_apple_layout_write_3d_job() */
struct pvr_apple_3d_job {
	u32 regs32[18];		/* RGX 0xf18, 0xf80, 0xf88, -, 0x6d0, 0x618 ... 0x11f8 */
	u64 regs64[29];		/* RGX 0xfa8 ... 0xf30, 0x11020 x 6, 0x11080 x 6, 0xfd0 */
};

void pvr_apple_layout_build_3d(u8 *heap);
void pvr_apple_layout_write_3d_job(u8 *heap, u32 seq, const struct pvr_apple_3d_job *job);
u32 pvr_apple_layout_3d_producer(void);
u32 pvr_apple_layout_ring_indices_offset(unsigned int ring);
u32 pvr_apple_layout_3d_stamp_offset(void);
u32 pvr_apple_layout_mmu_root_offset(unsigned int ctx);
u32 pvr_apple_layout_mmu_gen_offset(unsigned int ctx);
u32 pvr_apple_layout_init_offset(void);
u32 pvr_apple_layout_compute_producer(void);
u32 pvr_apple_layout_ring_index(u32 seq);
u32 pvr_apple_layout_stamp_offset(void);
u32 pvr_apple_layout_debug_flags_offset(void);
u32 pvr_apple_layout_sku_index(void);
u32 pvr_apple_layout_sku_record(unsigned int i);
u32 pvr_apple_layout_event_consumer(void);
u32 pvr_apple_layout_event_producer(void);
u32 pvr_apple_layout_event_entry(unsigned int i);

#endif /* PVR_APPLE_LAYOUT_H */
