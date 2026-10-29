// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple A10 (T8010) GPU firmware memory layout.
 *
 * Apple's GFX firmware runs on an A7 IOP with its own MMU. It sees two pieces of
 * host memory:
 *
 *  - the window: 1 MiB at IOP address 0, holding the firmware image, the RTKit
 *    buffers, the init page and the memory descriptors;
 *  - the heap: host memory the firmware maps at fixed firmware virtual
 *    addresses through those memory descriptors. It holds the firmware's
 *    contexts, rings and queues, context 0's page-table root and the
 *    command data.
 *
 * pvr_apple_layout_build() fills both from scratch. It is a pure function of
 * the two buffers and their physical addresses, so the layout can be checked
 * against reference images outside the kernel.
 *
 * Names follow the firmware's structures as far as they are understood:
 *   X  main context (init+0x18): context pointers, ring indices, ring pointers
 *   P  device context (X[0]): fixed GPU address windows, I/O table, unit masks
 *   Q  flush scratch (X[1]), Z (X[2]), T MMU context table (init+0x1c)
 *   U  debug/ready flags (init+0x98), V mapping-update ring (init+0x9c)
 */

#include "pvr_apple_layout.h"

#define FW_VA_HEAP		0x00100000u	/* heap descriptor 0 */
#define FW_VA_HEAP2		0x00140000u	/* heap descriptor 1, inner non-cacheable */
#define HEAP2_OFFSET		0x50000u

/* Window offsets (IOP address == window offset) */
#define WIN_INIT		0x98000u
#define WIN_DESC		0x9c000u

/* Heap offsets of the firmware contexts */
#define HEAP_X			0x00000u
#define HEAP_P			0x02000u
#define HEAP_T			0x04000u
#define HEAP_U			0x05000u
#define HEAP_Q			0x06000u
#define HEAP_V			0x08000u
#define HEAP_RING_INDICES	0x11000u
#define HEAP_MMU_ROOTS		0x26000u
#define HEAP_COMPUTE_RING	0x29000u
#define HEAP_QUEUE		0x2b000u
#define HEAP_QUEUE_STATE	0x2c000u
#define HEAP_CMD_RING		0x2d000u
#define HEAP_COMPUTE_CMD	0x2f000u
#define HEAP_TA_RING		0x30000u	/* ring 0: geometry work ring */
#define HEAP_TA_QUEUE		0x32000u
#define HEAP_TA_QUEUE_STATE	0x32200u
#define HEAP_TA_CMD_RING	0x32400u
#define HEAP_TA_STATUS		0x32800u
#define HEAP_TA_POOL		0x33000u	/* scheduling entries, 128 x 44 bytes */
#define HEAP_TA_CMD		0x35000u	/* geometry command, 0x600 bytes */
#define HEAP_TA_USEQ		0x35600u	/* microsequence, 0xb0 bytes */
#define HEAP_TA_REGS32		0x35700u
#define HEAP_TA_REGS64		0x35800u
#define HEAP_TA_SCRATCH		0x35a00u	/* flags and pointers the steps name */
#define HEAP_TA_FREE_LIST	0x36000u	/* free-list descriptor, 0x74 bytes */
#define HEAP_TA_PM_SLOT		0x36100u	/* parameter-manager slot, 0x100 bytes */
#define HEAP_TA_CONTEXT		0x36200u	/* geometry context, 0x84 bytes */
#define HEAP_TA_PM_STATE	0x36300u
#define HEAP_TA_PM_HEADER	0x36400u
#define HEAP_TA_UNITS		0x1c000u	/* X+0x134: geometry unit states */
#define HEAP_3D_UNITS		0x1d000u	/* X+0x138: fragment unit states */
#define HEAP_SEQ_PROGRAM	0x38000u
#define HEAP_MMU_GEN		0x52000u	/* fw VA 0x142000 */
#define HEAP_CTX_STATE		0x53000u	/* fw VA 0x143000 */
#define HEAP_USEQ		0x54000u	/* fw VA 0x144000: microsequence */
#define HEAP_STAMP		0x55000u	/* fw VA 0x145000: completion stamps */
#define HEAP_UNIT_STATE		0x56000u	/* fw VA 0x146000 */
#define HEAP_POWER_STATE	0x69000u	/* X+0x301: firmware power state, >= 0x4314 bytes */
#define HEAP_EVENT_RING		0x57000u	/* control page, then 256 x 0x24-byte events */
#define HEAP_CDM_ARGS		0x5b000u	/* fw VA 0x14b000 */
#define HEAP_CDM_REGS		0x5c000u	/* fw VA 0x14c000 */
#define HEAP_SKU_RECORDS	0x70000u	/* 512 x 36-byte trace records (debug flags) */
#define HEAP_SKU_CONTROL	0x75000u	/* +0x20: index of the next record */
#define HEAP_PDS_PD		0x80000u
#define HEAP_3D_CMD		0x76000u	/* fragment command, 0x908 bytes */
#define HEAP_3D_USEQ		0x77000u	/* microsequence, 0xc8 bytes */
#define HEAP_3D_SCRATCH		0x77100u
#define HEAP_3D_CONTEXT		0x77200u	/* fragment context, 0x84 bytes */
#define HEAP_3D_QUEUE		0x77300u
#define HEAP_3D_QUEUE_STATE	0x77400u
#define HEAP_3D_STATUS		0x77500u
#define HEAP_3D_CMD_RING	0x77800u
#define HEAP_3D_RING		0x78000u	/* ring 1: fragment work ring */
#define HEAP_3D_POOL		0x7a000u	/* scheduling entries */
#define HEAP_PDS_PT		0x7c000u
#define HEAP_PDS_PAGE		0x84000u	/* GPU VA 0x300000000, 16 KiB */
#define HEAP_JOBS		0x80000u	/* fw VA 0x170000: per-job slots */
#define JOB_SLOTS		16u
#define JOB_STRIDE		0x1000u
#define JOB_CMD			0x000u		/* compute command, 0x2c8 bytes */
#define JOB_USEQ		0x400u		/* microsequence, 0x80 bytes */
#define JOB_CDM_ARGS		0x500u		/* CDM register arguments */
#define RING_ENTRIES		256u
#define WORK_ENTRY_SIZE		20u
#define HEAP_IO_RAM		0x320000u	/* RAM behind unused I/O entries */

/* P: I/O mapping table, 0x18-byte entries {pa lo, pa hi, fw va, size, flags, rw} */
#define P_IO_TABLE		0x640u
#define IO_ENTRY_SIZE		0x18u
#define IO_FENDER		0
#define IO_RGX			3
#define IO_ENTRIES		7

#define FENDER_PA		0x201d00000ull
#define FENDER_SIZE		0x5000u
#define RGX_PA			0x201000000ull
#define RGX_SIZE		0x20000u

static inline void wr32(u8 *base, u32 offset, u32 value)
{
	base[offset + 0] = value;
	base[offset + 1] = value >> 8;
	base[offset + 2] = value >> 16;
	base[offset + 3] = value >> 24;
}

static inline void wr16(u8 *base, u32 offset, u16 value)
{
	base[offset + 0] = value;
	base[offset + 1] = value >> 8;
}

static inline void wr64(u8 *base, u32 offset, u64 value)
{
	wr32(base, offset, (u32)value);
	wr32(base, offset + 4, (u32)(value >> 32));
}

/* Firmware VA of a heap offset, through descriptor 0 or 1 */
static inline u32 fw_va(u32 offset)
{
	if (offset >= HEAP2_OFFSET)
		return FW_VA_HEAP2 + (offset - HEAP2_OFFSET);
	return FW_VA_HEAP + offset;
}

/* Memory descriptor: {fw va, pfn, 0, u16 pages, u16 flags}; flags 0 = inner non-cacheable */
static void mem_desc(u8 *win, unsigned int index, u32 va, u64 pa, u16 pages)
{
	u32 entry = WIN_DESC + 16 * index;

	wr32(win, entry + 0x0, va);
	wr32(win, entry + 0x4, (u32)(pa >> 12));
	wr32(win, entry + 0x8, 0);
	wr16(win, entry + 0xc, pages);
	wr16(win, entry + 0xe, 0);
}

static void io_entry(u8 *heap, unsigned int index, u64 pa, u32 size)
{
	u32 entry = HEAP_P + P_IO_TABLE + IO_ENTRY_SIZE * index;

	wr64(heap, entry + 0x0, pa);
	wr32(heap, entry + 0xc, size);
	wr32(heap, entry + 0x10, 0);
	wr32(heap, entry + 0x14, 1);	/* device, read/write */
}

static void build_main_context(u8 *heap)
{
	static const u32 page_pointers[] = {
		0xe4, 0xe8, 0x134, 0x138, 0x13c, 0x140, 0x188,
	};
	unsigned int i;

	wr32(heap, HEAP_X + 0x0, fw_va(HEAP_P));
	wr32(heap, HEAP_X + 0x4, fw_va(HEAP_Q));
	wr32(heap, HEAP_X + 0x8, fw_va(0x7000));	/* Z */

	/* 13 rings: three index pointers each into the index page */
	for (i = 0; i < 13; i++) {
		wr32(heap, HEAP_X + 0xc + 0x10 * i, fw_va(HEAP_RING_INDICES + 0x10 * i));
		wr32(heap, HEAP_X + 0x10 + 0x10 * i, fw_va(HEAP_RING_INDICES + 0x10 * i + 4));
		wr32(heap, HEAP_X + 0x14 + 0x10 * i, fw_va(HEAP_RING_INDICES + 0x10 * i + 8));
	}

	/* Ring 2: compute work ring */
	wr32(heap, HEAP_X + 0x38, fw_va(HEAP_COMPUTE_RING));

	/* One zeroed page each, from fw VA 0x118000 on, in the order the
	 * firmware expects them. +0xec..+0xf8 are trace rings, set below. */
	for (i = 0; i < 2; i++)
		wr32(heap, HEAP_X + page_pointers[i], fw_va(0x18000 + 0x1000 * i));
	for (i = 2; i < 6; i++)
		wr32(heap, HEAP_X + page_pointers[i], fw_va(0x1c000 + 0x1000 * (i - 2)));
	wr32(heap, HEAP_X + 0x188, fw_va(0x20000));	/* acknowledged MMU generations */

	wr32(heap, HEAP_X + 0xd8, fw_va(0x27000));	/* control ring data */
	wr32(heap, HEAP_X + 0xdc, fw_va(HEAP_EVENT_RING));	/* event ring control */
	wr32(heap, HEAP_X + 0xe0, fw_va(HEAP_EVENT_RING + 0x1000));	/* event ring entries */
	wr32(heap, HEAP_X + 0xec, fw_va(HEAP_SKU_CONTROL));	/* SKU trace control */
	wr32(heap, HEAP_X + 0xf0, fw_va(HEAP_SKU_RECORDS));	/* SKU trace records */
	wr32(heap, HEAP_X + 0xf4, fw_va(0x68000));	/* firmware trace control */
	wr32(heap, HEAP_X + 0xf8, fw_va(0x64000));	/* firmware trace records */

	/*
	 * X+0x301 (unaligned): the firmware's power-state struct, written by the
	 * firmware (fw 0x6a98, 0x7de0, 0x838a). It needs pages of its own: at
	 * the old fw VA 0x130000 it overlapped the geometry queue.
	 */
	for (i = 0; i < 4; i++)
		heap[HEAP_X + 0x301 + i] = fw_va(HEAP_POWER_STATE) >> (8 * i);
}

static void build_device_context(u8 *heap, u64 heap_pa, u64 pds_base, u64 usc_base)
{
	unsigned int k;

	/* GPU address windows; the kext uses 0x300000000 for both code bases */
	wr64(heap, HEAP_P + 0x00, 0x600000000ull);
	wr64(heap, HEAP_P + 0x08, 0xffc00000ull);
	wr64(heap, HEAP_P + 0x10, pds_base);		/* PDS exec base */
	wr64(heap, HEAP_P + 0x18, usc_base);		/* USC code base */
	wr64(heap, HEAP_P + 0x20, 0x1ffff8000ull);

	io_entry(heap, IO_FENDER, FENDER_PA, FENDER_SIZE);
	io_entry(heap, IO_RGX, RGX_PA, RGX_SIZE);
	for (k = 1; k < IO_ENTRIES; k++) {
		if (k == IO_RGX)
			continue;
		io_entry(heap, k, heap_pa + HEAP_IO_RAM + 0x20000 * k, 0x20000);
	}

	/* One scheduling unit per TA/3D/compute engine */
	wr32(heap, HEAP_P + 0x16c0, 1);
	wr32(heap, HEAP_P + 0x16c4, 1);
	wr32(heap, HEAP_P + 0x16c8, 1);
}

static void build_gpu_mmu(u8 *heap, u64 gpu_root)
{
	/* Context-0 root array (T+0x7e0) and MMU generations (T+0x7dc) */
	wr32(heap, HEAP_T + 0x7dc, fw_va(HEAP_MMU_GEN));
	wr32(heap, HEAP_T + 0x7e0, fw_va(HEAP_MMU_ROOTS));
	wr64(heap, HEAP_MMU_ROOTS, gpu_root);
	wr32(heap, HEAP_MMU_GEN, 0x400000);	/* forces a full invalidation */
}

static void build_compute_queue(u8 *heap)
{
	/* Compute work ring descriptor 0 -> queue, engine 2 (compute), head 1 */
	wr32(heap, HEAP_COMPUTE_RING + 0x8, fw_va(HEAP_QUEUE));
	wr32(heap, HEAP_COMPUTE_RING + 0xc, 2);
	wr16(heap, HEAP_COMPUTE_RING + 0x10, 1);

	wr32(heap, HEAP_QUEUE + 0x00, fw_va(HEAP_QUEUE_STATE));
	wr32(heap, HEAP_QUEUE + 0x04, fw_va(HEAP_CMD_RING));
	wr32(heap, HEAP_QUEUE + 0x0c, FW_VA_HEAP2);	/* scheduling entries */
	wr32(heap, HEAP_QUEUE + 0x44, 1);		/* hardware unit mask */
	wr32(heap, HEAP_QUEUE + 0x88, fw_va(0x2e000));
	wr32(heap, HEAP_QUEUE_STATE + 0x50, 256);
	wr32(heap, HEAP_CMD_RING, fw_va(HEAP_COMPUTE_CMD));
}

/*
 * The compute command of the reference job: Start, WaitIRQ (compute finished),
 * Finalize, End, with a host completion stamp at fw VA 0x145004.
 */
/*
 * CDM register arguments: context/storage base (RGX+0x498), terminate PDS
 * code/data (the driver's HALT program), control stream base (RGX+0x480).
 */
static void write_cdm_args(u8 *heap, u32 args, u64 stream, u64 ctx_state)
{
	wr64(heap, args + 0x00, ctx_state);
	wr32(heap, args + 0x08, 0x201000);
	wr32(heap, args + 0x0c, 0x201010);
	wr64(heap, args + 0x10, stream);
}

/*
 * Microsequence of a compute job: StartCompute, WaitIRQ (compute finished),
 * FinalizeCompute, End. Finalize writes @seq to the first completion stamp.
 * @flag is the word StartCompute requires to be 0 and FinalizeCompute checks
 * before stopping the CDM; iOS 17 points both at its compute command +0x2c4.
 */
static void write_microsequence(u8 *heap, u32 useq, u32 cdm_args, u32 seq, u32 flag)
{
	/* StartCompute (0x48 bytes) */
	wr32(heap, useq + 0x00, 11);
	wr32(heap, useq + 0x04, fw_va(HEAP_CDM_REGS));
	wr32(heap, useq + 0x08, fw_va(cdm_args));
	wr32(heap, useq + 0x0c, fw_va(0x1e000));	/* shared unit states */
	wr32(heap, useq + 0x2c, fw_va(HEAP_CTX_STATE));
	wr32(heap, useq + 0x38, fw_va(flag));
	wr32(heap, useq + 0x44, fw_va(HEAP_UNIT_STATE + 0x108));
	/* WaitIRQ: event mask << 6 (compute finished: 0x4 << 6) */
	wr32(heap, useq + 0x48, 0x100);
	/* FinalizeCompute (0x30 bytes) */
	wr32(heap, useq + 0x4c, 12);
	wr32(heap, useq + 0x50, fw_va(0x1e000));
	wr32(heap, useq + 0x68, fw_va(HEAP_STAMP));
	wr32(heap, useq + 0x6c, seq);
	wr32(heap, useq + 0x74, fw_va(flag));
	/* End */
	wr32(heap, useq + 0x7c, 2);
}

static void write_compute_command(u8 *heap, u32 cmd, u32 useq, u32 seq)
{
	wr32(heap, cmd + 0x000, 3);
	wr32(heap, cmd + 0x014, fw_va(HEAP_CTX_STATE));
	wr32(heap, cmd + 0x1e8, fw_va(useq));
	wr32(heap, cmd + 0x1ec, 0x80);			/* microsequence size */
	wr32(heap, cmd + 0x26c, fw_va(HEAP_STAMP + 4));	/* host stamp address */
	wr32(heap, cmd + 0x274, seq);			/* stamp value */
}

static void build_compute_command(u8 *heap)
{
	/* Reference job 1 (the bring-up layout); the driver writes per-job slots */
	write_compute_command(heap, HEAP_COMPUTE_CMD, HEAP_USEQ, 1);
	write_microsequence(heap, HEAP_USEQ, HEAP_CDM_ARGS, 1, HEAP_UNIT_STATE + 0x100);

	wr32(heap, HEAP_UNIT_STATE + 0x108, 1);

	write_cdm_args(heap, HEAP_CDM_ARGS, 0x200000, 0x202000);
	/* Context store/load PDS program (RGX_CDM_CONTEXT_PDS0) */
	wr32(heap, HEAP_CTX_STATE + 0x38, 0x201000);
	wr32(heap, HEAP_CTX_STATE + 0x3c, 0x201010);

	/* Q: flush scratch/fence address, programmed into RGX+0xe240 */
	wr64(heap, HEAP_Q, 0x203000);
}

static void build_init_page(u8 *win, u64 win_pa, u64 heap_pa)
{
	wr64(win, WIN_INIT + 0x08, heap_pa + HEAP_SEQ_PROGRAM);	/* empty sequence */
	wr32(win, WIN_INIT + 0x18, fw_va(HEAP_X));
	wr32(win, WIN_INIT + 0x1c, fw_va(HEAP_T));
	wr64(win, WIN_INIT + 0x8c, win_pa + WIN_DESC);
	wr32(win, WIN_INIT + 0x94, 2);			/* descriptor count */
	wr32(win, WIN_INIT + 0x98, fw_va(HEAP_U));
	wr32(win, WIN_INIT + 0x9c, fw_va(HEAP_V));
	wr32(win, WIN_INIT + 0xa0, fw_va(0x10000));	/* power path page */

	mem_desc(win, 0, FW_VA_HEAP, heap_pa, 56);
	mem_desc(win, 1, FW_VA_HEAP2, heap_pa + HEAP2_OFFSET, 64);
}

void pvr_apple_layout_build(u8 *win, u64 win_pa, u8 *heap, u64 heap_pa, u64 gpu_root,
			    u64 pds_base, u64 usc_base)
{
	build_init_page(win, win_pa, heap_pa);
	build_main_context(heap);
	build_device_context(heap, heap_pa, pds_base, usc_base);
	build_gpu_mmu(heap, gpu_root);
	build_compute_queue(heap);
	build_compute_command(heap);

	/*
	 * Debug flags (fw global 0x3004c): bit 0 enables the SKU trace events,
	 * bit 1 the cancel-path ones, both into the SKU ring (X+0xec/+0xf0),
	 * which nothing drains. Off: pvr_apple_fw_start() sets them from the
	 * apple_fw_debug parameter.
	 */
	wr32(heap, HEAP_U + 0x0, 0);
	wr32(heap, HEAP_V + 0x455c, fw_va(0x24000));	/* mapping ring control */
	wr32(heap, HEAP_V + 0x4560, fw_va(0x25000));	/* mapping ring entries */
	wr32(heap, HEAP_V + 0x45b4, 1);			/* engine scheduler init */
}

/*
 * Queue compute job @seq (1, 2, ...): its slot, command-ring entry @seq - 1 and
 * work-ring descriptor. The caller then sets the producer index to @seq and
 * rings the doorbell; completion shows as @seq in the stamp.
 */
void pvr_apple_layout_write_compute_job(u8 *heap, u32 seq, u64 stream, u64 ctx_state)
{
	u32 slot = HEAP_JOBS + ((seq - 1) % JOB_SLOTS) * JOB_STRIDE;
	u32 work = HEAP_COMPUTE_RING + ((seq - 1) % RING_ENTRIES) * WORK_ENTRY_SIZE;
	u32 i;

	for (i = 0; i < JOB_STRIDE; i++)
		heap[slot + i] = 0;
	write_compute_command(heap, slot + JOB_CMD, slot + JOB_USEQ, seq);
	write_microsequence(heap, slot + JOB_USEQ, slot + JOB_CDM_ARGS, seq, slot + JOB_CMD + 0x2c4);
	write_cdm_args(heap, slot + JOB_CDM_ARGS, stream, ctx_state);

	wr32(heap, HEAP_CMD_RING + 4 * ((seq - 1) % RING_ENTRIES), fw_va(slot + JOB_CMD));
	for (i = 0; i < WORK_ENTRY_SIZE; i++)
		heap[work + i] = 0;
	wr32(heap, work + 0x8, fw_va(HEAP_QUEUE));
	wr32(heap, work + 0xc, 2);			/* engine: compute */
	wr16(heap, work + 0x10, seq % RING_ENTRIES);	/* command-ring head after it */
}

/*
 * The geometry (TA) queue: work ring 0, a queue in scheduling slot 1 and its
 * own command ring and scheduling entries. The firmware takes the engine from
 * the ring (0 geometry, 1 fragment, 2 compute; one triple per priority) and
 * dispatches command type 0, whose microsequence is at +0x464.
 */
void pvr_apple_layout_build_ta(u8 *heap)
{
	wr32(heap, HEAP_X + 0x18, fw_va(HEAP_TA_RING));

	wr32(heap, HEAP_TA_QUEUE + 0x00, fw_va(HEAP_TA_QUEUE_STATE));
	wr32(heap, HEAP_TA_QUEUE + 0x04, fw_va(HEAP_TA_CMD_RING));
	wr32(heap, HEAP_TA_QUEUE + 0x0c, fw_va(HEAP_TA_POOL));
	wr32(heap, HEAP_TA_QUEUE + 0x44, 1);		/* hardware unit mask */
	wr32(heap, HEAP_TA_QUEUE + 0x88, fw_va(HEAP_TA_STATUS));
	wr32(heap, HEAP_TA_QUEUE_STATE + 0x50, 256);
}

/* Scratch words of the geometry job */
#define TA_SKIP			0x00	/* StartTA w7: non-zero skips the job */
#define TA_RESUME		0x10	/* StartTA w16: 0 = fresh parameter state */
#define TA_RESUME2		0x20
#define TA_CANCEL		0x30	/* StartTA w24: cancel threshold {lo, hi} */
#define TA_CHANNEL		0x40	/* StartTA w6: channel record */
#define TA_STAMP		0x80	/* completion stamp the finalize step writes */
#define TA_SLOT			1	/* scheduling slot; compute uses 0 */

/*
 * Queue geometry job @seq: one parameter-manager slot, StartTA, wait for
 * TA_FINISHED / TA_TERMINATE / PM_OUT_OF_MEMORY, FinalizeTA, End. Finalize
 * writes @seq to the geometry stamp (pvr_apple_layout_ta_stamp_offset()).
 */
void pvr_apple_layout_write_ta_job(u8 *heap, u32 seq, const struct pvr_apple_ta_job *job)
{
	u32 cmd = HEAP_TA_CMD, useq = HEAP_TA_USEQ, sc = HEAP_TA_SCRATCH;
	u32 fl = HEAP_TA_FREE_LIST, slot = HEAP_TA_PM_SLOT;
	u32 work = HEAP_TA_RING + ((seq - 1) % RING_ENTRIES) * WORK_ENTRY_SIZE;
	u32 i;

	for (i = HEAP_TA_CMD; i < HEAP_TA_PM_HEADER + 0x100; i++)
		heap[i] = 0;
	for (i = 0; i < 8; i++)
		wr32(heap, HEAP_TA_REGS32 + 4 * i, job->regs32[i]);
	for (i = 0; i < 28; i++)
		wr64(heap, HEAP_TA_REGS64 + 8 * i, job->regs64[i]);
	wr32(heap, sc + TA_CANCEL, 0xffffffff);
	wr32(heap, sc + TA_CANCEL + 4, 0xffffffff);

	/*
	 * Free list, pre-filled: the kext starts with an empty stack (top ~0) and
	 * lets the firmware grow it from 128 KiB blocks; a full stack needs no
	 * growing. +0x18 is the stack base, +0x24/+0x40 bottom and top.
	 */
	wr64(heap, fl + 0x18, job->free_list);
	wr32(heap, fl + 0x20, job->free_list_pages * 4);
	wr32(heap, fl + 0x24, 0);
	wr32(heap, fl + 0x28, job->free_list_pages / 32);
	wr32(heap, fl + 0x3c, fw_va(HEAP_TA_PM_STATE));
	wr32(heap, fl + 0x40, job->free_list_pages - 1);
	wr32(heap, fl + 0x50, fw_va(HEAP_TA_PM_STATE + 0x40));

	/* Parameter-manager slot: macrotile list, macrotile array, VHEAP table */
	wr64(heap, slot + 0xa0, job->mlist);
	wr64(heap, slot + 0xa8, job->mtile_array);
	wr64(heap, slot + 0xb0, job->vheap_table);
	wr32(heap, slot + 0xb8, job->skip);
	wr32(heap, slot + 0xc0, fw_va(HEAP_TA_PM_HEADER + 0x40));

	/* Command: type 0, context id 0, free list 0 */
	wr32(heap, cmd + 0x000, 0);
	wr32(heap, cmd + 0x00c, 0);
	wr32(heap, cmd + 0x014, fw_va(HEAP_TA_CONTEXT));
	wr32(heap, cmd + 0x018, 0);
	wr32(heap, cmd + 0x020, fw_va(fl));
	wr32(heap, cmd + 0x024, fw_va(slot));
	wr32(heap, cmd + 0x464, fw_va(useq));
	wr32(heap, cmd + 0x468, 0xb0);
	/*
	 * Scheduling descriptor (compute: +0x26c): +0x558 token, whose bits
	 * 8-14 select the scheduling entry, and +0x55c the scheduling slot, which
	 * must match the work-ring entry's.
	 */
	wr32(heap, cmd + 0x550, fw_va(sc + TA_STAMP));
	wr32(heap, cmd + 0x558, seq & 0xff);
	heap[cmd + 0x55c] = TA_SLOT;
	wr32(heap, cmd + 0x568, seq);

	/* StartTA (26 words) */
	wr32(heap, useq + 0x00, 5);
	wr32(heap, useq + 0x04, fw_va(HEAP_TA_REGS32));
	wr32(heap, useq + 0x08, fw_va(HEAP_TA_REGS64));
	wr32(heap, useq + 0x0c, fw_va(fl));
	wr32(heap, useq + 0x10, fw_va(slot));
	wr32(heap, useq + 0x14, fw_va(HEAP_TA_UNITS) + 4);
	wr32(heap, useq + 0x18, fw_va(sc + TA_CHANNEL));
	wr32(heap, useq + 0x1c, fw_va(sc + TA_SKIP));
	wr32(heap, useq + 0x40, fw_va(sc + TA_RESUME));
	wr32(heap, useq + 0x44, fw_va(sc + TA_RESUME2));
	wr32(heap, useq + 0x50, seq);
	wr32(heap, useq + 0x60, fw_va(sc + TA_CANCEL));
	/* Wait: TA_FINISHED | TA_TERMINATE | PM_OUT_OF_MEMORY (event bits 5-7) */
	wr32(heap, useq + 0x68, 0xe0 << 6);
	/* FinalizeTA (15 words) */
	wr32(heap, useq + 0x6c, 6);
	wr32(heap, useq + 0x70, fw_va(slot));
	wr32(heap, useq + 0x74, fw_va(fl));
	wr32(heap, useq + 0x78, fw_va(HEAP_TA_UNITS) + 4);
	wr32(heap, useq + 0x7c, fw_va(sc + TA_CHANNEL));
	wr32(heap, useq + 0x80, fw_va(sc + TA_SKIP));
	wr32(heap, useq + 0x8c, fw_va(sc + TA_RESUME));
	wr32(heap, useq + 0x94, seq);
	wr32(heap, useq + 0x98, fw_va(HEAP_STAMP + 0x10));	/* stamp address */
	wr32(heap, useq + 0x9c, seq);				/* stamp value */
	wr32(heap, useq + 0xa0, 0x3c);				/* to End on a store */
	wr32(heap, useq + 0xa4, fw_va(HEAP_TA_PM_STATE + 0x80));
	/* End */
	wr32(heap, useq + 0xa8, 2);

	wr32(heap, HEAP_TA_CMD_RING + 4 * ((seq - 1) % RING_ENTRIES), fw_va(cmd));
	for (i = 0; i < WORK_ENTRY_SIZE; i++)
		heap[work + i] = 0;
	wr32(heap, work + 0x8, fw_va(HEAP_TA_QUEUE));
	wr32(heap, work + 0xc, 0);			/* engine: geometry */
	wr16(heap, work + 0x10, seq % RING_ENTRIES);	/* command-ring head after it */
	heap[work + 0x12] = TA_SLOT;			/* scheduling slot */
}

/* The fragment (3D) queue: work ring 1, scheduling slot 2, command type 1 */
void pvr_apple_layout_build_3d(u8 *heap)
{
	wr32(heap, HEAP_X + 0x28, fw_va(HEAP_3D_RING));

	wr32(heap, HEAP_3D_QUEUE + 0x00, fw_va(HEAP_3D_QUEUE_STATE));
	wr32(heap, HEAP_3D_QUEUE + 0x04, fw_va(HEAP_3D_CMD_RING));
	wr32(heap, HEAP_3D_QUEUE + 0x0c, fw_va(HEAP_3D_POOL));
	wr32(heap, HEAP_3D_QUEUE + 0x44, 1);		/* hardware unit mask */
	wr32(heap, HEAP_3D_QUEUE + 0x88, fw_va(HEAP_3D_STATUS));
	wr32(heap, HEAP_3D_QUEUE_STATE + 0x50, 256);
}

/* Scratch words of the fragment job (Start3D/Finalize3D pointer arguments) */
#define F_PARTIAL		0x00	/* Start3D w5, w6: partial render / resume */
#define F_PARTIAL2		0x10
#define F_SKIP			0x20	/* Start3D w7, Finalize3D w8 */
#define F_CHANNEL		0x30
#define F_WAIT			0x40	/* Start3D w18: non-zero defers the kick */
#define F_COUNTER		0x50	/* Finalize3D w5: {count, ..., +0x10, +0x14} */
#define F_CANCEL		0x70	/* Start3D w28: cancel threshold {lo, hi} */
#define F_STAMP			0x80
#define F_SLOT			2

/*
 * Queue fragment job @seq on the geometry job's parameter-manager slot:
 * Start3D, wait for PIXELBE_END_RENDER, Finalize3D, End. Finalize writes @seq
 * to the fragment stamp (pvr_apple_layout_3d_stamp_offset()). The command
 * holds its register blocks itself, at +0x68 (64-bit) and +0x3b0 (32-bit),
 * as the iPadOS kext lays it out.
 */
void pvr_apple_layout_write_3d_job(u8 *heap, u32 seq, const struct pvr_apple_3d_job *job)
{
	u32 cmd = HEAP_3D_CMD, useq = HEAP_3D_USEQ, sc = HEAP_3D_SCRATCH;
	u32 work = HEAP_3D_RING + ((seq - 1) % RING_ENTRIES) * WORK_ENTRY_SIZE;
	u32 units = fw_va(HEAP_3D_UNITS) + 8, slot = fw_va(HEAP_TA_PM_SLOT);
	u32 i;

	for (i = HEAP_3D_CMD; i < HEAP_3D_CONTEXT + 0x100; i++)
		heap[i] = 0;
	for (i = 0; i < 18; i++)
		wr32(heap, cmd + 0x3b0 + 4 * i, job->regs32[i]);
	for (i = 0; i < 29; i++)
		wr64(heap, cmd + 0x68 + 8 * i, job->regs64[i]);
	wr32(heap, sc + F_CANCEL, 0xffffffff);
	wr32(heap, sc + F_CANCEL + 4, 0xffffffff);

	/*
	 * Command: type 1, microsequence at +0x14, context at +0x1c; free list
	 * and parameter-manager slot at +0x20/+0x24, as geometry's, which the
	 * dispatcher reads when the job retires.
	 */
	wr32(heap, cmd + 0x000, 1);
	wr32(heap, cmd + 0x020, fw_va(HEAP_TA_FREE_LIST));
	wr32(heap, cmd + 0x024, fw_va(HEAP_TA_PM_SLOT));
	wr32(heap, cmd + 0x014, fw_va(useq));
	wr32(heap, cmd + 0x018, 0xc8);
	wr32(heap, cmd + 0x01c, fw_va(HEAP_3D_CONTEXT));
	/* Scheduling descriptor, as geometry's +0x550 block */
	wr32(heap, cmd + 0x89c, fw_va(sc + F_STAMP));
	wr32(heap, cmd + 0x8a4, seq & 0xff);
	heap[cmd + 0x8a8] = F_SLOT;
	wr32(heap, cmd + 0x8b4, seq);

	/* Start3D (29 words) */
	wr32(heap, useq + 0x00, 7);
	wr32(heap, useq + 0x04, fw_va(cmd + 0x3b0));
	wr32(heap, useq + 0x08, fw_va(cmd + 0x68));
	wr32(heap, useq + 0x0c, slot);
	wr32(heap, useq + 0x10, units);
	wr32(heap, useq + 0x14, fw_va(sc + F_PARTIAL));
	wr32(heap, useq + 0x18, fw_va(sc + F_PARTIAL2));
	wr32(heap, useq + 0x1c, fw_va(sc + F_SKIP));
	wr32(heap, useq + 0x20, fw_va(sc + F_CHANNEL));
	wr32(heap, useq + 0x24, fw_va(cmd));
	wr32(heap, useq + 0x48, fw_va(sc + F_WAIT));
	wr32(heap, useq + 0x4c, fw_va(cmd + 0x8c0));
	wr32(heap, useq + 0x64, seq);
	wr32(heap, useq + 0x70, fw_va(sc + F_CANCEL));
	/* Wait: PIXELBE_END_RENDER (event bit 3) */
	wr32(heap, useq + 0x74, 0x8 << 6);
	/* Finalize3D (19 words) */
	wr32(heap, useq + 0x78, 8);
	wr32(heap, useq + 0x7c, seq);
	wr32(heap, useq + 0x80, fw_va(HEAP_STAMP + 0x20));	/* stamp address */
	wr32(heap, useq + 0x84, seq);				/* stamp value */
	wr32(heap, useq + 0x88, slot);
	wr32(heap, useq + 0x8c, fw_va(sc + F_COUNTER));
	wr32(heap, useq + 0x90, 1);
	wr32(heap, useq + 0x94, units);
	wr32(heap, useq + 0x98, fw_va(sc + F_SKIP));
	wr32(heap, useq + 0x9c, fw_va(sc + F_PARTIAL));
	wr32(heap, useq + 0xa0, fw_va(sc + F_CHANNEL));
	wr32(heap, useq + 0xa4, fw_va(cmd));
	wr32(heap, useq + 0xac, fw_va(cmd + 0x8c0));
	wr32(heap, useq + 0xbc, 0x4c);				/* to End */
	wr32(heap, useq + 0xc0, fw_va(cmd + 0x8f0));
	/* End */
	wr32(heap, useq + 0xc4, 2);

	wr32(heap, HEAP_3D_CMD_RING + 4 * ((seq - 1) % RING_ENTRIES), fw_va(cmd));
	for (i = 0; i < WORK_ENTRY_SIZE; i++)
		heap[work + i] = 0;
	wr32(heap, work + 0x8, fw_va(HEAP_3D_QUEUE));
	wr32(heap, work + 0xc, 1);			/* engine: fragment */
	wr16(heap, work + 0x10, seq % RING_ENTRIES);
	heap[work + 0x12] = F_SLOT;
}

/*
 * The producer index of a work ring after job @seq: ring indices wrap at the
 * ring size (the firmware's own indices do), sequence numbers do not.
 */
u32 pvr_apple_layout_ring_index(u32 seq)
{
	return seq % RING_ENTRIES;
}

/* Work ring @ring's index words: two the firmware advances, then the producer */
u32 pvr_apple_layout_ring_indices_offset(unsigned int ring)
{
	return HEAP_RING_INDICES + 0x10 * ring;
}

u32 pvr_apple_layout_3d_producer(void)
{
	return HEAP_RING_INDICES + 0x18;	/* ring 1 producer index */
}

/*
 * The firmware's event ring: it posts an event per completion at the
 * producer index (+0x20) and spins while the ring is full, i.e. while its
 * next index equals the host's consumer index (+0x00).
 */
/* Event @i (0-255) of the firmware's event ring: 0x24 bytes */
u32 pvr_apple_layout_event_entry(unsigned int i)
{
	return HEAP_EVENT_RING + 0x1000 + 0x24 * (i % 256);
}

u32 pvr_apple_layout_debug_flags_offset(void)
{
	return HEAP_U;
}

/* The SKU trace ring: index of the next record, and record @i (36 bytes) */
u32 pvr_apple_layout_sku_index(void)
{
	return HEAP_SKU_CONTROL + 0x20;
}

u32 pvr_apple_layout_sku_record(unsigned int i)
{
	return HEAP_SKU_RECORDS + 36 * (i % 512);
}

u32 pvr_apple_layout_event_consumer(void)
{
	return HEAP_EVENT_RING;
}

u32 pvr_apple_layout_event_producer(void)
{
	return HEAP_EVENT_RING + 0x20;
}

u32 pvr_apple_layout_3d_stamp_offset(void)
{
	return HEAP_STAMP + 0x20;
}

u32 pvr_apple_layout_ta_producer(void)
{
	return HEAP_RING_INDICES + 0x08;	/* ring 0 producer index */
}

u32 pvr_apple_layout_ta_stamp_offset(void)
{
	return HEAP_STAMP + 0x10;
}

u32 pvr_apple_layout_ta_units_offset(void)
{
	return HEAP_TA_UNITS;
}

u32 pvr_apple_layout_ta_pm_slot_offset(void)
{
	return HEAP_TA_PM_SLOT;
}

/* The parameter manager's state block the free list points at */
u32 pvr_apple_layout_ta_pm_state_offset(void)
{
	return HEAP_TA_PM_STATE;
}

u32 pvr_apple_layout_ta_free_list_offset(void)
{
	return HEAP_TA_FREE_LIST;
}

u32 pvr_apple_layout_mmu_root_offset(unsigned int ctx)
{
	return HEAP_MMU_ROOTS + 8 * ctx;
}

u32 pvr_apple_layout_mmu_gen_offset(unsigned int ctx)
{
	return HEAP_MMU_GEN + 4 * ctx;
}

u32 pvr_apple_layout_init_offset(void)
{
	return WIN_INIT;
}

u32 pvr_apple_layout_compute_producer(void)
{
	return HEAP_RING_INDICES + 0x28;	/* ring 2 producer index */
}

u32 pvr_apple_layout_stamp_offset(void)
{
	return HEAP_STAMP + 4;
}
