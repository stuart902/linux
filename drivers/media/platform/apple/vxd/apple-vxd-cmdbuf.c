// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - command buffers
 *
 * The pool of eight 256 KiB GART-mapped command buffers with their
 * host-only packet lists and the packet -> mailbox message arithmetic.
 * The encoders themselves are in apple-vxd-cmd.c.
 */
#include <linux/bitfield.h>
#include <linux/slab.h>

#include "apple-vxd.h"

/*
 * Decode message word 3.  The kext builds it from the GART directory
 * descriptor (+0x27a8, the object it also programs into MTX_MMU_DIR_ADR
 * 0x1020): (u32)(dir >> 8), low byte replaced by the hwContextID
 * (0xfffffff00628ae5c: lsr x8, x0, #8; str w8; strb w21).  It is the MMU
 * page-table context, not the command buffer's address; the 64 KiB
 * directory alignment is what keeps the replaced byte zero.
 */
u32 apple_vxd_mmu_ctx_word(struct apple_vxd_dev *vxd, u8 hw_ctx)
{
	return ((u32)(vxd->gart.dir_dma >> 8) & 0xffffff00) | hw_ctx;
}

/*
 * One mailbox message from one packet, SubmitCommandBufferInternal's
 * arithmetic for the three kinds AppleD5500 produces [E]:
 *
 *   kind 1 -> 0x81: w1 = (bytes >> 2) << 16 | flags | (last ? 0x4004 : 0x2000),
 *                   the flag OR-ed back into the packet as the kext does
 *   kind 3 -> 0x87: w1 = (bytes >> 2) << 16 | flags,
 *                   w4 = (payload +0x08 & ~0x6000) | 0x4000
 *   kind 4 -> 0x88: w1 = (bytes >> 3) << 16 | flags | (last ? 0x4004 : 0)
 *   all:            w2 = payload +0x10, w3 = MMU context word,
 *                   w4 = payload +0x08, w5 = payload +0x24
 *
 * Six-word form only (the kext adds w6 = payload +0x28 when the core has
 * a second pipe flag set; this core does not).  @pkt points at the packet
 * header.  Before pushing a 0x81 the submit path must rewrite 0x1008 =
 * 0xf00 if gart.dirty (the kext's +0x27a0 rule).  Returns the message type
 * or -EINVAL.
 */
int apple_vxd_packet_msg(struct apple_vxd_dev *vxd, u32 *pkt, u16 fence,
			 u8 hw_ctx, bool last, u32 msg[VXD_DECODE_MSG_WORDS])
{
	u32 *p = &pkt[VXD_PKT_HDR_BYTES / 4];
	u32 flags, words, w4;
	u8 type;

	if (pkt[0] != VXD_PKT_MAGIC0 || pkt[2] != VXD_PKT_MAGIC1 ||
	    pkt[1] < VXD_PKT_H264_PAYLOAD)
		return -EINVAL;

	w4 = p[VXD_PKT_W4 / 4];
	switch (p[VXD_PKT_KIND / 4]) {
	case VXD_PKT_KIND_PICTURE:
		type = VXD_MSG_DECODE;
		words = p[VXD_PKT_CMD_BYTES / 4] >> 2;
		p[VXD_PKT_FLAGS / 4] |= last ? VXD_MSG_FLAG_LAST : VXD_MSG_FLAG_MORE;
		flags = p[VXD_PKT_FLAGS / 4];
		break;
	case VXD_PKT_KIND_DEBLOCK:
		type = VXD_MSG_DEBLOCK;
		words = p[VXD_PKT_CMD_BYTES / 4] >> 2;
		flags = p[VXD_PKT_FLAGS / 4];
		w4 = (w4 & ~0x6000) | 0x4000;
		break;
	case VXD_PKT_KIND_RENDER_TARGET:
		type = VXD_MSG_RENDER_TARGET;
		words = p[VXD_PKT_CMD_BYTES / 4] >> 3;
		flags = p[VXD_PKT_FLAGS / 4] | (last ? VXD_MSG_FLAG_LAST : 0);
		break;
	default:
		return -EINVAL;
	}

	msg[0] = apple_vxd_msg_hdr(fence, type, VXD_DECODE_MSG_WORDS);
	msg[1] = (words << 16) | (flags & 0xffff);
	msg[2] = p[VXD_PKT_W2 / 4];
	msg[3] = apple_vxd_mmu_ctx_word(vxd, hw_ctx);
	msg[4] = w4;
	msg[5] = p[VXD_PKT_W5 / 4];
	return type;
}

/* --- pool ---------------------------------------------------------------- */

struct apple_vxd_cmdbuf *apple_vxd_cmdbuf_get(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_cmdbuf_pool *pool = &vxd->cmdbufs;
	struct apple_vxd_cmdbuf *cb = NULL;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	for (i = 0; pool->ready && i < APPLE_VXD_NUM_CMDBUFS; i++) {
		if (pool->bufs[i].state == APPLE_VXD_CMDBUF_FREE) {
			cb = &pool->bufs[i];
			cb->state = APPLE_VXD_CMDBUF_HOST;
			break;
		}
	}
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	if (cb)
		apple_vxd_cmdbuf_reset(cb);
	return cb;
}

/* HOST -> FREE: a buffer that was built but never submitted. */
void apple_vxd_cmdbuf_put(struct apple_vxd_dev *vxd,
			  struct apple_vxd_cmdbuf *cb)
{
	unsigned long flags;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	WARN_ON(cb->state != APPLE_VXD_CMDBUF_HOST);
	cb->state = APPLE_VXD_CMDBUF_FREE;
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
}

/*
 * HOST -> DEVICE, before the message that @fence tags is pushed: from now
 * on the core may read the buffer, and only that fence's completion (or a
 * reclaim after the firmware is stopped) gives it back.
 */
int apple_vxd_cmdbuf_bind(struct apple_vxd_dev *vxd,
			  struct apple_vxd_cmdbuf *cb,
			  struct apple_vxd_fence *fence)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	if (cb->state != APPLE_VXD_CMDBUF_HOST || cb->error ||
	    fence->state != APPLE_VXD_FENCE_ARMED || fence->cmdbuf) {
		ret = -EINVAL;
	} else {
		cb->state = APPLE_VXD_CMDBUF_DEVICE;
		cb->fence = fence;
		fence->cmdbuf = cb;
	}
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	return ret;
}

/* DEVICE -> FREE on its fence's 0xc0.  mbox_lock held. */
void apple_vxd_cmdbuf_retire_locked(struct apple_vxd_cmdbuf *cb)
{
	cb->state = APPLE_VXD_CMDBUF_FREE;
	cb->fence = NULL;
}

/* The firmware is stopped: nothing reads any buffer any more. */
void apple_vxd_cmdbufs_reclaim(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_cmdbuf_pool *pool = &vxd->cmdbufs;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	for (i = 0; i < APPLE_VXD_NUM_CMDBUFS; i++) {
		struct apple_vxd_cmdbuf *cb = &pool->bufs[i];

		if (cb->state == APPLE_VXD_CMDBUF_DEVICE) {
			if (cb->fence)
				cb->fence->cmdbuf = NULL;
			apple_vxd_cmdbuf_retire_locked(cb);
		}
	}
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
}

/*
 * Boot path, before the MTX starts: allocate and GART-map the eight
 * buffers once, so the core's MMU never sees them change under it.
 */
int apple_vxd_cmdbufs_init(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_cmdbuf_pool *pool = &vxd->cmdbufs;
	size_t total = (size_t)APPLE_VXD_NUM_CMDBUFS * APPLE_VXD_CMDBUF_SIZE;
	unsigned int i;
	int ret;

	ret = apple_vxd_dma_alloc_aligned(vxd, total, APPLE_VXD_CMDBUF_ALIGN,
					  &pool->mem);
	if (ret)
		return ret;

	ret = apple_vxd_gart_map(vxd, pool->mem.dma, total, &pool->map);
	if (ret)
		goto err_free;
	if (!IS_ALIGNED(pool->map.va, APPLE_VXD_CMDBUF_ALIGN)) {
		dev_err(vxd->dev,
			"command buffers at device VA %#x are not 64 KiB aligned\n",
			pool->map.va);
		ret = -EINVAL;
		goto err_unmap;
	}

	pool->pkt_mem = kvcalloc(APPLE_VXD_NUM_CMDBUFS, APPLE_VXD_PKTLIST_SIZE,
				 GFP_KERNEL);
	if (!pool->pkt_mem) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	for (i = 0; i < APPLE_VXD_NUM_CMDBUFS; i++) {
		struct apple_vxd_cmdbuf *cb = &pool->bufs[i];
		size_t off = (size_t)i * APPLE_VXD_CMDBUF_SIZE;

		cb->index = i;
		cb->state = APPLE_VXD_CMDBUF_FREE;
		cb->vaddr = pool->mem.vaddr + off;
		cb->dma = pool->mem.dma + off;
		cb->va = pool->map.va + off;
		cb->size = APPLE_VXD_CMDBUF_SIZE;
		cb->pkt = pool->pkt_mem + (size_t)i * APPLE_VXD_PKTLIST_SIZE;
		cb->pkt_size = APPLE_VXD_PKTLIST_SIZE;
		apple_vxd_cmdbuf_reset(cb);
	}
	pool->ready = true;

	dev_dbg(vxd->dev,
		"command buffers: %u x %u KiB at %pad, device VA %#x..%#x\n",
		APPLE_VXD_NUM_CMDBUFS, APPLE_VXD_CMDBUF_SIZE / SZ_1K,
		&pool->mem.dma, pool->map.va, pool->map.va + (u32)total - 1);
	return 0;

err_unmap:
	apple_vxd_gart_unmap(vxd, &pool->map);
err_free:
	apple_vxd_dma_free_aligned(vxd, &pool->mem);
	memset(pool, 0, sizeof(*pool));
	return ret;
}

/* After the mailbox is stopped. */
void apple_vxd_cmdbufs_fini(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_cmdbuf_pool *pool = &vxd->cmdbufs;
	unsigned long flags;
	unsigned int i;

	if (!pool->mem.raw_vaddr)
		return;

	apple_vxd_cmdbufs_reclaim(vxd);
	spin_lock_irqsave(&vxd->mbox_lock, flags);
	pool->ready = false;
	for (i = 0; i < APPLE_VXD_NUM_CMDBUFS; i++)
		WARN_ON(pool->bufs[i].state != APPLE_VXD_CMDBUF_FREE);
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	apple_vxd_gart_unmap(vxd, &pool->map);
	kvfree(pool->pkt_mem);
	apple_vxd_dma_free_aligned(vxd, &pool->mem);
	memset(pool, 0, sizeof(*pool));
}
