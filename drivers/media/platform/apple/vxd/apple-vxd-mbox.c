// SPDX-License-Identifier: GPL-2.0
/*
 * Apple VXD - host<->MTX mailbox
 *
 * docs/periph/ds-vxd-decode-protocol.md sections 2, 3 and 6.  The rings
 * are word windows in the register file, not DMA queues: after the ready
 * magic the DEVA firmware publishes each ring as (base << 16) | capacity
 * in 0x2024 (TO_MTX) and 0x2018 (TO_HOST), and word i of a ring is the
 * register at 0x2000 + base + 4 * i.
 *
 *   push (0xfffffff0062990bc):  words, 0x2030 = wr, flush reads of
 *                               0x2028/0x2030/0x201c/0x2020, 0x88 = 1,
 *                               flush read of 0x2000
 *   ISR  (0xfffffff00629a0ac):  read 0x410, 0x414 = 0x100000, then drain
 *                               TO_HOST (0x201c host rd, 0x2020 fw wr)
 *   init (0xfffffff006298d40):  one 0x80 message after the ready magic
 *
 * The firmware answers decode packets with a 0xc0 completion carrying the
 * fence; it answers nothing else.  0x80 and the benign 0x83..0x85 are
 * acknowledged only by TO_MTX_RD (0x2028) moving past them: the DEVA
 * reader publishes the new read index before it dispatches (fw 0x1000a9c).
 * That is what the boot waits for.
 *
 * Every ring access is bounded: indices are checked against the published
 * capacity, the geometry against the MMIO window, message lengths against
 * both sides' limits, and every wait has a timeout.
 */
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>

#include "apple-vxd.h"

#define VXD_MSG_CONSUME_TIMEOUT_US	100000	/* init message */
#define VXD_MSG_PUSH_TIMEOUT_US		100000	/* free-slot wait */
#define VXD_DRAIN_MAX_MSGS		0x70	/* the kext's bound */
#define VXD_RING_MAX_CAP		0x4000	/* sanity bound, words */
/*
 * The firmware raises bit 20 only after posting a message, so an interrupt
 * normally drains at least one.  This many in a row with nothing to drain
 * means 0x414 does not clear the level line: stop taking it rather than
 * livelock the CPU.
 */
#define VXD_IRQ_STORM_LIMIT		1000

/* --- ring arithmetic ---------------------------------------------------- */

static u32 vxd_ring_next(u32 idx, u32 cap)
{
	return idx + 1 < cap ? idx + 1 : 0;
}

/* Words from @from forward to @to. */
static u32 vxd_ring_dist(u32 from, u32 to, u32 cap)
{
	return to >= from ? to - from : to + cap - from;
}

/*
 * Free TO_MTX words, AppleD5500's helper (0xfffffff0062996b4 [E]):
 * rd + ~wr, plus the capacity unless wr < rd.  One word always stays free,
 * so rd == wr means empty.
 */
static u32 vxd_ring_free(u32 rd, u32 wr, u32 cap)
{
	if (wr < rd)
		return rd - wr - 1;
	return rd - wr - 1 + cap;
}

/*
 * The reader has taken the @t message once it moved at least nwords past
 * its start without overtaking the writer.  A read index still behind the
 * start (earlier messages pending) is "not yet".
 */
static bool vxd_ring_consumed(const struct apple_vxd_msg_ticket *t, u32 rd,
			      u32 wr, u32 cap)
{
	u32 done = vxd_ring_dist(t->start, rd, cap);

	return done >= t->nwords && done <= vxd_ring_dist(t->start, wr, cap);
}

/* Apple's fence: (submission << 4) | (packet * 2 & 0xe) | core, core 0. */
static u16 vxd_fence_id(u16 seq, unsigned int packet)
{
	return (seq << 4) | ((packet * 2) & 0xe);
}

/*
 * InitMessageBuffers [E]: w1 = firmware device address (the 0x6a00 value),
 * w2 = (u8)seq << 16 | image size in 4 KiB pages (the blob, not the
 * page-aligned allocation: 0x17e98 bytes -> 0x17), w3 = 0xffff0000.
 */
static void apple_vxd_build_init_msg(u32 msg[VXD_INIT_MSG_WORDS], u32 fw_va,
				     size_t fw_bytes, u16 seq)
{
	msg[0] = apple_vxd_msg_hdr(0, VXD_MSG_INIT, VXD_INIT_MSG_WORDS);
	msg[1] = fw_va;
	msg[2] = FIELD_PREP(VXD_INIT_SEQ, seq & 0xff) |
		 FIELD_PREP(VXD_INIT_PAGES, fw_bytes >> 12);
	msg[3] = VXD_INIT_SENTINEL;
	msg[4] = 0;
}

/* --- geometry ---------------------------------------------------------- */

static int vxd_ring_parse(struct apple_vxd_dev *vxd, const char *name,
			  u32 raw, struct apple_vxd_ring *ring)
{
	ring->base = FIELD_GET(VXD_RING_BASE, raw);
	ring->cap = FIELD_GET(VXD_RING_CAP, raw);

	if (ring->cap <= VXD_MSG_TO_MTX_MAX_WORDS ||
	    ring->cap > VXD_RING_MAX_CAP || ring->base & 3 ||
	    VXD_RING_WINDOW + ring->base + 4 * ring->cap > vxd->regs_size) {
		dev_err(vxd->dev,
			"%s ring geometry %#010x (base %#x, %u words) is not usable: needs %u < words <= %u, a 4-byte aligned base and the ring inside the %#llx-byte register window\n",
			name, raw, ring->base, ring->cap,
			VXD_MSG_TO_MTX_MAX_WORDS, VXD_RING_MAX_CAP,
			(unsigned long long)vxd->regs_size);
		return -EINVAL;
	}
	return 0;
}

static u32 vxd_ring_reg(const struct apple_vxd_ring *ring, u32 idx)
{
	return VXD_RING_WINDOW + ring->base + 4 * idx;
}

/*
 * Read the geometry the firmware published with the ready magic, once per
 * boot.  Called by the boot under power_lock.
 */
int apple_vxd_mbox_open(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_ring to_mtx, to_host;
	u32 raw_mtx, raw_host;
	unsigned long flags;
	unsigned int i;
	int ret;

	raw_mtx = vxd_read(vxd, VXD_REG_TO_MTX_RING);
	raw_host = vxd_read(vxd, VXD_REG_TO_HOST_RING);

	ret = vxd_ring_parse(vxd, "TO_MTX", raw_mtx, &to_mtx);
	if (!ret)
		ret = vxd_ring_parse(vxd, "TO_HOST", raw_host, &to_host);
	if (ret)
		return ret;

	if (to_mtx.base < to_host.base + 4 * to_host.cap &&
	    to_host.base < to_mtx.base + 4 * to_mtx.cap) {
		dev_err(vxd->dev, "TO_MTX and TO_HOST rings overlap\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	vxd->to_mtx = to_mtx;
	vxd->to_host = to_host;
	for (i = 0; i < APPLE_VXD_NUM_FENCES; i++)
		vxd->fences[i].state = APPLE_VXD_FENCE_FREE;
	vxd->fw_panicked = false;
	vxd->irq_storm = false;
	vxd->irq_empty = 0;
	vxd->mbox_ready = true;
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	dev_dbg(vxd->dev,
		"mailbox: TO_MTX regs %#x..%#x (%u words), TO_HOST regs %#x..%#x (%u words)\n",
		vxd_ring_reg(&to_mtx, 0), vxd_ring_reg(&to_mtx, to_mtx.cap) - 1,
		to_mtx.cap, vxd_ring_reg(&to_host, 0),
		vxd_ring_reg(&to_host, to_host.cap) - 1, to_host.cap);
	return 0;
}

/* --- push -------------------------------------------------------------- */

/*
 * Push one message to TO_MTX in AppleD5500's exact order, waiting (bounded)
 * for room.  Process context only.  @ticket records where it went so the
 * caller can wait for the firmware to take it.
 */
int apple_vxd_msg_push(struct apple_vxd_dev *vxd, const u32 *msg,
		       struct apple_vxd_msg_ticket *ticket)
{
	u32 nwords = FIELD_GET(VXD_MSG_NWORDS, msg[0]);
	u32 rd, wr, cap, i;
	unsigned long flags;
	ktime_t deadline;

	might_sleep();
	if (!nwords || nwords > VXD_MSG_TO_MTX_MAX_WORDS || msg[0] & BIT(7))
		return -EINVAL;

	deadline = ktime_add_us(ktime_get(), VXD_MSG_PUSH_TIMEOUT_US);
	for (;;) {
		spin_lock_irqsave(&vxd->mbox_lock, flags);
		if (!vxd->mbox_ready) {
			spin_unlock_irqrestore(&vxd->mbox_lock, flags);
			return -ENODEV;
		}

		cap = vxd->to_mtx.cap;
		rd = vxd_read(vxd, VXD_REG_TO_MTX_RD_INDEX);
		wr = vxd_read(vxd, VXD_REG_TO_MTX_WR_INDEX);
		if (rd >= cap || wr >= cap) {
			spin_unlock_irqrestore(&vxd->mbox_lock, flags);
			dev_err(vxd->dev,
				"TO_MTX indices out of range: rd %u wr %u (capacity %u)\n",
				rd, wr, cap);
			return -EIO;
		}
		if (vxd_ring_free(rd, wr, cap) >= nwords)
			break;
		spin_unlock_irqrestore(&vxd->mbox_lock, flags);

		if (ktime_after(ktime_get(), deadline)) {
			dev_err(vxd->dev,
				"TO_MTX full for 100 ms: rd %u wr %u, %u words free, %u needed\n",
				rd, wr, vxd_ring_free(rd, wr, cap), nwords);
			return -EBUSY;
		}
		usleep_range(100, 200);
	}

	ticket->start = wr;
	ticket->nwords = nwords;
	for (i = 0; i < nwords; i++) {
		vxd_write(vxd, vxd_ring_reg(&vxd->to_mtx, wr), msg[i]);
		wr = vxd_ring_next(wr, cap);
	}
	ticket->end = wr;
	vxd_write(vxd, VXD_REG_TO_MTX_WR_INDEX, wr);

	/* APB flush reads, doorbell, final flush: the kext's sequence. */
	vxd_read(vxd, VXD_REG_TO_MTX_RD_INDEX);
	vxd_read(vxd, VXD_REG_TO_MTX_WR_INDEX);
	vxd_read(vxd, VXD_REG_TO_HOST_RD_INDEX);
	vxd_read(vxd, VXD_REG_TO_HOST_WR_INDEX);
	vxd_write(vxd, VXD_REG_MTX_DOORBELL, 1);
	vxd_read(vxd, VXD_REG_COMMS_STATE);
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	return 0;
}

static bool vxd_msg_is_consumed(struct apple_vxd_dev *vxd,
				const struct apple_vxd_msg_ticket *ticket)
{
	u32 rd = vxd_read(vxd, VXD_REG_TO_MTX_RD_INDEX);
	u32 wr = vxd_read(vxd, VXD_REG_TO_MTX_WR_INDEX);

	if (rd >= vxd->to_mtx.cap || wr >= vxd->to_mtx.cap)
		return false;
	return vxd_ring_consumed(ticket, rd, wr, vxd->to_mtx.cap);
}

/* Poll 0x2028 until the firmware has taken the message (bounded). */
int apple_vxd_msg_wait_consumed(struct apple_vxd_dev *vxd,
				const struct apple_vxd_msg_ticket *ticket,
				unsigned int timeout_us, unsigned int *waited_us)
{
	ktime_t start = ktime_get();
	bool done;
	int ret;

	ret = read_poll_timeout(vxd_msg_is_consumed, done, done, 50,
				timeout_us, false, vxd, ticket);
	if (waited_us)
		*waited_us = ktime_us_delta(ktime_get(), start);
	return ret;
}

/* --- fences ------------------------------------------------------------ */

/* 12-bit submission counter, never 0: fence 0 is the init message's.  mbox_lock held. */
static u16 vxd_fence_next_seq_locked(struct apple_vxd_dev *vxd)
{
	vxd->fence_seq = (vxd->fence_seq + 1) & 0xfff;
	if (!vxd->fence_seq)
		vxd->fence_seq = 1;
	return vxd->fence_seq;
}

/* mbox_lock held. */
static struct apple_vxd_fence *vxd_fence_arm_locked(struct apple_vxd_dev *vxd,
						    u16 id, u8 type)
{
	unsigned int i;

	for (i = 0; i < APPLE_VXD_NUM_FENCES; i++) {
		struct apple_vxd_fence *fence = &vxd->fences[i];

		if (fence->state != APPLE_VXD_FENCE_FREE)
			continue;
		fence->id = id;
		fence->type = type;
		fence->status = 0;
		fence->error = false;
		fence->job = false;
		fence->cmdbuf = NULL;
		fence->state = APPLE_VXD_FENCE_ARMED;
		reinit_completion(&fence->done);
		return fence;
	}
	return NULL;
}

struct apple_vxd_fence *apple_vxd_fence_arm(struct apple_vxd_dev *vxd,
					    unsigned int packet, u8 type)
{
	struct apple_vxd_fence *fence;
	unsigned long flags;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	fence = vxd_fence_arm_locked(vxd,
				     vxd_fence_id(vxd_fence_next_seq_locked(vxd),
						  packet), type);
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);

	return fence;
}

void apple_vxd_fence_release(struct apple_vxd_dev *vxd,
			     struct apple_vxd_fence *fence)
{
	unsigned long flags;

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	/* Never completed: its command buffer stays with the device. */
	if (fence->cmdbuf) {
		fence->cmdbuf->fence = NULL;
		fence->cmdbuf = NULL;
	}
	fence->job = false;
	fence->state = APPLE_VXD_FENCE_FREE;
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
}

/*
 * Called with mbox_lock held.  A real 0xc0 (@device_done) means the core
 * is finished with the bound command buffer, whatever the status; a panic
 * or a teardown wakes the waiter but leaves the buffer quarantined until
 * apple_vxd_cmdbufs_reclaim().
 */
static void vxd_fence_complete(struct apple_vxd_dev *vxd,
			       struct apple_vxd_fence *fence, u32 status,
			       bool error, bool device_done)
{
	struct apple_vxd_cmdbuf *cb = fence->cmdbuf;

	if (cb) {
		if (device_done)
			apple_vxd_cmdbuf_retire_locked(cb);
		else
			cb->fence = NULL;
		fence->cmdbuf = NULL;
	}
	fence->status = status;
	fence->error = error;
	fence->state = APPLE_VXD_FENCE_SIGNALLED;
	complete_all(&fence->done);
	/* The job is finished in process context (v4l2-m2m, runtime PM). */
	if (fence->job)
		queue_work(system_highpri_wq, &vxd->job_done_work);
}

/* Called with mbox_lock held: wake every waiter with an error. */
static void vxd_fences_fail_all(struct apple_vxd_dev *vxd)
{
	unsigned int i;

	for (i = 0; i < APPLE_VXD_NUM_FENCES; i++)
		if (vxd->fences[i].state == APPLE_VXD_FENCE_ARMED)
			vxd_fence_complete(vxd, &vxd->fences[i], 0, true, false);
}

/* --- drain ------------------------------------------------------------- */

/*
 * Pop one TO_HOST message (the kext's 0xfffffff0062997b0, consuming mode).
 * mbox_lock held.  Returns the word count, 0 if the ring is empty, or
 * -EIO if it is corrupt.
 */
static int vxd_msg_pop(struct apple_vxd_dev *vxd, u32 *msg)
{
	u32 cap = vxd->to_host.cap;
	u32 rd = vxd_read(vxd, VXD_REG_TO_HOST_RD_INDEX);
	u32 wr = vxd_read(vxd, VXD_REG_TO_HOST_WR_INDEX);
	u32 n, i;

	if (rd >= cap || wr >= cap) {
		dev_err_ratelimited(vxd->dev,
				    "TO_HOST indices out of range: rd %u wr %u (capacity %u)\n",
				    rd, wr, cap);
		return -EIO;
	}
	if (rd == wr)
		return 0;

	msg[0] = vxd_read(vxd, vxd_ring_reg(&vxd->to_host, rd));
	n = FIELD_GET(VXD_MSG_NWORDS, msg[0]);
	if (n > VXD_MSG_TO_HOST_MAX_WORDS || n > vxd_ring_dist(rd, wr, cap)) {
		/*
		 * The kext logs "received oversize fw->host msg" and stops;
		 * resync to the writer so one bad header cannot pin the ring.
		 */
		dev_err_ratelimited(vxd->dev,
				    "TO_HOST message %#010x at %u claims %u words (%u queued); dropping the queue\n",
				    msg[0], rd, n, vxd_ring_dist(rd, wr, cap));
		vxd_write(vxd, VXD_REG_TO_HOST_RD_INDEX, wr);
		return -EIO;
	}
	if (!n) {
		/* The kext consumes a zero-length header as one word. */
		n = 1;
	}

	rd = vxd_ring_next(rd, cap);
	for (i = 1; i < n; i++) {
		msg[i] = vxd_read(vxd, vxd_ring_reg(&vxd->to_host, rd));
		rd = vxd_ring_next(rd, cap);
	}
	vxd_write(vxd, VXD_REG_TO_HOST_RD_INDEX, rd);
	return n;
}

static void vxd_msg_completion(struct apple_vxd_dev *vxd, const u32 *msg,
			       u32 n)
{
	u16 id = FIELD_GET(VXD_MSG_FENCE, msg[0]);
	u32 status = n > 2 ? msg[2] : 0;
	u32 w3 = n > 3 ? msg[3] : 0;
	bool fault = status & VXD_COMPLETION_FAULT;
	unsigned int i;

	if (fault) {
		if ((status & VXD_COMPLETION_AFTER_RESET) && !w3)
			dev_warn_ratelimited(vxd->dev,
					     "completion fence %#x after core reset (status %#x)\n",
					     id, status);
		else
			dev_err_ratelimited(vxd->dev,
					    "completion fence %#x with fault: status %#x, word1 %#x, word3 %#x\n",
					    id, status, n > 1 ? msg[1] : 0, w3);
	}
	if (status & VXD_COMPLETION_REATTEMPT)
		dev_warn_ratelimited(vxd->dev,
				     "completion fence %#x asks for REATTEMPT_SINGLEPIPE (0x200c %#x)\n",
				     id, vxd_read(vxd, VXD_REG_FW_PC));

	for (i = 0; i < APPLE_VXD_NUM_FENCES; i++) {
		struct apple_vxd_fence *fence = &vxd->fences[i];

		if (fence->state == APPLE_VXD_FENCE_ARMED && fence->id == id) {
			vxd_fence_complete(vxd, fence, status, fault, true);
			return;
		}
	}

	dev_warn_ratelimited(vxd->dev,
			     "completion for unknown fence %#x (status %#x)\n",
			     id, status);
}

/* ProcessFWMessages' type switch (0xfffffff006299bb4 [E]).  mbox_lock held. */
static void vxd_msg_dispatch(struct apple_vxd_dev *vxd, const u32 *msg, u32 n)
{
	u8 type = FIELD_GET(VXD_MSG_TYPE, msg[0]);

	switch (type) {
	case VXD_MSG_COMPLETION:
		vxd_msg_completion(vxd, msg, n);
		break;
	case VXD_MSG_CONTIGUITY:
		dev_warn_ratelimited(vxd->dev,
				     "firmware contiguity warning (%#x %#x)\n",
				     n > 1 ? msg[1] : 0, n > 2 ? msg[2] : 0);
		break;
	case VXD_MSG_PANIC:
		vxd->fw_panicked = true;
		dev_err_ratelimited(vxd->dev,
				    "FW_DEVA_PANIC: fence %#x, error int %#x; decode is off until the firmware is rebooted\n",
				    (u32)FIELD_GET(VXD_MSG_FENCE, msg[0]),
				    n > 1 ? msg[1] : 0);
		vxd_fences_fail_all(vxd);
		break;
	default:
		/* The kext drops 0xd1..0xd4 and 0xe1..0xe4 without a word. */
		if ((type >= 0xd1 && type <= 0xd4) ||
		    (type >= 0xe1 && type <= 0xe4)) {
			dev_dbg(vxd->dev, "firmware message %#x ignored\n", type);
			break;
		}
		dev_warn_ratelimited(vxd->dev,
				     "unknown message %#010x from the DEVA firmware (%u words)\n",
				     msg[0], n);
		break;
	}
}

/* mbox_lock held. */
static unsigned int vxd_mbox_drain_locked(struct apple_vxd_dev *vxd)
{
	u32 msg[VXD_MSG_TO_HOST_MAX_WORDS];
	unsigned int count;
	int n;

	for (count = 0; count < VXD_DRAIN_MAX_MSGS; count++) {
		n = vxd_msg_pop(vxd, msg);
		if (n <= 0)
			return count;
		vxd_msg_dispatch(vxd, msg, n);
	}

	dev_warn_ratelimited(vxd->dev, "too many messages in the fw2host queue\n");
	return count;
}

/*
 * Level interrupt.  Armed only while the firmware runs (the boot holds a
 * runtime PM reference for that whole time, so the block is powered).
 * Apple's order: status, clear, then drain.
 */
irqreturn_t apple_vxd_irq(int irq, void *data)
{
	struct apple_vxd_dev *vxd = data;
	u32 status;

	spin_lock(&vxd->mbox_lock);
	if (!vxd->mbox_ready) {
		spin_unlock(&vxd->mbox_lock);
		return IRQ_NONE;
	}

	status = vxd_read(vxd, VXD_REG_IRQ_STATUS);
	if (!(status & VXD_IRQ_FW_MSG)) {
		spin_unlock(&vxd->mbox_lock);
		return IRQ_NONE;
	}

	vxd_write(vxd, VXD_REG_IRQ_CLEAR, VXD_IRQ_FW_MSG);
	if (vxd_mbox_drain_locked(vxd)) {
		vxd->irq_empty = 0;
	} else if (++vxd->irq_empty == VXD_IRQ_STORM_LIMIT) {
		disable_irq_nosync(irq);
		vxd->irq_storm = true;
		dev_err(vxd->dev,
			"IRQ %d: %u interrupts in a row with nothing to drain (0x410 %#x); IRQ disabled\n",
			irq, VXD_IRQ_STORM_LIMIT, status);
	}
	spin_unlock(&vxd->mbox_lock);

	return IRQ_HANDLED;
}

/* --- start / stop ------------------------------------------------------- */

static void vxd_mbox_log_state(struct apple_vxd_dev *vxd, const char *what)
{
	dev_err(vxd->dev,
		"%s: TO_MTX rd %u wr %u, TO_HOST rd %u wr %u, 0x200c %#x, 0x0 %#x, comms state %#x, irq status %#x\n",
		what, vxd_read(vxd, VXD_REG_TO_MTX_RD_INDEX),
		vxd_read(vxd, VXD_REG_TO_MTX_WR_INDEX),
		vxd_read(vxd, VXD_REG_TO_HOST_RD_INDEX),
		vxd_read(vxd, VXD_REG_TO_HOST_WR_INDEX),
		vxd_read(vxd, VXD_REG_FW_PC),
		vxd_read(vxd, VXD_REG_MTX_MSG_CONSUMED),
		vxd_read(vxd, VXD_REG_COMMS_STATE),
		vxd_read(vxd, VXD_REG_IRQ_STATUS));
}

/*
 * The mandatory 0x80 init message, then the IRQ.  Without the init
 * message the firmware has no picture of its own image and never
 * schedules work (ds-vxd-decode-protocol.md section 3.1).  Boot path,
 * power_lock held, mailbox open.
 */
int apple_vxd_mbox_start(struct apple_vxd_dev *vxd)
{
	struct apple_vxd_msg_ticket ticket;
	u32 msg[VXD_INIT_MSG_WORDS];
	unsigned int waited;
	int ret;

	apple_vxd_build_init_msg(msg, vxd->fw_dev_va, vxd->fw_blob_size,
				 vxd->init_seq);
	ret = apple_vxd_msg_push(vxd, msg, &ticket);
	if (ret) {
		dev_err(vxd->dev, "init message push failed: %d\n", ret);
		return ret;
	}
	vxd->init_seq++;

	ret = apple_vxd_msg_wait_consumed(vxd, &ticket,
					  VXD_MSG_CONSUME_TIMEOUT_US, &waited);
	if (ret) {
		vxd_mbox_log_state(vxd, "init message not consumed");
		return ret;
	}
	dev_dbg(vxd->dev, "init message consumed after %u us\n", waited);

	enable_irq(vxd->irq);
	vxd->irq_armed = true;
	return 0;
}

/* Teardown, power_lock held: no IRQ, no pushes, every waiter woken. */
void apple_vxd_mbox_stop(struct apple_vxd_dev *vxd)
{
	unsigned long flags;

	if (vxd->irq_armed) {
		/* The storm guard already disabled it from the handler. */
		if (READ_ONCE(vxd->irq_storm))
			synchronize_irq(vxd->irq);
		else
			disable_irq(vxd->irq);
		vxd->irq_armed = false;
	}

	spin_lock_irqsave(&vxd->mbox_lock, flags);
	if (vxd->mbox_ready) {
		/* Quiesce the line the way known state does. */
		vxd_write(vxd, VXD_REG_IRQ_ENABLE, 0);
		vxd_write(vxd, VXD_REG_IRQ_CLEAR, 0xffffffff);
	}
	vxd->mbox_ready = false;
	vxd_fences_fail_all(vxd);
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
}

/*
 * Decode submission: one picture's command buffer, one mailbox message per
 * packet, in packet-list order (SubmitCommandBufferInternal
 * 0xfffffff00628a8d4 [E]).  The last message carries 0x4004 and its fence
 * completes the job (apple_vxd_job_done(), process context); the buffer
 * is bound to that fence and freed by its 0xc0.  Returns 0 once every
 * message is in TO_MTX; on an error nothing is left armed, and a buffer
 * the firmware may already have been told about stays quarantined.
 *
 * Messages are built twice (checked before anything is armed, rebuilt for
 * the push) so a 128-slice picture needs no per-packet array on the stack.
 */
int apple_vxd_submit(struct apple_vxd_ctx *ctx, struct apple_vxd_cmdbuf *cb)
{
	struct apple_vxd_fence *fences[APPLE_VXD_JOB_MAX_FENCES] = { };
	struct apple_vxd_dev *vxd = ctx->vxd;
	struct apple_vxd_msg_ticket ticket;
	u32 msg[VXD_DECODE_MSG_WORDS];
	unsigned int i, narmed, first_armed, pushed = 0;
	struct apple_vxd_fence *last = NULL;
	unsigned long flags;
	bool gart_dirty;
	int n, ret;
	size_t off;
	u16 seq;

	n = apple_vxd_cmdbuf_validate(cb);
	if (n <= 0 || n > APPLE_VXD_JOB_MAX_PACKETS) {
		dev_err(vxd->dev, "decode: %d packets in the command buffer (1..%u)\n",
			n, APPLE_VXD_JOB_MAX_PACKETS);
		return -EINVAL;
	}
	narmed = n <= APPLE_VXD_JOB_MAX_FENCES ? n : 1;
	first_armed = n - narmed;

	/* Every packet must make a message before anything is armed. */
	for (i = 0, off = 0; i < n; i++) {
		u32 *pkt = (u32 *)(cb->pkt + off);

		ret = apple_vxd_packet_msg(vxd, pkt, 0, 0, i == n - 1, msg);
		if (ret < 0)
			return ret;
		off += VXD_PKT_HDR_BYTES + pkt[1];
	}

	mutex_lock(&vxd->power_lock);
	if (!vxd->mbox_ready || !vxd->hw_ready || vxd->fw_panicked) {
		ret = -ENODEV;
		goto out_unlock;
	}

	/* Fences first: the firmware may answer before the last push. */
	spin_lock_irqsave(&vxd->mbox_lock, flags);
	seq = vxd_fence_next_seq_locked(vxd);
	for (i = 0; i < narmed; i++) {
		fences[i] = vxd_fence_arm_locked(vxd,
						 vxd_fence_id(seq, first_armed + i),
						 VXD_MSG_DECODE);
		if (!fences[i])
			break;
	}
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
	if (i < narmed) {
		ret = -EBUSY;
		goto out_fences;
	}

	last = fences[narmed - 1];
	last->job = true;
	ret = apple_vxd_cmdbuf_bind(vxd, cb, last);
	if (ret)
		goto out_fences;

	spin_lock_irqsave(&vxd->job_lock, flags);
	if (vxd->job_active) {
		spin_unlock_irqrestore(&vxd->job_lock, flags);
		ret = -EBUSY;
		goto out_unbind;
	}
	vxd->job.ctx = ctx;
	vxd->job.nfences = narmed;
	vxd->job.npackets = n;
	vxd->job.seq = seq;
	vxd->job.submitted = ktime_get();
	memcpy(vxd->job.fences, fences, sizeof(fences));
	vxd->job_active = true;
	spin_unlock_irqrestore(&vxd->job_lock, flags);

	/* The kext rewrites 0x1008 before a submit that follows a GART change. */
	mutex_lock(&vxd->gart.lock);
	gart_dirty = vxd->gart.dirty;
	vxd->gart.dirty = false;
	mutex_unlock(&vxd->gart.lock);
	if (gart_dirty)
		vxd_write(vxd, VXD_REG_MMU_CONTROL1, VXD_MMU_INVALDC_ALL);

	for (i = 0, off = 0; i < n; i++) {
		u32 *pkt = (u32 *)(cb->pkt + off);
		int type;

		off += VXD_PKT_HDR_BYTES + pkt[1];
		type = apple_vxd_packet_msg(vxd, pkt, vxd_fence_id(seq, i), 0,
					    i == n - 1, msg);
		if (i >= first_armed)
			fences[i - first_armed]->type = type;
		ret = apple_vxd_msg_push(vxd, msg, &ticket);
		if (ret)
			break;
		pushed++;
	}
	if (ret) {
		dev_err(vxd->dev, "decode: push %u of %d failed: %d%s\n", pushed + 1,
			n, ret, pushed ? "; the command buffer stays quarantined" : "");
		spin_lock_irqsave(&vxd->job_lock, flags);
		vxd->job_active = false;
		spin_unlock_irqrestore(&vxd->job_lock, flags);
		if (pushed)
			goto out_fences;	/* release leaves the buffer DEVICE */
		goto out_unbind;
	}

	schedule_delayed_work(&vxd->watchdog_work,
			      msecs_to_jiffies(APPLE_VXD_JOB_TIMEOUT_MS));
	mutex_unlock(&vxd->power_lock);
	return 0;

out_unbind:
	spin_lock_irqsave(&vxd->mbox_lock, flags);
	if (cb->state == APPLE_VXD_CMDBUF_DEVICE && cb->fence == last) {
		cb->state = APPLE_VXD_CMDBUF_HOST;
		cb->fence = NULL;
		last->cmdbuf = NULL;
	}
	spin_unlock_irqrestore(&vxd->mbox_lock, flags);
out_fences:
	for (i = 0; i < narmed; i++)
		if (fences[i])
			apple_vxd_fence_release(vxd, fences[i]);
out_unlock:
	mutex_unlock(&vxd->power_lock);
	return ret;
}
