// SPDX-License-Identifier: GPL-2.0-only
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <linux/power/apple-cbtl-idbus.h>

struct fake {
	u8 regs[256];
	u8 irq_status;
	unsigned int reads, writes, prepares, recovers, starts, retires;
	u64 now_ns, io_delay_ns;
	unsigned int fail_read, fail_write;
	bool bad_verify, fail_prepare, fail_recover, fail_retire;
	u8 write_reg[16], write_data[16][66];
	unsigned int write_len[16];
};

static int read_reg(void *p, u8 reg, u8 *data, unsigned int count)
{
	struct fake *f = p;

	f->reads++;
	f->now_ns += f->io_delay_ns;
	assert((unsigned int)reg + count <= sizeof(f->regs));
	if (f->fail_read == f->reads) {
		memset(data, 0x71, count); /* Partial data must never become response. */
		return -EIO;
	}
	memcpy(data, &f->regs[reg], count);
	if (f->bad_verify && reg != 0x20)
		data[0] ^= 1;
	return 0;
}

static int write_reg(void *p, u8 reg, const u8 *data, unsigned int count)
{
	struct fake *f = p;
	unsigned int i = f->writes++;
	f->now_ns += f->io_delay_ns;

	assert(i < 16 && count <= 66 && (unsigned int)reg + count <= sizeof(f->regs));
	f->write_reg[i] = reg;
	f->write_len[i] = count;
	memcpy(f->write_data[i], data, count);
	memcpy(&f->regs[reg], data, count); /* Error may be after commitment. */
	if (reg <= 0x61 && reg + count > 0x61)
		f->starts++;
	return f->fail_write == f->writes ? -EIO : 0;
}

static int prepare(void *p)
{
	struct fake *f = p;

	f->prepares++;
	return f->fail_prepare ? -EIO : 0;
}

static u64 now(void *p)
{
	return ((struct fake *)p)->now_ns;
}

static int retire(void *p)
{
	struct fake *f = p;

	f->retires++;
	if (!f->fail_retire)
		f->irq_status = 0;
	return f->fail_retire ? -EIO : 0;
}

static int recover(void *p)
{
	struct fake *f = p;

	f->recovers++;
	return f->fail_recover ? -EIO : 0;
}

static struct cbtl_idbus_cable cable = {
	.first = CBTL_IDBUS_ROUTE_USB0,
	.second = CBTL_IDBUS_ROUTE_NONE,
	.orientation = 1,
	.mode = 1 | 0x2000,
	.mode_valid = true,
	.payload = { 0x18, 1, 0, 0, 0xa5, 0x5a },
};

static struct cbtl_idbus_request request = {
	.owner = 10,
	.owner_generation = 1,
	.command = 3,
	.tx = { 0x72 },
	.tx_len = 1,
	.rx_len = 5,
	.timeout_ns = CBTL_IDBUS_TIMEOUT_NS,
};

static void setup(struct cbtl_idbus *bus, struct fake *fake)
{
	struct cbtl_idbus_io io = {
		.context = fake, .read = read_reg, .write = write_reg,
		.prepare = prepare, .recover = recover, .now_ns = now, .retire = retire,
	};

	u64 generation;

	memset(fake, 0, sizeof(*fake));
	cbtl_idbus_init(bus, &io);
	assert(!cbtl_idbus_attach(bus, &cable));
	assert(!cbtl_idbus_open_owner(bus, 10, &generation));
	assert(generation == 1);
}

static struct cbtl_idbus_token enqueue(struct cbtl_idbus *bus)
{
	struct cbtl_idbus_token token;

	assert(!cbtl_idbus_submit(bus, &request, &token));
	return token;
}

static u8 crc(const u8 *data, unsigned int len)
{
	unsigned int c = 0xff, i, bit;

	for (i = 0; i < len; i++) {
		c ^= data[i];
		for (bit = 0; bit < 8; bit++)
			c = (c >> 1) ^ ((c & 1) ? 0x8c : 0);
	}
	return c;
}

static void response(struct fake *f, unsigned int len, u8 opcode)
{
	f->irq_status = 0x10;
	memset(f->regs + 0x20, 0xa5, len);
	f->regs[0x20] = opcode;
	f->regs[0x20 + len] = crc(f->regs + 0x20, len);
}

static void test_framing(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_request r;
	struct cbtl_idbus_token token;
	struct cbtl_idbus_result result;
	unsigned int tx, rx, alt, verify, i;

	/* Every length and both encodings/verification modes, not only edges. */
	for (tx = 1; tx <= 64; tx++)
	for (rx = 1; rx <= 64; rx++)
	for (alt = 0; alt < 2; alt++)
	for (verify = 0; verify < 2; verify++) {
		setup(&bus, &f);
		bus.cable.mode = verify ? 1 : 0x11;
		bus.cable.alternate_rx = alt;
		r = request;
		r.tx_len = tx;
		r.rx_len = rx;
		for (i = 0; i < tx; i++)
			r.tx[i] = i + 7;
		assert(!cbtl_idbus_submit(&bus, &r, &token));
		memset(r.tx, 0xff, sizeof(r.tx)); /* Retained copy, not caller memory. */
		assert(!cbtl_idbus_start(&bus));
		assert(f.prepares == 1 && f.starts == 1 && f.writes == 1 + verify);
		assert(f.write_reg[0] == 0x60 - tx);
		assert(f.write_len[0] == tx + 2 - verify);
		for (i = 0; i < tx; i++)
			assert(f.write_data[0][i] == tx - i + 6);
		assert(f.write_data[0][tx] == (alt ? ((2 * rx) | 0x80) : 2 * rx - 2));
		assert(f.regs[0x61] == 1 + 2 * (tx - 1));
		if (verify) {
			assert(f.reads == 1 && f.write_reg[1] == 0x61 && f.write_len[1] == 1);
		}
		response(&f, rx, 0x73);
		assert(!cbtl_idbus_irq(&bus, token, 0x10));
		assert(cbtl_idbus_take_result(&bus, &result) == 1);
		assert(!result.status && result.rx_len == rx && result.rx[0] == 0x73);
		assert(!memcmp(result.cable.payload, cable.payload, 6));
		assert(cbtl_idbus_take_result(&bus, &result) == 0);
	}
}

static void test_admission(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_request r = request;
	struct cbtl_idbus_token token;
	struct cbtl_idbus_cable c = cable;
	unsigned int i;

	setup(&bus, &f);
	for (i = 0; i <= 65; i += 65) {
		r.tx_len = i;
		assert(cbtl_idbus_submit(&bus, &r, &token) == -EINVAL);
		r = request;
		r.rx_len = i;
		assert(cbtl_idbus_submit(&bus, &r, &token) == -EINVAL);
		r = request;
	}
	r.owner = 0;
	assert(cbtl_idbus_submit(&bus, &r, &token) == -EINVAL);
	r = request;
	r.timeout_ns = 0;
	assert(cbtl_idbus_submit(&bus, &r, &token) == -EINVAL);
	assert(cbtl_idbus_attach(&bus, &cable) == -EBUSY);
	assert(!cbtl_idbus_detach(&bus));
	c.mode_valid = false;
	assert(cbtl_idbus_attach(&bus, &c) == -EINVAL);
	c = cable;
	c.first = -1;
	assert(cbtl_idbus_attach(&bus, &c) == -EINVAL);
	c = cable;
	c.orientation = 0;
	assert(cbtl_idbus_attach(&bus, &c) == -EINVAL);
	assert(cbtl_idbus_submit(&bus, &request, &token) == -ENODEV);
	assert(!f.reads && !f.writes);
	setup(&bus, &f);
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++)
		enqueue(&bus);
	assert(cbtl_idbus_submit(&bus, &request, &token) == -ENOSPC);
	setup(&bus, &f);
	bus.next_sequence = ~(u64)0;
	assert(cbtl_idbus_submit(&bus, &request, &token) == -EOVERFLOW);
	assert(!cbtl_idbus_detach(&bus));
	bus.generation = ~(u64)0;
	assert(cbtl_idbus_attach(&bus, &cable) == -EOVERFLOW);
}

static void test_failures(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_token token;
	struct cbtl_idbus_result result;
	unsigned int mode, reads;

	for (mode = 0; mode < 5; mode++) {
		setup(&bus, &f);
		token = enqueue(&bus);
		f.fail_prepare = mode == 0;
		f.fail_write = mode == 1 ? 1 : mode == 4 ? 2 : 0;
		f.fail_read = mode == 2 ? 1 : 0;
		f.bad_verify = mode == 3;
		assert(cbtl_idbus_start(&bus) == -EIO);
		assert(f.recovers == 1 && bus.active == -1);
		assert(cbtl_idbus_take_result(&bus, &result) == 1);
		assert(result.status == -EIO && !result.rx_len && !result.rx[0]);
		assert(cbtl_idbus_timeout(&bus, token, ~(u64)0) == -ESTALE);
		assert(f.starts == (mode == 4));
	}
	for (mode = 0; mode < 3; mode++) {
		setup(&bus, &f);
		token = enqueue(&bus);
		assert(!cbtl_idbus_start(&bus));
		response(&f, 5, 0x73);
		if (mode == 0)
			f.fail_read = f.reads + 1;
		if (mode == 1)
			f.regs[0x25] ^= 1;
		assert(cbtl_idbus_irq(&bus, token, mode == 2 ? 0x18 : 0x10) < 0);
		assert(f.recovers == 1);
		assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.status < 0);
		assert(!result.rx_len && !result.rx[0]);
	}
	setup(&bus, &f);
	token = enqueue(&bus);
	f.now_ns = ~(u64)0;
	assert(cbtl_idbus_start(&bus) == -EOVERFLOW);
	assert(f.starts == 1 && f.recovers == 1);
	setup(&bus, &f);
	token = enqueue(&bus);
	f.now_ns = 20;
	assert(!cbtl_idbus_start(&bus));
	reads = f.reads;
	assert(cbtl_idbus_irq(&bus, token, 0) == -EAGAIN && f.reads == reads);
	assert(cbtl_idbus_timeout(&bus, token, 20 + CBTL_IDBUS_TIMEOUT_NS - 1) == -EAGAIN);
	assert(!cbtl_idbus_timeout(&bus, token, 20 + CBTL_IDBUS_TIMEOUT_NS));
	assert(cbtl_idbus_irq(&bus, token, 0x10) == -ESTALE && f.reads == reads);
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.status == -ETIMEDOUT);
	setup(&bus, &f);
	token = enqueue(&bus);
	assert(!cbtl_idbus_start(&bus));
	response(&f, 5, 0x73);
	assert(!cbtl_idbus_irq(&bus, token, 0x10));
	assert(cbtl_idbus_timeout(&bus, token, CBTL_IDBUS_TIMEOUT_NS) == -ESTALE);
	assert(!f.recovers);
}

static void test_lifecycle(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_token a, b, c;
	struct cbtl_idbus_result result;
	struct cbtl_idbus_request r = request;
	unsigned int reads;

	setup(&bus, &f);
	a = enqueue(&bus);
	r.owner = 20;
	assert(!cbtl_idbus_open_owner(&bus, 20, &r.owner_generation));
	assert(!cbtl_idbus_submit(&bus, &r, &b));
	c = enqueue(&bus);
	assert(!cbtl_idbus_start(&bus));
	assert(cbtl_idbus_start(&bus) == -EBUSY);
	assert(!cbtl_idbus_cancel_owner(&bus, 10));
	assert(f.recovers == 1);
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.token.sequence == a.sequence);
	assert(result.status == -ECANCELED);
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.token.sequence == c.sequence);
	assert(!cbtl_idbus_start(&bus));
	assert(bus.jobs[bus.active].result.token.sequence == b.sequence);
	reads = f.reads;
	assert(cbtl_idbus_irq(&bus, a, 0x10) == -ESTALE && f.reads == reads);
	assert(!cbtl_idbus_detach(&bus));
	assert(!cbtl_idbus_current(&bus, b));
	assert(cbtl_idbus_irq(&bus, b, 0x10) == -ESTALE && f.reads == reads);
	assert(!cbtl_idbus_attach(&bus, &cable));
	assert(!cbtl_idbus_current(&bus, b));
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.token.sequence == b.sequence);
	assert(!cbtl_idbus_current(&bus, result.token));
	assert(!cbtl_idbus_open_owner(&bus, 10, &r.owner_generation));
	r.owner = 10;
	assert(!cbtl_idbus_submit(&bus, &r, &c));
	assert(!cbtl_idbus_stop(&bus));
	assert(!cbtl_idbus_drained(&bus));
	assert(cbtl_idbus_take_result(&bus, &result) == 1);
	assert(cbtl_idbus_drained(&bus));
	assert(cbtl_idbus_attach(&bus, &cable) == -ESHUTDOWN);
	assert(cbtl_idbus_submit(&bus, &request, &a) == -ESHUTDOWN);

	/* Failed abort retains unresolved ownership and never claims drained. */
	setup(&bus, &f);
	a = enqueue(&bus);
	enqueue(&bus);
	assert(!cbtl_idbus_start(&bus));
	f.fail_recover = true;
	assert(cbtl_idbus_detach(&bus) == -EIO);
	assert(bus.active >= 0 && bus.faulted && !bus.attached);
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.status == -ECANCELED);
	assert(cbtl_idbus_take_result(&bus, &result) == 0);
	assert(cbtl_idbus_irq(&bus, a, 0x10) == -ESTALE);
	assert(cbtl_idbus_start(&bus) == -ESHUTDOWN);
	assert(cbtl_idbus_stop(&bus) == -EIO && !cbtl_idbus_drained(&bus));
	f.fail_recover = false;
	assert(!cbtl_idbus_stop(&bus));
	assert(!cbtl_idbus_drained(&bus));
	assert(cbtl_idbus_take_result(&bus, &result) == 1 && result.status == -ECANCELED);
	assert(cbtl_idbus_drained(&bus));
}

static void test_owner_epochs(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_result copied, queued;
	struct cbtl_dock_events dock;
	struct cbtl_idbus_token a, b;
	struct cbtl_idbus_request r = request;
	u64 epoch;

	setup(&bus, &f);
	a = enqueue(&bus);
	cbtl_dock_init(&dock);
	cbtl_dock_cable(&dock, bus.generation);
	assert(!cbtl_dock_prepare(&dock, 3, 0));
	assert(!cbtl_dock_track(&dock, a, 3));
	assert(!cbtl_idbus_start(&bus));
	response(&f, 5, 0x73);
	assert(!cbtl_idbus_irq(&bus, a, f.irq_status));
	assert(!f.irq_status && f.retires == 1);
	assert(cbtl_idbus_take_result(&bus, &copied) == 1);
	assert(cbtl_idbus_result_current(&bus, &copied));
	b = enqueue(&bus);
	assert(!cbtl_idbus_start(&bus));
	/* Successful retirement A drained untagged hardware completion status. */
	assert(cbtl_idbus_irq(&bus, b, f.irq_status) == -EAGAIN);
	response(&f, 5, 0x73);
	assert(!cbtl_idbus_irq(&bus, b, f.irq_status));
	assert(cbtl_dock_cancel(&dock, b) == -ESTALE && dock.pending_valid);
	assert(!cbtl_dock_cancel(&dock, a));
	assert(!cbtl_idbus_cancel_owner(&bus, 10));
	assert(!cbtl_idbus_result_current(&bus, &copied));
	assert(cbtl_idbus_take_result(&bus, &queued) == 1 && queued.status == -ECANCELED);
	assert(!cbtl_idbus_open_owner(&bus, 10, &epoch));
	assert(epoch != request.owner_generation);
	assert(!cbtl_idbus_result_current(&bus, &copied));
	assert(!cbtl_idbus_result_current(&bus, &queued));
	assert(cbtl_idbus_submit(&bus, &request, &a) == -ESTALE);
	r.owner_generation = epoch;
	assert(!cbtl_dock_prepare(&dock, 3, 0));
	assert(!cbtl_idbus_submit(&bus, &r, &a));
	assert(!cbtl_dock_track(&dock, a, 3));
	assert(cbtl_idbus_open_owner(&bus, 10, &epoch) == -EEXIST);
}

static void test_timing_and_recovery(void)
{
	struct cbtl_idbus bus;
	struct fake f;
	struct cbtl_idbus_result result;
	struct cbtl_idbus_token token;
	struct cbtl_idbus_request r = request;
	unsigned int failure;

	setup(&bus, &f);
	token = enqueue(&bus);
	f.io_delay_ns = 200000000;
	assert(!cbtl_idbus_start(&bus));
	assert(f.now_ns == 600000000);
	assert(bus.jobs[bus.active].deadline_ns == 700000000);
	assert(cbtl_idbus_timeout(&bus, token, 699999999) == -EAGAIN);
	assert(!cbtl_idbus_timeout(&bus, token, 700000000));

	for (failure = 0; failure < 4; failure++) {
		setup(&bus, &f);
		token = enqueue(&bus);
		f.fail_recover = true;
		if (failure == 0) {
			f.fail_write = 2; /* Start already committed. */
			assert(cbtl_idbus_start(&bus) == -EIO);
		} else {
			assert(!cbtl_idbus_start(&bus));
			if (failure == 1) {
				f.fail_read = f.reads + 1;
				assert(cbtl_idbus_irq(&bus, token, 0x10) == -EIO);
			} else if (failure == 2) {
				assert(cbtl_idbus_timeout(&bus, token, CBTL_IDBUS_TIMEOUT_NS) == -EIO);
			} else {
				response(&f, 5, 0x73);
				f.fail_retire = true;
				assert(cbtl_idbus_irq(&bus, token, 0x10) == -EIO);
			}
		}
		assert(bus.faulted && bus.active >= 0 && !bus.jobs[bus.active].timer_armed);
		assert(cbtl_idbus_take_result(&bus, &result) == 0);
		assert(cbtl_idbus_start(&bus) == -ESHUTDOWN);
		assert(!cbtl_idbus_current(&bus, token));
	}

	/* Fixed vectors calculated independently with MSB-first poly 31 on
	 * bit-reversed bytes, seed ff; not the fixture's reflected routine.
	 */
	setup(&bus, &f);
	r.rx_len = 1;
	assert(!cbtl_idbus_submit(&bus, &r, &token));
	assert(!cbtl_idbus_start(&bus));
	f.regs[0x20] = 0x71;
	f.regs[0x21] = 0x93;
	assert(!cbtl_idbus_irq(&bus, token, 0x10));
	setup(&bus, &f);
	token = enqueue(&bus);
	assert(!cbtl_idbus_start(&bus));
	memcpy(f.regs + 0x20, "\x73\x00\x00\x00\x00\xea", 6);
	assert(!cbtl_idbus_irq(&bus, token, 0x10));
	setup(&bus, &f);
	r.rx_len = 9;
	assert(!cbtl_idbus_submit(&bus, &r, &token));
	assert(!cbtl_idbus_start(&bus));
	memcpy(f.regs + 0x20, "123456789\x0b", 10);
	assert(!cbtl_idbus_irq(&bus, token, 0x10));
}

static void test_dock(void)
{
	struct cbtl_dock_events d;
	struct cbtl_idbus_result r = { .token = { .generation = 2, .sequence = 1 } };
	unsigned int command;

	cbtl_dock_init(&d);
	assert(cbtl_dock_prepare(&d, 4, 0) == -EINVAL);
	cbtl_dock_cable(&d, 2);
	assert(cbtl_dock_prepare(&d, 4, 0) == -ENODATA);
	cbtl_dock_source_epoch(&d, 100);
	assert(!cbtl_dock_prepare(&d, 4, 500));
	assert(cbtl_dock_deadline(&d) == 2000000500ULL && d.magic_companion);
	/* A subsequent submit error does NOT undo these earlier source events. */
	assert(cbtl_dock_deadline(&d) == 2000000500ULL);
	cbtl_dock_source_epoch(&d, 100);
	assert(cbtl_dock_deadline(&d));
	assert(!cbtl_dock_source_epoch(&d, 101));
	assert(cbtl_dock_source_epoch(&d, 100) == -ESTALE && d.source_epoch == 101);
	assert(!cbtl_dock_deadline(&d) && !d.magic_companion);

	r.command = 3;
	r.rx_len = 5;
	r.rx[0] = 0x75;
	assert(!cbtl_dock_track(&d, r.token, r.command));
	assert(cbtl_dock_complete(&d, &r, true, 0, 300000000) == -EPROTO);
	assert(!d.extended_valid);
	r.rx[0] = 0x73;
	r.rx[3] = 0x48;
	assert(!cbtl_dock_track(&d, r.token, r.command));
	assert(!cbtl_dock_complete(&d, &r, true, 0, 300000000));
	assert(d.extended_valid && d.extended[2] == 0x48);
	assert(cbtl_dock_complete(&d, &r, true, 0, 300000000) == -ESTALE);

	for (command = 8; command <= 11; command++) {
		r.command = command;
		r.rx_len = 1;
		r.rx[0] = 0x71;
		assert(!cbtl_dock_prepare(&d, command, 0));
		assert(d.inflow_pending);
		assert(!cbtl_dock_track(&d, r.token, command));
		assert(!cbtl_dock_complete(&d, &r, true, 0, 300000000));
		assert(d.ack_power == (command == 8 || command == 9));
		assert(d.ack_pin == (command == 9 || command == 10));
		assert(d.ack_pin_valid);
	}
	r.command = 6;
	r.rx[0] = 0x73;
	assert(!cbtl_dock_track(&d, r.token, 6));
	assert(cbtl_dock_complete(&d, &r, true, 1000, 300000000) == -EPROTO);
	assert(!d.handshake_deadline_ns);
	r.rx[0] = 0x71;
	r.status = -ETIMEDOUT;
	assert(!cbtl_dock_track(&d, r.token, 6));
	assert(cbtl_dock_complete(&d, &r, true, 1000, 300000000) == -ETIMEDOUT);
	assert(!d.handshake_deadline_ns);
	r.status = 0;
	assert(!cbtl_dock_track(&d, r.token, 6));
	assert(!cbtl_dock_complete(&d, &r, false, 1000, 300000000));
	assert(!d.handshake_deadline_ns);
	assert(!cbtl_dock_track(&d, r.token, 6));
	assert(!cbtl_dock_complete(&d, &r, true, 1000, 300000000));
	assert(d.handshake_deadline_ns == 300001000);
	cbtl_dock_source_epoch(&d, 200);
	assert(d.handshake_deadline_ns == 300001000); /* persistent expiry */
	cbtl_dock_power_request(&d, 3);
	assert(d.requested_pin);
	assert(!cbtl_dock_track(&d, r.token, 6));
	cbtl_dock_cable(&d, 4);
	assert(d.requested_pin && !d.ack_pin_valid && !d.extended_valid && !d.ack_power);
	assert(cbtl_dock_complete(&d, &r, true, 1000, 300000000) == -ESTALE);
	assert(cbtl_dock_track(&d, r.token, 6) == -ESTALE);
	cbtl_dock_power_request(&d, 1);
	assert(!d.requested_pin);
	assert(d.handshake == CBTL_DOCK_HANDSHAKE_UNKNOWN);
	assert(!cbtl_dock_handshake_update(&d, CBTL_DOCK_HANDSHAKE_UNKNOWN));
	assert(d.inflow_pending);
	assert(!cbtl_dock_handshake_update(&d, CBTL_DOCK_HANDSHAKE_FAILED));
	assert(d.inflow_pending);
	assert(!cbtl_dock_handshake_update(&d, CBTL_DOCK_HANDSHAKE_COMPLETED));
	assert(!d.inflow_pending);
	assert(!cbtl_dock_handshake_update(&d, CBTL_DOCK_HANDSHAKE_FAILED));
	assert(!d.inflow_pending);
	assert(cbtl_dock_handshake_update(&d, 3) == -EINVAL);
	r.token.generation = d.generation;
	assert(!cbtl_dock_track(&d, r.token, 6));
	assert(!cbtl_dock_cancel(&d, r.token));
	assert(cbtl_dock_cancel(&d, r.token) == -ESTALE);
	assert(d.handshake_deadline_ns == 300001000);
	assert(!cbtl_dock_prepare(&d, 6, 0));
	assert(!cbtl_dock_track(&d, r.token, 6));
}

int main(void)
{
	test_framing();
	test_admission();
	test_failures();
	test_lifecycle();
	test_owner_epochs();
	test_timing_and_recovery();
	test_dock();
	puts("CBTL IDBus: 16384 framing combinations, admission, failures, lifecycle and Dock events passed");
	return 0;
}
