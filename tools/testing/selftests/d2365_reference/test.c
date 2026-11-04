// SPDX-License-Identifier: GPL-2.0-only
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <linux/power/apple-d2365-reference.h>

struct transaction { unsigned int reg; size_t len; bool write; };
struct mock {
	uint8_t memory[0x5010];
	struct transaction log[64];
	unsigned int count, fail_at, pair_reads, property_calls;
	int property_error;
	unsigned int failure_commit;
	bool mutate_pairs;
	struct d2365_sources sources;
};
static int transfer(struct mock *m, unsigned int reg, uint8_t *out,
		    const uint8_t *in, size_t len)
{
	bool failed;
	size_t n;

	assert(reg + len <= sizeof(m->memory));
	assert(m->count < 64);
	m->log[m->count++] = (struct transaction){reg, len, in != NULL};
	failed = m->count == m->fail_at;
	n = failed ? (m->failure_commit == 2 ? len :
		      m->failure_commit == 1 ? len / 2 : 0) : len;
	if (!in && reg == 0x900 && ++m->pair_reads == 2 && m->mutate_pairs)
		for (unsigned int i = 0; i < 36; i++)
			m->memory[reg + i] = 0xa5;
	if (in)
		memcpy(m->memory + reg, in, n);
	else {
		memcpy(out, m->memory + reg, n);
		if (reg == 0x140)
			memset(m->memory + reg, 0, n);
	}
	return failed ? -EIO : 0;
}
static int rd(void *ctx, unsigned int reg, uint8_t *out, size_t len)
{
	return transfer(ctx, reg, out, NULL, len);
}
static int wr(void *ctx, unsigned int reg, const uint8_t *in, size_t len)
{
	return transfer(ctx, reg, NULL, in, len);
}
static int sources(void *ctx, struct d2365_sources *out)
{
	struct mock *m = ctx;

	assert(m->count == 5); /* Sample after bookkeeping, just once. */
	m->property_calls++;
	*out = m->sources;
	return m->property_error;
}
static struct d2365_io ops(struct mock *m)
{
	return (struct d2365_io){m, rd, wr, sources};
}
static void reset(struct mock *m)
{
	memset(m, 0, sizeof(*m));
	memset(m->memory, 0xff, sizeof(m->memory));
}
int main(void)
{
	struct mock m;
	struct d2365_io io = ops(&m);
	struct d2365_result r;
	unsigned int count;

	/* All property combinations, all preserved boot-byte values. Missing
	 * means absent Apple property/object, never an unimplemented provider.
	 */
	for (int a = 0; a <= 3; a++)
	for (int b = 0; b <= 3; b++)
	for (int v = 0; v < 256; v++) {
		reset(&m);
		m.sources = (struct d2365_sources){a, b};
		m.memory[0x500f] = v;
		r = d2365_reference_shutdown(&io, true);
		if ((a == 3 || b == 3) && a != 2 && b != 2) {
			assert(r.terminal == D2365_UNRESOLVED);
			assert(r.first_error == -ENODATA && m.count == 5);
		} else {
			bool standby = a != 2 && b != 2;
			assert(r.terminal == (standby ? D2365_STANDBY_WAIT : D2365_PLATFORM_RESET));
			assert(m.memory[0x500f] == ((v & 0x27) | (standby ? 0 : 0x90)));
			assert(!r.first_error && r.standby_attempted == standby);
		}
		assert(m.property_calls == 1);
	}
	for (unsigned int v = 0; v < 256; v++) {
		reset(&m); m.mutate_pairs = true;
		memset(m.memory + 0x900, v, 36);
		for (unsigned int i = 0; i < 6; i++) m.memory[0x929 + i * 6] = v;
		r = d2365_reference_shutdown(&io, true);
		assert(r.terminal == D2365_STANDBY_WAIT && !r.first_error);
		for (unsigned int i = 0; i < 18; i++) {
			assert(m.memory[0x900 + i * 2] == (v <= 0x3f ? (v & 0xfe) : v));
			assert(m.memory[0x901 + i * 2] == 0xa5);
		}
		for (unsigned int i = 0; i < 6; i++)
			assert(m.memory[0x929 + i * 6] == (v <= 0x3f ? (v & 0xfe) : v));
	}
	reset(&m); m.property_error = -EIO;
	m.sources = (struct d2365_sources){D2365_PROPERTY_TRUE, D2365_PROPERTY_TRUE};
	r = d2365_reference_shutdown(&io, true);
	assert(r.terminal == D2365_UNRESOLVED && r.first_error == -EIO && m.count == 5);
	/* A known true does not excuse malformed enum values in the snapshot. */
	for (int invalid = -1; invalid <= 4; invalid += 5)
	for (unsigned int slot = 0; slot < 2; slot++) {
		reset(&m);
		m.sources = (struct d2365_sources){
			slot ? D2365_PROPERTY_TRUE : invalid,
			slot ? invalid : D2365_PROPERTY_TRUE,
		};
		r = d2365_reference_shutdown(&io, true);
		assert(r.terminal == D2365_UNRESOLVED &&
		       r.first_error == -ENODATA && m.count == 5);
	}
	r = d2365_reference_shutdown(NULL, true);
	assert(r.terminal == D2365_UNRESOLVED && r.first_error == -EINVAL);
	r = d2365_reference_restart(NULL);
	assert(r.terminal == D2365_UNRESOLVED && r.first_error == -EINVAL);
	for (unsigned int missing = 0; missing < 3; missing++) {
		struct d2365_io malformed = io;

		reset(&m);
		if (missing == 0) malformed.read = NULL;
		if (missing == 1) malformed.write = NULL;
		if (missing == 2) malformed.sources = NULL;
		r = d2365_reference_shutdown(&malformed, true);
		assert(r.terminal == D2365_UNRESOLVED && r.first_error == -EINVAL && !m.count);
	}
	reset(&m); r = d2365_reference_shutdown(&io, false);
	assert(r.terminal == D2365_UNRESOLVED && !m.count);
	reset(&m); r = d2365_reference_shutdown(&io, true); count = m.count;
	assert(count == 25);
	assert(m.log[0].reg == 0x140 && !m.log[0].write && m.log[0].len == 30);
	assert(m.log[1].reg == 0x1c0 && m.log[1].write && m.log[1].len == 30);
	for (unsigned int i = 0; i < 30; i++)
		assert(m.memory[0x1c0 + i] == (i == 2 ? 0xfe : i == 5 ? 0xf0 : i == 9 ? 0xfc : 0xff));
	assert(m.log[2].reg == 0x500 && m.memory[0x500] == 1);
	assert(m.log[3].reg == 0x5001 && m.memory[0x5001] == 0);
	assert(m.log[4].reg == 0x5003 && m.memory[0x5003] == 0);
	assert(m.log[5].reg == 0x500f && !m.log[5].write);
	assert(m.log[6].reg == 0x500f && m.log[6].write);
	assert(m.log[22].reg == 0x140 && !m.log[22].write);
	assert(m.log[23].reg == 0x400 && !m.log[23].write);
	assert(m.log[24].reg == 0x400 && m.log[24].write);
	for (unsigned int i = 0; i < count; i++) assert(m.log[i].reg != 0x18a);
	/* Each operation fails, including partial data commitment. No retries,
	 * no invented RMW byte, and GPIO helper failures do not skip standby.
	 */
	for (unsigned int commit = 0; commit < 3; commit++)
	for (unsigned int fail = 1; fail <= count; fail++) {
		reset(&m); m.fail_at = fail; m.failure_commit = commit;
		r = d2365_reference_shutdown(&io, true);
		assert(r.first_error == -EIO);
		assert(m.count <= count);
		if (fail == 6 || fail == 24) {
			assert(r.terminal == D2365_UNRESOLVED && !r.standby_attempted);
		} else {
			assert(r.terminal == D2365_STANDBY_WAIT && r.standby_attempted);
			assert(r.standby_acknowledged == (fail != 25));
		}
	}
	for (unsigned int fail = 0; fail <= 2; fail++) {
		reset(&m); m.fail_at = fail;
		r = d2365_reference_restart(&io);
		assert(r.terminal == D2365_PLATFORM_RESET && m.count == 2);
		assert(m.log[0].reg == 0x140 && !m.log[0].write && m.log[0].len == 30);
		assert(m.log[1].reg == 0x5001 && m.log[1].write && m.log[1].len == 1);
		assert(r.first_error == (fail ? -EIO : 0));
	}
	puts("Apple-derived offline reference: all cases passed");
	return 0;
}
