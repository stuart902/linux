// SPDX-License-Identifier: GPL-2.0-only
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <linux/power/apple-d2365-source.h>

static struct d2365_source source;
static struct cbtl_dock_events dock;
static struct d2365_source_token token;

static void bind_source(bool connected, unsigned int current, bool rejected)
{
	struct d2365_source_initial initial = {
		.profile = D2365_SOURCE_USB_HOST_ONLY,
		.dock_history = D2365_SOURCE_HISTORY_ESTABLISHED,
		.effective = connected ? D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE,
		.effective_epoch = 10,
		.requested = connected ? D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE,
		.requested_epoch = 10,
		.last_event_epoch = 10,
		.host_ma = current,
		.host_sequence = 1,
		.raw_rejected = rejected ? D2365_SOURCE_LATCH_SET : D2365_SOURCE_LATCH_CLEAR,
		.allocated_ma = current > 3262 ? 3262 : current,
	};

	d2365_source_init(&source);
	cbtl_dock_init(&dock);
	cbtl_dock_cable(&dock, 1);
	assert(d2365_source_bind(&source, &initial, &dock, 10, &token) == 0);
}

static void expect(bool cable, bool raw)
{
	assert(source.published.usb_cable == (cable ? D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE));
	assert(source.published.raw_external == (raw ? D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE));
}

static void threshold_matrix(void)
{
	unsigned int grant, allocated;

	for (grant = 0; grant <= 350; grant++) {
		for (allocated = 0; allocated <= 350; allocated++) {
			bind_source(true, 0, false);
			assert(!d2365_source_host(&source, token, 2, grant));
			assert(!d2365_source_backend(&source, token, allocated, 0));
			assert(!d2365_source_refresh(&source, &dock, 11));
			expect(true, grant > 100 && !(allocated >= 100 && allocated < 300));
		}
	}
}

static void debounce_and_epochs(void)
{
	struct d2365_source_token old;

	bind_source(true, 500, false);
	old = token;
	assert(!cbtl_dock_prepare(&dock, 4, 10));
	assert(!d2365_source_sample(&source, token.generation, false, 20, &dock, 20));
	assert(source.pending && source.effective_epoch == 10 && source.host_ma == 500);
	expect(true, false); /* Current sampled class is battery, held cable is true. */
	assert(d2365_source_deadline(&source, &dock) == 2000000010ULL);
	assert(!d2365_source_refresh(&source, &dock, 2000000009ULL));
	expect(true, false);
	/* A later persistent handshake deadline extends the existing pending edge. */
	dock.handshake_deadline_ns = 3000000010ULL;
	assert(!d2365_source_refresh(&source, &dock, 2000000010ULL));
	assert(source.pending);
	cbtl_dock_cable(&dock, 2);
	assert(dock.handshake_deadline_ns == 3000000010ULL);
	assert(!d2365_source_refresh(&source, &dock, 3000000010ULL));
	expect(false, false);
	assert(source.effective_epoch == 20 && !source.host_ma && !dock.magic_deadline_ns);
	assert(d2365_source_host(&source, old, 3, 500) == -ESTALE);
	assert(d2365_source_backend(&source, old, 500, 0) == -ESTALE);
	assert(d2365_source_backend_reset(&source, old) == -ESTALE);
	assert(!d2365_source_sample(&source, token.generation, true, 3000000020ULL,
				     &dock, 3000000020ULL));
	token = d2365_source_token(&source);
	assert(!d2365_source_host(&source, token, 3, 500));
	assert(!d2365_source_refresh(&source, &dock, 3000000021ULL));
	expect(true, true);
	/* An unchanged physical sample must not expire the existing grant. */
	assert(!d2365_source_sample(&source, token.generation, true, 3000000030ULL,
				     &dock, 3000000030ULL));
	assert(source.effective_epoch == token.effective_epoch && source.host_ma == 500);
	assert(d2365_source_sample(&source, token.generation, false, 3000000025ULL,
				    &dock, 3000000031ULL) == -ESTALE);
}

static void cancelled_pending_and_zero(void)
{
	bind_source(true, 500, false);
	dock.handshake_deadline_ns = 100;
	assert(!d2365_source_sample(&source, token.generation, false, 20, &dock, 20));
	assert(!d2365_source_sample(&source, token.generation, true, 30, &dock, 30));
	assert(!d2365_source_refresh(&source, &dock, 100));
	assert(source.effective_epoch == 10 && source.host_ma == 500 && !source.pending);
	expect(true, true);
	assert(!d2365_source_host(&source, token, 3, 0));
	assert(!d2365_source_refresh(&source, &dock, 101));
	expect(true, false);
	assert(d2365_source_host(&source, token, 2, 500) == -ESTALE);
	assert(d2365_source_host(&source, token, 3, 500) == -ESTALE);
}

static void backend_and_sticky(void)
{
	bind_source(true, 0, false);
	assert(!d2365_source_host(&source, token, 2, 200));
	assert(!d2365_source_backend(&source, token, 200, 100));
	assert(!d2365_source_refresh(&source, &dock, 11));
	expect(true, true);
	assert(d2365_source_deadline(&source, &dock) == 100);
	assert(!d2365_source_refresh(&source, &dock, 99));
	expect(true, true);
	assert(!d2365_source_refresh(&source, &dock, 100));
	expect(true, false);
	assert(!d2365_source_host(&source, token, 3, 500));
	assert(!d2365_source_backend(&source, token, 500, 200));
	assert(!d2365_source_refresh(&source, &dock, 101));
	expect(true, false);
	assert(!d2365_source_sample(&source, token.generation, false, 102, &dock, 102));
	assert(source.raw_rejected); /* Physical false is not a completed reset. */
	assert(!d2365_source_backend_reset(&source, d2365_source_token(&source)));
	assert(!source.raw_rejected);
	assert(!d2365_source_sample(&source, token.generation, true, 103, &dock, 103));
	token = d2365_source_token(&source);
	assert(!d2365_source_host(&source, token, 4, 500));
	assert(!d2365_source_refresh(&source, &dock, 104));
	expect(true, true);
}

static void lifecycle_and_invalid(void)
{
	struct d2365_sources snapshot;
	struct d2365_source_initial initial = {};
	struct d2365_source copy;
	struct cbtl_dock_events other;
	struct d2365_source_token old;

	d2365_source_init(&source);
	cbtl_dock_init(&dock);
	assert(d2365_source_snapshot(&source, &snapshot) == -ENODATA);
	assert(snapshot.raw_external == D2365_PROPERTY_UNRESOLVED);
	assert(snapshot.usb_cable == D2365_PROPERTY_UNRESOLVED);
	assert(d2365_source_bind(&source, &initial, &dock, 10, &token) == -EOPNOTSUPP);
	bind_source(true, 500, false);
	assert(d2365_source_snapshot(&source, &snapshot) == -ENODATA);
	copy = source;
	other = dock;
	assert(d2365_source_refresh(&source, &other, 11) == -ESTALE);
	assert(d2365_source_refresh(&source, &dock, 9) == -ESTALE);
	assert(!memcmp(&copy, &source, sizeof(source)));
	assert(!d2365_source_freeze(&source, &dock, 11));
	assert(!d2365_source_snapshot(&source, &snapshot));
	assert(snapshot.raw_external == D2365_PROPERTY_TRUE);
	assert(snapshot.usb_cable == D2365_PROPERTY_TRUE);
	assert(d2365_source_host(&source, token, 3, 0) == -ESHUTDOWN);
	assert(d2365_source_backend(&source, token, 0, 0) == -ESHUTDOWN);
	assert(d2365_source_backend_reset(&source, d2365_source_token(&source)) == -ESHUTDOWN);
	assert(d2365_source_sample(&source, token.generation, false, 12, &dock, 12) == -ESHUTDOWN);
	assert(!d2365_source_deadline(&source, &dock));
	old = token;
	d2365_source_invalidate(&source);
	assert(d2365_source_snapshot(&source, &snapshot) == -ENODATA);
	initial.profile = D2365_SOURCE_USB_HOST_ONLY;
	initial.dock_history = D2365_SOURCE_HISTORY_ESTABLISHED;
	initial.effective = D2365_PROPERTY_TRUE;
	initial.effective_epoch = 10;
	initial.requested = D2365_PROPERTY_TRUE;
	initial.requested_epoch = 10;
	initial.last_event_epoch = 10;
	initial.raw_rejected = D2365_SOURCE_LATCH_CLEAR;
	assert(!d2365_source_bind(&source, &initial, &dock, 12, &token));
	assert(token.generation != old.generation);
	assert(d2365_source_host(&source, old, 100, 500) == -ESTALE);
	d2365_source_invalidate(&source);
	source.generation = ~(u64)0;
	assert(d2365_source_bind(&source, &initial, &dock, 13, &token) == -EOVERFLOW);
}

static void initial_pending_and_fail_closed(void)
{
	struct d2365_source_initial initial = {
		.profile = D2365_SOURCE_USB_HOST_ONLY,
		.dock_history = D2365_SOURCE_HISTORY_ESTABLISHED,
		.effective = D2365_PROPERTY_FALSE,
		.effective_epoch = 10,
		.requested = D2365_PROPERTY_TRUE,
		.requested_epoch = 20,
		.last_event_epoch = 20,
		.pending = true,
		.raw_rejected = D2365_SOURCE_LATCH_CLEAR,
	};
	struct d2365_sources snapshot;

	d2365_source_init(&source);
	cbtl_dock_init(&dock);
	dock.handshake_deadline_ns = 100;
	assert(!d2365_source_bind(&source, &initial, &dock, 30, &token));
	assert(source.pending && !source.effective);
	assert(d2365_source_host(&source, token, 1, 500) == -ENOTCONN);
	assert(!d2365_source_refresh(&source, &dock, 100));
	assert(source.effective && source.effective_epoch == 20);
	token = d2365_source_token(&source);
	assert(d2365_source_host(&source, token, 1, 0xffffffffU) == -ERANGE);
	assert(d2365_source_freeze(&source, &dock, 101) == -ENODATA);
	assert(d2365_source_snapshot(&source, &snapshot) == -ENODATA);
	bind_source(true, 500, false);
	assert(d2365_source_backend(&source, token, 3263, 0) == -ERANGE);
	assert(d2365_source_snapshot(&source, &snapshot) == -ENODATA);
	d2365_source_init(&source);
	cbtl_dock_init(&dock);
	initial.pending = false;
	assert(d2365_source_bind(&source, &initial, &dock, 100, &token) == -EOPNOTSUPP);
}

static int fake_read(void *context, unsigned int reg, u8 *data, size_t count)
{
	(void)context;
	(void)reg;
	memset(data, 0, count);
	return 0;
}

static int fake_write(void *context, unsigned int reg, const u8 *data, size_t count)
{
	(void)context;
	(void)reg;
	(void)data;
	(void)count;
	return 0;
}

static int fake_sources(void *context, struct d2365_sources *snapshot)
{
	return d2365_source_snapshot(context, snapshot);
}

static void reference_integration(void)
{
	struct d2365_io io = {
		.context = &source,
		.read = fake_read,
		.write = fake_write,
		.sources = fake_sources,
	};
	struct d2365_result result;

	bind_source(true, 100, false);
	assert(!d2365_source_freeze(&source, &dock, 11));
	result = d2365_reference_shutdown(&io, true);
	assert(result.terminal == D2365_PLATFORM_RESET && !result.standby_attempted);
	bind_source(false, 0, false);
	assert(!d2365_source_freeze(&source, &dock, 11));
	result = d2365_reference_shutdown(&io, true);
	assert(result.terminal == D2365_STANDBY_WAIT && result.standby_acknowledged);
	d2365_source_invalidate(&source);
	result = d2365_reference_shutdown(&io, true);
	assert(result.terminal == D2365_UNRESOLVED && !result.standby_attempted);
}

int main(void)
{
	threshold_matrix();
	debounce_and_epochs();
	cancelled_pending_and_zero();
	backend_and_sticky();
	lifecycle_and_invalid();
	initial_pending_and_fail_closed();
	reference_integration();
	puts("d2365 source: 123201 current/allocation pairs, "
	     "debounce, epochs, backend, freeze passed");
	return 0;
}
