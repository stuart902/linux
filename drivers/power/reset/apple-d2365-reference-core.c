// SPDX-License-Identifier: GPL-2.0-only
/* Independent implementation of the verified J71b ordinary-path operations.
 * Terminal results describe required actions; this core does not perform them.
 */
#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif
#include <linux/power/apple-d2365-reference.h>

static void record_error(struct d2365_result *r, int error)
{
	if (error && !r->first_error)
		r->first_error = error;
}

static int read_bytes(const struct d2365_io *io, struct d2365_result *r,
		      unsigned int reg, u8 *data, size_t len)
{
	int error = io->read(io->context, reg, data, len);

	record_error(r, error);
	return error;
}

static int write_bytes(const struct d2365_io *io, struct d2365_result *r,
		       unsigned int reg, const u8 *data, size_t len)
{
	int error = io->write(io->context, reg, data, len);

	record_error(r, error);
	return error;
}

static void gpio_prepare(const struct d2365_io *io, struct d2365_result *r)
{
	u8 initial[36], pairs[36], config[24];
	unsigned int i;

	if (read_bytes(io, r, 0x900, initial, sizeof(initial)))
		return;
	for (i = 0; i < 18; i++)
		config[i] = initial[2 * i];
	for (i = 18; i < 24; i++)
		if (read_bytes(io, r, 0x929 + 6 * (i - 18), &config[i], 1))
			return;
	for (i = 0; i < 24; i++)
		if (config[i] <= 0x3f)
			config[i] &= 0xfe;
	if (read_bytes(io, r, 0x900, pairs, sizeof(pairs)))
		return;
	for (i = 0; i < 18; i++)
		pairs[2 * i] = config[i];
	if (write_bytes(io, r, 0x900, pairs, sizeof(pairs)))
		return;
	for (i = 18; i < 24; i++)
		if (write_bytes(io, r, 0x929 + 6 * (i - 18), &config[i], 1))
			return;
}

static bool valid_io(const struct d2365_io *io)
{
	return io && io->read && io->write;
}

struct d2365_result d2365_reference_shutdown(const struct d2365_io *io,
					   bool normal_path_established)
{
	struct d2365_result r = { .terminal = D2365_UNRESOLVED };
	struct d2365_sources sources;
	u8 events[30], mask[30], value;
	bool standby;
	unsigned int i;
	int error;

	if (!normal_path_established || !valid_io(io) || !io->sources) {
		r.first_error = -EINVAL;
		return r;
	}
	/* Ordinary path: initial errors do not divert to a different sequence. */
	read_bytes(io, &r, 0x140, events, sizeof(events));
	for (i = 0; i < sizeof(mask); i++)
		mask[i] = 0xff;
	mask[2] = 0xfe;
	mask[5] = 0xf0;
	mask[9] = 0xfc;
	write_bytes(io, &r, 0x1c0, mask, sizeof(mask));
	value = 1;
	write_bytes(io, &r, 0x500, &value, 1);
	value = 0;
	write_bytes(io, &r, 0x5001, &value, 1);
	write_bytes(io, &r, 0x5003, &value, 1);
	/* These are Apple software properties, NOT two raw PMIC register reads. */
	sources.raw_external = D2365_PROPERTY_UNRESOLVED;
	sources.usb_cable = D2365_PROPERTY_UNRESOLVED;
	error = io->sources(io->context, &sources);
	if (error) {
		record_error(&r, error);
		return r;
	}
	if (sources.raw_external > D2365_PROPERTY_UNRESOLVED ||
	    sources.usb_cable > D2365_PROPERTY_UNRESOLVED ||
	    sources.raw_external < D2365_PROPERTY_ABSENT ||
	    sources.usb_cable < D2365_PROPERTY_ABSENT) {
		record_error(&r, -ENODATA);
		return r;
	}
	standby = sources.raw_external != D2365_PROPERTY_TRUE &&
		  sources.usb_cable != D2365_PROPERTY_TRUE;
	/* Either independently valid true property determines Apple's reset
	 * branch. Do not require an unrelated unknown value or invent false
	 * for it. Standby still needs both properties independently resolved.
	 */
	if (standby && (sources.raw_external == D2365_PROPERTY_UNRESOLVED ||
			sources.usb_cable == D2365_PROPERTY_UNRESOLVED)) {
		record_error(&r, -ENODATA);
		return r;
	}
	/* Apple continues on read errors with its existing scratch value. The
	 * model refuses an undefined/stale-value RMW rather than inventing data.
	 */
	if (read_bytes(io, &r, 0x500f, &value, 1))
		return r;
	value = (value & 0x27) | (standby ? 0 : 0x90);
	write_bytes(io, &r, 0x500f, &value, 1);
	if (!standby) {
		r.terminal = D2365_PLATFORM_RESET;
		return r;
	}
	/* A failed GPIO helper is logged by Apple, then standby continues. */
	gpio_prepare(io, &r);
	read_bytes(io, &r, 0x140, events, sizeof(events));
	if (read_bytes(io, &r, 0x400, &value, 1))
		return r;
	value |= 1;
	r.standby_attempted = true;
	r.standby_acknowledged = !write_bytes(io, &r, 0x400, &value, 1);
	/* Represent Apple's non-returning wait, never execute it or a reset. */
	r.terminal = D2365_STANDBY_WAIT;
	return r;
}

struct d2365_result d2365_reference_restart(const struct d2365_io *io)
{
	struct d2365_result r = { .terminal = D2365_UNRESOLVED };
	u8 events[30], zero = 0;

	if (!valid_io(io)) {
		r.first_error = -EINVAL;
		return r;
	}
	read_bytes(io, &r, 0x140, events, sizeof(events));
	write_bytes(io, &r, 0x5001, &zero, 1);
	r.terminal = D2365_PLATFORM_RESET;
	return r;
}
