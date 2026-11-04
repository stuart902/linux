// SPDX-License-Identifier: GPL-2.0-only
/* Independent bounded source policy; no firmware bytes or hardware caller. */
#include <linux/power/apple-d2365-source.h>
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#else
#include <errno.h>
#include <string.h>
#include <limits.h>
#endif

#define D2365_SOURCE_U64_MAX (~(u64)0)
#define D2365_SOURCE_BACKEND_MAX_MA 3262U

void d2365_source_init(struct d2365_source *source)
{
	memset(source, 0, sizeof(*source));
	source->published.raw_external = D2365_PROPERTY_UNRESOLVED;
	source->published.usb_cable = D2365_PROPERTY_UNRESOLVED;
}

void d2365_source_invalidate(struct d2365_source *source)
{
	source->valid = false;
	source->frozen = false;
	source->published.raw_external = D2365_PROPERTY_UNRESOLVED;
	source->published.usb_cable = D2365_PROPERTY_UNRESOLVED;
}

static int d2365_source_mutable(const struct d2365_source *source)
{
	if (!source->valid)
		return -ENODATA;
	return source->frozen ? -ESHUTDOWN : 0;
}

static void d2365_source_publish(struct d2365_source *source, u64 now_ns)
{
	bool raw = source->requested && source->host_ma > 100;

	/* Ordinary host class 4000, offset zero, no flags and zero backoff.
	 * A future actual backend protection deadline inhibits NEW rejection.
	 * Rejection remains sticky after a later, larger current grant.
	 */
	if (raw && source->allocated_ma >= 100 && source->allocated_ma < 300 &&
	    source->protection_deadline_ns <= now_ns)
		source->raw_rejected = true;
	source->published.raw_external = raw && !source->raw_rejected ?
		D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE;
	source->published.usb_cable = source->effective ?
		D2365_PROPERTY_TRUE : D2365_PROPERTY_FALSE;
}

int d2365_source_bind(struct d2365_source *source,
		       const struct d2365_source_initial *initial,
		       struct cbtl_dock_events *dock, u64 now_ns,
		       struct d2365_source_token *token)
{
	u64 generation;

	if (!initial || !dock || !token)
		return -EINVAL;
	if (source->valid)
		return -EBUSY;
	if (source->generation == D2365_SOURCE_U64_MAX)
		return -EOVERFLOW;
	if (initial->profile != D2365_SOURCE_USB_HOST_ONLY ||
	    initial->dock_history != D2365_SOURCE_HISTORY_ESTABLISHED ||
	    (initial->effective != D2365_PROPERTY_FALSE &&
	     initial->effective != D2365_PROPERTY_TRUE) ||
	    (initial->requested != D2365_PROPERTY_FALSE &&
	     initial->requested != D2365_PROPERTY_TRUE) ||
	    initial->requested_epoch < initial->effective_epoch ||
	    initial->last_event_epoch < initial->requested_epoch ||
	    (initial->requested != initial->effective &&
	     (!initial->pending || initial->requested_epoch == initial->effective_epoch)) ||
	    (initial->raw_rejected != D2365_SOURCE_LATCH_CLEAR &&
	     initial->raw_rejected != D2365_SOURCE_LATCH_SET) ||
	    initial->allocated_ma > D2365_SOURCE_BACKEND_MAX_MA ||
	    initial->host_ma > 0x7fffffffU ||
	    (initial->host_ma && (!initial->host_sequence ||
	     initial->effective != D2365_PROPERTY_TRUE)))
		return -EOPNOTSUPP;
	/* Caller establishes source/Dock identity; epochs only check coherence. */
	if (dock->source_valid && dock->source_epoch != initial->effective_epoch)
		return -ESTALE;
	generation = source->generation + 1;
	d2365_source_init(source);
	source->generation = generation;
	source->dock = dock;
	source->effective = initial->effective == D2365_PROPERTY_TRUE;
	source->requested = initial->requested == D2365_PROPERTY_TRUE;
	source->pending = initial->pending;
	source->effective_epoch = initial->effective_epoch;
	source->requested_epoch = initial->requested_epoch;
	source->last_event_epoch = initial->last_event_epoch;
	source->host_ma = initial->host_ma;
	source->host_sequence = initial->host_sequence;
	source->raw_rejected = initial->raw_rejected == D2365_SOURCE_LATCH_SET;
	source->allocated_ma = initial->allocated_ma;
	source->protection_deadline_ns = initial->protection_deadline_ns;
	source->last_now_ns = now_ns;
	source->valid = true;
	cbtl_dock_source_epoch(dock, source->effective_epoch);
	d2365_source_refresh(source, dock, now_ns);
	*token = d2365_source_token(source);
	return 0;
}

struct d2365_source_token d2365_source_token(const struct d2365_source *source)
{
	struct d2365_source_token token = {};

	if (source->valid && !source->frozen) {
		token.generation = source->generation;
		token.effective_epoch = source->effective_epoch;
	}
	return token;
}

int d2365_source_host(struct d2365_source *source, struct d2365_source_token token,
		     u64 sequence, unsigned int ma)
{
	int error = d2365_source_mutable(source);

	if (error)
		return error;
	if (token.generation != source->generation ||
	    token.effective_epoch != source->effective_epoch)
		return -ESTALE;
	if (!source->effective)
		return -ENOTCONN;
	if (!sequence || sequence <= source->host_sequence)
		return -ESTALE;
	/* The observed resolver uses signed current. */
	if (ma > 0x7fffffffU) {
		d2365_source_invalidate(source);
		return -ERANGE;
	}
	source->host_sequence = sequence;
	source->host_ma = ma;
	return 0;
}

int d2365_source_backend(struct d2365_source *source, struct d2365_source_token token,
			unsigned int allocated_ma, u64 protection_deadline_ns)
{
	int error = d2365_source_mutable(source);

	if (error)
		return error;
	if (token.generation != source->generation ||
	    token.effective_epoch != source->effective_epoch)
		return -ESTALE;
	if (allocated_ma > D2365_SOURCE_BACKEND_MAX_MA) {
		d2365_source_invalidate(source);
		return -ERANGE;
	}
	source->allocated_ma = allocated_ma;
	source->protection_deadline_ns = protection_deadline_ns;
	return 0;
}

int d2365_source_backend_reset(struct d2365_source *source, struct d2365_source_token token)
{
	int error = d2365_source_mutable(source);

	if (error)
		return error;
	if (token.generation != source->generation ||
	    token.effective_epoch != source->effective_epoch)
		return -ESTALE;
	source->raw_rejected = false;
	return 0;
}

int d2365_source_refresh(struct d2365_source *source,
			  struct cbtl_dock_events *dock, u64 now_ns)
{
	int error = d2365_source_mutable(source);

	if (error)
		return error;
	if (!dock || dock != source->dock || !dock->source_valid ||
	    dock->source_epoch != source->effective_epoch)
		return -ESTALE;
	if (now_ns < source->last_now_ns)
		return -ESTALE;
	source->last_now_ns = now_ns;
	if (source->pending && cbtl_dock_deadline(dock) <= now_ns) {
		source->pending = false;
		/* The timer only commits a newer, differing pending status. */
		if (source->requested_epoch > source->effective_epoch &&
		    source->requested != source->effective) {
			source->effective = source->requested;
			source->effective_epoch = source->requested_epoch;
			source->host_ma = 0;
			cbtl_dock_source_epoch(dock, source->effective_epoch);
		}
	}
	d2365_source_publish(source, now_ns);
	return 0;
}

int d2365_source_sample(struct d2365_source *source, u64 generation, bool usb,
		       u64 event_epoch, struct cbtl_dock_events *dock, u64 now_ns)
{
	int error = d2365_source_mutable(source);

	if (error)
		return error;
	if (generation != source->generation || event_epoch < source->last_event_epoch ||
	    now_ns < source->last_now_ns || !dock || dock != source->dock || !dock->source_valid ||
	    dock->source_epoch != source->effective_epoch)
		return -ESTALE;
	if (usb != source->requested && event_epoch <= source->last_event_epoch)
		return -ESTALE;
	source->last_event_epoch = event_epoch;
	/* Charger preserves the nonzero effective timestamp on unchanged status. */
	if (usb == source->effective && source->effective_epoch)
		event_epoch = source->effective_epoch;
	source->requested = usb;
	source->requested_epoch = event_epoch;
	if (cbtl_dock_deadline(dock) > now_ns) {
		source->pending = true;
	} else {
		source->pending = false;
		source->effective = usb;
		if (source->effective_epoch != event_epoch) {
			source->effective_epoch = event_epoch;
			source->host_ma = 0;
			cbtl_dock_source_epoch(dock, event_epoch);
		}
	}
	return d2365_source_refresh(source, dock, now_ns);
}

u64 d2365_source_deadline(const struct d2365_source *source,
			 const struct cbtl_dock_events *dock)
{
	u64 deadline;

	if (!source->valid || source->frozen || !dock || dock != source->dock ||
	    !dock->source_valid ||
	    dock->source_epoch != source->effective_epoch)
		return 0;
	deadline = source->pending ? cbtl_dock_deadline(dock) : 0;
	if (!source->raw_rejected && source->requested && source->host_ma > 100 &&
	    source->allocated_ma >= 100 && source->allocated_ma < 300 &&
	    source->protection_deadline_ns > source->last_now_ns &&
	    (!deadline || source->protection_deadline_ns < deadline))
		deadline = source->protection_deadline_ns;
	return deadline;
}

int d2365_source_freeze(struct d2365_source *source,
			 struct cbtl_dock_events *dock, u64 now_ns)
{
	int error = d2365_source_refresh(source, dock, now_ns);

	if (error)
		return error;
	source->frozen = true;
	return 0;
}

int d2365_source_snapshot(const struct d2365_source *source,
			   struct d2365_sources *snapshot)
{
	if (!snapshot)
		return -EINVAL;
	snapshot->raw_external = D2365_PROPERTY_UNRESOLVED;
	snapshot->usb_cable = D2365_PROPERTY_UNRESOLVED;
	if (!source->valid || !source->frozen)
		return -ENODATA;
	*snapshot = source->published;
	return 0;
}
