/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_APPLE_D2365_SOURCE_H
#define _LINUX_POWER_APPLE_D2365_SOURCE_H

#include <linux/power/apple-cbtl-idbus.h>
#include <linux/power/apple-d2365-reference.h>

/* Restricted reducer, not hardware discovery. The owner must establish one
 * selected ordinary USB source, one USBHost grant producer, no persistent
 * current/base/offset/flag/capability events, and the exact D2365 backend with
 * initialized zero backoff. Unknown or additional producers invalidate it.
 * All APIs, Dock event APIs, physical samples and publication share one lock.
 * No API accesses hardware, schedules work, or enables the reference adapter.
 */
enum d2365_source_profile {
	D2365_SOURCE_UNRESOLVED,
	D2365_SOURCE_USB_HOST_ONLY,
};

enum d2365_source_history {
	D2365_SOURCE_HISTORY_UNRESOLVED,
	D2365_SOURCE_HISTORY_ESTABLISHED,
};

enum d2365_source_latch {
	D2365_SOURCE_LATCH_UNRESOLVED,
	D2365_SOURCE_LATCH_CLEAR,
	D2365_SOURCE_LATCH_SET,
};

struct d2365_source_initial {
	enum d2365_source_profile profile;
	/* Complete, coherent Dock event/deadline history for this source.
	 * Fresh zero Dock state is valid only after known owner initialization.
	 */
	enum d2365_source_history dock_history;
	enum d2365_property effective;
	u64 effective_epoch;
	enum d2365_property requested;
	u64 requested_epoch;
	u64 last_event_epoch;
	bool pending;
	/* Existing ordinary host event, or explicit zero for an empty event set. */
	unsigned int host_ma;
	u64 host_sequence;
	enum d2365_source_latch raw_rejected;
	/* Actual allocated backend current and protection deadline, not the
	 * requested grant, quantized PMIC code, measured or ramp current.
	 */
	unsigned int allocated_ma;
	u64 protection_deadline_ns;
};

struct d2365_source_token {
	u64 generation;
	u64 effective_epoch;
};

struct d2365_source {
	/* Owned for the full binding lifetime; callbacks may not substitute it. */
	struct cbtl_dock_events *dock;
	u64 generation;
	bool valid;
	bool frozen;
	bool effective;
	u64 effective_epoch;
	bool requested;
	u64 requested_epoch;
	u64 last_event_epoch;
	bool pending;
	unsigned int host_ma;
	u64 host_sequence;
	unsigned int allocated_ma;
	u64 protection_deadline_ns;
	bool raw_rejected;
	u64 last_now_ns;
	struct d2365_sources published;
};

void d2365_source_init(struct d2365_source *source);
/* Rebind only after invalidation and the owner's producer drain. Generations
 * never wrap. Initial values must be an actual coherent owner snapshot.
 */
int d2365_source_bind(struct d2365_source *source, const struct d2365_source_initial *initial,
		       struct cbtl_dock_events *dock, u64 now_ns,
		       struct d2365_source_token *token);
void d2365_source_invalidate(struct d2365_source *source);
/* Capture token and strictly increasing sequence at grant PRODUCTION, never
 * stamp a delayed grant with the current source epoch at delivery. A zero
 * grant replaces the current contribution; it does not mean cable detach.
 */
int d2365_source_host(struct d2365_source *source, struct d2365_source_token token,
		     u64 sequence, unsigned int ma);
/* Qualified source status, not an unqualified 0189/018a online boolean.
 * In this restricted profile usb=false establishes battery classification;
 * any other/unknown source class must invalidate the reducer. A
 * failed physical read must invalidate the source instead of calling false.
 */
int d2365_source_sample(struct d2365_source *source, u64 generation, bool usb,
		       u64 event_epoch, struct cbtl_dock_events *dock, u64 now_ns);
/* Actual backend record update under the same source-owner lock. The caller
 * establishes initialized zero backoff; this is not a charging-control API.
 */
int d2365_source_backend(struct d2365_source *source, struct d2365_source_token token,
			unsigned int allocated_ma, u64 protection_deadline_ns);
/* Invoke only after the owner has actually reset the backend latches. A
 * false physical sample is not itself proof that this reset completed.
 */
int d2365_source_backend_reset(struct d2365_source *source, struct d2365_source_token token);
/* Run after EVERY successful producer transaction (host/backend/reset/Dock)
 * and at the returned deadline. sample() also refreshes before returning. Updates the
 * Dock source epoch when effective status changes and prunes old host grants.
 */
int d2365_source_refresh(struct d2365_source *source,
			  struct cbtl_dock_events *dock, u64 now_ns);
/* Owner must first exclude/drain grant, status, backend, Dock and timer
 * producers. Freezes one coherent terminal snapshot; it does not drain them.
 */
int d2365_source_freeze(struct d2365_source *source,
			 struct cbtl_dock_events *dock, u64 now_ns);
int d2365_source_snapshot(const struct d2365_source *source, struct d2365_sources *snapshot);
struct d2365_source_token d2365_source_token(const struct d2365_source *source);
u64 d2365_source_deadline(const struct d2365_source *source,
			 const struct cbtl_dock_events *dock);

#endif
