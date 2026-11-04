/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_APPLE_CBTL_IDBUS_H
#define _LINUX_POWER_APPLE_CBTL_IDBUS_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t u8;
typedef uint64_t u64;
#endif

#define CBTL_IDBUS_MAX_BYTES 64
#define CBTL_IDBUS_MAX_JOBS 8
#define CBTL_IDBUS_TIMEOUT_NS 100000000ULL

/* Caller owns one controller and serializes EVERY API, cached capability read,
 * mux change, IRQ, timer and lifecycle transition with the same sleepable lock.
 * No API schedules callbacks or retains caller buffers. Completion records are
 * copied out under that lock; an external consumer must recheck the generation
 * before publishing them. This core has no hardware caller or board enablement.
 */
struct cbtl_idbus_io {
	void *context;
	int (*read)(void *, u8 reg, u8 *data, unsigned int count);
	int (*write)(void *, u8 reg, const u8 *data, unsigned int count);
	/* Disable controller idle inhibit; preserve unrelated register 05 bits. */
	int (*prepare)(void *);
	/* Sample after the successful start write, not before synchronous I/O. */
	u64 (*now_ns)(void *);
	/* Acknowledge/drain the successful completion before another start.
	 * Return failure if stale untagged IRQ/status cannot be excluded.
	 */
	int (*retire)(void *);
	/* Abort AND reset/drain old IRQ/status before returning success. An error
	 * leaves hardware ownership unresolved: the core must not start another
	 * job, free its active record or claim cancellation has completed.
	 */
	int (*recover)(void *);
};

enum cbtl_idbus_route {
	CBTL_IDBUS_ROUTE_NONE,
	CBTL_IDBUS_ROUTE_USB0,
	CBTL_IDBUS_ROUTE_UART0,
	CBTL_IDBUS_ROUTE_MBUS,
	CBTL_IDBUS_ROUTE_USBC,
	CBTL_IDBUS_ROUTE_UART2,
};

struct cbtl_idbus_cable {
	enum cbtl_idbus_route first;
	enum cbtl_idbus_route second;
	u8 payload[6]; /* Entire validated 75 payload, not just USB-C/orientation. */
	unsigned int orientation;
	unsigned int mode;
	bool mode_valid;
	bool alternate_rx;
};

struct cbtl_idbus_token {
	u64 generation;
	u64 sequence;
};

struct cbtl_idbus_request {
	u64 owner; /* Stable owner identity; zero is invalid. */
	u64 owner_generation; /* Returned by cbtl_idbus_open_owner. */
	unsigned int command; /* Immutable Dock command state, not an opcode. */
	u8 tx[CBTL_IDBUS_MAX_BYTES];
	unsigned int tx_len;
	unsigned int rx_len;
	u64 timeout_ns;
};

struct cbtl_idbus_result {
	struct cbtl_idbus_token token;
	struct cbtl_idbus_cable cable;
	u64 owner;
	u64 owner_generation;
	unsigned int command;
	int status;
	unsigned int rx_len;
	u8 rx[CBTL_IDBUS_MAX_BYTES]; /* Zeroed on every error. */
};

enum cbtl_idbus_job_state {
	CBTL_IDBUS_FREE,
	CBTL_IDBUS_QUEUED,
	CBTL_IDBUS_ACTIVE,
	CBTL_IDBUS_DONE,
};

struct cbtl_idbus_job {
	struct cbtl_idbus_request request;
	struct cbtl_idbus_result result;
	enum cbtl_idbus_job_state state;
	u64 deadline_ns;
	bool timer_armed;
};

struct cbtl_idbus_owner {
	u64 identity;
	u64 generation;
};

struct cbtl_idbus {
	struct cbtl_idbus_io io;
	struct cbtl_idbus_cable cable;
	struct cbtl_idbus_job jobs[CBTL_IDBUS_MAX_JOBS];
	struct cbtl_idbus_owner owners[CBTL_IDBUS_MAX_JOBS];
	u64 next_owner_generation;
	u64 generation;
	u64 next_sequence;
	int active;
	bool attached;
	bool stopped;
	bool faulted;
};

void cbtl_idbus_init(struct cbtl_idbus *, const struct cbtl_idbus_io *);
/* Bind only after actual ownership, stable validated 75 frame and mode setup.
 * Rebinding requires prior detach and successful active cancellation/drain.
 * Generations and job IDs never wrap; exhaustion permanently rejects work.
 */
int cbtl_idbus_attach(struct cbtl_idbus *, const struct cbtl_idbus_cable *);
int cbtl_idbus_open_owner(struct cbtl_idbus *, u64 owner, u64 *owner_generation);
int cbtl_idbus_submit(struct cbtl_idbus *, const struct cbtl_idbus_request *,
		      struct cbtl_idbus_token *);
int cbtl_idbus_start(struct cbtl_idbus *);
int cbtl_idbus_irq(struct cbtl_idbus *, struct cbtl_idbus_token, u8 irq_status);
int cbtl_idbus_timeout(struct cbtl_idbus *, struct cbtl_idbus_token, u64 now_ns);
/* Invalidates the owner epoch before cancellation, including copied results.
 * The same identity can reopen only with a newly issued owner generation.
 */
int cbtl_idbus_cancel_owner(struct cbtl_idbus *, u64 owner);
int cbtl_idbus_detach(struct cbtl_idbus *);
int cbtl_idbus_stop(struct cbtl_idbus *);
/* Returns 1 with one copied completion, 0 if none; consumes no active record. */
int cbtl_idbus_take_result(struct cbtl_idbus *, struct cbtl_idbus_result *);
/* Internal record drain only. NEVER sufficient to free the owner/context:
 * wrapper must stop and synchronize IRQ/timer/work and copied-result consumers.
 */
bool cbtl_idbus_drained(const struct cbtl_idbus *);
/* Cable validity only, for IRQ/timer association, NOT owner publication. */
bool cbtl_idbus_current(const struct cbtl_idbus *, struct cbtl_idbus_token);
/* Consumer must check under the owner lock immediately before publication. */
bool cbtl_idbus_result_current(const struct cbtl_idbus *, const struct cbtl_idbus_result *);

/* A bounded Dock response/event producer. This is NOT the source reducer or
 * a shutdown-property provider. Unknown live policy never becomes false here.
 * The owner must independently establish command eligibility, lifecycle power
 * mode, special-open membership, source history, routing and current sources.
 */
enum cbtl_dock_handshake {
	CBTL_DOCK_HANDSHAKE_UNKNOWN,
	CBTL_DOCK_HANDSHAKE_COMPLETED,
	CBTL_DOCK_HANDSHAKE_FAILED,
};

struct cbtl_dock_events {
	u64 generation;
	u64 source_epoch;
	bool source_valid;
	bool requested_pin;
	bool ack_pin;
	bool ack_pin_valid;
	bool ack_power;
	bool extended_valid;
	u8 extended[4];
	enum cbtl_dock_handshake handshake;
	bool inflow_pending;
	bool magic_companion;
	u64 magic_deadline_ns;
	u64 magic_expiry_epoch;
	u64 handshake_deadline_ns; /* Persistent source event, not cable epoch. */
	struct cbtl_idbus_token pending;
	unsigned int pending_command;
	bool pending_valid;
};

void cbtl_dock_init(struct cbtl_dock_events *);
/* Invalidates acknowledgements and old response association, preserves request.
 * Source events are not cleared by cable detach: their own expiry governs them.
 */
void cbtl_dock_cable(struct cbtl_dock_events *, u64 generation);
/* Explicitly initialized coherent source snapshot; history is never fabricated. */
int cbtl_dock_source_epoch(struct cbtl_dock_events *, u64 epoch);
/* Lifecycle hook input is numeric accessory mode; 1 is PowerModeOff. */
void cbtl_dock_power_request(struct cbtl_dock_events *, unsigned int mode);
/* Property producer; FAILED does not implicitly clear an existing inflow hold.
 * Complete source-event reduction/notification for this property is separate.
 */
int cbtl_dock_handshake_update(struct cbtl_dock_events *, enum cbtl_dock_handshake);
/* Call before submitting a policy-approved command. This produces events even
 * when subsequent submission fails. MagicPullDown command 4 requires an owned
 * valid source epoch; normal commands 8..11 mark inflow pending.
 */
int cbtl_dock_prepare(struct cbtl_dock_events *, unsigned int command, u64 now_ns);
int cbtl_dock_track(struct cbtl_dock_events *, struct cbtl_idbus_token,
		    unsigned int command);
/* Cancel the exact pending association before revoking its transport owner.
 * Does not undo already-produced source events or acknowledged state.
 */
int cbtl_dock_cancel(struct cbtl_dock_events *, struct cbtl_idbus_token);
/* First check cbtl_idbus_result_current under the shared owner lock.
 * hardware_present and delay are current, validated owner inputs, not PMU
 * ONLINE substitutes. Successful command 6/71 needs this hardware predicate.
 */
int cbtl_dock_complete(struct cbtl_dock_events *, const struct cbtl_idbus_result *,
		       bool hardware_present, u64 now_ns, u64 handshake_delay_ns);
u64 cbtl_dock_deadline(const struct cbtl_dock_events *);

#endif
