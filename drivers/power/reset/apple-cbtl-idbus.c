// SPDX-License-Identifier: GPL-2.0-only
/* Independently implemented, unwired CBTL transaction owner and Dock producer.
 * Behavioral evidence and integration limits: d2365_reference/README.
 */
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#else
#include <errno.h>
#include <string.h>
#endif
#include <linux/power/apple-cbtl-idbus.h>

#define CBTL_U64_MAX (~(u64)0)

static bool cbtl_same_token(struct cbtl_idbus_token a, struct cbtl_idbus_token b)
{
	return a.generation == b.generation && a.sequence == b.sequence;
}

static u8 cbtl_idbus_crc(const u8 *data, unsigned int count)
{
	u8 crc = 0xff;
	unsigned int i, bit;

	for (i = 0; i < count; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}
	return crc;
}

void cbtl_idbus_init(struct cbtl_idbus *bus, const struct cbtl_idbus_io *io)
{
	memset(bus, 0, sizeof(*bus));
	if (io)
		bus->io = *io;
	bus->active = -1;
}

static bool cbtl_route_valid(enum cbtl_idbus_route route)
{
	return route >= CBTL_IDBUS_ROUTE_NONE && route <= CBTL_IDBUS_ROUTE_UART2;
}

int cbtl_idbus_attach(struct cbtl_idbus *bus, const struct cbtl_idbus_cable *cable)
{
	if (!cable || !bus->io.read || !bus->io.write ||
	    !bus->io.prepare || !bus->io.recover || !bus->io.now_ns ||
	    !bus->io.retire || !cable->mode_valid ||
	    (cable->orientation != 1 && cable->orientation != 2) ||
	    !cbtl_route_valid(cable->first) || !cbtl_route_valid(cable->second))
		return -EINVAL;
	if (bus->stopped || bus->faulted)
		return -ESHUTDOWN;
	if (bus->attached || bus->active >= 0)
		return -EBUSY;
	if (bus->generation == CBTL_U64_MAX)
		return -EOVERFLOW;
	bus->generation++;
	bus->cable = *cable;
	bus->attached = true;
	return 0;
}

static bool cbtl_owner_current(const struct cbtl_idbus *bus, u64 owner, u64 generation)
{
	unsigned int i;

	if (!owner || !generation)
		return false;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++)
		if (bus->owners[i].identity == owner && bus->owners[i].generation == generation)
			return true;
	return false;
}

int cbtl_idbus_open_owner(struct cbtl_idbus *bus, u64 owner, u64 *generation)
{
	struct cbtl_idbus_owner *slot = NULL;
	unsigned int i;

	if (!owner || !generation)
		return -EINVAL;
	if (bus->stopped || bus->faulted)
		return -ESHUTDOWN;
	if (!bus->attached)
		return -ENODEV;
	if (bus->next_owner_generation == CBTL_U64_MAX)
		return -EOVERFLOW;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++) {
		if (bus->owners[i].identity == owner)
			return -EEXIST;
		if (!bus->owners[i].identity && !slot)
			slot = &bus->owners[i];
	}
	if (!slot)
		return -ENOSPC;
	slot->identity = owner;
	slot->generation = ++bus->next_owner_generation;
	*generation = slot->generation;
	return 0;
}

int cbtl_idbus_submit(struct cbtl_idbus *bus, const struct cbtl_idbus_request *req,
		      struct cbtl_idbus_token *token)
{
	struct cbtl_idbus_job *job = NULL;
	unsigned int i;

	if (!req || !token || !req->owner || !req->tx_len ||
	    req->tx_len > CBTL_IDBUS_MAX_BYTES || !req->rx_len ||
	    req->rx_len > CBTL_IDBUS_MAX_BYTES || !req->timeout_ns)
		return -EINVAL;
	if (bus->stopped || bus->faulted)
		return -ESHUTDOWN;
	if (!bus->attached)
		return -ENODEV;
	if (!cbtl_owner_current(bus, req->owner, req->owner_generation))
		return -ESTALE;
	if (bus->next_sequence == CBTL_U64_MAX)
		return -EOVERFLOW;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++)
		if (bus->jobs[i].state == CBTL_IDBUS_FREE) {
			job = &bus->jobs[i];
			break;
		}
	if (!job)
		return -ENOSPC;
	memset(job, 0, sizeof(*job));
	job->request = *req;
	job->result.token.generation = bus->generation;
	job->result.token.sequence = ++bus->next_sequence;
	job->result.cable = bus->cable;
	job->result.owner = req->owner;
	job->result.owner_generation = req->owner_generation;
	job->result.command = req->command;
	job->state = CBTL_IDBUS_QUEUED;
	*token = job->result.token;
	return 0;
}

static void cbtl_finish(struct cbtl_idbus *bus, struct cbtl_idbus_job *job, int status)
{
	job->result.status = status;
	if (status) {
		memset(job->result.rx, 0, sizeof(job->result.rx));
		job->result.rx_len = 0;
	}
	job->timer_armed = false;
	job->state = CBTL_IDBUS_DONE;
	if (bus->active >= 0 && &bus->jobs[bus->active] == job)
		bus->active = -1;
}

static int cbtl_recover_finish(struct cbtl_idbus *bus, int status)
{
	struct cbtl_idbus_job *job = &bus->jobs[bus->active];
	int error = bus->io.recover(bus->io.context);

	if (error) {
		/* No successful cancellation, no synthetic callback and no reuse. */
		bus->faulted = true;
		job->timer_armed = false;
		return error;
	}
	cbtl_finish(bus, job, status);
	return 0;
}

int cbtl_idbus_start(struct cbtl_idbus *bus)
{
	struct cbtl_idbus_job *job = NULL;
	u8 staging[CBTL_IDBUS_MAX_BYTES + 2], verify[CBTL_IDBUS_MAX_BYTES + 1];
	unsigned int i, index = 0, len;
	u8 reg;
	u64 now_ns;
	int error;

	if (bus->stopped || bus->faulted)
		return -ESHUTDOWN;
	if (!bus->attached)
		return -ENODEV;
	if (bus->active >= 0)
		return -EBUSY;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++) {
		if (bus->jobs[i].state != CBTL_IDBUS_QUEUED)
			continue;
		if (!job || bus->jobs[i].result.token.sequence < job->result.token.sequence) {
			job = &bus->jobs[i];
			index = i;
		}
	}
	if (!job)
		return -ENOENT;
	if (job->result.token.generation != bus->generation) {
		cbtl_finish(bus, job, -ESTALE);
		return -ESTALE;
	}
	len = job->request.tx_len;
	reg = 0x60 - len;
	for (i = 0; i < len; i++)
		staging[i] = job->request.tx[len - i - 1];
	staging[len] = job->result.cable.alternate_rx ?
		(2 * job->request.rx_len) | 0x80 : 2 * job->request.rx_len - 2;
	staging[len + 1] = 1 + 2 * (len - 1);
	bus->active = index;
	job->state = CBTL_IDBUS_ACTIVE;
	error = bus->io.prepare(bus->io.context);
	if (error)
		goto fail;
	if (!(job->result.cable.mode & 0xf0)) {
		error = bus->io.write(bus->io.context, reg, staging, len + 1);
		if (error)
			goto fail;
		error = bus->io.read(bus->io.context, reg, verify, len + 1);
		if (!error && memcmp(staging, verify, len + 1))
			error = -EIO;
		if (error)
			goto fail;
		error = bus->io.write(bus->io.context, 0x61, &staging[len + 1], 1);
	} else {
		error = bus->io.write(bus->io.context, reg, staging, len + 2);
	}
	if (error)
		goto fail;
	now_ns = bus->io.now_ns(bus->io.context);
	if (now_ns > CBTL_U64_MAX - job->request.timeout_ns) {
		error = -EOVERFLOW;
		goto fail;
	}
	job->deadline_ns = now_ns + job->request.timeout_ns;
	job->timer_armed = true;
	return 0;
fail:
	/* A failed I2C start may already have committed: recover before reuse. */
	if (cbtl_recover_finish(bus, error))
		return -EIO;
	return error;
}

bool cbtl_idbus_current(const struct cbtl_idbus *bus, struct cbtl_idbus_token token)
{
	return bus->attached && !bus->stopped && !bus->faulted &&
	       token.generation == bus->generation && token.sequence != 0;
}

bool cbtl_idbus_result_current(const struct cbtl_idbus *bus,
			       const struct cbtl_idbus_result *result)
{
	return result && cbtl_idbus_current(bus, result->token) &&
	       cbtl_owner_current(bus, result->owner, result->owner_generation);
}

static struct cbtl_idbus_job *cbtl_active(struct cbtl_idbus *bus,
					struct cbtl_idbus_token token)
{
	struct cbtl_idbus_job *job;

	if (!cbtl_idbus_current(bus, token) || bus->active < 0)
		return NULL;
	job = &bus->jobs[bus->active];
	if (job->state != CBTL_IDBUS_ACTIVE || !cbtl_same_token(token, job->result.token))
		return NULL;
	return job;
}

int cbtl_idbus_irq(struct cbtl_idbus *bus, struct cbtl_idbus_token token, u8 status)
{
	struct cbtl_idbus_job *job = cbtl_active(bus, token);
	u8 packet[CBTL_IDBUS_MAX_BYTES + 1];
	int error;

	if (!job)
		return -ESTALE;
	if (!(status & 0x38))
		return -EAGAIN;
	if ((status & 0x38) != 0x10) {
		error = -EIO;
		goto fail;
	}
	error = bus->io.read(bus->io.context, 0x20, packet, job->request.rx_len + 1);
	if (!error && cbtl_idbus_crc(packet, job->request.rx_len) != packet[job->request.rx_len])
		error = -EBADMSG;
	if (error)
		goto fail;
	error = bus->io.retire(bus->io.context);
	if (error)
		goto fail;
	memcpy(job->result.rx, packet, job->request.rx_len);
	job->result.rx_len = job->request.rx_len;
	cbtl_finish(bus, job, 0);
	return 0;
fail:
	if (cbtl_recover_finish(bus, error))
		return -EIO;
	return error;
}

int cbtl_idbus_timeout(struct cbtl_idbus *bus, struct cbtl_idbus_token token, u64 now_ns)
{
	struct cbtl_idbus_job *job = cbtl_active(bus, token);

	if (!job)
		return -ESTALE;
	if (!job->timer_armed || now_ns < job->deadline_ns)
		return -EAGAIN;
	return cbtl_recover_finish(bus, -ETIMEDOUT);
}

static int cbtl_cancel(struct cbtl_idbus *bus, u64 owner)
{
	unsigned int i;
	int error = 0;

	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++) {
		struct cbtl_idbus_job *job = &bus->jobs[i];

		if (owner && job->request.owner != owner)
			continue;
		if (job->state == CBTL_IDBUS_QUEUED)
			cbtl_finish(bus, job, -ECANCELED);
	}
	if (bus->active >= 0 && (!owner || bus->jobs[bus->active].request.owner == owner))
		error = cbtl_recover_finish(bus, -ECANCELED);
	return error;
}

int cbtl_idbus_cancel_owner(struct cbtl_idbus *bus, u64 owner)
{
	unsigned int i;

	if (!owner)
		return -EINVAL;
	/* Revoke even already-copied completions before attempting abort. */
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++) {
		if (bus->owners[i].identity == owner)
			memset(&bus->owners[i], 0, sizeof(bus->owners[i]));
		if (bus->jobs[i].state == CBTL_IDBUS_DONE &&
		    bus->jobs[i].result.owner == owner)
			cbtl_finish(bus, &bus->jobs[i], -ECANCELED);
	}
	return cbtl_cancel(bus, owner);
}

int cbtl_idbus_detach(struct cbtl_idbus *bus)
{
	bus->attached = false;
	/* Invalidation happens before abort, including its error path. */
	if (bus->generation != CBTL_U64_MAX)
		bus->generation++;
	memset(&bus->cable, 0, sizeof(bus->cable));
	memset(bus->owners, 0, sizeof(bus->owners));
	return cbtl_cancel(bus, 0);
}

int cbtl_idbus_stop(struct cbtl_idbus *bus)
{
	bus->stopped = true;
	return cbtl_idbus_detach(bus);
}

int cbtl_idbus_take_result(struct cbtl_idbus *bus, struct cbtl_idbus_result *result)
{
	struct cbtl_idbus_job *job = NULL;
	unsigned int i;

	if (!result)
		return -EINVAL;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++) {
		if (bus->jobs[i].state != CBTL_IDBUS_DONE)
			continue;
		if (!job || bus->jobs[i].result.token.sequence < job->result.token.sequence)
			job = &bus->jobs[i];
	}
	if (!job)
		return 0;
	*result = job->result;
	memset(job, 0, sizeof(*job));
	return 1;
}

bool cbtl_idbus_drained(const struct cbtl_idbus *bus)
{
	unsigned int i;

	if (!bus->stopped || bus->active >= 0)
		return false;
	for (i = 0; i < CBTL_IDBUS_MAX_JOBS; i++)
		if (bus->jobs[i].state != CBTL_IDBUS_FREE)
			return false;
	return true;
}

void cbtl_dock_init(struct cbtl_dock_events *dock)
{
	memset(dock, 0, sizeof(*dock));
}

void cbtl_dock_cable(struct cbtl_dock_events *dock, u64 generation)
{
	dock->generation = generation;
	dock->ack_pin = false;
	dock->ack_pin_valid = false;
	dock->ack_power = false;
	dock->extended_valid = false;
	memset(dock->extended, 0, sizeof(dock->extended));
	dock->handshake = CBTL_DOCK_HANDSHAKE_UNKNOWN;
	dock->pending_valid = false;
}

int cbtl_dock_source_epoch(struct cbtl_dock_events *dock, u64 epoch)
{
	if (dock->source_valid && epoch < dock->source_epoch)
		return -ESTALE;
	dock->source_epoch = epoch;
	dock->source_valid = true;
	if (dock->magic_expiry_epoch < epoch) {
		dock->magic_deadline_ns = 0;
		dock->magic_companion = false;
	}
	return 0;
}

void cbtl_dock_power_request(struct cbtl_dock_events *dock, unsigned int mode)
{
	dock->requested_pin = mode != 1;
}

int cbtl_dock_handshake_update(struct cbtl_dock_events *dock,
			       enum cbtl_dock_handshake state)
{
	if (state < CBTL_DOCK_HANDSHAKE_UNKNOWN || state > CBTL_DOCK_HANDSHAKE_FAILED)
		return -EINVAL;
	dock->handshake = state;
	if (state == CBTL_DOCK_HANDSHAKE_UNKNOWN)
		dock->inflow_pending = true;
	else if (state == CBTL_DOCK_HANDSHAKE_COMPLETED)
		dock->inflow_pending = false;
	return 0;
}

static bool cbtl_dock_command(unsigned int command)
{
	return command == 3 || command == 4 || command == 5 || command == 6 ||
	       (command >= 8 && command <= 11);
}

int cbtl_dock_prepare(struct cbtl_dock_events *dock, unsigned int command, u64 now_ns)
{
	if (!dock->generation || !cbtl_dock_command(command))
		return -EINVAL;
	if (dock->pending_valid)
		return -EBUSY;
	if (command == 4) {
		if (!dock->source_valid)
			return -ENODATA;
		if (now_ns > CBTL_U64_MAX - 2000000000ULL)
			return -EOVERFLOW;
		dock->magic_deadline_ns = now_ns + 2000000000ULL;
		dock->magic_expiry_epoch = dock->source_epoch;
		dock->magic_companion = true;
	} else if (command >= 8 && command <= 11) {
		dock->inflow_pending = true;
	}
	return 0;
}

int cbtl_dock_track(struct cbtl_dock_events *dock, struct cbtl_idbus_token token,
		    unsigned int command)
{
	if (!token.sequence || token.generation != dock->generation)
		return -ESTALE;
	if (!cbtl_dock_command(command))
		return -EINVAL;
	if (dock->pending_valid)
		return -EBUSY;
	dock->pending = token;
	dock->pending_command = command;
	dock->pending_valid = true;
	return 0;
}

int cbtl_dock_cancel(struct cbtl_dock_events *dock, struct cbtl_idbus_token token)
{
	if (!dock->pending_valid || !cbtl_same_token(token, dock->pending))
		return -ESTALE;
	dock->pending_valid = false;
	return 0;
}

int cbtl_dock_complete(struct cbtl_dock_events *dock, const struct cbtl_idbus_result *result,
		       bool hardware_present, u64 now_ns, u64 handshake_delay_ns)
{
	unsigned int command;

	if (!result || !dock->pending_valid || result->token.generation != dock->generation ||
	    !cbtl_same_token(result->token, dock->pending) ||
	    result->command != dock->pending_command)
		return -ESTALE;
	command = result->command;
	dock->pending_valid = false;
	if (result->status)
		return result->status;
	if (command == 3) {
		if (result->rx_len != 5 || result->rx[0] != 0x73)
			return -EPROTO;
		memcpy(dock->extended, &result->rx[1], sizeof(dock->extended));
		dock->extended_valid = true;
		return 0;
	}
	if (result->rx_len != 1 || result->rx[0] != 0x71)
		return -EPROTO;
	if (command == 6) {
		if (hardware_present && now_ns > CBTL_U64_MAX - handshake_delay_ns)
			return -EOVERFLOW;
		dock->ack_power = false;
		if (hardware_present) {
			dock->magic_companion = false;
			dock->handshake_deadline_ns = now_ns + handshake_delay_ns;
		}
	} else if (command >= 8 && command <= 11) {
		dock->ack_power = command == 8 || command == 9;
		dock->ack_pin = command == 9 || command == 10;
		dock->ack_pin_valid = true;
	}
	return 0;
}

u64 cbtl_dock_deadline(const struct cbtl_dock_events *dock)
{
	return dock->magic_deadline_ns > dock->handshake_deadline_ns ?
		dock->magic_deadline_ns : dock->handshake_deadline_ns;
}
