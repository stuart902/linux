/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef D2365_REFERENCE_H
#define D2365_REFERENCE_H
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint8_t u8;
#endif

/* Shared sequence core. Hardware integration requires verified providers. */
enum d2365_property {
	D2365_PROPERTY_ABSENT,
	D2365_PROPERTY_FALSE,
	D2365_PROPERTY_TRUE,
	D2365_PROPERTY_UNRESOLVED,
};

enum d2365_terminal {
	D2365_UNRESOLVED,
	D2365_PLATFORM_RESET,
	D2365_STANDBY_WAIT,
};

struct d2365_sources {
	enum d2365_property raw_external;
	enum d2365_property usb_cable;
};

struct d2365_io {
	void *context;
	int (*read)(void *, unsigned int, u8 *, size_t);
	int (*write)(void *, unsigned int, const u8 *, size_t);
	int (*sources)(void *, struct d2365_sources *);
};

struct d2365_result {
	enum d2365_terminal terminal;
	int first_error;
	bool standby_attempted;
	/* Transport callback returned success; NOT physical standby proof. */
	bool standby_acknowledged;
};

/* Callers must establish the ordinary, no-alarm/no-UPO/no-external-controller
 * path. False refuses the run before any I/O; it is not a hardware discovery.
 */
struct d2365_result d2365_reference_shutdown(const struct d2365_io *io,
					   bool normal_path_established);
struct d2365_result d2365_reference_restart(const struct d2365_io *io);

#ifdef __KERNEL__
struct device;
struct regmap;

/* Providers and the PMIC transport must remain callable after device_shutdown().
 * The regmap must not be cache-only, and event/status reads and command writes
 * must reach hardware, never a cache. Device links alone guarantee none of this.
 * The source callback must belong to the bound source provider module
 * (which is pinned), never an unrelated module. No source provider meeting
 * this contract exists yet; the concrete GPIO event owner is used directly.
 */
struct d2365_reference_resources {
	struct device *source_dev;
	int (*snapshot_sources)(struct device *, struct d2365_sources *);
	/* Bound apple,d2365-pmic-gpio child of this regmap's PMIC. The adapter
	 * uses its checked event API to stop/drain IRQ/work and leave exclusive
	 * read-to-clear ownership with the core until reset/off.
	 */
	struct device *event_dev;
	/* Explicit integration precondition, not hardware autodetection. */
	bool ordinary_path_verified;
};

/* J71b only. Call from a consumer probe with valid references to providers,
 * and fail that probe if registration fails.
 * Managed device links follow the consumer driver lifecycle.
 */
int devm_d2365_reference_register(struct device *dev, struct regmap *regmap,
				const struct d2365_reference_resources *resources);
#endif
#endif
