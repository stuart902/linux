/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef D2365_REFERENCE_H
#define D2365_REFERENCE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Offline implementation. No Linux device, I2C or reset adapter is supplied. */
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
	int (*read)(void *, unsigned int, uint8_t *, size_t);
	int (*write)(void *, unsigned int, const uint8_t *, size_t);
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
#endif
