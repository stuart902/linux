/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple Always-On Processor (AOP) services
 */
#ifndef _LINUX_SOC_APPLE_AOP_H_
#define _LINUX_SOC_APPLE_AOP_H_

#include <linux/types.h>

struct apple_aop;

/*
 * An EPIC service announced by the AOP firmware. The AOP driver registers a
 * child platform device for each service that has a device tree child node
 * of the same name and passes this as its platform data.
 */
struct apple_aop_service {
	struct apple_aop *aop;
	u8 endpoint;
	u32 channel;
	char name[21];
};

/*
 * Called from the AOP's message work, in process context, for each report of
 * the service; @subtype is its EPIC report subtype (e.g. 0xc2 for motion
 * samples, 0xc4 for fake-HID reports).
 */
typedef void (*apple_aop_report_fn)(void *cookie, u16 subtype, const void *data, size_t len);

int apple_aop_listen(const struct apple_aop_service *service,
		     apple_aop_report_fn report, void *cookie);
void apple_aop_unlisten(const struct apple_aop_service *service);

/*
 * Send a notification to a service and wait for its reply. Returns the
 * service's return code (>= 0) and copies up to @reply_len bytes of the
 * reply's payload, or a negative error.
 */
int apple_aop_call(const struct apple_aop_service *service, u16 subtype,
		   const void *data, size_t len, void *reply, size_t reply_len);

#endif /* _LINUX_SOC_APPLE_AOP_H_ */
