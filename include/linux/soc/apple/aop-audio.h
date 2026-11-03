/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_APPLE_AOP_AUDIO_H
#define _LINUX_SOC_APPLE_AOP_AUDIO_H

#include <linux/types.h>

struct device;
struct apple_aop_pdm_clock;

/*
 * Claim the PDM clock through the consumer's apple,aop-audio phandle.
 * Exactly one consumer may own it. This does not send a firmware command.
 * The board integration must establish exclusive host HP-request ownership;
 * discovery of aop-audio alone does not establish that precondition.
 */
struct apple_aop_pdm_clock *apple_aop_pdm_clock_get(struct device *consumer);

/*
 * All operations sleep. ON/OFF describe acknowledged host high-power requests,
 * not physical clock state: firmware may retain a low-power audio request.
 * Firmware SET acknowledgement does not report all hardware configuration
 * failures, and GET returns a domain identifier, not a hardware-state readback.
 * STOP establishes initial UNKNOWN -> OFF after host ownership is established.
 * START is allowed only after OFF; changing an active rate requires STOP first.
 *
 * A failure may mean firmware executed the command. If needs_retention() is
 * true, retain the handle/provider claim and do not retry commands. Independently
 * stop the receiver and establish actual DMA quiescence before releasing any
 * AP resources. An acknowledged STOP is not a DMA fence. Recovery from uncertain
 * request ownership requires a verified device reboot.
 */
int apple_aop_pdm_clock_start(struct apple_aop_pdm_clock *clock, u32 rate);
int apple_aop_pdm_clock_stop(struct apple_aop_pdm_clock *clock);
bool apple_aop_pdm_clock_needs_retention(struct apple_aop_pdm_clock *clock);

/*
 * Caller must serialize PUT against all operations on this handle, including
 * needs_retention(). Success invalidates the handle. Returns -EBUSY without
 * releasing ownership if ON or uncertain; the caller must retain that handle.
 */
int apple_aop_pdm_clock_put(struct apple_aop_pdm_clock *clock);

#endif
