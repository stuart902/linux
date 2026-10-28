/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_SIO_H_
#define _LINUX_SOC_APPLE_SIO_H_

#include <linux/types.h>

struct apple_sio;
struct apple_sio_async;

/* Endpoint numbers are 8 bits; the firmware uses 0x00..0x81. */
#define APPLE_SIO_MAX_ENDPOINTS		0x82

/* Request tags are allocated per endpoint in 1..APPLE_SIO_MAX_TAG. */
#define APPLE_SIO_MAX_TAG		31

typedef void (*apple_sio_async_callback_t)(void *data, int status,
					   u64 response);
typedef void (*apple_sio_notify_t)(void *data, u64 message);

void *apple_sio_desc0_alloc(struct apple_sio *sio, unsigned int slots,
			    u32 *first_slot);
void apple_sio_desc0_free(struct apple_sio *sio, u32 first_slot,
			  unsigned int slots);

struct apple_sio_async *
apple_sio_request_async(struct apple_sio *sio, u8 endpoint, u8 opcode,
			u8 parameter, u32 data, u8 expected_opcode,
			apple_sio_async_callback_t callback,
			void *callback_data);
struct apple_sio_async *
apple_sio_request_async_atomic(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data);
int apple_sio_async_cancel(struct apple_sio_async *request);
void apple_sio_async_put(struct apple_sio_async *request);

int apple_sio_request(struct apple_sio *sio, u8 endpoint, u8 opcode,
		      u8 parameter, u32 data, u8 expected_opcode,
		      unsigned long timeout, u64 *response);

/* Fail every outstanding request on @endpoint with -ECANCELED. */
void apple_sio_cancel_endpoint(struct apple_sio *sio, u8 endpoint);

/*
 * Pass messages on @endpoint that match no pending request to @notify.
 * Called from the interrupt handler.
 */
void apple_sio_set_notify(struct apple_sio *sio, u8 endpoint,
			  apple_sio_notify_t notify, void *data);

#endif
