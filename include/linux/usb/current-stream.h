/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_USB_CURRENT_STREAM_H
#define _LINUX_USB_CURRENT_STREAM_H

#include <linux/usb/current.h>
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#else
#include <errno.h>
#include <string.h>
#endif

#define USB_CURRENT_QUEUE_SIZE 16

/* Caller serializes every operation. Loss has a reserved out-of-band record:
 * a full FIFO must never silently turn into a valid latest-current snapshot.
 * Only a newly owned session may initialize a lost queue again.
 */
struct usb_current_queue {
	struct usb_current_event events[USB_CURRENT_QUEUE_SIZE];
	struct usb_current_event loss;
	unsigned int head;
	unsigned int count;
	bool lost;
	bool loss_pending;
};

static inline void usb_current_queue_init(struct usb_current_queue *queue)
{
	memset(queue, 0, sizeof(*queue));
}

static inline void usb_current_queue_lose(struct usb_current_queue *queue,
					 const struct usb_current_event *event)
{
	if (queue->lost)
		return;
	queue->head = 0;
	queue->count = 0;
	queue->lost = true;
	queue->loss_pending = true;
	queue->loss = *event;
	queue->loss.reason = USB_CURRENT_LOST;
	queue->loss.milliamps = 0;
}

static inline int usb_current_queue_push(struct usb_current_queue *queue,
					const struct usb_current_event *event)
{
	if (queue->lost)
		return -EPIPE;
	if (event->reason == USB_CURRENT_LOST || queue->count == USB_CURRENT_QUEUE_SIZE) {
		usb_current_queue_lose(queue, event);
		return -ENOSPC;
	}
	queue->events[(queue->head + queue->count) % USB_CURRENT_QUEUE_SIZE] = *event;
	queue->count++;
	return 0;
}

static inline bool usb_current_queue_take(struct usb_current_queue *queue,
					 struct usb_current_event *event)
{
	if (queue->loss_pending) {
		*event = queue->loss;
		queue->loss_pending = false;
		return true;
	}
	if (!queue->count)
		return false;
	*event = queue->events[queue->head];
	queue->head = (queue->head + 1) % USB_CURRENT_QUEUE_SIZE;
	queue->count--;
	return true;
}

#endif
