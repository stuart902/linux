// SPDX-License-Identifier: GPL-2.0-only
/* T8010 AOP audio property client; no board enables this by default. */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/apple/aop.h>
#include <linux/soc/apple/aop-audio.h>

#include "aop-audio-clock.h"

struct apple_aop_pdm_clock {
	struct kref refs;
	struct list_head registry;
	struct apple_aop *owner;
	struct device *dev;
	struct mutex lock;
	struct apple_aop_service service;
	struct aop_audio_clock clock;
	bool attached;
	bool claimed;
};

static struct platform_driver apple_aop_audio_driver;
static DEFINE_MUTEX(apple_aop_audio_registry_lock);
static LIST_HEAD(apple_aop_audio_registry);

static void apple_aop_audio_free(struct kref *refs)
{
	struct apple_aop_pdm_clock *clock =
		container_of(refs, struct apple_aop_pdm_clock, refs);

	mutex_lock(&apple_aop_audio_registry_lock);
	list_del(&clock->registry);
	mutex_unlock(&apple_aop_audio_registry_lock);
	put_device(clock->dev);
	kfree(clock);
}

struct apple_aop_pdm_clock *apple_aop_pdm_clock_get(struct device *consumer)
{
	struct apple_aop_pdm_clock *clock;
	struct platform_device *pdev;
	struct device_node *node;
	int ret;

	node = of_parse_phandle(consumer->of_node, "apple,aop-audio", 0);
	if (!node)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	device_lock(&pdev->dev);
	ret = -EPROBE_DEFER;
	if (pdev->dev.driver != &apple_aop_audio_driver.driver)
		goto unlock_device;
	clock = platform_get_drvdata(pdev);
	mutex_lock(&clock->lock);
	if (!clock->attached) {
		ret = -ENODEV;
		goto unlock_clock;
	}
	if (clock->claimed) {
		ret = -EBUSY;
		goto unlock_clock;
	}
	if (!try_module_get(THIS_MODULE)) {
		ret = -ENODEV;
		goto unlock_clock;
	}
	if (!device_link_add(consumer, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		module_put(THIS_MODULE);
		ret = -ENOMEM;
		goto unlock_clock;
	}
	clock->claimed = true;
	kref_get(&clock->refs);
	mutex_unlock(&clock->lock);
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return clock;

unlock_clock:
	mutex_unlock(&clock->lock);
unlock_device:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(apple_aop_pdm_clock_get);

static int apple_aop_audio_exchange(void *cookie, const void *request,
				    size_t request_len, void *reply,
				    size_t capacity, size_t *actual)
{
	struct apple_aop_pdm_clock *clock = cookie;

	return apple_aop_call_inline(&clock->service, AOP_AUDIO_SUBTYPE,
				     request, request_len, reply, capacity, actual);
}

static int apple_aop_audio_program(struct apple_aop_pdm_clock *clock, u32 rate)
{
	int ret;

	mutex_lock(&clock->lock);
	if (!clock->attached)
		ret = -ENODEV;
	else if (!clock->claimed)
		ret = -EINVAL;
	else
		ret = aop_audio_clock_program(&clock->clock, rate,
					      apple_aop_audio_exchange, clock);
	mutex_unlock(&clock->lock);
	return ret;
}

int apple_aop_pdm_clock_start(struct apple_aop_pdm_clock *clock, u32 rate)
{
	if (!rate)
		return -EINVAL;
	return apple_aop_audio_program(clock, rate);
}
EXPORT_SYMBOL_GPL(apple_aop_pdm_clock_start);

int apple_aop_pdm_clock_stop(struct apple_aop_pdm_clock *clock)
{
	return apple_aop_audio_program(clock, 0);
}
EXPORT_SYMBOL_GPL(apple_aop_pdm_clock_stop);

bool apple_aop_pdm_clock_needs_retention(struct apple_aop_pdm_clock *clock)
{
	bool retain;

	mutex_lock(&clock->lock);
	retain = clock->clock.state == AOP_AUDIO_CLOCK_ON ||
		 clock->clock.state == AOP_AUDIO_CLOCK_FAULTED;
	mutex_unlock(&clock->lock);
	return retain;
}
EXPORT_SYMBOL_GPL(apple_aop_pdm_clock_needs_retention);

int apple_aop_pdm_clock_put(struct apple_aop_pdm_clock *clock)
{
	int ret = 0;

	mutex_lock(&clock->lock);
	if (!clock->claimed)
		ret = -EINVAL;
	else if (clock->clock.state == AOP_AUDIO_CLOCK_ON ||
		 clock->clock.state == AOP_AUDIO_CLOCK_FAULTED)
		ret = -EBUSY;
	else
		clock->claimed = false;
	mutex_unlock(&clock->lock);
	if (!ret) {
		kref_put(&clock->refs, apple_aop_audio_free);
		module_put(THIS_MODULE);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_aop_pdm_clock_put);

static int apple_aop_audio_probe(struct platform_device *pdev)
{
	const struct apple_aop_service *service = dev_get_platdata(&pdev->dev);
	struct apple_aop_pdm_clock *clock, *other;

	if (!service || !service->aop || strcmp(service->name, "aop-audio"))
		return -ENODEV;
	clock = kzalloc_obj(*clock);
	if (!clock)
		return -ENOMEM;
	kref_init(&clock->refs);
	mutex_init(&clock->lock);
	clock->dev = get_device(&pdev->dev);
	clock->service = *service;
	clock->owner = service->aop;
	clock->attached = true;
	/* A repeated announcement must not create another owner for pdm0. */
	mutex_lock(&apple_aop_audio_registry_lock);
	list_for_each_entry(other, &apple_aop_audio_registry, registry) {
		if (other->owner == clock->owner) {
			mutex_unlock(&apple_aop_audio_registry_lock);
			put_device(clock->dev);
			kfree(clock);
			return -EBUSY;
		}
	}
	list_add(&clock->registry, &apple_aop_audio_registry);
	mutex_unlock(&apple_aop_audio_registry_lock);
	/* UNKNOWN until an explicit OFF transaction; probe changes no hardware. */
	platform_set_drvdata(pdev, clock);
	return 0;
}

static void apple_aop_audio_detach(struct platform_device *pdev)
{
	struct apple_aop_pdm_clock *clock = platform_get_drvdata(pdev);

	/* Drain any in-flight call before the AOP parent can halt its firmware. */
	mutex_lock(&clock->lock);
	clock->attached = false;
	clock->service.aop = NULL;
	if (clock->clock.state == AOP_AUDIO_CLOCK_ON) {
		clock->clock.state = AOP_AUDIO_CLOCK_FAULTED;
		dev_err(clock->dev, "PDM clock detached while active; ownership retained\n");
	}
	mutex_unlock(&clock->lock);
}

static void apple_aop_audio_remove(struct platform_device *pdev)
{
	struct apple_aop_pdm_clock *clock = platform_get_drvdata(pdev);

	apple_aop_audio_detach(pdev);
	/* A claimed or faulted consumer keeps its separate reference. */
	kref_put(&clock->refs, apple_aop_audio_free);
}

static const struct of_device_id apple_aop_audio_match[] = {
	{ .compatible = "apple,t8010-aop-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_aop_audio_match);

static struct platform_driver apple_aop_audio_driver = {
	.driver = {
		.name = "apple-aop-audio",
		.of_match_table = apple_aop_audio_match,
		.suppress_bind_attrs = true,
	},
	.probe = apple_aop_audio_probe,
	.remove = apple_aop_audio_remove,
	.shutdown = apple_aop_audio_detach,
};
module_platform_driver(apple_aop_audio_driver);

MODULE_DESCRIPTION("Apple T8010 AOP PDM clock property service");
MODULE_LICENSE("GPL");
