// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO and PMU event interrupt driver for Apple/Dialog PMICs used with the
 * A8X, A9 and A10 SoCs.
 *
 * The D2365 latches 240 PMU events in a read-to-clear block. Any unmasked
 * pending event pulls the PMU interrupt line to the AP low; reading the
 * block acknowledges everything and releases the line. Every PMU client
 * shares that block, so this driver owns it and hands each event to its
 * consumer as an ordinary interrupt. PMIC GPIO line N is event 0x30 + N.
 * One more interrupt, numbered after the last event, is the line itself: its
 * consumers run before the block is read, for a consumer that must act
 * within the shortest time after an assertion (charger withdrawal).
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

/*
 * D2207, as iOS AppleD2207PMU accesses it: the configuration byte of line n
 * is 0x400 + n for lines 0-16 and 0x3e0 + 3 * n for lines 17-20
 * (_index2GPIOAddress). Bits 7:5 are the mode; _setGPIODirection() writes
 * mode 0 for output and mode 3 for input and keeps bits 4:0. Bit 1 is the
 * output level: _setGPIOFunction() drives a line high with
 * (old & 0x1d) | 0x02. Input levels are bit n % 8 of 0x63 + n / 8.
 */
#define D2207_GPIO_SPLIT		17
#define D2207_GPIO_CFG_0_16	0x400
#define D2207_GPIO_CFG_17_20	0x3e0
#define D2207_GPIO_VALUE		BIT(1)
#define D2207_GPIO_MODE_MASK		GENMASK(7, 5)
#define D2207_GPIO_OUTPUT		0
#define D2207_GPIO_INPUT		(3 << 5)

#define D2365_GPIO_SPLIT		18
#define D2365_GPIO_CFG_0_17	0x900
#define D2365_GPIO_CFG_18_23	0x8bd

#define D2365_GPIO_VALUE		BIT(0)
#define D2365_GPIO_INPUT		BIT(3)
#define D2365_GPIO_FUNCTION	BIT(4)

#define D2333_GPIO_VALUE		BIT(0)
#define D2333_GPIO_DIRECTION_MASK	GENMASK(7, 6)
#define D2333_GPIO_INPUT		0
#define D2333_GPIO_OUTPUT	BIT(7)

/*
 * D2365 PMU events, one bit each (event n: byte n / 8, bit n % 8). iOS
 * fetches them by reading all 30 bytes at 0x140 (clear on read); 0x1c0 is
 * the matching interrupt mask (set = masked), which iOS sets to all ones at
 * probe before unmasking what has a subscriber.
 */
#define D2365_EVENTS		0x140
#define D2365_EVENT_MASK	0x1c0
#define D2365_EVENT_BYTES	30
#define D2365_GPIO_EVENT	0x30

#define APPLE_PMIC_EVENT_BYTES_MAX	30
#define APPLE_PMIC_EVENTS_MAX		(APPLE_PMIC_EVENT_BYTES_MAX * 8)
/* The events plus the line interrupt, hwirq nevents. */
#define APPLE_PMIC_IRQS_MAX		(APPLE_PMIC_EVENTS_MAX + 1)

/* More PMU interrupts than this per second means the line is stuck. */
#define APPLE_PMIC_IRQ_BURST	64
#define APPLE_PMIC_IRQ_RETRIES	5

struct apple_pmic_event_info {
	unsigned int status_base;
	unsigned int mask_base;
	unsigned int bytes;
	unsigned int gpio_event;
};

struct apple_pmic_gpio_info {
	const char *label;
	unsigned int ngpio;
	unsigned int input_base;
	unsigned int value_mask;
	unsigned int direction_mask;
	unsigned int input_mode;
	unsigned int output_mode;
	unsigned int function_mask;
	unsigned int (*config_reg)(unsigned int offset);
	const struct apple_pmic_event_info *events;
};

struct apple_pmic_gpio {
	struct gpio_chip chip;
	struct regmap *regmap;
	const struct apple_pmic_gpio_info *info;

	/* PMU event interrupt controller, NULL without one. */
	struct irq_domain *domain;
	unsigned int nevents;
	int parent_irq;
	/* Serializes the mask registers, the event thread and the PM gate. */
	struct mutex irq_lock;
	struct delayed_work irq_retry;
	DECLARE_BITMAP(enabled, APPLE_PMIC_IRQS_MAX);	/* unmasked while awake */
	DECLARE_BITMAP(wake, APPLE_PMIC_EVENTS_MAX);	/* unmasked while asleep */
	u8 mask[APPLE_PMIC_EVENT_BYTES_MAX];		/* as last written */
	u8 boot_mask[APPLE_PMIC_EVENT_BYTES_MAX];
	int wake_delta;			/* parent wake references to apply */
	unsigned long irq_window;
	unsigned int irq_burst;
	unsigned int retries;
	bool suspended;
	bool irq_held;
	bool removing;
};

static unsigned int d2207_gpio_config_reg(unsigned int offset)
{
	if (offset < D2207_GPIO_SPLIT)
		return D2207_GPIO_CFG_0_16 + offset;

	return D2207_GPIO_CFG_17_20 + 3 * offset;
}

static unsigned int d2365_gpio_config_reg(unsigned int offset)
{
	if (offset < D2365_GPIO_SPLIT)
		return D2365_GPIO_CFG_0_17 + 2 * offset;

	return D2365_GPIO_CFG_18_23 + 6 * offset;
}

static const unsigned int d2333_gpio_config_regs[] = {
	0x900, 0x902, 0x904, 0x906, 0x908, 0x90a, 0x90c,
	0x90e, 0x910, 0x912, 0x914, 0x916, 0x918, 0x91a,
	0x91c, 0x91e, 0x920, 0x927, 0x92d, 0x933, 0x939,
};

static unsigned int d2333_gpio_config_reg(unsigned int offset)
{
	return d2333_gpio_config_regs[offset];
}

/* ---- PMU event interrupt controller ---- */

/* Write the mask bytes that differ from @unmasked. Caller holds irq_lock. */
static int apple_pmic_event_sync(struct apple_pmic_gpio *gpio,
				 const unsigned long *unmasked)
{
	const struct apple_pmic_event_info *ev = gpio->info->events;
	unsigned int byte, bit;
	int ret, err = 0;
	u8 mask;

	for (byte = 0; byte < ev->bytes; byte++) {
		mask = 0xff;
		for (bit = 0; bit < 8; bit++)
			if (test_bit(byte * 8 + bit, unmasked))
				mask &= ~BIT(bit);
		if (mask == gpio->mask[byte])
			continue;
		ret = regmap_write(gpio->regmap, ev->mask_base + byte, mask);
		if (ret)
			err = ret;
		else
			gpio->mask[byte] = mask;
	}
	return err;
}

static void apple_pmic_irq_bus_lock(struct irq_data *data)
{
	struct apple_pmic_gpio *gpio = irq_data_get_irq_chip_data(data);

	mutex_lock(&gpio->irq_lock);
}

static void apple_pmic_irq_bus_sync_unlock(struct irq_data *data)
{
	struct apple_pmic_gpio *gpio = irq_data_get_irq_chip_data(data);
	int delta = gpio->wake_delta;
	int ret;

	gpio->wake_delta = 0;
	/* No bus access between our suspend and resume callbacks. */
	if (!gpio->suspended) {
		ret = apple_pmic_event_sync(gpio, gpio->enabled);
		if (ret)
			dev_err_ratelimited(gpio->chip.parent,
					    "event mask write failed: %d\n", ret);
	}
	mutex_unlock(&gpio->irq_lock);

	/*
	 * A cascaded domain has no hierarchical parent for
	 * irq_chip_set_wake_parent(). Count the wake references on the shared
	 * PMU line here, outside the child descriptor lock, so that
	 * suspend_device_irqs() leaves that line armed.
	 */
	for (; delta > 0; delta--) {
		ret = irq_set_irq_wake(gpio->parent_irq, 1);
		if (ret)
			dev_err(gpio->chip.parent,
				"cannot arm the PMU interrupt for wake: %d\n", ret);
	}
	for (; delta < 0; delta++)
		irq_set_irq_wake(gpio->parent_irq, 0);
}

static void apple_pmic_irq_mask(struct irq_data *data)
{
	struct apple_pmic_gpio *gpio = irq_data_get_irq_chip_data(data);

	clear_bit(irqd_to_hwirq(data), gpio->enabled);
}

static void apple_pmic_irq_unmask(struct irq_data *data)
{
	struct apple_pmic_gpio *gpio = irq_data_get_irq_chip_data(data);

	set_bit(irqd_to_hwirq(data), gpio->enabled);
}

static int apple_pmic_irq_set_type(struct irq_data *data, unsigned int type)
{
	/*
	 * Events are latched; what triggers one (for a GPIO line, the edge
	 * configuration the boot firmware left) is not changed here.
	 */
	return type & IRQ_TYPE_LEVEL_MASK ? -EINVAL : 0;
}

/* Called with the bus lock held; the parent follows in sync_unlock. */
static int apple_pmic_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct apple_pmic_gpio *gpio = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);

	/* The line interrupt only runs while the system is awake. */
	if (hwirq >= gpio->nevents)
		return -EINVAL;
	if (on && !test_bit(hwirq, gpio->wake)) {
		__set_bit(hwirq, gpio->wake);
		gpio->wake_delta++;
	} else if (!on && test_bit(hwirq, gpio->wake)) {
		__clear_bit(hwirq, gpio->wake);
		gpio->wake_delta--;
	}
	return 0;
}

static const struct irq_chip apple_pmic_irq_chip = {
	.name = "apple-pmic-event",
	.irq_bus_lock = apple_pmic_irq_bus_lock,
	.irq_bus_sync_unlock = apple_pmic_irq_bus_sync_unlock,
	.irq_mask = apple_pmic_irq_mask,
	.irq_unmask = apple_pmic_irq_unmask,
	.irq_set_type = apple_pmic_irq_set_type,
	.irq_set_wake = apple_pmic_irq_set_wake,
};

static int apple_pmic_irq_map(struct irq_domain *domain, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	struct apple_pmic_gpio *gpio = domain->host_data;

	if (hwirq > gpio->nevents)
		return -EINVAL;

	irq_set_chip_data(irq, gpio);
	irq_set_chip_and_handler(irq, &apple_pmic_irq_chip, handle_simple_irq);
	irq_set_nested_thread(irq, 1);
	irq_set_parent(irq, gpio->parent_irq);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops apple_pmic_irq_domain_ops = {
	.map = apple_pmic_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

/* Called with irq_lock held, including from the event thread. */
static void apple_pmic_irq_hold(struct apple_pmic_gpio *gpio, bool retry)
{
	if (!gpio->irq_held) {
		disable_irq_nosync(gpio->parent_irq);
		gpio->irq_held = true;
	}
	if (!retry)
		return;
	if (gpio->retries < APPLE_PMIC_IRQ_RETRIES) {
		gpio->retries++;
		mod_delayed_work(system_dfl_wq, &gpio->irq_retry,
				 msecs_to_jiffies(100 << gpio->retries));
	} else {
		dev_err(gpio->chip.parent,
			"PMU interrupt held until the next resume\n");
	}
}

static void apple_pmic_irq_retry(struct work_struct *work)
{
	struct apple_pmic_gpio *gpio = container_of(to_delayed_work(work),
						    struct apple_pmic_gpio,
						    irq_retry);

	mutex_lock(&gpio->irq_lock);
	if (!gpio->suspended && !gpio->removing && gpio->irq_held) {
		gpio->irq_held = false;
		enable_irq(gpio->parent_irq);
	}
	mutex_unlock(&gpio->irq_lock);
}

/* May the event thread use the bus? Caller holds irq_lock. */
static bool apple_pmic_event_gate(struct apple_pmic_gpio *gpio)
{
	if (gpio->removing)
		return false;
	if (gpio->suspended) {
		/*
		 * resume_device_irqs() runs before our I2C ancestors resume.
		 * Leave the events latched and the level line held until
		 * ->resume. A wake event in the window before
		 * suspend_device_irqs() must abort the transition, as the armed
		 * interrupt does afterwards.
		 */
		if (!bitmap_empty(gpio->wake, gpio->nevents))
			pm_system_wakeup();
		apple_pmic_irq_hold(gpio, false);
		return false;
	}
	return true;
}

static void apple_pmic_dispatch(struct apple_pmic_gpio *gpio,
				irq_hw_number_t hwirq)
{
	unsigned int child = irq_find_mapping(gpio->domain, hwirq);

	if (child)
		handle_nested_irq(child);
}

/*
 * Per assertion of the PMU line: first the consumers of the line itself,
 * then one read of the event block; every set event with a consumer is then
 * dispatched in ascending event order, so a consumer with several events
 * sees the lower-numbered ones first.
 *
 * When the events are unknown (read error, storm), every unmasked event is
 * dispatched: consumers must treat a spurious call as "something may have
 * changed". An assertion with nothing unmasked latched reaches only the
 * consumers of the line.
 */
static irqreturn_t apple_pmic_event_thread(int irq, void *data)
{
	struct apple_pmic_gpio *gpio = data;
	const struct apple_pmic_event_info *ev = gpio->info->events;
	DECLARE_BITMAP(pending, APPLE_PMIC_EVENTS_MAX);
	u8 events[APPLE_PMIC_EVENT_BYTES_MAX];
	unsigned int hwirq;
	int ret;

	bitmap_zero(pending, APPLE_PMIC_EVENTS_MAX);

	mutex_lock(&gpio->irq_lock);
	if (!apple_pmic_event_gate(gpio))
		goto unlock;
	mutex_unlock(&gpio->irq_lock);

	/* Before the read, which takes an I2C transfer of the whole block. */
	if (test_bit(gpio->nevents, gpio->enabled))
		apple_pmic_dispatch(gpio, gpio->nevents);

	mutex_lock(&gpio->irq_lock);
	if (!apple_pmic_event_gate(gpio))
		goto unlock;

	if (time_after(jiffies, gpio->irq_window + HZ)) {
		gpio->irq_window = jiffies;
		gpio->irq_burst = 0;
	}
	if (++gpio->irq_burst > APPLE_PMIC_IRQ_BURST) {
		ret = -ELOOP;
		goto hold;
	}

	/* Reading the events clears them and releases the line. */
	ret = regmap_bulk_read(gpio->regmap, ev->status_base, events, ev->bytes);
	if (ret)
		goto hold;
	gpio->retries = 0;

	for (hwirq = 0; hwirq < gpio->nevents; hwirq++)
		if (events[hwirq / 8] & BIT(hwirq % 8) &&
		    test_bit(hwirq, gpio->enabled))
			__set_bit(hwirq, pending);
	mutex_unlock(&gpio->irq_lock);
	goto dispatch;

hold:
	dev_err_ratelimited(gpio->chip.parent,
			    "PMU event read failed or storm (%d), holding the line, retry %u/%u\n",
			    ret, gpio->retries + 1, APPLE_PMIC_IRQ_RETRIES);
	apple_pmic_irq_hold(gpio, true);
	bitmap_copy(pending, gpio->enabled, gpio->nevents);
	mutex_unlock(&gpio->irq_lock);

dispatch:
	/* Consumers may mask their interrupt, which takes the bus lock. */
	for_each_set_bit(hwirq, pending, gpio->nevents)
		apple_pmic_dispatch(gpio, hwirq);
	return IRQ_HANDLED;

unlock:
	mutex_unlock(&gpio->irq_lock);
	return IRQ_HANDLED;
}

static int apple_pmic_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	unsigned int irq;

	irq = irq_create_mapping(gpio->domain,
				 gpio->info->events->gpio_event + offset);
	return irq ?: -ENXIO;
}

static void apple_pmic_event_restore_mask(void *data)
{
	struct apple_pmic_gpio *gpio = data;

	regmap_bulk_write(gpio->regmap, gpio->info->events->mask_base,
			  gpio->boot_mask, gpio->info->events->bytes);
}

static void apple_pmic_event_remove_domain(void *data)
{
	struct apple_pmic_gpio *gpio = data;
	unsigned int hwirq, irq;

	for (hwirq = 0; hwirq <= gpio->nevents; hwirq++) {
		irq = irq_find_mapping(gpio->domain, hwirq);
		if (irq)
			irq_dispose_mapping(irq);
	}
	irq_domain_remove(gpio->domain);
}

static void apple_pmic_event_stop(void *data)
{
	struct apple_pmic_gpio *gpio = data;

	mutex_lock(&gpio->irq_lock);
	gpio->removing = true;
	mutex_unlock(&gpio->irq_lock);
	cancel_delayed_work_sync(&gpio->irq_retry);
	free_irq(gpio->parent_irq, gpio);
}

static int apple_pmic_event_probe(struct platform_device *pdev,
				  struct apple_pmic_gpio *gpio)
{
	const struct apple_pmic_event_info *ev = gpio->info->events;
	struct device *dev = &pdev->dev;
	u8 buf[APPLE_PMIC_EVENT_BYTES_MAX];
	int ret;

	if (!ev || !device_property_read_bool(dev, "interrupt-controller"))
		return 0;

	gpio->parent_irq = platform_get_irq(pdev, 0);
	if (gpio->parent_irq < 0)
		return gpio->parent_irq;
	gpio->nevents = ev->bytes * 8;

	ret = devm_mutex_init(dev, &gpio->irq_lock);
	if (ret)
		return ret;
	INIT_DELAYED_WORK(&gpio->irq_retry, apple_pmic_irq_retry);

	ret = regmap_bulk_read(gpio->regmap, ev->mask_base, gpio->boot_mask,
			       ev->bytes);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the event mask\n");

	/* As iOS does at probe: mask all, unmask only what has a consumer. */
	memset(gpio->mask, 0xff, sizeof(gpio->mask));
	ret = regmap_bulk_write(gpio->regmap, ev->mask_base, gpio->mask,
				ev->bytes);
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask the events\n");
	ret = devm_add_action_or_reset(dev, apple_pmic_event_restore_mask, gpio);
	if (ret)
		return ret;

	/* The events, then the line interrupt (hwirq nevents). */
	gpio->domain = irq_domain_create_linear(dev_fwnode(dev),
						gpio->nevents + 1,
						&apple_pmic_irq_domain_ops,
						gpio);
	if (!gpio->domain)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, apple_pmic_event_remove_domain,
				       gpio);
	if (ret)
		return ret;

	/* Drop stale events so the line is released before it goes live. */
	ret = regmap_bulk_read(gpio->regmap, ev->status_base, buf, ev->bytes);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the events\n");

	ret = request_threaded_irq(gpio->parent_irq, NULL,
				   apple_pmic_event_thread, IRQF_ONESHOT,
				   dev_name(dev), gpio);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request the PMU interrupt\n");
	return devm_add_action_or_reset(dev, apple_pmic_event_stop, gpio);
}

static int apple_pmic_gpio_suspend(struct device *dev)
{
	struct apple_pmic_gpio *gpio = dev_get_drvdata(dev);
	int ret;

	if (!gpio->domain)
		return 0;

	cancel_delayed_work_sync(&gpio->irq_retry);
	mutex_lock(&gpio->irq_lock);
	/* Only wake-enabled events may pull the line while asleep. */
	ret = apple_pmic_event_sync(gpio, gpio->wake);
	if (ret)
		dev_warn(dev, "event wake mask write failed: %d\n", ret);
	gpio->suspended = true;
	mutex_unlock(&gpio->irq_lock);
	/* A thread that passed the gate before it closed has finished now. */
	synchronize_irq(gpio->parent_irq);
	return 0;
}

static int apple_pmic_gpio_resume(struct device *dev)
{
	struct apple_pmic_gpio *gpio = dev_get_drvdata(dev);
	int ret;

	if (!gpio->domain)
		return 0;

	/* Our I2C ancestors have completed their resume callbacks. */
	mutex_lock(&gpio->irq_lock);
	gpio->suspended = false;
	gpio->retries = 0;
	ret = apple_pmic_event_sync(gpio, gpio->enabled);
	if (ret)
		dev_warn(dev, "event mask write failed: %d\n", ret);
	/* An event that arrived while gated is still latched. */
	if (gpio->irq_held) {
		gpio->irq_held = false;
		enable_irq(gpio->parent_irq);
	}
	mutex_unlock(&gpio->irq_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_pmic_gpio_pm_ops, apple_pmic_gpio_suspend,
				apple_pmic_gpio_resume);

/* ---- GPIO ---- */

static int apple_pmic_gpio_get_direction(struct gpio_chip *chip,
					 unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, info->config_reg(offset), &value);
	if (ret)
		return ret;
	if (value & info->function_mask)
		return -EINVAL;

	if ((value & info->direction_mask) == info->input_mode)
		return GPIO_LINE_DIRECTION_IN;
	if ((value & info->direction_mask) == info->output_mode)
		return GPIO_LINE_DIRECTION_OUT;

	return -EINVAL;
}

static int apple_pmic_gpio_direction_input(struct gpio_chip *chip,
					   unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->direction_mask | info->value_mask,
				  info->input_mode);
}

static int apple_pmic_gpio_direction_output(struct gpio_chip *chip,
					    unsigned int offset, int value)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->direction_mask | info->value_mask,
				  info->output_mode |
				  (value ? info->value_mask : 0));
}

static int apple_pmic_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;
	unsigned int value;
	int ret;

	ret = regmap_read(gpio->regmap, info->config_reg(offset), &value);
	if (ret)
		return ret;
	if ((value & info->direction_mask) == info->output_mode)
		return !!(value & info->value_mask);
	if ((value & info->direction_mask) != info->input_mode)
		return -EINVAL;

	ret = regmap_read(gpio->regmap, info->input_base + offset / 8,
			  &value);
	if (ret)
		return ret;

	return !!(value & BIT(offset % 8));
}

static int apple_pmic_gpio_set(struct gpio_chip *chip, unsigned int offset,
			       int value)
{
	struct apple_pmic_gpio *gpio = gpiochip_get_data(chip);
	const struct apple_pmic_gpio_info *info = gpio->info;

	return regmap_update_bits(gpio->regmap,
				  info->config_reg(offset),
				  info->value_mask,
				  value ? info->value_mask : 0);
}

static int apple_pmic_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmic_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;
	platform_set_drvdata(pdev, gpio);

	gpio->info = device_get_match_data(dev);
	if (!gpio->info)
		return -EINVAL;

	gpio->regmap = dev_get_regmap(dev->parent, NULL);
	if (!gpio->regmap)
		return dev_err_probe(dev, -ENODEV, "parent regmap unavailable\n");

	ret = apple_pmic_event_probe(pdev, gpio);
	if (ret)
		return ret;

	gpio->chip.label = gpio->info->label;
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = gpio->info->ngpio;
	gpio->chip.can_sleep = true;
	gpio->chip.get_direction = apple_pmic_gpio_get_direction;
	gpio->chip.direction_input = apple_pmic_gpio_direction_input;
	gpio->chip.direction_output = apple_pmic_gpio_direction_output;
	gpio->chip.get = apple_pmic_gpio_get;
	gpio->chip.set = apple_pmic_gpio_set;
	if (gpio->domain)
		gpio->chip.to_irq = apple_pmic_gpio_to_irq;

	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static const struct apple_pmic_gpio_info d2207_gpio_info = {
	.label = "d2207-pmic-gpio",
	.ngpio = 21,
	.input_base = 0x63,
	.value_mask = D2207_GPIO_VALUE,
	.direction_mask = D2207_GPIO_MODE_MASK,
	.input_mode = D2207_GPIO_INPUT,
	.output_mode = D2207_GPIO_OUTPUT,
	.config_reg = d2207_gpio_config_reg,
};

static const struct apple_pmic_gpio_info d2333_gpio_info = {
	.label = "d2333-pmic-gpio",
	.ngpio = ARRAY_SIZE(d2333_gpio_config_regs),
	.input_base = 0x186,
	.value_mask = D2333_GPIO_VALUE,
	.direction_mask = D2333_GPIO_DIRECTION_MASK,
	.input_mode = D2333_GPIO_INPUT,
	.output_mode = D2333_GPIO_OUTPUT,
	.config_reg = d2333_gpio_config_reg,
};

static const struct apple_pmic_event_info d2365_event_info = {
	.status_base = D2365_EVENTS,
	.mask_base = D2365_EVENT_MASK,
	.bytes = D2365_EVENT_BYTES,
	.gpio_event = D2365_GPIO_EVENT,
};

static const struct apple_pmic_gpio_info d2365_gpio_info = {
	.label = "d2365-pmic-gpio",
	.ngpio = 24,
	.input_base = 0x186,
	.value_mask = D2365_GPIO_VALUE,
	.direction_mask = D2365_GPIO_INPUT | D2365_GPIO_FUNCTION,
	.input_mode = D2365_GPIO_INPUT,
	.output_mode = 0,
	.function_mask = D2365_GPIO_FUNCTION,
	.config_reg = d2365_gpio_config_reg,
	.events = &d2365_event_info,
};

static const struct of_device_id d2365_gpio_of_match[] = {
	{
		.compatible = "apple,d2207-pmic-gpio",
		.data = &d2207_gpio_info,
	},
	{
		.compatible = "apple,d2333-pmic-gpio",
		.data = &d2333_gpio_info,
	},
	{
		.compatible = "apple,d2365-pmic-gpio",
		.data = &d2365_gpio_info,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, d2365_gpio_of_match);

static struct platform_driver d2365_gpio_driver = {
	.probe = apple_pmic_gpio_probe,
	.driver = {
		.name = "apple-d2365-pmic-gpio",
		.of_match_table = d2365_gpio_of_match,
		.pm = pm_sleep_ptr(&apple_pmic_gpio_pm_ops),
	},
};
module_platform_driver(d2365_gpio_driver);

MODULE_DESCRIPTION("Apple A8X/A9/A10 PMIC GPIO and event interrupt driver");
MODULE_LICENSE("GPL");
