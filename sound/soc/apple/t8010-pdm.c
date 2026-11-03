// SPDX-License-Identifier: GPL-2.0-only
/*
 * Experimental T8010 PDM consumer. Default-off, NOT ENABLEMENT-READY:
 * Requires ownership-preserving SmartIO drain and exclusive host PDM
 * high-power request ownership. Do not enable a board node until both integration contracts
 * are reviewed. Uncertain ownership can permanently block close, suspend,
 * and shutdown until a physical device reset.
 */
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of_platform.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/sio.h>
#include <linux/soc/apple/aop-audio.h>
#include <linux/unaligned.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "t8010-pdm.h"
#include "t8010-pdm-adt.h"

#define PDM_BUFFER_BYTES (128 * 1024)

struct pdm_decimator {
	void __iomem *base;
	resource_size_t size, physical;
	u32 config, delay_ms, force_gating;
	u8 mic_name[4];
	unsigned int nfilters;
	struct pdm_filter filters[PDM_MAX_RATES];
};

struct pdm_profile {
	const char *model;
};

struct t8010_pdm {
	const struct pdm_profile *profile;
	struct device *dev;
	void __iomem *base;
	resource_size_t physical, size;
	struct pdm_decimator dec[PDM_DECIMATORS];
	struct apple_aop_pdm_clock *clock;
	struct mutex lifecycle;
	struct completion fault_wait;
	struct dma_chan *dma;
	int irq;
	/* Serializes trigger/callback state and PCM position. */
	spinlock_t lock;
	struct snd_pcm_substream *stream;
	struct snd_pcm_hw_constraint_list rate_list;
	unsigned int rates[PDM_MAX_RATES];
	unsigned int rate, delay_ms;
	size_t position;
	dma_cookie_t cyclic_cookie;
	struct dma_async_tx_descriptor *cyclic_desc;
	void *discard;
	dma_addr_t discard_dma;
	size_t discard_bytes;
	bool prepared, running, primed, shutting_down;
	bool powered, clocked, irq_enabled;
	struct apple_sio_dma_config dma_config;
};

static const struct pdm_filter *pdm_find_filter(struct pdm_decimator *d, u32 rate)
{
	unsigned int i;

	for (i = 0; i < d->nfilters; i++)
		if (d->filters[i].rate == rate)
			return &d->filters[i];
	return NULL;
}

/* Return only a snapshot of the existing live ADT RAM MTD, never named flash. */
static void *pdm_adt_snapshot(size_t *bytes)
{
	struct device_node *reserved, *node, *selected = NULL;
	struct platform_device *provider;
	struct mtd_info *mtd;
	const char *label;
	void *snapshot;
	size_t received;
	int ret = -ENODEV;

	reserved = of_find_node_by_path("/reserved-memory");
	if (!reserved)
		return ERR_PTR(-ENODEV);
	for_each_child_of_node(reserved, node) {
		if (of_property_read_string(node, "label", &label) || strcmp(label, "adt"))
			continue;
		if (selected || !of_device_is_compatible(node, "phram") ||
		    !of_device_is_available(node)) {
			ret = -EINVAL;
			of_node_put(node);
			goto put_nodes;
		}
		selected = of_node_get(node);
	}
	if (!selected)
		goto put_nodes;
	provider = of_find_device_by_node(selected);
	if (!provider) {
		ret = -EPROBE_DEFER;
		goto put_nodes;
	}
	/* phram remove frees its mapping: exclude unbind while taking the copy. */
	device_lock(&provider->dev);
	if (!device_is_bound(&provider->dev)) {
		ret = -EPROBE_DEFER;
		goto unlock;
	}
	mtd = of_get_mtd_device_by_node(selected);
	if (IS_ERR(mtd)) {
		ret = PTR_ERR(mtd);
		goto unlock;
	}
	if (mtd->type != MTD_RAM || mtd->size < 8 || mtd->size > PDM_ADT_MAX_BYTES) {
		ret = -EINVAL;
		goto put_mtd;
	}
	*bytes = mtd->size;
	snapshot = kvmalloc(*bytes, GFP_KERNEL);
	if (!snapshot) {
		ret = -ENOMEM;
		goto put_mtd;
	}
	ret = mtd_read(mtd, 0, *bytes, &received, snapshot);
	if (ret || received != *bytes) {
		kvfree(snapshot);
		if (!ret)
			ret = -EIO;
		goto put_mtd;
	}
	put_mtd_device(mtd);
	device_unlock(&provider->dev);
	put_device(&provider->dev);
	of_node_put(selected);
	of_node_put(reserved);
	return snapshot;
put_mtd:
	put_mtd_device(mtd);
unlock:
	device_unlock(&provider->dev);
	put_device(&provider->dev);
put_nodes:
	of_node_put(selected);
	of_node_put(reserved);
	return ERR_PTR(ret);
}

static bool pdm_rate_present(const u32 *rates, unsigned int count, u32 rate)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (rates[i] == rate)
			return true;
	return false;
}

static int pdm_adt_filter_read(struct t8010_pdm *pdm, const struct pdm_adt *t,
			       u32 provider, struct pdm_decimator *d, u32 rate)
{
	struct pdm_filter *f = &d->filters[d->nfilters];
	const u8 *value;
	char name[40];
	u32 length;
	int ret;

	f->rate = rate;
	snprintf(name, sizeof(name), "pdmfreq-%u", rate);
	if (pdm_adt_u32(t, provider, name, &f->frequency))
		return -EINVAL;
	snprintf(name, sizeof(name), "latency-%u", rate);
	if (pdm_adt_u32(t, provider, name, &f->latency))
		return -EINVAL;
	snprintf(name, sizeof(name), "ratios-%u", rate);
	value = pdm_adt_property(t, provider, name, &length);
	if (!value || length != 4 || value[3])
		return -EINVAL;
	memcpy(f->ratios, value, 3);
	snprintf(name, sizeof(name), "lengths-%u", rate);
	value = pdm_adt_property(t, provider, name, &length);
	if (!value || length != 4 || value[3])
		return -EINVAL;
	memcpy(f->lengths, value, 3);
	snprintf(name, sizeof(name), "coeff-%u", rate);
	value = pdm_adt_property(t, provider, name, &length);
	if (!value)
		return -EINVAL;
	ret = pdm_filter_validate(f, d->size, length);
	if (ret)
		return ret;
	f->coefficients = devm_kmemdup(pdm->dev, value, length, GFP_KERNEL);
	if (!f->coefficients)
		return -ENOMEM;
	d->nfilters++;
	return 0;
}

static int pdm_adt_decode(struct t8010_pdm *pdm, const struct pdm_adt *t)
{
	u32 phandles[2], providers[2], dst[PDM_MAX_RATES];
	u32 mic[2][PDM_MAX_RATES], mic_count[2], length, index, phandle;
	const u8 *value;
	int arm_io, receiver, decimator, provider, count, ret;
	unsigned int i, j;

	if (!pdm_adt_string(t, 0, "model", pdm->profile->model))
		return -ENODEV;
	arm_io = pdm_adt_child(t, 0, "arm-io");
	if (arm_io < 0)
		return arm_io;
	receiver = pdm_adt_child(t, arm_io, "pdm");
	if (receiver < 0)
		return receiver;
	if (!pdm_adt_string(t, arm_io, "compatible", "arm-io,t8010") ||
	    !pdm_adt_string(t, receiver, "compatible", "pdm,t8010") ||
	    !pdm_adt_reg_matches(t, arm_io, receiver, pdm->physical, pdm->size))
		return -ENODEV;
	/* The receiver must select the same pdm0 HP service used by our API. */
	value = pdm_adt_property(t, receiver, "function-pdm-clk-cfg", &length);
	if (!value || length != 8 || get_unaligned_le32(value + 4) != 0x70646d43)
		return -EINVAL;
	provider = pdm_adt_phandle(t, get_unaligned_le32(value));
	if (provider < 0 || pdm_adt_u32(t, provider, "identifier", &index) ||
	    index != 0x70646d30)
		return -EINVAL;
	value = pdm_adt_property(t, receiver, "decimators", &length);
	if (!value || length != 8)
		return -EINVAL;
	phandles[0] = get_unaligned_le32(value);
	phandles[1] = get_unaligned_le32(value + 4);
	if (phandles[0] == phandles[1])
		return -EINVAL;
	count = pdm_adt_rates(t, receiver, "dst-samplerates", dst, PDM_MAX_RATES);
	if (count < 0)
		return count;
	for (i = 0; i < PDM_DECIMATORS; i++) {
		struct pdm_decimator *d = &pdm->dec[i];

		decimator = pdm_adt_phandle(t, phandles[i]);
		if (decimator < 0)
			return decimator;
		if (!pdm_adt_string(t, decimator, "compatible", "hifidec,t8010") ||
		    !pdm_adt_reg_matches(t, arm_io, decimator, d->physical, d->size) ||
		    pdm_adt_u32(t, decimator, "index", &index) || index != i ||
		    pdm_adt_u32(t, decimator, "config", &d->config) ||
		    pdm_adt_u32(t, decimator, "mic-startup-delay", &d->delay_ms) ||
		    pdm_adt_u32(t, decimator, "mic-coeff-data", &phandle) ||
		    d->delay_ms > PDM_MAX_DELAY_MS)
			return -EINVAL;
		value = pdm_adt_property(t, decimator, "mic-name", &length);
		if (!value || length != sizeof(d->mic_name))
			return -EINVAL;
		memcpy(d->mic_name, value, length);
		/* Apple's missing self-registry ForceClockGating property defaults false. */
		d->force_gating = 0;
		pdm->delay_ms = max(pdm->delay_ms, d->delay_ms);
		provider = pdm_adt_phandle(t, phandle);
		if (provider < 0)
			return provider;
		providers[i] = provider;
		ret = pdm_adt_rates(t, provider, "mic-samplerates", mic[i], PDM_MAX_RATES);
		if (ret < 0)
			return ret;
		mic_count[i] = ret;
	}
	for (j = 0; j < count; j++) {
		if (!pdm_rate_present(mic[0], mic_count[0], dst[j]) ||
		    !pdm_rate_present(mic[1], mic_count[1], dst[j]))
			continue;
		for (i = 0; i < PDM_DECIMATORS; i++) {
			ret = pdm_adt_filter_read(pdm, t, providers[i], &pdm->dec[i], dst[j]);
			if (ret)
				return ret;
		}
		index = pdm->dec[0].nfilters - 1;
		if (pdm->dec[0].filters[index].frequency != pdm->dec[1].filters[index].frequency)
			return -EINVAL;
		pdm->rates[index] = dst[j];
	}
	if (!pdm->dec[0].nfilters)
		return -EINVAL;
	pdm->rate_list.count = pdm->dec[0].nfilters;
	pdm->rate_list.list = pdm->rates;
	return 0;
}

static int pdm_calibration_read(struct t8010_pdm *pdm)
{
	struct pdm_adt tree = {};
	void *snapshot;
	int ret;

	snapshot = pdm_adt_snapshot(&tree.size);
	if (IS_ERR(snapshot))
		return PTR_ERR(snapshot);
	tree.data = snapshot;
	tree.nodes = kvcalloc(PDM_ADT_MAX_NODES, sizeof(*tree.nodes), GFP_KERNEL);
	if (!tree.nodes) {
		ret = -ENOMEM;
		goto free_snapshot;
	}
	ret = pdm_adt_parse(&tree);
	if (!ret)
		ret = pdm_adt_decode(pdm, &tree);
	kvfree(tree.nodes);
free_snapshot:
	/* Only the selected validated coefficient copies outlive probe. */
	kvfree(snapshot);
	return ret;
}

static int pdm_reset(struct t8010_pdm *pdm)
{
	u32 status;
	int ret;

	writel(1, pdm->base + 0x24);
	ret = readl_poll_timeout(pdm->base + 0x24, status, !(status & 1), 10, 10000);
	if (ret)
		return ret;
	if (readl(pdm->base + 0x18) & BIT(28))
		return -EBUSY;
	writel(2, pdm->base + 0x0c);
	return readl_poll_timeout(pdm->base + 0x0c, status, !(status & 2), 10, 10000);
}

static int pdm_stop_receiver(struct t8010_pdm *pdm)
{
	u32 status;

	if (!pdm->powered || !pdm->clocked)
		return 0;
	writel(0x38, pdm->base + 0x20);
	writel(0x0a, pdm->base + 0x10);
	return readl_poll_timeout(pdm->base + 0x18, status,
				  !(status & BIT(28)), 10, 10000);
}

static void pdm_program_decimator(struct pdm_decimator *d,
				  const struct pdm_filter *f, unsigned int index)
{
	/* Original HP-decimator LFSR seed tables, indexed by logical index. */
	static const u32 seed_a[PDM_DECIMATORS] = { 0x80000057, 0x80000062 };
	static const u32 seed_b[PDM_DECIMATORS] = { 0x08000004, 0x08000029 };
	size_t off;

	writel(d->force_gating ? 0x1111 : 0, d->base + 0x10);
	writel(1, d->base + 0x30); /* 16-bit output */
	writel(d->config & 0x11, d->base + 0x08);
	writel(f->ratio_reg, d->base + 0x14);
	writel(f->length_reg, d->base);
	for (off = 0; off < f->nbytes; off += 4)
		writel(get_unaligned_le32(f->coefficients + off),
		       d->base + PDM_COEFF_BASE + off);
	writel(1, d->base + 0x04);
	writel(seed_a[index], d->base + 0x3c);
	writel(seed_b[index], d->base + 0x40);
}

/*
 * No reset/fence has been proved for uncertain ownership. The
 * fail-closed policy retains the calling teardown and its resources until
 * the machine is reset; a timeout is not permission to free live resources.
 */
static void __noreturn pdm_retain_fault(struct t8010_pdm *pdm, const char *what, int error)
{
	dev_err(pdm->dev, "%s failed (%d); retaining resources until device reset\n",
		what, error);
	for (;;)
		wait_for_completion(&pdm->fault_wait);
}

static void pdm_power_down(struct t8010_pdm *pdm)
{
	unsigned int i;
	int ret, dma_ret;

	/* Every caller, including prepare unwind, must establish quiescence. */
	ret = pdm_stop_receiver(pdm);
	dma_ret = dmaengine_terminate_sync(pdm->dma);
	if (dma_ret)
		pdm_retain_fault(pdm, "DMA termination", dma_ret);
	if (ret)
		pdm_retain_fault(pdm, "PDM receiver stop", ret);
	if (pdm->irq_enabled) {
		disable_irq(pdm->irq);
		pdm->irq_enabled = false;
	}
	if (pdm->powered && pdm->clocked)
		for (i = 0; i < PDM_DECIMATORS; i++)
			writel(0x1111, pdm->dec[i].base + 0x10);
	if (pdm->powered) {
		ret = pm_runtime_put_sync(pdm->dev);
		if (ret < 0)
			pdm_retain_fault(pdm, "PDM runtime power release unresolved", ret);
		/* The runtime PM vote is released, not a physical-off readback. */
		pdm->powered = false;
	}
	/* AP receiver/DMA are quiescent before releasing the host HP request. */
	if (pdm->clocked) {
		ret = apple_aop_pdm_clock_stop(pdm->clock);
		if (ret)
			pdm_retain_fault(pdm, "PDM clock STOP", ret);
		pdm->clocked = false;
	}
	if (apple_aop_pdm_clock_needs_retention(pdm->clock))
		pdm_retain_fault(pdm, "PDM clock ownership", -EIO);
	pdm->prepared = false;
}

static void pdm_mark_stopped(struct t8010_pdm *pdm)
{
	unsigned long flags;

	spin_lock_irqsave(&pdm->lock, flags);
	pdm->running = false;
	pdm->primed = false;
	pdm->prepared = false;
	pdm->cyclic_desc = NULL;
	spin_unlock_irqrestore(&pdm->lock, flags);
}

static void pdm_prime_done(void *arg, const struct dmaengine_result *result)
{
	struct t8010_pdm *pdm = arg;
	struct snd_pcm_substream *stream = NULL;
	unsigned long flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&pdm->lock, flags);
	if (!pdm->running)
		goto unlock;
	stream = pdm->stream;
	if (result->result != DMA_TRANS_NOERROR || !pdm->cyclic_desc)
		goto unlock;
	/*
	 * The cyclic descriptor was prepared in sleepable context. Submit it
	 * only after successful priming. STOP serializes here before freeing
	 * any unsubmitted descriptor via DMAengine termination.
	 */
	cookie = dmaengine_submit(pdm->cyclic_desc);
	pdm->cyclic_desc = NULL;
	if (dma_submit_error(cookie))
		goto unlock;
	pdm->cyclic_cookie = cookie;
	pdm->primed = true;
	dma_async_issue_pending(pdm->dma);
	stream = NULL;
unlock:
	spin_unlock_irqrestore(&pdm->lock, flags);
	if (stream)
		snd_pcm_stop_xrun(stream);
	/* Separate discarded storage: never advance ALSA for this completion. */
}

static void pdm_period_done(void *arg, const struct dmaengine_result *result)
{
	struct t8010_pdm *pdm = arg;
	unsigned long flags;
	bool notify = false, failed = false;

	spin_lock_irqsave(&pdm->lock, flags);
	if (pdm->running && pdm->primed) {
		failed = result->result != DMA_TRANS_NOERROR;
		if (!failed) {
			/* DMA callbacks can coalesce; pointer uses provider residue. */
			notify = true;
		}
	}
	spin_unlock_irqrestore(&pdm->lock, flags);
	if (failed)
		snd_pcm_stop_xrun(pdm->stream);
	else if (notify)
		snd_pcm_period_elapsed(pdm->stream);
}

static int pdm_queue_dma(struct t8010_pdm *pdm, struct snd_pcm_substream *s)
{
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;

	desc = dmaengine_prep_slave_single(pdm->dma, pdm->discard_dma,
					   pdm->discard_bytes, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -ENOMEM;
	desc->callback_result = pdm_prime_done;
	desc->callback_param = pdm;
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie))
		return dma_submit_error(cookie);
	desc = dmaengine_prep_dma_cyclic(pdm->dma, s->runtime->dma_addr,
					 snd_pcm_lib_buffer_bytes(s),
					 snd_pcm_lib_period_bytes(s), DMA_DEV_TO_MEM,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -ENOMEM;
	desc->callback_result = pdm_period_done;
	desc->callback_param = pdm;
	/* DMAengine owns allocated descriptors until submit or termination. */
	pdm->cyclic_desc = desc;
	pdm->cyclic_cookie = 0;
	return 0;
}

static const struct snd_pcm_hardware pdm_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_KNOT,
	.rate_min = 1,
	.rate_max = 192000,
	.channels_min = PDM_DECIMATORS,
	.channels_max = PDM_DECIMATORS,
	.buffer_bytes_max = PDM_BUFFER_BYTES,
	.period_bytes_min = 512,
	.period_bytes_max = 32 * 1024,
	.periods_min = 2,
	.periods_max = 16,
};

static int pdm_open(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	if (READ_ONCE(pdm->shutting_down) || s->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;
	s->runtime->hw = pdm_pcm_hardware;
	ret = snd_pcm_hw_constraint_list(s->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &pdm->rate_list);
	if (ret)
		return ret;
	return snd_pcm_hw_constraint_integer(s->runtime, SNDRV_PCM_HW_PARAM_PERIODS);
}

static int pdm_hw_params_locked(struct snd_soc_component *component,
			 struct snd_pcm_substream *s, struct snd_pcm_hw_params *params)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	struct dma_slave_config config = { .direction = DMA_DEV_TO_MEM };
	unsigned int rate = params_rate(params);
	int ret;

	if (pdm->shutting_down || params_format(params) != SNDRV_PCM_FORMAT_S16_LE ||
	    params_channels(params) != PDM_DECIMATORS ||
	    !pdm_find_filter(&pdm->dec[0], rate) || pdm->discard)
		return -EINVAL;
	pdm->dma_config = (struct apple_sio_dma_config) {
		.xfer = 0x21,
		.timeout = 0,
		.fifo = 0x398,
		.trigger = 0x190,
		.limit = 2 * roundup(rate * PDM_DECIMATORS / 1000, 4),
	};
	if (!pdm->dma_config.limit || pdm->dma_config.limit > pdm->dma_config.trigger)
		return -EINVAL;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	config.peripheral_config = &pdm->dma_config;
	config.peripheral_size = sizeof(pdm->dma_config);
	ret = dmaengine_slave_config(pdm->dma, &config);
	if (ret)
		return ret;
	/* Conservative full settling interval, not Apple's buffer-tail cap. */
	ret = pdm_discard_size(rate, pdm->delay_ms, &pdm->discard_bytes);
	if (ret)
		return ret;
	pdm->discard = dma_alloc_coherent(pdm->dma->device->dev, pdm->discard_bytes,
					  &pdm->discard_dma, GFP_KERNEL);
	if (!pdm->discard)
		return -ENOMEM;
	pdm->stream = s;
	pdm->rate = rate;
	return 0;
}

static int pdm_sync_stop_locked(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	ret = pdm_stop_receiver(pdm);
	/* SmartIO must not return until actual DMA ownership has been returned. */
	dmaengine_synchronize(pdm->dma);
	if (ret)
		pdm_retain_fault(pdm, "PDM receiver stop", ret);
	if (pdm->clocked) {
		ret = pdm_reset(pdm);
		if (ret)
			pdm_retain_fault(pdm, "PDM reset", ret);
	}
	return 0;
}

static int pdm_hw_free_locked(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);

	pdm_mark_stopped(pdm);
	pdm_power_down(pdm);
	if (pdm->discard) {
		dma_free_coherent(pdm->dma->device->dev, pdm->discard_bytes,
				  pdm->discard, pdm->discard_dma);
		pdm->discard = NULL;
	}
	pdm->stream = NULL;
	return 0;
}

static int pdm_prepare_locked(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	const struct pdm_filter *filter = pdm_find_filter(&pdm->dec[0], pdm->rate);
	u32 capacity, threshold;
	unsigned int i;
	int ret;

	if (pdm->shutting_down || !filter || !pdm->discard)
		return -EINVAL;
	pdm_mark_stopped(pdm);
	pdm_power_down(pdm);
	/* Board enablement must establish exclusive host HP ownership first. */
	ret = apple_aop_pdm_clock_stop(pdm->clock);
	if (ret) {
		if (apple_aop_pdm_clock_needs_retention(pdm->clock))
			pdm_retain_fault(pdm, "initial PDM clock OFF", ret);
		goto power_down;
	}
	ret = apple_aop_pdm_clock_start(pdm->clock, filter->frequency);
	if (ret) {
		if (apple_aop_pdm_clock_needs_retention(pdm->clock))
			pdm_retain_fault(pdm, "PDM clock START", ret);
		goto power_down;
	}
	ret = pm_runtime_resume_and_get(pdm->dev);
	if (ret < 0)
		pdm_retain_fault(pdm, "PDM runtime power acquisition unresolved", ret);
	pdm->powered = true;
	pdm->clocked = true;
	/* No MMIO above this point: both power and firmware request are owned. */
	writel(0x38, pdm->base + 0x20);
	ret = pdm_reset(pdm);
	if (ret)
		pdm_retain_fault(pdm, "PDM reset", ret);
	capacity = (readl(pdm->base + 0x04) >> 16) & 0x3ff;
	if (capacity <= roundup(2 * PDM_DECIMATORS, 4)) {
		ret = -EINVAL;
		goto power_down;
	}
	threshold = min(roundup(pdm->rate / 1000, 4), capacity - 4);
	if (!threshold || threshold > 512) {
		ret = -EINVAL;
		goto power_down;
	}
	writel(8 | GENMASK(5, 4), pdm->base + 0x10);
	writel(0x22, pdm->base + 0x80);
	writel(0x1000, pdm->base + 0x08); /* (-16 << 8) & 0x1f00 */
	writel(4 | (threshold - 1) << 16, pdm->base + 0x84);
	for (i = 0; i < PDM_DECIMATORS; i++)
		pdm_program_decimator(&pdm->dec[i], pdm_find_filter(&pdm->dec[i], pdm->rate), i);
	ret = pdm_queue_dma(pdm, s);
	if (ret)
		goto power_down;
	pdm->position = 0;
	pdm->prepared = true;
	writel(0x18, pdm->base + 0x1c);
	enable_irq(pdm->irq);
	pdm->irq_enabled = true;
	return 0;
power_down:
	pdm_power_down(pdm);
	return ret;
}

static int pdm_trigger(struct snd_soc_component *component,
		       struct snd_pcm_substream *s, int cmd)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	unsigned long flags;
	u32 status;
	int ret = 0;

	spin_lock_irqsave(&pdm->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (pdm->shutting_down || !pdm->prepared || pdm->running) {
			ret = -EINVAL;
			break;
		}
		pdm->running = true;
		writel(0x20, pdm->base + 0x20);
		dma_async_issue_pending(pdm->dma);
		writel(9 | GENMASK(5, 4), pdm->base + 0x10);
		readl(pdm->base + 0x10);
		writel(1, pdm->base + 0x110);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pdm->running = false;
		pdm->primed = false;
		pdm->cyclic_desc = NULL;
		if (pdm->prepared) {
			writel(0x38, pdm->base + 0x20);
			writel(0x0a, pdm->base + 0x10);
			ret = readl_poll_timeout_atomic(pdm->base + 0x18, status,
							!(status & BIT(28)), 10, 1000);
		}
		/* Draining and sleepable clock release are deliberately elsewhere. */
		dmaengine_terminate_async(pdm->dma);
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&pdm->lock, flags);
	return ret;
}

static snd_pcm_uframes_t pdm_pointer(struct snd_soc_component *component,
				     struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	struct dma_tx_state state = {};
	enum dma_status status;
	unsigned long flags;
	snd_pcm_uframes_t position;

	spin_lock_irqsave(&pdm->lock, flags);
	if (pdm->running && pdm->primed) {
		status = dmaengine_tx_status(pdm->dma, pdm->cyclic_cookie, &state);
		if (status == DMA_ERROR ||
		    pdm_position_bytes(snd_pcm_lib_buffer_bytes(s), state.residue,
				       &pdm->position)) {
			position = SNDRV_PCM_POS_XRUN;
			goto unlock;
		}
	}
	position = bytes_to_frames(s->runtime, pdm->position);
unlock:
	spin_unlock_irqrestore(&pdm->lock, flags);
	return position;
}

static int pdm_pcm_construct(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);

	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
					     pdm->dma->device->dev,
					     PDM_BUFFER_BYTES, PDM_BUFFER_BYTES);
}

static irqreturn_t pdm_irq(int irq, void *data)
{
	struct t8010_pdm *pdm = data;
	u32 status = readl(pdm->base + 0x1c);

	if (!(status & 0x18))
		return IRQ_NONE;
	writel(status, pdm->base + 0x1c);
	writel(0x38, pdm->base + 0x20);
	if (READ_ONCE(pdm->running))
		snd_pcm_stop_xrun(pdm->stream);
	return IRQ_HANDLED;
}

static int pdm_component_suspend_locked(struct snd_soc_component *component)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);

	pdm_mark_stopped(pdm);
	pdm_power_down(pdm);
	return 0;
}

static void pdm_cleanup_locked(void *data)
{
	struct t8010_pdm *pdm = data;

	pdm_mark_stopped(pdm);
	pdm_power_down(pdm);
	if (pdm->discard) {
		dma_free_coherent(pdm->dma->device->dev, pdm->discard_bytes,
				  pdm->discard, pdm->discard_dma);
		pdm->discard = NULL;
	}
	pdm->stream = NULL;
}

static int pdm_hw_params(struct snd_soc_component *component,
			 struct snd_pcm_substream *s, struct snd_pcm_hw_params *params)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&pdm->lifecycle);
	ret = pdm_hw_params_locked(component, s, params);
	mutex_unlock(&pdm->lifecycle);
	return ret;
}

static int pdm_hw_free(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&pdm->lifecycle);
	ret = pdm_hw_free_locked(component, s);
	mutex_unlock(&pdm->lifecycle);
	return ret;
}

static int pdm_prepare(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&pdm->lifecycle);
	ret = pdm_prepare_locked(component, s);
	mutex_unlock(&pdm->lifecycle);
	return ret;
}

static int pdm_sync_stop(struct snd_soc_component *component, struct snd_pcm_substream *s)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&pdm->lifecycle);
	ret = pdm_sync_stop_locked(component, s);
	mutex_unlock(&pdm->lifecycle);
	return ret;
}

static int pdm_component_suspend(struct snd_soc_component *component)
{
	struct t8010_pdm *pdm = snd_soc_component_get_drvdata(component);
	int ret;

	mutex_lock(&pdm->lifecycle);
	ret = pdm_component_suspend_locked(component);
	mutex_unlock(&pdm->lifecycle);
	return ret;
}

static void pdm_cleanup(void *data)
{
	struct t8010_pdm *pdm = data;

	unsigned long flags;

	mutex_lock(&pdm->lifecycle);
	spin_lock_irqsave(&pdm->lock, flags);
	pdm->shutting_down = true;
	pdm->prepared = false;
	spin_unlock_irqrestore(&pdm->lock, flags);
	pdm_cleanup_locked(data);
	mutex_unlock(&pdm->lifecycle);
}

static const struct snd_soc_component_driver pdm_component = {
	.name = "apple-t8010-pdm",
	.suspend = pdm_component_suspend,
	.open = pdm_open,
	.hw_params = pdm_hw_params,
	.hw_free = pdm_hw_free,
	.prepare = pdm_prepare,
	.trigger = pdm_trigger,
	.sync_stop = pdm_sync_stop,
	.pointer = pdm_pointer,
	.pcm_new = pdm_pcm_construct,
};

static struct snd_soc_dai_driver pdm_dai = {
	.name = "t8010-pdm",
	.capture = {
		.stream_name = "PDM Capture",
		.channels_min = PDM_DECIMATORS,
		.channels_max = PDM_DECIMATORS,
		.rates = SNDRV_PCM_RATE_KNOT,
		.rate_min = 1,
		.rate_max = 192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static void pdm_release_dma(void *data)
{
	dma_release_channel(data);
}

static void pdm_release_clock(void *data)
{
	struct t8010_pdm *pdm = data;
	int ret = apple_aop_pdm_clock_put(pdm->clock);

	if (ret)
		pdm_retain_fault(pdm, "PDM clock release", ret);
}

static void pdm_disable_pm(void *data)
{
	pm_runtime_disable(data);
}

static int pdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct t8010_pdm *pdm;
	struct resource *res;
	unsigned int i;
	int ret;

	if (!dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "PDM power domain unavailable\n");
	pdm = devm_kzalloc(dev, sizeof(*pdm), GFP_KERNEL);
	if (!pdm)
		return -ENOMEM;
	pdm->profile = of_device_get_match_data(dev);
	if (!pdm->profile)
		return -ENODEV;
	pdm->dev = dev;
	spin_lock_init(&pdm->lock);
	mutex_init(&pdm->lifecycle);
	init_completion(&pdm->fault_wait);
	platform_set_drvdata(pdev, pdm);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pdmc");
	if (!res || resource_size(res) < 0x114)
		return -EINVAL;
	pdm->physical = res->start;
	pdm->size = resource_size(res);
	pdm->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdm->base))
		return PTR_ERR(pdm->base);
	for (i = 0; i < PDM_DECIMATORS; i++) {
		char name[16];

		snprintf(name, sizeof(name), "decimator%u", i);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (!res || resource_size(res) <= PDM_COEFF_BASE)
			return -EINVAL;
		pdm->dec[i].physical = res->start;
		pdm->dec[i].size = resource_size(res);
		pdm->dec[i].base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pdm->dec[i].base))
			return PTR_ERR(pdm->dec[i].base);
	}
	ret = pdm_calibration_read(pdm);
	if (ret)
		return dev_err_probe(dev, ret, "missing or invalid PDM calibration\n");
	pdm->dma = dma_request_chan(dev, "rx");
	if (IS_ERR(pdm->dma))
		return dev_err_probe(dev, PTR_ERR(pdm->dma), "PDM DMA unavailable\n");
	ret = devm_add_action_or_reset(dev, pdm_release_dma, pdm->dma);
	if (ret)
		return ret;
	pdm->irq = platform_get_irq(pdev, 0);
	if (pdm->irq < 0)
		return pdm->irq;
	ret = devm_request_irq(dev, pdm->irq, pdm_irq, IRQF_NO_AUTOEN, dev_name(dev), pdm);
	if (ret)
		return ret;
	pdm->clock = apple_aop_pdm_clock_get(dev);
	if (IS_ERR(pdm->clock))
		return dev_err_probe(dev, PTR_ERR(pdm->clock), "PDM firmware clock unavailable\n");
	ret = devm_add_action_or_reset(dev, pdm_release_clock, pdm);
	if (ret)
		return ret;
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, pdm_disable_pm, dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, pdm_cleanup, pdm);
	if (ret)
		return ret;
	return devm_snd_soc_register_component(dev, &pdm_component, &pdm_dai, 1);
}

static void pdm_shutdown(struct platform_device *pdev)
{
	pdm_cleanup(platform_get_drvdata(pdev));
}

static const struct pdm_profile pdm_j71b_profile = {
	.model = "iPad7,5",
};

static const struct of_device_id pdm_of_match[] = {
	{ .compatible = "apple,j71b-pdm", .data = &pdm_j71b_profile },
	{ }
};
MODULE_DEVICE_TABLE(of, pdm_of_match);

static struct platform_driver pdm_driver = {
	.probe = pdm_probe,
	.shutdown = pdm_shutdown,
	.driver = {
		.name = "apple-t8010-pdm",
		.of_match_table = pdm_of_match,
	},
};
module_platform_driver(pdm_driver);
MODULE_DESCRIPTION("Experimental Apple T8010 PDM capture consumer");
MODULE_LICENSE("GPL");
