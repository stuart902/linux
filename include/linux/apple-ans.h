/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_APPLE_ANS_H
#define _LINUX_APPLE_ANS_H

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

#define APPLE_ANS_NVMMU_MAX_REQS	36
#define APPLE_ANS_NVMMU_MAX_PAGES	256
#define APPLE_ANS_NVMMU_PAGE_SIZE	4096

/*
 * Map @npages pages for FlatDMA request @tag, or retire the tag if @npages is
 * zero. @to_memory: the controller writes the pages (an NVMe read).
 */
#if IS_REACHABLE(CONFIG_PCIE_APPLE_H9P)
int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
			const u64 *pages, unsigned int npages,
			bool to_memory, dma_addr_t *iova);
#else
static inline int apple_ans_nvmmu_map(struct device *dev, unsigned int tag,
				      const u64 *pages, unsigned int npages,
				      bool to_memory, dma_addr_t *iova)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* _LINUX_APPLE_ANS_H */
