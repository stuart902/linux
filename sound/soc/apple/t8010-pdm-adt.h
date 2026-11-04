/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __APPLE_T8010_PDM_ADT_H
#define __APPLE_T8010_PDM_ADT_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* Hard limits apply before allocation, traversal or interpreting calibration. */
#define PDM_ADT_MAX_BYTES (16 * 1024 * 1024)
#define PDM_ADT_MAX_NODES 4096
#define PDM_ADT_MAX_PROPERTIES 256
#define PDM_ADT_MAX_DEPTH 32

struct pdm_adt_node {
	u32 properties, count, parent;
};

struct pdm_adt {
	const u8 *data;
	size_t size;
	struct pdm_adt_node *nodes;
	u32 count;
};

/* Properties and node names are bounded byte strings, never trusted C strings. */
static inline int pdm_adt_parse_node(struct pdm_adt *t, size_t *cursor,
				   u32 parent, unsigned int depth)
{
	u32 index, props, children, i, j;
	size_t start, old;

	if (depth > PDM_ADT_MAX_DEPTH || t->count == PDM_ADT_MAX_NODES ||
	    *cursor > t->size || t->size - *cursor < 8)
		return -EINVAL;
	props = get_unaligned_le32(t->data + *cursor);
	children = get_unaligned_le32(t->data + *cursor + 4);
	if (props > PDM_ADT_MAX_PROPERTIES || children > PDM_ADT_MAX_NODES)
		return -EINVAL;
	*cursor += 8;
	index = t->count++;
	t->nodes[index] = (struct pdm_adt_node) {
		.properties = *cursor, .count = props, .parent = parent,
	};
	for (i = 0; i < props; i++) {
		u32 bytes;

		start = *cursor;
		if (t->size - start < 36 || !memchr(t->data + start, 0, 32))
			return -EINVAL;
		/* Reject repeated names, including properties we do not consume. */
		old = t->nodes[index].properties;
		for (j = 0; j < i; j++) {
			if (!strcmp((const char *)t->data + old, (const char *)t->data + start))
				return -EINVAL;
			bytes = get_unaligned_le32(t->data + old + 32) & 0xffffff;
			old += 36 + ((bytes + 3) & ~3U);
		}
		bytes = get_unaligned_le32(t->data + start + 32) & 0xffffff;
		bytes = (bytes + 3) & ~3U;
		if (bytes > t->size - start - 36)
			return -EINVAL;
		*cursor += 36 + bytes;
	}
	for (i = 0; i < children; i++) {
		int ret = pdm_adt_parse_node(t, cursor, index, depth + 1);

		if (ret)
			return ret;
	}
	return 0;
}

static inline int pdm_adt_parse(struct pdm_adt *t)
{
	size_t cursor = 0;

	if (!t->data || !t->nodes || t->size < 8 || t->size > PDM_ADT_MAX_BYTES)
		return -EINVAL;
	t->count = 0;
	return pdm_adt_parse_node(t, &cursor, ~0U, 0);
}

static inline const u8 *pdm_adt_property(const struct pdm_adt *t, u32 node,
				       const char *name, u32 *length)
{
	size_t cursor;
	u32 i, bytes;

	*length = 0;
	if (node >= t->count)
		return NULL;
	cursor = t->nodes[node].properties;
	for (i = 0; i < t->nodes[node].count; i++) {
		bytes = get_unaligned_le32(t->data + cursor + 32) & 0xffffff;
		if (!strcmp((const char *)t->data + cursor, name)) {
			*length = bytes;
			return t->data + cursor + 36;
		}
		cursor += 36 + ((bytes + 3) & ~3U);
	}
	return NULL;
}

static inline bool pdm_adt_string(const struct pdm_adt *t, u32 node,
				  const char *property, const char *expected)
{
	u32 length;
	const u8 *value = pdm_adt_property(t, node, property, &length);

	return value && length == strlen(expected) + 1 &&
	       !memcmp(value, expected, length);
}

static inline int pdm_adt_u32(const struct pdm_adt *t, u32 node,
			      const char *property, u32 *result)
{
	u32 length;
	const u8 *value = pdm_adt_property(t, node, property, &length);

	if (!value || length != 4)
		return -EINVAL;
	*result = get_unaligned_le32(value);
	return 0;
}

static inline int pdm_adt_child(const struct pdm_adt *t, u32 parent,
				const char *name)
{
	int found = -ENOENT;
	u32 i;

	for (i = 0; i < t->count; i++) {
		if (t->nodes[i].parent != parent || !pdm_adt_string(t, i, "name", name))
			continue;
		if (found >= 0)
			return -EINVAL;
		found = i;
	}
	return found;
}

static inline int pdm_adt_phandle(const struct pdm_adt *t, u32 phandle)
{
	int found = -ENOENT;
	u32 i, value;

	if (!phandle)
		return -EINVAL;
	for (i = 0; i < t->count; i++) {
		if (pdm_adt_u32(t, i, "AAPL,phandle", &value) || value != phandle)
			continue;
		if (found >= 0)
			return -EINVAL;
		found = i;
	}
	return found;
}

static inline int pdm_adt_rates(const struct pdm_adt *t, u32 node,
				const char *property, u32 *rates, u32 capacity)
{
	u32 length, i, j, count;
	const u8 *value = pdm_adt_property(t, node, property, &length);

	if (!value || !length || length % 8)
		return -EINVAL;
	count = length / 8;
	if (count > capacity)
		return -E2BIG;
	for (i = 0; i < count; i++) {
		/* The first word is zero padding, not the low half of a u64. */
		if (get_unaligned_le32(value + 8 * i))
			return -EINVAL;
		rates[i] = get_unaligned_le32(value + 8 * i + 4);
		if (!rates[i] || rates[i] > 192000)
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (rates[j] == rates[i])
				return -EINVAL;
	}
	return count;
}
/* Resolve one ADT MMIO tuple through the actual arm-io ranges. */
static inline bool pdm_adt_reg_matches(const struct pdm_adt *t, u32 arm_io,
				      u32 node, u64 physical, u64 resource_size)
{
	const u8 *reg, *ranges;
	u32 length, ranges_length, i, matches = 0;
	u64 address, bytes;

	reg = pdm_adt_property(t, node, "reg", &length);
	ranges = pdm_adt_property(t, arm_io, "ranges", &ranges_length);
	if (!reg || length < 16 || length % 16 || !ranges ||
	    !ranges_length || ranges_length % 24)
		return false;
	address = get_unaligned_le64(reg);
	bytes = get_unaligned_le64(reg + 8);
	if (!bytes || bytes != resource_size)
		return false;
	for (i = 0; i < ranges_length; i += 24) {
		u64 child = get_unaligned_le64(ranges + i);
		u64 parent = get_unaligned_le64(ranges + i + 8);
		u64 size = get_unaligned_le64(ranges + i + 16);
		u64 offset;

		if (address < child || bytes > size)
			continue;
		offset = address - child;
		if (offset > size - bytes)
			continue;
		if (parent > ~(u64)0 - offset || parent + offset != physical)
			return false;
		matches++;
	}
	return matches == 1;
}
#endif
