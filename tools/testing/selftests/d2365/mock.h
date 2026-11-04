/* SPDX-License-Identifier: MIT
 * Offline-only kernel shims. No hardware, reboot syscall, or device access.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define NOTIFY_DONE 0
#define POWER_SUPPLY_PROP_ONLINE 0
static char logs[4096];
static unsigned log_count;
static void log_message(void *dev, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(logs + strlen(logs), sizeof(logs) - strlen(logs), format, args);
    va_end(args);
    log_count++;
}
#define dev_info log_message
#define dev_err log_message
#define dev_emerg log_message
struct device { int unused; };
static unsigned quiesce_calls;
struct regmap { int unused; };
struct power_supply { int unused; };
union power_supply_propval { int intval; };
struct sys_off_data { void *cb_data; };
struct operation { char kind; unsigned reg, size, value; };
static struct operation operations[128];
static unsigned count, fail_at, source_reads, online_reads;
static void apple_d2365_gpio_quiesce(struct device *dev) { assert(count == 0); quiesce_calls++; }
static unsigned standby_attempts, standby_writes, restart_calls, sleeps;
static unsigned source_change_at, online_change_at, final_event_change;
static int source_value, online_value;
static u8 memory[0x6000];
static bool final_phase;
/* 0: no commit, 1: commit whole transfer then error, 2: partial bulk commit. */
static unsigned error_effect, pair_reads;
static bool mutate_pairs;
static jmp_buf restart_terminal;
static int record(char kind, unsigned reg, unsigned size, unsigned value)
{
    assert(!final_phase);
    assert(count < ARRAY_SIZE(operations));
    operations[count++] = (struct operation){kind, reg, size, value};
    return fail_at == count ? -5 : 0;
}
static int regmap_read(struct regmap *map, unsigned reg, unsigned *value)
{
    int ret = record('r', reg, 1, 0);
    if (ret) return ret;
    if (reg == 0x018a) {
        source_reads++;
        *value = source_change_at && source_reads >= source_change_at ? 1 : source_value;
    } else *value = memory[reg];
    return 0;
}
static int regmap_write(struct regmap *map, unsigned reg, unsigned value)
{
    int ret = record('w', reg, 1, value);
    if (reg == 0x0400) standby_attempts++;
    if (ret && !error_effect) return ret;
    memory[reg] = value;
    if (reg == 0x0400) standby_writes++;
    return ret;
}
static int regmap_bulk_read(struct regmap *map, unsigned reg, void *data, unsigned size)
{
    int ret = record('R', reg, size, 0);
    if (ret && !error_effect) return ret;
    if (reg == 0x0900 && ++pair_reads == 2 && mutate_pairs)
        for (unsigned i = 0; i < 18; i++) {
            memory[0x0900 + 2 * i] = 0xfe; /* New config must not replace snapshot. */
            memory[0x0901 + 2 * i] = 0x40 + i;
        }
    memcpy(data, memory + reg, size);
    if (reg == 0x0140)
        memset(memory + reg, 0, ret && error_effect == 2 ? size / 2 : size);
    if (reg == 0x0140 && final_event_change) {
        unsigned event_reads = 0;
        for (unsigned i = 0; i < count; i++)
            event_reads += operations[i].kind == 'R' && operations[i].reg == 0x0140;
        if (event_reads == 2) source_value = 1;
    }
    return ret;
}
static int regmap_bulk_write(struct regmap *map, unsigned reg, const void *data, unsigned size)
{
    int ret = record('W', reg, size, 0);
    if (ret && !error_effect) return ret;
    memcpy(memory + reg, data, ret && error_effect == 2 ? size / 2 : size);
    return ret;
}
static int regmap_write_bits(struct regmap *map, unsigned reg, unsigned mask, unsigned value)
{
    unsigned previous;
    int ret = regmap_read(map, reg, &previous);
    if (ret) return ret;
    return regmap_write(map, reg, (previous & ~mask) | (value & mask));
}
static int power_supply_get_property(struct power_supply *supply, int property,
                                     union power_supply_propval *value)
{
    int ret = record('p', 0, 0, 0);
    if (ret) return ret;
    online_reads++;
    value->intval = online_change_at && online_reads >= online_change_at ? 1 : online_value;
    return 0;
}
static void msleep(unsigned value)
{
    assert(!final_phase);
    assert(value <= 1000);
    sleeps++;
}
/* Model arm64's terminal reset-or-halt behavior, never a returning call. */
static _Noreturn void machine_restart(const char *command)
{
    restart_calls++;
    longjmp(restart_terminal, 1);
}
