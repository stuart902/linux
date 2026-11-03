/* SPDX-License-Identifier: MIT */
static struct apple_d2365_poweroff pmic;
static struct sys_off_data off = { &pmic };
static void reset(void)
{
    memset(memory, 0, sizeof(memory));
    memset(operations, 0, sizeof(operations));
    count = fail_at = source_reads = online_reads = quiesce_calls = 0;
    standby_attempts = standby_writes = restart_calls = sleeps = 0;
    source_change_at = online_change_at = final_event_change = 0;
    source_value = online_value = 0;
    final_phase = false;
    error_effect = pair_reads = log_count = 0;
    mutate_pairs = false;
    logs[0] = 0;
    pmic = (struct apple_d2365_poweroff){ .data = &apple_d2365_data, .restart = true };
    memset(memory + 0x0140, 0x55, 30);
    memory[0x0400] = 0xa4;
    memory[0x500f] = 0xff;
    for (unsigned i = 0; i < 18; i++) {
        memory[0x0900 + 2 * i] = i % 2 ? 0x81 : 0x3f;
        memory[0x0901 + 2 * i] = 0x80 + i;
    }
    for (unsigned i = 0; i < 6; i++) memory[0x0929 + 6 * i] = i % 2 ? 0xff : 0x01;
}
static void prepare(void) { assert(apple_d2365_poweroff_prepare(&off) == NOTIFY_DONE); assert(quiesce_calls == 1); }
static void rejected(void)
{
    assert(pmic.restart && !pmic.standby_acknowledged);
    assert(!sleeps);
}
static unsigned find(char kind, unsigned reg)
{
    for (unsigned i = 0; i < count; i++)
        if (operations[i].kind == kind && operations[i].reg == reg) return i;
    assert(!"missing operation");
    return 0;
}
static void check_order(void)
{
    reset(); prepare();
    assert(!pmic.restart && pmic.standby_acknowledged);
    assert(standby_writes == 1 && sleeps == 1);
    assert(memory[0x0400] == 0xa5 && memory[0x500f] == 0x27);
    unsigned events = find('R', 0x0140), mask = find('W', 0x01c0);
    unsigned rtc = find('w', 0x0500), state = find('w', 0x5001);
    unsigned flags = find('w', 0x5003), boot = find('w', 0x500f);
    unsigned gpio = find('W', 0x0900), standby = find('w', 0x0400);
    assert(events < mask && mask < rtc && rtc < state && state < flags);
    assert(flags < boot && boot < gpio && gpio < standby);
    assert(operations[gpio - 1].kind == 'R' &&
           operations[gpio - 1].reg == 0x0900 && operations[gpio - 1].size == 36);
    assert(operations[events].size == 30 && operations[mask].size == 30);
    assert(!memcmp(memory + 0x01c0, apple_d2365_data.event_mask, 30));
    assert(memory[0x0500] == 1 && !memory[0x5001] && !memory[0x5003]);
    unsigned event_count = 0, last_events = 0;
    for (unsigned i = 0; i < count; i++) {
        if (operations[i].reg == 0x0140) {
            assert(operations[i].kind == 'R' && operations[i].size == 30);
            event_count++; last_events = i;
        }
    }
    for (unsigned i = 0; i < 30; i++) assert(!memory[0x0140 + i]);
    assert(event_count == 2 && last_events > gpio && last_events < standby);
    assert(source_reads >= 2 && online_reads >= 2);
    /* Request must be forced, not optimized away when the bit is already set. */
    reset(); memory[0x0400] = 0xa5; prepare();
    assert(standby_writes == 1 && memory[0x0400] == 0xa5);
}
static void check_gpio(void)
{
    for (unsigned value = 0; value < 256; value++) {
        reset(); mutate_pairs = true;
        for (unsigned i = 0; i < 18; i++) memory[0x0900 + 2 * i] = value;
        for (unsigned i = 0; i < 6; i++) memory[0x0929 + 6 * i] = value;
        prepare();
        unsigned expected = value <= 0x3f ? value & ~1U : value;
        assert(pmic.standby_acknowledged && pair_reads == 2);
        for (unsigned i = 0; i < 18; i++) {
            assert(memory[0x0900 + 2 * i] == expected);
            assert(memory[0x0901 + 2 * i] == 0x40 + i);
        }
        for (unsigned i = 0; i < 6; i++) assert(memory[0x0929 + 6 * i] == expected);
    }
}
static void check_boot_state(void)
{
    for (unsigned value = 0; value < 256; value++) {
        for (unsigned source = 0; source < 2; source++) {
            reset(); memory[0x500f] = value; source_value = source;
            prepare();
            assert(memory[0x500f] == ((value & 0x27) | (source ? 0x90 : 0)));
        }
    }
}
static void check_sources(void)
{
    for (unsigned value = 1; value < 256; value++) {
        reset(); source_value = value; prepare(); rejected();
        assert(memory[0x500f] == 0xb7 && !standby_attempts);
    }
    const int online[] = {1, 2, -1};
    for (unsigned i = 0; i < ARRAY_SIZE(online); i++) {
        reset(); online_value = online[i]; prepare(); rejected();
        assert(memory[0x500f] == 0xb7);
    }
    reset(); fail_at = 1; prepare(); rejected(); /* Unknown supply. */
    assert(count == 1);
    reset(); fail_at = 2; prepare(); rejected(); /* Unknown live status. */
    assert(count == 2);
}
static void check_changes(void)
{
    reset(); source_change_at = 2; prepare(); rejected();
    assert(memory[0x500f] == 0xb7);
    reset(); online_change_at = 2; prepare(); rejected();
    assert(memory[0x500f] == 0xb7);
    /* Attach immediately after GPIO preparation and the final event read. */
    reset(); final_event_change = 1; prepare(); rejected();
    assert(memory[0x500f] == 0xb7 && !standby_attempts);
}
static void failure_route(unsigned route)
{
    if (route == 1) source_value = 1;
    if (route == 2) source_change_at = 2;
    if (route == 3) final_event_change = 1;
    if (route == 4) online_change_at = 2;
}
static void check_failures(void)
{
    for (unsigned route = 0; route < 5; route++) {
        reset(); failure_route(route); prepare();
        unsigned total = count;
        assert(total > 5);
        for (unsigned effect = 0; effect < 3; effect++) {
            for (unsigned failure = 1; failure <= total; failure++) {
                reset(); failure_route(route); fail_at = failure;
                error_effect = effect;
                prepare(); rejected();
                assert(count == failure); /* No retry or I/O after a failed transaction. */
                assert(standby_attempts == (route == 0 && failure == total));
                assert(standby_writes == (route == 0 && failure == total && effect));
                assert(pmic.standby_transaction_attempted == (route == 0 && failure >= total - 1));
                assert(strstr(logs, pmic.standby_transaction_attempted ?
                              "outcome unknown" : "not attempted"));
                assert(!strstr(logs, "without standby"));
                /* If a bulk transfer errors, check the mock really committed
                 * none/all/a prefix rather than merely toggling a return code. */
                if (operations[failure - 1].kind == 'W' &&
                    operations[failure - 1].reg == 0x0900) {
                    unsigned written = !effect ? 0 : effect == 1 ? 36 : 18;
                    for (unsigned i = 0; i < 18; i++) {
                        unsigned original = i % 2 ? 0x81 : 0x3f;
                        unsigned expected = 2 * i < written && original <= 0x3f ?
                                            original & ~1U : original;
                        assert(memory[0x0900 + 2 * i] == expected);
                    }
                }
                if (operations[failure - 1].kind == 'W' &&
                    operations[failure - 1].reg == 0x01c0) {
                    unsigned written = !effect ? 0 : effect == 1 ? 30 : 15;
                    for (unsigned i = 0; i < 30; i++)
                        assert(memory[0x01c0 + i] == (i < written ?
                               apple_d2365_data.event_mask[i] : 0));
                }
            }
        }
    }
}
static void check_final(void)
{
    reset(); prepare();
    unsigned before = count;
    final_phase = true;
    assert(apple_d2365_poweroff_final(&off) == NOTIFY_DONE);
    assert(count == before && !restart_calls && sleeps == 1);
    reset(); source_value = 1; prepare();
    before = count; final_phase = true;
    unsigned before_logs = log_count;
    if (!setjmp(restart_terminal)) {
        apple_d2365_poweroff_final(&off);
        assert(!"arm64 machine_restart must not return");
    }
    assert(count == before && restart_calls == 1 && !sleeps);
    assert(log_count == before_logs + 1);
}
static void check_restart(void)
{
    for (unsigned failure = 0; failure <= 2; failure++) {
        for (unsigned effect = 0; effect < 3; effect++) {
            reset(); fail_at = failure; error_effect = effect;
            memory[0x5001] = 0xa5;
            assert(apple_d2365_restart_prepare(&off) == NOTIFY_DONE);
            assert(quiesce_calls == 1);
            assert(count == 2);
            assert(operations[0].kind == 'R' && operations[0].reg == 0x0140 &&
                   operations[0].size == 30);
            assert(operations[1].kind == 'w' && operations[1].reg == 0x5001 &&
                   operations[1].value == 0);
            assert(memory[0x5001] == (failure == 2 && !effect ? 0xa5 : 0));
            assert(!standby_attempts && !standby_writes && !sleeps && !restart_calls);
            unsigned consumed = failure != 1 || effect == 1 ? 30 :
                                effect == 2 ? 15 : 0;
            for (unsigned i = 0; i < 30; i++)
                assert(memory[0x0140 + i] == (i < consumed ? 0 : 0x55));
            assert(log_count == !!failure);
            if (failure) assert(strstr(logs, failure == 1 ?
                "restart event acknowledgement failed" : "restart boot-stage clear failed"));
        }
    }
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "order")) check_order();
    else if (!strcmp(argv[1], "gpio")) check_gpio();
    else if (!strcmp(argv[1], "boot")) check_boot_state();
    else if (!strcmp(argv[1], "sources")) check_sources();
    else if (!strcmp(argv[1], "changes")) check_changes();
    else if (!strcmp(argv[1], "failures")) check_failures();
    else if (!strcmp(argv[1], "final")) check_final();
    else if (!strcmp(argv[1], "restart")) check_restart();
    else return 2;
    return 0;
}
