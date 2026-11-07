#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host tests of the actual diagnostic codec callbacks; no audio/device access.

The kernel GPIO, timer, devres and locking primitives are mocks. This is not
kernel concurrency, timer-latency, physical isolation or speaker qualification.
Run directly with Python 3.11+ and a host C compiler, without alsa-lib.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[4]
DRIVER = ROOT / "sound/soc/codecs/max98357a.c"
SOURCE = DRIVER.read_text()

STUBS = r'''
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define IS_ENABLED(x) (x)
#define module_param_named(...)
#define MODULE_PARM_DESC(...)
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define dev_dbg(...) ((void)0)
#define dev_notice(...) ((void)0)
#define dev_err_probe(dev, error, ...) (error)
#define SNDRV_PCM_TRIGGER_START 1
#define SNDRV_PCM_TRIGGER_STOP 2
#define SNDRV_PCM_TRIGGER_RESUME 3
#define SNDRV_PCM_TRIGGER_SUSPEND 4
#define SNDRV_PCM_TRIGGER_PAUSE_RELEASE 5
#define SNDRV_PCM_TRIGGER_PAUSE_PUSH 6
#define SND_SOC_DAPM_POST_PMU 1
#define SND_SOC_DAPM_POST_PMD 2
#define HRTIMER_MODE_ABS_HARD 4
#define ATOMIC_INIT(x) (x)
typedef _Atomic int atomic_t;
static int atomic_cmpxchg(atomic_t *p, int expected, int desired)
{
    atomic_compare_exchange_strong(p, &expected, desired);
    return expected;
}
typedef int64_t ktime_t;
static ktime_t now_ns;
static ktime_t start_delay;
static ktime_t ktime_get(void) { return now_ns; }
static ktime_t ktime_add_ms(ktime_t n, unsigned int ms) { return n + ms * 1000000LL; }
static int ktime_compare(ktime_t a, ktime_t b) { return (a > b) - (a < b); }
typedef pthread_mutex_t raw_spinlock_t;
#define raw_spin_lock_init(p) assert(!pthread_mutex_init(p, NULL))
#define raw_spin_lock_irqsave(p, f) do { (f) = 0; assert(!pthread_mutex_lock(p)); } while (0)
#define raw_spin_unlock_irqrestore(p, f) do { (void)(f); assert(!pthread_mutex_unlock(p)); } while (0)
enum hrtimer_restart { HRTIMER_NORESTART };
struct hrtimer {
    enum hrtimer_restart (*function)(struct hrtimer *);
    bool pending;
    unsigned int starts, cancels;
    ktime_t deadline;
};
static unsigned int event_sequence, timer_order, high_order;
static bool expire_in_cancel;
static void hrtimer_setup(struct hrtimer *t,
                         enum hrtimer_restart (*fn)(struct hrtimer *),
                         int clock, int mode)
{
    assert(clock == CLOCK_MONOTONIC && mode == HRTIMER_MODE_ABS_HARD);
    t->function = fn;
}
static void hrtimer_start(struct hrtimer *t, ktime_t deadline, int mode)
{
    assert(mode == HRTIMER_MODE_ABS_HARD);
    t->pending = true;
    t->starts++;
    t->deadline = deadline;
    timer_order = ++event_sequence;
    now_ns += start_delay;
}
static int hrtimer_cancel(struct hrtimer *t)
{
    t->cancels++;
    if (expire_in_cancel && t->pending)
        assert(t->function(t) == HRTIMER_NORESTART);
    t->pending = false;
    return 0;
}
struct gpio_desc { bool sleeping; int value; unsigned int highs, writes; };
static void gpiod_set_value(struct gpio_desc *g, int v)
{
    assert(g && !g->sleeping);
    g->value = v;
    g->writes++;
    if (v) { g->highs++; high_order = ++event_sequence; }
}
static bool gpiod_cansleep(struct gpio_desc *g) { return g->sleeping; }
static unsigned int delays;
static void mdelay(unsigned int n) { delays += n; }
struct device { bool tdm; bool action_fail; void (*action)(void *); void *data; };
static bool board_j71b;
static bool of_machine_is_compatible(const char *s)
{
    assert(!strcmp(s, "apple,j71b"));
    return board_j71b;
}
static bool device_property_read_bool(struct device *d, const char *p)
{
    assert(!strcmp(p, "maxim,tdm"));
    return d->tdm;
}
static int devm_add_action_or_reset(struct device *d, void (*fn)(void *), void *data)
{
    if (d->action_fail) { fn(data); return -ENOMEM; }
    d->action = fn;
    d->data = data;
    return 0;
}
struct snd_soc_component { struct device *dev; void *data; };
struct snd_soc_dai { struct snd_soc_component *component; };
struct snd_soc_dapm_widget { struct snd_soc_component *dapm; };
struct snd_kcontrol { int unused; };
struct snd_pcm_substream { int unused; };
static void *snd_soc_component_get_drvdata(struct snd_soc_component *c) { return c->data; }
static struct snd_soc_component *snd_soc_dapm_to_component(struct snd_soc_component *c) { return c; }
'''

CASES = r'''
struct fixture {
    struct gpio_desc gpio;
    struct device dev;
    struct max98357a_priv priv;
    struct snd_soc_component component;
    struct snd_soc_dai dai;
    struct snd_soc_dapm_widget widget;
};
#if CONFIG_SND_SOC_MAX98357A_J71B_TEST
static unsigned int cases;
#endif
static void reset_module(void)
{
    j71b_test_arm = false;
    atomic_store(&j71b_test_used, 0);
    board_j71b = true;
    now_ns = 1000000;
    start_delay = 0;
    expire_in_cancel = false;
    event_sequence = timer_order = high_order = delays = 0;
}
static void fixture(struct fixture *f)
{
    memset(f, 0, sizeof(*f));
    f->dev.tdm = true;
    f->priv.sdmode = &f->gpio;
    f->component.dev = &f->dev;
    f->component.data = &f->priv;
    f->dai.component = &f->component;
    f->widget.dapm = &f->component;
}
static int init(struct fixture *f)
{
    return max98357a_test_init(&f->dev, &f->priv);
}
static void power(struct fixture *f, int event)
{
    assert(!max98357a_sdmode_event(&f->widget, NULL, event));
}
static int trigger(struct fixture *f, int cmd)
{
    return max98357a_daiops_trigger(NULL, cmd, &f->dai);
}
static void dispose(struct fixture *f)
{
    if (f->dev.action) f->dev.action(f->dev.data);
    if (f->priv.test_gate) assert(!pthread_mutex_destroy(&f->priv.test_lock));
}
static void armed(struct fixture *f)
{
    reset_module(); fixture(f); j71b_test_arm = true;
    assert(!init(f)); power(f, SND_SOC_DAPM_POST_PMU);
}
static void *start_thread(void *p) { (void)trigger(p, SNDRV_PCM_TRIGGER_START); return NULL; }
static void *quiesce_thread(void *p) { max98357a_test_quiesce(&((struct fixture *)p)->priv); return NULL; }
static void *expire_thread(void *p) { max98357a_test_expire(&((struct fixture *)p)->priv.test_timer); return NULL; }
static void *stop_thread(void *p) { (void)trigger(p, SNDRV_PCM_TRIGGER_STOP); return NULL; }
static void race(struct fixture *f, void *(*a)(void *), void *(*b)(void *))
{
    pthread_t t1, t2;
    assert(!pthread_create(&t1, NULL, a, f));
    assert(!pthread_create(&t2, NULL, b, f));
    assert(!pthread_join(t1, NULL)); assert(!pthread_join(t2, NULL));
    assert(!f->gpio.value);
}
int main(void)
{
    struct fixture f;
#if CONFIG_SND_SOC_MAX98357A_J71B_TEST
    struct fixture other;
    const int stops[] = { SNDRV_PCM_TRIGGER_STOP, SNDRV_PCM_TRIGGER_SUSPEND,
                          SNDRV_PCM_TRIGGER_PAUSE_PUSH };
    const int resumes[] = { SNDRV_PCM_TRIGGER_RESUME, SNDRV_PCM_TRIGGER_PAUSE_RELEASE };
#endif
    reset_module(); fixture(&f);
    assert(!init(&f));
#if !CONFIG_SND_SOC_MAX98357A_J71B_TEST
    /* Compiled-out diagnostics preserve the original codec trigger behavior. */
    assert(!f.priv.test_gate && !f.dev.action);
    power(&f, SND_SOC_DAPM_POST_PMU);
    f.priv.sdmode_delay = 7;
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_START) && f.gpio.value);
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_STOP) && !f.gpio.value);
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_RESUME) && f.gpio.value);
    assert(delays == 14 && !f.priv.test_timer.starts);
    puts("config-off behavior passed");
    return 0;
#else
    assert(f.priv.test_gate && !f.gpio.value);
    power(&f, SND_SOC_DAPM_POST_PMU);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM);
    assert(!f.gpio.highs && !f.priv.test_timer.starts && !atomic_load(&j71b_test_used));
    dispose(&f); cases++;

    armed(&f);
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
    assert(f.gpio.value && f.gpio.highs == 1 && atomic_load(&j71b_test_used) == 1);
    assert(timer_order < high_order && f.priv.test_timer.starts == 1);
    assert(f.priv.test_timer.deadline == now_ns + 30000000LL);
    assert(!delays);
    assert(max98357a_test_expire(&f.priv.test_timer) == HRTIMER_NORESTART);
    assert(!f.gpio.value);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && f.gpio.highs == 1);
    dispose(&f); cases += 3;

    for (unsigned int n = 0; n < sizeof(stops)/sizeof(stops[0]); n++) {
        armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
        assert(!trigger(&f, stops[n]) && !f.gpio.value);
        max98357a_test_expire(&f.priv.test_timer);
        assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && f.gpio.highs == 1);
        dispose(&f); cases++;
    }
    for (unsigned int n = 0; n < sizeof(resumes)/sizeof(resumes[0]); n++) {
        armed(&f);
        assert(trigger(&f, resumes[n]) == -EPERM && !f.gpio.highs);
        assert(!atomic_load(&j71b_test_used));
        assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
        assert(trigger(&f, resumes[n]) == -EPERM && !f.gpio.value);
        assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && f.gpio.highs == 1);
        dispose(&f); cases++;
    }
    armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && !f.gpio.value);
    assert(f.gpio.highs == 1 && f.priv.test_timer.starts == 1);
    dispose(&f); cases++;

    armed(&f); power(&f, SND_SOC_DAPM_POST_PMD);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && !f.gpio.highs);
    assert(!atomic_load(&j71b_test_used));
    power(&f, SND_SOC_DAPM_POST_PMU);
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
    power(&f, SND_SOC_DAPM_POST_PMD); assert(!f.gpio.value);
    power(&f, SND_SOC_DAPM_POST_PMU);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && f.gpio.highs == 1);
    dispose(&f); cases++;

    armed(&f); start_delay = 30000000LL;
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -ETIME && !f.gpio.highs);
    assert(atomic_load(&j71b_test_used) == 1);
    dispose(&f); cases++;

    armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
    expire_in_cancel = true;
    max98357a_test_quiesce(&f.priv);
    assert(!f.gpio.value && f.priv.test_terminal && f.priv.test_timer.cancels == 1);
    assert(!f.priv.test_timer.pending);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM);
    dispose(&f); cases++;

    armed(&f); max98357a_test_quiesce(&f.priv);
    assert(trigger(&f, SNDRV_PCM_TRIGGER_START) == -EPERM && !f.gpio.highs);
    dispose(&f); cases++;

    armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START)); dispose(&f);
    fixture(&other); assert(!init(&other)); power(&other, SND_SOC_DAPM_POST_PMU);
    assert(trigger(&other, SNDRV_PCM_TRIGGER_START) == -EPERM && !other.gpio.highs);
    dispose(&other); cases++;

    for (unsigned int n = 0; n < 4; n++) {
        reset_module(); fixture(&f);
        if (n == 0) f.dev.tdm = false;
        if (n == 1) f.priv.sdmode = NULL;
        if (n == 2) f.gpio.sleeping = true;
        if (n == 3) f.priv.sdmode_delay = 1;
        assert(init(&f) == -EINVAL && !f.priv.test_gate && !f.dev.action);
        assert(!f.gpio.highs && !f.priv.test_timer.starts); cases++;
    }
    reset_module(); fixture(&f); f.dev.action_fail = true;
    assert(init(&f) == -ENOMEM && f.priv.test_terminal && !f.gpio.value);
    assert(f.priv.test_timer.cancels == 1);
    dispose(&f); cases++;

    reset_module(); fixture(&f); board_j71b = false;
    assert(!init(&f) && !f.priv.test_gate && !f.dev.action);
    power(&f, SND_SOC_DAPM_POST_PMU); f.priv.sdmode_delay = 3;
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_START) && f.gpio.value);
    assert(!trigger(&f, SNDRV_PCM_TRIGGER_STOP) && !f.gpio.value && delays == 3);
    assert(!f.priv.test_timer.starts); dispose(&f); cases++;

    armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
    assert(trigger(&f, 999) == -EINVAL && !f.gpio.value);
    dispose(&f); cases++;

    /* Actual callbacks with pthread locks exercise these serializations,
       not the real kernel's IRQ, hrtimer or driver-core scheduling. */
    for (unsigned int n = 0; n < 100; n++) {
        armed(&f); race(&f, start_thread, quiesce_thread);
        assert(f.gpio.highs <= 1 && f.priv.test_terminal);
        dispose(&f);
        armed(&f); assert(!trigger(&f, SNDRV_PCM_TRIGGER_START));
        race(&f, expire_thread, stop_thread);
        assert(f.gpio.highs == 1); dispose(&f);
    }
    cases += 2;
    printf("%u callback cases passed (including 200 mocked races)\n", cases);
    return 0;
#endif
}
'''


def callback_source():
    start = SOURCE.index("#define J71B_TEST_WINDOW_MS")
    end = SOURCE.index("static const struct snd_soc_dapm_widget")
    return SOURCE[start:end]


class GateTests(unittest.TestCase):
    def test_callbacks(self):
        compiler = os.environ.get("CC", "cc")
        with tempfile.TemporaryDirectory(prefix="j71b-codec-test-") as work:
            source = Path(work) / "test.c"
            source.write_text(STUBS + callback_source() + CASES)
            for enabled in (0, 1):
                for sanitizer in (False, True):
                    executable = Path(work) / f"test-{enabled}-{sanitizer}"
                    flags = ["-std=gnu11", "-Wall", "-Wextra", "-Werror",
                             "-Wno-unused-parameter", "-Wno-unused-function", "-g", "-pthread"]
                    if sanitizer:
                        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
                    subprocess.run([compiler, *flags,
                                    f"-DCONFIG_SND_SOC_MAX98357A_J71B_TEST={enabled}",
                                    str(source), "-o", str(executable)], check=True)
                    env = os.environ.copy()
                    # Some sandboxed runners cannot use LeakSanitizer's ptrace.
                    env.setdefault("ASAN_OPTIONS", "detect_leaks=0")
                    subprocess.run([str(executable)], check=True, env=env, timeout=30)

    def test_safety_regressions_are_detected(self):
        mutations = {
            "second_start": ("atomic_cmpxchg(&j71b_test_used, 0, 1)", "0"),
            "missing_mute": ("gpiod_set_value(max98357a->sdmode, 0);", "(void)max98357a;"),
            "missing_cancel": ("hrtimer_cancel(&max98357a->test_timer);", "(void)max98357a;"),
        }
        # reset_module deliberately simulates a fresh unarmed module, so the
        # source-policy test checks the initializer while runtime mutations
        # check the actual admission, GPIO and teardown callbacks.
        with tempfile.TemporaryDirectory(prefix="j71b-codec-mutation-") as work:
            for name, (old, new) in mutations.items():
                with self.subTest(name=name):
                    self.assertIn(old, callback_source())
                    source = Path(work) / f"{name}.c"
                    source.write_text(STUBS + callback_source().replace(old, new, 1) + CASES)
                    executable = Path(work) / name
                    subprocess.run([os.environ.get("CC", "cc"), "-std=gnu11", "-pthread",
                                    "-DCONFIG_SND_SOC_MAX98357A_J71B_TEST=1", str(source),
                                    "-o", str(executable)], check=True)
                    result = subprocess.run([str(executable)], capture_output=True,
                                            text=True, timeout=30)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("Assertion", result.stderr)

    def test_read_only_unarmed_default(self):
        self.assertIn("static bool j71b_test_arm;", SOURCE)
        self.assertIn("j71b_test_arm, bool, 0400)", SOURCE)
        self.assertNotIn("atomic_set(&j71b_test_used", SOURCE)
        self.assertEqual(SOURCE.count("atomic_cmpxchg(&j71b_test_used, 0, 1)"), 1)

    def test_resource_order(self):
        probe = SOURCE[SOURCE.index("static int max98357a_platform_probe"):]
        self.assertLess(probe.index("devm_gpiod_get_optional"), probe.index("max98357a_test_init"))
        self.assertLess(probe.index("max98357a_test_init"), probe.index("devm_snd_soc_register_component"))
        self.assertIn(".shutdown = max98357a_platform_shutdown", SOURCE)
        self.assertIn("max98357a_test_quiesce(dev_get_drvdata(&pdev->dev));", SOURCE)

    def test_diagnostic_configs(self):
        kconfig = (ROOT / "sound/soc/codecs/Kconfig").read_text()
        section = kconfig.split("config SND_SOC_MAX98357A_J71B_TEST\n", 1)[1].split("\nconfig ", 1)[0]
        self.assertIn("default n", section)
        self.assertIn("SND_SOC_MAX98357A && OF && HIGH_RES_TIMERS", section)
        for pages in (4, 16):
            config = (ROOT / f"arch/arm64/configs/orchard_apple{pages}k_defconfig").read_text()
            self.assertIn("CONFIG_SND_SOC_MAX98357A=m\n", config)
            self.assertIn("CONFIG_SND_SOC_MAX98357A_J71B_TEST=y\n", config)
            self.assertIn('CONFIG_LOCALVERSION="-j71b-speaker-window-nov7-r1"', config)

    def test_current_transport_unchanged(self):
        source = (ROOT / "sound/soc/apple/s8000-mca.c").read_text()
        t8010 = source.split("static const struct s8000_mca_hw t8010_mca_hw =", 1)[1]
        for expected in (".dai_format = SND_SOC_DAIFMT_DSP_A", ".channels = 4", ".slots = 8",
                         ".formats = SNDRV_PCM_FMTBIT_S16_LE", ".attenuation = 4"):
            self.assertIn(expected, t8010)
        self.assertIn("!mca->run_mask && !mca->hw->clock_provider", source)
        self.assertIn("sample[i] /= (int)mca->hw->attenuation", source)
        dts = (ROOT / "arch/arm64/boot/dts/apple/t8010-j71b.dts").read_text()
        self.assertIn('simple-audio-card,format = "dsp_a";', dts)
        self.assertIn("maxim,tdm;", dts)
        self.assertIn("sdmode-gpios = <&pinctrl_ap 20 GPIO_ACTIVE_HIGH>;", dts)


if __name__ == "__main__":
    unittest.main(verbosity=2)
