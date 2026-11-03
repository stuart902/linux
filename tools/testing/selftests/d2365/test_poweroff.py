#!/usr/bin/env python3
"""Offline D2365 shutdown regression checks; never access a device.

Compile the driver's actual prepare/final functions against an in-memory bus.
These tests establish software contracts, not PMIC electrical behavior.
"""
from pathlib import Path
import os
import sys
import re
import shutil
import subprocess
import tempfile
import unittest

DRIVER = 'drivers/power/reset/apple-d2365-poweroff.c'
ROOT = Path(os.environ['D2365_KERNEL_SRC']) if 'D2365_KERNEL_SRC' in os.environ else next(
    (parent for parent in Path(__file__).resolve().parents if (parent / DRIVER).is_file()),
    Path(__file__).resolve().parent)


def driver_source():
    return (ROOT / DRIVER).read_text()


class D2365Poweroff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = driver_source()
        compiler = shutil.which('cc') or shutil.which('clang')
        if compiler is None:
            raise unittest.SkipTest('A host C compiler is required for shutdown checks')
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        directory = Path(cls.temporary.name)
        # Include the actual algorithm and data, not a Python reimplementation.
        algorithm = cls.source.split('static const struct of_device_id')[0]
        algorithm = re.sub(r'^#include.*$', '', algorithm, flags=re.M)
        fixture = Path(__file__).resolve().parent
        program = (fixture / 'mock.h').read_text() + algorithm
        program += (fixture / 'checks.c').read_text()
        source = directory / 'check.c'
        source.write_text(program)
        cls.binary = directory / 'check'
        result = subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra',
                                 '-Wno-unused-parameter', '-Wno-sign-compare',
                                 str(source), '-o', str(cls.binary)],
                                capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def check(self, scenario):
        result = subprocess.run([str(self.binary), scenario], capture_output=True,
                                text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_standby_order_and_event_read_acknowledgement(self):
        self.check('order')

    def test_all_gpio_values_and_paired_bytes_preserved(self):
        self.check('gpio')

    def test_boot_state_preserves_only_verified_bits(self):
        self.check('boot')

    def test_every_nonzero_source_status_and_unknown_state_restarts(self):
        self.check('sources')

    def test_source_changes_before_final_request(self):
        self.check('changes')

    def test_each_transaction_failure_prevents_successful_standby(self):
        self.check('failures')

    def test_normal_restart_only_acknowledges_events_and_clears_boot_stage(self):
        self.check('restart')

    def test_final_callback_never_touches_bus_or_sleeps(self):
        self.check('final')

    def test_driver_enabled_in_both_platform_configs(self):
        for platform in ('apple16k', 'apple4k'):
            config = (ROOT / 'arch/arm64/configs' / ('orchard_' + platform + '_defconfig')).read_text()
            self.assertIn('CONFIG_POWER_RESET_APPLE_D2365=y\n', config)

    def test_j71b_poweroff_node_has_supply(self):
        source = (ROOT / 'arch/arm64/boot/dts/apple/t8010-j71b.dts').read_text()
        self.assertIn('compatible = "apple,d2365-poweroff";', source)
        self.assertIn('power-supplies = <&tristar_charger>;', source)

    def test_both_prepare_and_atomic_final_handlers_are_registered(self):
        for mode, callback in (
                ('SYS_OFF_MODE_RESTART_PREPARE', 'apple_d2365_restart_prepare'),
                ('SYS_OFF_MODE_POWER_OFF_PREPARE', 'apple_d2365_poweroff_prepare'),
                ('SYS_OFF_MODE_POWER_OFF', 'apple_d2365_poweroff_final')):
            self.assertRegex(self.source,
                             r'devm_register_sys_off_handler\(dev,\s*' + mode +
                             r',\s*SYS_OFF_PRIO_DEFAULT,\s*' + callback + r',\s*pmic\)')
        final = self.source.split('static int apple_d2365_poweroff_final(')[1]
        final = final.split('\n}\n', 1)[0]
        self.assertNotRegex(final, r'\b(?:regmap_\w+|msleep|usleep_range|mdelay)\s*\(')


class D2365EventQuiesce(unittest.TestCase):
    def test_shutdown_drains_irq_and_retry_after_closing_gate(self):
        post = (ROOT / 'drivers/gpio/gpio-apple-d2365.c').read_text()
        start = post.index('static void apple_pmic_gpio_shutdown(')
        shutdown = post[start:post.index('\n}', start) + 2]
        self.assertIn('.shutdown = apple_pmic_gpio_shutdown', post)
        self.assertIn('if (!retry || gpio->shutting_down)', post)
        self.assertIn('!gpio->removing && !gpio->shutting_down &&', post)
        self.assertIn('if (gpio->removing || gpio->shutting_down)', post)
        self.assertRegex(post, r'if \(gpio->shutting_down\) \{\s*'
                              r'mutex_unlock\(&gpio->irq_lock\);\s*return;')
        self.assertNotRegex(re.sub(r'/\*.*?\*/', '', shutdown, flags=re.S),
                            r'\b(?:free_irq|regmap_\w+|apple_pmic_event_sync)\s*\(')
        compiler = shutil.which('cc') or shutil.which('clang')
        if compiler is None:
            self.skipTest('A host C compiler is required for shutdown checks')
        # Compile the actual new shutdown callback. The helper assertions verify
        # lock ownership, closing the gate before holding IRQ, and drain order.
        program = r'''
#include <assert.h>
#include <stdbool.h>
struct apple_pmic_gpio {
    void *domain;
    bool shutting_down;
    int irq_lock, parent_irq, irq_retry;
};
struct platform_device { struct apple_pmic_gpio *data; };
static unsigned step;
static struct apple_pmic_gpio *platform_get_drvdata(struct platform_device *p)
{ return p->data; }
static void mutex_lock(int *lock) { assert(step++ == 0 && !*lock); *lock = 1; }
static void apple_pmic_irq_hold(struct apple_pmic_gpio *gpio, bool retry)
{ assert(step++ == 1 && gpio->irq_lock && gpio->shutting_down && !retry); }
static void mutex_unlock(int *lock) { assert(step++ == 2 && *lock); *lock = 0; }
static void synchronize_irq(int irq) { assert(step++ == 3 && irq == 7); }
static void cancel_delayed_work_sync(int *work) { assert(step++ == 4); }
'''
        program += shutdown + r'''
int main(void)
{
    struct apple_pmic_gpio gpio = { .parent_irq = 7 };
    struct platform_device pdev = { &gpio };
    apple_pmic_gpio_shutdown(&pdev);
    assert(!step && !gpio.shutting_down);
    gpio.domain = &gpio;
    apple_pmic_gpio_shutdown(&pdev);
    assert(step == 5 && gpio.shutting_down && !gpio.irq_lock);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'check.c'
            binary = Path(temporary) / 'check'
            source.write_text(program)
            subprocess.run([compiler, '-std=c11', str(source), '-o', str(binary)],
                           check=True, capture_output=True, text=True, timeout=30)
            subprocess.run([str(binary)], check=True, capture_output=True,
                           text=True, timeout=5)


if __name__ == '__main__':
    if not (ROOT / DRIVER).is_file():
        print('SKIP: matching kernel sources required; set D2365_KERNEL_SRC')
        sys.exit(4)
    unittest.main()
