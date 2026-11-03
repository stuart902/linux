#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Offline checks of the test-branch observer; no device access."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(os.environ.get('D2365_KERNEL_SRC', Path(__file__).resolve().parents[4]))
SOURCE = ROOT / 'drivers/gpio/gpio-apple-d2365.c'


class MaskReadback(unittest.TestCase):
    def test_actual_observer_and_sync(self):
        text = SOURCE.read_text()
        code = text[text.index('static void apple_pmic_mask_observe'):text.index('static void apple_pmic_irq_bus_lock')]
        preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
typedef uint8_t u8;
#define APPLE_PMIC_EVENT_BYTES_MAX 30
#define BIT(n) (1UL << (n))
#define test_bit(n,p) (!!((p)[(n)/64] & BIT((n)%64)))
struct apple_pmic_event_info {unsigned int mask_base, bytes;};
struct info {const struct apple_pmic_event_info *events;};
struct apple_pmic_gpio {bool mask_test; unsigned int mask_test_samples;
 void *regmap; const struct info *info; u8 mask[30],boot_mask[30];};
static u8 hw[30];
static int reads,writes,logs,read_error,write_error,ignore_write,last_match;
static void *regmap_get_device(void *map) {return map;}
static int regmap_bulk_read(void *map,unsigned int reg,u8 *buf,unsigned int len) {
 (void)map; assert(reg>=0x1c0 && reg+len<=0x1de); reads++;
 if(read_error) return read_error;
 memcpy(buf,hw+reg-0x1c0,len); return 0;
}
static int regmap_write(void *map,unsigned int reg,u8 value) {
 (void)map; assert(reg>=0x1c0 && reg<0x1de); writes++;
 if(write_error) return write_error;
 if(!ignore_write) hw[reg-0x1c0]=value;
 return 0;
}
static void dev_info(void *dev,const char *format,...) {
 va_list ap; (void)dev; logs++; va_start(ap,format);
 if(strstr(format,"match=")) {
  (void)va_arg(ap,unsigned int); (void)va_arg(ap,unsigned int);
  last_match=va_arg(ap,int);
 }
 va_end(ap);
}
'''
        checks = r'''
int main(void) {
 struct apple_pmic_event_info ev={.mask_base=0x1c0,.bytes=30};
 struct info info={.events=&ev};
 struct apple_pmic_gpio g={.mask_test=true,.info=&info};
 unsigned long enabled[4]={0}; u8 expected[30];
 memset(expected,0xff,30); memset(hw,0xff,30); memset(g.mask,0xff,30);
 /* Probe success, mismatch/partial bulk, read failure: no state/write changes. */
 apple_pmic_mask_observe(&g,0x1c0,expected,30,0,true);
 assert(reads==1 && last_match==1 && logs==2 && writes==0);
 hw[15]=0x33;
 apple_pmic_mask_observe(&g,0x1c0,expected,30,0,true);
 assert(last_match==0 && writes==0 && g.mask[15]==0xff);
 read_error=-5;
 apple_pmic_mask_observe(&g,0x1c0,expected,30,0,true);
 assert(reads==3 && writes==0 && !g.mask_test); read_error=0;
 apple_pmic_mask_observe(&g,0x1c0,expected,30,0,true);
 assert(reads==3); g.mask_test=true;
 apple_pmic_mask_observe(&g,0x1c0,expected,30,-6,true);
 assert(reads==3 && writes==0 && !g.mask_test); g.mask_test=true;
 /* Ordinary byte writes preserve normal return and software shadow. */
 enabled[0]=1;
 assert(apple_pmic_event_sync(&g,enabled,true)==0);
 assert(writes==1 && reads==4 && g.mask[0]==0xfe && last_match==1);
 assert(apple_pmic_event_sync(&g,enabled,true)==0);
 assert(writes==1 && reads==4); /* unchanged skips entirely */
 ignore_write=1; enabled[0]=3;
 assert(apple_pmic_event_sync(&g,enabled,true)==0);
 assert(writes==2 && reads==5 && g.mask[0]==0xfc && last_match==0);
 write_error=-7; enabled[0]=7;
 assert(apple_pmic_event_sync(&g,enabled,true)==-7);
 assert(writes==3 && reads==5 && g.mask[0]==0xfc && !g.mask_test);
 g.mask_test=true;
 write_error=0; read_error=-8;
 assert(apple_pmic_event_sync(&g,enabled,true)==0);
 assert(writes==4 && reads==6 && g.mask[0]==0xf8 && !g.mask_test);
 g.mask_test=true;
 read_error=0; ignore_write=0;
 /* Hard cap does not wrap, and config/board gating causes zero observations. */
 g.mask_test_samples=15;
 apple_pmic_mask_observe(&g,0x1c0,expected,1,0,false);
 assert(g.mask_test_samples==16 && reads==7);
 for(int i=0;i<100;i++) apple_pmic_mask_observe(&g,0x1c0,expected,1,0,false);
 assert(g.mask_test_samples==16 && reads==7);
 g.mask_test=false;
 apple_pmic_mask_observe(&g,0x1c0,expected,30,0,true);
 assert(reads==7);
 g.mask_test=true; enabled[0]=15;
 assert(apple_pmic_event_sync(&g,enabled,false)==0);
 assert(writes==5 && reads==7 && g.mask[0]==0xf0);
 g.mask_test=false; enabled[0]=31;
 assert(apple_pmic_event_sync(&g,enabled,true)==0);
 assert(writes==6 && reads==7 && g.mask[0]==0xe0);
 puts("observer and sync cases passed");
}
'''
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'test.c'
            source.write_text(preamble + code + checks)
            binary = Path(d) / 'test'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=10)

    def test_probe_guard_and_cleanup_are_preserved(self):
        text = SOURCE.read_text()
        probe = text[text.index('static int apple_pmic_event_probe'):text.index('static struct platform_driver d2365_gpio_driver;')]
        self.assertIn('IS_ENABLED(CONFIG_GPIO_APPLE_D2365_MASK_TEST) &&', probe)
        self.assertIn('of_machine_is_compatible("apple,j71b")', probe)
        tail = probe.split('apple_pmic_mask_observe(gpio, ev->mask_base, gpio->mask,')[1]
        self.assertIn('ev->bytes, ret, true);\n\tif (ret)\n\t\treturn dev_err_probe(dev, ret, "failed to mask the events\\n");\n\tret = devm_add_action_or_reset(dev, apple_pmic_event_restore_mask, gpio);', tail)
        observer = text[text.index('static void apple_pmic_mask_observe'):text.index('/* Write the mask bytes')]
        self.assertNotIn('regmap_write', observer)
        self.assertNotIn('status_base', observer)
        self.assertNotIn('gpio->mask[', observer)


if __name__ == '__main__':
    unittest.main()
