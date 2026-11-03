#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile actual GPIO handoff functions with instrumented host IRQ/work primitives.
Deterministic boundary/interleaving tests, not proof of Linux IRQ/lock semantics.
Pass the GPIO source path to test the integrated file rather than this snapshot.
"""
from pathlib import Path
import re, subprocess, tempfile, sys, os, shlex
ROOT = Path(__file__).resolve().parents[4]
src = Path(sys.argv[1] if len(sys.argv)>1 else ROOT / 'drivers/gpio/gpio-apple-d2365.c').read_text()
names = ['apple_pmic_event_sync', 'apple_pmic_irq_bus_sync_unlock', 'apple_pmic_irq_hold', 'apple_pmic_irq_retry', 'apple_pmic_event_gate', 'apple_pmic_dispatch', 'apple_pmic_event_thread', 'apple_d2365_gpio_events_ready', 'apple_d2365_gpio_quiesce', 'apple_pmic_event_stop']
def extract(name):
    m = re.search(r'^(?:static )?(?:int|void|bool|irqreturn_t) '+name+r'\([^;]*?\n\{', src, re.M)
    assert m, name
    end = src.index('\n}', m.end()) + 2
    return src[m.start():end]
shim = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
typedef uint8_t u8;
typedef unsigned long irq_hw_number_t;
typedef int irqreturn_t;
#define IRQ_HANDLED 1
#define APPLE_PMIC_EVENTS_MAX 240
#define APPLE_PMIC_EVENT_BYTES_MAX 30
#define APPLE_PMIC_IRQ_RETRIES 5
#define APPLE_PMIC_IRQ_BURST 64
#define BIT(n) (1UL << (n))
#define DECLARE_BITMAP(n,b) unsigned long n[4]
#define test_bit(n,b) (!!((b)[(n)/64] & BIT((n)%64)))
#define __set_bit(n,b) ((b)[(n)/64] |= BIT((n)%64))
#define bitmap_zero(b,n) memset(b,0,32)
#define bitmap_copy(a,b,n) memcpy(a,b,32)
#define for_each_set_bit(i,b,n) for(i=0;i<(n);i++) if(test_bit(i,b))
static bool bitmap_empty(const unsigned long *b,unsigned n) { for(unsigned i=0;i<n;i++)if(test_bit(i,b))return false;return true; }
#define mutex_lock pthread_mutex_lock
#define mutex_unlock pthread_mutex_unlock
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define to_delayed_work(p) container_of(p,struct delayed_work,work)
#define HZ 100
#define jiffies 10
#define time_after(a,b) ((a)>(b))
#define msecs_to_jiffies(n) (n)
#define dev_err(...) ((void)0)
#define dev_err_ratelimited(...) ((void)0)
#define pm_system_wakeup() ((void)0)
#define system_dfl_wq NULL
struct device { void *driver; void *data; };
static struct { int driver; } d2365_gpio_driver;
#define dev_get_drvdata(d) ((d)->data)
struct irq_data { void *data; };
#define irq_data_get_irq_chip_data(d) ((d)->data)
struct work_struct { int unused; };
struct delayed_work { struct work_struct work; bool pending; };
struct apple_pmic_event_info { unsigned status_base,mask_base,bytes; };
struct apple_pmic_gpio_info { const struct apple_pmic_event_info *events; };
struct apple_pmic_gpio {
 struct { struct device *parent; } chip;
 void *regmap; const struct apple_pmic_gpio_info *info; void *domain;
 unsigned nevents; int parent_irq; pthread_mutex_t irq_lock;
 struct delayed_work irq_retry;
 unsigned long enabled[4],wake[4]; u8 mask[30];
 int wake_delta; unsigned long irq_window; unsigned irq_burst,retries;
 bool suspended,irq_held,removing,shutting_down;
};
static struct apple_pmic_gpio g;
static struct device dev;
static int disables,enables,reads,writes,cancels,queues,syncs,wake_calls,frees;
static int read_error;
static pthread_mutex_t ctl=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv=PTHREAD_COND_INITIALIZER;
static bool running, pause_dispatch,entered,release_dispatch, sync_entered,quiesce_done,inject_retry;
static void apple_pmic_irq_retry(struct work_struct *work);
static void disable_irq_nosync(int irq) { assert(irq==1);disables++; }
static void enable_irq(int irq) { assert(irq==1);enables++; }
static void mod_delayed_work(void *q,struct delayed_work *w,unsigned long delay) { queues++;w->pending=true; }
static void cancel_delayed_work_sync(struct delayed_work *w) { cancels++;w->pending=false; }
static void free_irq(int irq,void *data) {
 assert(irq==1 && data==&g && !running && !g.irq_retry.pending);
 assert(++frees==1);
}
static void synchronize_irq(int irq) {
 assert(irq==1);syncs++;
 /* Simulate a retry already dequeued before cancellation, after the gate closes. */
 if(inject_retry)apple_pmic_irq_retry(&g.irq_retry.work);
 pthread_mutex_lock(&ctl);sync_entered=true;pthread_cond_broadcast(&cv);
 while(running)pthread_cond_wait(&cv,&ctl);
 pthread_mutex_unlock(&ctl);
}
static int irq_set_irq_wake(int irq,int on) {wake_calls++;return 0;}
static int regmap_write(void *m,unsigned r,unsigned v) {writes++;return 0;}
static int regmap_bulk_read(void *m,unsigned r,u8 *v,unsigned n) {reads++;memset(v,0,n);v[0]=1;return read_error;}
static unsigned irq_find_mapping(void *d,unsigned h) {return h+1;}
static void handle_nested_irq(unsigned child) {
 pthread_mutex_lock(&ctl);
 if(pause_dispatch) {
  entered=true;pthread_cond_broadcast(&cv);
  while(!release_dispatch)pthread_cond_wait(&cv,&ctl);
 }
 pthread_mutex_unlock(&ctl);
}
'''
tests = r'''
static void init(bool domain) {
 memset(&g,0,sizeof(g));pthread_mutex_init(&g.irq_lock,NULL);
 static const struct apple_pmic_event_info ev={0x140,0x1c0,30};
 static const struct apple_pmic_gpio_info info={&ev};
 dev=(struct device){&d2365_gpio_driver.driver,&g};
 g.info=&info;g.domain=domain?&g:NULL;g.parent_irq=1;g.nevents=240;g.chip.parent=&dev;
 memset(g.mask,0xff,sizeof(g.mask));
 disables=enables=reads=writes=cancels=queues=syncs=wake_calls=read_error=frees=0;
 running=pause_dispatch=entered=release_dispatch=sync_entered=quiesce_done=inject_retry=false;
}
static void fini(void) {pthread_mutex_destroy(&g.irq_lock);}
static void assert_closed(void) {
 int r=reads,w=writes,e=enables,q=queues;
 apple_pmic_event_thread(1,&g);
 apple_pmic_irq_retry(&g.irq_retry.work);
 pthread_mutex_lock(&g.irq_lock);apple_pmic_irq_hold(&g,true);pthread_mutex_unlock(&g.irq_lock);
 struct irq_data d={&g};g.enabled[0]=1;g.wake_delta=1;
 pthread_mutex_lock(&g.irq_lock);apple_pmic_irq_bus_sync_unlock(&d);
 assert(reads==r&&writes==w&&enables==e&&queues==q&&wake_calls==0);
 assert(g.shutting_down&&g.irq_held&&!g.irq_retry.pending);
}
static void *event_worker(void *p) {
 apple_pmic_event_thread(1,&g);
 pthread_mutex_lock(&ctl);running=false;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&ctl);return NULL;
}
static void *quiesce_worker(void *p) {
 apple_d2365_gpio_quiesce(&dev);
 pthread_mutex_lock(&ctl);quiesce_done=true;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&ctl);return NULL;
}
static void drain_test(bool preread) {
 init(true);pause_dispatch=true;running=true;inject_retry=true;
 /* Pre-read line callback or post-read nested event handler. */
 __set_bit(preread?g.nevents:0,g.enabled);
 g.irq_retry.pending=true;
 pthread_t ev,off;assert(!pthread_create(&ev,NULL,event_worker,NULL));
 pthread_mutex_lock(&ctl);while(!entered)pthread_cond_wait(&cv,&ctl);pthread_mutex_unlock(&ctl);
 assert(!pthread_create(&off,NULL,quiesce_worker,NULL));
 pthread_mutex_lock(&ctl);while(!sync_entered)pthread_cond_wait(&cv,&ctl);
 assert(!quiesce_done);assert(g.shutting_down&&g.irq_held);assert(enables==0);
 release_dispatch=true;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&ctl);
 pthread_join(ev,NULL);pthread_join(off,NULL);
 assert(quiesce_done&&disables==1&&syncs==1&&cancels==1);
 assert(reads==(preread?0:1));assert_closed();fini();
}
int main(void) {
 init(false);assert(!apple_d2365_gpio_events_ready(&dev));
 apple_d2365_gpio_quiesce(&dev);assert(disables==0&&syncs==0&&cancels==0);fini();
 init(true);assert(apple_d2365_gpio_events_ready(&dev));
 dev.driver=NULL;assert(!apple_d2365_gpio_events_ready(&dev));dev.driver=&d2365_gpio_driver.driver;
 apple_d2365_gpio_quiesce(&dev);assert(disables==1&&syncs==1&&cancels==1);assert_closed();
 apple_d2365_gpio_quiesce(&dev);assert(disables==1&&syncs==2&&cancels==2);assert_closed();
 assert(frees==0);apple_pmic_event_stop(&g);assert(frees==1&&g.removing);assert_closed();fini();
 init(true);pthread_mutex_lock(&g.irq_lock);apple_pmic_irq_hold(&g,true);pthread_mutex_unlock(&g.irq_lock);
 assert(disables==1&&queues==1&&g.irq_retry.pending);
 inject_retry=true;apple_d2365_gpio_quiesce(&dev);assert(disables==1&&enables==0);assert_closed();fini();
 /* Exercise actual failure->hold->retry path before the handoff. */
 init(true);read_error=-EIO;apple_pmic_event_thread(1,&g);
 assert(reads==1&&queues==1&&g.irq_held);apple_d2365_gpio_quiesce(&dev);assert_closed();fini();
 drain_test(true);drain_test(false);
 puts("PASS: actual GPIO readiness, handoff, retry, event thread and mask-sync functions; deterministic IRQ-drain interleavings");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='d2365-event-test-') as tmp:
    c=Path(tmp)/'test.c';exe=Path(tmp)/'test'
    c.write_text(shim+'\n'+'\n\n'.join(extract(n) for n in names)+'\n'+tests)
    subprocess.run([os.environ.get('CC','cc'),'-std=gnu11','-pthread','-Wall','-Wextra','-Werror','-Wno-unused-parameter',*shlex.split(os.environ.get('CFLAGS','')),str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=20)
