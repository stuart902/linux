#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile actual USB current API functions with instrumented host primitives.

Tests callback/fallback semantics, provenance, finite global session allocation,
concurrent allocation, disabled stubs, and supply parent-unbind guards. These
host tests do not prove Linux IRQ/locking semantics or replace kernel builds.
Set CC/CFLAGS to select a compiler and optional sanitizer instrumentation.
"""
from pathlib import Path
import hashlib
import os
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
def func(file, signature):
 s=(root/file).read_text(); start=s.index(signature); end=s.index('{',start); depth=1; end+=1
 while depth:
  depth += (s[end]=='{')-(s[end]=='}'); end+=1
 return s[start:end]
pre=r'''
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "linux/usb/current.h"
#define S64_MAX INT64_MAX
#define IS_ERR_OR_NULL(p) (!(p) || (uintptr_t)(p) >= (uintptr_t)-4095)
#define KOBJ_CHANGE 1
#define max(a,b) ((a)>(b)?(a):(b))
#define DBG(...) do {} while (0)
struct mutex { pthread_mutex_t lock; bool held; };
static void mutex_lock(struct mutex *m) { assert(!pthread_mutex_lock(&m->lock)); m->held=true; }
static void mutex_unlock(struct mutex *m) { m->held=false; assert(!pthread_mutex_unlock(&m->lock)); }
typedef atomic_llong atomic64_t;
#define ATOMIC64_INIT(v) ATOMIC_VAR_INIT(v)
static s64 atomic64_read(atomic64_t *v) { return atomic_load(v); }
static s64 atomic64_cmpxchg(atomic64_t *v, s64 old, s64 val) { long long expected=old; atomic_compare_exchange_strong(v,&expected,val); return expected; }
typedef atomic_int atomic_t;
static int atomic_read(atomic_t *v) { return atomic_load(v); }
struct usb_gadget;
struct usb_gadget_ops {
 int (*vbus_draw)(struct usb_gadget *,unsigned);
 int (*vbus_draw_reason)(struct usb_gadget *,unsigned,enum usb_current_reason);
};
struct usb_gadget { const struct usb_gadget_ops *ops; unsigned mA; };
struct usb_composite_dev { struct usb_gadget *gadget; unsigned vbus_draw_ma; int apple_extra_ma; };
static int trace_count, trace_ret, calls, legacy_calls, cb_ret;
static unsigned seen_ma;
static enum usb_current_reason seen_reason;
static void trace_usb_gadget_vbus_draw(struct usb_gadget *g, int r) { (void)g; trace_count++; trace_ret=r; }
static int draw(struct usb_gadget *g,unsigned m) { (void)g; legacy_calls++; seen_ma=m; return cb_ret; }
static int draw_reason(struct usb_gadget *g,unsigned m,enum usb_current_reason r) { (void)g; calls++; seen_ma=m; seen_reason=r; return cb_ret; }
int usb_gadget_vbus_draw_reason(struct usb_gadget *,unsigned,enum usb_current_reason);
enum usb_role { USB_ROLE_NONE, USB_ROLE_HOST, USB_ROLE_DEVICE };
struct usb_role_switch {
 struct mutex lock; bool registered; enum usb_role role;
 int (*set)(struct usb_role_switch *,enum usb_role);
 int (*set_with_current_session)(struct usb_role_switch *,enum usb_role,u64);
 struct { int kobj; } dev;
 int calls, legacy_calls, ret; u64 seen_session; enum usb_role seen_role;
};
static void kobject_uevent(int *obj,int type) { assert(type==KOBJ_CHANGE); ++*obj; }
static int set_role(struct usb_role_switch *s,enum usb_role role) { assert(s->lock.held); s->legacy_calls++; s->seen_role=role; return s->ret; }
static int set_session(struct usb_role_switch *s,enum usb_role role,u64 token) { assert(s->lock.held); s->calls++; s->seen_role=role; s->seen_session=token; return s->ret; }
struct power_supply;
struct power_supply_desc { int (*usb_current_event)(struct power_supply *,const struct usb_current_event *); };
#define READ_ONCE(x) (x)
#define DL_DEV_DRIVER_BOUND 2
struct device { struct mutex lock; bool bound; struct { int status; } links; };
static void device_lock(struct device *dev) { mutex_lock(&dev->lock); }
static void device_unlock(struct device *dev) { mutex_unlock(&dev->lock); }
static bool device_is_bound(struct device *dev) { assert(dev->lock.held); return dev->bound; }
struct power_supply { atomic_t use_cnt; bool initialized,removing; const struct power_supply_desc *desc; struct device *usb_current_parent; };
static const struct usb_current_event *seen_event;
static int power_calls, power_ret;
static int submit_event(struct power_supply *p,const struct usb_current_event *e) { assert(p->usb_current_parent->lock.held); power_calls++; seen_event=e; return power_ret; }
'''
parts=[pre]
for file, signature in [
 ('drivers/usb/gadget/udc/core.c','int usb_gadget_vbus_draw('),
 ('drivers/usb/gadget/udc/core.c','int usb_gadget_vbus_draw_reason('),
 ('drivers/usb/gadget/composite.c','static void composite_vbus_draw('),
 ('drivers/usb/gadget/composite.c','static int composite_apple_current_limit('),
 ('drivers/usb/roles/class.c','int usb_role_switch_set_role('),
]: parts.append(func(file,signature))
parts.append('static atomic64_t usb_current_session = ATOMIC64_INIT(0);')
for file,signature in [
 ('drivers/usb/roles/class.c','static int usb_role_switch_next_current_session('),
 ('drivers/usb/roles/class.c','int usb_role_switch_set_role_with_current_session('),
 ('drivers/power/supply/power_supply_core.c','int power_supply_submit_usb_current('),
]: parts.append(func(file,signature))
parts.append(r'''
static void init_switch(struct usb_role_switch *s, bool typed) {
 *s=(struct usb_role_switch){.registered=true,.set=set_role};
 assert(!pthread_mutex_init(&s->lock.lock,NULL));
 if(typed) s->set_with_current_session=set_session;
}
static void test_gadget(void) {
 struct usb_gadget_ops ops={0}; struct usb_gadget g={.ops=&ops,.mA=70};
 assert(usb_gadget_vbus_draw(&g,500)==-EOPNOTSUPP); assert(g.mA==70); assert(trace_count==1 && trace_ret==-EOPNOTSUPP);
 ops.vbus_draw=draw; cb_ret=0;
 assert(!usb_gadget_vbus_draw_reason(&g,100,USB_CURRENT_RESET)); assert(g.mA==100 && seen_ma==100 && legacy_calls==1 && calls==0);
 cb_ret=-EIO; assert(usb_gadget_vbus_draw(&g,200)==-EIO); assert(g.mA==100 && trace_ret==-EIO && trace_count==3);
 ops.vbus_draw_reason=draw_reason; cb_ret=0;
 assert(!usb_gadget_vbus_draw(&g,333)); assert(seen_reason==USB_CURRENT_UNKNOWN && g.mA==333 && calls==1 && legacy_calls==2);
 assert(!usb_gadget_vbus_draw_reason(&g,500,USB_CURRENT_CONFIGURED)); assert(seen_reason==USB_CURRENT_CONFIGURED && g.mA==500);
 cb_ret=-EINVAL; assert(usb_gadget_vbus_draw_reason(&g,900,USB_CURRENT_VENDOR)==-EINVAL); assert(g.mA==500 && legacy_calls==2 && trace_count==6);
 cb_ret=0;
 struct usb_composite_dev c={.gadget=&g};
 for(int r=USB_CURRENT_UNKNOWN;r<=USB_CURRENT_LOST;r++) { composite_vbus_draw(&c,100,r); assert(seen_ma==100 && seen_reason==(enum usb_current_reason)r && c.vbus_draw_ma==100); }
 c.apple_extra_ma=1000; composite_vbus_draw(&c,100,USB_CURRENT_CONFIGURED); assert(seen_ma==1500 && seen_reason==USB_CURRENT_VENDOR && c.vbus_draw_ma==100);
 composite_vbus_draw(&c,900,USB_CURRENT_RESUME); assert(seen_ma==1500 && seen_reason==USB_CURRENT_VENDOR && c.vbus_draw_ma==900);
 composite_vbus_draw(&c,0,USB_CURRENT_DISCONNECT); assert(seen_ma==0 && seen_reason==USB_CURRENT_DISCONNECT && c.vbus_draw_ma==0);
 c.apple_extra_ma=-1000; composite_vbus_draw(&c,100,USB_CURRENT_RESUME); assert(seen_ma==0 && seen_reason==USB_CURRENT_VENDOR && c.vbus_draw_ma==100);
 assert(!composite_apple_current_limit(&c,0,0)); assert(seen_reason==USB_CURRENT_VENDOR && seen_ma==100);
 assert(!composite_apple_current_limit(&c,0,(uint16_t)-500)); assert(seen_reason==USB_CURRENT_VENDOR && seen_ma==0);
 puts("gadget: legacy fallback, return/current/trace, UNKNOWN wrapper, all reason preservation, Apple overrides passed");
}
#define NTHREAD 8
#define NSESSION 1000
static u64 allocated[NTHREAD * NSESSION];
static void *allocator_thread(void *arg) {
 size_t ix=(size_t)arg; struct usb_role_switch s; init_switch(&s,true);
 for(int i=0;i<NSESSION;i++) { assert(!usb_role_switch_set_role_with_current_session(&s,&allocated[ix*NSESSION+i])); assert(allocated[ix*NSESSION+i]>0); }
 assert(s.calls==NSESSION && s.dev.kobj==NSESSION); assert(!pthread_mutex_destroy(&s.lock.lock)); return NULL;
}
static int cmp_u64(const void *a,const void *b) { u64 x=*(const u64 *)a,y=*(const u64 *)b; return (x>y)-(x<y); }
static void test_role(void) {
 struct usb_role_switch legacy, typed, reprobe; u64 token=999,first;
 init_switch(&legacy,false); init_switch(&typed,true); init_switch(&reprobe,true);
 assert(usb_role_switch_set_role_with_current_session(&legacy,&token)==-EOPNOTSUPP && token==0);
 assert(usb_role_switch_set_role_with_current_session(NULL,&token)==-EOPNOTSUPP && token==0);
 assert(usb_role_switch_set_role_with_current_session(&typed,NULL)==-EINVAL);
 assert(!usb_role_switch_set_role(&legacy,USB_ROLE_HOST)); assert(legacy.legacy_calls==1 && legacy.role==USB_ROLE_HOST && legacy.dev.kobj==1);
 assert(!usb_role_switch_set_role(&typed,USB_ROLE_HOST)); assert(typed.calls==1 && typed.seen_session==0 && typed.role==USB_ROLE_HOST);
 assert(!usb_role_switch_set_role_with_current_session(&typed,&token)); first=token; assert(first==1 && typed.seen_session==first && typed.seen_role==USB_ROLE_DEVICE && typed.role==USB_ROLE_DEVICE);
 typed.ret=-EBUSY; typed.role=USB_ROLE_NONE; int events=typed.dev.kobj;
 assert(usb_role_switch_set_role_with_current_session(&typed,&token)==-EBUSY && token==0); assert(typed.seen_session==first+1 && typed.role==USB_ROLE_NONE && typed.dev.kobj==events);
 assert(!usb_role_switch_set_role_with_current_session(&reprobe,&token)); assert(token==first+2);
 typed.registered=false; int oldcalls=typed.calls;
 token=999; assert(usb_role_switch_set_role_with_current_session(&typed,&token)==-EOPNOTSUPP && token==0 && typed.calls==oldcalls);
 assert(usb_role_switch_set_role(&typed,USB_ROLE_DEVICE)==-EOPNOTSUPP && typed.calls==oldcalls);
 pthread_t threads[NTHREAD];
 for(size_t i=0;i<NTHREAD;i++) assert(!pthread_create(&threads[i],NULL,allocator_thread,(void *)i));
 for(int i=0;i<NTHREAD;i++) assert(!pthread_join(threads[i],NULL));
 qsort(allocated,NTHREAD*NSESSION,sizeof(u64),cmp_u64);
 u64 *a=(u64 *)allocated; for(int i=1;i<NTHREAD*NSESSION;i++) assert(a[i]==a[i-1]+1);
 atomic_store(&usb_current_session,S64_MAX-1);
 assert(!usb_role_switch_set_role_with_current_session(&reprobe,&token) && token==S64_MAX);
 oldcalls=reprobe.calls; token=999;
 assert(usb_role_switch_set_role_with_current_session(&reprobe,&token)==-EOVERFLOW && token==0 && reprobe.calls==oldcalls);
 assert(atomic64_read(&usb_current_session)==S64_MAX);
 assert(!pthread_mutex_destroy(&legacy.lock.lock)); assert(!pthread_mutex_destroy(&typed.lock.lock)); assert(!pthread_mutex_destroy(&reprobe.lock.lock));
 puts("role: legacy/session dispatch, lock scope, failure output, consumed tokens, reprobe uniqueness, 8000 concurrent sessions, S64_MAX nonwrap passed");
}
static void test_power(void) {
 struct device parent={.bound=true,.links.status=DL_DEV_DRIVER_BOUND};
 assert(!pthread_mutex_init(&parent.lock.lock,NULL));
 struct power_supply_desc desc={.usb_current_event=submit_event}; struct power_supply p={.use_cnt=1,.initialized=true,.desc=&desc,.usb_current_parent=&parent}; struct usb_current_event e={.session=1,.sequence=1,.milliamps=100};
 for(int r=USB_CURRENT_UNKNOWN;r<=USB_CURRENT_LOST;r++) { e.reason=r; assert(!power_supply_submit_usb_current(&p,&e) && seen_event==&e); }
 int n=power_calls; e.reason=(enum usb_current_reason)-1; assert(power_supply_submit_usb_current(&p,&e)==-EINVAL && power_calls==n);
 e.reason=USB_CURRENT_LOST+1; assert(power_supply_submit_usb_current(&p,&e)==-EINVAL); e.reason=USB_CURRENT_CONFIGURED;
 e.session=0; assert(power_supply_submit_usb_current(&p,&e)==-EINVAL); e.session=1; e.sequence=0; assert(power_supply_submit_usb_current(&p,&e)==-EINVAL); e.sequence=1;
 assert(power_supply_submit_usb_current(&p,NULL)==-EINVAL && power_calls==n);
 p.use_cnt=0; p.initialized=false; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV); p.initialized=true; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV);
 p.use_cnt=-1; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV); p.use_cnt=1;
 desc.usb_current_event=NULL; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV); desc.usb_current_event=submit_event;
 p.removing=true; p.desc=(void *)1; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV);
 p.removing=false; parent.bound=false; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV);
 parent.bound=true; parent.links.status=0; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV);
 parent.links.status=DL_DEV_DRIVER_BOUND; p.usb_current_parent=NULL; assert(power_supply_submit_usb_current(&p,&e)==-ENODEV);
 p.usb_current_parent=&parent; p.desc=&desc;
 power_ret=-EAGAIN; assert(power_supply_submit_usb_current(&p,&e)==-EAGAIN);
 assert(!parent.lock.held); assert(!pthread_mutex_destroy(&parent.lock.lock));
 puts("power: immutable pointer, validation, parent lock/binding/removal guards, stale descriptor exclusion, propagated result passed");
}
int main(void) { test_gadget(); test_role(); test_power(); return 0; }
''')

def extract(text, sig):
 start=text.index(sig); end=text.index('{',start)+1; depth=1
 while depth:
  depth+=(text[end]=='{')-(text[end]=='}'); end+=1
 return text[start:end]

extra=[]
for name,sig,old,new in [
 ('include/linux/usb/gadget.h','static inline int usb_gadget_vbus_draw_reason(', 'usb_gadget_vbus_draw_reason','gadget_disabled'),
 ('include/linux/usb/role.h','static inline int\nusb_role_switch_set_role_with_current_session(', 'usb_role_switch_set_role_with_current_session','role_disabled'),
 ('include/linux/power_supply.h','static inline int\npower_supply_submit_usb_current(', 'power_supply_submit_usb_current','power_disabled'),
]: extra.append(extract((root/name).read_text(),sig).replace(old,new))
old = r'''static void composite_vbus_draw(struct usb_composite_dev *cdev, unsigned int mA)
{
	cdev->vbus_draw_ma = mA;
	if (mA && cdev->apple_extra_ma)
		mA = max(500 + cdev->apple_extra_ma, 0);
	usb_gadget_vbus_draw(cdev->gadget, mA);
}'''
extra.append(extract(old,'static void composite_vbus_draw(').replace('composite_vbus_draw(', 'reference_composite_vbus_draw('))
extra.append(r'''
static void test_more(void) {
 u64 token=999;
 assert(gadget_disabled(NULL,100,USB_CURRENT_RESET)==0);
 assert(role_disabled(NULL,&token)==-EOPNOTSUPP && token==0);
 assert(role_disabled(NULL,NULL)==-EOPNOTSUPP);
 assert(power_disabled(NULL,NULL)==-EOPNOTSUPP);
 struct usb_gadget_ops ops={.vbus_draw_reason=draw_reason}; struct usb_gadget g={.ops=&ops};
 struct usb_composite_dev a={.gadget=&g},b={.gadget=&g}; unsigned bases[]={0,2,100,500,900};
 cb_ret=0;
 for(int extra=INT16_MIN;extra<=INT16_MAX;extra++) {
  for(size_t i=0;i<sizeof(bases)/sizeof(*bases);i++) {
   a.apple_extra_ma=extra; b.apple_extra_ma=extra;
   reference_composite_vbus_draw(&a,bases[i]); unsigned old_ma=seen_ma;
   composite_vbus_draw(&b,bases[i],USB_CURRENT_CONFIGURED);
   assert(old_ma==seen_ma && a.vbus_draw_ma==b.vbus_draw_ma);
   assert(seen_reason==(bases[i] && extra ? USB_CURRENT_VENDOR : USB_CURRENT_CONFIGURED));
  }
 }
 puts("disabled stubs and 327680 exhaustive signed Apple-current legacy comparisons passed");
}
''')

source = '\n\n'.join(parts)
source = source.replace('int main(void) {', '\n\n'.join(extra) + '\nint main(void) {')
source = source.replace('test_power(); return 0;', 'test_power(); test_more(); return 0;')

setter_source = (root / 'drivers/power/supply/power_supply_core.c').read_text()
a = 'static int __power_supply_set_property('
b = 'EXPORT_SYMBOL_GPL(power_supply_set_property);'
setter = setter_source[setter_source.index(a):setter_source.index(b) + len(b)]
assert hashlib.sha256(setter.encode()).hexdigest() == '2e09389b35839b3bb46c44c64e60f5a7fb2113d539e829062b026bee613e50c9'
assert 'psy->usb_current_parent = get_device(parent);' in setter_source
assert 'put_device(psy->usb_current_parent);' in func('drivers/power/supply/power_supply_core.c', 'static void power_supply_dev_release(')
print('Legacy power setter unchanged; opted-in parent retention and release present', flush=True)

with tempfile.TemporaryDirectory(prefix='usb-current-api-') as tmp:
    tmp = Path(tmp)
    (tmp / 'include/linux').mkdir(parents=True)
    (tmp / 'include/linux/types.h').write_text(
        '#include <stdint.h>\ntypedef uint64_t u64;\n'
        'typedef int64_t s64;\ntypedef int16_t s16;\ntypedef uint16_t u16;\n')
    (tmp / 'test.c').write_text(source)
    command = shlex.split(os.environ.get('CC', 'cc'))
    command += ['-std=gnu11', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-parameter', '-pthread']
    command += shlex.split(os.environ.get('CFLAGS', ''))
    command += ['-I' + str(tmp / 'include'), '-I' + str(root / 'include'),
                str(tmp / 'test.c'), '-o', str(tmp / 'test')]
    subprocess.run(command, check=True)
    subprocess.run([str(tmp / 'test')], check=True)
