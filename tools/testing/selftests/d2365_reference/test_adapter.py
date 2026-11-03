#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile the actual adapter against a bounded host lifecycle shim, not hardware."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
HEADER = ROOT / 'include/linux/power/apple-d2365-reference.h'
ADAPTER = ROOT / 'drivers/power/reset/apple-d2365-reference-adapter.c'
CORE = ROOT / 'drivers/power/reset/apple-d2365-reference-core.c'
MOCK = r'''
#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
#define __KERNEL__ 1
#define EPROBE_DEFER 517
#define GFP_KERNEL 0
#define DL_FLAG_AUTOREMOVE_CONSUMER 1
#define DL_DEV_PROBING 1
#define DL_DEV_DRIVER_BOUND 2
#define DL_DEV_UNBINDING 3
#define SYS_OFF_MODE_POWER_OFF_PREPARE 0
#define SYS_OFF_MODE_RESTART_PREPARE 1
#define SYS_OFF_MODE_POWER_OFF 2
#define SYS_OFF_PRIO_HIGH 192
#define NOTIFY_DONE 0
#define EXPORT_SYMBOL_GPL(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_LICENSE(x)
#define dev_err(...) ((void)0)
#define pr_emerg(...) ((void)0)
struct module { int refs; };
struct device_driver { struct module *owner; };
struct device_node { int compatible; };
struct device {
 struct device *parent;
 struct device_driver *driver;
 struct device_node *of_node;
 struct { int status; } links;
 int refs, locked, bound, event_owner;
};
struct regmap { struct device *dev; int val_bytes; u8 memory[0x5010]; };
struct device_link { int unused; };
struct sys_off_data { void *cb_data; };
struct handler { int active; int (*callback)(struct sys_off_data *); void *data; };
static struct handler handlers[3];
static struct {void (*fn)(void *); void *data;} actions[32];
static void *allocations[32];
static int action_count, allocation_count, steps, fail_at, bus_count, quiesce_error;
static int source_unresolved, sources_called, quiesce_called, existing_handler;
static int source_external, source_cable;
static int source_error, module_error, reset_called, irq_disabled;
static int machine_match;
static struct device_link link;
static jmp_buf terminal;
static bool fail(void) { return ++steps == fail_at; }
static bool device_trylock(struct device *d) {
 if(d->locked) return false;
 d->locked=1; return true;
}
static void device_unlock(struct device *d) { assert(d->locked); d->locked=0; }
static bool device_is_bound(struct device *d) { assert(d->locked); return d->bound; }
static bool try_module_get(struct module *m) { if(module_error) return false; if(m) m->refs++; return true; }
static void module_put(struct module *m) { if(m) { assert(m->refs>0); m->refs--; } }
static struct device *get_device(struct device *d) { d->refs++; return d; }
static void put_device(struct device *d) { assert(d->refs>0); d->refs--; }
static void *devm_kzalloc(struct device *d, size_t n, int flags) {
 if(fail()) return NULL;
 void *p=calloc(1,n); assert(p); allocations[allocation_count++]=p; return p;
}
static int devm_add_action_or_reset(struct device *d, void (*fn)(void *), void *data) {
 if(fail()) { fn(data); return -ENOMEM; }
 actions[action_count].fn=fn; actions[action_count++].data=data; return 0;
}
static struct device_link *device_link_add(struct device *c, struct device *p, int flags) {
 assert(p->locked && p->bound && p->links.status==DL_DEV_DRIVER_BOUND && c->links.status==DL_DEV_PROBING);
 return fail()?NULL:&link;
}
static void *devres_open_group(struct device *d, void *id, int flags) {
 assert(!action_count && !allocation_count); return fail()?NULL:id;
}
static void devres_remove_group(struct device *d, void *id) {}
static void devres_release_group(struct device *d, void *id) {
 while(action_count) { --action_count; actions[action_count].fn(actions[action_count].data); }
 while(allocation_count) free(allocations[--allocation_count]);
}
static bool kernel_can_power_off(void) { return existing_handler || handlers[2].active; }
static void unregister_handler(void *data) { ((struct handler *)data)->active=0; }
static int devm_register_sys_off_handler(struct device *d, int mode, int priority,
 int (*fn)(struct sys_off_data *), void *data) {
 assert(priority==SYS_OFF_PRIO_HIGH);
 if(fail()) return -EIO;
 assert(!handlers[mode].active);
 handlers[mode]=(struct handler){1,fn,data};
 return devm_add_action_or_reset(d,unregister_handler,&handlers[mode]);
}
static int regmap_get_val_bytes(struct regmap *r) {return r->val_bytes;}
static struct device *regmap_get_device(struct regmap *r) { return r->dev; }
static bool of_machine_is_compatible(const char *name) {
 assert(!strcmp(name,"apple,j71b")); return machine_match;
}
static bool of_device_is_compatible(struct device_node *node,const char *name) {
 assert(!strcmp(name,"apple,d2365-pmic-gpio")); return node && node->compatible;
}
static bool apple_d2365_gpio_events_ready(struct device *d) {
 return d->event_owner && !quiesce_error;
}
static void apple_d2365_gpio_quiesce(struct device *d) {
 assert(apple_d2365_gpio_events_ready(d)); quiesce_called++;
}
static int regmap_bulk_read(struct regmap *r,unsigned int reg,void *p,size_t n) {
 assert(quiesce_called && reg+n<=sizeof(r->memory)); bus_count++; memcpy(p,r->memory+reg,n); return 0;
}
static int regmap_bulk_write(struct regmap *r,unsigned int reg,const void *p,size_t n) {
 assert(quiesce_called && reg+n<=sizeof(r->memory)); bus_count++; memcpy(r->memory+reg,p,n); return 0;
}
static void machine_restart(const char *reason) { reset_called++; longjmp(terminal,1); }
static void local_irq_disable(void) { irq_disabled++; }
static void cpu_relax(void) { longjmp(terminal,2); }
'''
TEST = r'''
static int snapshot(struct device *d,struct d2365_sources *s) {
 sources_called++;
 *s=(struct d2365_sources){source_unresolved?D2365_PROPERTY_UNRESOLVED:
  source_external?D2365_PROPERTY_TRUE:D2365_PROPERTY_FALSE,
  source_cable?D2365_PROPERTY_TRUE:D2365_PROPERTY_FALSE};
 return source_error;
}
static struct module module;
static struct device_driver driver={&module};
static struct device_node node;
static struct device consumer,bus,source,event;
static struct regmap map;
static struct d2365_reference_resources resources;
static void clear(void) {
 devres_release_group(&consumer,NULL);
 assert(module.refs==0 && !bus.refs && !source.refs && !event.refs);
 memset(handlers,0,sizeof(handlers));
 steps=fail_at=bus_count=quiesce_error=source_unresolved=sources_called=0;
 source_error=module_error=reset_called=irq_disabled=quiesce_called=existing_handler=0;
 source_external=source_cable=0;
 machine_match=node.compatible=1;
 consumer=(struct device){.driver=&driver,.links={DL_DEV_PROBING}};
 bus=(struct device){.driver=&driver,.links={DL_DEV_DRIVER_BOUND},.bound=1};
 source=(struct device){.driver=&driver,.links={DL_DEV_DRIVER_BOUND},.bound=1};
 event=(struct device){.parent=&bus,.driver=&driver,.of_node=&node,.links={DL_DEV_DRIVER_BOUND},.bound=1,.event_owner=1};
 map=(struct regmap){.dev=&bus,.val_bytes=1};
 resources=(struct d2365_reference_resources){&source,snapshot,&event,true};
}
static void invoke(int mode) {
 assert(handlers[mode].active);
 struct sys_off_data off={handlers[mode].data}; handlers[mode].callback(&off);
}
int main(void) {
 int total;
 clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources)); total=steps;
 assert(module.refs==3 && handlers[0].active && handlers[1].active && handlers[2].active);
 for(int fail_step=1;fail_step<=total;fail_step++) {
  clear(); fail_at=fail_step;
  assert(devm_d2365_reference_register(&consumer,&map,&resources)<0);
  assert(!bus_count && !module.refs && !bus.refs && !source.refs && !event.refs);
  assert(!handlers[0].active && !handlers[1].active && !handlers[2].active);
 }
 for(int alias=0;alias<2;alias++) {
  clear(); resources.source_dev=alias?&bus:&event;
  assert(!devm_d2365_reference_register(&consumer,&map,&resources));
  assert(module.refs==3 && bus.refs==(alias?2:1) && event.refs==(alias?1:2));
 }
 for(int which=0;which<7;which++) {
  clear();
  if(which==0) bus.bound=0;
  if(which==1) source.bound=0;
  if(which==2) event.bound=0;
  if(which==3) resources.source_dev=NULL;
  if(which==4) module_error=1;
  if(which==5) consumer.links.status=0;
  if(which==6) map.val_bytes=4;
  assert(devm_d2365_reference_register(&consumer,&map,&resources)<0);
  assert(!bus_count && !module.refs);
 }
 for(int which=0;which<4;which++) {
  clear();
  if(which==0) machine_match=0;
  if(which==1) node.compatible=0;
  if(which==2) event.event_owner=0;
  if(which==3) event.of_node=NULL;
  assert(devm_d2365_reference_register(&consumer,&map,&resources)==-ENODEV);
  assert(!bus_count && !quiesce_called && !module.refs);
  assert(!handlers[0].active && !handlers[1].active && !handlers[2].active);
 }
 /* A supplier can still look bound while its unbind path drops its lock
  * to detach consumers. Refuse new links in every non-bound link state.
  */
 for(int which=0;which<3;which++) {
  clear();
  struct device *provider=which==0?&bus:which==1?&source:&event;
  provider->links.status=DL_DEV_UNBINDING;
  assert(provider->bound);
  assert(devm_d2365_reference_register(&consumer,&map,&resources)==-EPROBE_DEFER);
  assert(!bus_count && !module.refs && !bus.refs && !source.refs && !event.refs);
  assert(!handlers[0].active && !handlers[1].active && !handlers[2].active);
 }
 for(int which=0;which<3;which++) {
  clear();
  struct device *provider=which==0?&bus:which==1?&source:&event;
  provider->locked=1;
  assert(devm_d2365_reference_register(&consumer,&map,&resources)==-EPROBE_DEFER);
  assert(provider->locked && !bus_count && !module.refs && !bus.refs && !source.refs && !event.refs);
  provider->locked=0;
 }
 clear(); existing_handler=1;
 assert(devm_d2365_reference_register(&consumer,&map,&resources)==-EBUSY && !steps);
 for(int mode=0;mode<2;mode++) {
  clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources));
  quiesce_error=-EIO; invoke(mode);
  assert(!quiesce_called && !bus_count && !sources_called && !reset_called);
 }
 clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources));
 source_unresolved=1; invoke(0);
 assert(bus_count==5 && sources_called==1);
 int before=bus_count;
 if(!setjmp(terminal)) invoke(2);
 assert(bus_count==before && !reset_called && irq_disabled==1);
 clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources));
 invoke(0); assert(bus_count==25);
 before=bus_count;
 if(!setjmp(terminal)) invoke(2);
 assert(bus_count==before && !reset_called && irq_disabled==1);
 for(int which=0;which<2;which++) {
  clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources));
  source_external=which==0; source_cable=which==1;
  invoke(0); assert(bus_count==7 && sources_called==1 && quiesce_called==1);
  assert(((struct d2365_adapter *)handlers[2].data)->result.terminal==D2365_PLATFORM_RESET);
  before=bus_count;
  if(!setjmp(terminal)) invoke(2);
  assert(reset_called==1 && bus_count==before && !irq_disabled);
 }
 clear(); assert(!devm_d2365_reference_register(&consumer,&map,&resources));
 invoke(1); assert(bus_count==2 && sources_called==0);
 clear(); puts("adapter lifecycle shim: all cases passed"); return 0;
}
'''

def strip_includes(text):
    return re.sub(r'^#include[^\n]*\n', '', text, flags=re.M)

with tempfile.TemporaryDirectory() as tmp:
    src = Path(tmp) / 'adapter.c'
    src.write_text(MOCK + strip_includes(HEADER.read_text()) +
                   strip_includes(ADAPTER.read_text()) + TEST)
    out = Path(tmp) / 'adapter'
    command = shlex.split(os.environ.get('CC', 'cc'))
    command += ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter']
    command += shlex.split(os.environ.get('CFLAGS', ''))
    command += ['-I' + str(ROOT / 'include'), str(src), str(CORE), '-o', str(out)]
    subprocess.run(command, check=True)
    subprocess.run([str(out)], check=True)
