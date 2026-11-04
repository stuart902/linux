#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Execute production DWC2/Tristar stream functions with host lock/work/I/O shims."""
from pathlib import Path
import os, re, shlex, subprocess, tempfile

ROOT = Path(__file__).resolve().parents[4]

def function(path, name):
    text = (ROOT / path).read_text()
    match = re.search(r'^(?:static )?(?:void|int|u64|bool)\s+' + name + r'\([^;]*?\)\n\{', text, re.M)
    if not match:
        raise AssertionError(name)
    start, at, depth = match.start(), match.end(), 1
    while depth:
        depth += (text[at] == '{') - (text[at] == '}')
        at += 1
    return text[start:at] + '\n'

preamble = r'''
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <linux/usb/current-stream.h>
#define U64_MAX UINT64_MAX
#define IS_ENABLED(x) 1
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define dev_err(...) ((void)0)
typedef pthread_mutex_t spinlock_t;
struct mutex { pthread_mutex_t lock; };
#define spin_lock_init(p) assert(!pthread_mutex_init((p),NULL))
#define spin_lock_irqsave(p,f) do { (f)=0; assert(!pthread_mutex_lock(p)); } while(0)
#define spin_unlock_irqrestore(p,f) do { (void)(f); assert(!pthread_mutex_unlock(p)); } while(0)
#define mutex_lock(p) assert(!pthread_mutex_lock(&(p)->lock))
#define mutex_unlock(p) assert(!pthread_mutex_unlock(&(p)->lock))
struct work_struct { void (*fn)(struct work_struct *); unsigned int scheduled; };
#define INIT_WORK(p,f) do { (p)->fn=(f); (p)->scheduled=0; } while(0)
static void schedule_work(struct work_struct *work) { work->scheduled++; }
static void flush_work(struct work_struct *work) { work->scheduled=0; work->fn(work); }
enum usb_role { USB_ROLE_NONE, USB_ROLE_HOST, USB_ROLE_DEVICE };
#define USB_DR_MODE_PERIPHERAL 2
#define USB_DR_MODE_UNKNOWN 0
#define GINTSTS_USBRST 1
#define GINTSTS_RESETDET 2
#define GINTSTS 3
struct dwc2_hsotg {
 spinlock_t lock,current_lock;
 struct mutex role_sw_lock;
 struct work_struct current_work;
 struct usb_current_queue current_queue;
 u64 current_pending,current_active,current_sequence;
 bool current_stopped,current_armed,role_sw_driven,role_sw_hold,in_ppd,hibernated,bus_suspended,ll_hw_enabled,enabled;
 int dr_mode,role_sw_default_mode;
 const char *usb_psy_name;
 unsigned int writes,roles;
};
struct usb_role_switch { struct dwc2_hsotg *hsotg; };
struct tristar_charger {
 struct mutex lock;
 spinlock_t current_lock;
 struct usb_current_queue current_queue;
 u64 current_session,current_sequence;
 bool current_stopped,usb_on,stopping;
 unsigned int epoch,usb_ma;
 struct usb_role_switch *role_sw;
 void *dev;
};
struct power_supply { struct tristar_charger *chg; };
static struct power_supply *supply;
static void *power_supply_get_drvdata(struct power_supply *psy) { return psy->chg; }
static struct power_supply *power_supply_get_by_name(const char *name)
{ assert(name); return supply; }
static void power_supply_put(struct power_supply *psy) { (void)psy; }
static int tristar_usb_current_event(struct power_supply *,const struct usb_current_event *);
static int power_supply_submit_usb_current(struct power_supply *psy,const struct usb_current_event *event)
{ return tristar_usb_current_event(psy,event); }
static void *usb_role_switch_get_drvdata(struct usb_role_switch *sw) { return sw->hsotg; }
static bool dwc2_is_device_enabled(struct dwc2_hsotg *h) { return h->enabled; }
static void dwc2_writel(struct dwc2_hsotg *h,unsigned int value,unsigned int reg)
{ assert(value==(GINTSTS_USBRST|GINTSTS_RESETDET)); assert(reg==GINTSTS);h->writes++; }
static int dwc2_drd_set_role(struct dwc2_hsotg *h,enum usb_role role)
{ h->roles++;h->role_sw_hold=role!=USB_ROLE_DEVICE;return 0; }
'''
names = ['dwc2_current_emit_locked','dwc2_current_lose_locked','dwc2_current_emit',
         'dwc2_current_work','dwc2_current_init','dwc2_current_bind',
         'dwc2_current_arm','dwc2_current_disconnect','dwc2_current_pending','dwc2_current_reset','dwc2_current_stop']
production = ''.join(function('drivers/usb/dwc2/gadget.c', n) for n in names)
production += function('drivers/usb/dwc2/drd.c','dwc2_drd_soft_role')
production += function('drivers/usb/dwc2/drd.c','dwc2_drd_role_sw_set_current')
production += function('drivers/power/supply/apple_tristar_charger.c','tristar_current_invalidate')
production += function('drivers/power/supply/apple_tristar_charger.c','tristar_usb_current_event')
role_fake = r'''
static bool inject_source_event,inject_reset,spawn_delivery;
static u64 next_session;
static pthread_t delivery;
static void *deliver_thread(void *arg) { dwc2_current_work(arg);return NULL; }
static int usb_role_switch_set_role_with_current_session(struct usb_role_switch *sw,u64 *session)
{
 int ret=dwc2_drd_role_sw_set_current(sw,USB_ROLE_DEVICE,++next_session);
 if(ret) { *session=0;return ret; }
 *session=next_session;
 if(inject_source_event) {
  unsigned long flags;
  spin_lock_irqsave(&supply->chg->current_lock,flags);
  supply->chg->current_session=0;supply->chg->epoch++;
  spin_unlock_irqrestore(&supply->chg->current_lock,flags);
 }
 if(inject_reset) dwc2_current_reset(sw->hsotg,dwc2_current_pending(sw->hsotg));
 if(spawn_delivery) assert(!pthread_create(&delivery,NULL,deliver_thread,&sw->hsotg->current_work));
 return 0;
}
static int usb_role_switch_set_role(struct usb_role_switch *sw,enum usb_role role)
{ return dwc2_drd_role_sw_set_current(sw,role,0); }
'''
production2 = function('drivers/power/supply/apple_tristar_charger.c','tristar_set_usb')
tests = r'''
struct fixture { struct dwc2_hsotg h;struct tristar_charger c;struct usb_role_switch sw;struct power_supply psy; };
static void setup(struct fixture *f)
{
 memset(f,0,sizeof(*f));f->sw.hsotg=&f->h;f->c.role_sw=&f->sw;f->psy.chg=&f->c;supply=&f->psy;
 spin_lock_init(&f->h.lock);spin_lock_init(&f->c.current_lock);
 assert(!pthread_mutex_init(&f->h.role_sw_lock.lock,NULL));assert(!pthread_mutex_init(&f->c.lock.lock,NULL));
 dwc2_current_init(&f->h);usb_current_queue_init(&f->c.current_queue);
 f->h.dr_mode=USB_DR_MODE_PERIPHERAL;f->h.role_sw_default_mode=USB_DR_MODE_UNKNOWN;
 f->h.role_sw_hold=true;f->h.enabled=true;f->h.ll_hw_enabled=true;f->h.usb_psy_name="lightning";
 inject_source_event=inject_reset=spawn_delivery=false;
}
static void finish(struct fixture *f)
{
 dwc2_current_stop(&f->h);
 assert(!pthread_mutex_destroy(&f->h.lock));assert(!pthread_mutex_destroy(&f->h.current_lock));
 assert(!pthread_mutex_destroy(&f->c.current_lock));assert(!pthread_mutex_destroy(&f->h.role_sw_lock.lock));
 assert(!pthread_mutex_destroy(&f->c.lock.lock));
}
static void begin(struct fixture *f)
{
 mutex_lock(&f->c.lock);tristar_set_usb(&f->c,true);mutex_unlock(&f->c.lock);
 assert(f->c.current_session && f->h.current_pending==f->c.current_session);
}
static struct usb_current_event take(struct fixture *f)
{
 struct usb_current_event e;assert(usb_current_queue_take(&f->c.current_queue,&e));return e;
}
static void path_and_reasons(void)
{
 struct fixture f;struct usb_current_event e;setup(&f);begin(&f);
 dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);assert(!f.h.current_queue.count);
 dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));
 dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);dwc2_current_emit(&f.h,USB_CURRENT_VENDOR,1200);
 dwc2_current_emit(&f.h,USB_CURRENT_SUSPEND,2);dwc2_current_emit(&f.h,USB_CURRENT_RESUME,500);
 flush_work(&f.h.current_work);assert(f.c.current_queue.count==5);
 e=take(&f);assert(e.reason==USB_CURRENT_RESET&&e.milliamps==100&&e.sequence==1);
 e=take(&f);assert(e.reason==USB_CURRENT_CONFIGURED&&e.milliamps==500&&e.sequence==2);
 e=take(&f);assert(e.reason==USB_CURRENT_VENDOR&&e.milliamps==1200&&e.sequence==3);
 e=take(&f);assert(e.reason==USB_CURRENT_SUSPEND&&e.milliamps==2&&e.sequence==4);
 e=take(&f);assert(e.reason==USB_CURRENT_RESUME&&e.milliamps==500&&e.sequence==5);
 dwc2_current_disconnect(&f.h);dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);
 flush_work(&f.h.current_work);e=take(&f);assert(e.reason==USB_CURRENT_DISCONNECT&&e.sequence==6);
 assert(!f.h.current_active);finish(&f);
}
static void stale_reset_and_roles(void)
{
 struct fixture f;u64 captured,old;setup(&f);begin(&f);captured=dwc2_current_pending(&f.h);old=f.c.current_session;
 mutex_lock(&f.c.lock);tristar_set_usb(&f.c,false);tristar_set_usb(&f.c,true);mutex_unlock(&f.c.lock);
 assert(f.c.current_session!=old);dwc2_current_reset(&f.h,captured);assert(!f.h.current_active);
 dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));flush_work(&f.h.current_work);
 assert(take(&f).session==f.c.current_session);
 assert(dwc2_drd_role_sw_set_current(&f.sw,USB_ROLE_DEVICE,++next_session)==-EBUSY);
 assert(!f.h.current_active);finish(&f);
 setup(&f);f.h.ll_hw_enabled=false;
 assert(dwc2_drd_role_sw_set_current(&f.sw,USB_ROLE_DEVICE,++next_session)==-EBUSY);
 assert(!f.h.writes);finish(&f);
 setup(&f);f.h.ll_hw_enabled=false;f.h.enabled=false;begin(&f);
 assert(!f.h.writes&&!f.h.current_armed);captured=dwc2_current_pending(&f.h);assert(!captured);
 dwc2_current_reset(&f.h,captured);assert(!f.h.current_active);
 f.h.ll_hw_enabled=true;dwc2_current_arm(&f.h);f.h.enabled=true;
 dwc2_current_reset(&f.h,captured);assert(!f.h.current_active);
 dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));flush_work(&f.h.current_work);
 assert(take(&f).reason==USB_CURRENT_RESET);finish(&f);
}
static void loss_and_stop(void)
{
 struct fixture f;struct usb_current_event e;unsigned int i;setup(&f);begin(&f);
 dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));
 for(i=0;i<USB_CURRENT_QUEUE_SIZE;i++)dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500+i);
 assert(f.h.current_queue.lost);flush_work(&f.h.current_work);e=take(&f);assert(e.reason==USB_CURRENT_LOST);
 assert(f.c.current_queue.lost);dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,900);assert(!f.h.current_queue.count);
 finish(&f);
 setup(&f);begin(&f);dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));flush_work(&f.h.current_work);
 for(i=0;i<USB_CURRENT_QUEUE_SIZE;i++){dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);flush_work(&f.h.current_work);}
 assert(f.c.current_queue.lost);assert(take(&f).reason==USB_CURRENT_LOST);finish(&f);
 setup(&f);begin(&f);dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));f.h.current_sequence=U64_MAX;
 dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);flush_work(&f.h.current_work);
 assert(take(&f).reason==USB_CURRENT_LOST);finish(&f);
 setup(&f);begin(&f);dwc2_current_stop(&f.h);assert(take(&f).reason==USB_CURRENT_LOST);
 assert(dwc2_current_bind(&f.h,999)==-EOPNOTSUPP);assert(!f.h.current_pending);finish(&f);
}
static void delivery_and_handoff(void)
{
 struct fixture f;struct usb_current_event e;setup(&f);inject_reset=spawn_delivery=true;begin(&f);
 assert(!pthread_join(delivery,NULL));e=take(&f);assert(e.reason==USB_CURRENT_RESET&&e.session==f.c.current_session);
 finish(&f);
 setup(&f);inject_reset=inject_source_event=true;mutex_lock(&f.c.lock);tristar_set_usb(&f.c,true);mutex_unlock(&f.c.lock);
 assert(!f.c.current_session);flush_work(&f.h.current_work);assert(f.h.current_queue.lost);finish(&f);
 setup(&f);begin(&f);dwc2_current_reset(&f.h,dwc2_current_pending(&f.h));supply=NULL;
 flush_work(&f.h.current_work);assert(f.h.current_queue.lost);supply=&f.psy;
 dwc2_current_emit(&f.h,USB_CURRENT_CONFIGURED,500);assert(!f.c.current_queue.count);finish(&f);
 setup(&f);begin(&f);e=(struct usb_current_event){.session=f.c.current_session,.sequence=2,.reason=USB_CURRENT_CONFIGURED,.milliamps=500};
 assert(!tristar_usb_current_event(&f.psy,&e));assert(f.c.current_queue.lost);finish(&f);
 setup(&f);begin(&f);tristar_current_invalidate(&f.c,true);
 e=(struct usb_current_event){.session=f.h.current_pending,.sequence=1,.reason=USB_CURRENT_RESET,.milliamps=100};
 assert(tristar_usb_current_event(&f.psy,&e)==-ESHUTDOWN);finish(&f);
}
int main(void)
{
 path_and_reasons();stale_reset_and_roles();loss_and_stop();delivery_and_handoff();
 puts("actual DWC2/Tristar current stream: reasons, pending/reset epochs, FIFO loss, lifetime and handoff passed");return 0;
}
'''
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    (tmp/'linux').mkdir()
    (tmp/'linux/types.h').write_text('#include <stdint.h>\n#include <stdbool.h>\ntypedef uint64_t u64;\n')
    source = tmp/'test.c'
    source.write_text(preamble+production+role_fake+production2+tests)
    cmd = shlex.split(os.environ.get('CC','cc'))+['-std=gnu11','-Wall','-Wextra','-Werror','-pthread']
    cmd += shlex.split(os.environ.get('CFLAGS',''))+['-I'+str(tmp),'-I'+str(ROOT/'include'),str(source),'-o',str(tmp/'test')]
    subprocess.run(cmd,check=True)
    subprocess.run([str(tmp/'test')],check=True)
