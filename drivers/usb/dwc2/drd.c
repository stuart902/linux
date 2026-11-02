// SPDX-License-Identifier: GPL-2.0
/*
 * drd.c - DesignWare USB2 DRD Controller Dual-role support
 *
 * Copyright (C) 2020 STMicroelectronics
 *
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com>
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/usb/role.h>
#include "core.h"

/*
 * A peripheral-only controller whose role switch has no default mode stays
 * disconnected until a connector driver sets a role. If none has within this
 * time (missing, deferred or stuck), connect as a device anyway so that the
 * gadget stays reachable.
 */
#define DWC2_ROLE_SW_FALLBACK_MS	1000

#define dwc2_ovr_gotgctl(gotgctl) \
	((gotgctl) |= GOTGCTL_BVALOEN | GOTGCTL_AVALOEN | GOTGCTL_VBVALOEN | \
	 GOTGCTL_DBNCE_FLTR_BYPASS)

static void dwc2_ovr_init(struct dwc2_hsotg *hsotg)
{
	unsigned long flags;
	u32 gotgctl;

	spin_lock_irqsave(&hsotg->lock, flags);

	gotgctl = dwc2_readl(hsotg, GOTGCTL);
	dwc2_ovr_gotgctl(gotgctl);
	gotgctl &= ~(GOTGCTL_BVALOVAL | GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL);
	if (hsotg->role_sw_default_mode == USB_DR_MODE_HOST)
		gotgctl |= GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL;
	else if (hsotg->role_sw_default_mode == USB_DR_MODE_PERIPHERAL)
		gotgctl |= GOTGCTL_BVALOVAL | GOTGCTL_VBVALOVAL;
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	spin_unlock_irqrestore(&hsotg->lock, flags);

	dwc2_force_mode(hsotg, (hsotg->dr_mode == USB_DR_MODE_HOST) ||
				(hsotg->role_sw_default_mode == USB_DR_MODE_HOST));
}

static int dwc2_ovr_avalid(struct dwc2_hsotg *hsotg, bool valid)
{
	u32 gotgctl = dwc2_readl(hsotg, GOTGCTL);

	/* Check if A-Session is already in the right state */
	if ((valid && (gotgctl & GOTGCTL_ASESVLD)) ||
	    (!valid && !(gotgctl & GOTGCTL_ASESVLD)))
		return -EALREADY;

	/* Always enable overrides to handle the resume case */
	dwc2_ovr_gotgctl(gotgctl);

	gotgctl &= ~GOTGCTL_BVALOVAL;
	if (valid)
		gotgctl |= GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL;
	else
		gotgctl &= ~(GOTGCTL_AVALOVAL | GOTGCTL_VBVALOVAL);
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	return 0;
}

static int dwc2_ovr_bvalid(struct dwc2_hsotg *hsotg, bool valid)
{
	u32 gotgctl = dwc2_readl(hsotg, GOTGCTL);

	/* Check if B-Session is already in the right state */
	if ((valid && (gotgctl & GOTGCTL_BSESVLD)) ||
	    (!valid && !(gotgctl & GOTGCTL_BSESVLD)))
		return -EALREADY;

	/* Always enable overrides to handle the resume case */
	dwc2_ovr_gotgctl(gotgctl);

	gotgctl &= ~GOTGCTL_AVALOVAL;
	if (valid)
		gotgctl |= GOTGCTL_BVALOVAL | GOTGCTL_VBVALOVAL;
	else
		gotgctl &= ~(GOTGCTL_BVALOVAL | GOTGCTL_VBVALOVAL);
	dwc2_writel(hsotg, gotgctl, GOTGCTL);

	return 0;
}

/*
 * A peripheral-only controller without a default mode: no session override.
 * Forcing B-valid low on the Apple A10's controller left an interrupt
 * pending that nothing cleared, and CPU 0 never left its handler. A role is
 * the pull-up instead: held off (DCTL.SFTDISCON) until the device role.
 */
static bool dwc2_drd_soft_role(struct dwc2_hsotg *hsotg)
{
	return hsotg->dr_mode == USB_DR_MODE_PERIPHERAL &&
	       hsotg->role_sw_default_mode == USB_DR_MODE_UNKNOWN;
}

static int dwc2_drd_set_soft_role(struct dwc2_hsotg *hsotg, enum usb_role role)
{
	unsigned long flags;

	if (role == USB_ROLE_HOST)
		return -EINVAL;

	spin_lock_irqsave(&hsotg->lock, flags);
	hsotg->role_sw_hold = role != USB_ROLE_DEVICE;
	if (dwc2_is_device_enabled(hsotg)) {
		if (hsotg->role_sw_hold)
			dwc2_hsotg_core_disconnect(hsotg);
		else
			dwc2_hsotg_core_connect(hsotg);
	}
	spin_unlock_irqrestore(&hsotg->lock, flags);

	dev_dbg(hsotg->dev, "pull-up %s\n", hsotg->role_sw_hold ? "held off" : "released");
	return 0;
}

static int dwc2_drd_set_role(struct dwc2_hsotg *hsotg, enum usb_role role)
{
	unsigned long flags;
	int already = 0;

	if (dwc2_drd_soft_role(hsotg))
		return dwc2_drd_set_soft_role(hsotg, role);

	/* Skip session not in line with dr_mode */
	if ((role == USB_ROLE_DEVICE && hsotg->dr_mode == USB_DR_MODE_HOST) ||
	    (role == USB_ROLE_HOST && hsotg->dr_mode == USB_DR_MODE_PERIPHERAL))
		return -EINVAL;

#if IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) || \
	IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)
	/* Skip session if core is in test mode */
	if (role == USB_ROLE_NONE && hsotg->test_mode) {
		dev_dbg(hsotg->dev, "Core is in test mode\n");
		return -EBUSY;
	}
#endif

	/*
	 * In case of USB_DR_MODE_PERIPHERAL, clock is disabled at the end of
	 * the probe and enabled on udc_start.
	 * If role-switch set is called before the udc_start, we need to enable
	 * the clock to read/write GOTGCTL and GUSBCFG registers to override
	 * mode and sessions. It is the case if cable is plugged at boot.
	 */
	if (!hsotg->ll_hw_enabled && hsotg->clk) {
		int ret = clk_prepare_enable(hsotg->clk);

		if (ret)
			return ret;
	}

	spin_lock_irqsave(&hsotg->lock, flags);

	if (role == USB_ROLE_NONE) {
		/* default operation mode when usb role is USB_ROLE_NONE */
		if (hsotg->role_sw_default_mode == USB_DR_MODE_HOST)
			role = USB_ROLE_HOST;
		else if (hsotg->role_sw_default_mode == USB_DR_MODE_PERIPHERAL)
			role = USB_ROLE_DEVICE;
	}

	if ((IS_ENABLED(CONFIG_USB_DWC2_PERIPHERAL) ||
	     IS_ENABLED(CONFIG_USB_DWC2_DUAL_ROLE)) &&
	     dwc2_is_device_mode(hsotg) &&
	     hsotg->lx_state == DWC2_L2 &&
	     hsotg->params.power_down == DWC2_POWER_DOWN_PARAM_NONE &&
	     hsotg->bus_suspended &&
	     !hsotg->params.no_clock_gating)
		dwc2_gadget_exit_clock_gating(hsotg, 0);

	if (role == USB_ROLE_HOST) {
		already = dwc2_ovr_avalid(hsotg, true);
	} else if (role == USB_ROLE_DEVICE) {
		already = dwc2_ovr_bvalid(hsotg, true);
		if (dwc2_is_device_enabled(hsotg)) {
			/* This clear DCTL.SFTDISCON bit */
			dwc2_hsotg_core_connect(hsotg);
		}
	} else {
		if (dwc2_is_device_mode(hsotg)) {
			if (!dwc2_ovr_bvalid(hsotg, false))
				/* This set DCTL.SFTDISCON bit */
				dwc2_hsotg_core_disconnect(hsotg);
		} else {
			dwc2_ovr_avalid(hsotg, false);
		}
	}

	spin_unlock_irqrestore(&hsotg->lock, flags);

	if (!already && hsotg->dr_mode == USB_DR_MODE_OTG)
		/* This will raise a Connector ID Status Change Interrupt */
		dwc2_force_mode(hsotg, role == USB_ROLE_HOST);

	if (!hsotg->ll_hw_enabled && hsotg->clk)
		clk_disable_unprepare(hsotg->clk);

	dev_dbg(hsotg->dev, "%s-session valid\n",
		role == USB_ROLE_NONE ? "No" :
		role == USB_ROLE_HOST ? "A" : "B");

	return 0;
}

static int dwc2_drd_role_sw_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct dwc2_hsotg *hsotg = usb_role_switch_get_drvdata(sw);
	int ret;

	mutex_lock(&hsotg->role_sw_lock);
	hsotg->role_sw_driven = true;
	ret = dwc2_drd_set_role(hsotg, role);
	mutex_unlock(&hsotg->role_sw_lock);
	return ret;
}

static void dwc2_drd_role_sw_fallback(struct work_struct *work)
{
	struct dwc2_hsotg *hsotg = container_of(to_delayed_work(work),
						struct dwc2_hsotg,
						role_sw_fallback);

	mutex_lock(&hsotg->role_sw_lock);
	if (!hsotg->role_sw_driven) {
		dev_warn(hsotg->dev, "no role set within %u ms, connecting as a device\n",
			 DWC2_ROLE_SW_FALLBACK_MS);
		dwc2_drd_set_role(hsotg, USB_ROLE_DEVICE);
	}
	mutex_unlock(&hsotg->role_sw_lock);
}

int dwc2_drd_init(struct dwc2_hsotg *hsotg)
{
	struct usb_role_switch_desc role_sw_desc = {0};
	struct usb_role_switch *role_sw;
	int ret;

	if (!device_property_read_bool(hsotg->dev, "usb-role-switch"))
		return 0;

	hsotg->role_sw_default_mode = usb_get_role_switch_default_mode(hsotg->dev);
	mutex_init(&hsotg->role_sw_lock);
	INIT_DELAYED_WORK(&hsotg->role_sw_fallback, dwc2_drd_role_sw_fallback);
	role_sw_desc.driver_data = hsotg;
	role_sw_desc.fwnode = dev_fwnode(hsotg->dev);
	role_sw_desc.set = dwc2_drd_role_sw_set;
	role_sw_desc.allow_userspace_control = true;

	role_sw = usb_role_switch_register(hsotg->dev, &role_sw_desc);
	if (IS_ERR(role_sw)) {
		ret = PTR_ERR(role_sw);
		dev_err(hsotg->dev,
			"failed to register role switch: %d\n", ret);
		return ret;
	}

	hsotg->role_sw = role_sw;

	if (dwc2_drd_soft_role(hsotg)) {
		/* No session yet: the pull-up stays off until the device role */
		hsotg->role_sw_hold = true;
	} else {
		/* Enable override and initialize values */
		dwc2_ovr_init(hsotg);
	}

	if (hsotg->dr_mode == USB_DR_MODE_PERIPHERAL &&
	    hsotg->role_sw_default_mode == USB_DR_MODE_UNKNOWN)
		schedule_delayed_work(&hsotg->role_sw_fallback,
				      msecs_to_jiffies(DWC2_ROLE_SW_FALLBACK_MS));

	return 0;
}

void dwc2_drd_suspend(struct dwc2_hsotg *hsotg)
{
	u32 gintsts, gintmsk;

	if (hsotg->role_sw && !hsotg->params.external_id_pin_ctl) {
		gintmsk = dwc2_readl(hsotg, GINTMSK);
		gintmsk &= ~GINTSTS_CONIDSTSCHNG;
		dwc2_writel(hsotg, gintmsk, GINTMSK);
		gintsts = dwc2_readl(hsotg, GINTSTS);
		dwc2_writel(hsotg, gintsts | GINTSTS_CONIDSTSCHNG, GINTSTS);
	}
}

void dwc2_drd_resume(struct dwc2_hsotg *hsotg)
{
	u32 gintsts, gintmsk;
	enum usb_role role;

	if (hsotg->role_sw) {
		/* get last known role (as the get ops isn't implemented by this driver) */
		role = usb_role_switch_get_role(hsotg->role_sw);

		if (role == USB_ROLE_NONE) {
			if (hsotg->role_sw_default_mode == USB_DR_MODE_HOST)
				role = USB_ROLE_HOST;
			else if (hsotg->role_sw_default_mode == USB_DR_MODE_PERIPHERAL)
				role = USB_ROLE_DEVICE;
		}

		/* restore last role that may have been lost */
		if (role == USB_ROLE_HOST)
			dwc2_ovr_avalid(hsotg, true);
		else if (role == USB_ROLE_DEVICE)
			dwc2_ovr_bvalid(hsotg, true);

		dwc2_force_mode(hsotg, role == USB_ROLE_HOST);

		dev_dbg(hsotg->dev, "resuming %s-session valid\n",
			role == USB_ROLE_NONE ? "No" :
			role == USB_ROLE_HOST ? "A" : "B");
	}

	if (hsotg->role_sw && !hsotg->params.external_id_pin_ctl) {
		gintsts = dwc2_readl(hsotg, GINTSTS);
		dwc2_writel(hsotg, gintsts | GINTSTS_CONIDSTSCHNG, GINTSTS);
		gintmsk = dwc2_readl(hsotg, GINTMSK);
		gintmsk |= GINTSTS_CONIDSTSCHNG;
		dwc2_writel(hsotg, gintmsk, GINTMSK);
	}
}

void dwc2_drd_exit(struct dwc2_hsotg *hsotg)
{
	if (hsotg->role_sw) {
		cancel_delayed_work_sync(&hsotg->role_sw_fallback);
		usb_role_switch_unregister(hsotg->role_sw);
	}
}
