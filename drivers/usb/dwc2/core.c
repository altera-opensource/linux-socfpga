// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * core.c - DesignWare HS OTG Controller common routines
 *
 * Copyright (C) 2004-2013 Synopsys, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * The Core code provides basic services for accessing and managing the
 * DWC_otg hardware. These services are used by both the Host Controller
 * Driver and the Peripheral Controller Driver.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <linux/usb/hcd.h>
#include <linux/usb/ch11.h>

#include "core.h"
#include "hcd.h"

/**
 * dwc2_backup_global_registers() - Backup global controller registers.
 * When suspending usb bus, registers needs to be backuped
 * if controller power is disabled once suspended.
 *
 * @hsotg: Programming view of the DWC_otg controller
 */
static int dwc2_backup_global_registers(struct dwc2_hsotg *hsotg)
{
	struct dwc2_gregs_backup *gr;
	int i;

	/* Backup global regs */
	gr = &hsotg->gr_backup;

	gr->gotgctl = dwc2_readl(hsotg->regs + GOTGCTL);
	gr->gintmsk = dwc2_readl(hsotg->regs + GINTMSK);
	gr->gahbcfg = dwc2_readl(hsotg->regs + GAHBCFG);
	gr->gusbcfg = dwc2_readl(hsotg->regs + GUSBCFG);
	gr->grxfsiz = dwc2_readl(hsotg->regs + GRXFSIZ);
	gr->gnptxfsiz = dwc2_readl(hsotg->regs + GNPTXFSIZ);
	gr->hptxfsiz = dwc2_readl(hsotg->regs + HPTXFSIZ);
	gr->gdfifocfg = dwc2_readl(hsotg->regs + GDFIFOCFG);
	for (i = 0; i < MAX_EPS_CHANNELS; i++)
		gr->dtxfsiz[i] = dwc2_readl(hsotg->regs + DPTXFSIZN(i));

	gr->valid = true;
	return 0;
}

/**
 * dwc2_restore_global_registers() - Restore controller global registers.
 * When resuming usb bus, device registers needs to be restored
 * if controller power were disabled.
 *
 * @hsotg: Programming view of the DWC_otg controller
 */
static int dwc2_restore_global_registers(struct dwc2_hsotg *hsotg)
{
	struct dwc2_gregs_backup *gr;
	int i;

	dev_dbg(hsotg->dev, "%s\n", __func__);

	/* Restore global regs */
	gr = &hsotg->gr_backup;
	if (!gr->valid) {
		dev_err(hsotg->dev, "%s: no global registers to restore\n",
			__func__);
		return -EINVAL;
	}
	gr->valid = false;

	dwc2_writel(0xffffffff, hsotg->regs + GINTSTS);
	dwc2_writel(gr->gotgctl, hsotg->regs + GOTGCTL);
	dwc2_writel(gr->gintmsk, hsotg->regs + GINTMSK);
	dwc2_writel(gr->gusbcfg, hsotg->regs + GUSBCFG);
	dwc2_writel(gr->gahbcfg, hsotg->regs + GAHBCFG);
	dwc2_writel(gr->grxfsiz, hsotg->regs + GRXFSIZ);
	dwc2_writel(gr->gnptxfsiz, hsotg->regs + GNPTXFSIZ);
	dwc2_writel(gr->hptxfsiz, hsotg->regs + HPTXFSIZ);
	dwc2_writel(gr->gdfifocfg, hsotg->regs + GDFIFOCFG);
	for (i = 0; i < MAX_EPS_CHANNELS; i++)
		dwc2_writel(gr->dtxfsiz[i], hsotg->regs + DPTXFSIZN(i));

	return 0;
}

/**
 * dwc2_exit_hibernation() - Exit controller from Partial Power Down.
 *
 * @hsotg: Programming view of the DWC_otg controller
 * @restore: Controller registers need to be restored
 */
int dwc2_exit_hibernation(struct dwc2_hsotg *hsotg, bool restore)
{
	u32 pcgcctl;
	int ret = 0;

	if (!hsotg->params.hibernation)
		return -ENOTSUPP;

	pcgcctl = dwc2_readl(hsotg->regs + PCGCTL);
	pcgcctl &= ~PCGCTL_STOPPCLK;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);

	pcgcctl = dwc2_readl(hsotg->regs + PCGCTL);
	pcgcctl &= ~PCGCTL_PWRCLMP;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);

	pcgcctl = dwc2_readl(hsotg->regs + PCGCTL);
	pcgcctl &= ~PCGCTL_RSTPDWNMODULE;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);

	udelay(100);
	if (restore) {
		ret = dwc2_restore_global_registers(hsotg);
		if (ret) {
			dev_err(hsotg->dev, "%s: failed to restore registers\n",
				__func__);
			return ret;
		}
		if (dwc2_is_host_mode(hsotg)) {
			ret = dwc2_restore_host_registers(hsotg);
			if (ret) {
				dev_err(hsotg->dev, "%s: failed to restore host registers\n",
					__func__);
				return ret;
			}
		} else {
			ret = dwc2_restore_device_registers(hsotg);
			if (ret) {
				dev_err(hsotg->dev, "%s: failed to restore device registers\n",
					__func__);
				return ret;
			}
		}
	}

	return ret;
}

/**
 * dwc2_enter_hibernation() - Put controller in Partial Power Down.
 *
 * @hsotg: Programming view of the DWC_otg controller
 */
int dwc2_enter_hibernation(struct dwc2_hsotg *hsotg)
{
	u32 pcgcctl;
	int ret = 0;

	if (!hsotg->params.hibernation)
		return -ENOTSUPP;

	/* Backup all registers */
	ret = dwc2_backup_global_registers(hsotg);
	if (ret) {
		dev_err(hsotg->dev, "%s: failed to backup global registers\n",
			__func__);
		return ret;
	}

	if (dwc2_is_host_mode(hsotg)) {
		ret = dwc2_backup_host_registers(hsotg);
		if (ret) {
			dev_err(hsotg->dev, "%s: failed to backup host registers\n",
				__func__);
			return ret;
		}
	} else {
		ret = dwc2_backup_device_registers(hsotg);
		if (ret) {
			dev_err(hsotg->dev, "%s: failed to backup device registers\n",
				__func__);
			return ret;
		}
	}

	/*
	 * Clear any pending interrupts since dwc2 will not be able to
	 * clear them after entering hibernation.
	 */
	dwc2_writel(0xffffffff, hsotg->regs + GINTSTS);

	/* Put the controller in low power state */
	pcgcctl = dwc2_readl(hsotg->regs + PCGCTL);

	pcgcctl |= PCGCTL_PWRCLMP;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);
	ndelay(20);

	pcgcctl |= PCGCTL_RSTPDWNMODULE;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);
	ndelay(20);

	pcgcctl |= PCGCTL_STOPPCLK;
	dwc2_writel(pcgcctl, hsotg->regs + PCGCTL);

	return ret;
}

/**
 * dwc2_wait_for_mode() - Waits for the controller mode.
 * @hsotg:	Programming view of the DWC_otg controller.
 * @host_mode:	If true, waits for host mode, otherwise device mode.
 */
static void dwc2_wait_for_mode(struct dwc2_hsotg *hsotg,
			       bool host_mode)
{
	ktime_t start;
	ktime_t end;
	unsigned int timeout = 110;

	dev_vdbg(hsotg->dev, "Waiting for %s mode\n",
		 host_mode ? "host" : "device");

	start = ktime_get();

	while (1) {
		s64 ms;

		if (dwc2_is_host_mode(hsotg) == host_mode) {
			dev_vdbg(hsotg->dev, "%s mode set\n",
				 host_mode ? "Host" : "Device");
			break;
		}

		end = ktime_get();
		ms = ktime_to_ms(ktime_sub(end, start));

		if (ms >= (s64)timeout) {
			dev_warn(hsotg->dev, "%s: Couldn't set %s mode\n",
				 __func__, host_mode ? "host" : "device");
			break;
		}

		usleep_range(1000, 2000);
	}
}

/**
 * dwc2_iddig_filter_enabled() - Returns true if the IDDIG debounce
 * filter is enabled.
 */
static bool dwc2_iddig_filter_enabled(struct dwc2_hsotg *hsotg)
{
	u32 gsnpsid;
	u32 ghwcfg4;

	if (!dwc2_hw_is_otg(hsotg))
		return false;

	/* Check if core configuration includes the IDDIG filter. */
	ghwcfg4 = dwc2_readl(hsotg->regs + GHWCFG4);
	if (!(ghwcfg4 & GHWCFG4_IDDIG_FILT_EN))
		return false;

	/*
	 * Check if the IDDIG debounce filter is bypassed. Available
	 * in core version >= 3.10a.
	 */
	gsnpsid = dwc2_readl(hsotg->regs + GSNPSID);
	if (gsnpsid >= DWC2_CORE_REV_3_10a) {
		u32 gotgctl = dwc2_readl(hsotg->regs + GOTGCTL);

		if (gotgctl & GOTGCTL_DBNCE_FLTR_BYPASS)
			return false;
	}

	return true;
}

/*
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
int dwc2_core_reset(struct dwc2_hsotg *hsotg, bool skip_wait)
{
	u32 greset;
	int count = 0;
	bool wait_for_host_mode = false;

	dev_vdbg(hsotg->dev, "%s()\n", __func__);

	/*
	 * If the current mode is host, either due to the force mode
	 * bit being set (which persists after core reset) or the
	 * connector id pin, a core soft reset will temporarily reset
	 * the mode to device. A delay from the IDDIG debounce filter
	 * will occur before going back to host mode.
	 *
	 * Determine whether we will go back into host mode after a
	 * reset and account for this delay after the reset.
	 */
	if (dwc2_iddig_filter_enabled(hsotg)) {
		u32 gotgctl = dwc2_readl(hsotg->regs + GOTGCTL);
		u32 gusbcfg = dwc2_readl(hsotg->regs + GUSBCFG);

		if (!(gotgctl & GOTGCTL_CONID_B) ||
		    (gusbcfg & GUSBCFG_FORCEHOSTMODE)) {
			wait_for_host_mode = true;
		}
	}

	/* Core Soft Reset */
	greset = dwc2_readl(hsotg->regs + GRSTCTL);
	greset |= GRSTCTL_CSFTRST;
	dwc2_writel(greset, hsotg->regs + GRSTCTL);
	do {
		udelay(1);
		greset = dwc2_readl(hsotg->regs + GRSTCTL);
		if (++count > 50) {
			dev_warn(hsotg->dev,
				 "%s() HANG! Soft Reset GRSTCTL=%0x\n",
				 __func__, greset);
			return -EBUSY;
		}
	} while (greset & GRSTCTL_CSFTRST);

	/* Wait for AHB master IDLE state */
	count = 0;
	do {
		udelay(1);
		greset = dwc2_readl(hsotg->regs + GRSTCTL);
		if (++count > 50) {
			dev_warn(hsotg->dev,
				 "%s() HANG! AHB Idle GRSTCTL=%0x\n",
				 __func__, greset);
			return -EBUSY;
		}
	} while (!(greset & GRSTCTL_AHBIDLE));

	if (wait_for_host_mode && !skip_wait)
		dwc2_wait_for_mode(hsotg, true);

	return 0;
}

/*
 * Force the mode of the controller.
 *
 * Forcing the mode is needed for two cases:
 *
 * 1) If the dr_mode is set to either HOST or PERIPHERAL we force the
 * controller to stay in a particular mode regardless of ID pin
 * changes. We do this usually after a core reset.
 *
 * 2) During probe we want to read reset values of the hw
 * configuration registers that are only available in either host or
 * device mode. We may need to force the mode if the current mode does
 * not allow us to access the register in the mode that we want.
 *
 * In either case it only makes sense to force the mode if the
 * controller hardware is OTG capable.
 *
 * Checks are done in this function to determine whether doing a force
 * would be valid or not.
 *
 * If a force is done, it requires a IDDIG debounce filter delay if
 * the filter is configured and enabled. We poll the current mode of
 * the controller to account for this delay.
 */
static bool dwc2_force_mode(struct dwc2_hsotg *hsotg, bool host)
{
	u32 gusbcfg;
	u32 set;
	u32 clear;

	dev_dbg(hsotg->dev, "Forcing mode to %s\n", host ? "host" : "device");

	/*
	 * Force mode has no effect if the hardware is not OTG.
	 */
	if (!dwc2_hw_is_otg(hsotg))
		return false;

	/*
	 * If dr_mode is either peripheral or host only, there is no
	 * need to ever force the mode to the opposite mode.
	 */
	if (WARN_ON(host && hsotg->dr_mode == USB_DR_MODE_PERIPHERAL))
		return false;

	if (WARN_ON(!host && hsotg->dr_mode == USB_DR_MODE_HOST))
		return false;

	gusbcfg = dwc2_readl(hsotg->regs + GUSBCFG);

	set = host ? GUSBCFG_FORCEHOSTMODE : GUSBCFG_FORCEDEVMODE;
	clear = host ? GUSBCFG_FORCEDEVMODE : GUSBCFG_FORCEHOSTMODE;

	gusbcfg &= ~clear;
	gusbcfg |= set;
	dwc2_writel(gusbcfg, hsotg->regs + GUSBCFG);

	dwc2_wait_for_mode(hsotg, host);
	return true;
}

/**
 * dwc2_clear_force_mode() - Clears the force mode bits.
 *
 * After clearing the bits, wait up to 100 ms to account for any
 * potential IDDIG filter delay. We can't know if we expect this delay
 * or not because the value of the connector ID status is affected by
 * the force mode. We only need to call this once during probe if
 * dr_mode == OTG.
 */
void dwc2_clear_force_mode(struct dwc2_hsotg *hsotg)
{
	u32 gusbcfg;

	gusbcfg = dwc2_readl(hsotg->regs + GUSBCFG);
	gusbcfg &= ~GUSBCFG_FORCEHOSTMODE;
	gusbcfg &= ~GUSBCFG_FORCEDEVMODE;
	dwc2_writel(gusbcfg, hsotg->regs + GUSBCFG);

	if (dwc2_iddig_filter_enabled(hsotg))
		msleep(100);
}

/*
 * Sets or clears force mode based on the dr_mode parameter.
 */
void dwc2_force_dr_mode(struct dwc2_hsotg *hsotg)
{
	bool ret;

	switch (hsotg->dr_mode) {
	case USB_DR_MODE_HOST:
		ret = dwc2_force_mode(hsotg, true);
		/*
		 * NOTE: This is required for some rockchip soc based
		 * platforms on their host-only dwc2.
		 */
		if (!ret)
			msleep(50);

		break;
	case USB_DR_MODE_PERIPHERAL:
		dwc2_force_mode(hsotg, false);
		break;
	case USB_DR_MODE_OTG:
		dwc2_clear_force_mode(hsotg);
		break;
	default:
		dev_warn(hsotg->dev, "%s() Invalid dr_mode=%d\n",
			 __func__, hsotg->dr_mode);
		break;
	}
}

/*
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 *
 * Additionally this will apply force mode as per the hsotg->dr_mode
 * parameter.
 */
int dwc2_core_reset_and_force_dr_mode(struct dwc2_hsotg *hsotg)
{
	int retval;

	retval = dwc2_core_reset(hsotg, false);
	if (retval)
		return retval;

	dwc2_force_dr_mode(hsotg);
	return 0;
}

/**
 * dwc2_dump_host_registers() - Prints the host registers
 *
 * @hsotg: Programming view of DWC_otg controller
 *
 * NOTE: This function will be removed once the peripheral controller code
 * is integrated and the driver is stable
 */
void dwc2_dump_host_registers(struct dwc2_hsotg *hsotg)
{
#ifdef DEBUG
	u32 __iomem *addr;
	int i;

	dev_dbg(hsotg->dev, "Host Global Registers\n");
	addr = hsotg->regs + HCFG;
	dev_dbg(hsotg->dev, "HCFG	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HFIR;
	dev_dbg(hsotg->dev, "HFIR	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HFNUM;
	dev_dbg(hsotg->dev, "HFNUM	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HPTXSTS;
	dev_dbg(hsotg->dev, "HPTXSTS	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HAINT;
	dev_dbg(hsotg->dev, "HAINT	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HAINTMSK;
	dev_dbg(hsotg->dev, "HAINTMSK	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	if (hsotg->params.dma_desc_enable) {
		addr = hsotg->regs + HFLBADDR;
		dev_dbg(hsotg->dev, "HFLBADDR @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
	}

	addr = hsotg->regs + HPRT0;
	dev_dbg(hsotg->dev, "HPRT0	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));

	for (i = 0; i < hsotg->params.host_channels; i++) {
		dev_dbg(hsotg->dev, "Host Channel %d Specific Registers\n", i);
		addr = hsotg->regs + HCCHAR(i);
		dev_dbg(hsotg->dev, "HCCHAR	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		addr = hsotg->regs + HCSPLT(i);
		dev_dbg(hsotg->dev, "HCSPLT	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		addr = hsotg->regs + HCINT(i);
		dev_dbg(hsotg->dev, "HCINT	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		addr = hsotg->regs + HCINTMSK(i);
		dev_dbg(hsotg->dev, "HCINTMSK	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		addr = hsotg->regs + HCTSIZ(i);
		dev_dbg(hsotg->dev, "HCTSIZ	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		addr = hsotg->regs + HCDMA(i);
		dev_dbg(hsotg->dev, "HCDMA	 @0x%08lX : 0x%08X\n",
			(unsigned long)addr, dwc2_readl(addr));
		if (hsotg->params.dma_desc_enable) {
			addr = hsotg->regs + HCDMAB(i);
			dev_dbg(hsotg->dev, "HCDMAB	 @0x%08lX : 0x%08X\n",
				(unsigned long)addr, dwc2_readl(addr));
		}
	}
#endif
}

/**
 * dwc2_dump_global_registers() - Prints the core global registers
 *
 * @hsotg: Programming view of DWC_otg controller
 *
 * NOTE: This function will be removed once the peripheral controller code
 * is integrated and the driver is stable
 */
void dwc2_dump_global_registers(struct dwc2_hsotg *hsotg)
{
#ifdef DEBUG
	u32 __iomem *addr;

	dev_dbg(hsotg->dev, "Core Global Registers\n");
	addr = hsotg->regs + GOTGCTL;
	dev_dbg(hsotg->dev, "GOTGCTL	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GOTGINT;
	dev_dbg(hsotg->dev, "GOTGINT	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GAHBCFG;
	dev_dbg(hsotg->dev, "GAHBCFG	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GUSBCFG;
	dev_dbg(hsotg->dev, "GUSBCFG	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GRSTCTL;
	dev_dbg(hsotg->dev, "GRSTCTL	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GINTSTS;
	dev_dbg(hsotg->dev, "GINTSTS	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GINTMSK;
	dev_dbg(hsotg->dev, "GINTMSK	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GRXSTSR;
	dev_dbg(hsotg->dev, "GRXSTSR	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GRXFSIZ;
	dev_dbg(hsotg->dev, "GRXFSIZ	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GNPTXFSIZ;
	dev_dbg(hsotg->dev, "GNPTXFSIZ	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GNPTXSTS;
	dev_dbg(hsotg->dev, "GNPTXSTS	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GI2CCTL;
	dev_dbg(hsotg->dev, "GI2CCTL	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GPVNDCTL;
	dev_dbg(hsotg->dev, "GPVNDCTL	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GGPIO;
	dev_dbg(hsotg->dev, "GGPIO	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GUID;
	dev_dbg(hsotg->dev, "GUID	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GSNPSID;
	dev_dbg(hsotg->dev, "GSNPSID	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GHWCFG1;
	dev_dbg(hsotg->dev, "GHWCFG1	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GHWCFG2;
	dev_dbg(hsotg->dev, "GHWCFG2	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GHWCFG3;
	dev_dbg(hsotg->dev, "GHWCFG3	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GHWCFG4;
	dev_dbg(hsotg->dev, "GHWCFG4	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GLPMCFG;
	dev_dbg(hsotg->dev, "GLPMCFG	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GPWRDN;
	dev_dbg(hsotg->dev, "GPWRDN	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + GDFIFOCFG;
	dev_dbg(hsotg->dev, "GDFIFOCFG	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
	addr = hsotg->regs + HPTXFSIZ;
	dev_dbg(hsotg->dev, "HPTXFSIZ	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));

	addr = hsotg->regs + PCGCTL;
	dev_dbg(hsotg->dev, "PCGCTL	 @0x%08lX : 0x%08X\n",
		(unsigned long)addr, dwc2_readl(addr));
#endif
}

/**
 * dwc2_flush_tx_fifo() - Flushes a Tx FIFO
 *
 * @hsotg: Programming view of DWC_otg controller
 * @num:   Tx FIFO to flush
 */
void dwc2_flush_tx_fifo(struct dwc2_hsotg *hsotg, const int num)
{
	u32 greset;
	int count = 0;

	dev_vdbg(hsotg->dev, "Flush Tx FIFO %d\n", num);

	greset = GRSTCTL_TXFFLSH;
	greset |= num << GRSTCTL_TXFNUM_SHIFT & GRSTCTL_TXFNUM_MASK;
	dwc2_writel(greset, hsotg->regs + GRSTCTL);

	do {
		greset = dwc2_readl(hsotg->regs + GRSTCTL);
		if (++count > 10000) {
			dev_warn(hsotg->dev,
				 "%s() HANG! GRSTCTL=%0x GNPTXSTS=0x%08x\n",
				 __func__, greset,
				 dwc2_readl(hsotg->regs + GNPTXSTS));
			break;
		}
		udelay(1);
	} while (greset & GRSTCTL_TXFFLSH);

	/* Wait for at least 3 PHY Clocks */
	udelay(1);
}

/**
 * dwc2_flush_rx_fifo() - Flushes the Rx FIFO
 *
 * @hsotg: Programming view of DWC_otg controller
 */
void dwc2_flush_rx_fifo(struct dwc2_hsotg *hsotg)
{
	u32 greset;
	int count = 0;

	dev_vdbg(hsotg->dev, "%s()\n", __func__);

	greset = GRSTCTL_RXFFLSH;
	dwc2_writel(greset, hsotg->regs + GRSTCTL);

	do {
		greset = dwc2_readl(hsotg->regs + GRSTCTL);
		if (++count > 10000) {
			dev_warn(hsotg->dev, "%s() HANG! GRSTCTL=%0x\n",
				 __func__, greset);
			break;
		}
		udelay(1);
	} while (greset & GRSTCTL_RXFFLSH);

	/* Wait for at least 3 PHY Clocks */
	udelay(1);
}

/*
 * Forces either host or device mode if the controller is not
 * currently in that mode.
 *
 * Returns true if the mode was forced.
 */
bool dwc2_force_mode_if_needed(struct dwc2_hsotg *hsotg, bool host)
{
	if (host && dwc2_is_host_mode(hsotg))
		return false;
	else if (!host && dwc2_is_device_mode(hsotg))
		return false;

	return dwc2_force_mode(hsotg, host);
}

bool dwc2_is_controller_alive(struct dwc2_hsotg *hsotg)
{
	if (dwc2_readl(hsotg->regs + GSNPSID) == 0xffffffff)
		return false;
	else
		return true;
}

/**
 * dwc2_enable_global_interrupts() - Enables the controller's Global
 * Interrupt in the AHB Config register
 *
 * @hsotg: Programming view of DWC_otg controller
 */
void dwc2_enable_global_interrupts(struct dwc2_hsotg *hsotg)
{
	u32 ahbcfg = dwc2_readl(hsotg->regs + GAHBCFG);

	ahbcfg |= GAHBCFG_GLBL_INTR_EN;
	dwc2_writel(ahbcfg, hsotg->regs + GAHBCFG);
}

/**
 * dwc2_disable_global_interrupts() - Disables the controller's Global
 * Interrupt in the AHB Config register
 *
 * @hsotg: Programming view of DWC_otg controller
 */
void dwc2_disable_global_interrupts(struct dwc2_hsotg *hsotg)
{
	u32 ahbcfg = dwc2_readl(hsotg->regs + GAHBCFG);

	ahbcfg &= ~GAHBCFG_GLBL_INTR_EN;
	dwc2_writel(ahbcfg, hsotg->regs + GAHBCFG);
}

/* Returns the controller's GHWCFG2.OTG_MODE. */
unsigned int dwc2_op_mode(struct dwc2_hsotg *hsotg)
{
	u32 ghwcfg2 = dwc2_readl(hsotg->regs + GHWCFG2);

	return (ghwcfg2 & GHWCFG2_OP_MODE_MASK) >>
		GHWCFG2_OP_MODE_SHIFT;
}

/* Returns true if the controller is capable of DRD. */
bool dwc2_hw_is_otg(struct dwc2_hsotg *hsotg)
{
	unsigned int op_mode = dwc2_op_mode(hsotg);

	return (op_mode == GHWCFG2_OP_MODE_HNP_SRP_CAPABLE) ||
		(op_mode == GHWCFG2_OP_MODE_SRP_ONLY_CAPABLE) ||
		(op_mode == GHWCFG2_OP_MODE_NO_HNP_SRP_CAPABLE);
}

/* Returns true if the controller is host-only. */
bool dwc2_hw_is_host(struct dwc2_hsotg *hsotg)
{
	unsigned int op_mode = dwc2_op_mode(hsotg);

	return (op_mode == GHWCFG2_OP_MODE_SRP_CAPABLE_HOST) ||
		(op_mode == GHWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST);
}

/* Returns true if the controller is device-only. */
bool dwc2_hw_is_device(struct dwc2_hsotg *hsotg)
{
	unsigned int op_mode = dwc2_op_mode(hsotg);

	return (op_mode == GHWCFG2_OP_MODE_SRP_CAPABLE_DEVICE) ||
		(op_mode == GHWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE);
}

MODULE_DESCRIPTION("DESIGNWARE HS OTG Core");
MODULE_AUTHOR("Synopsys, Inc.");
MODULE_LICENSE("Dual BSD/GPL");
