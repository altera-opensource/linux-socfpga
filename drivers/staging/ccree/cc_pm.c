// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2012-2018 ARM Limited or its affiliates. */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include "cc_driver.h"
#include "cc_buffer_mgr.h"
#include "cc_request_mgr.h"
#include "cc_sram_mgr.h"
#include "cc_ivgen.h"
#include "cc_hash.h"
#include "cc_pm.h"

#define POWER_DOWN_ENABLE 0x01
#define POWER_DOWN_DISABLE 0x00

const struct dev_pm_ops ccree_pm = {
	SET_RUNTIME_PM_OPS(cc_pm_suspend, cc_pm_resume, NULL)
};

int cc_pm_suspend(struct device *dev)
{
	struct cc_drvdata *drvdata = dev_get_drvdata(dev);
	int rc;

	dev_dbg(dev, "set HOST_POWER_DOWN_EN\n");
	cc_iowrite(drvdata, CC_REG(HOST_POWER_DOWN_EN), POWER_DOWN_ENABLE);
	rc = cc_suspend_req_queue(drvdata);
	if (rc) {
		dev_err(dev, "cc_suspend_req_queue (%x)\n", rc);
		return rc;
	}
	fini_cc_regs(drvdata);
	cc_clk_off(drvdata);
	return 0;
}

int cc_pm_resume(struct device *dev)
{
	int rc;
	struct cc_drvdata *drvdata = dev_get_drvdata(dev);

	dev_dbg(dev, "unset HOST_POWER_DOWN_EN\n");
	cc_iowrite(drvdata, CC_REG(HOST_POWER_DOWN_EN), POWER_DOWN_DISABLE);

	rc = cc_clk_on(drvdata);
	if (rc) {
		dev_err(dev, "failed getting clock back on. We're toast.\n");
		return rc;
	}

	rc = init_cc_regs(drvdata, false);
	if (rc) {
		dev_err(dev, "init_cc_regs (%x)\n", rc);
		return rc;
	}

	rc = cc_resume_req_queue(drvdata);
	if (rc) {
		dev_err(dev, "cc_resume_req_queue (%x)\n", rc);
		return rc;
	}

	/* must be after the queue resuming as it uses the HW queue*/
	cc_init_hash_sram(drvdata);

	cc_init_iv_sram(drvdata);
	return 0;
}

int cc_pm_get(struct device *dev)
{
	int rc = 0;
	struct cc_drvdata *drvdata = dev_get_drvdata(dev);

	if (cc_req_queue_suspended(drvdata))
		rc = pm_runtime_get_sync(dev);
	else
		pm_runtime_get_noresume(dev);

	return rc;
}

int cc_pm_put_suspend(struct device *dev)
{
	int rc = 0;
	struct cc_drvdata *drvdata = dev_get_drvdata(dev);

	if (!cc_req_queue_suspended(drvdata)) {
		pm_runtime_mark_last_busy(dev);
		rc = pm_runtime_put_autosuspend(dev);
	} else {
		/* Something wrong happens*/
		dev_err(dev, "request to suspend already suspended queue");
		rc = -EBUSY;
	}
	return rc;
}

int cc_pm_init(struct cc_drvdata *drvdata)
{
	int rc = 0;
	struct device *dev = drvdata_to_dev(drvdata);

	/* must be before the enabling to avoid resdundent suspending */
	pm_runtime_set_autosuspend_delay(dev, CC_SUSPEND_TIMEOUT);
	pm_runtime_use_autosuspend(dev);
	/* activate the PM module */
	rc = pm_runtime_set_active(dev);
	if (rc)
		return rc;
	/* enable the PM module*/
	pm_runtime_enable(dev);

	return rc;
}

void cc_pm_fini(struct cc_drvdata *drvdata)
{
	pm_runtime_disable(drvdata_to_dev(drvdata));
}
