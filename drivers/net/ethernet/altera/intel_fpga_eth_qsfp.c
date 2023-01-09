// SPDX-License-Identifier: GPL

/* Intel(R) QSFP Memory controller poller api
 *
 * Copyright (C) 2023 Intel Corporation. All rights reserved.
 *
 * Contributors:
 *	Preetam Narayan
 */

#include "intel_fpga_eth_main.h"
#include <linux/platform_device.h>
#include <linux/of_platform.h>


static struct platform_device *
qsfp_lane_mapper(intel_fpga_xtile_eth_private *priv) {

	struct platform_device *pdev;
	struct platform_device *pdev_qsfp;
	struct device_node *dev_qsfp;

	pdev = container_of(priv->device, struct platform_device, dev);

        /* QSFP has four lanes, each lane is considered one port and
         * corresponds to linux network interface
         */
        if (priv->qsfp_lane >= 0 && priv->qsfp_lane <=3)
                dev_qsfp  = of_parse_phandle(pdev->dev.of_node, "qsfp0", 0);
        else
                dev_qsfp  = of_parse_phandle(pdev->dev.of_node, "qsfp1", 0);

        pdev_qsfp = of_find_device_by_node(dev_qsfp);

	return pdev_qsfp;
}

static void
qsfp_addr_space_mapper(intel_fpga_xtile_eth_private *priv,
		       struct platform_device *pdev_qsfp) {

	struct resource *qsfp_ctrl_res;

        qsfp_ctrl_res =
                platform_get_resource_byname(pdev_qsfp, 
					     IORESOURCE_MEM, 
					     "qsfp-mem-ctrl");

        priv->qsfp_reg = devm_ioremap(&pdev_qsfp->dev, 
				      qsfp_ctrl_res->start,
                                      resource_size(qsfp_ctrl_res));
}
 
void qsfp_link_status(struct work_struct *work) {

	bool is_link_stable;
        struct delayed_work *dwork;
	bool is_next_iter_fine = false;
        intel_fpga_xtile_eth_private *priv;
	static bool is_prev_iter_fine = true;

        dwork = to_delayed_work(work);
        priv = container_of(dwork, intel_fpga_xtile_eth_private, dwork);

        priv->cable_unplugged = 
		priv->qsfp_reg->status_reg.qsfp_stat_bits.mod_presence;

        is_link_stable = 
		hssi_ethport_is_stable(priv->pdev_hssi, priv->chan, false);

	is_next_iter_fine = 
		(is_link_stable == true) && (priv->cable_unplugged == false);

	/* If the link is not stable to perform the networking function then 
	 * necessary defence need to be taken
	 */
	if (is_prev_iter_fine == is_next_iter_fine)
	        /* This is for the 1st iteration where it is assumed link is up
		 * because on ndo_open system has been prior initalized 
		 */	
		goto reshed;

	else if (is_next_iter_fine == true) {
		if ( priv->qsfp_ops->qsfp_link_up )
			priv->qsfp_ops->qsfp_link_up(priv);
        }
        else {
		if ( priv->qsfp_ops->qsfp_link_down )
			priv->qsfp_ops->qsfp_link_down(priv);
        }
	
	is_prev_iter_fine = is_next_iter_fine;

reshed:
        schedule_delayed_work(&priv->dwork, msecs_to_jiffies(QSFP_POLL_TIMEOUT));
}


static void 
init_worker_thread(intel_fpga_xtile_eth_private *priv) {

	INIT_DELAYED_WORK(&priv->dwork, qsfp_link_status);
	qsfp_link_status(&priv->dwork.work);
}

void init_qsfp_ctrl_space(intel_fpga_xtile_eth_private *priv) {

	struct platform_device *pdev_qsfp;

	pdev_qsfp = qsfp_lane_mapper(priv);
	qsfp_addr_space_mapper(priv, pdev_qsfp);

	init_worker_thread(priv);
}

static void 
release_worker_thread(intel_fpga_xtile_eth_private *priv) {
	cancel_delayed_work_sync(&priv->dwork);
}

void deinit_qsfp_ctrl_space(intel_fpga_xtile_eth_private *priv) {
	release_worker_thread(priv);
}
