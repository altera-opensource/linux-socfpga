// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Intel Corporation
 */

/*
 * This driver exposes some optional features of the Intel Stratix 10 SoC FPGA.
 * The SysFS interfaces exposed here are FPGA Remote System Update (RSU)
 * related.  They allow user space software to query the configuration system
 * status and to request optional reboot behavior specific to Intel FPGAs.
 */

#include <linux/arm-smccc.h>
#include <linux/completion.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#define MAX_U64_STR_LEN 22

/*
 * Private data structure
 */
struct intel_rsu_priv {
	struct stratix10_svc_chan *chan;
	struct stratix10_svc_client client;
	struct completion svc_completion;
	struct {
		unsigned long current_image;
		unsigned long fail_image;
		unsigned int version;
		unsigned int state;
		unsigned int error_details;
		unsigned int error_location;
	} status;
};

/*
 * status_svc_callback() - Callback from intel-service layer that returns SMC
 *                         response with RSU status data. Parses up data and
 *                         update driver private data structure.
 * client - returned context from intel-service layer
 * data - SMC response data
 */
static void status_svc_callback(struct stratix10_svc_client *client,
				struct stratix10_svc_cb_data *data)
{
	struct intel_rsu_priv *priv = client->priv;
	struct arm_smccc_res *res = (struct arm_smccc_res *)data->kaddr1;

	if (data->status == BIT(SVC_STATUS_RSU_OK)) {
		priv->status.version =
		    (unsigned int)(res->a2 >> 32) & 0xFFFFFFFF;
		priv->status.state = (unsigned int)res->a2 & 0xFFFFFFFF;
		priv->status.fail_image = res->a1;
		priv->status.current_image = res->a0;
		priv->status.error_location =
		    (unsigned int)res->a3 & 0xFFFFFFFF;
		priv->status.error_details =
		    (unsigned int)(res->a3 >> 32) & 0xFFFFFFFF;
	} else {
		dev_err(client->dev, "COMMAND_RSU_STATUS returned 0x%lX\n",
			res->a0);
		priv->status.version = 0;
		priv->status.state = 0;
		priv->status.fail_image = 0;
		priv->status.current_image = 0;
		priv->status.error_location = 0;
		priv->status.error_details = 0;
	}

	complete(&priv->svc_completion);
}

/*
 * get_status() - Start an intel-service layer transaction to perform the SMC
 *                that is necessary to get RSU status information. Wait for
 *                completion and timeout if needed.
 * priv - driver private data
 *
 * Returns 0 on success
 */
static int get_status(struct intel_rsu_priv *priv)
{
	struct stratix10_svc_client_msg msg;
	int ret;
	unsigned long timeout;

	reinit_completion(&priv->svc_completion);
	priv->client.receive_cb = status_svc_callback;

	msg.command = COMMAND_RSU_STATUS;
	ret = stratix10_svc_send(priv->chan, &msg);
	if (ret < 0)
		goto status_done;

	timeout = msecs_to_jiffies(SVC_RSU_REQUEST_TIMEOUT_MS);
	ret =
	    wait_for_completion_interruptible_timeout(&priv->svc_completion,
						      timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for COMMAND_RSU_STATUS\n");
		ret = -ETIMEDOUT;
		goto status_done;
	}
	if (ret < 0) {
		dev_err(priv->client.dev,
			"error (%d) waiting for COMMAND_RSU_STATUS\n", ret);
		goto status_done;
	}

	ret = 0;

status_done:
	stratix10_svc_done(priv->chan);
	return ret;
}

/* current_image_show() - DEVICE_ATTR callback to show current_image status */
static ssize_t current_image_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%ld", priv->status.current_image);
}

/* fail_image_show() - DEVICE_ATTR callback to show fail_image status */
static ssize_t fail_image_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%ld", priv->status.fail_image);
}

/* version_show() - DEVICE_ATTR callback to show version status */
static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d", priv->status.version);
}

/* state_show() - DEVICE_ATTR callback to show state status */
static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d", priv->status.state);
}

/* error_location_show() - DEVICE_ATTR callback to show error_location status */
static ssize_t error_location_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d", priv->status.error_location);
}

/* error_details_show() - DEVICE_ATTR callback to show error_details status */
static ssize_t error_details_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d", priv->status.error_details);
}

/*
 * update_svc_callback() - Callback from intel-service layer that returns SMC
 *                         response from RSU update. Checks for success/fail.
 * client - returned context from intel-service layer
 * data - SMC repsonse data
 */
static void update_svc_callback(struct stratix10_svc_client *client,
				struct stratix10_svc_cb_data *data)
{
	struct intel_rsu_priv *priv = client->priv;

	if (data->status != BIT(SVC_STATUS_RSU_OK))
		dev_err(client->dev, "COMMAND_RSU_UPDATE returned %i\n",
			data->status);

	complete(&priv->svc_completion);
}

/*
 * send_update() - Start an intel-service layer transaction to perform the SMC
 *                 that is necessary to send an RSU update request. Wait for
 *                 completion and timeout if needed.
 * priv - driver private data
 *
 * Returns 0 on success
 */
static int send_update(struct intel_rsu_priv *priv,
		       unsigned long address)
{
	struct stratix10_svc_client_msg msg;
	int ret;
	unsigned long timeout;

	reinit_completion(&priv->svc_completion);
	priv->client.receive_cb = update_svc_callback;

	msg.command = COMMAND_RSU_UPDATE;
	msg.arg[0] = address;

	ret = stratix10_svc_send(priv->chan, &msg);
	if (ret < 0)
		goto update_done;

	timeout = msecs_to_jiffies(SVC_RSU_REQUEST_TIMEOUT_MS);
	ret = wait_for_completion_interruptible_timeout(&priv->svc_completion,
							timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for COMMAND_RSU_UPDATE\n");
		ret = -ETIMEDOUT;
		goto update_done;
	}
	if (ret < 0) {
		dev_err(priv->client.dev,
			"error (%d) waiting for COMMAND_RSU_UPDATE\n", ret);
		goto update_done;
	}

	ret = 0;

update_done:
	stratix10_svc_done(priv->chan);
	return ret;
}

/* reboot_image_store() - DEVICE_ATTR callback to store reboot_image request */
static ssize_t reboot_image_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);
	unsigned long address;
	int ret;

	if (priv == 0)
		return -ENODEV;

	/* Ensure the input buffer is null terminated and not too long */
	if (strnlen(buf, MAX_U64_STR_LEN) == MAX_U64_STR_LEN)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &address);
	if (ret)
		return ret;

	send_update(priv, address);

	return count;
}

/*
 * Attribute structures
 */

static DEVICE_ATTR_RO(current_image);
static DEVICE_ATTR_RO(fail_image);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(error_location);
static DEVICE_ATTR_RO(error_details);
static DEVICE_ATTR_WO(reboot_image);

static struct attribute *attrs[] = {
	&dev_attr_current_image.attr,
	&dev_attr_fail_image.attr,
	&dev_attr_state.attr,
	&dev_attr_version.attr,
	&dev_attr_error_location.attr,
	&dev_attr_error_details.attr,
	&dev_attr_reboot_image.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs
};

static int intel_rsu_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct intel_rsu_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = update_svc_callback;
	priv->client.priv = priv;

	priv->status.current_image = 0;
	priv->status.fail_image = 0;
	priv->status.error_location = 0;
	priv->status.error_details = 0;
	priv->status.version = 0;
	priv->status.state = 0;

	priv->chan = stratix10_svc_request_channel_byname(&priv->client,
							 SVC_CLIENT_RSU);
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "couldn't get service channel (%s)\n",
			SVC_CLIENT_RSU);
		return PTR_ERR(priv->chan);
	}

	init_completion(&priv->svc_completion);

	platform_set_drvdata(pdev, priv);

	ret = get_status(priv);
	if (ret) {
		dev_err(dev, "Error getting RSU status (%i)\n", ret);
		stratix10_svc_free_channel(priv->chan);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if (ret)
		stratix10_svc_free_channel(priv->chan);

	pr_info("Intel RSU Driver Initialized\n");
	return ret;
}

static int intel_rsu_remove(struct platform_device *pdev)
{
	struct intel_rsu_priv *priv = platform_get_drvdata(pdev);

	stratix10_svc_free_channel(priv->chan);

	return 0;
}

static const struct of_device_id intel_rsu_of_match[] = {
	{.compatible = "intel,stratix10-rsu",},
	{},
};
MODULE_DEVICE_TABLE(of, intel_rsu_of_match);

static struct platform_driver intel_rsu_driver = {
	.probe = intel_rsu_probe,
	.remove = intel_rsu_remove,
	.driver = {
		   .name = "intel-rsu",
		   .of_match_table = intel_rsu_of_match,
		   },
};

static int __init stratix_rsu_init(void)
{
	struct device_node *fw_np;
        struct device_node *np;
        int ret;

        fw_np = of_find_node_by_name(NULL, "svc");
        if (!fw_np)
                return -ENODEV;

        np = of_find_matching_node(fw_np, intel_rsu_of_match);
        if (!np)
                return -ENODEV;

        of_node_put(np);
        ret = of_platform_populate(fw_np, intel_rsu_of_match, NULL, NULL);
        if (ret)
                return ret;

        return platform_driver_register(&intel_rsu_driver);
}

static void __exit stratix_rsu_exit(void)
{
	return platform_driver_unregister(&intel_rsu_driver);
}

module_init(stratix_rsu_init);
module_exit(stratix_rsu_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Remote System Update SysFS Driver");
MODULE_AUTHOR("David Koltak <david.koltak@linux.intel.com>");
