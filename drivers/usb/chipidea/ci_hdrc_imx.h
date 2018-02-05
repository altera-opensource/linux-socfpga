// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 */

#ifndef __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H
#define __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H

struct imx_usbmisc_data {
	struct device *dev;
	int index;

	unsigned int disable_oc:1; /* over current detect disabled */
	unsigned int oc_polarity:1; /* over current polarity if oc enabled */
	unsigned int evdo:1; /* set external vbus divider option */
	unsigned int ulpi:1; /* connected to an ULPI phy */
};

int imx_usbmisc_init(struct imx_usbmisc_data *);
int imx_usbmisc_init_post(struct imx_usbmisc_data *);
int imx_usbmisc_set_wakeup(struct imx_usbmisc_data *, bool);

#endif /* __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H */
