/* SPDX-License-Identifier: GPL-2.0 */
/*
 *    Copyright IBM Corp. 2013
 *    Author(s): Eugene Crosser <eugene.crosser@ru.ibm.com>
 */

#ifndef __QETH_L2_H__
#define __QETH_L2_H__

#include "qeth_core.h"

extern const struct attribute_group *qeth_l2_attr_groups[];

int qeth_l2_create_device_attributes(struct device *);
void qeth_l2_remove_device_attributes(struct device *);
void qeth_l2_setup_bridgeport_attrs(struct qeth_card *card);

int qeth_l2_vnicc_set_state(struct qeth_card *card, u32 vnicc, bool state);
int qeth_l2_vnicc_get_state(struct qeth_card *card, u32 vnicc, bool *state);
int qeth_l2_vnicc_set_timeout(struct qeth_card *card, u32 timeout);
int qeth_l2_vnicc_get_timeout(struct qeth_card *card, u32 *timeout);
bool qeth_l2_vnicc_is_in_use(struct qeth_card *card);

struct qeth_mac {
	u8 mac_addr[ETH_ALEN];
	u8 disp_flag:2;
	struct hlist_node hnode;
};

#endif /* __QETH_L2_H__ */
