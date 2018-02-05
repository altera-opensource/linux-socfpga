/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _NETNS_NFTABLES_H_
#define _NETNS_NFTABLES_H_

#include <linux/list.h>

struct nft_af_info;

struct netns_nftables {
	struct list_head	tables;
	struct list_head	commit_list;
	unsigned int		base_seq;
	u8			gencursor;
};

#endif
