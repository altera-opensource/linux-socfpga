/*
 * Copyright (c) 2015-2016, Mellanox Technologies. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __MLX5_EN_STATS_H__
#define __MLX5_EN_STATS_H__

#define MLX5E_READ_CTR64_CPU(ptr, dsc, i) \
	(*(u64 *)((char *)ptr + dsc[i].offset))
#define MLX5E_READ_CTR64_BE(ptr, dsc, i) \
	be64_to_cpu(*(__be64 *)((char *)ptr + dsc[i].offset))
#define MLX5E_READ_CTR32_CPU(ptr, dsc, i) \
	(*(u32 *)((char *)ptr + dsc[i].offset))
#define MLX5E_READ_CTR32_BE(ptr, dsc, i) \
	be32_to_cpu(*(__be32 *)((char *)ptr + dsc[i].offset))

#define MLX5E_DECLARE_STAT(type, fld) #fld, offsetof(type, fld)
#define MLX5E_DECLARE_RX_STAT(type, fld) "rx%d_"#fld, offsetof(type, fld)
#define MLX5E_DECLARE_TX_STAT(type, fld) "tx%d_"#fld, offsetof(type, fld)
#define MLX5E_DECLARE_CH_STAT(type, fld) "ch%d_"#fld, offsetof(type, fld)

struct counter_desc {
	char		format[ETH_GSTRING_LEN];
	size_t		offset; /* Byte offset */
};

struct mlx5e_sw_stats {
	u64 rx_packets;
	u64 rx_bytes;
	u64 tx_packets;
	u64 tx_bytes;
	u64 tx_tso_packets;
	u64 tx_tso_bytes;
	u64 tx_tso_inner_packets;
	u64 tx_tso_inner_bytes;
	u64 tx_added_vlan_packets;
	u64 rx_lro_packets;
	u64 rx_lro_bytes;
	u64 rx_removed_vlan_packets;
	u64 rx_csum_unnecessary;
	u64 rx_csum_none;
	u64 rx_csum_complete;
	u64 rx_csum_unnecessary_inner;
	u64 rx_xdp_drop;
	u64 rx_xdp_tx;
	u64 rx_xdp_tx_full;
	u64 tx_csum_none;
	u64 tx_csum_partial;
	u64 tx_csum_partial_inner;
	u64 tx_queue_stopped;
	u64 tx_queue_wake;
	u64 tx_queue_dropped;
	u64 tx_xmit_more;
	u64 rx_wqe_err;
	u64 rx_mpwqe_filler;
	u64 rx_buff_alloc_err;
	u64 rx_cqe_compress_blks;
	u64 rx_cqe_compress_pkts;
	u64 rx_page_reuse;
	u64 rx_cache_reuse;
	u64 rx_cache_full;
	u64 rx_cache_empty;
	u64 rx_cache_busy;
	u64 rx_cache_waive;
	u64 ch_eq_rearm;

	/* Special handling counters */
	u64 link_down_events_phy;
};

struct mlx5e_qcounter_stats {
	u32 rx_out_of_buffer;
};

#define VPORT_COUNTER_GET(vstats, c) MLX5_GET64(query_vport_counter_out, \
						vstats->query_vport_out, c)

struct mlx5e_vport_stats {
	__be64 query_vport_out[MLX5_ST_SZ_QW(query_vport_counter_out)];
};

#define PPORT_802_3_GET(pstats, c) \
	MLX5_GET64(ppcnt_reg, pstats->IEEE_802_3_counters, \
		   counter_set.eth_802_3_cntrs_grp_data_layout.c##_high)
#define PPORT_2863_GET(pstats, c) \
	MLX5_GET64(ppcnt_reg, pstats->RFC_2863_counters, \
		   counter_set.eth_2863_cntrs_grp_data_layout.c##_high)
#define PPORT_2819_GET(pstats, c) \
	MLX5_GET64(ppcnt_reg, pstats->RFC_2819_counters, \
		   counter_set.eth_2819_cntrs_grp_data_layout.c##_high)
#define PPORT_PHY_STATISTICAL_GET(pstats, c) \
	MLX5_GET64(ppcnt_reg, (pstats)->phy_statistical_counters, \
		   counter_set.phys_layer_statistical_cntrs.c##_high)
#define PPORT_PER_PRIO_GET(pstats, prio, c) \
	MLX5_GET64(ppcnt_reg, pstats->per_prio_counters[prio], \
		   counter_set.eth_per_prio_grp_data_layout.c##_high)
#define NUM_PPORT_PRIO				8
#define PPORT_ETH_EXT_GET(pstats, c) \
	MLX5_GET64(ppcnt_reg, (pstats)->eth_ext_counters, \
		   counter_set.eth_extended_cntrs_grp_data_layout.c##_high)

struct mlx5e_pport_stats {
	__be64 IEEE_802_3_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 RFC_2863_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 RFC_2819_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 per_prio_counters[NUM_PPORT_PRIO][MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 phy_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 phy_statistical_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
	__be64 eth_ext_counters[MLX5_ST_SZ_QW(ppcnt_reg)];
};

#define PCIE_PERF_GET(pcie_stats, c) \
	MLX5_GET(mpcnt_reg, (pcie_stats)->pcie_perf_counters, \
		 counter_set.pcie_perf_cntrs_grp_data_layout.c)

#define PCIE_PERF_GET64(pcie_stats, c) \
	MLX5_GET64(mpcnt_reg, (pcie_stats)->pcie_perf_counters, \
		   counter_set.pcie_perf_cntrs_grp_data_layout.c##_high)

struct mlx5e_pcie_stats {
	__be64 pcie_perf_counters[MLX5_ST_SZ_QW(mpcnt_reg)];
};

struct mlx5e_rq_stats {
	u64 packets;
	u64 bytes;
	u64 csum_complete;
	u64 csum_unnecessary;
	u64 csum_unnecessary_inner;
	u64 csum_none;
	u64 lro_packets;
	u64 lro_bytes;
	u64 removed_vlan_packets;
	u64 xdp_drop;
	u64 xdp_tx;
	u64 xdp_tx_full;
	u64 wqe_err;
	u64 mpwqe_filler;
	u64 buff_alloc_err;
	u64 cqe_compress_blks;
	u64 cqe_compress_pkts;
	u64 page_reuse;
	u64 cache_reuse;
	u64 cache_full;
	u64 cache_empty;
	u64 cache_busy;
	u64 cache_waive;
};

struct mlx5e_sq_stats {
	/* commonly accessed in data path */
	u64 packets;
	u64 bytes;
	u64 xmit_more;
	u64 tso_packets;
	u64 tso_bytes;
	u64 tso_inner_packets;
	u64 tso_inner_bytes;
	u64 csum_partial;
	u64 csum_partial_inner;
	u64 added_vlan_packets;
	u64 nop;
	/* less likely accessed in data path */
	u64 csum_none;
	u64 stopped;
	u64 wake;
	u64 dropped;
};

struct mlx5e_ch_stats {
	u64 eq_rearm;
};

struct mlx5e_stats {
	struct mlx5e_sw_stats sw;
	struct mlx5e_qcounter_stats qcnt;
	struct mlx5e_vport_stats vport;
	struct mlx5e_pport_stats pport;
	struct rtnl_link_stats64 vf_vport;
	struct mlx5e_pcie_stats pcie;
};

enum {
	MLX5E_NDO_UPDATE_STATS = BIT(0x1),
};

struct mlx5e_priv;
struct mlx5e_stats_grp {
	u16 update_stats_mask;
	int (*get_num_stats)(struct mlx5e_priv *priv);
	int (*fill_strings)(struct mlx5e_priv *priv, u8 *data, int idx);
	int (*fill_stats)(struct mlx5e_priv *priv, u64 *data, int idx);
	void (*update_stats)(struct mlx5e_priv *priv);
};

extern const struct mlx5e_stats_grp mlx5e_stats_grps[];
extern const int mlx5e_num_stats_grps;

#endif /* __MLX5_EN_STATS_H__ */
