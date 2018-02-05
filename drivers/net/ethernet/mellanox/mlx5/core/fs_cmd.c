/*
 * Copyright (c) 2015, Mellanox Technologies. All rights reserved.
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

#include <linux/mlx5/driver.h>
#include <linux/mlx5/device.h>
#include <linux/mlx5/mlx5_ifc.h>

#include "fs_core.h"
#include "fs_cmd.h"
#include "mlx5_core.h"
#include "eswitch.h"

int mlx5_cmd_update_root_ft(struct mlx5_core_dev *dev,
			    struct mlx5_flow_table *ft, u32 underlay_qpn,
			    bool disconnect)
{
	u32 in[MLX5_ST_SZ_DW(set_flow_table_root_in)]   = {0};
	u32 out[MLX5_ST_SZ_DW(set_flow_table_root_out)] = {0};

	if ((MLX5_CAP_GEN(dev, port_type) == MLX5_CAP_PORT_TYPE_IB) &&
	    underlay_qpn == 0)
		return 0;

	MLX5_SET(set_flow_table_root_in, in, opcode,
		 MLX5_CMD_OP_SET_FLOW_TABLE_ROOT);
	MLX5_SET(set_flow_table_root_in, in, table_type, ft->type);

	if (disconnect) {
		MLX5_SET(set_flow_table_root_in, in, op_mod, 1);
		MLX5_SET(set_flow_table_root_in, in, table_id, 0);
	} else {
		MLX5_SET(set_flow_table_root_in, in, op_mod, 0);
		MLX5_SET(set_flow_table_root_in, in, table_id, ft->id);
	}

	MLX5_SET(set_flow_table_root_in, in, underlay_qpn, underlay_qpn);
	if (ft->vport) {
		MLX5_SET(set_flow_table_root_in, in, vport_number, ft->vport);
		MLX5_SET(set_flow_table_root_in, in, other_vport, 1);
	}

	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_cmd_create_flow_table(struct mlx5_core_dev *dev,
			       u16 vport,
			       enum fs_flow_table_op_mod op_mod,
			       enum fs_flow_table_type type, unsigned int level,
			       unsigned int log_size, struct mlx5_flow_table
			       *next_ft, unsigned int *table_id, u32 flags)
{
	int en_encap_decap = !!(flags & MLX5_FLOW_TABLE_TUNNEL_EN);
	u32 out[MLX5_ST_SZ_DW(create_flow_table_out)] = {0};
	u32 in[MLX5_ST_SZ_DW(create_flow_table_in)]   = {0};
	int err;

	MLX5_SET(create_flow_table_in, in, opcode,
		 MLX5_CMD_OP_CREATE_FLOW_TABLE);

	MLX5_SET(create_flow_table_in, in, table_type, type);
	MLX5_SET(create_flow_table_in, in, flow_table_context.level, level);
	MLX5_SET(create_flow_table_in, in, flow_table_context.log_size, log_size);
	if (vport) {
		MLX5_SET(create_flow_table_in, in, vport_number, vport);
		MLX5_SET(create_flow_table_in, in, other_vport, 1);
	}

	MLX5_SET(create_flow_table_in, in, flow_table_context.decap_en,
		 en_encap_decap);
	MLX5_SET(create_flow_table_in, in, flow_table_context.encap_en,
		 en_encap_decap);

	switch (op_mod) {
	case FS_FT_OP_MOD_NORMAL:
		if (next_ft) {
			MLX5_SET(create_flow_table_in, in,
				 flow_table_context.table_miss_action, 1);
			MLX5_SET(create_flow_table_in, in,
				 flow_table_context.table_miss_id, next_ft->id);
		}
		break;

	case FS_FT_OP_MOD_LAG_DEMUX:
		MLX5_SET(create_flow_table_in, in, op_mod, 0x1);
		if (next_ft)
			MLX5_SET(create_flow_table_in, in,
				 flow_table_context.lag_master_next_table_id,
				 next_ft->id);
		break;
	}

	err = mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
	if (!err)
		*table_id = MLX5_GET(create_flow_table_out, out,
				     table_id);
	return err;
}

int mlx5_cmd_destroy_flow_table(struct mlx5_core_dev *dev,
				struct mlx5_flow_table *ft)
{
	u32 in[MLX5_ST_SZ_DW(destroy_flow_table_in)]   = {0};
	u32 out[MLX5_ST_SZ_DW(destroy_flow_table_out)] = {0};

	MLX5_SET(destroy_flow_table_in, in, opcode,
		 MLX5_CMD_OP_DESTROY_FLOW_TABLE);
	MLX5_SET(destroy_flow_table_in, in, table_type, ft->type);
	MLX5_SET(destroy_flow_table_in, in, table_id, ft->id);
	if (ft->vport) {
		MLX5_SET(destroy_flow_table_in, in, vport_number, ft->vport);
		MLX5_SET(destroy_flow_table_in, in, other_vport, 1);
	}

	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_cmd_modify_flow_table(struct mlx5_core_dev *dev,
			       struct mlx5_flow_table *ft,
			       struct mlx5_flow_table *next_ft)
{
	u32 in[MLX5_ST_SZ_DW(modify_flow_table_in)]   = {0};
	u32 out[MLX5_ST_SZ_DW(modify_flow_table_out)] = {0};

	MLX5_SET(modify_flow_table_in, in, opcode,
		 MLX5_CMD_OP_MODIFY_FLOW_TABLE);
	MLX5_SET(modify_flow_table_in, in, table_type, ft->type);
	MLX5_SET(modify_flow_table_in, in, table_id, ft->id);

	if (ft->op_mod == FS_FT_OP_MOD_LAG_DEMUX) {
		MLX5_SET(modify_flow_table_in, in, modify_field_select,
			 MLX5_MODIFY_FLOW_TABLE_LAG_NEXT_TABLE_ID);
		if (next_ft) {
			MLX5_SET(modify_flow_table_in, in,
				 flow_table_context.lag_master_next_table_id, next_ft->id);
		} else {
			MLX5_SET(modify_flow_table_in, in,
				 flow_table_context.lag_master_next_table_id, 0);
		}
	} else {
		if (ft->vport) {
			MLX5_SET(modify_flow_table_in, in, vport_number,
				 ft->vport);
			MLX5_SET(modify_flow_table_in, in, other_vport, 1);
		}
		MLX5_SET(modify_flow_table_in, in, modify_field_select,
			 MLX5_MODIFY_FLOW_TABLE_MISS_TABLE_ID);
		if (next_ft) {
			MLX5_SET(modify_flow_table_in, in,
				 flow_table_context.table_miss_action, 1);
			MLX5_SET(modify_flow_table_in, in,
				 flow_table_context.table_miss_id,
				 next_ft->id);
		} else {
			MLX5_SET(modify_flow_table_in, in,
				 flow_table_context.table_miss_action, 0);
		}
	}

	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_cmd_create_flow_group(struct mlx5_core_dev *dev,
			       struct mlx5_flow_table *ft,
			       u32 *in,
			       unsigned int *group_id)
{
	u32 out[MLX5_ST_SZ_DW(create_flow_group_out)] = {0};
	int inlen = MLX5_ST_SZ_BYTES(create_flow_group_in);
	int err;

	MLX5_SET(create_flow_group_in, in, opcode,
		 MLX5_CMD_OP_CREATE_FLOW_GROUP);
	MLX5_SET(create_flow_group_in, in, table_type, ft->type);
	MLX5_SET(create_flow_group_in, in, table_id, ft->id);
	if (ft->vport) {
		MLX5_SET(create_flow_group_in, in, vport_number, ft->vport);
		MLX5_SET(create_flow_group_in, in, other_vport, 1);
	}

	err = mlx5_cmd_exec(dev, in, inlen, out, sizeof(out));
	if (!err)
		*group_id = MLX5_GET(create_flow_group_out, out,
				     group_id);
	return err;
}

int mlx5_cmd_destroy_flow_group(struct mlx5_core_dev *dev,
				struct mlx5_flow_table *ft,
				unsigned int group_id)
{
	u32 out[MLX5_ST_SZ_DW(destroy_flow_group_out)] = {0};
	u32 in[MLX5_ST_SZ_DW(destroy_flow_group_in)]   = {0};

	MLX5_SET(destroy_flow_group_in, in, opcode,
		 MLX5_CMD_OP_DESTROY_FLOW_GROUP);
	MLX5_SET(destroy_flow_group_in, in, table_type, ft->type);
	MLX5_SET(destroy_flow_group_in, in, table_id, ft->id);
	MLX5_SET(destroy_flow_group_in, in, group_id, group_id);
	if (ft->vport) {
		MLX5_SET(destroy_flow_group_in, in, vport_number, ft->vport);
		MLX5_SET(destroy_flow_group_in, in, other_vport, 1);
	}

	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

static int mlx5_cmd_set_fte(struct mlx5_core_dev *dev,
			    int opmod, int modify_mask,
			    struct mlx5_flow_table *ft,
			    unsigned group_id,
			    struct fs_fte *fte)
{
	unsigned int inlen = MLX5_ST_SZ_BYTES(set_fte_in) +
		fte->dests_size * MLX5_ST_SZ_BYTES(dest_format_struct);
	u32 out[MLX5_ST_SZ_DW(set_fte_out)] = {0};
	struct mlx5_flow_rule *dst;
	void *in_flow_context;
	void *in_match_value;
	void *in_dests;
	u32 *in;
	int err;

	in = kvzalloc(inlen, GFP_KERNEL);
	if (!in)
		return -ENOMEM;

	MLX5_SET(set_fte_in, in, opcode, MLX5_CMD_OP_SET_FLOW_TABLE_ENTRY);
	MLX5_SET(set_fte_in, in, op_mod, opmod);
	MLX5_SET(set_fte_in, in, modify_enable_mask, modify_mask);
	MLX5_SET(set_fte_in, in, table_type, ft->type);
	MLX5_SET(set_fte_in, in, table_id,   ft->id);
	MLX5_SET(set_fte_in, in, flow_index, fte->index);
	if (ft->vport) {
		MLX5_SET(set_fte_in, in, vport_number, ft->vport);
		MLX5_SET(set_fte_in, in, other_vport, 1);
	}

	in_flow_context = MLX5_ADDR_OF(set_fte_in, in, flow_context);
	MLX5_SET(flow_context, in_flow_context, group_id, group_id);
	MLX5_SET(flow_context, in_flow_context, flow_tag, fte->flow_tag);
	MLX5_SET(flow_context, in_flow_context, action, fte->action);
	MLX5_SET(flow_context, in_flow_context, encap_id, fte->encap_id);
	MLX5_SET(flow_context, in_flow_context, modify_header_id, fte->modify_id);
	in_match_value = MLX5_ADDR_OF(flow_context, in_flow_context,
				      match_value);
	memcpy(in_match_value, &fte->val, sizeof(fte->val));

	in_dests = MLX5_ADDR_OF(flow_context, in_flow_context, destination);
	if (fte->action & MLX5_FLOW_CONTEXT_ACTION_FWD_DEST) {
		int list_size = 0;

		list_for_each_entry(dst, &fte->node.children, node.list) {
			unsigned int id;

			if (dst->dest_attr.type == MLX5_FLOW_DESTINATION_TYPE_COUNTER)
				continue;

			MLX5_SET(dest_format_struct, in_dests, destination_type,
				 dst->dest_attr.type);
			if (dst->dest_attr.type ==
			    MLX5_FLOW_DESTINATION_TYPE_FLOW_TABLE) {
				id = dst->dest_attr.ft->id;
			} else {
				id = dst->dest_attr.tir_num;
			}
			MLX5_SET(dest_format_struct, in_dests, destination_id, id);
			in_dests += MLX5_ST_SZ_BYTES(dest_format_struct);
			list_size++;
		}

		MLX5_SET(flow_context, in_flow_context, destination_list_size,
			 list_size);
	}

	if (fte->action & MLX5_FLOW_CONTEXT_ACTION_COUNT) {
		int max_list_size = BIT(MLX5_CAP_FLOWTABLE_TYPE(dev,
					log_max_flow_counter,
					ft->type));
		int list_size = 0;

		list_for_each_entry(dst, &fte->node.children, node.list) {
			if (dst->dest_attr.type !=
			    MLX5_FLOW_DESTINATION_TYPE_COUNTER)
				continue;

			MLX5_SET(flow_counter_list, in_dests, flow_counter_id,
				 dst->dest_attr.counter->id);
			in_dests += MLX5_ST_SZ_BYTES(dest_format_struct);
			list_size++;
		}
		if (list_size > max_list_size) {
			err = -EINVAL;
			goto err_out;
		}

		MLX5_SET(flow_context, in_flow_context, flow_counter_list_size,
			 list_size);
	}

	err = mlx5_cmd_exec(dev, in, inlen, out, sizeof(out));
err_out:
	kvfree(in);
	return err;
}

int mlx5_cmd_create_fte(struct mlx5_core_dev *dev,
			struct mlx5_flow_table *ft,
			unsigned group_id,
			struct fs_fte *fte)
{
	return mlx5_cmd_set_fte(dev, 0, 0, ft, group_id, fte);
}

int mlx5_cmd_update_fte(struct mlx5_core_dev *dev,
			struct mlx5_flow_table *ft,
			unsigned group_id,
			int modify_mask,
			struct fs_fte *fte)
{
	int opmod;
	int atomic_mod_cap = MLX5_CAP_FLOWTABLE(dev,
						flow_table_properties_nic_receive.
						flow_modify_en);
	if (!atomic_mod_cap)
		return -EOPNOTSUPP;
	opmod = 1;

	return	mlx5_cmd_set_fte(dev, opmod, modify_mask, ft, group_id, fte);
}

int mlx5_cmd_delete_fte(struct mlx5_core_dev *dev,
			struct mlx5_flow_table *ft,
			unsigned int index)
{
	u32 out[MLX5_ST_SZ_DW(delete_fte_out)] = {0};
	u32 in[MLX5_ST_SZ_DW(delete_fte_in)]   = {0};

	MLX5_SET(delete_fte_in, in, opcode, MLX5_CMD_OP_DELETE_FLOW_TABLE_ENTRY);
	MLX5_SET(delete_fte_in, in, table_type, ft->type);
	MLX5_SET(delete_fte_in, in, table_id, ft->id);
	MLX5_SET(delete_fte_in, in, flow_index, index);
	if (ft->vport) {
		MLX5_SET(delete_fte_in, in, vport_number, ft->vport);
		MLX5_SET(delete_fte_in, in, other_vport, 1);
	}

	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_cmd_fc_alloc(struct mlx5_core_dev *dev, u32 *id)
{
	u32 in[MLX5_ST_SZ_DW(alloc_flow_counter_in)]   = {0};
	u32 out[MLX5_ST_SZ_DW(alloc_flow_counter_out)] = {0};
	int err;

	MLX5_SET(alloc_flow_counter_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_FLOW_COUNTER);

	err = mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
	if (!err)
		*id = MLX5_GET(alloc_flow_counter_out, out, flow_counter_id);
	return err;
}

int mlx5_cmd_fc_free(struct mlx5_core_dev *dev, u32 id)
{
	u32 in[MLX5_ST_SZ_DW(dealloc_flow_counter_in)]   = {0};
	u32 out[MLX5_ST_SZ_DW(dealloc_flow_counter_out)] = {0};

	MLX5_SET(dealloc_flow_counter_in, in, opcode,
		 MLX5_CMD_OP_DEALLOC_FLOW_COUNTER);
	MLX5_SET(dealloc_flow_counter_in, in, flow_counter_id, id);
	return mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_cmd_fc_query(struct mlx5_core_dev *dev, u32 id,
		      u64 *packets, u64 *bytes)
{
	u32 out[MLX5_ST_SZ_BYTES(query_flow_counter_out) +
		MLX5_ST_SZ_BYTES(traffic_counter)]   = {0};
	u32 in[MLX5_ST_SZ_DW(query_flow_counter_in)] = {0};
	void *stats;
	int err = 0;

	MLX5_SET(query_flow_counter_in, in, opcode,
		 MLX5_CMD_OP_QUERY_FLOW_COUNTER);
	MLX5_SET(query_flow_counter_in, in, op_mod, 0);
	MLX5_SET(query_flow_counter_in, in, flow_counter_id, id);
	err = mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
	if (err)
		return err;

	stats = MLX5_ADDR_OF(query_flow_counter_out, out, flow_statistics);
	*packets = MLX5_GET64(traffic_counter, stats, packets);
	*bytes = MLX5_GET64(traffic_counter, stats, octets);
	return 0;
}

struct mlx5_cmd_fc_bulk {
	u32 id;
	int num;
	int outlen;
	u32 out[0];
};

struct mlx5_cmd_fc_bulk *
mlx5_cmd_fc_bulk_alloc(struct mlx5_core_dev *dev, u32 id, int num)
{
	struct mlx5_cmd_fc_bulk *b;
	int outlen =
		MLX5_ST_SZ_BYTES(query_flow_counter_out) +
		MLX5_ST_SZ_BYTES(traffic_counter) * num;

	b = kzalloc(sizeof(*b) + outlen, GFP_KERNEL);
	if (!b)
		return NULL;

	b->id = id;
	b->num = num;
	b->outlen = outlen;

	return b;
}

void mlx5_cmd_fc_bulk_free(struct mlx5_cmd_fc_bulk *b)
{
	kfree(b);
}

int
mlx5_cmd_fc_bulk_query(struct mlx5_core_dev *dev, struct mlx5_cmd_fc_bulk *b)
{
	u32 in[MLX5_ST_SZ_DW(query_flow_counter_in)] = {0};

	MLX5_SET(query_flow_counter_in, in, opcode,
		 MLX5_CMD_OP_QUERY_FLOW_COUNTER);
	MLX5_SET(query_flow_counter_in, in, op_mod, 0);
	MLX5_SET(query_flow_counter_in, in, flow_counter_id, b->id);
	MLX5_SET(query_flow_counter_in, in, num_of_counters, b->num);
	return mlx5_cmd_exec(dev, in, sizeof(in), b->out, b->outlen);
}

void mlx5_cmd_fc_bulk_get(struct mlx5_core_dev *dev,
			  struct mlx5_cmd_fc_bulk *b, u32 id,
			  u64 *packets, u64 *bytes)
{
	int index = id - b->id;
	void *stats;

	if (index < 0 || index >= b->num) {
		mlx5_core_warn(dev, "Flow counter id (0x%x) out of range (0x%x..0x%x). Counter ignored.\n",
			       id, b->id, b->id + b->num - 1);
		return;
	}

	stats = MLX5_ADDR_OF(query_flow_counter_out, b->out,
			     flow_statistics[index]);
	*packets = MLX5_GET64(traffic_counter, stats, packets);
	*bytes = MLX5_GET64(traffic_counter, stats, octets);
}

int mlx5_encap_alloc(struct mlx5_core_dev *dev,
		     int header_type,
		     size_t size,
		     void *encap_header,
		     u32 *encap_id)
{
	int max_encap_size = MLX5_CAP_ESW(dev, max_encap_header_size);
	u32 out[MLX5_ST_SZ_DW(alloc_encap_header_out)];
	void *encap_header_in;
	void *header;
	int inlen;
	int err;
	u32 *in;

	if (size > max_encap_size) {
		mlx5_core_warn(dev, "encap size %zd too big, max supported is %d\n",
			       size, max_encap_size);
		return -EINVAL;
	}

	in = kzalloc(MLX5_ST_SZ_BYTES(alloc_encap_header_in) + size,
		     GFP_KERNEL);
	if (!in)
		return -ENOMEM;

	encap_header_in = MLX5_ADDR_OF(alloc_encap_header_in, in, encap_header);
	header = MLX5_ADDR_OF(encap_header_in, encap_header_in, encap_header);
	inlen = header - (void *)in  + size;

	memset(in, 0, inlen);
	MLX5_SET(alloc_encap_header_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_ENCAP_HEADER);
	MLX5_SET(encap_header_in, encap_header_in, encap_header_size, size);
	MLX5_SET(encap_header_in, encap_header_in, header_type, header_type);
	memcpy(header, encap_header, size);

	memset(out, 0, sizeof(out));
	err = mlx5_cmd_exec(dev, in, inlen, out, sizeof(out));

	*encap_id = MLX5_GET(alloc_encap_header_out, out, encap_id);
	kfree(in);
	return err;
}

void mlx5_encap_dealloc(struct mlx5_core_dev *dev, u32 encap_id)
{
	u32 in[MLX5_ST_SZ_DW(dealloc_encap_header_in)];
	u32 out[MLX5_ST_SZ_DW(dealloc_encap_header_out)];

	memset(in, 0, sizeof(in));
	MLX5_SET(dealloc_encap_header_in, in, opcode,
		 MLX5_CMD_OP_DEALLOC_ENCAP_HEADER);
	MLX5_SET(dealloc_encap_header_in, in, encap_id, encap_id);

	mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}

int mlx5_modify_header_alloc(struct mlx5_core_dev *dev,
			     u8 namespace, u8 num_actions,
			     void *modify_actions, u32 *modify_header_id)
{
	u32 out[MLX5_ST_SZ_DW(alloc_modify_header_context_out)];
	int max_actions, actions_size, inlen, err;
	void *actions_in;
	u8 table_type;
	u32 *in;

	switch (namespace) {
	case MLX5_FLOW_NAMESPACE_FDB:
		max_actions = MLX5_CAP_ESW_FLOWTABLE_FDB(dev, max_modify_header_actions);
		table_type = FS_FT_FDB;
		break;
	case MLX5_FLOW_NAMESPACE_KERNEL:
		max_actions = MLX5_CAP_FLOWTABLE_NIC_RX(dev, max_modify_header_actions);
		table_type = FS_FT_NIC_RX;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (num_actions > max_actions) {
		mlx5_core_warn(dev, "too many modify header actions %d, max supported %d\n",
			       num_actions, max_actions);
		return -EOPNOTSUPP;
	}

	actions_size = MLX5_UN_SZ_BYTES(set_action_in_add_action_in_auto) * num_actions;
	inlen = MLX5_ST_SZ_BYTES(alloc_modify_header_context_in) + actions_size;

	in = kzalloc(inlen, GFP_KERNEL);
	if (!in)
		return -ENOMEM;

	MLX5_SET(alloc_modify_header_context_in, in, opcode,
		 MLX5_CMD_OP_ALLOC_MODIFY_HEADER_CONTEXT);
	MLX5_SET(alloc_modify_header_context_in, in, table_type, table_type);
	MLX5_SET(alloc_modify_header_context_in, in, num_of_actions, num_actions);

	actions_in = MLX5_ADDR_OF(alloc_modify_header_context_in, in, actions);
	memcpy(actions_in, modify_actions, actions_size);

	memset(out, 0, sizeof(out));
	err = mlx5_cmd_exec(dev, in, inlen, out, sizeof(out));

	*modify_header_id = MLX5_GET(alloc_modify_header_context_out, out, modify_header_id);
	kfree(in);
	return err;
}

void mlx5_modify_header_dealloc(struct mlx5_core_dev *dev, u32 modify_header_id)
{
	u32 in[MLX5_ST_SZ_DW(dealloc_modify_header_context_in)];
	u32 out[MLX5_ST_SZ_DW(dealloc_modify_header_context_out)];

	memset(in, 0, sizeof(in));
	MLX5_SET(dealloc_modify_header_context_in, in, opcode,
		 MLX5_CMD_OP_DEALLOC_MODIFY_HEADER_CONTEXT);
	MLX5_SET(dealloc_modify_header_context_in, in, modify_header_id,
		 modify_header_id);

	mlx5_cmd_exec(dev, in, sizeof(in), out, sizeof(out));
}
