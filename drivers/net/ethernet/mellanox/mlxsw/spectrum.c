/*
 * drivers/net/ethernet/mellanox/mlxsw/spectrum.c
 * Copyright (c) 2015-2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2015-2017 Jiri Pirko <jiri@mellanox.com>
 * Copyright (c) 2015 Ido Schimmel <idosch@mellanox.com>
 * Copyright (c) 2015 Elad Raz <eladr@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/skbuff.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/dcbnl.h>
#include <linux/inetdevice.h>
#include <linux/netlink.h>
#include <net/switchdev.h>
#include <net/pkt_cls.h>
#include <net/tc_act/tc_mirred.h>
#include <net/netevent.h>
#include <net/tc_act/tc_sample.h>
#include <net/addrconf.h>

#include "spectrum.h"
#include "pci.h"
#include "core.h"
#include "reg.h"
#include "port.h"
#include "trap.h"
#include "txheader.h"
#include "spectrum_cnt.h"
#include "spectrum_dpipe.h"
#include "spectrum_acl_flex_actions.h"
#include "../mlxfw/mlxfw.h"

#define MLXSW_FWREV_MAJOR 13
#define MLXSW_FWREV_MINOR 1530
#define MLXSW_FWREV_SUBMINOR 152
#define MLXSW_FWREV_MINOR_TO_BRANCH(minor) ((minor) / 100)

#define MLXSW_SP_FW_FILENAME \
	"mellanox/mlxsw_spectrum-" __stringify(MLXSW_FWREV_MAJOR) \
	"." __stringify(MLXSW_FWREV_MINOR) \
	"." __stringify(MLXSW_FWREV_SUBMINOR) ".mfa2"

static const char mlxsw_sp_driver_name[] = "mlxsw_spectrum";
static const char mlxsw_sp_driver_version[] = "1.0";

/* tx_hdr_version
 * Tx header version.
 * Must be set to 1.
 */
MLXSW_ITEM32(tx, hdr, version, 0x00, 28, 4);

/* tx_hdr_ctl
 * Packet control type.
 * 0 - Ethernet control (e.g. EMADs, LACP)
 * 1 - Ethernet data
 */
MLXSW_ITEM32(tx, hdr, ctl, 0x00, 26, 2);

/* tx_hdr_proto
 * Packet protocol type. Must be set to 1 (Ethernet).
 */
MLXSW_ITEM32(tx, hdr, proto, 0x00, 21, 3);

/* tx_hdr_rx_is_router
 * Packet is sent from the router. Valid for data packets only.
 */
MLXSW_ITEM32(tx, hdr, rx_is_router, 0x00, 19, 1);

/* tx_hdr_fid_valid
 * Indicates if the 'fid' field is valid and should be used for
 * forwarding lookup. Valid for data packets only.
 */
MLXSW_ITEM32(tx, hdr, fid_valid, 0x00, 16, 1);

/* tx_hdr_swid
 * Switch partition ID. Must be set to 0.
 */
MLXSW_ITEM32(tx, hdr, swid, 0x00, 12, 3);

/* tx_hdr_control_tclass
 * Indicates if the packet should use the control TClass and not one
 * of the data TClasses.
 */
MLXSW_ITEM32(tx, hdr, control_tclass, 0x00, 6, 1);

/* tx_hdr_etclass
 * Egress TClass to be used on the egress device on the egress port.
 */
MLXSW_ITEM32(tx, hdr, etclass, 0x00, 0, 4);

/* tx_hdr_port_mid
 * Destination local port for unicast packets.
 * Destination multicast ID for multicast packets.
 *
 * Control packets are directed to a specific egress port, while data
 * packets are transmitted through the CPU port (0) into the switch partition,
 * where forwarding rules are applied.
 */
MLXSW_ITEM32(tx, hdr, port_mid, 0x04, 16, 16);

/* tx_hdr_fid
 * Forwarding ID used for L2 forwarding lookup. Valid only if 'fid_valid' is
 * set, otherwise calculated based on the packet's VID using VID to FID mapping.
 * Valid for data packets only.
 */
MLXSW_ITEM32(tx, hdr, fid, 0x08, 0, 16);

/* tx_hdr_type
 * 0 - Data packets
 * 6 - Control packets
 */
MLXSW_ITEM32(tx, hdr, type, 0x0C, 0, 4);

struct mlxsw_sp_mlxfw_dev {
	struct mlxfw_dev mlxfw_dev;
	struct mlxsw_sp *mlxsw_sp;
};

static int mlxsw_sp_component_query(struct mlxfw_dev *mlxfw_dev,
				    u16 component_index, u32 *p_max_size,
				    u8 *p_align_bits, u16 *p_max_write_size)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcqi_pl[MLXSW_REG_MCQI_LEN];
	int err;

	mlxsw_reg_mcqi_pack(mcqi_pl, component_index);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(mcqi), mcqi_pl);
	if (err)
		return err;
	mlxsw_reg_mcqi_unpack(mcqi_pl, p_max_size, p_align_bits,
			      p_max_write_size);

	*p_align_bits = max_t(u8, *p_align_bits, 2);
	*p_max_write_size = min_t(u16, *p_max_write_size,
				  MLXSW_REG_MCDA_MAX_DATA_LEN);
	return 0;
}

static int mlxsw_sp_fsm_lock(struct mlxfw_dev *mlxfw_dev, u32 *fwhandle)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];
	u8 control_state;
	int err;

	mlxsw_reg_mcc_pack(mcc_pl, 0, 0, 0, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
	if (err)
		return err;

	mlxsw_reg_mcc_unpack(mcc_pl, fwhandle, NULL, &control_state);
	if (control_state != MLXFW_FSM_STATE_IDLE)
		return -EBUSY;

	mlxsw_reg_mcc_pack(mcc_pl,
			   MLXSW_REG_MCC_INSTRUCTION_LOCK_UPDATE_HANDLE,
			   0, *fwhandle, 0);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static int mlxsw_sp_fsm_component_update(struct mlxfw_dev *mlxfw_dev,
					 u32 fwhandle, u16 component_index,
					 u32 component_size)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];

	mlxsw_reg_mcc_pack(mcc_pl, MLXSW_REG_MCC_INSTRUCTION_UPDATE_COMPONENT,
			   component_index, fwhandle, component_size);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static int mlxsw_sp_fsm_block_download(struct mlxfw_dev *mlxfw_dev,
				       u32 fwhandle, u8 *data, u16 size,
				       u32 offset)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcda_pl[MLXSW_REG_MCDA_LEN];

	mlxsw_reg_mcda_pack(mcda_pl, fwhandle, offset, size, data);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcda), mcda_pl);
}

static int mlxsw_sp_fsm_component_verify(struct mlxfw_dev *mlxfw_dev,
					 u32 fwhandle, u16 component_index)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];

	mlxsw_reg_mcc_pack(mcc_pl, MLXSW_REG_MCC_INSTRUCTION_VERIFY_COMPONENT,
			   component_index, fwhandle, 0);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static int mlxsw_sp_fsm_activate(struct mlxfw_dev *mlxfw_dev, u32 fwhandle)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];

	mlxsw_reg_mcc_pack(mcc_pl, MLXSW_REG_MCC_INSTRUCTION_ACTIVATE, 0,
			   fwhandle, 0);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static int mlxsw_sp_fsm_query_state(struct mlxfw_dev *mlxfw_dev, u32 fwhandle,
				    enum mlxfw_fsm_state *fsm_state,
				    enum mlxfw_fsm_state_err *fsm_state_err)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];
	u8 control_state;
	u8 error_code;
	int err;

	mlxsw_reg_mcc_pack(mcc_pl, 0, 0, fwhandle, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
	if (err)
		return err;

	mlxsw_reg_mcc_unpack(mcc_pl, NULL, &error_code, &control_state);
	*fsm_state = control_state;
	*fsm_state_err = min_t(enum mlxfw_fsm_state_err, error_code,
			       MLXFW_FSM_STATE_ERR_MAX);
	return 0;
}

static void mlxsw_sp_fsm_cancel(struct mlxfw_dev *mlxfw_dev, u32 fwhandle)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];

	mlxsw_reg_mcc_pack(mcc_pl, MLXSW_REG_MCC_INSTRUCTION_CANCEL, 0,
			   fwhandle, 0);
	mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static void mlxsw_sp_fsm_release(struct mlxfw_dev *mlxfw_dev, u32 fwhandle)
{
	struct mlxsw_sp_mlxfw_dev *mlxsw_sp_mlxfw_dev =
		container_of(mlxfw_dev, struct mlxsw_sp_mlxfw_dev, mlxfw_dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_mlxfw_dev->mlxsw_sp;
	char mcc_pl[MLXSW_REG_MCC_LEN];

	mlxsw_reg_mcc_pack(mcc_pl,
			   MLXSW_REG_MCC_INSTRUCTION_RELEASE_UPDATE_HANDLE, 0,
			   fwhandle, 0);
	mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mcc), mcc_pl);
}

static const struct mlxfw_dev_ops mlxsw_sp_mlxfw_dev_ops = {
	.component_query	= mlxsw_sp_component_query,
	.fsm_lock		= mlxsw_sp_fsm_lock,
	.fsm_component_update	= mlxsw_sp_fsm_component_update,
	.fsm_block_download	= mlxsw_sp_fsm_block_download,
	.fsm_component_verify	= mlxsw_sp_fsm_component_verify,
	.fsm_activate		= mlxsw_sp_fsm_activate,
	.fsm_query_state	= mlxsw_sp_fsm_query_state,
	.fsm_cancel		= mlxsw_sp_fsm_cancel,
	.fsm_release		= mlxsw_sp_fsm_release
};

static int mlxsw_sp_firmware_flash(struct mlxsw_sp *mlxsw_sp,
				   const struct firmware *firmware)
{
	struct mlxsw_sp_mlxfw_dev mlxsw_sp_mlxfw_dev = {
		.mlxfw_dev = {
			.ops = &mlxsw_sp_mlxfw_dev_ops,
			.psid = mlxsw_sp->bus_info->psid,
			.psid_size = strlen(mlxsw_sp->bus_info->psid),
		},
		.mlxsw_sp = mlxsw_sp
	};

	return mlxfw_firmware_flash(&mlxsw_sp_mlxfw_dev.mlxfw_dev, firmware);
}

static int mlxsw_sp_fw_rev_validate(struct mlxsw_sp *mlxsw_sp)
{
	const struct mlxsw_fw_rev *rev = &mlxsw_sp->bus_info->fw_rev;
	const struct firmware *firmware;
	int err;

	/* Validate driver & FW are compatible */
	if (rev->major != MLXSW_FWREV_MAJOR) {
		WARN(1, "Mismatch in major FW version [%d:%d] is never expected; Please contact support\n",
		     rev->major, MLXSW_FWREV_MAJOR);
		return -EINVAL;
	}
	if (MLXSW_FWREV_MINOR_TO_BRANCH(rev->minor) ==
	    MLXSW_FWREV_MINOR_TO_BRANCH(MLXSW_FWREV_MINOR))
		return 0;

	dev_info(mlxsw_sp->bus_info->dev, "The firmware version %d.%d.%d is incompatible with the driver\n",
		 rev->major, rev->minor, rev->subminor);
	dev_info(mlxsw_sp->bus_info->dev, "Flashing firmware using file %s\n",
		 MLXSW_SP_FW_FILENAME);

	err = request_firmware_direct(&firmware, MLXSW_SP_FW_FILENAME,
				      mlxsw_sp->bus_info->dev);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Could not request firmware file %s\n",
			MLXSW_SP_FW_FILENAME);
		return err;
	}

	err = mlxsw_sp_firmware_flash(mlxsw_sp, firmware);
	release_firmware(firmware);
	return err;
}

int mlxsw_sp_flow_counter_get(struct mlxsw_sp *mlxsw_sp,
			      unsigned int counter_index, u64 *packets,
			      u64 *bytes)
{
	char mgpc_pl[MLXSW_REG_MGPC_LEN];
	int err;

	mlxsw_reg_mgpc_pack(mgpc_pl, counter_index, MLXSW_REG_MGPC_OPCODE_NOP,
			    MLXSW_REG_FLOW_COUNTER_SET_TYPE_PACKETS_BYTES);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(mgpc), mgpc_pl);
	if (err)
		return err;
	if (packets)
		*packets = mlxsw_reg_mgpc_packet_counter_get(mgpc_pl);
	if (bytes)
		*bytes = mlxsw_reg_mgpc_byte_counter_get(mgpc_pl);
	return 0;
}

static int mlxsw_sp_flow_counter_clear(struct mlxsw_sp *mlxsw_sp,
				       unsigned int counter_index)
{
	char mgpc_pl[MLXSW_REG_MGPC_LEN];

	mlxsw_reg_mgpc_pack(mgpc_pl, counter_index, MLXSW_REG_MGPC_OPCODE_CLEAR,
			    MLXSW_REG_FLOW_COUNTER_SET_TYPE_PACKETS_BYTES);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mgpc), mgpc_pl);
}

int mlxsw_sp_flow_counter_alloc(struct mlxsw_sp *mlxsw_sp,
				unsigned int *p_counter_index)
{
	int err;

	err = mlxsw_sp_counter_alloc(mlxsw_sp, MLXSW_SP_COUNTER_SUB_POOL_FLOW,
				     p_counter_index);
	if (err)
		return err;
	err = mlxsw_sp_flow_counter_clear(mlxsw_sp, *p_counter_index);
	if (err)
		goto err_counter_clear;
	return 0;

err_counter_clear:
	mlxsw_sp_counter_free(mlxsw_sp, MLXSW_SP_COUNTER_SUB_POOL_FLOW,
			      *p_counter_index);
	return err;
}

void mlxsw_sp_flow_counter_free(struct mlxsw_sp *mlxsw_sp,
				unsigned int counter_index)
{
	 mlxsw_sp_counter_free(mlxsw_sp, MLXSW_SP_COUNTER_SUB_POOL_FLOW,
			       counter_index);
}

static void mlxsw_sp_txhdr_construct(struct sk_buff *skb,
				     const struct mlxsw_tx_info *tx_info)
{
	char *txhdr = skb_push(skb, MLXSW_TXHDR_LEN);

	memset(txhdr, 0, MLXSW_TXHDR_LEN);

	mlxsw_tx_hdr_version_set(txhdr, MLXSW_TXHDR_VERSION_1);
	mlxsw_tx_hdr_ctl_set(txhdr, MLXSW_TXHDR_ETH_CTL);
	mlxsw_tx_hdr_proto_set(txhdr, MLXSW_TXHDR_PROTO_ETH);
	mlxsw_tx_hdr_swid_set(txhdr, 0);
	mlxsw_tx_hdr_control_tclass_set(txhdr, 1);
	mlxsw_tx_hdr_port_mid_set(txhdr, tx_info->local_port);
	mlxsw_tx_hdr_type_set(txhdr, MLXSW_TXHDR_TYPE_CONTROL);
}

int mlxsw_sp_port_vid_stp_set(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid,
			      u8 state)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	enum mlxsw_reg_spms_state spms_state;
	char *spms_pl;
	int err;

	switch (state) {
	case BR_STATE_FORWARDING:
		spms_state = MLXSW_REG_SPMS_STATE_FORWARDING;
		break;
	case BR_STATE_LEARNING:
		spms_state = MLXSW_REG_SPMS_STATE_LEARNING;
		break;
	case BR_STATE_LISTENING: /* fall-through */
	case BR_STATE_DISABLED: /* fall-through */
	case BR_STATE_BLOCKING:
		spms_state = MLXSW_REG_SPMS_STATE_DISCARDING;
		break;
	default:
		BUG();
	}

	spms_pl = kmalloc(MLXSW_REG_SPMS_LEN, GFP_KERNEL);
	if (!spms_pl)
		return -ENOMEM;
	mlxsw_reg_spms_pack(spms_pl, mlxsw_sp_port->local_port);
	mlxsw_reg_spms_vid_pack(spms_pl, vid, spms_state);

	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spms), spms_pl);
	kfree(spms_pl);
	return err;
}

static int mlxsw_sp_base_mac_get(struct mlxsw_sp *mlxsw_sp)
{
	char spad_pl[MLXSW_REG_SPAD_LEN] = {0};
	int err;

	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(spad), spad_pl);
	if (err)
		return err;
	mlxsw_reg_spad_base_mac_memcpy_from(spad_pl, mlxsw_sp->base_mac);
	return 0;
}

static int mlxsw_sp_span_init(struct mlxsw_sp *mlxsw_sp)
{
	int i;

	if (!MLXSW_CORE_RES_VALID(mlxsw_sp->core, MAX_SPAN))
		return -EIO;

	mlxsw_sp->span.entries_count = MLXSW_CORE_RES_GET(mlxsw_sp->core,
							  MAX_SPAN);
	mlxsw_sp->span.entries = kcalloc(mlxsw_sp->span.entries_count,
					 sizeof(struct mlxsw_sp_span_entry),
					 GFP_KERNEL);
	if (!mlxsw_sp->span.entries)
		return -ENOMEM;

	for (i = 0; i < mlxsw_sp->span.entries_count; i++)
		INIT_LIST_HEAD(&mlxsw_sp->span.entries[i].bound_ports_list);

	return 0;
}

static void mlxsw_sp_span_fini(struct mlxsw_sp *mlxsw_sp)
{
	int i;

	for (i = 0; i < mlxsw_sp->span.entries_count; i++) {
		struct mlxsw_sp_span_entry *curr = &mlxsw_sp->span.entries[i];

		WARN_ON_ONCE(!list_empty(&curr->bound_ports_list));
	}
	kfree(mlxsw_sp->span.entries);
}

static struct mlxsw_sp_span_entry *
mlxsw_sp_span_entry_create(struct mlxsw_sp_port *port)
{
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	struct mlxsw_sp_span_entry *span_entry;
	char mpat_pl[MLXSW_REG_MPAT_LEN];
	u8 local_port = port->local_port;
	int index;
	int i;
	int err;

	/* find a free entry to use */
	index = -1;
	for (i = 0; i < mlxsw_sp->span.entries_count; i++) {
		if (!mlxsw_sp->span.entries[i].used) {
			index = i;
			span_entry = &mlxsw_sp->span.entries[i];
			break;
		}
	}
	if (index < 0)
		return NULL;

	/* create a new port analayzer entry for local_port */
	mlxsw_reg_mpat_pack(mpat_pl, index, local_port, true);
	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mpat), mpat_pl);
	if (err)
		return NULL;

	span_entry->used = true;
	span_entry->id = index;
	span_entry->ref_count = 1;
	span_entry->local_port = local_port;
	return span_entry;
}

static void mlxsw_sp_span_entry_destroy(struct mlxsw_sp *mlxsw_sp,
					struct mlxsw_sp_span_entry *span_entry)
{
	u8 local_port = span_entry->local_port;
	char mpat_pl[MLXSW_REG_MPAT_LEN];
	int pa_id = span_entry->id;

	mlxsw_reg_mpat_pack(mpat_pl, pa_id, local_port, false);
	mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mpat), mpat_pl);
	span_entry->used = false;
}

struct mlxsw_sp_span_entry *
mlxsw_sp_span_entry_find(struct mlxsw_sp *mlxsw_sp, u8 local_port)
{
	int i;

	for (i = 0; i < mlxsw_sp->span.entries_count; i++) {
		struct mlxsw_sp_span_entry *curr = &mlxsw_sp->span.entries[i];

		if (curr->used && curr->local_port == local_port)
			return curr;
	}
	return NULL;
}

static struct mlxsw_sp_span_entry
*mlxsw_sp_span_entry_get(struct mlxsw_sp_port *port)
{
	struct mlxsw_sp_span_entry *span_entry;

	span_entry = mlxsw_sp_span_entry_find(port->mlxsw_sp,
					      port->local_port);
	if (span_entry) {
		/* Already exists, just take a reference */
		span_entry->ref_count++;
		return span_entry;
	}

	return mlxsw_sp_span_entry_create(port);
}

static int mlxsw_sp_span_entry_put(struct mlxsw_sp *mlxsw_sp,
				   struct mlxsw_sp_span_entry *span_entry)
{
	WARN_ON(!span_entry->ref_count);
	if (--span_entry->ref_count == 0)
		mlxsw_sp_span_entry_destroy(mlxsw_sp, span_entry);
	return 0;
}

static bool mlxsw_sp_span_is_egress_mirror(struct mlxsw_sp_port *port)
{
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	struct mlxsw_sp_span_inspected_port *p;
	int i;

	for (i = 0; i < mlxsw_sp->span.entries_count; i++) {
		struct mlxsw_sp_span_entry *curr = &mlxsw_sp->span.entries[i];

		list_for_each_entry(p, &curr->bound_ports_list, list)
			if (p->local_port == port->local_port &&
			    p->type == MLXSW_SP_SPAN_EGRESS)
				return true;
	}

	return false;
}

static int mlxsw_sp_span_mtu_to_buffsize(const struct mlxsw_sp *mlxsw_sp,
					 int mtu)
{
	return mlxsw_sp_bytes_cells(mlxsw_sp, mtu * 5 / 2) + 1;
}

static int mlxsw_sp_span_port_mtu_update(struct mlxsw_sp_port *port, u16 mtu)
{
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	char sbib_pl[MLXSW_REG_SBIB_LEN];
	int err;

	/* If port is egress mirrored, the shared buffer size should be
	 * updated according to the mtu value
	 */
	if (mlxsw_sp_span_is_egress_mirror(port)) {
		u32 buffsize = mlxsw_sp_span_mtu_to_buffsize(mlxsw_sp, mtu);

		mlxsw_reg_sbib_pack(sbib_pl, port->local_port, buffsize);
		err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sbib), sbib_pl);
		if (err) {
			netdev_err(port->dev, "Could not update shared buffer for mirroring\n");
			return err;
		}
	}

	return 0;
}

static struct mlxsw_sp_span_inspected_port *
mlxsw_sp_span_entry_bound_port_find(struct mlxsw_sp_port *port,
				    struct mlxsw_sp_span_entry *span_entry)
{
	struct mlxsw_sp_span_inspected_port *p;

	list_for_each_entry(p, &span_entry->bound_ports_list, list)
		if (port->local_port == p->local_port)
			return p;
	return NULL;
}

static int
mlxsw_sp_span_inspected_port_bind(struct mlxsw_sp_port *port,
				  struct mlxsw_sp_span_entry *span_entry,
				  enum mlxsw_sp_span_type type,
				  bool bind)
{
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	char mpar_pl[MLXSW_REG_MPAR_LEN];
	int pa_id = span_entry->id;

	/* bind the port to the SPAN entry */
	mlxsw_reg_mpar_pack(mpar_pl, port->local_port,
			    (enum mlxsw_reg_mpar_i_e) type, bind, pa_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mpar), mpar_pl);
}

static int
mlxsw_sp_span_inspected_port_add(struct mlxsw_sp_port *port,
				 struct mlxsw_sp_span_entry *span_entry,
				 enum mlxsw_sp_span_type type,
				 bool bind)
{
	struct mlxsw_sp_span_inspected_port *inspected_port;
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	char sbib_pl[MLXSW_REG_SBIB_LEN];
	int err;

	/* if it is an egress SPAN, bind a shared buffer to it */
	if (type == MLXSW_SP_SPAN_EGRESS) {
		u32 buffsize = mlxsw_sp_span_mtu_to_buffsize(mlxsw_sp,
							     port->dev->mtu);

		mlxsw_reg_sbib_pack(sbib_pl, port->local_port, buffsize);
		err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sbib), sbib_pl);
		if (err) {
			netdev_err(port->dev, "Could not create shared buffer for mirroring\n");
			return err;
		}
	}

	if (bind) {
		err = mlxsw_sp_span_inspected_port_bind(port, span_entry, type,
							true);
		if (err)
			goto err_port_bind;
	}

	inspected_port = kzalloc(sizeof(*inspected_port), GFP_KERNEL);
	if (!inspected_port) {
		err = -ENOMEM;
		goto err_inspected_port_alloc;
	}
	inspected_port->local_port = port->local_port;
	inspected_port->type = type;
	list_add_tail(&inspected_port->list, &span_entry->bound_ports_list);

	return 0;

err_inspected_port_alloc:
	if (bind)
		mlxsw_sp_span_inspected_port_bind(port, span_entry, type,
						  false);
err_port_bind:
	if (type == MLXSW_SP_SPAN_EGRESS) {
		mlxsw_reg_sbib_pack(sbib_pl, port->local_port, 0);
		mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sbib), sbib_pl);
	}
	return err;
}

static void
mlxsw_sp_span_inspected_port_del(struct mlxsw_sp_port *port,
				 struct mlxsw_sp_span_entry *span_entry,
				 enum mlxsw_sp_span_type type,
				 bool bind)
{
	struct mlxsw_sp_span_inspected_port *inspected_port;
	struct mlxsw_sp *mlxsw_sp = port->mlxsw_sp;
	char sbib_pl[MLXSW_REG_SBIB_LEN];

	inspected_port = mlxsw_sp_span_entry_bound_port_find(port, span_entry);
	if (!inspected_port)
		return;

	if (bind)
		mlxsw_sp_span_inspected_port_bind(port, span_entry, type,
						  false);
	/* remove the SBIB buffer if it was egress SPAN */
	if (type == MLXSW_SP_SPAN_EGRESS) {
		mlxsw_reg_sbib_pack(sbib_pl, port->local_port, 0);
		mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sbib), sbib_pl);
	}

	mlxsw_sp_span_entry_put(mlxsw_sp, span_entry);

	list_del(&inspected_port->list);
	kfree(inspected_port);
}

int mlxsw_sp_span_mirror_add(struct mlxsw_sp_port *from,
			     struct mlxsw_sp_port *to,
			     enum mlxsw_sp_span_type type, bool bind)
{
	struct mlxsw_sp *mlxsw_sp = from->mlxsw_sp;
	struct mlxsw_sp_span_entry *span_entry;
	int err;

	span_entry = mlxsw_sp_span_entry_get(to);
	if (!span_entry)
		return -ENOENT;

	netdev_dbg(from->dev, "Adding inspected port to SPAN entry %d\n",
		   span_entry->id);

	err = mlxsw_sp_span_inspected_port_add(from, span_entry, type, bind);
	if (err)
		goto err_port_bind;

	return 0;

err_port_bind:
	mlxsw_sp_span_entry_put(mlxsw_sp, span_entry);
	return err;
}

void mlxsw_sp_span_mirror_del(struct mlxsw_sp_port *from, u8 destination_port,
			      enum mlxsw_sp_span_type type, bool bind)
{
	struct mlxsw_sp_span_entry *span_entry;

	span_entry = mlxsw_sp_span_entry_find(from->mlxsw_sp,
					      destination_port);
	if (!span_entry) {
		netdev_err(from->dev, "no span entry found\n");
		return;
	}

	netdev_dbg(from->dev, "removing inspected port from SPAN entry %d\n",
		   span_entry->id);
	mlxsw_sp_span_inspected_port_del(from, span_entry, type, bind);
}

static int mlxsw_sp_port_sample_set(struct mlxsw_sp_port *mlxsw_sp_port,
				    bool enable, u32 rate)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char mpsc_pl[MLXSW_REG_MPSC_LEN];

	mlxsw_reg_mpsc_pack(mpsc_pl, mlxsw_sp_port->local_port, enable, rate);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mpsc), mpsc_pl);
}

static int mlxsw_sp_port_admin_status_set(struct mlxsw_sp_port *mlxsw_sp_port,
					  bool is_up)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char paos_pl[MLXSW_REG_PAOS_LEN];

	mlxsw_reg_paos_pack(paos_pl, mlxsw_sp_port->local_port,
			    is_up ? MLXSW_PORT_ADMIN_STATUS_UP :
			    MLXSW_PORT_ADMIN_STATUS_DOWN);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(paos), paos_pl);
}

static int mlxsw_sp_port_dev_addr_set(struct mlxsw_sp_port *mlxsw_sp_port,
				      unsigned char *addr)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char ppad_pl[MLXSW_REG_PPAD_LEN];

	mlxsw_reg_ppad_pack(ppad_pl, true, mlxsw_sp_port->local_port);
	mlxsw_reg_ppad_mac_memcpy_to(ppad_pl, addr);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(ppad), ppad_pl);
}

static int mlxsw_sp_port_dev_addr_init(struct mlxsw_sp_port *mlxsw_sp_port)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	unsigned char *addr = mlxsw_sp_port->dev->dev_addr;

	ether_addr_copy(addr, mlxsw_sp->base_mac);
	addr[ETH_ALEN - 1] += mlxsw_sp_port->local_port;
	return mlxsw_sp_port_dev_addr_set(mlxsw_sp_port, addr);
}

static int mlxsw_sp_port_mtu_set(struct mlxsw_sp_port *mlxsw_sp_port, u16 mtu)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char pmtu_pl[MLXSW_REG_PMTU_LEN];
	int max_mtu;
	int err;

	mtu += MLXSW_TXHDR_LEN + ETH_HLEN;
	mlxsw_reg_pmtu_pack(pmtu_pl, mlxsw_sp_port->local_port, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(pmtu), pmtu_pl);
	if (err)
		return err;
	max_mtu = mlxsw_reg_pmtu_max_mtu_get(pmtu_pl);

	if (mtu > max_mtu)
		return -EINVAL;

	mlxsw_reg_pmtu_pack(pmtu_pl, mlxsw_sp_port->local_port, mtu);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(pmtu), pmtu_pl);
}

static int mlxsw_sp_port_swid_set(struct mlxsw_sp_port *mlxsw_sp_port, u8 swid)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char pspa_pl[MLXSW_REG_PSPA_LEN];

	mlxsw_reg_pspa_pack(pspa_pl, swid, mlxsw_sp_port->local_port);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(pspa), pspa_pl);
}

int mlxsw_sp_port_vp_mode_set(struct mlxsw_sp_port *mlxsw_sp_port, bool enable)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char svpe_pl[MLXSW_REG_SVPE_LEN];

	mlxsw_reg_svpe_pack(svpe_pl, mlxsw_sp_port->local_port, enable);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(svpe), svpe_pl);
}

int mlxsw_sp_port_vid_learning_set(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid,
				   bool learn_enable)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char *spvmlr_pl;
	int err;

	spvmlr_pl = kmalloc(MLXSW_REG_SPVMLR_LEN, GFP_KERNEL);
	if (!spvmlr_pl)
		return -ENOMEM;
	mlxsw_reg_spvmlr_pack(spvmlr_pl, mlxsw_sp_port->local_port, vid, vid,
			      learn_enable);
	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spvmlr), spvmlr_pl);
	kfree(spvmlr_pl);
	return err;
}

static int __mlxsw_sp_port_pvid_set(struct mlxsw_sp_port *mlxsw_sp_port,
				    u16 vid)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char spvid_pl[MLXSW_REG_SPVID_LEN];

	mlxsw_reg_spvid_pack(spvid_pl, mlxsw_sp_port->local_port, vid);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spvid), spvid_pl);
}

static int mlxsw_sp_port_allow_untagged_set(struct mlxsw_sp_port *mlxsw_sp_port,
					    bool allow)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char spaft_pl[MLXSW_REG_SPAFT_LEN];

	mlxsw_reg_spaft_pack(spaft_pl, mlxsw_sp_port->local_port, allow);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spaft), spaft_pl);
}

int mlxsw_sp_port_pvid_set(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid)
{
	int err;

	if (!vid) {
		err = mlxsw_sp_port_allow_untagged_set(mlxsw_sp_port, false);
		if (err)
			return err;
	} else {
		err = __mlxsw_sp_port_pvid_set(mlxsw_sp_port, vid);
		if (err)
			return err;
		err = mlxsw_sp_port_allow_untagged_set(mlxsw_sp_port, true);
		if (err)
			goto err_port_allow_untagged_set;
	}

	mlxsw_sp_port->pvid = vid;
	return 0;

err_port_allow_untagged_set:
	__mlxsw_sp_port_pvid_set(mlxsw_sp_port, mlxsw_sp_port->pvid);
	return err;
}

static int
mlxsw_sp_port_system_port_mapping_set(struct mlxsw_sp_port *mlxsw_sp_port)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char sspr_pl[MLXSW_REG_SSPR_LEN];

	mlxsw_reg_sspr_pack(sspr_pl, mlxsw_sp_port->local_port);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sspr), sspr_pl);
}

static int mlxsw_sp_port_module_info_get(struct mlxsw_sp *mlxsw_sp,
					 u8 local_port, u8 *p_module,
					 u8 *p_width, u8 *p_lane)
{
	char pmlp_pl[MLXSW_REG_PMLP_LEN];
	int err;

	mlxsw_reg_pmlp_pack(pmlp_pl, local_port);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(pmlp), pmlp_pl);
	if (err)
		return err;
	*p_module = mlxsw_reg_pmlp_module_get(pmlp_pl, 0);
	*p_width = mlxsw_reg_pmlp_width_get(pmlp_pl);
	*p_lane = mlxsw_reg_pmlp_tx_lane_get(pmlp_pl, 0);
	return 0;
}

static int mlxsw_sp_port_module_map(struct mlxsw_sp_port *mlxsw_sp_port,
				    u8 module, u8 width, u8 lane)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char pmlp_pl[MLXSW_REG_PMLP_LEN];
	int i;

	mlxsw_reg_pmlp_pack(pmlp_pl, mlxsw_sp_port->local_port);
	mlxsw_reg_pmlp_width_set(pmlp_pl, width);
	for (i = 0; i < width; i++) {
		mlxsw_reg_pmlp_module_set(pmlp_pl, i, module);
		mlxsw_reg_pmlp_tx_lane_set(pmlp_pl, i, lane + i);  /* Rx & Tx */
	}

	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(pmlp), pmlp_pl);
}

static int mlxsw_sp_port_module_unmap(struct mlxsw_sp_port *mlxsw_sp_port)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char pmlp_pl[MLXSW_REG_PMLP_LEN];

	mlxsw_reg_pmlp_pack(pmlp_pl, mlxsw_sp_port->local_port);
	mlxsw_reg_pmlp_width_set(pmlp_pl, 0);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(pmlp), pmlp_pl);
}

static int mlxsw_sp_port_open(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	int err;

	err = mlxsw_sp_port_admin_status_set(mlxsw_sp_port, true);
	if (err)
		return err;
	netif_start_queue(dev);
	return 0;
}

static int mlxsw_sp_port_stop(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	netif_stop_queue(dev);
	return mlxsw_sp_port_admin_status_set(mlxsw_sp_port, false);
}

static netdev_tx_t mlxsw_sp_port_xmit(struct sk_buff *skb,
				      struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct mlxsw_sp_port_pcpu_stats *pcpu_stats;
	const struct mlxsw_tx_info tx_info = {
		.local_port = mlxsw_sp_port->local_port,
		.is_emad = false,
	};
	u64 len;
	int err;

	if (mlxsw_core_skb_transmit_busy(mlxsw_sp->core, &tx_info))
		return NETDEV_TX_BUSY;

	if (unlikely(skb_headroom(skb) < MLXSW_TXHDR_LEN)) {
		struct sk_buff *skb_orig = skb;

		skb = skb_realloc_headroom(skb, MLXSW_TXHDR_LEN);
		if (!skb) {
			this_cpu_inc(mlxsw_sp_port->pcpu_stats->tx_dropped);
			dev_kfree_skb_any(skb_orig);
			return NETDEV_TX_OK;
		}
		dev_consume_skb_any(skb_orig);
	}

	if (eth_skb_pad(skb)) {
		this_cpu_inc(mlxsw_sp_port->pcpu_stats->tx_dropped);
		return NETDEV_TX_OK;
	}

	mlxsw_sp_txhdr_construct(skb, &tx_info);
	/* TX header is consumed by HW on the way so we shouldn't count its
	 * bytes as being sent.
	 */
	len = skb->len - MLXSW_TXHDR_LEN;

	/* Due to a race we might fail here because of a full queue. In that
	 * unlikely case we simply drop the packet.
	 */
	err = mlxsw_core_skb_transmit(mlxsw_sp->core, skb, &tx_info);

	if (!err) {
		pcpu_stats = this_cpu_ptr(mlxsw_sp_port->pcpu_stats);
		u64_stats_update_begin(&pcpu_stats->syncp);
		pcpu_stats->tx_packets++;
		pcpu_stats->tx_bytes += len;
		u64_stats_update_end(&pcpu_stats->syncp);
	} else {
		this_cpu_inc(mlxsw_sp_port->pcpu_stats->tx_dropped);
		dev_kfree_skb_any(skb);
	}
	return NETDEV_TX_OK;
}

static void mlxsw_sp_set_rx_mode(struct net_device *dev)
{
}

static int mlxsw_sp_port_set_mac_address(struct net_device *dev, void *p)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct sockaddr *addr = p;
	int err;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	err = mlxsw_sp_port_dev_addr_set(mlxsw_sp_port, addr->sa_data);
	if (err)
		return err;
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	return 0;
}

static u16 mlxsw_sp_pg_buf_threshold_get(const struct mlxsw_sp *mlxsw_sp,
					 int mtu)
{
	return 2 * mlxsw_sp_bytes_cells(mlxsw_sp, mtu);
}

#define MLXSW_SP_CELL_FACTOR 2	/* 2 * cell_size / (IPG + cell_size + 1) */

static u16 mlxsw_sp_pfc_delay_get(const struct mlxsw_sp *mlxsw_sp, int mtu,
				  u16 delay)
{
	delay = mlxsw_sp_bytes_cells(mlxsw_sp, DIV_ROUND_UP(delay,
							    BITS_PER_BYTE));
	return MLXSW_SP_CELL_FACTOR * delay + mlxsw_sp_bytes_cells(mlxsw_sp,
								   mtu);
}

/* Maximum delay buffer needed in case of PAUSE frames, in bytes.
 * Assumes 100m cable and maximum MTU.
 */
#define MLXSW_SP_PAUSE_DELAY 58752

static u16 mlxsw_sp_pg_buf_delay_get(const struct mlxsw_sp *mlxsw_sp, int mtu,
				     u16 delay, bool pfc, bool pause)
{
	if (pfc)
		return mlxsw_sp_pfc_delay_get(mlxsw_sp, mtu, delay);
	else if (pause)
		return mlxsw_sp_bytes_cells(mlxsw_sp, MLXSW_SP_PAUSE_DELAY);
	else
		return 0;
}

static void mlxsw_sp_pg_buf_pack(char *pbmc_pl, int index, u16 size, u16 thres,
				 bool lossy)
{
	if (lossy)
		mlxsw_reg_pbmc_lossy_buffer_pack(pbmc_pl, index, size);
	else
		mlxsw_reg_pbmc_lossless_buffer_pack(pbmc_pl, index, size,
						    thres);
}

int __mlxsw_sp_port_headroom_set(struct mlxsw_sp_port *mlxsw_sp_port, int mtu,
				 u8 *prio_tc, bool pause_en,
				 struct ieee_pfc *my_pfc)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	u8 pfc_en = !!my_pfc ? my_pfc->pfc_en : 0;
	u16 delay = !!my_pfc ? my_pfc->delay : 0;
	char pbmc_pl[MLXSW_REG_PBMC_LEN];
	int i, j, err;

	mlxsw_reg_pbmc_pack(pbmc_pl, mlxsw_sp_port->local_port, 0, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(pbmc), pbmc_pl);
	if (err)
		return err;

	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		bool configure = false;
		bool pfc = false;
		bool lossy;
		u16 thres;

		for (j = 0; j < IEEE_8021QAZ_MAX_TCS; j++) {
			if (prio_tc[j] == i) {
				pfc = pfc_en & BIT(j);
				configure = true;
				break;
			}
		}

		if (!configure)
			continue;

		lossy = !(pfc || pause_en);
		thres = mlxsw_sp_pg_buf_threshold_get(mlxsw_sp, mtu);
		delay = mlxsw_sp_pg_buf_delay_get(mlxsw_sp, mtu, delay, pfc,
						  pause_en);
		mlxsw_sp_pg_buf_pack(pbmc_pl, i, thres + delay, thres, lossy);
	}

	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(pbmc), pbmc_pl);
}

static int mlxsw_sp_port_headroom_set(struct mlxsw_sp_port *mlxsw_sp_port,
				      int mtu, bool pause_en)
{
	u8 def_prio_tc[IEEE_8021QAZ_MAX_TCS] = {0};
	bool dcb_en = !!mlxsw_sp_port->dcb.ets;
	struct ieee_pfc *my_pfc;
	u8 *prio_tc;

	prio_tc = dcb_en ? mlxsw_sp_port->dcb.ets->prio_tc : def_prio_tc;
	my_pfc = dcb_en ? mlxsw_sp_port->dcb.pfc : NULL;

	return __mlxsw_sp_port_headroom_set(mlxsw_sp_port, mtu, prio_tc,
					    pause_en, my_pfc);
}

static int mlxsw_sp_port_change_mtu(struct net_device *dev, int mtu)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	bool pause_en = mlxsw_sp_port_is_pause_en(mlxsw_sp_port);
	int err;

	err = mlxsw_sp_port_headroom_set(mlxsw_sp_port, mtu, pause_en);
	if (err)
		return err;
	err = mlxsw_sp_span_port_mtu_update(mlxsw_sp_port, mtu);
	if (err)
		goto err_span_port_mtu_update;
	err = mlxsw_sp_port_mtu_set(mlxsw_sp_port, mtu);
	if (err)
		goto err_port_mtu_set;
	dev->mtu = mtu;
	return 0;

err_port_mtu_set:
	mlxsw_sp_span_port_mtu_update(mlxsw_sp_port, dev->mtu);
err_span_port_mtu_update:
	mlxsw_sp_port_headroom_set(mlxsw_sp_port, dev->mtu, pause_en);
	return err;
}

static int
mlxsw_sp_port_get_sw_stats64(const struct net_device *dev,
			     struct rtnl_link_stats64 *stats)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp_port_pcpu_stats *p;
	u64 rx_packets, rx_bytes, tx_packets, tx_bytes;
	u32 tx_dropped = 0;
	unsigned int start;
	int i;

	for_each_possible_cpu(i) {
		p = per_cpu_ptr(mlxsw_sp_port->pcpu_stats, i);
		do {
			start = u64_stats_fetch_begin_irq(&p->syncp);
			rx_packets	= p->rx_packets;
			rx_bytes	= p->rx_bytes;
			tx_packets	= p->tx_packets;
			tx_bytes	= p->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&p->syncp, start));

		stats->rx_packets	+= rx_packets;
		stats->rx_bytes		+= rx_bytes;
		stats->tx_packets	+= tx_packets;
		stats->tx_bytes		+= tx_bytes;
		/* tx_dropped is u32, updated without syncp protection. */
		tx_dropped	+= p->tx_dropped;
	}
	stats->tx_dropped	= tx_dropped;
	return 0;
}

static bool mlxsw_sp_port_has_offload_stats(const struct net_device *dev, int attr_id)
{
	switch (attr_id) {
	case IFLA_OFFLOAD_XSTATS_CPU_HIT:
		return true;
	}

	return false;
}

static int mlxsw_sp_port_get_offload_stats(int attr_id, const struct net_device *dev,
					   void *sp)
{
	switch (attr_id) {
	case IFLA_OFFLOAD_XSTATS_CPU_HIT:
		return mlxsw_sp_port_get_sw_stats64(dev, sp);
	}

	return -EINVAL;
}

static int mlxsw_sp_port_get_stats_raw(struct net_device *dev, int grp,
				       int prio, char *ppcnt_pl)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;

	mlxsw_reg_ppcnt_pack(ppcnt_pl, mlxsw_sp_port->local_port, grp, prio);
	return mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(ppcnt), ppcnt_pl);
}

static int mlxsw_sp_port_get_hw_stats(struct net_device *dev,
				      struct rtnl_link_stats64 *stats)
{
	char ppcnt_pl[MLXSW_REG_PPCNT_LEN];
	int err;

	err = mlxsw_sp_port_get_stats_raw(dev, MLXSW_REG_PPCNT_IEEE_8023_CNT,
					  0, ppcnt_pl);
	if (err)
		goto out;

	stats->tx_packets =
		mlxsw_reg_ppcnt_a_frames_transmitted_ok_get(ppcnt_pl);
	stats->rx_packets =
		mlxsw_reg_ppcnt_a_frames_received_ok_get(ppcnt_pl);
	stats->tx_bytes =
		mlxsw_reg_ppcnt_a_octets_transmitted_ok_get(ppcnt_pl);
	stats->rx_bytes =
		mlxsw_reg_ppcnt_a_octets_received_ok_get(ppcnt_pl);
	stats->multicast =
		mlxsw_reg_ppcnt_a_multicast_frames_received_ok_get(ppcnt_pl);

	stats->rx_crc_errors =
		mlxsw_reg_ppcnt_a_frame_check_sequence_errors_get(ppcnt_pl);
	stats->rx_frame_errors =
		mlxsw_reg_ppcnt_a_alignment_errors_get(ppcnt_pl);

	stats->rx_length_errors = (
		mlxsw_reg_ppcnt_a_in_range_length_errors_get(ppcnt_pl) +
		mlxsw_reg_ppcnt_a_out_of_range_length_field_get(ppcnt_pl) +
		mlxsw_reg_ppcnt_a_frame_too_long_errors_get(ppcnt_pl));

	stats->rx_errors = (stats->rx_crc_errors +
		stats->rx_frame_errors + stats->rx_length_errors);

out:
	return err;
}

static void
mlxsw_sp_port_get_hw_xstats(struct net_device *dev,
			    struct mlxsw_sp_port_xstats *xstats)
{
	char ppcnt_pl[MLXSW_REG_PPCNT_LEN];
	int err, i;

	err = mlxsw_sp_port_get_stats_raw(dev, MLXSW_REG_PPCNT_EXT_CNT, 0,
					  ppcnt_pl);
	if (!err)
		xstats->ecn = mlxsw_reg_ppcnt_ecn_marked_get(ppcnt_pl);

	for (i = 0; i < TC_MAX_QUEUE; i++) {
		err = mlxsw_sp_port_get_stats_raw(dev,
						  MLXSW_REG_PPCNT_TC_CONG_TC,
						  i, ppcnt_pl);
		if (!err)
			xstats->wred_drop[i] =
				mlxsw_reg_ppcnt_wred_discard_get(ppcnt_pl);

		err = mlxsw_sp_port_get_stats_raw(dev, MLXSW_REG_PPCNT_TC_CNT,
						  i, ppcnt_pl);
		if (err)
			continue;

		xstats->backlog[i] =
			mlxsw_reg_ppcnt_tc_transmit_queue_get(ppcnt_pl);
		xstats->tail_drop[i] =
			mlxsw_reg_ppcnt_tc_no_buffer_discard_uc_get(ppcnt_pl);
	}
}

static void update_stats_cache(struct work_struct *work)
{
	struct mlxsw_sp_port *mlxsw_sp_port =
		container_of(work, struct mlxsw_sp_port,
			     periodic_hw_stats.update_dw.work);

	if (!netif_carrier_ok(mlxsw_sp_port->dev))
		goto out;

	mlxsw_sp_port_get_hw_stats(mlxsw_sp_port->dev,
				   &mlxsw_sp_port->periodic_hw_stats.stats);
	mlxsw_sp_port_get_hw_xstats(mlxsw_sp_port->dev,
				    &mlxsw_sp_port->periodic_hw_stats.xstats);

out:
	mlxsw_core_schedule_dw(&mlxsw_sp_port->periodic_hw_stats.update_dw,
			       MLXSW_HW_STATS_UPDATE_TIME);
}

/* Return the stats from a cache that is updated periodically,
 * as this function might get called in an atomic context.
 */
static void
mlxsw_sp_port_get_stats64(struct net_device *dev,
			  struct rtnl_link_stats64 *stats)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	memcpy(stats, &mlxsw_sp_port->periodic_hw_stats.stats, sizeof(*stats));
}

static int __mlxsw_sp_port_vlan_set(struct mlxsw_sp_port *mlxsw_sp_port,
				    u16 vid_begin, u16 vid_end,
				    bool is_member, bool untagged)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char *spvm_pl;
	int err;

	spvm_pl = kmalloc(MLXSW_REG_SPVM_LEN, GFP_KERNEL);
	if (!spvm_pl)
		return -ENOMEM;

	mlxsw_reg_spvm_pack(spvm_pl, mlxsw_sp_port->local_port,	vid_begin,
			    vid_end, is_member, untagged);
	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spvm), spvm_pl);
	kfree(spvm_pl);
	return err;
}

int mlxsw_sp_port_vlan_set(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid_begin,
			   u16 vid_end, bool is_member, bool untagged)
{
	u16 vid, vid_e;
	int err;

	for (vid = vid_begin; vid <= vid_end;
	     vid += MLXSW_REG_SPVM_REC_MAX_COUNT) {
		vid_e = min((u16) (vid + MLXSW_REG_SPVM_REC_MAX_COUNT - 1),
			    vid_end);

		err = __mlxsw_sp_port_vlan_set(mlxsw_sp_port, vid, vid_e,
					       is_member, untagged);
		if (err)
			return err;
	}

	return 0;
}

static void mlxsw_sp_port_vlan_flush(struct mlxsw_sp_port *mlxsw_sp_port)
{
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan, *tmp;

	list_for_each_entry_safe(mlxsw_sp_port_vlan, tmp,
				 &mlxsw_sp_port->vlans_list, list)
		mlxsw_sp_port_vlan_put(mlxsw_sp_port_vlan);
}

static struct mlxsw_sp_port_vlan *
mlxsw_sp_port_vlan_create(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid)
{
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan;
	bool untagged = vid == 1;
	int err;

	err = mlxsw_sp_port_vlan_set(mlxsw_sp_port, vid, vid, true, untagged);
	if (err)
		return ERR_PTR(err);

	mlxsw_sp_port_vlan = kzalloc(sizeof(*mlxsw_sp_port_vlan), GFP_KERNEL);
	if (!mlxsw_sp_port_vlan) {
		err = -ENOMEM;
		goto err_port_vlan_alloc;
	}

	mlxsw_sp_port_vlan->mlxsw_sp_port = mlxsw_sp_port;
	mlxsw_sp_port_vlan->vid = vid;
	list_add(&mlxsw_sp_port_vlan->list, &mlxsw_sp_port->vlans_list);

	return mlxsw_sp_port_vlan;

err_port_vlan_alloc:
	mlxsw_sp_port_vlan_set(mlxsw_sp_port, vid, vid, false, false);
	return ERR_PTR(err);
}

static void
mlxsw_sp_port_vlan_destroy(struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan)
{
	struct mlxsw_sp_port *mlxsw_sp_port = mlxsw_sp_port_vlan->mlxsw_sp_port;
	u16 vid = mlxsw_sp_port_vlan->vid;

	list_del(&mlxsw_sp_port_vlan->list);
	kfree(mlxsw_sp_port_vlan);
	mlxsw_sp_port_vlan_set(mlxsw_sp_port, vid, vid, false, false);
}

struct mlxsw_sp_port_vlan *
mlxsw_sp_port_vlan_get(struct mlxsw_sp_port *mlxsw_sp_port, u16 vid)
{
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan;

	mlxsw_sp_port_vlan = mlxsw_sp_port_vlan_find_by_vid(mlxsw_sp_port, vid);
	if (mlxsw_sp_port_vlan)
		return mlxsw_sp_port_vlan;

	return mlxsw_sp_port_vlan_create(mlxsw_sp_port, vid);
}

void mlxsw_sp_port_vlan_put(struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan)
{
	struct mlxsw_sp_fid *fid = mlxsw_sp_port_vlan->fid;

	if (mlxsw_sp_port_vlan->bridge_port)
		mlxsw_sp_port_vlan_bridge_leave(mlxsw_sp_port_vlan);
	else if (fid)
		mlxsw_sp_port_vlan_router_leave(mlxsw_sp_port_vlan);

	mlxsw_sp_port_vlan_destroy(mlxsw_sp_port_vlan);
}

static int mlxsw_sp_port_add_vid(struct net_device *dev,
				 __be16 __always_unused proto, u16 vid)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	/* VLAN 0 is added to HW filter when device goes up, but it is
	 * reserved in our case, so simply return.
	 */
	if (!vid)
		return 0;

	return PTR_ERR_OR_ZERO(mlxsw_sp_port_vlan_get(mlxsw_sp_port, vid));
}

static int mlxsw_sp_port_kill_vid(struct net_device *dev,
				  __be16 __always_unused proto, u16 vid)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan;

	/* VLAN 0 is removed from HW filter when device goes down, but
	 * it is reserved in our case, so simply return.
	 */
	if (!vid)
		return 0;

	mlxsw_sp_port_vlan = mlxsw_sp_port_vlan_find_by_vid(mlxsw_sp_port, vid);
	if (!mlxsw_sp_port_vlan)
		return 0;
	mlxsw_sp_port_vlan_put(mlxsw_sp_port_vlan);

	return 0;
}

static int mlxsw_sp_port_get_phys_port_name(struct net_device *dev, char *name,
					    size_t len)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	u8 module = mlxsw_sp_port->mapping.module;
	u8 width = mlxsw_sp_port->mapping.width;
	u8 lane = mlxsw_sp_port->mapping.lane;
	int err;

	if (!mlxsw_sp_port->split)
		err = snprintf(name, len, "p%d", module + 1);
	else
		err = snprintf(name, len, "p%ds%d", module + 1,
			       lane / width);

	if (err >= len)
		return -EINVAL;

	return 0;
}

static struct mlxsw_sp_port_mall_tc_entry *
mlxsw_sp_port_mall_tc_entry_find(struct mlxsw_sp_port *port,
				 unsigned long cookie) {
	struct mlxsw_sp_port_mall_tc_entry *mall_tc_entry;

	list_for_each_entry(mall_tc_entry, &port->mall_tc_list, list)
		if (mall_tc_entry->cookie == cookie)
			return mall_tc_entry;

	return NULL;
}

static int
mlxsw_sp_port_add_cls_matchall_mirror(struct mlxsw_sp_port *mlxsw_sp_port,
				      struct mlxsw_sp_port_mall_mirror_tc_entry *mirror,
				      const struct tc_action *a,
				      bool ingress)
{
	enum mlxsw_sp_span_type span_type;
	struct mlxsw_sp_port *to_port;
	struct net_device *to_dev;

	to_dev = tcf_mirred_dev(a);
	if (!to_dev) {
		netdev_err(mlxsw_sp_port->dev, "Could not find requested device\n");
		return -EINVAL;
	}

	if (!mlxsw_sp_port_dev_check(to_dev)) {
		netdev_err(mlxsw_sp_port->dev, "Cannot mirror to a non-spectrum port");
		return -EOPNOTSUPP;
	}
	to_port = netdev_priv(to_dev);

	mirror->to_local_port = to_port->local_port;
	mirror->ingress = ingress;
	span_type = ingress ? MLXSW_SP_SPAN_INGRESS : MLXSW_SP_SPAN_EGRESS;
	return mlxsw_sp_span_mirror_add(mlxsw_sp_port, to_port, span_type,
					true);
}

static void
mlxsw_sp_port_del_cls_matchall_mirror(struct mlxsw_sp_port *mlxsw_sp_port,
				      struct mlxsw_sp_port_mall_mirror_tc_entry *mirror)
{
	enum mlxsw_sp_span_type span_type;

	span_type = mirror->ingress ?
			MLXSW_SP_SPAN_INGRESS : MLXSW_SP_SPAN_EGRESS;
	mlxsw_sp_span_mirror_del(mlxsw_sp_port, mirror->to_local_port,
				 span_type, true);
}

static int
mlxsw_sp_port_add_cls_matchall_sample(struct mlxsw_sp_port *mlxsw_sp_port,
				      struct tc_cls_matchall_offload *cls,
				      const struct tc_action *a,
				      bool ingress)
{
	int err;

	if (!mlxsw_sp_port->sample)
		return -EOPNOTSUPP;
	if (rtnl_dereference(mlxsw_sp_port->sample->psample_group)) {
		netdev_err(mlxsw_sp_port->dev, "sample already active\n");
		return -EEXIST;
	}
	if (tcf_sample_rate(a) > MLXSW_REG_MPSC_RATE_MAX) {
		netdev_err(mlxsw_sp_port->dev, "sample rate not supported\n");
		return -EOPNOTSUPP;
	}

	rcu_assign_pointer(mlxsw_sp_port->sample->psample_group,
			   tcf_sample_psample_group(a));
	mlxsw_sp_port->sample->truncate = tcf_sample_truncate(a);
	mlxsw_sp_port->sample->trunc_size = tcf_sample_trunc_size(a);
	mlxsw_sp_port->sample->rate = tcf_sample_rate(a);

	err = mlxsw_sp_port_sample_set(mlxsw_sp_port, true, tcf_sample_rate(a));
	if (err)
		goto err_port_sample_set;
	return 0;

err_port_sample_set:
	RCU_INIT_POINTER(mlxsw_sp_port->sample->psample_group, NULL);
	return err;
}

static void
mlxsw_sp_port_del_cls_matchall_sample(struct mlxsw_sp_port *mlxsw_sp_port)
{
	if (!mlxsw_sp_port->sample)
		return;

	mlxsw_sp_port_sample_set(mlxsw_sp_port, false, 1);
	RCU_INIT_POINTER(mlxsw_sp_port->sample->psample_group, NULL);
}

static int mlxsw_sp_port_add_cls_matchall(struct mlxsw_sp_port *mlxsw_sp_port,
					  struct tc_cls_matchall_offload *f,
					  bool ingress)
{
	struct mlxsw_sp_port_mall_tc_entry *mall_tc_entry;
	__be16 protocol = f->common.protocol;
	const struct tc_action *a;
	LIST_HEAD(actions);
	int err;

	if (!tcf_exts_has_one_action(f->exts)) {
		netdev_err(mlxsw_sp_port->dev, "only singular actions are supported\n");
		return -EOPNOTSUPP;
	}

	mall_tc_entry = kzalloc(sizeof(*mall_tc_entry), GFP_KERNEL);
	if (!mall_tc_entry)
		return -ENOMEM;
	mall_tc_entry->cookie = f->cookie;

	tcf_exts_to_list(f->exts, &actions);
	a = list_first_entry(&actions, struct tc_action, list);

	if (is_tcf_mirred_egress_mirror(a) && protocol == htons(ETH_P_ALL)) {
		struct mlxsw_sp_port_mall_mirror_tc_entry *mirror;

		mall_tc_entry->type = MLXSW_SP_PORT_MALL_MIRROR;
		mirror = &mall_tc_entry->mirror;
		err = mlxsw_sp_port_add_cls_matchall_mirror(mlxsw_sp_port,
							    mirror, a, ingress);
	} else if (is_tcf_sample(a) && protocol == htons(ETH_P_ALL)) {
		mall_tc_entry->type = MLXSW_SP_PORT_MALL_SAMPLE;
		err = mlxsw_sp_port_add_cls_matchall_sample(mlxsw_sp_port, f,
							    a, ingress);
	} else {
		err = -EOPNOTSUPP;
	}

	if (err)
		goto err_add_action;

	list_add_tail(&mall_tc_entry->list, &mlxsw_sp_port->mall_tc_list);
	return 0;

err_add_action:
	kfree(mall_tc_entry);
	return err;
}

static void mlxsw_sp_port_del_cls_matchall(struct mlxsw_sp_port *mlxsw_sp_port,
					   struct tc_cls_matchall_offload *f)
{
	struct mlxsw_sp_port_mall_tc_entry *mall_tc_entry;

	mall_tc_entry = mlxsw_sp_port_mall_tc_entry_find(mlxsw_sp_port,
							 f->cookie);
	if (!mall_tc_entry) {
		netdev_dbg(mlxsw_sp_port->dev, "tc entry not found on port\n");
		return;
	}
	list_del(&mall_tc_entry->list);

	switch (mall_tc_entry->type) {
	case MLXSW_SP_PORT_MALL_MIRROR:
		mlxsw_sp_port_del_cls_matchall_mirror(mlxsw_sp_port,
						      &mall_tc_entry->mirror);
		break;
	case MLXSW_SP_PORT_MALL_SAMPLE:
		mlxsw_sp_port_del_cls_matchall_sample(mlxsw_sp_port);
		break;
	default:
		WARN_ON(1);
	}

	kfree(mall_tc_entry);
}

static int mlxsw_sp_setup_tc_cls_matchall(struct mlxsw_sp_port *mlxsw_sp_port,
					  struct tc_cls_matchall_offload *f,
					  bool ingress)
{
	switch (f->command) {
	case TC_CLSMATCHALL_REPLACE:
		return mlxsw_sp_port_add_cls_matchall(mlxsw_sp_port, f,
						      ingress);
	case TC_CLSMATCHALL_DESTROY:
		mlxsw_sp_port_del_cls_matchall(mlxsw_sp_port, f);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int
mlxsw_sp_setup_tc_cls_flower(struct mlxsw_sp_acl_block *acl_block,
			     struct tc_cls_flower_offload *f)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_acl_block_mlxsw_sp(acl_block);

	switch (f->command) {
	case TC_CLSFLOWER_REPLACE:
		return mlxsw_sp_flower_replace(mlxsw_sp, acl_block, f);
	case TC_CLSFLOWER_DESTROY:
		mlxsw_sp_flower_destroy(mlxsw_sp, acl_block, f);
		return 0;
	case TC_CLSFLOWER_STATS:
		return mlxsw_sp_flower_stats(mlxsw_sp, acl_block, f);
	default:
		return -EOPNOTSUPP;
	}
}

static int mlxsw_sp_setup_tc_block_cb_matchall(enum tc_setup_type type,
					       void *type_data,
					       void *cb_priv, bool ingress)
{
	struct mlxsw_sp_port *mlxsw_sp_port = cb_priv;

	switch (type) {
	case TC_SETUP_CLSMATCHALL:
		if (!tc_cls_can_offload_and_chain0(mlxsw_sp_port->dev,
						   type_data))
			return -EOPNOTSUPP;

		return mlxsw_sp_setup_tc_cls_matchall(mlxsw_sp_port, type_data,
						      ingress);
	case TC_SETUP_CLSFLOWER:
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int mlxsw_sp_setup_tc_block_cb_matchall_ig(enum tc_setup_type type,
						  void *type_data,
						  void *cb_priv)
{
	return mlxsw_sp_setup_tc_block_cb_matchall(type, type_data,
						   cb_priv, true);
}

static int mlxsw_sp_setup_tc_block_cb_matchall_eg(enum tc_setup_type type,
						  void *type_data,
						  void *cb_priv)
{
	return mlxsw_sp_setup_tc_block_cb_matchall(type, type_data,
						   cb_priv, false);
}

static int mlxsw_sp_setup_tc_block_cb_flower(enum tc_setup_type type,
					     void *type_data, void *cb_priv)
{
	struct mlxsw_sp_acl_block *acl_block = cb_priv;

	switch (type) {
	case TC_SETUP_CLSMATCHALL:
		return 0;
	case TC_SETUP_CLSFLOWER:
		if (mlxsw_sp_acl_block_disabled(acl_block))
			return -EOPNOTSUPP;

		return mlxsw_sp_setup_tc_cls_flower(acl_block, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static int
mlxsw_sp_setup_tc_block_flower_bind(struct mlxsw_sp_port *mlxsw_sp_port,
				    struct tcf_block *block, bool ingress)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct mlxsw_sp_acl_block *acl_block;
	struct tcf_block_cb *block_cb;
	int err;

	block_cb = tcf_block_cb_lookup(block, mlxsw_sp_setup_tc_block_cb_flower,
				       mlxsw_sp);
	if (!block_cb) {
		acl_block = mlxsw_sp_acl_block_create(mlxsw_sp, block->net);
		if (!acl_block)
			return -ENOMEM;
		block_cb = __tcf_block_cb_register(block,
						   mlxsw_sp_setup_tc_block_cb_flower,
						   mlxsw_sp, acl_block);
		if (IS_ERR(block_cb)) {
			err = PTR_ERR(block_cb);
			goto err_cb_register;
		}
	} else {
		acl_block = tcf_block_cb_priv(block_cb);
	}
	tcf_block_cb_incref(block_cb);
	err = mlxsw_sp_acl_block_bind(mlxsw_sp, acl_block,
				      mlxsw_sp_port, ingress);
	if (err)
		goto err_block_bind;

	if (ingress)
		mlxsw_sp_port->ing_acl_block = acl_block;
	else
		mlxsw_sp_port->eg_acl_block = acl_block;

	return 0;

err_block_bind:
	if (!tcf_block_cb_decref(block_cb)) {
		__tcf_block_cb_unregister(block_cb);
err_cb_register:
		mlxsw_sp_acl_block_destroy(acl_block);
	}
	return err;
}

static void
mlxsw_sp_setup_tc_block_flower_unbind(struct mlxsw_sp_port *mlxsw_sp_port,
				      struct tcf_block *block, bool ingress)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct mlxsw_sp_acl_block *acl_block;
	struct tcf_block_cb *block_cb;
	int err;

	block_cb = tcf_block_cb_lookup(block, mlxsw_sp_setup_tc_block_cb_flower,
				       mlxsw_sp);
	if (!block_cb)
		return;

	if (ingress)
		mlxsw_sp_port->ing_acl_block = NULL;
	else
		mlxsw_sp_port->eg_acl_block = NULL;

	acl_block = tcf_block_cb_priv(block_cb);
	err = mlxsw_sp_acl_block_unbind(mlxsw_sp, acl_block,
					mlxsw_sp_port, ingress);
	if (!err && !tcf_block_cb_decref(block_cb)) {
		__tcf_block_cb_unregister(block_cb);
		mlxsw_sp_acl_block_destroy(acl_block);
	}
}

static int mlxsw_sp_setup_tc_block(struct mlxsw_sp_port *mlxsw_sp_port,
				   struct tc_block_offload *f)
{
	tc_setup_cb_t *cb;
	bool ingress;
	int err;

	if (f->binder_type == TCF_BLOCK_BINDER_TYPE_CLSACT_INGRESS) {
		cb = mlxsw_sp_setup_tc_block_cb_matchall_ig;
		ingress = true;
	} else if (f->binder_type == TCF_BLOCK_BINDER_TYPE_CLSACT_EGRESS) {
		cb = mlxsw_sp_setup_tc_block_cb_matchall_eg;
		ingress = false;
	} else {
		return -EOPNOTSUPP;
	}

	switch (f->command) {
	case TC_BLOCK_BIND:
		err = tcf_block_cb_register(f->block, cb, mlxsw_sp_port,
					    mlxsw_sp_port);
		if (err)
			return err;
		err = mlxsw_sp_setup_tc_block_flower_bind(mlxsw_sp_port,
							  f->block, ingress);
		if (err) {
			tcf_block_cb_unregister(f->block, cb, mlxsw_sp_port);
			return err;
		}
		return 0;
	case TC_BLOCK_UNBIND:
		mlxsw_sp_setup_tc_block_flower_unbind(mlxsw_sp_port,
						      f->block, ingress);
		tcf_block_cb_unregister(f->block, cb, mlxsw_sp_port);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int mlxsw_sp_setup_tc(struct net_device *dev, enum tc_setup_type type,
			     void *type_data)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	switch (type) {
	case TC_SETUP_BLOCK:
		return mlxsw_sp_setup_tc_block(mlxsw_sp_port, type_data);
	case TC_SETUP_QDISC_RED:
		return mlxsw_sp_setup_tc_red(mlxsw_sp_port, type_data);
	case TC_SETUP_QDISC_PRIO:
		return mlxsw_sp_setup_tc_prio(mlxsw_sp_port, type_data);
	default:
		return -EOPNOTSUPP;
	}
}


static int mlxsw_sp_feature_hw_tc(struct net_device *dev, bool enable)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	if (!enable) {
		if (mlxsw_sp_acl_block_rule_count(mlxsw_sp_port->ing_acl_block) ||
		    mlxsw_sp_acl_block_rule_count(mlxsw_sp_port->eg_acl_block) ||
		    !list_empty(&mlxsw_sp_port->mall_tc_list)) {
			netdev_err(dev, "Active offloaded tc filters, can't turn hw_tc_offload off\n");
			return -EINVAL;
		}
		mlxsw_sp_acl_block_disable_inc(mlxsw_sp_port->ing_acl_block);
		mlxsw_sp_acl_block_disable_inc(mlxsw_sp_port->eg_acl_block);
	} else {
		mlxsw_sp_acl_block_disable_dec(mlxsw_sp_port->ing_acl_block);
		mlxsw_sp_acl_block_disable_dec(mlxsw_sp_port->eg_acl_block);
	}
	return 0;
}

typedef int (*mlxsw_sp_feature_handler)(struct net_device *dev, bool enable);

static int mlxsw_sp_handle_feature(struct net_device *dev,
				   netdev_features_t wanted_features,
				   netdev_features_t feature,
				   mlxsw_sp_feature_handler feature_handler)
{
	netdev_features_t changes = wanted_features ^ dev->features;
	bool enable = !!(wanted_features & feature);
	int err;

	if (!(changes & feature))
		return 0;

	err = feature_handler(dev, enable);
	if (err) {
		netdev_err(dev, "%s feature %pNF failed, err %d\n",
			   enable ? "Enable" : "Disable", &feature, err);
		return err;
	}

	if (enable)
		dev->features |= feature;
	else
		dev->features &= ~feature;

	return 0;
}
static int mlxsw_sp_set_features(struct net_device *dev,
				 netdev_features_t features)
{
	return mlxsw_sp_handle_feature(dev, features, NETIF_F_HW_TC,
				       mlxsw_sp_feature_hw_tc);
}

static const struct net_device_ops mlxsw_sp_port_netdev_ops = {
	.ndo_open		= mlxsw_sp_port_open,
	.ndo_stop		= mlxsw_sp_port_stop,
	.ndo_start_xmit		= mlxsw_sp_port_xmit,
	.ndo_setup_tc           = mlxsw_sp_setup_tc,
	.ndo_set_rx_mode	= mlxsw_sp_set_rx_mode,
	.ndo_set_mac_address	= mlxsw_sp_port_set_mac_address,
	.ndo_change_mtu		= mlxsw_sp_port_change_mtu,
	.ndo_get_stats64	= mlxsw_sp_port_get_stats64,
	.ndo_has_offload_stats	= mlxsw_sp_port_has_offload_stats,
	.ndo_get_offload_stats	= mlxsw_sp_port_get_offload_stats,
	.ndo_vlan_rx_add_vid	= mlxsw_sp_port_add_vid,
	.ndo_vlan_rx_kill_vid	= mlxsw_sp_port_kill_vid,
	.ndo_get_phys_port_name	= mlxsw_sp_port_get_phys_port_name,
	.ndo_set_features	= mlxsw_sp_set_features,
};

static void mlxsw_sp_port_get_drvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *drvinfo)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;

	strlcpy(drvinfo->driver, mlxsw_sp_driver_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, mlxsw_sp_driver_version,
		sizeof(drvinfo->version));
	snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
		 "%d.%d.%d",
		 mlxsw_sp->bus_info->fw_rev.major,
		 mlxsw_sp->bus_info->fw_rev.minor,
		 mlxsw_sp->bus_info->fw_rev.subminor);
	strlcpy(drvinfo->bus_info, mlxsw_sp->bus_info->device_name,
		sizeof(drvinfo->bus_info));
}

static void mlxsw_sp_port_get_pauseparam(struct net_device *dev,
					 struct ethtool_pauseparam *pause)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);

	pause->rx_pause = mlxsw_sp_port->link.rx_pause;
	pause->tx_pause = mlxsw_sp_port->link.tx_pause;
}

static int mlxsw_sp_port_pause_set(struct mlxsw_sp_port *mlxsw_sp_port,
				   struct ethtool_pauseparam *pause)
{
	char pfcc_pl[MLXSW_REG_PFCC_LEN];

	mlxsw_reg_pfcc_pack(pfcc_pl, mlxsw_sp_port->local_port);
	mlxsw_reg_pfcc_pprx_set(pfcc_pl, pause->rx_pause);
	mlxsw_reg_pfcc_pptx_set(pfcc_pl, pause->tx_pause);

	return mlxsw_reg_write(mlxsw_sp_port->mlxsw_sp->core, MLXSW_REG(pfcc),
			       pfcc_pl);
}

static int mlxsw_sp_port_set_pauseparam(struct net_device *dev,
					struct ethtool_pauseparam *pause)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	bool pause_en = pause->tx_pause || pause->rx_pause;
	int err;

	if (mlxsw_sp_port->dcb.pfc && mlxsw_sp_port->dcb.pfc->pfc_en) {
		netdev_err(dev, "PFC already enabled on port\n");
		return -EINVAL;
	}

	if (pause->autoneg) {
		netdev_err(dev, "PAUSE frames autonegotiation isn't supported\n");
		return -EINVAL;
	}

	err = mlxsw_sp_port_headroom_set(mlxsw_sp_port, dev->mtu, pause_en);
	if (err) {
		netdev_err(dev, "Failed to configure port's headroom\n");
		return err;
	}

	err = mlxsw_sp_port_pause_set(mlxsw_sp_port, pause);
	if (err) {
		netdev_err(dev, "Failed to set PAUSE parameters\n");
		goto err_port_pause_configure;
	}

	mlxsw_sp_port->link.rx_pause = pause->rx_pause;
	mlxsw_sp_port->link.tx_pause = pause->tx_pause;

	return 0;

err_port_pause_configure:
	pause_en = mlxsw_sp_port_is_pause_en(mlxsw_sp_port);
	mlxsw_sp_port_headroom_set(mlxsw_sp_port, dev->mtu, pause_en);
	return err;
}

struct mlxsw_sp_port_hw_stats {
	char str[ETH_GSTRING_LEN];
	u64 (*getter)(const char *payload);
	bool cells_bytes;
};

static struct mlxsw_sp_port_hw_stats mlxsw_sp_port_hw_stats[] = {
	{
		.str = "a_frames_transmitted_ok",
		.getter = mlxsw_reg_ppcnt_a_frames_transmitted_ok_get,
	},
	{
		.str = "a_frames_received_ok",
		.getter = mlxsw_reg_ppcnt_a_frames_received_ok_get,
	},
	{
		.str = "a_frame_check_sequence_errors",
		.getter = mlxsw_reg_ppcnt_a_frame_check_sequence_errors_get,
	},
	{
		.str = "a_alignment_errors",
		.getter = mlxsw_reg_ppcnt_a_alignment_errors_get,
	},
	{
		.str = "a_octets_transmitted_ok",
		.getter = mlxsw_reg_ppcnt_a_octets_transmitted_ok_get,
	},
	{
		.str = "a_octets_received_ok",
		.getter = mlxsw_reg_ppcnt_a_octets_received_ok_get,
	},
	{
		.str = "a_multicast_frames_xmitted_ok",
		.getter = mlxsw_reg_ppcnt_a_multicast_frames_xmitted_ok_get,
	},
	{
		.str = "a_broadcast_frames_xmitted_ok",
		.getter = mlxsw_reg_ppcnt_a_broadcast_frames_xmitted_ok_get,
	},
	{
		.str = "a_multicast_frames_received_ok",
		.getter = mlxsw_reg_ppcnt_a_multicast_frames_received_ok_get,
	},
	{
		.str = "a_broadcast_frames_received_ok",
		.getter = mlxsw_reg_ppcnt_a_broadcast_frames_received_ok_get,
	},
	{
		.str = "a_in_range_length_errors",
		.getter = mlxsw_reg_ppcnt_a_in_range_length_errors_get,
	},
	{
		.str = "a_out_of_range_length_field",
		.getter = mlxsw_reg_ppcnt_a_out_of_range_length_field_get,
	},
	{
		.str = "a_frame_too_long_errors",
		.getter = mlxsw_reg_ppcnt_a_frame_too_long_errors_get,
	},
	{
		.str = "a_symbol_error_during_carrier",
		.getter = mlxsw_reg_ppcnt_a_symbol_error_during_carrier_get,
	},
	{
		.str = "a_mac_control_frames_transmitted",
		.getter = mlxsw_reg_ppcnt_a_mac_control_frames_transmitted_get,
	},
	{
		.str = "a_mac_control_frames_received",
		.getter = mlxsw_reg_ppcnt_a_mac_control_frames_received_get,
	},
	{
		.str = "a_unsupported_opcodes_received",
		.getter = mlxsw_reg_ppcnt_a_unsupported_opcodes_received_get,
	},
	{
		.str = "a_pause_mac_ctrl_frames_received",
		.getter = mlxsw_reg_ppcnt_a_pause_mac_ctrl_frames_received_get,
	},
	{
		.str = "a_pause_mac_ctrl_frames_xmitted",
		.getter = mlxsw_reg_ppcnt_a_pause_mac_ctrl_frames_transmitted_get,
	},
};

#define MLXSW_SP_PORT_HW_STATS_LEN ARRAY_SIZE(mlxsw_sp_port_hw_stats)

static struct mlxsw_sp_port_hw_stats mlxsw_sp_port_hw_prio_stats[] = {
	{
		.str = "rx_octets_prio",
		.getter = mlxsw_reg_ppcnt_rx_octets_get,
	},
	{
		.str = "rx_frames_prio",
		.getter = mlxsw_reg_ppcnt_rx_frames_get,
	},
	{
		.str = "tx_octets_prio",
		.getter = mlxsw_reg_ppcnt_tx_octets_get,
	},
	{
		.str = "tx_frames_prio",
		.getter = mlxsw_reg_ppcnt_tx_frames_get,
	},
	{
		.str = "rx_pause_prio",
		.getter = mlxsw_reg_ppcnt_rx_pause_get,
	},
	{
		.str = "rx_pause_duration_prio",
		.getter = mlxsw_reg_ppcnt_rx_pause_duration_get,
	},
	{
		.str = "tx_pause_prio",
		.getter = mlxsw_reg_ppcnt_tx_pause_get,
	},
	{
		.str = "tx_pause_duration_prio",
		.getter = mlxsw_reg_ppcnt_tx_pause_duration_get,
	},
};

#define MLXSW_SP_PORT_HW_PRIO_STATS_LEN ARRAY_SIZE(mlxsw_sp_port_hw_prio_stats)

static struct mlxsw_sp_port_hw_stats mlxsw_sp_port_hw_tc_stats[] = {
	{
		.str = "tc_transmit_queue_tc",
		.getter = mlxsw_reg_ppcnt_tc_transmit_queue_get,
		.cells_bytes = true,
	},
	{
		.str = "tc_no_buffer_discard_uc_tc",
		.getter = mlxsw_reg_ppcnt_tc_no_buffer_discard_uc_get,
	},
};

#define MLXSW_SP_PORT_HW_TC_STATS_LEN ARRAY_SIZE(mlxsw_sp_port_hw_tc_stats)

#define MLXSW_SP_PORT_ETHTOOL_STATS_LEN (MLXSW_SP_PORT_HW_STATS_LEN + \
					 (MLXSW_SP_PORT_HW_PRIO_STATS_LEN + \
					  MLXSW_SP_PORT_HW_TC_STATS_LEN) * \
					 IEEE_8021QAZ_MAX_TCS)

static void mlxsw_sp_port_get_prio_strings(u8 **p, int prio)
{
	int i;

	for (i = 0; i < MLXSW_SP_PORT_HW_PRIO_STATS_LEN; i++) {
		snprintf(*p, ETH_GSTRING_LEN, "%s_%d",
			 mlxsw_sp_port_hw_prio_stats[i].str, prio);
		*p += ETH_GSTRING_LEN;
	}
}

static void mlxsw_sp_port_get_tc_strings(u8 **p, int tc)
{
	int i;

	for (i = 0; i < MLXSW_SP_PORT_HW_TC_STATS_LEN; i++) {
		snprintf(*p, ETH_GSTRING_LEN, "%s_%d",
			 mlxsw_sp_port_hw_tc_stats[i].str, tc);
		*p += ETH_GSTRING_LEN;
	}
}

static void mlxsw_sp_port_get_strings(struct net_device *dev,
				      u32 stringset, u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < MLXSW_SP_PORT_HW_STATS_LEN; i++) {
			memcpy(p, mlxsw_sp_port_hw_stats[i].str,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}

		for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++)
			mlxsw_sp_port_get_prio_strings(&p, i);

		for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++)
			mlxsw_sp_port_get_tc_strings(&p, i);

		break;
	}
}

static int mlxsw_sp_port_set_phys_id(struct net_device *dev,
				     enum ethtool_phys_id_state state)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char mlcr_pl[MLXSW_REG_MLCR_LEN];
	bool active;

	switch (state) {
	case ETHTOOL_ID_ACTIVE:
		active = true;
		break;
	case ETHTOOL_ID_INACTIVE:
		active = false;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mlxsw_reg_mlcr_pack(mlcr_pl, mlxsw_sp_port->local_port, active);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(mlcr), mlcr_pl);
}

static int
mlxsw_sp_get_hw_stats_by_group(struct mlxsw_sp_port_hw_stats **p_hw_stats,
			       int *p_len, enum mlxsw_reg_ppcnt_grp grp)
{
	switch (grp) {
	case  MLXSW_REG_PPCNT_IEEE_8023_CNT:
		*p_hw_stats = mlxsw_sp_port_hw_stats;
		*p_len = MLXSW_SP_PORT_HW_STATS_LEN;
		break;
	case MLXSW_REG_PPCNT_PRIO_CNT:
		*p_hw_stats = mlxsw_sp_port_hw_prio_stats;
		*p_len = MLXSW_SP_PORT_HW_PRIO_STATS_LEN;
		break;
	case MLXSW_REG_PPCNT_TC_CNT:
		*p_hw_stats = mlxsw_sp_port_hw_tc_stats;
		*p_len = MLXSW_SP_PORT_HW_TC_STATS_LEN;
		break;
	default:
		WARN_ON(1);
		return -EOPNOTSUPP;
	}
	return 0;
}

static void __mlxsw_sp_port_get_stats(struct net_device *dev,
				      enum mlxsw_reg_ppcnt_grp grp, int prio,
				      u64 *data, int data_index)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct mlxsw_sp_port_hw_stats *hw_stats;
	char ppcnt_pl[MLXSW_REG_PPCNT_LEN];
	int i, len;
	int err;

	err = mlxsw_sp_get_hw_stats_by_group(&hw_stats, &len, grp);
	if (err)
		return;
	mlxsw_sp_port_get_stats_raw(dev, grp, prio, ppcnt_pl);
	for (i = 0; i < len; i++) {
		data[data_index + i] = hw_stats[i].getter(ppcnt_pl);
		if (!hw_stats[i].cells_bytes)
			continue;
		data[data_index + i] = mlxsw_sp_cells_bytes(mlxsw_sp,
							    data[data_index + i]);
	}
}

static void mlxsw_sp_port_get_stats(struct net_device *dev,
				    struct ethtool_stats *stats, u64 *data)
{
	int i, data_index = 0;

	/* IEEE 802.3 Counters */
	__mlxsw_sp_port_get_stats(dev, MLXSW_REG_PPCNT_IEEE_8023_CNT, 0,
				  data, data_index);
	data_index = MLXSW_SP_PORT_HW_STATS_LEN;

	/* Per-Priority Counters */
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		__mlxsw_sp_port_get_stats(dev, MLXSW_REG_PPCNT_PRIO_CNT, i,
					  data, data_index);
		data_index += MLXSW_SP_PORT_HW_PRIO_STATS_LEN;
	}

	/* Per-TC Counters */
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		__mlxsw_sp_port_get_stats(dev, MLXSW_REG_PPCNT_TC_CNT, i,
					  data, data_index);
		data_index += MLXSW_SP_PORT_HW_TC_STATS_LEN;
	}
}

static int mlxsw_sp_port_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return MLXSW_SP_PORT_ETHTOOL_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

struct mlxsw_sp_port_link_mode {
	enum ethtool_link_mode_bit_indices mask_ethtool;
	u32 mask;
	u32 speed;
};

static const struct mlxsw_sp_port_link_mode mlxsw_sp_port_link_mode[] = {
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_100BASE_T,
		.mask_ethtool	= ETHTOOL_LINK_MODE_100baseT_Full_BIT,
		.speed		= SPEED_100,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_SGMII |
				  MLXSW_REG_PTYS_ETH_SPEED_1000BASE_KX,
		.mask_ethtool	= ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
		.speed		= SPEED_1000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_10GBASE_T,
		.mask_ethtool	= ETHTOOL_LINK_MODE_10000baseT_Full_BIT,
		.speed		= SPEED_10000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_10GBASE_CX4 |
				  MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KX4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT,
		.speed		= SPEED_10000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KR |
				  MLXSW_REG_PTYS_ETH_SPEED_10GBASE_CR |
				  MLXSW_REG_PTYS_ETH_SPEED_10GBASE_SR |
				  MLXSW_REG_PTYS_ETH_SPEED_10GBASE_ER_LR,
		.mask_ethtool	= ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
		.speed		= SPEED_10000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_20GBASE_KR2,
		.mask_ethtool	= ETHTOOL_LINK_MODE_20000baseKR2_Full_BIT,
		.speed		= SPEED_20000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_40GBASE_CR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT,
		.speed		= SPEED_40000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_40GBASE_KR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT,
		.speed		= SPEED_40000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_40GBASE_SR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_40000baseSR4_Full_BIT,
		.speed		= SPEED_40000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_40GBASE_LR4_ER4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_40000baseLR4_Full_BIT,
		.speed		= SPEED_40000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_25GBASE_CR,
		.mask_ethtool	= ETHTOOL_LINK_MODE_25000baseCR_Full_BIT,
		.speed		= SPEED_25000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_25GBASE_KR,
		.mask_ethtool	= ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
		.speed		= SPEED_25000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_25GBASE_SR,
		.mask_ethtool	= ETHTOOL_LINK_MODE_25000baseSR_Full_BIT,
		.speed		= SPEED_25000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_25GBASE_SR,
		.mask_ethtool	= ETHTOOL_LINK_MODE_25000baseSR_Full_BIT,
		.speed		= SPEED_25000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_50GBASE_CR2,
		.mask_ethtool	= ETHTOOL_LINK_MODE_50000baseCR2_Full_BIT,
		.speed		= SPEED_50000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_50GBASE_KR2,
		.mask_ethtool	= ETHTOOL_LINK_MODE_50000baseKR2_Full_BIT,
		.speed		= SPEED_50000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_50GBASE_SR2,
		.mask_ethtool	= ETHTOOL_LINK_MODE_50000baseSR2_Full_BIT,
		.speed		= SPEED_50000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_56GBASE_R4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_56000baseKR4_Full_BIT,
		.speed		= SPEED_56000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_56GBASE_R4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_56000baseCR4_Full_BIT,
		.speed		= SPEED_56000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_56GBASE_R4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_56000baseSR4_Full_BIT,
		.speed		= SPEED_56000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_56GBASE_R4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_56000baseLR4_Full_BIT,
		.speed		= SPEED_56000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_100GBASE_CR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT,
		.speed		= SPEED_100000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_100GBASE_SR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_100000baseSR4_Full_BIT,
		.speed		= SPEED_100000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_100GBASE_KR4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT,
		.speed		= SPEED_100000,
	},
	{
		.mask		= MLXSW_REG_PTYS_ETH_SPEED_100GBASE_LR4_ER4,
		.mask_ethtool	= ETHTOOL_LINK_MODE_100000baseLR4_ER4_Full_BIT,
		.speed		= SPEED_100000,
	},
};

#define MLXSW_SP_PORT_LINK_MODE_LEN ARRAY_SIZE(mlxsw_sp_port_link_mode)

static void
mlxsw_sp_from_ptys_supported_port(u32 ptys_eth_proto,
				  struct ethtool_link_ksettings *cmd)
{
	if (ptys_eth_proto & (MLXSW_REG_PTYS_ETH_SPEED_10GBASE_CR |
			      MLXSW_REG_PTYS_ETH_SPEED_10GBASE_SR |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_CR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_SR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_100GBASE_SR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_SGMII))
		ethtool_link_ksettings_add_link_mode(cmd, supported, FIBRE);

	if (ptys_eth_proto & (MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KR |
			      MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KX4 |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_KR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_100GBASE_KR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_1000BASE_KX))
		ethtool_link_ksettings_add_link_mode(cmd, supported, Backplane);
}

static void mlxsw_sp_from_ptys_link(u32 ptys_eth_proto, unsigned long *mode)
{
	int i;

	for (i = 0; i < MLXSW_SP_PORT_LINK_MODE_LEN; i++) {
		if (ptys_eth_proto & mlxsw_sp_port_link_mode[i].mask)
			__set_bit(mlxsw_sp_port_link_mode[i].mask_ethtool,
				  mode);
	}
}

static void mlxsw_sp_from_ptys_speed_duplex(bool carrier_ok, u32 ptys_eth_proto,
					    struct ethtool_link_ksettings *cmd)
{
	u32 speed = SPEED_UNKNOWN;
	u8 duplex = DUPLEX_UNKNOWN;
	int i;

	if (!carrier_ok)
		goto out;

	for (i = 0; i < MLXSW_SP_PORT_LINK_MODE_LEN; i++) {
		if (ptys_eth_proto & mlxsw_sp_port_link_mode[i].mask) {
			speed = mlxsw_sp_port_link_mode[i].speed;
			duplex = DUPLEX_FULL;
			break;
		}
	}
out:
	cmd->base.speed = speed;
	cmd->base.duplex = duplex;
}

static u8 mlxsw_sp_port_connector_port(u32 ptys_eth_proto)
{
	if (ptys_eth_proto & (MLXSW_REG_PTYS_ETH_SPEED_10GBASE_SR |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_SR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_100GBASE_SR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_SGMII))
		return PORT_FIBRE;

	if (ptys_eth_proto & (MLXSW_REG_PTYS_ETH_SPEED_10GBASE_CR |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_CR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_100GBASE_CR4))
		return PORT_DA;

	if (ptys_eth_proto & (MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KR |
			      MLXSW_REG_PTYS_ETH_SPEED_10GBASE_KX4 |
			      MLXSW_REG_PTYS_ETH_SPEED_40GBASE_KR4 |
			      MLXSW_REG_PTYS_ETH_SPEED_100GBASE_KR4))
		return PORT_NONE;

	return PORT_OTHER;
}

static u32
mlxsw_sp_to_ptys_advert_link(const struct ethtool_link_ksettings *cmd)
{
	u32 ptys_proto = 0;
	int i;

	for (i = 0; i < MLXSW_SP_PORT_LINK_MODE_LEN; i++) {
		if (test_bit(mlxsw_sp_port_link_mode[i].mask_ethtool,
			     cmd->link_modes.advertising))
			ptys_proto |= mlxsw_sp_port_link_mode[i].mask;
	}
	return ptys_proto;
}

static u32 mlxsw_sp_to_ptys_speed(u32 speed)
{
	u32 ptys_proto = 0;
	int i;

	for (i = 0; i < MLXSW_SP_PORT_LINK_MODE_LEN; i++) {
		if (speed == mlxsw_sp_port_link_mode[i].speed)
			ptys_proto |= mlxsw_sp_port_link_mode[i].mask;
	}
	return ptys_proto;
}

static u32 mlxsw_sp_to_ptys_upper_speed(u32 upper_speed)
{
	u32 ptys_proto = 0;
	int i;

	for (i = 0; i < MLXSW_SP_PORT_LINK_MODE_LEN; i++) {
		if (mlxsw_sp_port_link_mode[i].speed <= upper_speed)
			ptys_proto |= mlxsw_sp_port_link_mode[i].mask;
	}
	return ptys_proto;
}

static void mlxsw_sp_port_get_link_supported(u32 eth_proto_cap,
					     struct ethtool_link_ksettings *cmd)
{
	ethtool_link_ksettings_add_link_mode(cmd, supported, Asym_Pause);
	ethtool_link_ksettings_add_link_mode(cmd, supported, Autoneg);
	ethtool_link_ksettings_add_link_mode(cmd, supported, Pause);

	mlxsw_sp_from_ptys_supported_port(eth_proto_cap, cmd);
	mlxsw_sp_from_ptys_link(eth_proto_cap, cmd->link_modes.supported);
}

static void mlxsw_sp_port_get_link_advertise(u32 eth_proto_admin, bool autoneg,
					     struct ethtool_link_ksettings *cmd)
{
	if (!autoneg)
		return;

	ethtool_link_ksettings_add_link_mode(cmd, advertising, Autoneg);
	mlxsw_sp_from_ptys_link(eth_proto_admin, cmd->link_modes.advertising);
}

static void
mlxsw_sp_port_get_link_lp_advertise(u32 eth_proto_lp, u8 autoneg_status,
				    struct ethtool_link_ksettings *cmd)
{
	if (autoneg_status != MLXSW_REG_PTYS_AN_STATUS_OK || !eth_proto_lp)
		return;

	ethtool_link_ksettings_add_link_mode(cmd, lp_advertising, Autoneg);
	mlxsw_sp_from_ptys_link(eth_proto_lp, cmd->link_modes.lp_advertising);
}

static int mlxsw_sp_port_get_link_ksettings(struct net_device *dev,
					    struct ethtool_link_ksettings *cmd)
{
	u32 eth_proto_cap, eth_proto_admin, eth_proto_oper, eth_proto_lp;
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char ptys_pl[MLXSW_REG_PTYS_LEN];
	u8 autoneg_status;
	bool autoneg;
	int err;

	autoneg = mlxsw_sp_port->link.autoneg;
	mlxsw_reg_ptys_eth_pack(ptys_pl, mlxsw_sp_port->local_port, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(ptys), ptys_pl);
	if (err)
		return err;
	mlxsw_reg_ptys_eth_unpack(ptys_pl, &eth_proto_cap, &eth_proto_admin,
				  &eth_proto_oper);

	mlxsw_sp_port_get_link_supported(eth_proto_cap, cmd);

	mlxsw_sp_port_get_link_advertise(eth_proto_admin, autoneg, cmd);

	eth_proto_lp = mlxsw_reg_ptys_eth_proto_lp_advertise_get(ptys_pl);
	autoneg_status = mlxsw_reg_ptys_an_status_get(ptys_pl);
	mlxsw_sp_port_get_link_lp_advertise(eth_proto_lp, autoneg_status, cmd);

	cmd->base.autoneg = autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;
	cmd->base.port = mlxsw_sp_port_connector_port(eth_proto_oper);
	mlxsw_sp_from_ptys_speed_duplex(netif_carrier_ok(dev), eth_proto_oper,
					cmd);

	return 0;
}

static int
mlxsw_sp_port_set_link_ksettings(struct net_device *dev,
				 const struct ethtool_link_ksettings *cmd)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char ptys_pl[MLXSW_REG_PTYS_LEN];
	u32 eth_proto_cap, eth_proto_new;
	bool autoneg;
	int err;

	mlxsw_reg_ptys_eth_pack(ptys_pl, mlxsw_sp_port->local_port, 0);
	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(ptys), ptys_pl);
	if (err)
		return err;
	mlxsw_reg_ptys_eth_unpack(ptys_pl, &eth_proto_cap, NULL, NULL);

	autoneg = cmd->base.autoneg == AUTONEG_ENABLE;
	eth_proto_new = autoneg ?
		mlxsw_sp_to_ptys_advert_link(cmd) :
		mlxsw_sp_to_ptys_speed(cmd->base.speed);

	eth_proto_new = eth_proto_new & eth_proto_cap;
	if (!eth_proto_new) {
		netdev_err(dev, "No supported speed requested\n");
		return -EINVAL;
	}

	mlxsw_reg_ptys_eth_pack(ptys_pl, mlxsw_sp_port->local_port,
				eth_proto_new);
	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(ptys), ptys_pl);
	if (err)
		return err;

	if (!netif_running(dev))
		return 0;

	mlxsw_sp_port->link.autoneg = autoneg;

	mlxsw_sp_port_admin_status_set(mlxsw_sp_port, false);
	mlxsw_sp_port_admin_status_set(mlxsw_sp_port, true);

	return 0;
}

static int mlxsw_sp_flash_device(struct net_device *dev,
				 struct ethtool_flash *flash)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	const struct firmware *firmware;
	int err;

	if (flash->region != ETHTOOL_FLASH_ALL_REGIONS)
		return -EOPNOTSUPP;

	dev_hold(dev);
	rtnl_unlock();

	err = request_firmware_direct(&firmware, flash->data, &dev->dev);
	if (err)
		goto out;
	err = mlxsw_sp_firmware_flash(mlxsw_sp, firmware);
	release_firmware(firmware);
out:
	rtnl_lock();
	dev_put(dev);
	return err;
}

#define MLXSW_SP_I2C_ADDR_LOW 0x50
#define MLXSW_SP_I2C_ADDR_HIGH 0x51
#define MLXSW_SP_EEPROM_PAGE_LENGTH 256

static int mlxsw_sp_query_module_eeprom(struct mlxsw_sp_port *mlxsw_sp_port,
					u16 offset, u16 size, void *data,
					unsigned int *p_read_size)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char eeprom_tmp[MLXSW_SP_REG_MCIA_EEPROM_SIZE];
	char mcia_pl[MLXSW_REG_MCIA_LEN];
	u16 i2c_addr;
	int status;
	int err;

	size = min_t(u16, size, MLXSW_SP_REG_MCIA_EEPROM_SIZE);

	if (offset < MLXSW_SP_EEPROM_PAGE_LENGTH &&
	    offset + size > MLXSW_SP_EEPROM_PAGE_LENGTH)
		/* Cross pages read, read until offset 256 in low page */
		size = MLXSW_SP_EEPROM_PAGE_LENGTH - offset;

	i2c_addr = MLXSW_SP_I2C_ADDR_LOW;
	if (offset >= MLXSW_SP_EEPROM_PAGE_LENGTH) {
		i2c_addr = MLXSW_SP_I2C_ADDR_HIGH;
		offset -= MLXSW_SP_EEPROM_PAGE_LENGTH;
	}

	mlxsw_reg_mcia_pack(mcia_pl, mlxsw_sp_port->mapping.module,
			    0, 0, offset, size, i2c_addr);

	err = mlxsw_reg_query(mlxsw_sp->core, MLXSW_REG(mcia), mcia_pl);
	if (err)
		return err;

	status = mlxsw_reg_mcia_status_get(mcia_pl);
	if (status)
		return -EIO;

	mlxsw_reg_mcia_eeprom_memcpy_from(mcia_pl, eeprom_tmp);
	memcpy(data, eeprom_tmp, size);
	*p_read_size = size;

	return 0;
}

enum mlxsw_sp_eeprom_module_info_rev_id {
	MLXSW_SP_EEPROM_MODULE_INFO_REV_ID_UNSPC      = 0x00,
	MLXSW_SP_EEPROM_MODULE_INFO_REV_ID_8436       = 0x01,
	MLXSW_SP_EEPROM_MODULE_INFO_REV_ID_8636       = 0x03,
};

enum mlxsw_sp_eeprom_module_info_id {
	MLXSW_SP_EEPROM_MODULE_INFO_ID_SFP              = 0x03,
	MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP             = 0x0C,
	MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP_PLUS        = 0x0D,
	MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP28           = 0x11,
};

enum mlxsw_sp_eeprom_module_info {
	MLXSW_SP_EEPROM_MODULE_INFO_ID,
	MLXSW_SP_EEPROM_MODULE_INFO_REV_ID,
	MLXSW_SP_EEPROM_MODULE_INFO_SIZE,
};

static int mlxsw_sp_get_module_info(struct net_device *netdev,
				    struct ethtool_modinfo *modinfo)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(netdev);
	u8 module_info[MLXSW_SP_EEPROM_MODULE_INFO_SIZE];
	u8 module_rev_id, module_id;
	unsigned int read_size;
	int err;

	err = mlxsw_sp_query_module_eeprom(mlxsw_sp_port, 0,
					   MLXSW_SP_EEPROM_MODULE_INFO_SIZE,
					   module_info, &read_size);
	if (err)
		return err;

	if (read_size < MLXSW_SP_EEPROM_MODULE_INFO_SIZE)
		return -EIO;

	module_rev_id = module_info[MLXSW_SP_EEPROM_MODULE_INFO_REV_ID];
	module_id = module_info[MLXSW_SP_EEPROM_MODULE_INFO_ID];

	switch (module_id) {
	case MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP:
		modinfo->type       = ETH_MODULE_SFF_8436;
		modinfo->eeprom_len = ETH_MODULE_SFF_8436_LEN;
		break;
	case MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP_PLUS:
	case MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP28:
		if (module_id  == MLXSW_SP_EEPROM_MODULE_INFO_ID_QSFP28 ||
		    module_rev_id >= MLXSW_SP_EEPROM_MODULE_INFO_REV_ID_8636) {
			modinfo->type       = ETH_MODULE_SFF_8636;
			modinfo->eeprom_len = ETH_MODULE_SFF_8636_LEN;
		} else {
			modinfo->type       = ETH_MODULE_SFF_8436;
			modinfo->eeprom_len = ETH_MODULE_SFF_8436_LEN;
		}
		break;
	case MLXSW_SP_EEPROM_MODULE_INFO_ID_SFP:
		modinfo->type       = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mlxsw_sp_get_module_eeprom(struct net_device *netdev,
				      struct ethtool_eeprom *ee,
				      u8 *data)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(netdev);
	int offset = ee->offset;
	unsigned int read_size;
	int i = 0;
	int err;

	if (!ee->len)
		return -EINVAL;

	memset(data, 0, ee->len);

	while (i < ee->len) {
		err = mlxsw_sp_query_module_eeprom(mlxsw_sp_port, offset,
						   ee->len - i, data + i,
						   &read_size);
		if (err) {
			netdev_err(mlxsw_sp_port->dev, "Eeprom query failed\n");
			return err;
		}

		i += read_size;
		offset += read_size;
	}

	return 0;
}

static const struct ethtool_ops mlxsw_sp_port_ethtool_ops = {
	.get_drvinfo		= mlxsw_sp_port_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_pauseparam		= mlxsw_sp_port_get_pauseparam,
	.set_pauseparam		= mlxsw_sp_port_set_pauseparam,
	.get_strings		= mlxsw_sp_port_get_strings,
	.set_phys_id		= mlxsw_sp_port_set_phys_id,
	.get_ethtool_stats	= mlxsw_sp_port_get_stats,
	.get_sset_count		= mlxsw_sp_port_get_sset_count,
	.get_link_ksettings	= mlxsw_sp_port_get_link_ksettings,
	.set_link_ksettings	= mlxsw_sp_port_set_link_ksettings,
	.flash_device		= mlxsw_sp_flash_device,
	.get_module_info	= mlxsw_sp_get_module_info,
	.get_module_eeprom	= mlxsw_sp_get_module_eeprom,
};

static int
mlxsw_sp_port_speed_by_width_set(struct mlxsw_sp_port *mlxsw_sp_port, u8 width)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	u32 upper_speed = MLXSW_SP_PORT_BASE_SPEED * width;
	char ptys_pl[MLXSW_REG_PTYS_LEN];
	u32 eth_proto_admin;

	eth_proto_admin = mlxsw_sp_to_ptys_upper_speed(upper_speed);
	mlxsw_reg_ptys_eth_pack(ptys_pl, mlxsw_sp_port->local_port,
				eth_proto_admin);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(ptys), ptys_pl);
}

int mlxsw_sp_port_ets_set(struct mlxsw_sp_port *mlxsw_sp_port,
			  enum mlxsw_reg_qeec_hr hr, u8 index, u8 next_index,
			  bool dwrr, u8 dwrr_weight)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char qeec_pl[MLXSW_REG_QEEC_LEN];

	mlxsw_reg_qeec_pack(qeec_pl, mlxsw_sp_port->local_port, hr, index,
			    next_index);
	mlxsw_reg_qeec_de_set(qeec_pl, true);
	mlxsw_reg_qeec_dwrr_set(qeec_pl, dwrr);
	mlxsw_reg_qeec_dwrr_weight_set(qeec_pl, dwrr_weight);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(qeec), qeec_pl);
}

int mlxsw_sp_port_ets_maxrate_set(struct mlxsw_sp_port *mlxsw_sp_port,
				  enum mlxsw_reg_qeec_hr hr, u8 index,
				  u8 next_index, u32 maxrate)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char qeec_pl[MLXSW_REG_QEEC_LEN];

	mlxsw_reg_qeec_pack(qeec_pl, mlxsw_sp_port->local_port, hr, index,
			    next_index);
	mlxsw_reg_qeec_mase_set(qeec_pl, true);
	mlxsw_reg_qeec_max_shaper_rate_set(qeec_pl, maxrate);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(qeec), qeec_pl);
}

int mlxsw_sp_port_prio_tc_set(struct mlxsw_sp_port *mlxsw_sp_port,
			      u8 switch_prio, u8 tclass)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char qtct_pl[MLXSW_REG_QTCT_LEN];

	mlxsw_reg_qtct_pack(qtct_pl, mlxsw_sp_port->local_port, switch_prio,
			    tclass);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(qtct), qtct_pl);
}

static int mlxsw_sp_port_ets_init(struct mlxsw_sp_port *mlxsw_sp_port)
{
	int err, i;

	/* Setup the elements hierarcy, so that each TC is linked to
	 * one subgroup, which are all member in the same group.
	 */
	err = mlxsw_sp_port_ets_set(mlxsw_sp_port,
				    MLXSW_REG_QEEC_HIERARCY_GROUP, 0, 0, false,
				    0);
	if (err)
		return err;
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		err = mlxsw_sp_port_ets_set(mlxsw_sp_port,
					    MLXSW_REG_QEEC_HIERARCY_SUBGROUP, i,
					    0, false, 0);
		if (err)
			return err;
	}
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		err = mlxsw_sp_port_ets_set(mlxsw_sp_port,
					    MLXSW_REG_QEEC_HIERARCY_TC, i, i,
					    false, 0);
		if (err)
			return err;
	}

	/* Make sure the max shaper is disabled in all hierarcies that
	 * support it.
	 */
	err = mlxsw_sp_port_ets_maxrate_set(mlxsw_sp_port,
					    MLXSW_REG_QEEC_HIERARCY_PORT, 0, 0,
					    MLXSW_REG_QEEC_MAS_DIS);
	if (err)
		return err;
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		err = mlxsw_sp_port_ets_maxrate_set(mlxsw_sp_port,
						    MLXSW_REG_QEEC_HIERARCY_SUBGROUP,
						    i, 0,
						    MLXSW_REG_QEEC_MAS_DIS);
		if (err)
			return err;
	}
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		err = mlxsw_sp_port_ets_maxrate_set(mlxsw_sp_port,
						    MLXSW_REG_QEEC_HIERARCY_TC,
						    i, i,
						    MLXSW_REG_QEEC_MAS_DIS);
		if (err)
			return err;
	}

	/* Map all priorities to traffic class 0. */
	for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++) {
		err = mlxsw_sp_port_prio_tc_set(mlxsw_sp_port, i, 0);
		if (err)
			return err;
	}

	return 0;
}

static int mlxsw_sp_port_create(struct mlxsw_sp *mlxsw_sp, u8 local_port,
				bool split, u8 module, u8 width, u8 lane)
{
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan;
	struct mlxsw_sp_port *mlxsw_sp_port;
	struct net_device *dev;
	int err;

	err = mlxsw_core_port_init(mlxsw_sp->core, local_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to init core port\n",
			local_port);
		return err;
	}

	dev = alloc_etherdev(sizeof(struct mlxsw_sp_port));
	if (!dev) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}
	SET_NETDEV_DEV(dev, mlxsw_sp->bus_info->dev);
	mlxsw_sp_port = netdev_priv(dev);
	mlxsw_sp_port->dev = dev;
	mlxsw_sp_port->mlxsw_sp = mlxsw_sp;
	mlxsw_sp_port->local_port = local_port;
	mlxsw_sp_port->pvid = 1;
	mlxsw_sp_port->split = split;
	mlxsw_sp_port->mapping.module = module;
	mlxsw_sp_port->mapping.width = width;
	mlxsw_sp_port->mapping.lane = lane;
	mlxsw_sp_port->link.autoneg = 1;
	INIT_LIST_HEAD(&mlxsw_sp_port->vlans_list);
	INIT_LIST_HEAD(&mlxsw_sp_port->mall_tc_list);

	mlxsw_sp_port->pcpu_stats =
		netdev_alloc_pcpu_stats(struct mlxsw_sp_port_pcpu_stats);
	if (!mlxsw_sp_port->pcpu_stats) {
		err = -ENOMEM;
		goto err_alloc_stats;
	}

	mlxsw_sp_port->sample = kzalloc(sizeof(*mlxsw_sp_port->sample),
					GFP_KERNEL);
	if (!mlxsw_sp_port->sample) {
		err = -ENOMEM;
		goto err_alloc_sample;
	}

	INIT_DELAYED_WORK(&mlxsw_sp_port->periodic_hw_stats.update_dw,
			  &update_stats_cache);

	dev->netdev_ops = &mlxsw_sp_port_netdev_ops;
	dev->ethtool_ops = &mlxsw_sp_port_ethtool_ops;

	err = mlxsw_sp_port_module_map(mlxsw_sp_port, module, width, lane);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to map module\n",
			mlxsw_sp_port->local_port);
		goto err_port_module_map;
	}

	err = mlxsw_sp_port_swid_set(mlxsw_sp_port, 0);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to set SWID\n",
			mlxsw_sp_port->local_port);
		goto err_port_swid_set;
	}

	err = mlxsw_sp_port_dev_addr_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Unable to init port mac address\n",
			mlxsw_sp_port->local_port);
		goto err_dev_addr_init;
	}

	netif_carrier_off(dev);

	dev->features |= NETIF_F_NETNS_LOCAL | NETIF_F_LLTX | NETIF_F_SG |
			 NETIF_F_HW_VLAN_CTAG_FILTER | NETIF_F_HW_TC;
	dev->hw_features |= NETIF_F_HW_TC;

	dev->min_mtu = 0;
	dev->max_mtu = ETH_MAX_MTU;

	/* Each packet needs to have a Tx header (metadata) on top all other
	 * headers.
	 */
	dev->needed_headroom = MLXSW_TXHDR_LEN;

	err = mlxsw_sp_port_system_port_mapping_set(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to set system port mapping\n",
			mlxsw_sp_port->local_port);
		goto err_port_system_port_mapping_set;
	}

	err = mlxsw_sp_port_speed_by_width_set(mlxsw_sp_port, width);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to enable speeds\n",
			mlxsw_sp_port->local_port);
		goto err_port_speed_by_width_set;
	}

	err = mlxsw_sp_port_mtu_set(mlxsw_sp_port, ETH_DATA_LEN);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to set MTU\n",
			mlxsw_sp_port->local_port);
		goto err_port_mtu_set;
	}

	err = mlxsw_sp_port_admin_status_set(mlxsw_sp_port, false);
	if (err)
		goto err_port_admin_status_set;

	err = mlxsw_sp_port_buffers_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to initialize buffers\n",
			mlxsw_sp_port->local_port);
		goto err_port_buffers_init;
	}

	err = mlxsw_sp_port_ets_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to initialize ETS\n",
			mlxsw_sp_port->local_port);
		goto err_port_ets_init;
	}

	/* ETS and buffers must be initialized before DCB. */
	err = mlxsw_sp_port_dcb_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to initialize DCB\n",
			mlxsw_sp_port->local_port);
		goto err_port_dcb_init;
	}

	err = mlxsw_sp_port_fids_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to initialize FIDs\n",
			mlxsw_sp_port->local_port);
		goto err_port_fids_init;
	}

	err = mlxsw_sp_tc_qdisc_init(mlxsw_sp_port);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to initialize TC qdiscs\n",
			mlxsw_sp_port->local_port);
		goto err_port_qdiscs_init;
	}

	mlxsw_sp_port_vlan = mlxsw_sp_port_vlan_get(mlxsw_sp_port, 1);
	if (IS_ERR(mlxsw_sp_port_vlan)) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to create VID 1\n",
			mlxsw_sp_port->local_port);
		err = PTR_ERR(mlxsw_sp_port_vlan);
		goto err_port_vlan_get;
	}

	mlxsw_sp_port_switchdev_init(mlxsw_sp_port);
	mlxsw_sp->ports[local_port] = mlxsw_sp_port;
	err = register_netdev(dev);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Port %d: Failed to register netdev\n",
			mlxsw_sp_port->local_port);
		goto err_register_netdev;
	}

	mlxsw_core_port_eth_set(mlxsw_sp->core, mlxsw_sp_port->local_port,
				mlxsw_sp_port, dev, mlxsw_sp_port->split,
				module);
	mlxsw_core_schedule_dw(&mlxsw_sp_port->periodic_hw_stats.update_dw, 0);
	return 0;

err_register_netdev:
	mlxsw_sp->ports[local_port] = NULL;
	mlxsw_sp_port_switchdev_fini(mlxsw_sp_port);
	mlxsw_sp_port_vlan_put(mlxsw_sp_port_vlan);
err_port_vlan_get:
	mlxsw_sp_tc_qdisc_fini(mlxsw_sp_port);
err_port_qdiscs_init:
	mlxsw_sp_port_fids_fini(mlxsw_sp_port);
err_port_fids_init:
	mlxsw_sp_port_dcb_fini(mlxsw_sp_port);
err_port_dcb_init:
err_port_ets_init:
err_port_buffers_init:
err_port_admin_status_set:
err_port_mtu_set:
err_port_speed_by_width_set:
err_port_system_port_mapping_set:
err_dev_addr_init:
	mlxsw_sp_port_swid_set(mlxsw_sp_port, MLXSW_PORT_SWID_DISABLED_PORT);
err_port_swid_set:
	mlxsw_sp_port_module_unmap(mlxsw_sp_port);
err_port_module_map:
	kfree(mlxsw_sp_port->sample);
err_alloc_sample:
	free_percpu(mlxsw_sp_port->pcpu_stats);
err_alloc_stats:
	free_netdev(dev);
err_alloc_etherdev:
	mlxsw_core_port_fini(mlxsw_sp->core, local_port);
	return err;
}

static void mlxsw_sp_port_remove(struct mlxsw_sp *mlxsw_sp, u8 local_port)
{
	struct mlxsw_sp_port *mlxsw_sp_port = mlxsw_sp->ports[local_port];

	cancel_delayed_work_sync(&mlxsw_sp_port->periodic_hw_stats.update_dw);
	mlxsw_core_port_clear(mlxsw_sp->core, local_port, mlxsw_sp);
	unregister_netdev(mlxsw_sp_port->dev); /* This calls ndo_stop */
	mlxsw_sp->ports[local_port] = NULL;
	mlxsw_sp_port_switchdev_fini(mlxsw_sp_port);
	mlxsw_sp_port_vlan_flush(mlxsw_sp_port);
	mlxsw_sp_tc_qdisc_fini(mlxsw_sp_port);
	mlxsw_sp_port_fids_fini(mlxsw_sp_port);
	mlxsw_sp_port_dcb_fini(mlxsw_sp_port);
	mlxsw_sp_port_swid_set(mlxsw_sp_port, MLXSW_PORT_SWID_DISABLED_PORT);
	mlxsw_sp_port_module_unmap(mlxsw_sp_port);
	kfree(mlxsw_sp_port->sample);
	free_percpu(mlxsw_sp_port->pcpu_stats);
	WARN_ON_ONCE(!list_empty(&mlxsw_sp_port->vlans_list));
	free_netdev(mlxsw_sp_port->dev);
	mlxsw_core_port_fini(mlxsw_sp->core, local_port);
}

static bool mlxsw_sp_port_created(struct mlxsw_sp *mlxsw_sp, u8 local_port)
{
	return mlxsw_sp->ports[local_port] != NULL;
}

static void mlxsw_sp_ports_remove(struct mlxsw_sp *mlxsw_sp)
{
	int i;

	for (i = 1; i < mlxsw_core_max_ports(mlxsw_sp->core); i++)
		if (mlxsw_sp_port_created(mlxsw_sp, i))
			mlxsw_sp_port_remove(mlxsw_sp, i);
	kfree(mlxsw_sp->port_to_module);
	kfree(mlxsw_sp->ports);
}

static int mlxsw_sp_ports_create(struct mlxsw_sp *mlxsw_sp)
{
	unsigned int max_ports = mlxsw_core_max_ports(mlxsw_sp->core);
	u8 module, width, lane;
	size_t alloc_size;
	int i;
	int err;

	alloc_size = sizeof(struct mlxsw_sp_port *) * max_ports;
	mlxsw_sp->ports = kzalloc(alloc_size, GFP_KERNEL);
	if (!mlxsw_sp->ports)
		return -ENOMEM;

	mlxsw_sp->port_to_module = kmalloc_array(max_ports, sizeof(int),
						 GFP_KERNEL);
	if (!mlxsw_sp->port_to_module) {
		err = -ENOMEM;
		goto err_port_to_module_alloc;
	}

	for (i = 1; i < max_ports; i++) {
		/* Mark as invalid */
		mlxsw_sp->port_to_module[i] = -1;

		err = mlxsw_sp_port_module_info_get(mlxsw_sp, i, &module,
						    &width, &lane);
		if (err)
			goto err_port_module_info_get;
		if (!width)
			continue;
		mlxsw_sp->port_to_module[i] = module;
		err = mlxsw_sp_port_create(mlxsw_sp, i, false,
					   module, width, lane);
		if (err)
			goto err_port_create;
	}
	return 0;

err_port_create:
err_port_module_info_get:
	for (i--; i >= 1; i--)
		if (mlxsw_sp_port_created(mlxsw_sp, i))
			mlxsw_sp_port_remove(mlxsw_sp, i);
	kfree(mlxsw_sp->port_to_module);
err_port_to_module_alloc:
	kfree(mlxsw_sp->ports);
	return err;
}

static u8 mlxsw_sp_cluster_base_port_get(u8 local_port)
{
	u8 offset = (local_port - 1) % MLXSW_SP_PORTS_PER_CLUSTER_MAX;

	return local_port - offset;
}

static int mlxsw_sp_port_split_create(struct mlxsw_sp *mlxsw_sp, u8 base_port,
				      u8 module, unsigned int count)
{
	u8 width = MLXSW_PORT_MODULE_MAX_WIDTH / count;
	int err, i;

	for (i = 0; i < count; i++) {
		err = mlxsw_sp_port_create(mlxsw_sp, base_port + i, true,
					   module, width, i * width);
		if (err)
			goto err_port_create;
	}

	return 0;

err_port_create:
	for (i--; i >= 0; i--)
		if (mlxsw_sp_port_created(mlxsw_sp, base_port + i))
			mlxsw_sp_port_remove(mlxsw_sp, base_port + i);
	return err;
}

static void mlxsw_sp_port_unsplit_create(struct mlxsw_sp *mlxsw_sp,
					 u8 base_port, unsigned int count)
{
	u8 local_port, module, width = MLXSW_PORT_MODULE_MAX_WIDTH;
	int i;

	/* Split by four means we need to re-create two ports, otherwise
	 * only one.
	 */
	count = count / 2;

	for (i = 0; i < count; i++) {
		local_port = base_port + i * 2;
		if (mlxsw_sp->port_to_module[local_port] < 0)
			continue;
		module = mlxsw_sp->port_to_module[local_port];

		mlxsw_sp_port_create(mlxsw_sp, local_port, false, module,
				     width, 0);
	}
}

static int mlxsw_sp_port_split(struct mlxsw_core *mlxsw_core, u8 local_port,
			       unsigned int count)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_core_driver_priv(mlxsw_core);
	struct mlxsw_sp_port *mlxsw_sp_port;
	u8 module, cur_width, base_port;
	int i;
	int err;

	mlxsw_sp_port = mlxsw_sp->ports[local_port];
	if (!mlxsw_sp_port) {
		dev_err(mlxsw_sp->bus_info->dev, "Port number \"%d\" does not exist\n",
			local_port);
		return -EINVAL;
	}

	module = mlxsw_sp_port->mapping.module;
	cur_width = mlxsw_sp_port->mapping.width;

	if (count != 2 && count != 4) {
		netdev_err(mlxsw_sp_port->dev, "Port can only be split into 2 or 4 ports\n");
		return -EINVAL;
	}

	if (cur_width != MLXSW_PORT_MODULE_MAX_WIDTH) {
		netdev_err(mlxsw_sp_port->dev, "Port cannot be split further\n");
		return -EINVAL;
	}

	/* Make sure we have enough slave (even) ports for the split. */
	if (count == 2) {
		base_port = local_port;
		if (mlxsw_sp->ports[base_port + 1]) {
			netdev_err(mlxsw_sp_port->dev, "Invalid split configuration\n");
			return -EINVAL;
		}
	} else {
		base_port = mlxsw_sp_cluster_base_port_get(local_port);
		if (mlxsw_sp->ports[base_port + 1] ||
		    mlxsw_sp->ports[base_port + 3]) {
			netdev_err(mlxsw_sp_port->dev, "Invalid split configuration\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < count; i++)
		if (mlxsw_sp_port_created(mlxsw_sp, base_port + i))
			mlxsw_sp_port_remove(mlxsw_sp, base_port + i);

	err = mlxsw_sp_port_split_create(mlxsw_sp, base_port, module, count);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to create split ports\n");
		goto err_port_split_create;
	}

	return 0;

err_port_split_create:
	mlxsw_sp_port_unsplit_create(mlxsw_sp, base_port, count);
	return err;
}

static int mlxsw_sp_port_unsplit(struct mlxsw_core *mlxsw_core, u8 local_port)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_core_driver_priv(mlxsw_core);
	struct mlxsw_sp_port *mlxsw_sp_port;
	u8 cur_width, base_port;
	unsigned int count;
	int i;

	mlxsw_sp_port = mlxsw_sp->ports[local_port];
	if (!mlxsw_sp_port) {
		dev_err(mlxsw_sp->bus_info->dev, "Port number \"%d\" does not exist\n",
			local_port);
		return -EINVAL;
	}

	if (!mlxsw_sp_port->split) {
		netdev_err(mlxsw_sp_port->dev, "Port wasn't split\n");
		return -EINVAL;
	}

	cur_width = mlxsw_sp_port->mapping.width;
	count = cur_width == 1 ? 4 : 2;

	base_port = mlxsw_sp_cluster_base_port_get(local_port);

	/* Determine which ports to remove. */
	if (count == 2 && local_port >= base_port + 2)
		base_port = base_port + 2;

	for (i = 0; i < count; i++)
		if (mlxsw_sp_port_created(mlxsw_sp, base_port + i))
			mlxsw_sp_port_remove(mlxsw_sp, base_port + i);

	mlxsw_sp_port_unsplit_create(mlxsw_sp, base_port, count);

	return 0;
}

static void mlxsw_sp_pude_event_func(const struct mlxsw_reg_info *reg,
				     char *pude_pl, void *priv)
{
	struct mlxsw_sp *mlxsw_sp = priv;
	struct mlxsw_sp_port *mlxsw_sp_port;
	enum mlxsw_reg_pude_oper_status status;
	u8 local_port;

	local_port = mlxsw_reg_pude_local_port_get(pude_pl);
	mlxsw_sp_port = mlxsw_sp->ports[local_port];
	if (!mlxsw_sp_port)
		return;

	status = mlxsw_reg_pude_oper_status_get(pude_pl);
	if (status == MLXSW_PORT_OPER_STATUS_UP) {
		netdev_info(mlxsw_sp_port->dev, "link up\n");
		netif_carrier_on(mlxsw_sp_port->dev);
	} else {
		netdev_info(mlxsw_sp_port->dev, "link down\n");
		netif_carrier_off(mlxsw_sp_port->dev);
	}
}

static void mlxsw_sp_rx_listener_no_mark_func(struct sk_buff *skb,
					      u8 local_port, void *priv)
{
	struct mlxsw_sp *mlxsw_sp = priv;
	struct mlxsw_sp_port *mlxsw_sp_port = mlxsw_sp->ports[local_port];
	struct mlxsw_sp_port_pcpu_stats *pcpu_stats;

	if (unlikely(!mlxsw_sp_port)) {
		dev_warn_ratelimited(mlxsw_sp->bus_info->dev, "Port %d: skb received for non-existent port\n",
				     local_port);
		return;
	}

	skb->dev = mlxsw_sp_port->dev;

	pcpu_stats = this_cpu_ptr(mlxsw_sp_port->pcpu_stats);
	u64_stats_update_begin(&pcpu_stats->syncp);
	pcpu_stats->rx_packets++;
	pcpu_stats->rx_bytes += skb->len;
	u64_stats_update_end(&pcpu_stats->syncp);

	skb->protocol = eth_type_trans(skb, skb->dev);
	netif_receive_skb(skb);
}

static void mlxsw_sp_rx_listener_mark_func(struct sk_buff *skb, u8 local_port,
					   void *priv)
{
	skb->offload_fwd_mark = 1;
	return mlxsw_sp_rx_listener_no_mark_func(skb, local_port, priv);
}

static void mlxsw_sp_rx_listener_mr_mark_func(struct sk_buff *skb,
					      u8 local_port, void *priv)
{
	skb->offload_mr_fwd_mark = 1;
	skb->offload_fwd_mark = 1;
	return mlxsw_sp_rx_listener_no_mark_func(skb, local_port, priv);
}

static void mlxsw_sp_rx_listener_sample_func(struct sk_buff *skb, u8 local_port,
					     void *priv)
{
	struct mlxsw_sp *mlxsw_sp = priv;
	struct mlxsw_sp_port *mlxsw_sp_port = mlxsw_sp->ports[local_port];
	struct psample_group *psample_group;
	u32 size;

	if (unlikely(!mlxsw_sp_port)) {
		dev_warn_ratelimited(mlxsw_sp->bus_info->dev, "Port %d: sample skb received for non-existent port\n",
				     local_port);
		goto out;
	}
	if (unlikely(!mlxsw_sp_port->sample)) {
		dev_warn_ratelimited(mlxsw_sp->bus_info->dev, "Port %d: sample skb received on unsupported port\n",
				     local_port);
		goto out;
	}

	size = mlxsw_sp_port->sample->truncate ?
		  mlxsw_sp_port->sample->trunc_size : skb->len;

	rcu_read_lock();
	psample_group = rcu_dereference(mlxsw_sp_port->sample->psample_group);
	if (!psample_group)
		goto out_unlock;
	psample_sample_packet(psample_group, skb, size,
			      mlxsw_sp_port->dev->ifindex, 0,
			      mlxsw_sp_port->sample->rate);
out_unlock:
	rcu_read_unlock();
out:
	consume_skb(skb);
}

#define MLXSW_SP_RXL_NO_MARK(_trap_id, _action, _trap_group, _is_ctrl)	\
	MLXSW_RXL(mlxsw_sp_rx_listener_no_mark_func, _trap_id, _action,	\
		  _is_ctrl, SP_##_trap_group, DISCARD)

#define MLXSW_SP_RXL_MARK(_trap_id, _action, _trap_group, _is_ctrl)	\
	MLXSW_RXL(mlxsw_sp_rx_listener_mark_func, _trap_id, _action,	\
		_is_ctrl, SP_##_trap_group, DISCARD)

#define MLXSW_SP_RXL_MR_MARK(_trap_id, _action, _trap_group, _is_ctrl)	\
	MLXSW_RXL(mlxsw_sp_rx_listener_mr_mark_func, _trap_id, _action,	\
		_is_ctrl, SP_##_trap_group, DISCARD)

#define MLXSW_SP_EVENTL(_func, _trap_id)		\
	MLXSW_EVENTL(_func, _trap_id, SP_EVENT)

static const struct mlxsw_listener mlxsw_sp_listener[] = {
	/* Events */
	MLXSW_SP_EVENTL(mlxsw_sp_pude_event_func, PUDE),
	/* L2 traps */
	MLXSW_SP_RXL_NO_MARK(STP, TRAP_TO_CPU, STP, true),
	MLXSW_SP_RXL_NO_MARK(LACP, TRAP_TO_CPU, LACP, true),
	MLXSW_SP_RXL_NO_MARK(LLDP, TRAP_TO_CPU, LLDP, true),
	MLXSW_SP_RXL_MARK(DHCP, MIRROR_TO_CPU, DHCP, false),
	MLXSW_SP_RXL_MARK(IGMP_QUERY, MIRROR_TO_CPU, IGMP, false),
	MLXSW_SP_RXL_NO_MARK(IGMP_V1_REPORT, TRAP_TO_CPU, IGMP, false),
	MLXSW_SP_RXL_NO_MARK(IGMP_V2_REPORT, TRAP_TO_CPU, IGMP, false),
	MLXSW_SP_RXL_NO_MARK(IGMP_V2_LEAVE, TRAP_TO_CPU, IGMP, false),
	MLXSW_SP_RXL_NO_MARK(IGMP_V3_REPORT, TRAP_TO_CPU, IGMP, false),
	MLXSW_SP_RXL_MARK(ARPBC, MIRROR_TO_CPU, ARP, false),
	MLXSW_SP_RXL_MARK(ARPUC, MIRROR_TO_CPU, ARP, false),
	MLXSW_SP_RXL_NO_MARK(FID_MISS, TRAP_TO_CPU, IP2ME, false),
	MLXSW_SP_RXL_MARK(IPV6_MLDV12_LISTENER_QUERY, MIRROR_TO_CPU, IPV6_MLD,
			  false),
	MLXSW_SP_RXL_NO_MARK(IPV6_MLDV1_LISTENER_REPORT, TRAP_TO_CPU, IPV6_MLD,
			     false),
	MLXSW_SP_RXL_NO_MARK(IPV6_MLDV1_LISTENER_DONE, TRAP_TO_CPU, IPV6_MLD,
			     false),
	MLXSW_SP_RXL_NO_MARK(IPV6_MLDV2_LISTENER_REPORT, TRAP_TO_CPU, IPV6_MLD,
			     false),
	/* L3 traps */
	MLXSW_SP_RXL_MARK(MTUERROR, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(TTLERROR, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(LBERROR, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(IP2ME, TRAP_TO_CPU, IP2ME, false),
	MLXSW_SP_RXL_MARK(IPV6_UNSPECIFIED_ADDRESS, TRAP_TO_CPU, ROUTER_EXP,
			  false),
	MLXSW_SP_RXL_MARK(IPV6_LINK_LOCAL_DEST, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(IPV6_LINK_LOCAL_SRC, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(IPV6_ALL_NODES_LINK, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(IPV6_ALL_ROUTERS_LINK, TRAP_TO_CPU, ROUTER_EXP,
			  false),
	MLXSW_SP_RXL_MARK(IPV4_OSPF, TRAP_TO_CPU, OSPF, false),
	MLXSW_SP_RXL_MARK(IPV6_OSPF, TRAP_TO_CPU, OSPF, false),
	MLXSW_SP_RXL_MARK(IPV6_DHCP, TRAP_TO_CPU, DHCP, false),
	MLXSW_SP_RXL_MARK(RTR_INGRESS0, TRAP_TO_CPU, REMOTE_ROUTE, false),
	MLXSW_SP_RXL_MARK(IPV4_BGP, TRAP_TO_CPU, BGP, false),
	MLXSW_SP_RXL_MARK(IPV6_BGP, TRAP_TO_CPU, BGP, false),
	MLXSW_SP_RXL_MARK(L3_IPV6_ROUTER_SOLICITATION, TRAP_TO_CPU, IPV6_ND,
			  false),
	MLXSW_SP_RXL_MARK(L3_IPV6_ROUTER_ADVERTISMENT, TRAP_TO_CPU, IPV6_ND,
			  false),
	MLXSW_SP_RXL_MARK(L3_IPV6_NEIGHBOR_SOLICITATION, TRAP_TO_CPU, IPV6_ND,
			  false),
	MLXSW_SP_RXL_MARK(L3_IPV6_NEIGHBOR_ADVERTISMENT, TRAP_TO_CPU, IPV6_ND,
			  false),
	MLXSW_SP_RXL_MARK(L3_IPV6_REDIRECTION, TRAP_TO_CPU, IPV6_ND, false),
	MLXSW_SP_RXL_MARK(IPV6_MC_LINK_LOCAL_DEST, TRAP_TO_CPU, ROUTER_EXP,
			  false),
	MLXSW_SP_RXL_MARK(HOST_MISS_IPV4, TRAP_TO_CPU, HOST_MISS, false),
	MLXSW_SP_RXL_MARK(HOST_MISS_IPV6, TRAP_TO_CPU, HOST_MISS, false),
	MLXSW_SP_RXL_MARK(ROUTER_ALERT_IPV4, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(ROUTER_ALERT_IPV6, TRAP_TO_CPU, ROUTER_EXP, false),
	MLXSW_SP_RXL_MARK(IPIP_DECAP_ERROR, TRAP_TO_CPU, ROUTER_EXP, false),
	/* PKT Sample trap */
	MLXSW_RXL(mlxsw_sp_rx_listener_sample_func, PKT_SAMPLE, MIRROR_TO_CPU,
		  false, SP_IP2ME, DISCARD),
	/* ACL trap */
	MLXSW_SP_RXL_NO_MARK(ACL0, TRAP_TO_CPU, IP2ME, false),
	/* Multicast Router Traps */
	MLXSW_SP_RXL_MARK(IPV4_PIM, TRAP_TO_CPU, PIM, false),
	MLXSW_SP_RXL_MARK(RPF, TRAP_TO_CPU, RPF, false),
	MLXSW_SP_RXL_MARK(ACL1, TRAP_TO_CPU, MULTICAST, false),
	MLXSW_SP_RXL_MR_MARK(ACL2, TRAP_TO_CPU, MULTICAST, false),
};

static int mlxsw_sp_cpu_policers_set(struct mlxsw_core *mlxsw_core)
{
	char qpcr_pl[MLXSW_REG_QPCR_LEN];
	enum mlxsw_reg_qpcr_ir_units ir_units;
	int max_cpu_policers;
	bool is_bytes;
	u8 burst_size;
	u32 rate;
	int i, err;

	if (!MLXSW_CORE_RES_VALID(mlxsw_core, MAX_CPU_POLICERS))
		return -EIO;

	max_cpu_policers = MLXSW_CORE_RES_GET(mlxsw_core, MAX_CPU_POLICERS);

	ir_units = MLXSW_REG_QPCR_IR_UNITS_M;
	for (i = 0; i < max_cpu_policers; i++) {
		is_bytes = false;
		switch (i) {
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_STP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_LACP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_LLDP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_OSPF:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_PIM:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_RPF:
			rate = 128;
			burst_size = 7;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IGMP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IPV6_MLD:
			rate = 16 * 1024;
			burst_size = 10;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_BGP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_ARP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_DHCP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_HOST_MISS:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_ROUTER_EXP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_REMOTE_ROUTE:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IPV6_ND:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_MULTICAST:
			rate = 1024;
			burst_size = 7;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IP2ME:
			is_bytes = true;
			rate = 4 * 1024;
			burst_size = 4;
			break;
		default:
			continue;
		}

		mlxsw_reg_qpcr_pack(qpcr_pl, i, ir_units, is_bytes, rate,
				    burst_size);
		err = mlxsw_reg_write(mlxsw_core, MLXSW_REG(qpcr), qpcr_pl);
		if (err)
			return err;
	}

	return 0;
}

static int mlxsw_sp_trap_groups_set(struct mlxsw_core *mlxsw_core)
{
	char htgt_pl[MLXSW_REG_HTGT_LEN];
	enum mlxsw_reg_htgt_trap_group i;
	int max_cpu_policers;
	int max_trap_groups;
	u8 priority, tc;
	u16 policer_id;
	int err;

	if (!MLXSW_CORE_RES_VALID(mlxsw_core, MAX_TRAP_GROUPS))
		return -EIO;

	max_trap_groups = MLXSW_CORE_RES_GET(mlxsw_core, MAX_TRAP_GROUPS);
	max_cpu_policers = MLXSW_CORE_RES_GET(mlxsw_core, MAX_CPU_POLICERS);

	for (i = 0; i < max_trap_groups; i++) {
		policer_id = i;
		switch (i) {
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_STP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_LACP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_LLDP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_OSPF:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_PIM:
			priority = 5;
			tc = 5;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_BGP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_DHCP:
			priority = 4;
			tc = 4;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IGMP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IP2ME:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IPV6_MLD:
			priority = 3;
			tc = 3;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_ARP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_IPV6_ND:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_RPF:
			priority = 2;
			tc = 2;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_HOST_MISS:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_ROUTER_EXP:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_REMOTE_ROUTE:
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_MULTICAST:
			priority = 1;
			tc = 1;
			break;
		case MLXSW_REG_HTGT_TRAP_GROUP_SP_EVENT:
			priority = MLXSW_REG_HTGT_DEFAULT_PRIORITY;
			tc = MLXSW_REG_HTGT_DEFAULT_TC;
			policer_id = MLXSW_REG_HTGT_INVALID_POLICER;
			break;
		default:
			continue;
		}

		if (max_cpu_policers <= policer_id &&
		    policer_id != MLXSW_REG_HTGT_INVALID_POLICER)
			return -EIO;

		mlxsw_reg_htgt_pack(htgt_pl, i, policer_id, priority, tc);
		err = mlxsw_reg_write(mlxsw_core, MLXSW_REG(htgt), htgt_pl);
		if (err)
			return err;
	}

	return 0;
}

static int mlxsw_sp_traps_init(struct mlxsw_sp *mlxsw_sp)
{
	int i;
	int err;

	err = mlxsw_sp_cpu_policers_set(mlxsw_sp->core);
	if (err)
		return err;

	err = mlxsw_sp_trap_groups_set(mlxsw_sp->core);
	if (err)
		return err;

	for (i = 0; i < ARRAY_SIZE(mlxsw_sp_listener); i++) {
		err = mlxsw_core_trap_register(mlxsw_sp->core,
					       &mlxsw_sp_listener[i],
					       mlxsw_sp);
		if (err)
			goto err_listener_register;

	}
	return 0;

err_listener_register:
	for (i--; i >= 0; i--) {
		mlxsw_core_trap_unregister(mlxsw_sp->core,
					   &mlxsw_sp_listener[i],
					   mlxsw_sp);
	}
	return err;
}

static void mlxsw_sp_traps_fini(struct mlxsw_sp *mlxsw_sp)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mlxsw_sp_listener); i++) {
		mlxsw_core_trap_unregister(mlxsw_sp->core,
					   &mlxsw_sp_listener[i],
					   mlxsw_sp);
	}
}

static int mlxsw_sp_lag_init(struct mlxsw_sp *mlxsw_sp)
{
	char slcr_pl[MLXSW_REG_SLCR_LEN];
	int err;

	mlxsw_reg_slcr_pack(slcr_pl, MLXSW_REG_SLCR_LAG_HASH_SMAC |
				     MLXSW_REG_SLCR_LAG_HASH_DMAC |
				     MLXSW_REG_SLCR_LAG_HASH_ETHERTYPE |
				     MLXSW_REG_SLCR_LAG_HASH_VLANID |
				     MLXSW_REG_SLCR_LAG_HASH_SIP |
				     MLXSW_REG_SLCR_LAG_HASH_DIP |
				     MLXSW_REG_SLCR_LAG_HASH_SPORT |
				     MLXSW_REG_SLCR_LAG_HASH_DPORT |
				     MLXSW_REG_SLCR_LAG_HASH_IPPROTO);
	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(slcr), slcr_pl);
	if (err)
		return err;

	if (!MLXSW_CORE_RES_VALID(mlxsw_sp->core, MAX_LAG) ||
	    !MLXSW_CORE_RES_VALID(mlxsw_sp->core, MAX_LAG_MEMBERS))
		return -EIO;

	mlxsw_sp->lags = kcalloc(MLXSW_CORE_RES_GET(mlxsw_sp->core, MAX_LAG),
				 sizeof(struct mlxsw_sp_upper),
				 GFP_KERNEL);
	if (!mlxsw_sp->lags)
		return -ENOMEM;

	return 0;
}

static void mlxsw_sp_lag_fini(struct mlxsw_sp *mlxsw_sp)
{
	kfree(mlxsw_sp->lags);
}

static int mlxsw_sp_basic_trap_groups_set(struct mlxsw_core *mlxsw_core)
{
	char htgt_pl[MLXSW_REG_HTGT_LEN];

	mlxsw_reg_htgt_pack(htgt_pl, MLXSW_REG_HTGT_TRAP_GROUP_EMAD,
			    MLXSW_REG_HTGT_INVALID_POLICER,
			    MLXSW_REG_HTGT_DEFAULT_PRIORITY,
			    MLXSW_REG_HTGT_DEFAULT_TC);
	return mlxsw_reg_write(mlxsw_core, MLXSW_REG(htgt), htgt_pl);
}

static int mlxsw_sp_netdevice_event(struct notifier_block *unused,
				    unsigned long event, void *ptr);

static int mlxsw_sp_init(struct mlxsw_core *mlxsw_core,
			 const struct mlxsw_bus_info *mlxsw_bus_info)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_core_driver_priv(mlxsw_core);
	int err;

	mlxsw_sp->core = mlxsw_core;
	mlxsw_sp->bus_info = mlxsw_bus_info;

	err = mlxsw_sp_fw_rev_validate(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Could not upgrade firmware\n");
		return err;
	}

	err = mlxsw_sp_base_mac_get(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to get base mac\n");
		return err;
	}

	err = mlxsw_sp_kvdl_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize KVDL\n");
		return err;
	}

	err = mlxsw_sp_fids_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize FIDs\n");
		goto err_fids_init;
	}

	err = mlxsw_sp_traps_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to set traps\n");
		goto err_traps_init;
	}

	err = mlxsw_sp_buffers_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize buffers\n");
		goto err_buffers_init;
	}

	err = mlxsw_sp_lag_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize LAG\n");
		goto err_lag_init;
	}

	err = mlxsw_sp_switchdev_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize switchdev\n");
		goto err_switchdev_init;
	}

	err = mlxsw_sp_counter_pool_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to init counter pool\n");
		goto err_counter_pool_init;
	}

	err = mlxsw_sp_afa_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize ACL actions\n");
		goto err_afa_init;
	}

	err = mlxsw_sp_router_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize router\n");
		goto err_router_init;
	}

	/* Initialize netdevice notifier after router is initialized, so that
	 * the event handler can use router structures.
	 */
	mlxsw_sp->netdevice_nb.notifier_call = mlxsw_sp_netdevice_event;
	err = register_netdevice_notifier(&mlxsw_sp->netdevice_nb);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to register netdev notifier\n");
		goto err_netdev_notifier;
	}

	err = mlxsw_sp_span_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to init span system\n");
		goto err_span_init;
	}

	err = mlxsw_sp_acl_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to initialize ACL\n");
		goto err_acl_init;
	}

	err = mlxsw_sp_dpipe_init(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to init pipeline debug\n");
		goto err_dpipe_init;
	}

	err = mlxsw_sp_ports_create(mlxsw_sp);
	if (err) {
		dev_err(mlxsw_sp->bus_info->dev, "Failed to create ports\n");
		goto err_ports_create;
	}

	return 0;

err_ports_create:
	mlxsw_sp_dpipe_fini(mlxsw_sp);
err_dpipe_init:
	mlxsw_sp_acl_fini(mlxsw_sp);
err_acl_init:
	mlxsw_sp_span_fini(mlxsw_sp);
err_span_init:
	unregister_netdevice_notifier(&mlxsw_sp->netdevice_nb);
err_netdev_notifier:
	mlxsw_sp_router_fini(mlxsw_sp);
err_router_init:
	mlxsw_sp_afa_fini(mlxsw_sp);
err_afa_init:
	mlxsw_sp_counter_pool_fini(mlxsw_sp);
err_counter_pool_init:
	mlxsw_sp_switchdev_fini(mlxsw_sp);
err_switchdev_init:
	mlxsw_sp_lag_fini(mlxsw_sp);
err_lag_init:
	mlxsw_sp_buffers_fini(mlxsw_sp);
err_buffers_init:
	mlxsw_sp_traps_fini(mlxsw_sp);
err_traps_init:
	mlxsw_sp_fids_fini(mlxsw_sp);
err_fids_init:
	mlxsw_sp_kvdl_fini(mlxsw_sp);
	return err;
}

static void mlxsw_sp_fini(struct mlxsw_core *mlxsw_core)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_core_driver_priv(mlxsw_core);

	mlxsw_sp_ports_remove(mlxsw_sp);
	mlxsw_sp_dpipe_fini(mlxsw_sp);
	mlxsw_sp_acl_fini(mlxsw_sp);
	mlxsw_sp_span_fini(mlxsw_sp);
	unregister_netdevice_notifier(&mlxsw_sp->netdevice_nb);
	mlxsw_sp_router_fini(mlxsw_sp);
	mlxsw_sp_afa_fini(mlxsw_sp);
	mlxsw_sp_counter_pool_fini(mlxsw_sp);
	mlxsw_sp_switchdev_fini(mlxsw_sp);
	mlxsw_sp_lag_fini(mlxsw_sp);
	mlxsw_sp_buffers_fini(mlxsw_sp);
	mlxsw_sp_traps_fini(mlxsw_sp);
	mlxsw_sp_fids_fini(mlxsw_sp);
	mlxsw_sp_kvdl_fini(mlxsw_sp);
}

static const struct mlxsw_config_profile mlxsw_sp_config_profile = {
	.used_max_vepa_channels		= 1,
	.max_vepa_channels		= 0,
	.used_max_mid			= 1,
	.max_mid			= MLXSW_SP_MID_MAX,
	.used_max_pgt			= 1,
	.max_pgt			= 0,
	.used_flood_tables		= 1,
	.used_flood_mode		= 1,
	.flood_mode			= 3,
	.max_fid_offset_flood_tables	= 3,
	.fid_offset_flood_table_size	= VLAN_N_VID - 1,
	.max_fid_flood_tables		= 3,
	.fid_flood_table_size		= MLXSW_SP_FID_8021D_MAX,
	.used_max_ib_mc			= 1,
	.max_ib_mc			= 0,
	.used_max_pkey			= 1,
	.max_pkey			= 0,
	.used_kvd_split_data		= 1,
	.kvd_hash_granularity		= MLXSW_SP_KVD_GRANULARITY,
	.kvd_hash_single_parts		= 59,
	.kvd_hash_double_parts		= 41,
	.kvd_linear_size		= MLXSW_SP_KVD_LINEAR_SIZE,
	.swid_config			= {
		{
			.used_type	= 1,
			.type		= MLXSW_PORT_SWID_TYPE_ETH,
		}
	},
	.resource_query_enable		= 1,
};

static bool
mlxsw_sp_resource_kvd_granularity_validate(struct netlink_ext_ack *extack,
					   u64 size)
{
	const struct mlxsw_config_profile *profile;

	profile = &mlxsw_sp_config_profile;
	if (size % profile->kvd_hash_granularity) {
		NL_SET_ERR_MSG_MOD(extack, "resource set with wrong granularity");
		return false;
	}
	return true;
}

static int
mlxsw_sp_resource_kvd_size_validate(struct devlink *devlink, u64 size,
				    struct netlink_ext_ack *extack)
{
	NL_SET_ERR_MSG_MOD(extack, "kvd size cannot be changed");
	return -EINVAL;
}

static int
mlxsw_sp_resource_kvd_linear_size_validate(struct devlink *devlink, u64 size,
					   struct netlink_ext_ack *extack)
{
	if (!mlxsw_sp_resource_kvd_granularity_validate(extack, size))
		return -EINVAL;

	return 0;
}

static int
mlxsw_sp_resource_kvd_hash_single_size_validate(struct devlink *devlink, u64 size,
						struct netlink_ext_ack *extack)
{
	struct mlxsw_core *mlxsw_core = devlink_priv(devlink);

	if (!mlxsw_sp_resource_kvd_granularity_validate(extack, size))
		return -EINVAL;

	if (size < MLXSW_CORE_RES_GET(mlxsw_core, KVD_SINGLE_MIN_SIZE)) {
		NL_SET_ERR_MSG_MOD(extack, "hash single size is smaller than minimum");
		return -EINVAL;
	}
	return 0;
}

static int
mlxsw_sp_resource_kvd_hash_double_size_validate(struct devlink *devlink, u64 size,
						struct netlink_ext_ack *extack)
{
	struct mlxsw_core *mlxsw_core = devlink_priv(devlink);

	if (!mlxsw_sp_resource_kvd_granularity_validate(extack, size))
		return -EINVAL;

	if (size < MLXSW_CORE_RES_GET(mlxsw_core, KVD_DOUBLE_MIN_SIZE)) {
		NL_SET_ERR_MSG_MOD(extack, "hash double size is smaller than minimum");
		return -EINVAL;
	}
	return 0;
}

static u64 mlxsw_sp_resource_kvd_linear_occ_get(struct devlink *devlink)
{
	struct mlxsw_core *mlxsw_core = devlink_priv(devlink);
	struct mlxsw_sp *mlxsw_sp = mlxsw_core_driver_priv(mlxsw_core);

	return mlxsw_sp_kvdl_occ_get(mlxsw_sp);
}

static struct devlink_resource_ops mlxsw_sp_resource_kvd_ops = {
	.size_validate = mlxsw_sp_resource_kvd_size_validate,
};

static struct devlink_resource_ops mlxsw_sp_resource_kvd_linear_ops = {
	.size_validate = mlxsw_sp_resource_kvd_linear_size_validate,
	.occ_get = mlxsw_sp_resource_kvd_linear_occ_get,
};

static struct devlink_resource_ops mlxsw_sp_resource_kvd_hash_single_ops = {
	.size_validate = mlxsw_sp_resource_kvd_hash_single_size_validate,
};

static struct devlink_resource_ops mlxsw_sp_resource_kvd_hash_double_ops = {
	.size_validate = mlxsw_sp_resource_kvd_hash_double_size_validate,
};

static struct devlink_resource_size_params mlxsw_sp_kvd_size_params;
static struct devlink_resource_size_params mlxsw_sp_linear_size_params;
static struct devlink_resource_size_params mlxsw_sp_hash_single_size_params;
static struct devlink_resource_size_params mlxsw_sp_hash_double_size_params;

static void
mlxsw_sp_resource_size_params_prepare(struct mlxsw_core *mlxsw_core)
{
	u32 single_size_min = MLXSW_CORE_RES_GET(mlxsw_core,
						 KVD_SINGLE_MIN_SIZE);
	u32 double_size_min = MLXSW_CORE_RES_GET(mlxsw_core,
						 KVD_DOUBLE_MIN_SIZE);
	u32 kvd_size = MLXSW_CORE_RES_GET(mlxsw_core, KVD_SIZE);
	u32 linear_size_min = 0;

	/* KVD top resource */
	mlxsw_sp_kvd_size_params.size_min = kvd_size;
	mlxsw_sp_kvd_size_params.size_max = kvd_size;
	mlxsw_sp_kvd_size_params.size_granularity = MLXSW_SP_KVD_GRANULARITY;
	mlxsw_sp_kvd_size_params.unit = DEVLINK_RESOURCE_UNIT_ENTRY;

	/* Linear part init */
	mlxsw_sp_linear_size_params.size_min = linear_size_min;
	mlxsw_sp_linear_size_params.size_max = kvd_size - single_size_min -
					       double_size_min;
	mlxsw_sp_linear_size_params.size_granularity = MLXSW_SP_KVD_GRANULARITY;
	mlxsw_sp_linear_size_params.unit = DEVLINK_RESOURCE_UNIT_ENTRY;

	/* Hash double part init */
	mlxsw_sp_hash_double_size_params.size_min = double_size_min;
	mlxsw_sp_hash_double_size_params.size_max = kvd_size - single_size_min -
						    linear_size_min;
	mlxsw_sp_hash_double_size_params.size_granularity = MLXSW_SP_KVD_GRANULARITY;
	mlxsw_sp_hash_double_size_params.unit = DEVLINK_RESOURCE_UNIT_ENTRY;

	/* Hash single part init */
	mlxsw_sp_hash_single_size_params.size_min = single_size_min;
	mlxsw_sp_hash_single_size_params.size_max = kvd_size - double_size_min -
						    linear_size_min;
	mlxsw_sp_hash_single_size_params.size_granularity = MLXSW_SP_KVD_GRANULARITY;
	mlxsw_sp_hash_single_size_params.unit = DEVLINK_RESOURCE_UNIT_ENTRY;
}

static int mlxsw_sp_resources_register(struct mlxsw_core *mlxsw_core)
{
	struct devlink *devlink = priv_to_devlink(mlxsw_core);
	u32 kvd_size, single_size, double_size, linear_size;
	const struct mlxsw_config_profile *profile;
	int err;

	profile = &mlxsw_sp_config_profile;
	if (!MLXSW_CORE_RES_VALID(mlxsw_core, KVD_SIZE))
		return -EIO;

	mlxsw_sp_resource_size_params_prepare(mlxsw_core);
	kvd_size = MLXSW_CORE_RES_GET(mlxsw_core, KVD_SIZE);
	err = devlink_resource_register(devlink, MLXSW_SP_RESOURCE_NAME_KVD,
					true, kvd_size,
					MLXSW_SP_RESOURCE_KVD,
					DEVLINK_RESOURCE_ID_PARENT_TOP,
					&mlxsw_sp_kvd_size_params,
					&mlxsw_sp_resource_kvd_ops);
	if (err)
		return err;

	linear_size = profile->kvd_linear_size;
	err = devlink_resource_register(devlink, MLXSW_SP_RESOURCE_NAME_KVD_LINEAR,
					false, linear_size,
					MLXSW_SP_RESOURCE_KVD_LINEAR,
					MLXSW_SP_RESOURCE_KVD,
					&mlxsw_sp_linear_size_params,
					&mlxsw_sp_resource_kvd_linear_ops);
	if (err)
		return err;

	double_size = kvd_size - linear_size;
	double_size *= profile->kvd_hash_double_parts;
	double_size /= profile->kvd_hash_double_parts +
		       profile->kvd_hash_single_parts;
	double_size = rounddown(double_size, profile->kvd_hash_granularity);
	err = devlink_resource_register(devlink, MLXSW_SP_RESOURCE_NAME_KVD_HASH_DOUBLE,
					false, double_size,
					MLXSW_SP_RESOURCE_KVD_HASH_DOUBLE,
					MLXSW_SP_RESOURCE_KVD,
					&mlxsw_sp_hash_double_size_params,
					&mlxsw_sp_resource_kvd_hash_double_ops);
	if (err)
		return err;

	single_size = kvd_size - double_size - linear_size;
	err = devlink_resource_register(devlink, MLXSW_SP_RESOURCE_NAME_KVD_HASH_SINGLE,
					false, single_size,
					MLXSW_SP_RESOURCE_KVD_HASH_SINGLE,
					MLXSW_SP_RESOURCE_KVD,
					&mlxsw_sp_hash_single_size_params,
					&mlxsw_sp_resource_kvd_hash_single_ops);
	if (err)
		return err;

	return 0;
}

static int mlxsw_sp_kvd_sizes_get(struct mlxsw_core *mlxsw_core,
				  const struct mlxsw_config_profile *profile,
				  u64 *p_single_size, u64 *p_double_size,
				  u64 *p_linear_size)
{
	struct devlink *devlink = priv_to_devlink(mlxsw_core);
	u32 double_size;
	int err;

	if (!MLXSW_CORE_RES_VALID(mlxsw_core, KVD_SINGLE_MIN_SIZE) ||
	    !MLXSW_CORE_RES_VALID(mlxsw_core, KVD_DOUBLE_MIN_SIZE) ||
	    !profile->used_kvd_split_data)
		return -EIO;

	/* The hash part is what left of the kvd without the
	 * linear part. It is split to the single size and
	 * double size by the parts ratio from the profile.
	 * Both sizes must be a multiplications of the
	 * granularity from the profile. In case the user
	 * provided the sizes they are obtained via devlink.
	 */
	err = devlink_resource_size_get(devlink,
					MLXSW_SP_RESOURCE_KVD_LINEAR,
					p_linear_size);
	if (err)
		*p_linear_size = profile->kvd_linear_size;

	err = devlink_resource_size_get(devlink,
					MLXSW_SP_RESOURCE_KVD_HASH_DOUBLE,
					p_double_size);
	if (err) {
		double_size = MLXSW_CORE_RES_GET(mlxsw_core, KVD_SIZE) -
			      *p_linear_size;
		double_size *= profile->kvd_hash_double_parts;
		double_size /= profile->kvd_hash_double_parts +
			       profile->kvd_hash_single_parts;
		*p_double_size = rounddown(double_size,
					   profile->kvd_hash_granularity);
	}

	err = devlink_resource_size_get(devlink,
					MLXSW_SP_RESOURCE_KVD_HASH_SINGLE,
					p_single_size);
	if (err)
		*p_single_size = MLXSW_CORE_RES_GET(mlxsw_core, KVD_SIZE) -
				 *p_double_size - *p_linear_size;

	/* Check results are legal. */
	if (*p_single_size < MLXSW_CORE_RES_GET(mlxsw_core, KVD_SINGLE_MIN_SIZE) ||
	    *p_double_size < MLXSW_CORE_RES_GET(mlxsw_core, KVD_DOUBLE_MIN_SIZE) ||
	    MLXSW_CORE_RES_GET(mlxsw_core, KVD_SIZE) < *p_linear_size)
		return -EIO;

	return 0;
}

static struct mlxsw_driver mlxsw_sp_driver = {
	.kind				= mlxsw_sp_driver_name,
	.priv_size			= sizeof(struct mlxsw_sp),
	.init				= mlxsw_sp_init,
	.fini				= mlxsw_sp_fini,
	.basic_trap_groups_set		= mlxsw_sp_basic_trap_groups_set,
	.port_split			= mlxsw_sp_port_split,
	.port_unsplit			= mlxsw_sp_port_unsplit,
	.sb_pool_get			= mlxsw_sp_sb_pool_get,
	.sb_pool_set			= mlxsw_sp_sb_pool_set,
	.sb_port_pool_get		= mlxsw_sp_sb_port_pool_get,
	.sb_port_pool_set		= mlxsw_sp_sb_port_pool_set,
	.sb_tc_pool_bind_get		= mlxsw_sp_sb_tc_pool_bind_get,
	.sb_tc_pool_bind_set		= mlxsw_sp_sb_tc_pool_bind_set,
	.sb_occ_snapshot		= mlxsw_sp_sb_occ_snapshot,
	.sb_occ_max_clear		= mlxsw_sp_sb_occ_max_clear,
	.sb_occ_port_pool_get		= mlxsw_sp_sb_occ_port_pool_get,
	.sb_occ_tc_port_bind_get	= mlxsw_sp_sb_occ_tc_port_bind_get,
	.txhdr_construct		= mlxsw_sp_txhdr_construct,
	.resources_register		= mlxsw_sp_resources_register,
	.kvd_sizes_get			= mlxsw_sp_kvd_sizes_get,
	.txhdr_len			= MLXSW_TXHDR_LEN,
	.profile			= &mlxsw_sp_config_profile,
};

bool mlxsw_sp_port_dev_check(const struct net_device *dev)
{
	return dev->netdev_ops == &mlxsw_sp_port_netdev_ops;
}

static int mlxsw_sp_lower_dev_walk(struct net_device *lower_dev, void *data)
{
	struct mlxsw_sp_port **p_mlxsw_sp_port = data;
	int ret = 0;

	if (mlxsw_sp_port_dev_check(lower_dev)) {
		*p_mlxsw_sp_port = netdev_priv(lower_dev);
		ret = 1;
	}

	return ret;
}

struct mlxsw_sp_port *mlxsw_sp_port_dev_lower_find(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port;

	if (mlxsw_sp_port_dev_check(dev))
		return netdev_priv(dev);

	mlxsw_sp_port = NULL;
	netdev_walk_all_lower_dev(dev, mlxsw_sp_lower_dev_walk, &mlxsw_sp_port);

	return mlxsw_sp_port;
}

struct mlxsw_sp *mlxsw_sp_lower_get(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port;

	mlxsw_sp_port = mlxsw_sp_port_dev_lower_find(dev);
	return mlxsw_sp_port ? mlxsw_sp_port->mlxsw_sp : NULL;
}

struct mlxsw_sp_port *mlxsw_sp_port_dev_lower_find_rcu(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port;

	if (mlxsw_sp_port_dev_check(dev))
		return netdev_priv(dev);

	mlxsw_sp_port = NULL;
	netdev_walk_all_lower_dev_rcu(dev, mlxsw_sp_lower_dev_walk,
				      &mlxsw_sp_port);

	return mlxsw_sp_port;
}

struct mlxsw_sp_port *mlxsw_sp_port_lower_dev_hold(struct net_device *dev)
{
	struct mlxsw_sp_port *mlxsw_sp_port;

	rcu_read_lock();
	mlxsw_sp_port = mlxsw_sp_port_dev_lower_find_rcu(dev);
	if (mlxsw_sp_port)
		dev_hold(mlxsw_sp_port->dev);
	rcu_read_unlock();
	return mlxsw_sp_port;
}

void mlxsw_sp_port_dev_put(struct mlxsw_sp_port *mlxsw_sp_port)
{
	dev_put(mlxsw_sp_port->dev);
}

static int mlxsw_sp_lag_create(struct mlxsw_sp *mlxsw_sp, u16 lag_id)
{
	char sldr_pl[MLXSW_REG_SLDR_LEN];

	mlxsw_reg_sldr_lag_create_pack(sldr_pl, lag_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sldr), sldr_pl);
}

static int mlxsw_sp_lag_destroy(struct mlxsw_sp *mlxsw_sp, u16 lag_id)
{
	char sldr_pl[MLXSW_REG_SLDR_LEN];

	mlxsw_reg_sldr_lag_destroy_pack(sldr_pl, lag_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sldr), sldr_pl);
}

static int mlxsw_sp_lag_col_port_add(struct mlxsw_sp_port *mlxsw_sp_port,
				     u16 lag_id, u8 port_index)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char slcor_pl[MLXSW_REG_SLCOR_LEN];

	mlxsw_reg_slcor_port_add_pack(slcor_pl, mlxsw_sp_port->local_port,
				      lag_id, port_index);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(slcor), slcor_pl);
}

static int mlxsw_sp_lag_col_port_remove(struct mlxsw_sp_port *mlxsw_sp_port,
					u16 lag_id)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char slcor_pl[MLXSW_REG_SLCOR_LEN];

	mlxsw_reg_slcor_port_remove_pack(slcor_pl, mlxsw_sp_port->local_port,
					 lag_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(slcor), slcor_pl);
}

static int mlxsw_sp_lag_col_port_enable(struct mlxsw_sp_port *mlxsw_sp_port,
					u16 lag_id)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char slcor_pl[MLXSW_REG_SLCOR_LEN];

	mlxsw_reg_slcor_col_enable_pack(slcor_pl, mlxsw_sp_port->local_port,
					lag_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(slcor), slcor_pl);
}

static int mlxsw_sp_lag_col_port_disable(struct mlxsw_sp_port *mlxsw_sp_port,
					 u16 lag_id)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char slcor_pl[MLXSW_REG_SLCOR_LEN];

	mlxsw_reg_slcor_col_disable_pack(slcor_pl, mlxsw_sp_port->local_port,
					 lag_id);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(slcor), slcor_pl);
}

static int mlxsw_sp_lag_index_get(struct mlxsw_sp *mlxsw_sp,
				  struct net_device *lag_dev,
				  u16 *p_lag_id)
{
	struct mlxsw_sp_upper *lag;
	int free_lag_id = -1;
	u64 max_lag;
	int i;

	max_lag = MLXSW_CORE_RES_GET(mlxsw_sp->core, MAX_LAG);
	for (i = 0; i < max_lag; i++) {
		lag = mlxsw_sp_lag_get(mlxsw_sp, i);
		if (lag->ref_count) {
			if (lag->dev == lag_dev) {
				*p_lag_id = i;
				return 0;
			}
		} else if (free_lag_id < 0) {
			free_lag_id = i;
		}
	}
	if (free_lag_id < 0)
		return -EBUSY;
	*p_lag_id = free_lag_id;
	return 0;
}

static bool
mlxsw_sp_master_lag_check(struct mlxsw_sp *mlxsw_sp,
			  struct net_device *lag_dev,
			  struct netdev_lag_upper_info *lag_upper_info,
			  struct netlink_ext_ack *extack)
{
	u16 lag_id;

	if (mlxsw_sp_lag_index_get(mlxsw_sp, lag_dev, &lag_id) != 0) {
		NL_SET_ERR_MSG(extack,
			       "spectrum: Exceeded number of supported LAG devices");
		return false;
	}
	if (lag_upper_info->tx_type != NETDEV_LAG_TX_TYPE_HASH) {
		NL_SET_ERR_MSG(extack,
			       "spectrum: LAG device using unsupported Tx type");
		return false;
	}
	return true;
}

static int mlxsw_sp_port_lag_index_get(struct mlxsw_sp *mlxsw_sp,
				       u16 lag_id, u8 *p_port_index)
{
	u64 max_lag_members;
	int i;

	max_lag_members = MLXSW_CORE_RES_GET(mlxsw_sp->core,
					     MAX_LAG_MEMBERS);
	for (i = 0; i < max_lag_members; i++) {
		if (!mlxsw_sp_port_lagged_get(mlxsw_sp, lag_id, i)) {
			*p_port_index = i;
			return 0;
		}
	}
	return -EBUSY;
}

static int mlxsw_sp_port_lag_join(struct mlxsw_sp_port *mlxsw_sp_port,
				  struct net_device *lag_dev)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct mlxsw_sp_port_vlan *mlxsw_sp_port_vlan;
	struct mlxsw_sp_upper *lag;
	u16 lag_id;
	u8 port_index;
	int err;

	err = mlxsw_sp_lag_index_get(mlxsw_sp, lag_dev, &lag_id);
	if (err)
		return err;
	lag = mlxsw_sp_lag_get(mlxsw_sp, lag_id);
	if (!lag->ref_count) {
		err = mlxsw_sp_lag_create(mlxsw_sp, lag_id);
		if (err)
			return err;
		lag->dev = lag_dev;
	}

	err = mlxsw_sp_port_lag_index_get(mlxsw_sp, lag_id, &port_index);
	if (err)
		return err;
	err = mlxsw_sp_lag_col_port_add(mlxsw_sp_port, lag_id, port_index);
	if (err)
		goto err_col_port_add;
	err = mlxsw_sp_lag_col_port_enable(mlxsw_sp_port, lag_id);
	if (err)
		goto err_col_port_enable;

	mlxsw_core_lag_mapping_set(mlxsw_sp->core, lag_id, port_index,
				   mlxsw_sp_port->local_port);
	mlxsw_sp_port->lag_id = lag_id;
	mlxsw_sp_port->lagged = 1;
	lag->ref_count++;

	/* Port is no longer usable as a router interface */
	mlxsw_sp_port_vlan = mlxsw_sp_port_vlan_find_by_vid(mlxsw_sp_port, 1);
	if (mlxsw_sp_port_vlan->fid)
		mlxsw_sp_port_vlan_router_leave(mlxsw_sp_port_vlan);

	return 0;

err_col_port_enable:
	mlxsw_sp_lag_col_port_remove(mlxsw_sp_port, lag_id);
err_col_port_add:
	if (!lag->ref_count)
		mlxsw_sp_lag_destroy(mlxsw_sp, lag_id);
	return err;
}

static void mlxsw_sp_port_lag_leave(struct mlxsw_sp_port *mlxsw_sp_port,
				    struct net_device *lag_dev)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	u16 lag_id = mlxsw_sp_port->lag_id;
	struct mlxsw_sp_upper *lag;

	if (!mlxsw_sp_port->lagged)
		return;
	lag = mlxsw_sp_lag_get(mlxsw_sp, lag_id);
	WARN_ON(lag->ref_count == 0);

	mlxsw_sp_lag_col_port_disable(mlxsw_sp_port, lag_id);
	mlxsw_sp_lag_col_port_remove(mlxsw_sp_port, lag_id);

	/* Any VLANs configured on the port are no longer valid */
	mlxsw_sp_port_vlan_flush(mlxsw_sp_port);

	if (lag->ref_count == 1)
		mlxsw_sp_lag_destroy(mlxsw_sp, lag_id);

	mlxsw_core_lag_mapping_clear(mlxsw_sp->core, lag_id,
				     mlxsw_sp_port->local_port);
	mlxsw_sp_port->lagged = 0;
	lag->ref_count--;

	mlxsw_sp_port_vlan_get(mlxsw_sp_port, 1);
	/* Make sure untagged frames are allowed to ingress */
	mlxsw_sp_port_pvid_set(mlxsw_sp_port, 1);
}

static int mlxsw_sp_lag_dist_port_add(struct mlxsw_sp_port *mlxsw_sp_port,
				      u16 lag_id)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char sldr_pl[MLXSW_REG_SLDR_LEN];

	mlxsw_reg_sldr_lag_add_port_pack(sldr_pl, lag_id,
					 mlxsw_sp_port->local_port);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sldr), sldr_pl);
}

static int mlxsw_sp_lag_dist_port_remove(struct mlxsw_sp_port *mlxsw_sp_port,
					 u16 lag_id)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	char sldr_pl[MLXSW_REG_SLDR_LEN];

	mlxsw_reg_sldr_lag_remove_port_pack(sldr_pl, lag_id,
					    mlxsw_sp_port->local_port);
	return mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(sldr), sldr_pl);
}

static int mlxsw_sp_port_lag_tx_en_set(struct mlxsw_sp_port *mlxsw_sp_port,
				       bool lag_tx_enabled)
{
	if (lag_tx_enabled)
		return mlxsw_sp_lag_dist_port_add(mlxsw_sp_port,
						  mlxsw_sp_port->lag_id);
	else
		return mlxsw_sp_lag_dist_port_remove(mlxsw_sp_port,
						     mlxsw_sp_port->lag_id);
}

static int mlxsw_sp_port_lag_changed(struct mlxsw_sp_port *mlxsw_sp_port,
				     struct netdev_lag_lower_state_info *info)
{
	return mlxsw_sp_port_lag_tx_en_set(mlxsw_sp_port, info->tx_enabled);
}

static int mlxsw_sp_port_stp_set(struct mlxsw_sp_port *mlxsw_sp_port,
				 bool enable)
{
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	enum mlxsw_reg_spms_state spms_state;
	char *spms_pl;
	u16 vid;
	int err;

	spms_state = enable ? MLXSW_REG_SPMS_STATE_FORWARDING :
			      MLXSW_REG_SPMS_STATE_DISCARDING;

	spms_pl = kmalloc(MLXSW_REG_SPMS_LEN, GFP_KERNEL);
	if (!spms_pl)
		return -ENOMEM;
	mlxsw_reg_spms_pack(spms_pl, mlxsw_sp_port->local_port);

	for (vid = 0; vid < VLAN_N_VID; vid++)
		mlxsw_reg_spms_vid_pack(spms_pl, vid, spms_state);

	err = mlxsw_reg_write(mlxsw_sp->core, MLXSW_REG(spms), spms_pl);
	kfree(spms_pl);
	return err;
}

static int mlxsw_sp_port_ovs_join(struct mlxsw_sp_port *mlxsw_sp_port)
{
	u16 vid = 1;
	int err;

	err = mlxsw_sp_port_vp_mode_set(mlxsw_sp_port, true);
	if (err)
		return err;
	err = mlxsw_sp_port_stp_set(mlxsw_sp_port, true);
	if (err)
		goto err_port_stp_set;
	err = mlxsw_sp_port_vlan_set(mlxsw_sp_port, 2, VLAN_N_VID - 1,
				     true, false);
	if (err)
		goto err_port_vlan_set;

	for (; vid <= VLAN_N_VID - 1; vid++) {
		err = mlxsw_sp_port_vid_learning_set(mlxsw_sp_port,
						     vid, false);
		if (err)
			goto err_vid_learning_set;
	}

	return 0;

err_vid_learning_set:
	for (vid--; vid >= 1; vid--)
		mlxsw_sp_port_vid_learning_set(mlxsw_sp_port, vid, true);
err_port_vlan_set:
	mlxsw_sp_port_stp_set(mlxsw_sp_port, false);
err_port_stp_set:
	mlxsw_sp_port_vp_mode_set(mlxsw_sp_port, false);
	return err;
}

static void mlxsw_sp_port_ovs_leave(struct mlxsw_sp_port *mlxsw_sp_port)
{
	u16 vid;

	for (vid = VLAN_N_VID - 1; vid >= 1; vid--)
		mlxsw_sp_port_vid_learning_set(mlxsw_sp_port,
					       vid, true);

	mlxsw_sp_port_vlan_set(mlxsw_sp_port, 2, VLAN_N_VID - 1,
			       false, false);
	mlxsw_sp_port_stp_set(mlxsw_sp_port, false);
	mlxsw_sp_port_vp_mode_set(mlxsw_sp_port, false);
}

static int mlxsw_sp_netdevice_port_upper_event(struct net_device *lower_dev,
					       struct net_device *dev,
					       unsigned long event, void *ptr)
{
	struct netdev_notifier_changeupper_info *info;
	struct mlxsw_sp_port *mlxsw_sp_port;
	struct netlink_ext_ack *extack;
	struct net_device *upper_dev;
	struct mlxsw_sp *mlxsw_sp;
	int err = 0;

	mlxsw_sp_port = netdev_priv(dev);
	mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	info = ptr;
	extack = netdev_notifier_info_to_extack(&info->info);

	switch (event) {
	case NETDEV_PRECHANGEUPPER:
		upper_dev = info->upper_dev;
		if (!is_vlan_dev(upper_dev) &&
		    !netif_is_lag_master(upper_dev) &&
		    !netif_is_bridge_master(upper_dev) &&
		    !netif_is_ovs_master(upper_dev)) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Unknown upper device type");
			return -EINVAL;
		}
		if (!info->linking)
			break;
		if (netdev_has_any_upper_dev(upper_dev) &&
		    (!netif_is_bridge_master(upper_dev) ||
		     !mlxsw_sp_bridge_device_is_offloaded(mlxsw_sp,
							  upper_dev))) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Enslaving a port to a device that already has an upper device is not supported");
			return -EINVAL;
		}
		if (netif_is_lag_master(upper_dev) &&
		    !mlxsw_sp_master_lag_check(mlxsw_sp, upper_dev,
					       info->upper_info, extack))
			return -EINVAL;
		if (netif_is_lag_master(upper_dev) && vlan_uses_dev(dev)) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Master device is a LAG master and this device has a VLAN");
			return -EINVAL;
		}
		if (netif_is_lag_port(dev) && is_vlan_dev(upper_dev) &&
		    !netif_is_lag_master(vlan_dev_real_dev(upper_dev))) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Can not put a VLAN on a LAG port");
			return -EINVAL;
		}
		if (netif_is_ovs_master(upper_dev) && vlan_uses_dev(dev)) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Master device is an OVS master and this device has a VLAN");
			return -EINVAL;
		}
		if (netif_is_ovs_port(dev) && is_vlan_dev(upper_dev)) {
			NL_SET_ERR_MSG(extack,
				       "spectrum: Can not put a VLAN on an OVS port");
			return -EINVAL;
		}
		break;
	case NETDEV_CHANGEUPPER:
		upper_dev = info->upper_dev;
		if (netif_is_bridge_master(upper_dev)) {
			if (info->linking)
				err = mlxsw_sp_port_bridge_join(mlxsw_sp_port,
								lower_dev,
								upper_dev,
								extack);
			else
				mlxsw_sp_port_bridge_leave(mlxsw_sp_port,
							   lower_dev,
							   upper_dev);
		} else if (netif_is_lag_master(upper_dev)) {
			if (info->linking)
				err = mlxsw_sp_port_lag_join(mlxsw_sp_port,
							     upper_dev);
			else
				mlxsw_sp_port_lag_leave(mlxsw_sp_port,
							upper_dev);
		} else if (netif_is_ovs_master(upper_dev)) {
			if (info->linking)
				err = mlxsw_sp_port_ovs_join(mlxsw_sp_port);
			else
				mlxsw_sp_port_ovs_leave(mlxsw_sp_port);
		}
		break;
	}

	return err;
}

static int mlxsw_sp_netdevice_port_lower_event(struct net_device *dev,
					       unsigned long event, void *ptr)
{
	struct netdev_notifier_changelowerstate_info *info;
	struct mlxsw_sp_port *mlxsw_sp_port;
	int err;

	mlxsw_sp_port = netdev_priv(dev);
	info = ptr;

	switch (event) {
	case NETDEV_CHANGELOWERSTATE:
		if (netif_is_lag_port(dev) && mlxsw_sp_port->lagged) {
			err = mlxsw_sp_port_lag_changed(mlxsw_sp_port,
							info->lower_state_info);
			if (err)
				netdev_err(dev, "Failed to reflect link aggregation lower state change\n");
		}
		break;
	}

	return 0;
}

static int mlxsw_sp_netdevice_port_event(struct net_device *lower_dev,
					 struct net_device *port_dev,
					 unsigned long event, void *ptr)
{
	switch (event) {
	case NETDEV_PRECHANGEUPPER:
	case NETDEV_CHANGEUPPER:
		return mlxsw_sp_netdevice_port_upper_event(lower_dev, port_dev,
							   event, ptr);
	case NETDEV_CHANGELOWERSTATE:
		return mlxsw_sp_netdevice_port_lower_event(port_dev, event,
							   ptr);
	}

	return 0;
}

static int mlxsw_sp_netdevice_lag_event(struct net_device *lag_dev,
					unsigned long event, void *ptr)
{
	struct net_device *dev;
	struct list_head *iter;
	int ret;

	netdev_for_each_lower_dev(lag_dev, dev, iter) {
		if (mlxsw_sp_port_dev_check(dev)) {
			ret = mlxsw_sp_netdevice_port_event(lag_dev, dev, event,
							    ptr);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int mlxsw_sp_netdevice_port_vlan_event(struct net_device *vlan_dev,
					      struct net_device *dev,
					      unsigned long event, void *ptr,
					      u16 vid)
{
	struct mlxsw_sp_port *mlxsw_sp_port = netdev_priv(dev);
	struct mlxsw_sp *mlxsw_sp = mlxsw_sp_port->mlxsw_sp;
	struct netdev_notifier_changeupper_info *info = ptr;
	struct netlink_ext_ack *extack;
	struct net_device *upper_dev;
	int err = 0;

	extack = netdev_notifier_info_to_extack(&info->info);

	switch (event) {
	case NETDEV_PRECHANGEUPPER:
		upper_dev = info->upper_dev;
		if (!netif_is_bridge_master(upper_dev)) {
			NL_SET_ERR_MSG(extack, "spectrum: VLAN devices only support bridge and VRF uppers");
			return -EINVAL;
		}
		if (!info->linking)
			break;
		if (netdev_has_any_upper_dev(upper_dev) &&
		    (!netif_is_bridge_master(upper_dev) ||
		     !mlxsw_sp_bridge_device_is_offloaded(mlxsw_sp,
							  upper_dev))) {
			NL_SET_ERR_MSG(extack, "spectrum: Enslaving a port to a device that already has an upper device is not supported");
			return -EINVAL;
		}
		break;
	case NETDEV_CHANGEUPPER:
		upper_dev = info->upper_dev;
		if (netif_is_bridge_master(upper_dev)) {
			if (info->linking)
				err = mlxsw_sp_port_bridge_join(mlxsw_sp_port,
								vlan_dev,
								upper_dev,
								extack);
			else
				mlxsw_sp_port_bridge_leave(mlxsw_sp_port,
							   vlan_dev,
							   upper_dev);
		} else {
			err = -EINVAL;
			WARN_ON(1);
		}
		break;
	}

	return err;
}

static int mlxsw_sp_netdevice_lag_port_vlan_event(struct net_device *vlan_dev,
						  struct net_device *lag_dev,
						  unsigned long event,
						  void *ptr, u16 vid)
{
	struct net_device *dev;
	struct list_head *iter;
	int ret;

	netdev_for_each_lower_dev(lag_dev, dev, iter) {
		if (mlxsw_sp_port_dev_check(dev)) {
			ret = mlxsw_sp_netdevice_port_vlan_event(vlan_dev, dev,
								 event, ptr,
								 vid);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int mlxsw_sp_netdevice_vlan_event(struct net_device *vlan_dev,
					 unsigned long event, void *ptr)
{
	struct net_device *real_dev = vlan_dev_real_dev(vlan_dev);
	u16 vid = vlan_dev_vlan_id(vlan_dev);

	if (mlxsw_sp_port_dev_check(real_dev))
		return mlxsw_sp_netdevice_port_vlan_event(vlan_dev, real_dev,
							  event, ptr, vid);
	else if (netif_is_lag_master(real_dev))
		return mlxsw_sp_netdevice_lag_port_vlan_event(vlan_dev,
							      real_dev, event,
							      ptr, vid);

	return 0;
}

static bool mlxsw_sp_is_vrf_event(unsigned long event, void *ptr)
{
	struct netdev_notifier_changeupper_info *info = ptr;

	if (event != NETDEV_PRECHANGEUPPER && event != NETDEV_CHANGEUPPER)
		return false;
	return netif_is_l3_master(info->upper_dev);
}

static int mlxsw_sp_netdevice_event(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct mlxsw_sp *mlxsw_sp;
	int err = 0;

	mlxsw_sp = container_of(nb, struct mlxsw_sp, netdevice_nb);
	if (mlxsw_sp_netdev_is_ipip_ol(mlxsw_sp, dev))
		err = mlxsw_sp_netdevice_ipip_ol_event(mlxsw_sp, dev,
						       event, ptr);
	else if (mlxsw_sp_netdev_is_ipip_ul(mlxsw_sp, dev))
		err = mlxsw_sp_netdevice_ipip_ul_event(mlxsw_sp, dev,
						       event, ptr);
	else if (event == NETDEV_CHANGEADDR || event == NETDEV_CHANGEMTU)
		err = mlxsw_sp_netdevice_router_port_event(dev);
	else if (mlxsw_sp_is_vrf_event(event, ptr))
		err = mlxsw_sp_netdevice_vrf_event(dev, event, ptr);
	else if (mlxsw_sp_port_dev_check(dev))
		err = mlxsw_sp_netdevice_port_event(dev, dev, event, ptr);
	else if (netif_is_lag_master(dev))
		err = mlxsw_sp_netdevice_lag_event(dev, event, ptr);
	else if (is_vlan_dev(dev))
		err = mlxsw_sp_netdevice_vlan_event(dev, event, ptr);

	return notifier_from_errno(err);
}

static struct notifier_block mlxsw_sp_inetaddr_valid_nb __read_mostly = {
	.notifier_call = mlxsw_sp_inetaddr_valid_event,
};

static struct notifier_block mlxsw_sp_inetaddr_nb __read_mostly = {
	.notifier_call = mlxsw_sp_inetaddr_event,
};

static struct notifier_block mlxsw_sp_inet6addr_valid_nb __read_mostly = {
	.notifier_call = mlxsw_sp_inet6addr_valid_event,
};

static struct notifier_block mlxsw_sp_inet6addr_nb __read_mostly = {
	.notifier_call = mlxsw_sp_inet6addr_event,
};

static const struct pci_device_id mlxsw_sp_pci_id_table[] = {
	{PCI_VDEVICE(MELLANOX, PCI_DEVICE_ID_MELLANOX_SPECTRUM), 0},
	{0, },
};

static struct pci_driver mlxsw_sp_pci_driver = {
	.name = mlxsw_sp_driver_name,
	.id_table = mlxsw_sp_pci_id_table,
};

static int __init mlxsw_sp_module_init(void)
{
	int err;

	register_inetaddr_validator_notifier(&mlxsw_sp_inetaddr_valid_nb);
	register_inetaddr_notifier(&mlxsw_sp_inetaddr_nb);
	register_inet6addr_validator_notifier(&mlxsw_sp_inet6addr_valid_nb);
	register_inet6addr_notifier(&mlxsw_sp_inet6addr_nb);

	err = mlxsw_core_driver_register(&mlxsw_sp_driver);
	if (err)
		goto err_core_driver_register;

	err = mlxsw_pci_driver_register(&mlxsw_sp_pci_driver);
	if (err)
		goto err_pci_driver_register;

	return 0;

err_pci_driver_register:
	mlxsw_core_driver_unregister(&mlxsw_sp_driver);
err_core_driver_register:
	unregister_inet6addr_notifier(&mlxsw_sp_inet6addr_nb);
	unregister_inet6addr_validator_notifier(&mlxsw_sp_inet6addr_valid_nb);
	unregister_inetaddr_notifier(&mlxsw_sp_inetaddr_nb);
	unregister_inetaddr_validator_notifier(&mlxsw_sp_inetaddr_valid_nb);
	return err;
}

static void __exit mlxsw_sp_module_exit(void)
{
	mlxsw_pci_driver_unregister(&mlxsw_sp_pci_driver);
	mlxsw_core_driver_unregister(&mlxsw_sp_driver);
	unregister_inet6addr_notifier(&mlxsw_sp_inet6addr_nb);
	unregister_inet6addr_validator_notifier(&mlxsw_sp_inet6addr_valid_nb);
	unregister_inetaddr_notifier(&mlxsw_sp_inetaddr_nb);
	unregister_inetaddr_validator_notifier(&mlxsw_sp_inetaddr_valid_nb);
}

module_init(mlxsw_sp_module_init);
module_exit(mlxsw_sp_module_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Jiri Pirko <jiri@mellanox.com>");
MODULE_DESCRIPTION("Mellanox Spectrum driver");
MODULE_DEVICE_TABLE(pci, mlxsw_sp_pci_id_table);
MODULE_FIRMWARE(MLXSW_SP_FW_FILENAME);
