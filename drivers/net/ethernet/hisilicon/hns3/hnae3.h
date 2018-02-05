/*
 * Copyright (c) 2016-2017 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __HNAE3_H
#define __HNAE3_H

/* Names used in this framework:
 *      ae handle (handle):
 *        a set of queues provided by AE
 *      ring buffer queue (rbq):
 *        the channel between upper layer and the AE, can do tx and rx
 *      ring:
 *        a tx or rx channel within a rbq
 *      ring description (desc):
 *        an element in the ring with packet information
 *      buffer:
 *        a memory region referred by desc with the full packet payload
 *
 * "num" means a static number set as a parameter, "count" mean a dynamic
 *   number set while running
 * "cb" means control block
 */

#include <linux/acpi.h>
#include <linux/dcbnl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/types.h>

/* Device IDs */
#define HNAE3_DEV_ID_GE				0xA220
#define HNAE3_DEV_ID_25GE			0xA221
#define HNAE3_DEV_ID_25GE_RDMA			0xA222
#define HNAE3_DEV_ID_25GE_RDMA_MACSEC		0xA223
#define HNAE3_DEV_ID_50GE_RDMA			0xA224
#define HNAE3_DEV_ID_50GE_RDMA_MACSEC		0xA225
#define HNAE3_DEV_ID_100G_RDMA_MACSEC		0xA226
#define HNAE3_DEV_ID_100G_VF			0xA22E
#define HNAE3_DEV_ID_100G_RDMA_DCB_PFC_VF	0xA22F

#define HNAE3_CLASS_NAME_SIZE 16

#define HNAE3_DEV_INITED_B			0x0
#define HNAE3_DEV_SUPPORT_ROCE_B		0x1
#define HNAE3_DEV_SUPPORT_DCB_B			0x2

#define HNAE3_DEV_SUPPORT_ROCE_DCB_BITS (BIT(HNAE3_DEV_SUPPORT_DCB_B) |\
		BIT(HNAE3_DEV_SUPPORT_ROCE_B))

#define hnae3_dev_roce_supported(hdev) \
	hnae_get_bit(hdev->ae_dev->flag, HNAE3_DEV_SUPPORT_ROCE_B)

#define hnae3_dev_dcb_supported(hdev) \
	hnae_get_bit(hdev->ae_dev->flag, HNAE3_DEV_SUPPORT_DCB_B)

#define ring_ptr_move_fw(ring, p) \
	((ring)->p = ((ring)->p + 1) % (ring)->desc_num)
#define ring_ptr_move_bw(ring, p) \
	((ring)->p = ((ring)->p - 1 + (ring)->desc_num) % (ring)->desc_num)

enum hns_desc_type {
	DESC_TYPE_SKB,
	DESC_TYPE_PAGE,
};

struct hnae3_handle;

struct hnae3_queue {
	void __iomem *io_base;
	struct hnae3_ae_algo *ae_algo;
	struct hnae3_handle *handle;
	int tqp_index;	/* index in a handle */
	u32 buf_size;	/* size for hnae_desc->addr, preset by AE */
	u16 desc_num;	/* total number of desc */
};

/*hnae3 loop mode*/
enum hnae3_loop {
	HNAE3_MAC_INTER_LOOP_MAC,
	HNAE3_MAC_INTER_LOOP_SERDES,
	HNAE3_MAC_INTER_LOOP_PHY,
	HNAE3_MAC_LOOP_NONE,
};

enum hnae3_client_type {
	HNAE3_CLIENT_KNIC,
	HNAE3_CLIENT_UNIC,
	HNAE3_CLIENT_ROCE,
};

enum hnae3_dev_type {
	HNAE3_DEV_KNIC,
	HNAE3_DEV_UNIC,
};

/* mac media type */
enum hnae3_media_type {
	HNAE3_MEDIA_TYPE_UNKNOWN,
	HNAE3_MEDIA_TYPE_FIBER,
	HNAE3_MEDIA_TYPE_COPPER,
	HNAE3_MEDIA_TYPE_BACKPLANE,
};

enum hnae3_reset_notify_type {
	HNAE3_UP_CLIENT,
	HNAE3_DOWN_CLIENT,
	HNAE3_INIT_CLIENT,
	HNAE3_UNINIT_CLIENT,
};

enum hnae3_reset_type {
	HNAE3_FUNC_RESET,
	HNAE3_CORE_RESET,
	HNAE3_GLOBAL_RESET,
	HNAE3_IMP_RESET,
	HNAE3_NONE_RESET,
};

struct hnae3_vector_info {
	u8 __iomem *io_addr;
	int vector;
};

#define HNAE3_RING_TYPE_B 0
#define HNAE3_RING_TYPE_TX 0
#define HNAE3_RING_TYPE_RX 1
#define HNAE3_RING_GL_IDX_S 0
#define HNAE3_RING_GL_IDX_M GENMASK(1, 0)
#define HNAE3_RING_GL_RX 0
#define HNAE3_RING_GL_TX 1

struct hnae3_ring_chain_node {
	struct hnae3_ring_chain_node *next;
	u32 tqp_index;
	u32 flag;
	u32 int_gl_idx;
};

#define HNAE3_IS_TX_RING(node) \
	(((node)->flag & (1 << HNAE3_RING_TYPE_B)) == HNAE3_RING_TYPE_TX)

struct hnae3_client_ops {
	int (*init_instance)(struct hnae3_handle *handle);
	void (*uninit_instance)(struct hnae3_handle *handle, bool reset);
	void (*link_status_change)(struct hnae3_handle *handle, bool state);
	int (*setup_tc)(struct hnae3_handle *handle, u8 tc);
	int (*reset_notify)(struct hnae3_handle *handle,
			    enum hnae3_reset_notify_type type);
};

#define HNAE3_CLIENT_NAME_LENGTH 16
struct hnae3_client {
	char name[HNAE3_CLIENT_NAME_LENGTH];
	u16 version;
	unsigned long state;
	enum hnae3_client_type type;
	const struct hnae3_client_ops *ops;
	struct list_head node;
};

struct hnae3_ae_dev {
	struct pci_dev *pdev;
	const struct hnae3_ae_ops *ops;
	struct list_head node;
	u32 flag;
	enum hnae3_dev_type dev_type;
	void *priv;
};

/* This struct defines the operation on the handle.
 *
 * init_ae_dev(): (mandatory)
 *   Get PF configure from pci_dev and initialize PF hardware
 * uninit_ae_dev()
 *   Disable PF device and release PF resource
 * register_client
 *   Register client to ae_dev
 * unregister_client()
 *   Unregister client from ae_dev
 * start()
 *   Enable the hardware
 * stop()
 *   Disable the hardware
 * get_status()
 *   Get the carrier state of the back channel of the handle, 1 for ok, 0 for
 *   non-ok
 * get_ksettings_an_result()
 *   Get negotiation status,speed and duplex
 * update_speed_duplex_h()
 *   Update hardware speed and duplex
 * get_media_type()
 *   Get media type of MAC
 * adjust_link()
 *   Adjust link status
 * set_loopback()
 *   Set loopback
 * set_promisc_mode
 *   Set promisc mode
 * set_mtu()
 *   set mtu
 * get_pauseparam()
 *   get tx and rx of pause frame use
 * set_pauseparam()
 *   set tx and rx of pause frame use
 * set_autoneg()
 *   set auto autonegotiation of pause frame use
 * get_autoneg()
 *   get auto autonegotiation of pause frame use
 * get_coalesce_usecs()
 *   get usecs to delay a TX interrupt after a packet is sent
 * get_rx_max_coalesced_frames()
 *   get Maximum number of packets to be sent before a TX interrupt.
 * set_coalesce_usecs()
 *   set usecs to delay a TX interrupt after a packet is sent
 * set_coalesce_frames()
 *   set Maximum number of packets to be sent before a TX interrupt.
 * get_mac_addr()
 *   get mac address
 * set_mac_addr()
 *   set mac address
 * add_uc_addr
 *   Add unicast addr to mac table
 * rm_uc_addr
 *   Remove unicast addr from mac table
 * set_mc_addr()
 *   Set multicast address
 * add_mc_addr
 *   Add multicast address to mac table
 * rm_mc_addr
 *   Remove multicast address from mac table
 * update_stats()
 *   Update Old network device statistics
 * get_ethtool_stats()
 *   Get ethtool network device statistics
 * get_strings()
 *   Get a set of strings that describe the requested objects
 * get_sset_count()
 *   Get number of strings that @get_strings will write
 * update_led_status()
 *   Update the led status
 * set_led_id()
 *   Set led id
 * get_regs()
 *   Get regs dump
 * get_regs_len()
 *   Get the len of the regs dump
 * get_rss_key_size()
 *   Get rss key size
 * get_rss_indir_size()
 *   Get rss indirection table size
 * get_rss()
 *   Get rss table
 * set_rss()
 *   Set rss table
 * get_tc_size()
 *   Get tc size of handle
 * get_vector()
 *   Get vector number and vector information
 * map_ring_to_vector()
 *   Map rings to vector
 * unmap_ring_from_vector()
 *   Unmap rings from vector
 * add_tunnel_udp()
 *   Add tunnel information to hardware
 * del_tunnel_udp()
 *   Delete tunnel information from hardware
 * reset_queue()
 *   Reset queue
 * get_fw_version()
 *   Get firmware version
 * get_mdix_mode()
 *   Get media typr of phy
 * enable_vlan_filter()
 *   Enable vlan filter
 * set_vlan_filter()
 *   Set vlan filter config of Ports
 * set_vf_vlan_filter()
 *   Set vlan filter config of vf
 * enable_hw_strip_rxvtag()
 *   Enable/disable hardware strip vlan tag of packets received
 */
struct hnae3_ae_ops {
	int (*init_ae_dev)(struct hnae3_ae_dev *ae_dev);
	void (*uninit_ae_dev)(struct hnae3_ae_dev *ae_dev);

	int (*init_client_instance)(struct hnae3_client *client,
				    struct hnae3_ae_dev *ae_dev);
	void (*uninit_client_instance)(struct hnae3_client *client,
				       struct hnae3_ae_dev *ae_dev);
	int (*start)(struct hnae3_handle *handle);
	void (*stop)(struct hnae3_handle *handle);
	int (*get_status)(struct hnae3_handle *handle);
	void (*get_ksettings_an_result)(struct hnae3_handle *handle,
					u8 *auto_neg, u32 *speed, u8 *duplex);

	int (*update_speed_duplex_h)(struct hnae3_handle *handle);
	int (*cfg_mac_speed_dup_h)(struct hnae3_handle *handle, int speed,
				   u8 duplex);

	void (*get_media_type)(struct hnae3_handle *handle, u8 *media_type);
	void (*adjust_link)(struct hnae3_handle *handle, int speed, int duplex);
	int (*set_loopback)(struct hnae3_handle *handle,
			    enum hnae3_loop loop_mode, bool en);

	void (*set_promisc_mode)(struct hnae3_handle *handle, u32 en);
	int (*set_mtu)(struct hnae3_handle *handle, int new_mtu);

	void (*get_pauseparam)(struct hnae3_handle *handle,
			       u32 *auto_neg, u32 *rx_en, u32 *tx_en);
	int (*set_pauseparam)(struct hnae3_handle *handle,
			      u32 auto_neg, u32 rx_en, u32 tx_en);

	int (*set_autoneg)(struct hnae3_handle *handle, bool enable);
	int (*get_autoneg)(struct hnae3_handle *handle);

	void (*get_coalesce_usecs)(struct hnae3_handle *handle,
				   u32 *tx_usecs, u32 *rx_usecs);
	void (*get_rx_max_coalesced_frames)(struct hnae3_handle *handle,
					    u32 *tx_frames, u32 *rx_frames);
	int (*set_coalesce_usecs)(struct hnae3_handle *handle, u32 timeout);
	int (*set_coalesce_frames)(struct hnae3_handle *handle,
				   u32 coalesce_frames);
	void (*get_coalesce_range)(struct hnae3_handle *handle,
				   u32 *tx_frames_low, u32 *rx_frames_low,
				   u32 *tx_frames_high, u32 *rx_frames_high,
				   u32 *tx_usecs_low, u32 *rx_usecs_low,
				   u32 *tx_usecs_high, u32 *rx_usecs_high);

	void (*get_mac_addr)(struct hnae3_handle *handle, u8 *p);
	int (*set_mac_addr)(struct hnae3_handle *handle, void *p);
	int (*add_uc_addr)(struct hnae3_handle *handle,
			   const unsigned char *addr);
	int (*rm_uc_addr)(struct hnae3_handle *handle,
			  const unsigned char *addr);
	int (*set_mc_addr)(struct hnae3_handle *handle, void *addr);
	int (*add_mc_addr)(struct hnae3_handle *handle,
			   const unsigned char *addr);
	int (*rm_mc_addr)(struct hnae3_handle *handle,
			  const unsigned char *addr);

	void (*set_tso_stats)(struct hnae3_handle *handle, int enable);
	void (*update_stats)(struct hnae3_handle *handle,
			     struct net_device_stats *net_stats);
	void (*get_stats)(struct hnae3_handle *handle, u64 *data);

	void (*get_strings)(struct hnae3_handle *handle,
			    u32 stringset, u8 *data);
	int (*get_sset_count)(struct hnae3_handle *handle, int stringset);

	void (*get_regs)(struct hnae3_handle *handle, u32 *version,
			 void *data);
	int (*get_regs_len)(struct hnae3_handle *handle);

	u32 (*get_rss_key_size)(struct hnae3_handle *handle);
	u32 (*get_rss_indir_size)(struct hnae3_handle *handle);
	int (*get_rss)(struct hnae3_handle *handle, u32 *indir, u8 *key,
		       u8 *hfunc);
	int (*set_rss)(struct hnae3_handle *handle, const u32 *indir,
		       const u8 *key, const u8 hfunc);
	int (*set_rss_tuple)(struct hnae3_handle *handle,
			     struct ethtool_rxnfc *cmd);
	int (*get_rss_tuple)(struct hnae3_handle *handle,
			     struct ethtool_rxnfc *cmd);

	int (*get_tc_size)(struct hnae3_handle *handle);

	int (*get_vector)(struct hnae3_handle *handle, u16 vector_num,
			  struct hnae3_vector_info *vector_info);
	int (*map_ring_to_vector)(struct hnae3_handle *handle,
				  int vector_num,
				  struct hnae3_ring_chain_node *vr_chain);
	int (*unmap_ring_from_vector)(struct hnae3_handle *handle,
				      int vector_num,
				      struct hnae3_ring_chain_node *vr_chain);

	int (*add_tunnel_udp)(struct hnae3_handle *handle, u16 port_num);
	int (*del_tunnel_udp)(struct hnae3_handle *handle, u16 port_num);

	void (*reset_queue)(struct hnae3_handle *handle, u16 queue_id);
	u32 (*get_fw_version)(struct hnae3_handle *handle);
	void (*get_mdix_mode)(struct hnae3_handle *handle,
			      u8 *tp_mdix_ctrl, u8 *tp_mdix);

	void (*enable_vlan_filter)(struct hnae3_handle *handle, bool enable);
	int (*set_vlan_filter)(struct hnae3_handle *handle, __be16 proto,
			       u16 vlan_id, bool is_kill);
	int (*set_vf_vlan_filter)(struct hnae3_handle *handle, int vfid,
				  u16 vlan, u8 qos, __be16 proto);
	int (*enable_hw_strip_rxvtag)(struct hnae3_handle *handle, bool enable);
	void (*reset_event)(struct hnae3_handle *handle,
			    enum hnae3_reset_type reset);
	void (*get_channels)(struct hnae3_handle *handle,
			     struct ethtool_channels *ch);
	void (*get_tqps_and_rss_info)(struct hnae3_handle *h,
				      u16 *free_tqps, u16 *max_rss_size);
	int (*set_channels)(struct hnae3_handle *handle, u32 new_tqps_num);
	void (*get_flowctrl_adv)(struct hnae3_handle *handle,
				 u32 *flowctrl_adv);
	int (*set_led_id)(struct hnae3_handle *handle,
			  enum ethtool_phys_id_state status);
};

struct hnae3_dcb_ops {
	/* IEEE 802.1Qaz std */
	int (*ieee_getets)(struct hnae3_handle *, struct ieee_ets *);
	int (*ieee_setets)(struct hnae3_handle *, struct ieee_ets *);
	int (*ieee_getpfc)(struct hnae3_handle *, struct ieee_pfc *);
	int (*ieee_setpfc)(struct hnae3_handle *, struct ieee_pfc *);

	/* DCBX configuration */
	u8   (*getdcbx)(struct hnae3_handle *);
	u8   (*setdcbx)(struct hnae3_handle *, u8);

	int (*map_update)(struct hnae3_handle *);
	int (*setup_tc)(struct hnae3_handle *, u8, u8 *);
};

struct hnae3_ae_algo {
	const struct hnae3_ae_ops *ops;
	struct list_head node;
	char name[HNAE3_CLASS_NAME_SIZE];
	const struct pci_device_id *pdev_id_table;
};

#define HNAE3_INT_NAME_LEN        (IFNAMSIZ + 16)
#define HNAE3_ITR_COUNTDOWN_START 100

struct hnae3_tc_info {
	u16	tqp_offset;	/* TQP offset from base TQP */
	u16	tqp_count;	/* Total TQPs */
	u8	tc;		/* TC index */
	bool	enable;		/* If this TC is enable or not */
};

#define HNAE3_MAX_TC		8
#define HNAE3_MAX_USER_PRIO	8
struct hnae3_knic_private_info {
	struct net_device *netdev; /* Set by KNIC client when init instance */
	u16 rss_size;		   /* Allocated RSS queues */
	u16 rx_buf_len;
	u16 num_desc;

	u8 num_tc;		   /* Total number of enabled TCs */
	u8 prio_tc[HNAE3_MAX_USER_PRIO];  /* TC indexed by prio */
	struct hnae3_tc_info tc_info[HNAE3_MAX_TC]; /* Idx of array is HW TC */

	u16 num_tqps;		  /* total number of TQPs in this handle */
	struct hnae3_queue **tqp;  /* array base of all TQPs in this instance */
	const struct hnae3_dcb_ops *dcb_ops;

	u16 int_rl_setting;
};

struct hnae3_roce_private_info {
	struct net_device *netdev;
	void __iomem *roce_io_base;
	int base_vector;
	int num_vectors;
};

struct hnae3_unic_private_info {
	struct net_device *netdev;
	u16 rx_buf_len;
	u16 num_desc;
	u16 num_tqps;	/* total number of tqps in this handle */
	struct hnae3_queue **tqp;  /* array base of all TQPs of this instance */
};

#define HNAE3_SUPPORT_MAC_LOOPBACK    BIT(0)
#define HNAE3_SUPPORT_PHY_LOOPBACK    BIT(1)
#define HNAE3_SUPPORT_SERDES_LOOPBACK BIT(2)
#define HNAE3_SUPPORT_VF	      BIT(3)

struct hnae3_handle {
	struct hnae3_client *client;
	struct pci_dev *pdev;
	void *priv;
	struct hnae3_ae_algo *ae_algo;  /* the class who provides this handle */
	u64 flags; /* Indicate the capabilities for this handle*/

	union {
		struct net_device *netdev; /* first member */
		struct hnae3_knic_private_info kinfo;
		struct hnae3_unic_private_info uinfo;
		struct hnae3_roce_private_info rinfo;
	};

	u32 numa_node_mask;	/* for multi-chip support */
};

#define hnae_set_field(origin, mask, shift, val) \
	do { \
		(origin) &= (~(mask)); \
		(origin) |= ((val) << (shift)) & (mask); \
	} while (0)
#define hnae_get_field(origin, mask, shift) (((origin) & (mask)) >> (shift))

#define hnae_set_bit(origin, shift, val) \
	hnae_set_field((origin), (0x1 << (shift)), (shift), (val))
#define hnae_get_bit(origin, shift) \
	hnae_get_field((origin), (0x1 << (shift)), (shift))

int hnae3_register_ae_dev(struct hnae3_ae_dev *ae_dev);
void hnae3_unregister_ae_dev(struct hnae3_ae_dev *ae_dev);

void hnae3_unregister_ae_algo(struct hnae3_ae_algo *ae_algo);
int hnae3_register_ae_algo(struct hnae3_ae_algo *ae_algo);

void hnae3_unregister_client(struct hnae3_client *client);
int hnae3_register_client(struct hnae3_client *client);
#endif
