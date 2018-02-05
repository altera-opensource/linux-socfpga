/*
 * Copyright (c) 2011-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _NVME_H
#define _NVME_H

#include <linux/nvme.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/kref.h>
#include <linux/blk-mq.h>
#include <linux/lightnvm.h>
#include <linux/sed-opal.h>

extern unsigned int nvme_io_timeout;
#define NVME_IO_TIMEOUT	(nvme_io_timeout * HZ)

extern unsigned int admin_timeout;
#define ADMIN_TIMEOUT	(admin_timeout * HZ)

#define NVME_DEFAULT_KATO	5
#define NVME_KATO_GRACE		10

extern struct workqueue_struct *nvme_wq;
extern struct workqueue_struct *nvme_reset_wq;
extern struct workqueue_struct *nvme_delete_wq;

enum {
	NVME_NS_LBA		= 0,
	NVME_NS_LIGHTNVM	= 1,
};

/*
 * List of workarounds for devices that required behavior not specified in
 * the standard.
 */
enum nvme_quirks {
	/*
	 * Prefers I/O aligned to a stripe size specified in a vendor
	 * specific Identify field.
	 */
	NVME_QUIRK_STRIPE_SIZE			= (1 << 0),

	/*
	 * The controller doesn't handle Identify value others than 0 or 1
	 * correctly.
	 */
	NVME_QUIRK_IDENTIFY_CNS			= (1 << 1),

	/*
	 * The controller deterministically returns O's on reads to
	 * logical blocks that deallocate was called on.
	 */
	NVME_QUIRK_DEALLOCATE_ZEROES		= (1 << 2),

	/*
	 * The controller needs a delay before starts checking the device
	 * readiness, which is done by reading the NVME_CSTS_RDY bit.
	 */
	NVME_QUIRK_DELAY_BEFORE_CHK_RDY		= (1 << 3),

	/*
	 * APST should not be used.
	 */
	NVME_QUIRK_NO_APST			= (1 << 4),

	/*
	 * The deepest sleep state should not be used.
	 */
	NVME_QUIRK_NO_DEEPEST_PS		= (1 << 5),

	/*
	 * Supports the LighNVM command set if indicated in vs[1].
	 */
	NVME_QUIRK_LIGHTNVM			= (1 << 6),
};

/*
 * Common request structure for NVMe passthrough.  All drivers must have
 * this structure as the first member of their request-private data.
 */
struct nvme_request {
	struct nvme_command	*cmd;
	union nvme_result	result;
	u8			retries;
	u8			flags;
	u16			status;
};

/*
 * Mark a bio as coming in through the mpath node.
 */
#define REQ_NVME_MPATH		REQ_DRV

enum {
	NVME_REQ_CANCELLED		= (1 << 0),
};

static inline struct nvme_request *nvme_req(struct request *req)
{
	return blk_mq_rq_to_pdu(req);
}

/* The below value is the specific amount of delay needed before checking
 * readiness in case of the PCI_DEVICE(0x1c58, 0x0003), which needs the
 * NVME_QUIRK_DELAY_BEFORE_CHK_RDY quirk enabled. The value (in ms) was
 * found empirically.
 */
#define NVME_QUIRK_DELAY_AMOUNT		2300

enum nvme_ctrl_state {
	NVME_CTRL_NEW,
	NVME_CTRL_LIVE,
	NVME_CTRL_ADMIN_ONLY,    /* Only admin queue live */
	NVME_CTRL_RESETTING,
	NVME_CTRL_RECONNECTING,
	NVME_CTRL_DELETING,
	NVME_CTRL_DEAD,
};

struct nvme_ctrl {
	enum nvme_ctrl_state state;
	bool identified;
	spinlock_t lock;
	const struct nvme_ctrl_ops *ops;
	struct request_queue *admin_q;
	struct request_queue *connect_q;
	struct device *dev;
	int instance;
	struct blk_mq_tag_set *tagset;
	struct blk_mq_tag_set *admin_tagset;
	struct list_head namespaces;
	struct mutex namespaces_mutex;
	struct device ctrl_device;
	struct device *device;	/* char device */
	struct cdev cdev;
	struct work_struct reset_work;
	struct work_struct delete_work;

	struct nvme_subsystem *subsys;
	struct list_head subsys_entry;

	struct opal_dev *opal_dev;

	char name[12];
	u16 cntlid;

	u32 ctrl_config;
	u16 mtfa;
	u32 queue_count;

	u64 cap;
	u32 page_size;
	u32 max_hw_sectors;
	u16 oncs;
	u16 oacs;
	u16 nssa;
	u16 nr_streams;
	atomic_t abort_limit;
	u8 vwc;
	u32 vs;
	u32 sgls;
	u16 kas;
	u8 npss;
	u8 apsta;
	u32 aen_result;
	unsigned int shutdown_timeout;
	unsigned int kato;
	bool subsystem;
	unsigned long quirks;
	struct nvme_id_power_state psd[32];
	struct nvme_effects_log *effects;
	struct work_struct scan_work;
	struct work_struct async_event_work;
	struct delayed_work ka_work;
	struct work_struct fw_act_work;

	/* Power saving configuration */
	u64 ps_max_latency_us;
	bool apst_enabled;

	/* PCIe only: */
	u32 hmpre;
	u32 hmmin;
	u32 hmminds;
	u16 hmmaxd;

	/* Fabrics only */
	u16 sqsize;
	u32 ioccsz;
	u32 iorcsz;
	u16 icdoff;
	u16 maxcmd;
	int nr_reconnects;
	struct nvmf_ctrl_options *opts;
};

struct nvme_subsystem {
	int			instance;
	struct device		dev;
	/*
	 * Because we unregister the device on the last put we need
	 * a separate refcount.
	 */
	struct kref		ref;
	struct list_head	entry;
	struct mutex		lock;
	struct list_head	ctrls;
	struct list_head	nsheads;
	char			subnqn[NVMF_NQN_SIZE];
	char			serial[20];
	char			model[40];
	char			firmware_rev[8];
	u8			cmic;
	u16			vendor_id;
	struct ida		ns_ida;
};

/*
 * Container structure for uniqueue namespace identifiers.
 */
struct nvme_ns_ids {
	u8	eui64[8];
	u8	nguid[16];
	uuid_t	uuid;
};

/*
 * Anchor structure for namespaces.  There is one for each namespace in a
 * NVMe subsystem that any of our controllers can see, and the namespace
 * structure for each controller is chained of it.  For private namespaces
 * there is a 1:1 relation to our namespace structures, that is ->list
 * only ever has a single entry for private namespaces.
 */
struct nvme_ns_head {
#ifdef CONFIG_NVME_MULTIPATH
	struct gendisk		*disk;
	struct nvme_ns __rcu	*current_path;
	struct bio_list		requeue_list;
	spinlock_t		requeue_lock;
	struct work_struct	requeue_work;
#endif
	struct list_head	list;
	struct srcu_struct      srcu;
	struct nvme_subsystem	*subsys;
	unsigned		ns_id;
	struct nvme_ns_ids	ids;
	struct list_head	entry;
	struct kref		ref;
	int			instance;
};

struct nvme_ns {
	struct list_head list;

	struct nvme_ctrl *ctrl;
	struct request_queue *queue;
	struct gendisk *disk;
	struct list_head siblings;
	struct nvm_dev *ndev;
	struct kref kref;
	struct nvme_ns_head *head;

	int lba_shift;
	u16 ms;
	u16 sgs;
	u32 sws;
	bool ext;
	u8 pi_type;
	unsigned long flags;
#define NVME_NS_REMOVING 0
#define NVME_NS_DEAD     1
	u16 noiob;
};

struct nvme_ctrl_ops {
	const char *name;
	struct module *module;
	unsigned int flags;
#define NVME_F_FABRICS			(1 << 0)
#define NVME_F_METADATA_SUPPORTED	(1 << 1)
	int (*reg_read32)(struct nvme_ctrl *ctrl, u32 off, u32 *val);
	int (*reg_write32)(struct nvme_ctrl *ctrl, u32 off, u32 val);
	int (*reg_read64)(struct nvme_ctrl *ctrl, u32 off, u64 *val);
	void (*free_ctrl)(struct nvme_ctrl *ctrl);
	void (*submit_async_event)(struct nvme_ctrl *ctrl);
	void (*delete_ctrl)(struct nvme_ctrl *ctrl);
	int (*get_address)(struct nvme_ctrl *ctrl, char *buf, int size);
	int (*reinit_request)(void *data, struct request *rq);
};

static inline bool nvme_ctrl_ready(struct nvme_ctrl *ctrl)
{
	u32 val = 0;

	if (ctrl->ops->reg_read32(ctrl, NVME_REG_CSTS, &val))
		return false;
	return val & NVME_CSTS_RDY;
}

static inline int nvme_reset_subsystem(struct nvme_ctrl *ctrl)
{
	if (!ctrl->subsystem)
		return -ENOTTY;
	return ctrl->ops->reg_write32(ctrl, NVME_REG_NSSR, 0x4E564D65);
}

static inline u64 nvme_block_nr(struct nvme_ns *ns, sector_t sector)
{
	return (sector >> (ns->lba_shift - 9));
}

static inline void nvme_cleanup_cmd(struct request *req)
{
	if (req->rq_flags & RQF_SPECIAL_PAYLOAD) {
		kfree(page_address(req->special_vec.bv_page) +
		      req->special_vec.bv_offset);
	}
}

static inline void nvme_end_request(struct request *req, __le16 status,
		union nvme_result result)
{
	struct nvme_request *rq = nvme_req(req);

	rq->status = le16_to_cpu(status) >> 1;
	rq->result = result;
	blk_mq_complete_request(req);
}

static inline void nvme_get_ctrl(struct nvme_ctrl *ctrl)
{
	get_device(ctrl->device);
}

static inline void nvme_put_ctrl(struct nvme_ctrl *ctrl)
{
	put_device(ctrl->device);
}

void nvme_complete_rq(struct request *req);
void nvme_cancel_request(struct request *req, void *data, bool reserved);
bool nvme_change_ctrl_state(struct nvme_ctrl *ctrl,
		enum nvme_ctrl_state new_state);
int nvme_disable_ctrl(struct nvme_ctrl *ctrl, u64 cap);
int nvme_enable_ctrl(struct nvme_ctrl *ctrl, u64 cap);
int nvme_shutdown_ctrl(struct nvme_ctrl *ctrl);
int nvme_init_ctrl(struct nvme_ctrl *ctrl, struct device *dev,
		const struct nvme_ctrl_ops *ops, unsigned long quirks);
void nvme_uninit_ctrl(struct nvme_ctrl *ctrl);
void nvme_start_ctrl(struct nvme_ctrl *ctrl);
void nvme_stop_ctrl(struct nvme_ctrl *ctrl);
void nvme_put_ctrl(struct nvme_ctrl *ctrl);
int nvme_init_identify(struct nvme_ctrl *ctrl);

void nvme_queue_scan(struct nvme_ctrl *ctrl);
void nvme_remove_namespaces(struct nvme_ctrl *ctrl);

int nvme_sec_submit(void *data, u16 spsp, u8 secp, void *buffer, size_t len,
		bool send);

void nvme_complete_async_event(struct nvme_ctrl *ctrl, __le16 status,
		union nvme_result *res);

void nvme_stop_queues(struct nvme_ctrl *ctrl);
void nvme_start_queues(struct nvme_ctrl *ctrl);
void nvme_kill_queues(struct nvme_ctrl *ctrl);
void nvme_unfreeze(struct nvme_ctrl *ctrl);
void nvme_wait_freeze(struct nvme_ctrl *ctrl);
void nvme_wait_freeze_timeout(struct nvme_ctrl *ctrl, long timeout);
void nvme_start_freeze(struct nvme_ctrl *ctrl);
int nvme_reinit_tagset(struct nvme_ctrl *ctrl, struct blk_mq_tag_set *set);

#define NVME_QID_ANY -1
struct request *nvme_alloc_request(struct request_queue *q,
		struct nvme_command *cmd, blk_mq_req_flags_t flags, int qid);
blk_status_t nvme_setup_cmd(struct nvme_ns *ns, struct request *req,
		struct nvme_command *cmd);
int nvme_submit_sync_cmd(struct request_queue *q, struct nvme_command *cmd,
		void *buf, unsigned bufflen);
int __nvme_submit_sync_cmd(struct request_queue *q, struct nvme_command *cmd,
		union nvme_result *result, void *buffer, unsigned bufflen,
		unsigned timeout, int qid, int at_head,
		blk_mq_req_flags_t flags);
int nvme_set_queue_count(struct nvme_ctrl *ctrl, int *count);
void nvme_start_keep_alive(struct nvme_ctrl *ctrl);
void nvme_stop_keep_alive(struct nvme_ctrl *ctrl);
int nvme_reset_ctrl(struct nvme_ctrl *ctrl);
int nvme_reset_ctrl_sync(struct nvme_ctrl *ctrl);
int nvme_delete_ctrl(struct nvme_ctrl *ctrl);
int nvme_delete_ctrl_sync(struct nvme_ctrl *ctrl);

extern const struct attribute_group nvme_ns_id_attr_group;
extern const struct block_device_operations nvme_ns_head_ops;

#ifdef CONFIG_NVME_MULTIPATH
void nvme_failover_req(struct request *req);
bool nvme_req_needs_failover(struct request *req, blk_status_t error);
void nvme_kick_requeue_lists(struct nvme_ctrl *ctrl);
int nvme_mpath_alloc_disk(struct nvme_ctrl *ctrl,struct nvme_ns_head *head);
void nvme_mpath_add_disk(struct nvme_ns_head *head);
void nvme_mpath_add_disk_links(struct nvme_ns *ns);
void nvme_mpath_remove_disk(struct nvme_ns_head *head);
void nvme_mpath_remove_disk_links(struct nvme_ns *ns);

static inline void nvme_mpath_clear_current_path(struct nvme_ns *ns)
{
	struct nvme_ns_head *head = ns->head;

	if (head && ns == srcu_dereference(head->current_path, &head->srcu))
		rcu_assign_pointer(head->current_path, NULL);
}
struct nvme_ns *nvme_find_path(struct nvme_ns_head *head);

static inline void nvme_mpath_check_last_path(struct nvme_ns *ns)
{
	struct nvme_ns_head *head = ns->head;

	if (head->disk && list_empty(&head->list))
		kblockd_schedule_work(&head->requeue_work);
}

#else
static inline void nvme_failover_req(struct request *req)
{
}
static inline bool nvme_req_needs_failover(struct request *req,
					   blk_status_t error)
{
	return false;
}
static inline void nvme_kick_requeue_lists(struct nvme_ctrl *ctrl)
{
}
static inline int nvme_mpath_alloc_disk(struct nvme_ctrl *ctrl,
		struct nvme_ns_head *head)
{
	return 0;
}
static inline void nvme_mpath_add_disk(struct nvme_ns_head *head)
{
}
static inline void nvme_mpath_remove_disk(struct nvme_ns_head *head)
{
}
static inline void nvme_mpath_add_disk_links(struct nvme_ns *ns)
{
}
static inline void nvme_mpath_remove_disk_links(struct nvme_ns *ns)
{
}
static inline void nvme_mpath_clear_current_path(struct nvme_ns *ns)
{
}
static inline void nvme_mpath_check_last_path(struct nvme_ns *ns)
{
}
#endif /* CONFIG_NVME_MULTIPATH */

#ifdef CONFIG_NVM
int nvme_nvm_register(struct nvme_ns *ns, char *disk_name, int node);
void nvme_nvm_unregister(struct nvme_ns *ns);
int nvme_nvm_register_sysfs(struct nvme_ns *ns);
void nvme_nvm_unregister_sysfs(struct nvme_ns *ns);
int nvme_nvm_ioctl(struct nvme_ns *ns, unsigned int cmd, unsigned long arg);
#else
static inline int nvme_nvm_register(struct nvme_ns *ns, char *disk_name,
				    int node)
{
	return 0;
}

static inline void nvme_nvm_unregister(struct nvme_ns *ns) {};
static inline int nvme_nvm_register_sysfs(struct nvme_ns *ns)
{
	return 0;
}
static inline void nvme_nvm_unregister_sysfs(struct nvme_ns *ns) {};
static inline int nvme_nvm_ioctl(struct nvme_ns *ns, unsigned int cmd,
							unsigned long arg)
{
	return -ENOTTY;
}
#endif /* CONFIG_NVM */

static inline struct nvme_ns *nvme_get_ns_from_dev(struct device *dev)
{
	return dev_to_disk(dev)->private_data;
}

int __init nvme_core_init(void);
void nvme_core_exit(void);

#endif /* _NVME_H */
