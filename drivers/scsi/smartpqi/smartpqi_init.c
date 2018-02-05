/*
 *    driver for Microsemi PQI-based storage controllers
 *    Copyright (c) 2016-2017 Microsemi Corporation
 *    Copyright (c) 2016 PMC-Sierra, Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; version 2 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *    NON INFRINGEMENT.  See the GNU General Public License for more details.
 *
 *    Questions/Comments/Bugfixes to esc.storagedev@microsemi.com
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/reboot.h>
#include <linux/cciss_ioctl.h>
#include <linux/blk-mq-pci.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_transport_sas.h>
#include <asm/unaligned.h>
#include "smartpqi.h"
#include "smartpqi_sis.h"

#if !defined(BUILD_TIMESTAMP)
#define BUILD_TIMESTAMP
#endif

#define DRIVER_VERSION		"1.1.2-126"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		1
#define DRIVER_RELEASE		2
#define DRIVER_REVISION		126

#define DRIVER_NAME		"Microsemi PQI Driver (v" \
				DRIVER_VERSION BUILD_TIMESTAMP ")"
#define DRIVER_NAME_SHORT	"smartpqi"

#define PQI_EXTRA_SGL_MEMORY	(12 * sizeof(struct pqi_sg_descriptor))

MODULE_AUTHOR("Microsemi");
MODULE_DESCRIPTION("Driver for Microsemi Smart Family Controller version "
	DRIVER_VERSION);
MODULE_SUPPORTED_DEVICE("Microsemi Smart Family Controllers");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

static void pqi_take_ctrl_offline(struct pqi_ctrl_info *ctrl_info);
static void pqi_ctrl_offline_worker(struct work_struct *work);
static void pqi_retry_raid_bypass_requests(struct pqi_ctrl_info *ctrl_info);
static int pqi_scan_scsi_devices(struct pqi_ctrl_info *ctrl_info);
static void pqi_scan_start(struct Scsi_Host *shost);
static void pqi_start_io(struct pqi_ctrl_info *ctrl_info,
	struct pqi_queue_group *queue_group, enum pqi_io_path path,
	struct pqi_io_request *io_request);
static int pqi_submit_raid_request_synchronous(struct pqi_ctrl_info *ctrl_info,
	struct pqi_iu_header *request, unsigned int flags,
	struct pqi_raid_error_info *error_info, unsigned long timeout_msecs);
static int pqi_aio_submit_io(struct pqi_ctrl_info *ctrl_info,
	struct scsi_cmnd *scmd, u32 aio_handle, u8 *cdb,
	unsigned int cdb_length, struct pqi_queue_group *queue_group,
	struct pqi_encryption_info *encryption_info, bool raid_bypass);

/* for flags argument to pqi_submit_raid_request_synchronous() */
#define PQI_SYNC_FLAGS_INTERRUPTABLE	0x1

static struct scsi_transport_template *pqi_sas_transport_template;

static atomic_t pqi_controller_count = ATOMIC_INIT(0);

enum pqi_lockup_action {
	NONE,
	REBOOT,
	PANIC
};

static enum pqi_lockup_action pqi_lockup_action = NONE;

static struct {
	enum pqi_lockup_action	action;
	char			*name;
} pqi_lockup_actions[] = {
	{
		.action = NONE,
		.name = "none",
	},
	{
		.action = REBOOT,
		.name = "reboot",
	},
	{
		.action = PANIC,
		.name = "panic",
	},
};

static unsigned int pqi_supported_event_types[] = {
	PQI_EVENT_TYPE_HOTPLUG,
	PQI_EVENT_TYPE_HARDWARE,
	PQI_EVENT_TYPE_PHYSICAL_DEVICE,
	PQI_EVENT_TYPE_LOGICAL_DEVICE,
	PQI_EVENT_TYPE_AIO_STATE_CHANGE,
	PQI_EVENT_TYPE_AIO_CONFIG_CHANGE,
};

static int pqi_disable_device_id_wildcards;
module_param_named(disable_device_id_wildcards,
	pqi_disable_device_id_wildcards, int, 0644);
MODULE_PARM_DESC(disable_device_id_wildcards,
	"Disable device ID wildcards.");

static int pqi_disable_heartbeat;
module_param_named(disable_heartbeat,
	pqi_disable_heartbeat, int, 0644);
MODULE_PARM_DESC(disable_heartbeat,
	"Disable heartbeat.");

static int pqi_disable_ctrl_shutdown;
module_param_named(disable_ctrl_shutdown,
	pqi_disable_ctrl_shutdown, int, 0644);
MODULE_PARM_DESC(disable_ctrl_shutdown,
	"Disable controller shutdown when controller locked up.");

static char *pqi_lockup_action_param;
module_param_named(lockup_action,
	pqi_lockup_action_param, charp, 0644);
MODULE_PARM_DESC(lockup_action, "Action to take when controller locked up.\n"
	"\t\tSupported: none, reboot, panic\n"
	"\t\tDefault: none");

static char *raid_levels[] = {
	"RAID-0",
	"RAID-4",
	"RAID-1(1+0)",
	"RAID-5",
	"RAID-5+1",
	"RAID-ADG",
	"RAID-1(ADM)",
};

static char *pqi_raid_level_to_string(u8 raid_level)
{
	if (raid_level < ARRAY_SIZE(raid_levels))
		return raid_levels[raid_level];

	return "RAID UNKNOWN";
}

#define SA_RAID_0		0
#define SA_RAID_4		1
#define SA_RAID_1		2	/* also used for RAID 10 */
#define SA_RAID_5		3	/* also used for RAID 50 */
#define SA_RAID_51		4
#define SA_RAID_6		5	/* also used for RAID 60 */
#define SA_RAID_ADM		6	/* also used for RAID 1+0 ADM */
#define SA_RAID_MAX		SA_RAID_ADM
#define SA_RAID_UNKNOWN		0xff

static inline void pqi_scsi_done(struct scsi_cmnd *scmd)
{
	pqi_prep_for_scsi_done(scmd);
	scmd->scsi_done(scmd);
}

static inline bool pqi_scsi3addr_equal(u8 *scsi3addr1, u8 *scsi3addr2)
{
	return memcmp(scsi3addr1, scsi3addr2, 8) == 0;
}

static inline struct pqi_ctrl_info *shost_to_hba(struct Scsi_Host *shost)
{
	void *hostdata = shost_priv(shost);

	return *((struct pqi_ctrl_info **)hostdata);
}

static inline bool pqi_is_logical_device(struct pqi_scsi_dev *device)
{
	return !device->is_physical_device;
}

static inline bool pqi_is_external_raid_addr(u8 *scsi3addr)
{
	return scsi3addr[2] != 0;
}

static inline bool pqi_ctrl_offline(struct pqi_ctrl_info *ctrl_info)
{
	return !ctrl_info->controller_online;
}

static inline void pqi_check_ctrl_health(struct pqi_ctrl_info *ctrl_info)
{
	if (ctrl_info->controller_online)
		if (!sis_is_firmware_running(ctrl_info))
			pqi_take_ctrl_offline(ctrl_info);
}

static inline bool pqi_is_hba_lunid(u8 *scsi3addr)
{
	return pqi_scsi3addr_equal(scsi3addr, RAID_CTLR_LUNID);
}

static inline enum pqi_ctrl_mode pqi_get_ctrl_mode(
	struct pqi_ctrl_info *ctrl_info)
{
	return sis_read_driver_scratch(ctrl_info);
}

static inline void pqi_save_ctrl_mode(struct pqi_ctrl_info *ctrl_info,
	enum pqi_ctrl_mode mode)
{
	sis_write_driver_scratch(ctrl_info, mode);
}

static inline void pqi_ctrl_block_requests(struct pqi_ctrl_info *ctrl_info)
{
	ctrl_info->block_requests = true;
	scsi_block_requests(ctrl_info->scsi_host);
}

static inline void pqi_ctrl_unblock_requests(struct pqi_ctrl_info *ctrl_info)
{
	ctrl_info->block_requests = false;
	wake_up_all(&ctrl_info->block_requests_wait);
	pqi_retry_raid_bypass_requests(ctrl_info);
	scsi_unblock_requests(ctrl_info->scsi_host);
}

static inline bool pqi_ctrl_blocked(struct pqi_ctrl_info *ctrl_info)
{
	return ctrl_info->block_requests;
}

static unsigned long pqi_wait_if_ctrl_blocked(struct pqi_ctrl_info *ctrl_info,
	unsigned long timeout_msecs)
{
	unsigned long remaining_msecs;

	if (!pqi_ctrl_blocked(ctrl_info))
		return timeout_msecs;

	atomic_inc(&ctrl_info->num_blocked_threads);

	if (timeout_msecs == NO_TIMEOUT) {
		wait_event(ctrl_info->block_requests_wait,
			!pqi_ctrl_blocked(ctrl_info));
		remaining_msecs = timeout_msecs;
	} else {
		unsigned long remaining_jiffies;

		remaining_jiffies =
			wait_event_timeout(ctrl_info->block_requests_wait,
				!pqi_ctrl_blocked(ctrl_info),
				msecs_to_jiffies(timeout_msecs));
		remaining_msecs = jiffies_to_msecs(remaining_jiffies);
	}

	atomic_dec(&ctrl_info->num_blocked_threads);

	return remaining_msecs;
}

static inline void pqi_ctrl_busy(struct pqi_ctrl_info *ctrl_info)
{
	atomic_inc(&ctrl_info->num_busy_threads);
}

static inline void pqi_ctrl_unbusy(struct pqi_ctrl_info *ctrl_info)
{
	atomic_dec(&ctrl_info->num_busy_threads);
}

static inline void pqi_ctrl_wait_until_quiesced(struct pqi_ctrl_info *ctrl_info)
{
	while (atomic_read(&ctrl_info->num_busy_threads) >
		atomic_read(&ctrl_info->num_blocked_threads))
		usleep_range(1000, 2000);
}

static inline bool pqi_device_offline(struct pqi_scsi_dev *device)
{
	return device->device_offline;
}

static inline void pqi_device_reset_start(struct pqi_scsi_dev *device)
{
	device->in_reset = true;
}

static inline void pqi_device_reset_done(struct pqi_scsi_dev *device)
{
	device->in_reset = false;
}

static inline bool pqi_device_in_reset(struct pqi_scsi_dev *device)
{
	return device->in_reset;
}

static inline void pqi_schedule_rescan_worker_with_delay(
	struct pqi_ctrl_info *ctrl_info, unsigned long delay)
{
	if (pqi_ctrl_offline(ctrl_info))
		return;

	schedule_delayed_work(&ctrl_info->rescan_work, delay);
}

static inline void pqi_schedule_rescan_worker(struct pqi_ctrl_info *ctrl_info)
{
	pqi_schedule_rescan_worker_with_delay(ctrl_info, 0);
}

#define PQI_RESCAN_WORK_DELAY  (10 * HZ)

static inline void pqi_schedule_rescan_worker_delayed(
	struct pqi_ctrl_info *ctrl_info)
{
	pqi_schedule_rescan_worker_with_delay(ctrl_info, PQI_RESCAN_WORK_DELAY);
}

static inline void pqi_cancel_rescan_worker(struct pqi_ctrl_info *ctrl_info)
{
	cancel_delayed_work_sync(&ctrl_info->rescan_work);
}

static inline u32 pqi_read_heartbeat_counter(struct pqi_ctrl_info *ctrl_info)
{
	if (!ctrl_info->heartbeat_counter)
		return 0;

	return readl(ctrl_info->heartbeat_counter);
}

static int pqi_map_single(struct pci_dev *pci_dev,
	struct pqi_sg_descriptor *sg_descriptor, void *buffer,
	size_t buffer_length, int data_direction)
{
	dma_addr_t bus_address;

	if (!buffer || buffer_length == 0 || data_direction == PCI_DMA_NONE)
		return 0;

	bus_address = pci_map_single(pci_dev, buffer, buffer_length,
		data_direction);
	if (pci_dma_mapping_error(pci_dev, bus_address))
		return -ENOMEM;

	put_unaligned_le64((u64)bus_address, &sg_descriptor->address);
	put_unaligned_le32(buffer_length, &sg_descriptor->length);
	put_unaligned_le32(CISS_SG_LAST, &sg_descriptor->flags);

	return 0;
}

static void pqi_pci_unmap(struct pci_dev *pci_dev,
	struct pqi_sg_descriptor *descriptors, int num_descriptors,
	int data_direction)
{
	int i;

	if (data_direction == PCI_DMA_NONE)
		return;

	for (i = 0; i < num_descriptors; i++)
		pci_unmap_single(pci_dev,
			(dma_addr_t)get_unaligned_le64(&descriptors[i].address),
			get_unaligned_le32(&descriptors[i].length),
			data_direction);
}

static int pqi_build_raid_path_request(struct pqi_ctrl_info *ctrl_info,
	struct pqi_raid_path_request *request, u8 cmd,
	u8 *scsi3addr, void *buffer, size_t buffer_length,
	u16 vpd_page, int *pci_direction)
{
	u8 *cdb;
	int pci_dir;

	memset(request, 0, sizeof(*request));

	request->header.iu_type = PQI_REQUEST_IU_RAID_PATH_IO;
	put_unaligned_le16(offsetof(struct pqi_raid_path_request,
		sg_descriptors[1]) - PQI_REQUEST_HEADER_LENGTH,
		&request->header.iu_length);
	put_unaligned_le32(buffer_length, &request->buffer_length);
	memcpy(request->lun_number, scsi3addr, sizeof(request->lun_number));
	request->task_attribute = SOP_TASK_ATTRIBUTE_SIMPLE;
	request->additional_cdb_bytes_usage = SOP_ADDITIONAL_CDB_BYTES_0;

	cdb = request->cdb;

	switch (cmd) {
	case INQUIRY:
		request->data_direction = SOP_READ_FLAG;
		cdb[0] = INQUIRY;
		if (vpd_page & VPD_PAGE) {
			cdb[1] = 0x1;
			cdb[2] = (u8)vpd_page;
		}
		cdb[4] = (u8)buffer_length;
		break;
	case CISS_REPORT_LOG:
	case CISS_REPORT_PHYS:
		request->data_direction = SOP_READ_FLAG;
		cdb[0] = cmd;
		if (cmd == CISS_REPORT_PHYS)
			cdb[1] = CISS_REPORT_PHYS_EXTENDED;
		else
			cdb[1] = CISS_REPORT_LOG_EXTENDED;
		put_unaligned_be32(buffer_length, &cdb[6]);
		break;
	case CISS_GET_RAID_MAP:
		request->data_direction = SOP_READ_FLAG;
		cdb[0] = CISS_READ;
		cdb[1] = CISS_GET_RAID_MAP;
		put_unaligned_be32(buffer_length, &cdb[6]);
		break;
	case SA_FLUSH_CACHE:
		request->data_direction = SOP_WRITE_FLAG;
		cdb[0] = BMIC_WRITE;
		cdb[6] = BMIC_FLUSH_CACHE;
		put_unaligned_be16(buffer_length, &cdb[7]);
		break;
	case BMIC_IDENTIFY_CONTROLLER:
	case BMIC_IDENTIFY_PHYSICAL_DEVICE:
		request->data_direction = SOP_READ_FLAG;
		cdb[0] = BMIC_READ;
		cdb[6] = cmd;
		put_unaligned_be16(buffer_length, &cdb[7]);
		break;
	case BMIC_WRITE_HOST_WELLNESS:
		request->data_direction = SOP_WRITE_FLAG;
		cdb[0] = BMIC_WRITE;
		cdb[6] = cmd;
		put_unaligned_be16(buffer_length, &cdb[7]);
		break;
	default:
		dev_err(&ctrl_info->pci_dev->dev, "unknown command 0x%c\n",
			cmd);
		break;
	}

	switch (request->data_direction) {
	case SOP_READ_FLAG:
		pci_dir = PCI_DMA_FROMDEVICE;
		break;
	case SOP_WRITE_FLAG:
		pci_dir = PCI_DMA_TODEVICE;
		break;
	case SOP_NO_DIRECTION_FLAG:
		pci_dir = PCI_DMA_NONE;
		break;
	default:
		pci_dir = PCI_DMA_BIDIRECTIONAL;
		break;
	}

	*pci_direction = pci_dir;

	return pqi_map_single(ctrl_info->pci_dev, &request->sg_descriptors[0],
		buffer, buffer_length, pci_dir);
}

static inline void pqi_reinit_io_request(struct pqi_io_request *io_request)
{
	io_request->scmd = NULL;
	io_request->status = 0;
	io_request->error_info = NULL;
	io_request->raid_bypass = false;
}

static struct pqi_io_request *pqi_alloc_io_request(
	struct pqi_ctrl_info *ctrl_info)
{
	struct pqi_io_request *io_request;
	u16 i = ctrl_info->next_io_request_slot;	/* benignly racy */

	while (1) {
		io_request = &ctrl_info->io_request_pool[i];
		if (atomic_inc_return(&io_request->refcount) == 1)
			break;
		atomic_dec(&io_request->refcount);
		i = (i + 1) % ctrl_info->max_io_slots;
	}

	/* benignly racy */
	ctrl_info->next_io_request_slot = (i + 1) % ctrl_info->max_io_slots;

	pqi_reinit_io_request(io_request);

	return io_request;
}

static void pqi_free_io_request(struct pqi_io_request *io_request)
{
	atomic_dec(&io_request->refcount);
}

static int pqi_identify_controller(struct pqi_ctrl_info *ctrl_info,
	struct bmic_identify_controller *buffer)
{
	int rc;
	int pci_direction;
	struct pqi_raid_path_request request;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		BMIC_IDENTIFY_CONTROLLER, RAID_CTLR_LUNID, buffer,
		sizeof(*buffer), 0, &pci_direction);
	if (rc)
		return rc;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header, 0,
		NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	return rc;
}

static int pqi_scsi_inquiry(struct pqi_ctrl_info *ctrl_info,
	u8 *scsi3addr, u16 vpd_page, void *buffer, size_t buffer_length)
{
	int rc;
	int pci_direction;
	struct pqi_raid_path_request request;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		INQUIRY, scsi3addr, buffer, buffer_length, vpd_page,
		&pci_direction);
	if (rc)
		return rc;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header, 0,
		NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	return rc;
}

static int pqi_identify_physical_device(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device,
	struct bmic_identify_physical_device *buffer,
	size_t buffer_length)
{
	int rc;
	int pci_direction;
	u16 bmic_device_index;
	struct pqi_raid_path_request request;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		BMIC_IDENTIFY_PHYSICAL_DEVICE, RAID_CTLR_LUNID, buffer,
		buffer_length, 0, &pci_direction);
	if (rc)
		return rc;

	bmic_device_index = CISS_GET_DRIVE_NUMBER(device->scsi3addr);
	request.cdb[2] = (u8)bmic_device_index;
	request.cdb[9] = (u8)(bmic_device_index >> 8);

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header,
		0, NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	return rc;
}

static int pqi_flush_cache(struct pqi_ctrl_info *ctrl_info,
	enum bmic_flush_cache_shutdown_event shutdown_event)
{
	int rc;
	struct pqi_raid_path_request request;
	int pci_direction;
	struct bmic_flush_cache *flush_cache;

	/*
	 * Don't bother trying to flush the cache if the controller is
	 * locked up.
	 */
	if (pqi_ctrl_offline(ctrl_info))
		return -ENXIO;

	flush_cache = kzalloc(sizeof(*flush_cache), GFP_KERNEL);
	if (!flush_cache)
		return -ENOMEM;

	flush_cache->shutdown_event = shutdown_event;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		SA_FLUSH_CACHE, RAID_CTLR_LUNID, flush_cache,
		sizeof(*flush_cache), 0, &pci_direction);
	if (rc)
		goto out;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header,
		0, NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

out:
	kfree(flush_cache);

	return rc;
}

static int pqi_write_host_wellness(struct pqi_ctrl_info *ctrl_info,
	void *buffer, size_t buffer_length)
{
	int rc;
	struct pqi_raid_path_request request;
	int pci_direction;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		BMIC_WRITE_HOST_WELLNESS, RAID_CTLR_LUNID, buffer,
		buffer_length, 0, &pci_direction);
	if (rc)
		return rc;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header,
		0, NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	return rc;
}

#pragma pack(1)

struct bmic_host_wellness_driver_version {
	u8	start_tag[4];
	u8	driver_version_tag[2];
	__le16	driver_version_length;
	char	driver_version[32];
	u8	end_tag[2];
};

#pragma pack()

static int pqi_write_driver_version_to_host_wellness(
	struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct bmic_host_wellness_driver_version *buffer;
	size_t buffer_length;

	buffer_length = sizeof(*buffer);

	buffer = kmalloc(buffer_length, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer->start_tag[0] = '<';
	buffer->start_tag[1] = 'H';
	buffer->start_tag[2] = 'W';
	buffer->start_tag[3] = '>';
	buffer->driver_version_tag[0] = 'D';
	buffer->driver_version_tag[1] = 'V';
	put_unaligned_le16(sizeof(buffer->driver_version),
		&buffer->driver_version_length);
	strncpy(buffer->driver_version, "Linux " DRIVER_VERSION,
		sizeof(buffer->driver_version) - 1);
	buffer->driver_version[sizeof(buffer->driver_version) - 1] = '\0';
	buffer->end_tag[0] = 'Z';
	buffer->end_tag[1] = 'Z';

	rc = pqi_write_host_wellness(ctrl_info, buffer, buffer_length);

	kfree(buffer);

	return rc;
}

#pragma pack(1)

struct bmic_host_wellness_time {
	u8	start_tag[4];
	u8	time_tag[2];
	__le16	time_length;
	u8	time[8];
	u8	dont_write_tag[2];
	u8	end_tag[2];
};

#pragma pack()

static int pqi_write_current_time_to_host_wellness(
	struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct bmic_host_wellness_time *buffer;
	size_t buffer_length;
	time64_t local_time;
	unsigned int year;
	struct tm tm;

	buffer_length = sizeof(*buffer);

	buffer = kmalloc(buffer_length, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer->start_tag[0] = '<';
	buffer->start_tag[1] = 'H';
	buffer->start_tag[2] = 'W';
	buffer->start_tag[3] = '>';
	buffer->time_tag[0] = 'T';
	buffer->time_tag[1] = 'D';
	put_unaligned_le16(sizeof(buffer->time),
		&buffer->time_length);

	local_time = ktime_get_real_seconds();
	time64_to_tm(local_time, -sys_tz.tz_minuteswest * 60, &tm);
	year = tm.tm_year + 1900;

	buffer->time[0] = bin2bcd(tm.tm_hour);
	buffer->time[1] = bin2bcd(tm.tm_min);
	buffer->time[2] = bin2bcd(tm.tm_sec);
	buffer->time[3] = 0;
	buffer->time[4] = bin2bcd(tm.tm_mon + 1);
	buffer->time[5] = bin2bcd(tm.tm_mday);
	buffer->time[6] = bin2bcd(year / 100);
	buffer->time[7] = bin2bcd(year % 100);

	buffer->dont_write_tag[0] = 'D';
	buffer->dont_write_tag[1] = 'W';
	buffer->end_tag[0] = 'Z';
	buffer->end_tag[1] = 'Z';

	rc = pqi_write_host_wellness(ctrl_info, buffer, buffer_length);

	kfree(buffer);

	return rc;
}

#define PQI_UPDATE_TIME_WORK_INTERVAL	(24UL * 60 * 60 * HZ)

static void pqi_update_time_worker(struct work_struct *work)
{
	int rc;
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = container_of(to_delayed_work(work), struct pqi_ctrl_info,
		update_time_work);

	if (pqi_ctrl_offline(ctrl_info))
		return;

	rc = pqi_write_current_time_to_host_wellness(ctrl_info);
	if (rc)
		dev_warn(&ctrl_info->pci_dev->dev,
			"error updating time on controller\n");

	schedule_delayed_work(&ctrl_info->update_time_work,
		PQI_UPDATE_TIME_WORK_INTERVAL);
}

static inline void pqi_schedule_update_time_worker(
	struct pqi_ctrl_info *ctrl_info)
{
	schedule_delayed_work(&ctrl_info->update_time_work, 0);
}

static inline void pqi_cancel_update_time_worker(
	struct pqi_ctrl_info *ctrl_info)
{
	cancel_delayed_work_sync(&ctrl_info->update_time_work);
}

static int pqi_report_luns(struct pqi_ctrl_info *ctrl_info, u8 cmd,
	void *buffer, size_t buffer_length)
{
	int rc;
	int pci_direction;
	struct pqi_raid_path_request request;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		cmd, RAID_CTLR_LUNID, buffer, buffer_length, 0, &pci_direction);
	if (rc)
		return rc;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header, 0,
		NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	return rc;
}

static int pqi_report_phys_logical_luns(struct pqi_ctrl_info *ctrl_info, u8 cmd,
	void **buffer)
{
	int rc;
	size_t lun_list_length;
	size_t lun_data_length;
	size_t new_lun_list_length;
	void *lun_data = NULL;
	struct report_lun_header *report_lun_header;

	report_lun_header = kmalloc(sizeof(*report_lun_header), GFP_KERNEL);
	if (!report_lun_header) {
		rc = -ENOMEM;
		goto out;
	}

	rc = pqi_report_luns(ctrl_info, cmd, report_lun_header,
		sizeof(*report_lun_header));
	if (rc)
		goto out;

	lun_list_length = get_unaligned_be32(&report_lun_header->list_length);

again:
	lun_data_length = sizeof(struct report_lun_header) + lun_list_length;

	lun_data = kmalloc(lun_data_length, GFP_KERNEL);
	if (!lun_data) {
		rc = -ENOMEM;
		goto out;
	}

	if (lun_list_length == 0) {
		memcpy(lun_data, report_lun_header, sizeof(*report_lun_header));
		goto out;
	}

	rc = pqi_report_luns(ctrl_info, cmd, lun_data, lun_data_length);
	if (rc)
		goto out;

	new_lun_list_length = get_unaligned_be32(
		&((struct report_lun_header *)lun_data)->list_length);

	if (new_lun_list_length > lun_list_length) {
		lun_list_length = new_lun_list_length;
		kfree(lun_data);
		goto again;
	}

out:
	kfree(report_lun_header);

	if (rc) {
		kfree(lun_data);
		lun_data = NULL;
	}

	*buffer = lun_data;

	return rc;
}

static inline int pqi_report_phys_luns(struct pqi_ctrl_info *ctrl_info,
	void **buffer)
{
	return pqi_report_phys_logical_luns(ctrl_info, CISS_REPORT_PHYS,
		buffer);
}

static inline int pqi_report_logical_luns(struct pqi_ctrl_info *ctrl_info,
	void **buffer)
{
	return pqi_report_phys_logical_luns(ctrl_info, CISS_REPORT_LOG, buffer);
}

static int pqi_get_device_lists(struct pqi_ctrl_info *ctrl_info,
	struct report_phys_lun_extended **physdev_list,
	struct report_log_lun_extended **logdev_list)
{
	int rc;
	size_t logdev_list_length;
	size_t logdev_data_length;
	struct report_log_lun_extended *internal_logdev_list;
	struct report_log_lun_extended *logdev_data;
	struct report_lun_header report_lun_header;

	rc = pqi_report_phys_luns(ctrl_info, (void **)physdev_list);
	if (rc)
		dev_err(&ctrl_info->pci_dev->dev,
			"report physical LUNs failed\n");

	rc = pqi_report_logical_luns(ctrl_info, (void **)logdev_list);
	if (rc)
		dev_err(&ctrl_info->pci_dev->dev,
			"report logical LUNs failed\n");

	/*
	 * Tack the controller itself onto the end of the logical device list.
	 */

	logdev_data = *logdev_list;

	if (logdev_data) {
		logdev_list_length =
			get_unaligned_be32(&logdev_data->header.list_length);
	} else {
		memset(&report_lun_header, 0, sizeof(report_lun_header));
		logdev_data =
			(struct report_log_lun_extended *)&report_lun_header;
		logdev_list_length = 0;
	}

	logdev_data_length = sizeof(struct report_lun_header) +
		logdev_list_length;

	internal_logdev_list = kmalloc(logdev_data_length +
		sizeof(struct report_log_lun_extended), GFP_KERNEL);
	if (!internal_logdev_list) {
		kfree(*logdev_list);
		*logdev_list = NULL;
		return -ENOMEM;
	}

	memcpy(internal_logdev_list, logdev_data, logdev_data_length);
	memset((u8 *)internal_logdev_list + logdev_data_length, 0,
		sizeof(struct report_log_lun_extended_entry));
	put_unaligned_be32(logdev_list_length +
		sizeof(struct report_log_lun_extended_entry),
		&internal_logdev_list->header.list_length);

	kfree(*logdev_list);
	*logdev_list = internal_logdev_list;

	return 0;
}

static inline void pqi_set_bus_target_lun(struct pqi_scsi_dev *device,
	int bus, int target, int lun)
{
	device->bus = bus;
	device->target = target;
	device->lun = lun;
}

static void pqi_assign_bus_target_lun(struct pqi_scsi_dev *device)
{
	u8 *scsi3addr;
	u32 lunid;
	int bus;
	int target;
	int lun;

	scsi3addr = device->scsi3addr;
	lunid = get_unaligned_le32(scsi3addr);

	if (pqi_is_hba_lunid(scsi3addr)) {
		/* The specified device is the controller. */
		pqi_set_bus_target_lun(device, PQI_HBA_BUS, 0, lunid & 0x3fff);
		device->target_lun_valid = true;
		return;
	}

	if (pqi_is_logical_device(device)) {
		if (device->is_external_raid_device) {
			bus = PQI_EXTERNAL_RAID_VOLUME_BUS;
			target = (lunid >> 16) & 0x3fff;
			lun = lunid & 0xff;
		} else {
			bus = PQI_RAID_VOLUME_BUS;
			target = 0;
			lun = lunid & 0x3fff;
		}
		pqi_set_bus_target_lun(device, bus, target, lun);
		device->target_lun_valid = true;
		return;
	}

	/*
	 * Defer target and LUN assignment for non-controller physical devices
	 * because the SAS transport layer will make these assignments later.
	 */
	pqi_set_bus_target_lun(device, PQI_PHYSICAL_DEVICE_BUS, 0, 0);
}

static void pqi_get_raid_level(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	u8 raid_level;
	u8 *buffer;

	raid_level = SA_RAID_UNKNOWN;

	buffer = kmalloc(64, GFP_KERNEL);
	if (buffer) {
		rc = pqi_scsi_inquiry(ctrl_info, device->scsi3addr,
			VPD_PAGE | CISS_VPD_LV_DEVICE_GEOMETRY, buffer, 64);
		if (rc == 0) {
			raid_level = buffer[8];
			if (raid_level > SA_RAID_MAX)
				raid_level = SA_RAID_UNKNOWN;
		}
		kfree(buffer);
	}

	device->raid_level = raid_level;
}

static int pqi_validate_raid_map(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device, struct raid_map *raid_map)
{
	char *err_msg;
	u32 raid_map_size;
	u32 r5or6_blocks_per_row;
	unsigned int num_phys_disks;
	unsigned int num_raid_map_entries;

	raid_map_size = get_unaligned_le32(&raid_map->structure_size);

	if (raid_map_size < offsetof(struct raid_map, disk_data)) {
		err_msg = "RAID map too small";
		goto bad_raid_map;
	}

	if (raid_map_size > sizeof(*raid_map)) {
		err_msg = "RAID map too large";
		goto bad_raid_map;
	}

	num_phys_disks = get_unaligned_le16(&raid_map->layout_map_count) *
		(get_unaligned_le16(&raid_map->data_disks_per_row) +
		get_unaligned_le16(&raid_map->metadata_disks_per_row));
	num_raid_map_entries = num_phys_disks *
		get_unaligned_le16(&raid_map->row_cnt);

	if (num_raid_map_entries > RAID_MAP_MAX_ENTRIES) {
		err_msg = "invalid number of map entries in RAID map";
		goto bad_raid_map;
	}

	if (device->raid_level == SA_RAID_1) {
		if (get_unaligned_le16(&raid_map->layout_map_count) != 2) {
			err_msg = "invalid RAID-1 map";
			goto bad_raid_map;
		}
	} else if (device->raid_level == SA_RAID_ADM) {
		if (get_unaligned_le16(&raid_map->layout_map_count) != 3) {
			err_msg = "invalid RAID-1(ADM) map";
			goto bad_raid_map;
		}
	} else if ((device->raid_level == SA_RAID_5 ||
		device->raid_level == SA_RAID_6) &&
		get_unaligned_le16(&raid_map->layout_map_count) > 1) {
		/* RAID 50/60 */
		r5or6_blocks_per_row =
			get_unaligned_le16(&raid_map->strip_size) *
			get_unaligned_le16(&raid_map->data_disks_per_row);
		if (r5or6_blocks_per_row == 0) {
			err_msg = "invalid RAID-5 or RAID-6 map";
			goto bad_raid_map;
		}
	}

	return 0;

bad_raid_map:
	dev_warn(&ctrl_info->pci_dev->dev,
		"logical device %08x%08x %s\n",
		*((u32 *)&device->scsi3addr),
		*((u32 *)&device->scsi3addr[4]), err_msg);

	return -EINVAL;
}

static int pqi_get_raid_map(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	int pci_direction;
	struct pqi_raid_path_request request;
	struct raid_map *raid_map;

	raid_map = kmalloc(sizeof(*raid_map), GFP_KERNEL);
	if (!raid_map)
		return -ENOMEM;

	rc = pqi_build_raid_path_request(ctrl_info, &request,
		CISS_GET_RAID_MAP, device->scsi3addr, raid_map,
		sizeof(*raid_map), 0, &pci_direction);
	if (rc)
		goto error;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header, 0,
		NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
		pci_direction);

	if (rc)
		goto error;

	rc = pqi_validate_raid_map(ctrl_info, device, raid_map);
	if (rc)
		goto error;

	device->raid_map = raid_map;

	return 0;

error:
	kfree(raid_map);

	return rc;
}

static void pqi_get_raid_bypass_status(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	u8 *buffer;
	u8 bypass_status;

	buffer = kmalloc(64, GFP_KERNEL);
	if (!buffer)
		return;

	rc = pqi_scsi_inquiry(ctrl_info, device->scsi3addr,
		VPD_PAGE | CISS_VPD_LV_BYPASS_STATUS, buffer, 64);
	if (rc)
		goto out;

#define RAID_BYPASS_STATUS	4
#define RAID_BYPASS_CONFIGURED	0x1
#define RAID_BYPASS_ENABLED	0x2

	bypass_status = buffer[RAID_BYPASS_STATUS];
	device->raid_bypass_configured =
		(bypass_status & RAID_BYPASS_CONFIGURED) != 0;
	if (device->raid_bypass_configured &&
		(bypass_status & RAID_BYPASS_ENABLED) &&
		pqi_get_raid_map(ctrl_info, device) == 0)
		device->raid_bypass_enabled = true;

out:
	kfree(buffer);
}

/*
 * Use vendor-specific VPD to determine online/offline status of a volume.
 */

static void pqi_get_volume_status(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	size_t page_length;
	u8 volume_status = CISS_LV_STATUS_UNAVAILABLE;
	bool volume_offline = true;
	u32 volume_flags;
	struct ciss_vpd_logical_volume_status *vpd;

	vpd = kmalloc(sizeof(*vpd), GFP_KERNEL);
	if (!vpd)
		goto no_buffer;

	rc = pqi_scsi_inquiry(ctrl_info, device->scsi3addr,
		VPD_PAGE | CISS_VPD_LV_STATUS, vpd, sizeof(*vpd));
	if (rc)
		goto out;

	page_length = offsetof(struct ciss_vpd_logical_volume_status,
		volume_status) + vpd->page_length;
	if (page_length < sizeof(*vpd))
		goto out;

	volume_status = vpd->volume_status;
	volume_flags = get_unaligned_be32(&vpd->flags);
	volume_offline = (volume_flags & CISS_LV_FLAGS_NO_HOST_IO) != 0;

out:
	kfree(vpd);
no_buffer:
	device->volume_status = volume_status;
	device->volume_offline = volume_offline;
}

static int pqi_get_device_info(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	u8 *buffer;

	buffer = kmalloc(64, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* Send an inquiry to the device to see what it is. */
	rc = pqi_scsi_inquiry(ctrl_info, device->scsi3addr, 0, buffer, 64);
	if (rc)
		goto out;

	scsi_sanitize_inquiry_string(&buffer[8], 8);
	scsi_sanitize_inquiry_string(&buffer[16], 16);

	device->devtype = buffer[0] & 0x1f;
	memcpy(device->vendor, &buffer[8], sizeof(device->vendor));
	memcpy(device->model, &buffer[16], sizeof(device->model));

	if (pqi_is_logical_device(device) && device->devtype == TYPE_DISK) {
		if (device->is_external_raid_device) {
			device->raid_level = SA_RAID_UNKNOWN;
			device->volume_status = CISS_LV_OK;
			device->volume_offline = false;
		} else {
			pqi_get_raid_level(ctrl_info, device);
			pqi_get_raid_bypass_status(ctrl_info, device);
			pqi_get_volume_status(ctrl_info, device);
		}
	}

out:
	kfree(buffer);

	return rc;
}

static void pqi_get_physical_disk_info(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device,
	struct bmic_identify_physical_device *id_phys)
{
	int rc;

	memset(id_phys, 0, sizeof(*id_phys));

	rc = pqi_identify_physical_device(ctrl_info, device,
		id_phys, sizeof(*id_phys));
	if (rc) {
		device->queue_depth = PQI_PHYSICAL_DISK_DEFAULT_MAX_QUEUE_DEPTH;
		return;
	}

	device->queue_depth =
		get_unaligned_le16(&id_phys->current_queue_depth_limit);
	device->device_type = id_phys->device_type;
	device->active_path_index = id_phys->active_path_number;
	device->path_map = id_phys->redundant_path_present_map;
	memcpy(&device->box,
		&id_phys->alternate_paths_phys_box_on_port,
		sizeof(device->box));
	memcpy(&device->phys_connector,
		&id_phys->alternate_paths_phys_connector,
		sizeof(device->phys_connector));
	device->bay = id_phys->phys_bay_in_box;
}

static void pqi_show_volume_status(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	char *status;
	static const char unknown_state_str[] =
		"Volume is in an unknown state (%u)";
	char unknown_state_buffer[sizeof(unknown_state_str) + 10];

	switch (device->volume_status) {
	case CISS_LV_OK:
		status = "Volume online";
		break;
	case CISS_LV_FAILED:
		status = "Volume failed";
		break;
	case CISS_LV_NOT_CONFIGURED:
		status = "Volume not configured";
		break;
	case CISS_LV_DEGRADED:
		status = "Volume degraded";
		break;
	case CISS_LV_READY_FOR_RECOVERY:
		status = "Volume ready for recovery operation";
		break;
	case CISS_LV_UNDERGOING_RECOVERY:
		status = "Volume undergoing recovery";
		break;
	case CISS_LV_WRONG_PHYSICAL_DRIVE_REPLACED:
		status = "Wrong physical drive was replaced";
		break;
	case CISS_LV_PHYSICAL_DRIVE_CONNECTION_PROBLEM:
		status = "A physical drive not properly connected";
		break;
	case CISS_LV_HARDWARE_OVERHEATING:
		status = "Hardware is overheating";
		break;
	case CISS_LV_HARDWARE_HAS_OVERHEATED:
		status = "Hardware has overheated";
		break;
	case CISS_LV_UNDERGOING_EXPANSION:
		status = "Volume undergoing expansion";
		break;
	case CISS_LV_NOT_AVAILABLE:
		status = "Volume waiting for transforming volume";
		break;
	case CISS_LV_QUEUED_FOR_EXPANSION:
		status = "Volume queued for expansion";
		break;
	case CISS_LV_DISABLED_SCSI_ID_CONFLICT:
		status = "Volume disabled due to SCSI ID conflict";
		break;
	case CISS_LV_EJECTED:
		status = "Volume has been ejected";
		break;
	case CISS_LV_UNDERGOING_ERASE:
		status = "Volume undergoing background erase";
		break;
	case CISS_LV_READY_FOR_PREDICTIVE_SPARE_REBUILD:
		status = "Volume ready for predictive spare rebuild";
		break;
	case CISS_LV_UNDERGOING_RPI:
		status = "Volume undergoing rapid parity initialization";
		break;
	case CISS_LV_PENDING_RPI:
		status = "Volume queued for rapid parity initialization";
		break;
	case CISS_LV_ENCRYPTED_NO_KEY:
		status = "Encrypted volume inaccessible - key not present";
		break;
	case CISS_LV_UNDERGOING_ENCRYPTION:
		status = "Volume undergoing encryption process";
		break;
	case CISS_LV_UNDERGOING_ENCRYPTION_REKEYING:
		status = "Volume undergoing encryption re-keying process";
		break;
	case CISS_LV_ENCRYPTED_IN_NON_ENCRYPTED_CONTROLLER:
		status = "Volume encrypted but encryption is disabled";
		break;
	case CISS_LV_PENDING_ENCRYPTION:
		status = "Volume pending migration to encrypted state";
		break;
	case CISS_LV_PENDING_ENCRYPTION_REKEYING:
		status = "Volume pending encryption rekeying";
		break;
	case CISS_LV_NOT_SUPPORTED:
		status = "Volume not supported on this controller";
		break;
	case CISS_LV_STATUS_UNAVAILABLE:
		status = "Volume status not available";
		break;
	default:
		snprintf(unknown_state_buffer, sizeof(unknown_state_buffer),
			unknown_state_str, device->volume_status);
		status = unknown_state_buffer;
		break;
	}

	dev_info(&ctrl_info->pci_dev->dev,
		"scsi %d:%d:%d:%d %s\n",
		ctrl_info->scsi_host->host_no,
		device->bus, device->target, device->lun, status);
}

static void pqi_rescan_worker(struct work_struct *work)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = container_of(to_delayed_work(work), struct pqi_ctrl_info,
		rescan_work);

	pqi_scan_scsi_devices(ctrl_info);
}

static int pqi_add_device(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;

	if (pqi_is_logical_device(device))
		rc = scsi_add_device(ctrl_info->scsi_host, device->bus,
			device->target, device->lun);
	else
		rc = pqi_add_sas_device(ctrl_info->sas_host, device);

	return rc;
}

static inline void pqi_remove_device(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	if (pqi_is_logical_device(device))
		scsi_remove_device(device->sdev);
	else
		pqi_remove_sas_device(device);
}

/* Assumes the SCSI device list lock is held. */

static struct pqi_scsi_dev *pqi_find_scsi_dev(struct pqi_ctrl_info *ctrl_info,
	int bus, int target, int lun)
{
	struct pqi_scsi_dev *device;

	list_for_each_entry(device, &ctrl_info->scsi_device_list,
		scsi_device_list_entry)
		if (device->bus == bus && device->target == target &&
			device->lun == lun)
			return device;

	return NULL;
}

static inline bool pqi_device_equal(struct pqi_scsi_dev *dev1,
	struct pqi_scsi_dev *dev2)
{
	if (dev1->is_physical_device != dev2->is_physical_device)
		return false;

	if (dev1->is_physical_device)
		return dev1->wwid == dev2->wwid;

	return memcmp(dev1->volume_id, dev2->volume_id,
		sizeof(dev1->volume_id)) == 0;
}

enum pqi_find_result {
	DEVICE_NOT_FOUND,
	DEVICE_CHANGED,
	DEVICE_SAME,
};

static enum pqi_find_result pqi_scsi_find_entry(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device_to_find,
	struct pqi_scsi_dev **matching_device)
{
	struct pqi_scsi_dev *device;

	list_for_each_entry(device, &ctrl_info->scsi_device_list,
		scsi_device_list_entry) {
		if (pqi_scsi3addr_equal(device_to_find->scsi3addr,
			device->scsi3addr)) {
			*matching_device = device;
			if (pqi_device_equal(device_to_find, device)) {
				if (device_to_find->volume_offline)
					return DEVICE_CHANGED;
				return DEVICE_SAME;
			}
			return DEVICE_CHANGED;
		}
	}

	return DEVICE_NOT_FOUND;
}

#define PQI_DEV_INFO_BUFFER_LENGTH	128

static void pqi_dev_info(struct pqi_ctrl_info *ctrl_info,
	char *action, struct pqi_scsi_dev *device)
{
	ssize_t count;
	char buffer[PQI_DEV_INFO_BUFFER_LENGTH];

	count = snprintf(buffer, PQI_DEV_INFO_BUFFER_LENGTH,
		"%d:%d:", ctrl_info->scsi_host->host_no, device->bus);

	if (device->target_lun_valid)
		count += snprintf(buffer + count,
			PQI_DEV_INFO_BUFFER_LENGTH - count,
			"%d:%d",
			device->target,
			device->lun);
	else
		count += snprintf(buffer + count,
			PQI_DEV_INFO_BUFFER_LENGTH - count,
			"-:-");

	if (pqi_is_logical_device(device))
		count += snprintf(buffer + count,
			PQI_DEV_INFO_BUFFER_LENGTH - count,
			" %08x%08x",
			*((u32 *)&device->scsi3addr),
			*((u32 *)&device->scsi3addr[4]));
	else
		count += snprintf(buffer + count,
			PQI_DEV_INFO_BUFFER_LENGTH - count,
			" %016llx", device->sas_address);

	count += snprintf(buffer + count, PQI_DEV_INFO_BUFFER_LENGTH - count,
		" %s %.8s %.16s ",
		scsi_device_type(device->devtype),
		device->vendor,
		device->model);

	if (pqi_is_logical_device(device)) {
		if (device->devtype == TYPE_DISK)
			count += snprintf(buffer + count,
				PQI_DEV_INFO_BUFFER_LENGTH - count,
				"SSDSmartPathCap%c En%c %-12s",
				device->raid_bypass_configured ? '+' : '-',
				device->raid_bypass_enabled ? '+' : '-',
				pqi_raid_level_to_string(device->raid_level));
	} else {
		count += snprintf(buffer + count,
			PQI_DEV_INFO_BUFFER_LENGTH - count,
			"AIO%c", device->aio_enabled ? '+' : '-');
		if (device->devtype == TYPE_DISK ||
			device->devtype == TYPE_ZBC)
			count += snprintf(buffer + count,
				PQI_DEV_INFO_BUFFER_LENGTH - count,
				" qd=%-6d", device->queue_depth);
	}

	dev_info(&ctrl_info->pci_dev->dev, "%s %s\n", action, buffer);
}

/* Assumes the SCSI device list lock is held. */

static void pqi_scsi_update_device(struct pqi_scsi_dev *existing_device,
	struct pqi_scsi_dev *new_device)
{
	existing_device->devtype = new_device->devtype;
	existing_device->device_type = new_device->device_type;
	existing_device->bus = new_device->bus;
	if (new_device->target_lun_valid) {
		existing_device->target = new_device->target;
		existing_device->lun = new_device->lun;
		existing_device->target_lun_valid = true;
	}

	/* By definition, the scsi3addr and wwid fields are already the same. */

	existing_device->is_physical_device = new_device->is_physical_device;
	existing_device->is_external_raid_device =
		new_device->is_external_raid_device;
	existing_device->aio_enabled = new_device->aio_enabled;
	memcpy(existing_device->vendor, new_device->vendor,
		sizeof(existing_device->vendor));
	memcpy(existing_device->model, new_device->model,
		sizeof(existing_device->model));
	existing_device->sas_address = new_device->sas_address;
	existing_device->raid_level = new_device->raid_level;
	existing_device->queue_depth = new_device->queue_depth;
	existing_device->aio_handle = new_device->aio_handle;
	existing_device->volume_status = new_device->volume_status;
	existing_device->active_path_index = new_device->active_path_index;
	existing_device->path_map = new_device->path_map;
	existing_device->bay = new_device->bay;
	memcpy(existing_device->box, new_device->box,
		sizeof(existing_device->box));
	memcpy(existing_device->phys_connector, new_device->phys_connector,
		sizeof(existing_device->phys_connector));
	existing_device->offload_to_mirror = 0;
	kfree(existing_device->raid_map);
	existing_device->raid_map = new_device->raid_map;
	existing_device->raid_bypass_configured =
		new_device->raid_bypass_configured;
	existing_device->raid_bypass_enabled =
		new_device->raid_bypass_enabled;

	/* To prevent this from being freed later. */
	new_device->raid_map = NULL;
}

static inline void pqi_free_device(struct pqi_scsi_dev *device)
{
	if (device) {
		kfree(device->raid_map);
		kfree(device);
	}
}

/*
 * Called when exposing a new device to the OS fails in order to re-adjust
 * our internal SCSI device list to match the SCSI ML's view.
 */

static inline void pqi_fixup_botched_add(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);
	list_del(&device->scsi_device_list_entry);
	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	/* Allow the device structure to be freed later. */
	device->keep_device = false;
}

static void pqi_update_device_list(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *new_device_list[], unsigned int num_new_devices)
{
	int rc;
	unsigned int i;
	unsigned long flags;
	enum pqi_find_result find_result;
	struct pqi_scsi_dev *device;
	struct pqi_scsi_dev *next;
	struct pqi_scsi_dev *matching_device;
	LIST_HEAD(add_list);
	LIST_HEAD(delete_list);

	/*
	 * The idea here is to do as little work as possible while holding the
	 * spinlock.  That's why we go to great pains to defer anything other
	 * than updating the internal device list until after we release the
	 * spinlock.
	 */

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

	/* Assume that all devices in the existing list have gone away. */
	list_for_each_entry(device, &ctrl_info->scsi_device_list,
		scsi_device_list_entry)
		device->device_gone = true;

	for (i = 0; i < num_new_devices; i++) {
		device = new_device_list[i];

		find_result = pqi_scsi_find_entry(ctrl_info, device,
						&matching_device);

		switch (find_result) {
		case DEVICE_SAME:
			/*
			 * The newly found device is already in the existing
			 * device list.
			 */
			device->new_device = false;
			matching_device->device_gone = false;
			pqi_scsi_update_device(matching_device, device);
			break;
		case DEVICE_NOT_FOUND:
			/*
			 * The newly found device is NOT in the existing device
			 * list.
			 */
			device->new_device = true;
			break;
		case DEVICE_CHANGED:
			/*
			 * The original device has gone away and we need to add
			 * the new device.
			 */
			device->new_device = true;
			break;
		}
	}

	/* Process all devices that have gone away. */
	list_for_each_entry_safe(device, next, &ctrl_info->scsi_device_list,
		scsi_device_list_entry) {
		if (device->device_gone) {
			list_del(&device->scsi_device_list_entry);
			list_add_tail(&device->delete_list_entry, &delete_list);
		}
	}

	/* Process all new devices. */
	for (i = 0; i < num_new_devices; i++) {
		device = new_device_list[i];
		if (!device->new_device)
			continue;
		if (device->volume_offline)
			continue;
		list_add_tail(&device->scsi_device_list_entry,
			&ctrl_info->scsi_device_list);
		list_add_tail(&device->add_list_entry, &add_list);
		/* To prevent this device structure from being freed later. */
		device->keep_device = true;
	}

	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	/* Remove all devices that have gone away. */
	list_for_each_entry_safe(device, next, &delete_list,
		delete_list_entry) {
		if (device->volume_offline) {
			pqi_dev_info(ctrl_info, "offline", device);
			pqi_show_volume_status(ctrl_info, device);
		} else {
			pqi_dev_info(ctrl_info, "removed", device);
		}
		if (device->sdev)
			pqi_remove_device(ctrl_info, device);
		list_del(&device->delete_list_entry);
		pqi_free_device(device);
	}

	/*
	 * Notify the SCSI ML if the queue depth of any existing device has
	 * changed.
	 */
	list_for_each_entry(device, &ctrl_info->scsi_device_list,
		scsi_device_list_entry) {
		if (device->sdev && device->queue_depth !=
			device->advertised_queue_depth) {
			device->advertised_queue_depth = device->queue_depth;
			scsi_change_queue_depth(device->sdev,
				device->advertised_queue_depth);
		}
	}

	/* Expose any new devices. */
	list_for_each_entry_safe(device, next, &add_list, add_list_entry) {
		if (!device->sdev) {
			pqi_dev_info(ctrl_info, "added", device);
			rc = pqi_add_device(ctrl_info, device);
			if (rc) {
				dev_warn(&ctrl_info->pci_dev->dev,
					"scsi %d:%d:%d:%d addition failed, device not added\n",
					ctrl_info->scsi_host->host_no,
					device->bus, device->target,
					device->lun);
				pqi_fixup_botched_add(ctrl_info, device);
			}
		}
	}
}

static bool pqi_is_supported_device(struct pqi_scsi_dev *device)
{
	bool is_supported = false;

	switch (device->devtype) {
	case TYPE_DISK:
	case TYPE_ZBC:
	case TYPE_TAPE:
	case TYPE_MEDIUM_CHANGER:
	case TYPE_ENCLOSURE:
		is_supported = true;
		break;
	case TYPE_RAID:
		/*
		 * Only support the HBA controller itself as a RAID
		 * controller.  If it's a RAID controller other than
		 * the HBA itself (an external RAID controller, for
		 * example), we don't support it.
		 */
		if (pqi_is_hba_lunid(device->scsi3addr))
			is_supported = true;
		break;
	}

	return is_supported;
}

static inline bool pqi_skip_device(u8 *scsi3addr)
{
	/* Ignore all masked devices. */
	if (MASKED_DEVICE(scsi3addr))
		return true;

	return false;
}

static int pqi_update_scsi_devices(struct pqi_ctrl_info *ctrl_info)
{
	int i;
	int rc;
	LIST_HEAD(new_device_list_head);
	struct report_phys_lun_extended *physdev_list = NULL;
	struct report_log_lun_extended *logdev_list = NULL;
	struct report_phys_lun_extended_entry *phys_lun_ext_entry;
	struct report_log_lun_extended_entry *log_lun_ext_entry;
	struct bmic_identify_physical_device *id_phys = NULL;
	u32 num_physicals;
	u32 num_logicals;
	struct pqi_scsi_dev **new_device_list = NULL;
	struct pqi_scsi_dev *device;
	struct pqi_scsi_dev *next;
	unsigned int num_new_devices;
	unsigned int num_valid_devices;
	bool is_physical_device;
	u8 *scsi3addr;
	static char *out_of_memory_msg =
		"failed to allocate memory, device discovery stopped";

	rc = pqi_get_device_lists(ctrl_info, &physdev_list, &logdev_list);
	if (rc)
		goto out;

	if (physdev_list)
		num_physicals =
			get_unaligned_be32(&physdev_list->header.list_length)
				/ sizeof(physdev_list->lun_entries[0]);
	else
		num_physicals = 0;

	if (logdev_list)
		num_logicals =
			get_unaligned_be32(&logdev_list->header.list_length)
				/ sizeof(logdev_list->lun_entries[0]);
	else
		num_logicals = 0;

	if (num_physicals) {
		/*
		 * We need this buffer for calls to pqi_get_physical_disk_info()
		 * below.  We allocate it here instead of inside
		 * pqi_get_physical_disk_info() because it's a fairly large
		 * buffer.
		 */
		id_phys = kmalloc(sizeof(*id_phys), GFP_KERNEL);
		if (!id_phys) {
			dev_warn(&ctrl_info->pci_dev->dev, "%s\n",
				out_of_memory_msg);
			rc = -ENOMEM;
			goto out;
		}
	}

	num_new_devices = num_physicals + num_logicals;

	new_device_list = kmalloc(sizeof(*new_device_list) *
		num_new_devices, GFP_KERNEL);
	if (!new_device_list) {
		dev_warn(&ctrl_info->pci_dev->dev, "%s\n", out_of_memory_msg);
		rc = -ENOMEM;
		goto out;
	}

	for (i = 0; i < num_new_devices; i++) {
		device = kzalloc(sizeof(*device), GFP_KERNEL);
		if (!device) {
			dev_warn(&ctrl_info->pci_dev->dev, "%s\n",
				out_of_memory_msg);
			rc = -ENOMEM;
			goto out;
		}
		list_add_tail(&device->new_device_list_entry,
			&new_device_list_head);
	}

	device = NULL;
	num_valid_devices = 0;

	for (i = 0; i < num_new_devices; i++) {

		if (i < num_physicals) {
			is_physical_device = true;
			phys_lun_ext_entry = &physdev_list->lun_entries[i];
			log_lun_ext_entry = NULL;
			scsi3addr = phys_lun_ext_entry->lunid;
		} else {
			is_physical_device = false;
			phys_lun_ext_entry = NULL;
			log_lun_ext_entry =
				&logdev_list->lun_entries[i - num_physicals];
			scsi3addr = log_lun_ext_entry->lunid;
		}

		if (is_physical_device && pqi_skip_device(scsi3addr))
			continue;

		if (device)
			device = list_next_entry(device, new_device_list_entry);
		else
			device = list_first_entry(&new_device_list_head,
				struct pqi_scsi_dev, new_device_list_entry);

		memcpy(device->scsi3addr, scsi3addr, sizeof(device->scsi3addr));
		device->is_physical_device = is_physical_device;
		if (!is_physical_device)
			device->is_external_raid_device =
				pqi_is_external_raid_addr(scsi3addr);

		/* Gather information about the device. */
		rc = pqi_get_device_info(ctrl_info, device);
		if (rc == -ENOMEM) {
			dev_warn(&ctrl_info->pci_dev->dev, "%s\n",
				out_of_memory_msg);
			goto out;
		}
		if (rc) {
			if (device->is_physical_device)
				dev_warn(&ctrl_info->pci_dev->dev,
					"obtaining device info failed, skipping physical device %016llx\n",
					get_unaligned_be64(
						&phys_lun_ext_entry->wwid));
			else
				dev_warn(&ctrl_info->pci_dev->dev,
					"obtaining device info failed, skipping logical device %08x%08x\n",
					*((u32 *)&device->scsi3addr),
					*((u32 *)&device->scsi3addr[4]));
			rc = 0;
			continue;
		}

		if (!pqi_is_supported_device(device))
			continue;

		pqi_assign_bus_target_lun(device);

		if (device->is_physical_device) {
			device->wwid = phys_lun_ext_entry->wwid;
			if ((phys_lun_ext_entry->device_flags &
				REPORT_PHYS_LUN_DEV_FLAG_AIO_ENABLED) &&
				phys_lun_ext_entry->aio_handle)
				device->aio_enabled = true;
		} else {
			memcpy(device->volume_id, log_lun_ext_entry->volume_id,
				sizeof(device->volume_id));
		}

		switch (device->devtype) {
		case TYPE_DISK:
		case TYPE_ZBC:
		case TYPE_ENCLOSURE:
			if (device->is_physical_device) {
				device->sas_address =
					get_unaligned_be64(&device->wwid);
				if (device->devtype == TYPE_DISK ||
					device->devtype == TYPE_ZBC) {
					device->aio_handle =
						phys_lun_ext_entry->aio_handle;
					pqi_get_physical_disk_info(ctrl_info,
						device, id_phys);
				}
			}
			break;
		}

		new_device_list[num_valid_devices++] = device;
	}

	pqi_update_device_list(ctrl_info, new_device_list, num_valid_devices);

out:
	list_for_each_entry_safe(device, next, &new_device_list_head,
		new_device_list_entry) {
		if (device->keep_device)
			continue;
		list_del(&device->new_device_list_entry);
		pqi_free_device(device);
	}

	kfree(new_device_list);
	kfree(physdev_list);
	kfree(logdev_list);
	kfree(id_phys);

	return rc;
}

static void pqi_remove_all_scsi_devices(struct pqi_ctrl_info *ctrl_info)
{
	unsigned long flags;
	struct pqi_scsi_dev *device;

	while (1) {
		spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

		device = list_first_entry_or_null(&ctrl_info->scsi_device_list,
			struct pqi_scsi_dev, scsi_device_list_entry);
		if (device)
			list_del(&device->scsi_device_list_entry);

		spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock,
			flags);

		if (!device)
			break;

		if (device->sdev)
			pqi_remove_device(ctrl_info, device);
		pqi_free_device(device);
	}
}

static int pqi_scan_scsi_devices(struct pqi_ctrl_info *ctrl_info)
{
	int rc;

	if (pqi_ctrl_offline(ctrl_info))
		return -ENXIO;

	mutex_lock(&ctrl_info->scan_mutex);

	rc = pqi_update_scsi_devices(ctrl_info);
	if (rc)
		pqi_schedule_rescan_worker_delayed(ctrl_info);

	mutex_unlock(&ctrl_info->scan_mutex);

	return rc;
}

static void pqi_scan_start(struct Scsi_Host *shost)
{
	pqi_scan_scsi_devices(shost_to_hba(shost));
}

/* Returns TRUE if scan is finished. */

static int pqi_scan_finished(struct Scsi_Host *shost,
	unsigned long elapsed_time)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = shost_priv(shost);

	return !mutex_is_locked(&ctrl_info->scan_mutex);
}

static void pqi_wait_until_scan_finished(struct pqi_ctrl_info *ctrl_info)
{
	mutex_lock(&ctrl_info->scan_mutex);
	mutex_unlock(&ctrl_info->scan_mutex);
}

static void pqi_wait_until_lun_reset_finished(struct pqi_ctrl_info *ctrl_info)
{
	mutex_lock(&ctrl_info->lun_reset_mutex);
	mutex_unlock(&ctrl_info->lun_reset_mutex);
}

static inline void pqi_set_encryption_info(
	struct pqi_encryption_info *encryption_info, struct raid_map *raid_map,
	u64 first_block)
{
	u32 volume_blk_size;

	/*
	 * Set the encryption tweak values based on logical block address.
	 * If the block size is 512, the tweak value is equal to the LBA.
	 * For other block sizes, tweak value is (LBA * block size) / 512.
	 */
	volume_blk_size = get_unaligned_le32(&raid_map->volume_blk_size);
	if (volume_blk_size != 512)
		first_block = (first_block * volume_blk_size) / 512;

	encryption_info->data_encryption_key_index =
		get_unaligned_le16(&raid_map->data_encryption_key_index);
	encryption_info->encrypt_tweak_lower = lower_32_bits(first_block);
	encryption_info->encrypt_tweak_upper = upper_32_bits(first_block);
}

/*
 * Attempt to perform RAID bypass mapping for a logical volume I/O.
 */

#define PQI_RAID_BYPASS_INELIGIBLE	1

static int pqi_raid_bypass_submit_scsi_cmd(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device, struct scsi_cmnd *scmd,
	struct pqi_queue_group *queue_group)
{
	struct raid_map *raid_map;
	bool is_write = false;
	u32 map_index;
	u64 first_block;
	u64 last_block;
	u32 block_cnt;
	u32 blocks_per_row;
	u64 first_row;
	u64 last_row;
	u32 first_row_offset;
	u32 last_row_offset;
	u32 first_column;
	u32 last_column;
	u64 r0_first_row;
	u64 r0_last_row;
	u32 r5or6_blocks_per_row;
	u64 r5or6_first_row;
	u64 r5or6_last_row;
	u32 r5or6_first_row_offset;
	u32 r5or6_last_row_offset;
	u32 r5or6_first_column;
	u32 r5or6_last_column;
	u16 data_disks_per_row;
	u32 total_disks_per_row;
	u16 layout_map_count;
	u32 stripesize;
	u16 strip_size;
	u32 first_group;
	u32 last_group;
	u32 current_group;
	u32 map_row;
	u32 aio_handle;
	u64 disk_block;
	u32 disk_block_cnt;
	u8 cdb[16];
	u8 cdb_length;
	int offload_to_mirror;
	struct pqi_encryption_info *encryption_info_ptr;
	struct pqi_encryption_info encryption_info;
#if BITS_PER_LONG == 32
	u64 tmpdiv;
#endif

	/* Check for valid opcode, get LBA and block count. */
	switch (scmd->cmnd[0]) {
	case WRITE_6:
		is_write = true;
		/* fall through */
	case READ_6:
		first_block = (u64)(((scmd->cmnd[1] & 0x1f) << 16) |
			(scmd->cmnd[2] << 8) | scmd->cmnd[3]);
		block_cnt = (u32)scmd->cmnd[4];
		if (block_cnt == 0)
			block_cnt = 256;
		break;
	case WRITE_10:
		is_write = true;
		/* fall through */
	case READ_10:
		first_block = (u64)get_unaligned_be32(&scmd->cmnd[2]);
		block_cnt = (u32)get_unaligned_be16(&scmd->cmnd[7]);
		break;
	case WRITE_12:
		is_write = true;
		/* fall through */
	case READ_12:
		first_block = (u64)get_unaligned_be32(&scmd->cmnd[2]);
		block_cnt = get_unaligned_be32(&scmd->cmnd[6]);
		break;
	case WRITE_16:
		is_write = true;
		/* fall through */
	case READ_16:
		first_block = get_unaligned_be64(&scmd->cmnd[2]);
		block_cnt = get_unaligned_be32(&scmd->cmnd[10]);
		break;
	default:
		/* Process via normal I/O path. */
		return PQI_RAID_BYPASS_INELIGIBLE;
	}

	/* Check for write to non-RAID-0. */
	if (is_write && device->raid_level != SA_RAID_0)
		return PQI_RAID_BYPASS_INELIGIBLE;

	if (unlikely(block_cnt == 0))
		return PQI_RAID_BYPASS_INELIGIBLE;

	last_block = first_block + block_cnt - 1;
	raid_map = device->raid_map;

	/* Check for invalid block or wraparound. */
	if (last_block >= get_unaligned_le64(&raid_map->volume_blk_cnt) ||
		last_block < first_block)
		return PQI_RAID_BYPASS_INELIGIBLE;

	data_disks_per_row = get_unaligned_le16(&raid_map->data_disks_per_row);
	strip_size = get_unaligned_le16(&raid_map->strip_size);
	layout_map_count = get_unaligned_le16(&raid_map->layout_map_count);

	/* Calculate stripe information for the request. */
	blocks_per_row = data_disks_per_row * strip_size;
#if BITS_PER_LONG == 32
	tmpdiv = first_block;
	do_div(tmpdiv, blocks_per_row);
	first_row = tmpdiv;
	tmpdiv = last_block;
	do_div(tmpdiv, blocks_per_row);
	last_row = tmpdiv;
	first_row_offset = (u32)(first_block - (first_row * blocks_per_row));
	last_row_offset = (u32)(last_block - (last_row * blocks_per_row));
	tmpdiv = first_row_offset;
	do_div(tmpdiv, strip_size);
	first_column = tmpdiv;
	tmpdiv = last_row_offset;
	do_div(tmpdiv, strip_size);
	last_column = tmpdiv;
#else
	first_row = first_block / blocks_per_row;
	last_row = last_block / blocks_per_row;
	first_row_offset = (u32)(first_block - (first_row * blocks_per_row));
	last_row_offset = (u32)(last_block - (last_row * blocks_per_row));
	first_column = first_row_offset / strip_size;
	last_column = last_row_offset / strip_size;
#endif

	/* If this isn't a single row/column then give to the controller. */
	if (first_row != last_row || first_column != last_column)
		return PQI_RAID_BYPASS_INELIGIBLE;

	/* Proceeding with driver mapping. */
	total_disks_per_row = data_disks_per_row +
		get_unaligned_le16(&raid_map->metadata_disks_per_row);
	map_row = ((u32)(first_row >> raid_map->parity_rotation_shift)) %
		get_unaligned_le16(&raid_map->row_cnt);
	map_index = (map_row * total_disks_per_row) + first_column;

	/* RAID 1 */
	if (device->raid_level == SA_RAID_1) {
		if (device->offload_to_mirror)
			map_index += data_disks_per_row;
		device->offload_to_mirror = !device->offload_to_mirror;
	} else if (device->raid_level == SA_RAID_ADM) {
		/* RAID ADM */
		/*
		 * Handles N-way mirrors  (R1-ADM) and R10 with # of drives
		 * divisible by 3.
		 */
		offload_to_mirror = device->offload_to_mirror;
		if (offload_to_mirror == 0)  {
			/* use physical disk in the first mirrored group. */
			map_index %= data_disks_per_row;
		} else {
			do {
				/*
				 * Determine mirror group that map_index
				 * indicates.
				 */
				current_group = map_index / data_disks_per_row;

				if (offload_to_mirror != current_group) {
					if (current_group <
						layout_map_count - 1) {
						/*
						 * Select raid index from
						 * next group.
						 */
						map_index += data_disks_per_row;
						current_group++;
					} else {
						/*
						 * Select raid index from first
						 * group.
						 */
						map_index %= data_disks_per_row;
						current_group = 0;
					}
				}
			} while (offload_to_mirror != current_group);
		}

		/* Set mirror group to use next time. */
		offload_to_mirror =
			(offload_to_mirror >= layout_map_count - 1) ?
				0 : offload_to_mirror + 1;
		WARN_ON(offload_to_mirror >= layout_map_count);
		device->offload_to_mirror = offload_to_mirror;
		/*
		 * Avoid direct use of device->offload_to_mirror within this
		 * function since multiple threads might simultaneously
		 * increment it beyond the range of device->layout_map_count -1.
		 */
	} else if ((device->raid_level == SA_RAID_5 ||
		device->raid_level == SA_RAID_6) && layout_map_count > 1) {
		/* RAID 50/60 */
		/* Verify first and last block are in same RAID group */
		r5or6_blocks_per_row = strip_size * data_disks_per_row;
		stripesize = r5or6_blocks_per_row * layout_map_count;
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		first_group = do_div(tmpdiv, stripesize);
		tmpdiv = first_group;
		do_div(tmpdiv, r5or6_blocks_per_row);
		first_group = tmpdiv;
		tmpdiv = last_block;
		last_group = do_div(tmpdiv, stripesize);
		tmpdiv = last_group;
		do_div(tmpdiv, r5or6_blocks_per_row);
		last_group = tmpdiv;
#else
		first_group = (first_block % stripesize) / r5or6_blocks_per_row;
		last_group = (last_block % stripesize) / r5or6_blocks_per_row;
#endif
		if (first_group != last_group)
			return PQI_RAID_BYPASS_INELIGIBLE;

		/* Verify request is in a single row of RAID 5/6 */
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		do_div(tmpdiv, stripesize);
		first_row = r5or6_first_row = r0_first_row = tmpdiv;
		tmpdiv = last_block;
		do_div(tmpdiv, stripesize);
		r5or6_last_row = r0_last_row = tmpdiv;
#else
		first_row = r5or6_first_row = r0_first_row =
			first_block / stripesize;
		r5or6_last_row = r0_last_row = last_block / stripesize;
#endif
		if (r5or6_first_row != r5or6_last_row)
			return PQI_RAID_BYPASS_INELIGIBLE;

		/* Verify request is in a single column */
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		first_row_offset = do_div(tmpdiv, stripesize);
		tmpdiv = first_row_offset;
		first_row_offset = (u32)do_div(tmpdiv, r5or6_blocks_per_row);
		r5or6_first_row_offset = first_row_offset;
		tmpdiv = last_block;
		r5or6_last_row_offset = do_div(tmpdiv, stripesize);
		tmpdiv = r5or6_last_row_offset;
		r5or6_last_row_offset = do_div(tmpdiv, r5or6_blocks_per_row);
		tmpdiv = r5or6_first_row_offset;
		do_div(tmpdiv, strip_size);
		first_column = r5or6_first_column = tmpdiv;
		tmpdiv = r5or6_last_row_offset;
		do_div(tmpdiv, strip_size);
		r5or6_last_column = tmpdiv;
#else
		first_row_offset = r5or6_first_row_offset =
			(u32)((first_block % stripesize) %
			r5or6_blocks_per_row);

		r5or6_last_row_offset =
			(u32)((last_block % stripesize) %
			r5or6_blocks_per_row);

		first_column = r5or6_first_row_offset / strip_size;
		r5or6_first_column = first_column;
		r5or6_last_column = r5or6_last_row_offset / strip_size;
#endif
		if (r5or6_first_column != r5or6_last_column)
			return PQI_RAID_BYPASS_INELIGIBLE;

		/* Request is eligible */
		map_row =
			((u32)(first_row >> raid_map->parity_rotation_shift)) %
			get_unaligned_le16(&raid_map->row_cnt);

		map_index = (first_group *
			(get_unaligned_le16(&raid_map->row_cnt) *
			total_disks_per_row)) +
			(map_row * total_disks_per_row) + first_column;
	}

	if (unlikely(map_index >= RAID_MAP_MAX_ENTRIES))
		return PQI_RAID_BYPASS_INELIGIBLE;

	aio_handle = raid_map->disk_data[map_index].aio_handle;
	disk_block = get_unaligned_le64(&raid_map->disk_starting_blk) +
		first_row * strip_size +
		(first_row_offset - first_column * strip_size);
	disk_block_cnt = block_cnt;

	/* Handle differing logical/physical block sizes. */
	if (raid_map->phys_blk_shift) {
		disk_block <<= raid_map->phys_blk_shift;
		disk_block_cnt <<= raid_map->phys_blk_shift;
	}

	if (unlikely(disk_block_cnt > 0xffff))
		return PQI_RAID_BYPASS_INELIGIBLE;

	/* Build the new CDB for the physical disk I/O. */
	if (disk_block > 0xffffffff) {
		cdb[0] = is_write ? WRITE_16 : READ_16;
		cdb[1] = 0;
		put_unaligned_be64(disk_block, &cdb[2]);
		put_unaligned_be32(disk_block_cnt, &cdb[10]);
		cdb[14] = 0;
		cdb[15] = 0;
		cdb_length = 16;
	} else {
		cdb[0] = is_write ? WRITE_10 : READ_10;
		cdb[1] = 0;
		put_unaligned_be32((u32)disk_block, &cdb[2]);
		cdb[6] = 0;
		put_unaligned_be16((u16)disk_block_cnt, &cdb[7]);
		cdb[9] = 0;
		cdb_length = 10;
	}

	if (get_unaligned_le16(&raid_map->flags) &
		RAID_MAP_ENCRYPTION_ENABLED) {
		pqi_set_encryption_info(&encryption_info, raid_map,
			first_block);
		encryption_info_ptr = &encryption_info;
	} else {
		encryption_info_ptr = NULL;
	}

	return pqi_aio_submit_io(ctrl_info, scmd, aio_handle,
		cdb, cdb_length, queue_group, encryption_info_ptr, true);
}

#define PQI_STATUS_IDLE		0x0

#define PQI_CREATE_ADMIN_QUEUE_PAIR	1
#define PQI_DELETE_ADMIN_QUEUE_PAIR	2

#define PQI_DEVICE_STATE_POWER_ON_AND_RESET		0x0
#define PQI_DEVICE_STATE_STATUS_AVAILABLE		0x1
#define PQI_DEVICE_STATE_ALL_REGISTERS_READY		0x2
#define PQI_DEVICE_STATE_ADMIN_QUEUE_PAIR_READY		0x3
#define PQI_DEVICE_STATE_ERROR				0x4

#define PQI_MODE_READY_TIMEOUT_SECS		30
#define PQI_MODE_READY_POLL_INTERVAL_MSECS	1

static int pqi_wait_for_pqi_mode_ready(struct pqi_ctrl_info *ctrl_info)
{
	struct pqi_device_registers __iomem *pqi_registers;
	unsigned long timeout;
	u64 signature;
	u8 status;

	pqi_registers = ctrl_info->pqi_registers;
	timeout = (PQI_MODE_READY_TIMEOUT_SECS * HZ) + jiffies;

	while (1) {
		signature = readq(&pqi_registers->signature);
		if (memcmp(&signature, PQI_DEVICE_SIGNATURE,
			sizeof(signature)) == 0)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(&ctrl_info->pci_dev->dev,
				"timed out waiting for PQI signature\n");
			return -ETIMEDOUT;
		}
		msleep(PQI_MODE_READY_POLL_INTERVAL_MSECS);
	}

	while (1) {
		status = readb(&pqi_registers->function_and_status_code);
		if (status == PQI_STATUS_IDLE)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(&ctrl_info->pci_dev->dev,
				"timed out waiting for PQI IDLE\n");
			return -ETIMEDOUT;
		}
		msleep(PQI_MODE_READY_POLL_INTERVAL_MSECS);
	}

	while (1) {
		if (readl(&pqi_registers->device_status) ==
			PQI_DEVICE_STATE_ALL_REGISTERS_READY)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(&ctrl_info->pci_dev->dev,
				"timed out waiting for PQI all registers ready\n");
			return -ETIMEDOUT;
		}
		msleep(PQI_MODE_READY_POLL_INTERVAL_MSECS);
	}

	return 0;
}

static inline void pqi_aio_path_disabled(struct pqi_io_request *io_request)
{
	struct pqi_scsi_dev *device;

	device = io_request->scmd->device->hostdata;
	device->raid_bypass_enabled = false;
	device->aio_enabled = false;
}

static inline void pqi_take_device_offline(struct scsi_device *sdev, char *path)
{
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_scsi_dev *device;

	device = sdev->hostdata;
	if (device->device_offline)
		return;

	device->device_offline = true;
	scsi_device_set_state(sdev, SDEV_OFFLINE);
	ctrl_info = shost_to_hba(sdev->host);
	pqi_schedule_rescan_worker(ctrl_info);
	dev_err(&ctrl_info->pci_dev->dev, "offlined %s scsi %d:%d:%d:%d\n",
		path, ctrl_info->scsi_host->host_no, device->bus,
		device->target, device->lun);
}

static void pqi_process_raid_io_error(struct pqi_io_request *io_request)
{
	u8 scsi_status;
	u8 host_byte;
	struct scsi_cmnd *scmd;
	struct pqi_raid_error_info *error_info;
	size_t sense_data_length;
	int residual_count;
	int xfer_count;
	struct scsi_sense_hdr sshdr;

	scmd = io_request->scmd;
	if (!scmd)
		return;

	error_info = io_request->error_info;
	scsi_status = error_info->status;
	host_byte = DID_OK;

	switch (error_info->data_out_result) {
	case PQI_DATA_IN_OUT_GOOD:
		break;
	case PQI_DATA_IN_OUT_UNDERFLOW:
		xfer_count =
			get_unaligned_le32(&error_info->data_out_transferred);
		residual_count = scsi_bufflen(scmd) - xfer_count;
		scsi_set_resid(scmd, residual_count);
		if (xfer_count < scmd->underflow)
			host_byte = DID_SOFT_ERROR;
		break;
	case PQI_DATA_IN_OUT_UNSOLICITED_ABORT:
	case PQI_DATA_IN_OUT_ABORTED:
		host_byte = DID_ABORT;
		break;
	case PQI_DATA_IN_OUT_TIMEOUT:
		host_byte = DID_TIME_OUT;
		break;
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW:
	case PQI_DATA_IN_OUT_PROTOCOL_ERROR:
	case PQI_DATA_IN_OUT_BUFFER_ERROR:
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW_DESCRIPTOR_AREA:
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW_BRIDGE:
	case PQI_DATA_IN_OUT_ERROR:
	case PQI_DATA_IN_OUT_HARDWARE_ERROR:
	case PQI_DATA_IN_OUT_PCIE_FABRIC_ERROR:
	case PQI_DATA_IN_OUT_PCIE_COMPLETION_TIMEOUT:
	case PQI_DATA_IN_OUT_PCIE_COMPLETER_ABORT_RECEIVED:
	case PQI_DATA_IN_OUT_PCIE_UNSUPPORTED_REQUEST_RECEIVED:
	case PQI_DATA_IN_OUT_PCIE_ECRC_CHECK_FAILED:
	case PQI_DATA_IN_OUT_PCIE_UNSUPPORTED_REQUEST:
	case PQI_DATA_IN_OUT_PCIE_ACS_VIOLATION:
	case PQI_DATA_IN_OUT_PCIE_TLP_PREFIX_BLOCKED:
	case PQI_DATA_IN_OUT_PCIE_POISONED_MEMORY_READ:
	default:
		host_byte = DID_ERROR;
		break;
	}

	sense_data_length = get_unaligned_le16(&error_info->sense_data_length);
	if (sense_data_length == 0)
		sense_data_length =
			get_unaligned_le16(&error_info->response_data_length);
	if (sense_data_length) {
		if (sense_data_length > sizeof(error_info->data))
			sense_data_length = sizeof(error_info->data);

		if (scsi_status == SAM_STAT_CHECK_CONDITION &&
			scsi_normalize_sense(error_info->data,
				sense_data_length, &sshdr) &&
				sshdr.sense_key == HARDWARE_ERROR &&
				sshdr.asc == 0x3e &&
				sshdr.ascq == 0x1) {
			pqi_take_device_offline(scmd->device, "RAID");
			host_byte = DID_NO_CONNECT;
		}

		if (sense_data_length > SCSI_SENSE_BUFFERSIZE)
			sense_data_length = SCSI_SENSE_BUFFERSIZE;
		memcpy(scmd->sense_buffer, error_info->data,
			sense_data_length);
	}

	scmd->result = scsi_status;
	set_host_byte(scmd, host_byte);
}

static void pqi_process_aio_io_error(struct pqi_io_request *io_request)
{
	u8 scsi_status;
	u8 host_byte;
	struct scsi_cmnd *scmd;
	struct pqi_aio_error_info *error_info;
	size_t sense_data_length;
	int residual_count;
	int xfer_count;
	bool device_offline;

	scmd = io_request->scmd;
	error_info = io_request->error_info;
	host_byte = DID_OK;
	sense_data_length = 0;
	device_offline = false;

	switch (error_info->service_response) {
	case PQI_AIO_SERV_RESPONSE_COMPLETE:
		scsi_status = error_info->status;
		break;
	case PQI_AIO_SERV_RESPONSE_FAILURE:
		switch (error_info->status) {
		case PQI_AIO_STATUS_IO_ABORTED:
			scsi_status = SAM_STAT_TASK_ABORTED;
			break;
		case PQI_AIO_STATUS_UNDERRUN:
			scsi_status = SAM_STAT_GOOD;
			residual_count = get_unaligned_le32(
						&error_info->residual_count);
			scsi_set_resid(scmd, residual_count);
			xfer_count = scsi_bufflen(scmd) - residual_count;
			if (xfer_count < scmd->underflow)
				host_byte = DID_SOFT_ERROR;
			break;
		case PQI_AIO_STATUS_OVERRUN:
			scsi_status = SAM_STAT_GOOD;
			break;
		case PQI_AIO_STATUS_AIO_PATH_DISABLED:
			pqi_aio_path_disabled(io_request);
			scsi_status = SAM_STAT_GOOD;
			io_request->status = -EAGAIN;
			break;
		case PQI_AIO_STATUS_NO_PATH_TO_DEVICE:
		case PQI_AIO_STATUS_INVALID_DEVICE:
			if (!io_request->raid_bypass) {
				device_offline = true;
				pqi_take_device_offline(scmd->device, "AIO");
				host_byte = DID_NO_CONNECT;
			}
			scsi_status = SAM_STAT_CHECK_CONDITION;
			break;
		case PQI_AIO_STATUS_IO_ERROR:
		default:
			scsi_status = SAM_STAT_CHECK_CONDITION;
			break;
		}
		break;
	case PQI_AIO_SERV_RESPONSE_TMF_COMPLETE:
	case PQI_AIO_SERV_RESPONSE_TMF_SUCCEEDED:
		scsi_status = SAM_STAT_GOOD;
		break;
	case PQI_AIO_SERV_RESPONSE_TMF_REJECTED:
	case PQI_AIO_SERV_RESPONSE_TMF_INCORRECT_LUN:
	default:
		scsi_status = SAM_STAT_CHECK_CONDITION;
		break;
	}

	if (error_info->data_present) {
		sense_data_length =
			get_unaligned_le16(&error_info->data_length);
		if (sense_data_length) {
			if (sense_data_length > sizeof(error_info->data))
				sense_data_length = sizeof(error_info->data);
			if (sense_data_length > SCSI_SENSE_BUFFERSIZE)
				sense_data_length = SCSI_SENSE_BUFFERSIZE;
			memcpy(scmd->sense_buffer, error_info->data,
				sense_data_length);
		}
	}

	if (device_offline && sense_data_length == 0)
		scsi_build_sense_buffer(0, scmd->sense_buffer, HARDWARE_ERROR,
			0x3e, 0x1);

	scmd->result = scsi_status;
	set_host_byte(scmd, host_byte);
}

static void pqi_process_io_error(unsigned int iu_type,
	struct pqi_io_request *io_request)
{
	switch (iu_type) {
	case PQI_RESPONSE_IU_RAID_PATH_IO_ERROR:
		pqi_process_raid_io_error(io_request);
		break;
	case PQI_RESPONSE_IU_AIO_PATH_IO_ERROR:
		pqi_process_aio_io_error(io_request);
		break;
	}
}

static int pqi_interpret_task_management_response(
	struct pqi_task_management_response *response)
{
	int rc;

	switch (response->response_code) {
	case SOP_TMF_COMPLETE:
	case SOP_TMF_FUNCTION_SUCCEEDED:
		rc = 0;
		break;
	default:
		rc = -EIO;
		break;
	}

	return rc;
}

static unsigned int pqi_process_io_intr(struct pqi_ctrl_info *ctrl_info,
	struct pqi_queue_group *queue_group)
{
	unsigned int num_responses;
	pqi_index_t oq_pi;
	pqi_index_t oq_ci;
	struct pqi_io_request *io_request;
	struct pqi_io_response *response;
	u16 request_id;

	num_responses = 0;
	oq_ci = queue_group->oq_ci_copy;

	while (1) {
		oq_pi = *queue_group->oq_pi;
		if (oq_pi == oq_ci)
			break;

		num_responses++;
		response = queue_group->oq_element_array +
			(oq_ci * PQI_OPERATIONAL_OQ_ELEMENT_LENGTH);

		request_id = get_unaligned_le16(&response->request_id);
		WARN_ON(request_id >= ctrl_info->max_io_slots);

		io_request = &ctrl_info->io_request_pool[request_id];
		WARN_ON(atomic_read(&io_request->refcount) == 0);

		switch (response->header.iu_type) {
		case PQI_RESPONSE_IU_RAID_PATH_IO_SUCCESS:
		case PQI_RESPONSE_IU_AIO_PATH_IO_SUCCESS:
		case PQI_RESPONSE_IU_GENERAL_MANAGEMENT:
			break;
		case PQI_RESPONSE_IU_TASK_MANAGEMENT:
			io_request->status =
				pqi_interpret_task_management_response(
					(void *)response);
			break;
		case PQI_RESPONSE_IU_AIO_PATH_DISABLED:
			pqi_aio_path_disabled(io_request);
			io_request->status = -EAGAIN;
			break;
		case PQI_RESPONSE_IU_RAID_PATH_IO_ERROR:
		case PQI_RESPONSE_IU_AIO_PATH_IO_ERROR:
			io_request->error_info = ctrl_info->error_buffer +
				(get_unaligned_le16(&response->error_index) *
				PQI_ERROR_BUFFER_ELEMENT_LENGTH);
			pqi_process_io_error(response->header.iu_type,
				io_request);
			break;
		default:
			dev_err(&ctrl_info->pci_dev->dev,
				"unexpected IU type: 0x%x\n",
				response->header.iu_type);
			break;
		}

		io_request->io_complete_callback(io_request,
			io_request->context);

		/*
		 * Note that the I/O request structure CANNOT BE TOUCHED after
		 * returning from the I/O completion callback!
		 */

		oq_ci = (oq_ci + 1) % ctrl_info->num_elements_per_oq;
	}

	if (num_responses) {
		queue_group->oq_ci_copy = oq_ci;
		writel(oq_ci, queue_group->oq_ci);
	}

	return num_responses;
}

static inline unsigned int pqi_num_elements_free(unsigned int pi,
	unsigned int ci, unsigned int elements_in_queue)
{
	unsigned int num_elements_used;

	if (pi >= ci)
		num_elements_used = pi - ci;
	else
		num_elements_used = elements_in_queue - ci + pi;

	return elements_in_queue - num_elements_used - 1;
}

static void pqi_send_event_ack(struct pqi_ctrl_info *ctrl_info,
	struct pqi_event_acknowledge_request *iu, size_t iu_length)
{
	pqi_index_t iq_pi;
	pqi_index_t iq_ci;
	unsigned long flags;
	void *next_element;
	struct pqi_queue_group *queue_group;

	queue_group = &ctrl_info->queue_groups[PQI_DEFAULT_QUEUE_GROUP];
	put_unaligned_le16(queue_group->oq_id, &iu->header.response_queue_id);

	while (1) {
		spin_lock_irqsave(&queue_group->submit_lock[RAID_PATH], flags);

		iq_pi = queue_group->iq_pi_copy[RAID_PATH];
		iq_ci = *queue_group->iq_ci[RAID_PATH];

		if (pqi_num_elements_free(iq_pi, iq_ci,
			ctrl_info->num_elements_per_iq))
			break;

		spin_unlock_irqrestore(
			&queue_group->submit_lock[RAID_PATH], flags);

		if (pqi_ctrl_offline(ctrl_info))
			return;
	}

	next_element = queue_group->iq_element_array[RAID_PATH] +
		(iq_pi * PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

	memcpy(next_element, iu, iu_length);

	iq_pi = (iq_pi + 1) % ctrl_info->num_elements_per_iq;
	queue_group->iq_pi_copy[RAID_PATH] = iq_pi;

	/*
	 * This write notifies the controller that an IU is available to be
	 * processed.
	 */
	writel(iq_pi, queue_group->iq_pi[RAID_PATH]);

	spin_unlock_irqrestore(&queue_group->submit_lock[RAID_PATH], flags);
}

static void pqi_acknowledge_event(struct pqi_ctrl_info *ctrl_info,
	struct pqi_event *event)
{
	struct pqi_event_acknowledge_request request;

	memset(&request, 0, sizeof(request));

	request.header.iu_type = PQI_REQUEST_IU_ACKNOWLEDGE_VENDOR_EVENT;
	put_unaligned_le16(sizeof(request) - PQI_REQUEST_HEADER_LENGTH,
		&request.header.iu_length);
	request.event_type = event->event_type;
	request.event_id = event->event_id;
	request.additional_event_id = event->additional_event_id;

	pqi_send_event_ack(ctrl_info, &request, sizeof(request));
}

static void pqi_event_worker(struct work_struct *work)
{
	unsigned int i;
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_event *event;

	ctrl_info = container_of(work, struct pqi_ctrl_info, event_work);

	pqi_ctrl_busy(ctrl_info);
	pqi_wait_if_ctrl_blocked(ctrl_info, NO_TIMEOUT);
	if (pqi_ctrl_offline(ctrl_info))
		goto out;

	pqi_schedule_rescan_worker_delayed(ctrl_info);

	event = ctrl_info->events;
	for (i = 0; i < PQI_NUM_SUPPORTED_EVENTS; i++) {
		if (event->pending) {
			event->pending = false;
			pqi_acknowledge_event(ctrl_info, event);
		}
		event++;
	}

out:
	pqi_ctrl_unbusy(ctrl_info);
}

#define PQI_HEARTBEAT_TIMER_INTERVAL	(10 * HZ)

static void pqi_heartbeat_timer_handler(struct timer_list *t)
{
	int num_interrupts;
	u32 heartbeat_count;
	struct pqi_ctrl_info *ctrl_info = from_timer(ctrl_info, t,
						     heartbeat_timer);

	pqi_check_ctrl_health(ctrl_info);
	if (pqi_ctrl_offline(ctrl_info))
		return;

	num_interrupts = atomic_read(&ctrl_info->num_interrupts);
	heartbeat_count = pqi_read_heartbeat_counter(ctrl_info);

	if (num_interrupts == ctrl_info->previous_num_interrupts) {
		if (heartbeat_count == ctrl_info->previous_heartbeat_count) {
			dev_err(&ctrl_info->pci_dev->dev,
				"no heartbeat detected - last heartbeat count: %u\n",
				heartbeat_count);
			pqi_take_ctrl_offline(ctrl_info);
			return;
		}
	} else {
		ctrl_info->previous_num_interrupts = num_interrupts;
	}

	ctrl_info->previous_heartbeat_count = heartbeat_count;
	mod_timer(&ctrl_info->heartbeat_timer,
		jiffies + PQI_HEARTBEAT_TIMER_INTERVAL);
}

static void pqi_start_heartbeat_timer(struct pqi_ctrl_info *ctrl_info)
{
	if (!ctrl_info->heartbeat_counter)
		return;

	ctrl_info->previous_num_interrupts =
		atomic_read(&ctrl_info->num_interrupts);
	ctrl_info->previous_heartbeat_count =
		pqi_read_heartbeat_counter(ctrl_info);

	ctrl_info->heartbeat_timer.expires =
		jiffies + PQI_HEARTBEAT_TIMER_INTERVAL;
	add_timer(&ctrl_info->heartbeat_timer);
}

static inline void pqi_stop_heartbeat_timer(struct pqi_ctrl_info *ctrl_info)
{
	del_timer_sync(&ctrl_info->heartbeat_timer);
}

static inline int pqi_event_type_to_event_index(unsigned int event_type)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(pqi_supported_event_types); index++)
		if (event_type == pqi_supported_event_types[index])
			return index;

	return -1;
}

static inline bool pqi_is_supported_event(unsigned int event_type)
{
	return pqi_event_type_to_event_index(event_type) != -1;
}

static unsigned int pqi_process_event_intr(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int num_events;
	pqi_index_t oq_pi;
	pqi_index_t oq_ci;
	struct pqi_event_queue *event_queue;
	struct pqi_event_response *response;
	struct pqi_event *event;
	int event_index;

	event_queue = &ctrl_info->event_queue;
	num_events = 0;
	oq_ci = event_queue->oq_ci_copy;

	while (1) {
		oq_pi = *event_queue->oq_pi;
		if (oq_pi == oq_ci)
			break;

		num_events++;
		response = event_queue->oq_element_array +
			(oq_ci * PQI_EVENT_OQ_ELEMENT_LENGTH);

		event_index =
			pqi_event_type_to_event_index(response->event_type);

		if (event_index >= 0) {
			if (response->request_acknowlege) {
				event = &ctrl_info->events[event_index];
				event->pending = true;
				event->event_type = response->event_type;
				event->event_id = response->event_id;
				event->additional_event_id =
					response->additional_event_id;
			}
		}

		oq_ci = (oq_ci + 1) % PQI_NUM_EVENT_QUEUE_ELEMENTS;
	}

	if (num_events) {
		event_queue->oq_ci_copy = oq_ci;
		writel(oq_ci, event_queue->oq_ci);
		schedule_work(&ctrl_info->event_work);
	}

	return num_events;
}

#define PQI_LEGACY_INTX_MASK	0x1

static inline void pqi_configure_legacy_intx(struct pqi_ctrl_info *ctrl_info,
						bool enable_intx)
{
	u32 intx_mask;
	struct pqi_device_registers __iomem *pqi_registers;
	volatile void __iomem *register_addr;

	pqi_registers = ctrl_info->pqi_registers;

	if (enable_intx)
		register_addr = &pqi_registers->legacy_intx_mask_clear;
	else
		register_addr = &pqi_registers->legacy_intx_mask_set;

	intx_mask = readl(register_addr);
	intx_mask |= PQI_LEGACY_INTX_MASK;
	writel(intx_mask, register_addr);
}

static void pqi_change_irq_mode(struct pqi_ctrl_info *ctrl_info,
	enum pqi_irq_mode new_mode)
{
	switch (ctrl_info->irq_mode) {
	case IRQ_MODE_MSIX:
		switch (new_mode) {
		case IRQ_MODE_MSIX:
			break;
		case IRQ_MODE_INTX:
			pqi_configure_legacy_intx(ctrl_info, true);
			sis_enable_intx(ctrl_info);
			break;
		case IRQ_MODE_NONE:
			break;
		}
		break;
	case IRQ_MODE_INTX:
		switch (new_mode) {
		case IRQ_MODE_MSIX:
			pqi_configure_legacy_intx(ctrl_info, false);
			sis_enable_msix(ctrl_info);
			break;
		case IRQ_MODE_INTX:
			break;
		case IRQ_MODE_NONE:
			pqi_configure_legacy_intx(ctrl_info, false);
			break;
		}
		break;
	case IRQ_MODE_NONE:
		switch (new_mode) {
		case IRQ_MODE_MSIX:
			sis_enable_msix(ctrl_info);
			break;
		case IRQ_MODE_INTX:
			pqi_configure_legacy_intx(ctrl_info, true);
			sis_enable_intx(ctrl_info);
			break;
		case IRQ_MODE_NONE:
			break;
		}
		break;
	}

	ctrl_info->irq_mode = new_mode;
}

#define PQI_LEGACY_INTX_PENDING		0x1

static inline bool pqi_is_valid_irq(struct pqi_ctrl_info *ctrl_info)
{
	bool valid_irq;
	u32 intx_status;

	switch (ctrl_info->irq_mode) {
	case IRQ_MODE_MSIX:
		valid_irq = true;
		break;
	case IRQ_MODE_INTX:
		intx_status =
			readl(&ctrl_info->pqi_registers->legacy_intx_status);
		if (intx_status & PQI_LEGACY_INTX_PENDING)
			valid_irq = true;
		else
			valid_irq = false;
		break;
	case IRQ_MODE_NONE:
	default:
		valid_irq = false;
		break;
	}

	return valid_irq;
}

static irqreturn_t pqi_irq_handler(int irq, void *data)
{
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_queue_group *queue_group;
	unsigned int num_responses_handled;

	queue_group = data;
	ctrl_info = queue_group->ctrl_info;

	if (!pqi_is_valid_irq(ctrl_info))
		return IRQ_NONE;

	num_responses_handled = pqi_process_io_intr(ctrl_info, queue_group);

	if (irq == ctrl_info->event_irq)
		num_responses_handled += pqi_process_event_intr(ctrl_info);

	if (num_responses_handled)
		atomic_inc(&ctrl_info->num_interrupts);

	pqi_start_io(ctrl_info, queue_group, RAID_PATH, NULL);
	pqi_start_io(ctrl_info, queue_group, AIO_PATH, NULL);

	return IRQ_HANDLED;
}

static int pqi_request_irqs(struct pqi_ctrl_info *ctrl_info)
{
	struct pci_dev *pci_dev = ctrl_info->pci_dev;
	int i;
	int rc;

	ctrl_info->event_irq = pci_irq_vector(pci_dev, 0);

	for (i = 0; i < ctrl_info->num_msix_vectors_enabled; i++) {
		rc = request_irq(pci_irq_vector(pci_dev, i), pqi_irq_handler, 0,
			DRIVER_NAME_SHORT, &ctrl_info->queue_groups[i]);
		if (rc) {
			dev_err(&pci_dev->dev,
				"irq %u init failed with error %d\n",
				pci_irq_vector(pci_dev, i), rc);
			return rc;
		}
		ctrl_info->num_msix_vectors_initialized++;
	}

	return 0;
}

static void pqi_free_irqs(struct pqi_ctrl_info *ctrl_info)
{
	int i;

	for (i = 0; i < ctrl_info->num_msix_vectors_initialized; i++)
		free_irq(pci_irq_vector(ctrl_info->pci_dev, i),
			&ctrl_info->queue_groups[i]);

	ctrl_info->num_msix_vectors_initialized = 0;
}

static int pqi_enable_msix_interrupts(struct pqi_ctrl_info *ctrl_info)
{
	int num_vectors_enabled;

	num_vectors_enabled = pci_alloc_irq_vectors(ctrl_info->pci_dev,
			PQI_MIN_MSIX_VECTORS, ctrl_info->num_queue_groups,
			PCI_IRQ_MSIX | PCI_IRQ_AFFINITY);
	if (num_vectors_enabled < 0) {
		dev_err(&ctrl_info->pci_dev->dev,
			"MSI-X init failed with error %d\n",
			num_vectors_enabled);
		return num_vectors_enabled;
	}

	ctrl_info->num_msix_vectors_enabled = num_vectors_enabled;
	ctrl_info->irq_mode = IRQ_MODE_MSIX;
	return 0;
}

static void pqi_disable_msix_interrupts(struct pqi_ctrl_info *ctrl_info)
{
	if (ctrl_info->num_msix_vectors_enabled) {
		pci_free_irq_vectors(ctrl_info->pci_dev);
		ctrl_info->num_msix_vectors_enabled = 0;
	}
}

static int pqi_alloc_operational_queues(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	size_t alloc_length;
	size_t element_array_length_per_iq;
	size_t element_array_length_per_oq;
	void *element_array;
	void *next_queue_index;
	void *aligned_pointer;
	unsigned int num_inbound_queues;
	unsigned int num_outbound_queues;
	unsigned int num_queue_indexes;
	struct pqi_queue_group *queue_group;

	element_array_length_per_iq =
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH *
		ctrl_info->num_elements_per_iq;
	element_array_length_per_oq =
		PQI_OPERATIONAL_OQ_ELEMENT_LENGTH *
		ctrl_info->num_elements_per_oq;
	num_inbound_queues = ctrl_info->num_queue_groups * 2;
	num_outbound_queues = ctrl_info->num_queue_groups;
	num_queue_indexes = (ctrl_info->num_queue_groups * 3) + 1;

	aligned_pointer = NULL;

	for (i = 0; i < num_inbound_queues; i++) {
		aligned_pointer = PTR_ALIGN(aligned_pointer,
			PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
		aligned_pointer += element_array_length_per_iq;
	}

	for (i = 0; i < num_outbound_queues; i++) {
		aligned_pointer = PTR_ALIGN(aligned_pointer,
			PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
		aligned_pointer += element_array_length_per_oq;
	}

	aligned_pointer = PTR_ALIGN(aligned_pointer,
		PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
	aligned_pointer += PQI_NUM_EVENT_QUEUE_ELEMENTS *
		PQI_EVENT_OQ_ELEMENT_LENGTH;

	for (i = 0; i < num_queue_indexes; i++) {
		aligned_pointer = PTR_ALIGN(aligned_pointer,
			PQI_OPERATIONAL_INDEX_ALIGNMENT);
		aligned_pointer += sizeof(pqi_index_t);
	}

	alloc_length = (size_t)aligned_pointer +
		PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT;

	alloc_length += PQI_EXTRA_SGL_MEMORY;

	ctrl_info->queue_memory_base =
		dma_zalloc_coherent(&ctrl_info->pci_dev->dev,
			alloc_length,
			&ctrl_info->queue_memory_base_dma_handle, GFP_KERNEL);

	if (!ctrl_info->queue_memory_base)
		return -ENOMEM;

	ctrl_info->queue_memory_length = alloc_length;

	element_array = PTR_ALIGN(ctrl_info->queue_memory_base,
		PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		queue_group = &ctrl_info->queue_groups[i];
		queue_group->iq_element_array[RAID_PATH] = element_array;
		queue_group->iq_element_array_bus_addr[RAID_PATH] =
			ctrl_info->queue_memory_base_dma_handle +
				(element_array - ctrl_info->queue_memory_base);
		element_array += element_array_length_per_iq;
		element_array = PTR_ALIGN(element_array,
			PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
		queue_group->iq_element_array[AIO_PATH] = element_array;
		queue_group->iq_element_array_bus_addr[AIO_PATH] =
			ctrl_info->queue_memory_base_dma_handle +
			(element_array - ctrl_info->queue_memory_base);
		element_array += element_array_length_per_iq;
		element_array = PTR_ALIGN(element_array,
			PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
	}

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		queue_group = &ctrl_info->queue_groups[i];
		queue_group->oq_element_array = element_array;
		queue_group->oq_element_array_bus_addr =
			ctrl_info->queue_memory_base_dma_handle +
			(element_array - ctrl_info->queue_memory_base);
		element_array += element_array_length_per_oq;
		element_array = PTR_ALIGN(element_array,
			PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
	}

	ctrl_info->event_queue.oq_element_array = element_array;
	ctrl_info->event_queue.oq_element_array_bus_addr =
		ctrl_info->queue_memory_base_dma_handle +
		(element_array - ctrl_info->queue_memory_base);
	element_array += PQI_NUM_EVENT_QUEUE_ELEMENTS *
		PQI_EVENT_OQ_ELEMENT_LENGTH;

	next_queue_index = PTR_ALIGN(element_array,
		PQI_OPERATIONAL_INDEX_ALIGNMENT);

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		queue_group = &ctrl_info->queue_groups[i];
		queue_group->iq_ci[RAID_PATH] = next_queue_index;
		queue_group->iq_ci_bus_addr[RAID_PATH] =
			ctrl_info->queue_memory_base_dma_handle +
			(next_queue_index - ctrl_info->queue_memory_base);
		next_queue_index += sizeof(pqi_index_t);
		next_queue_index = PTR_ALIGN(next_queue_index,
			PQI_OPERATIONAL_INDEX_ALIGNMENT);
		queue_group->iq_ci[AIO_PATH] = next_queue_index;
		queue_group->iq_ci_bus_addr[AIO_PATH] =
			ctrl_info->queue_memory_base_dma_handle +
			(next_queue_index - ctrl_info->queue_memory_base);
		next_queue_index += sizeof(pqi_index_t);
		next_queue_index = PTR_ALIGN(next_queue_index,
			PQI_OPERATIONAL_INDEX_ALIGNMENT);
		queue_group->oq_pi = next_queue_index;
		queue_group->oq_pi_bus_addr =
			ctrl_info->queue_memory_base_dma_handle +
			(next_queue_index - ctrl_info->queue_memory_base);
		next_queue_index += sizeof(pqi_index_t);
		next_queue_index = PTR_ALIGN(next_queue_index,
			PQI_OPERATIONAL_INDEX_ALIGNMENT);
	}

	ctrl_info->event_queue.oq_pi = next_queue_index;
	ctrl_info->event_queue.oq_pi_bus_addr =
		ctrl_info->queue_memory_base_dma_handle +
		(next_queue_index - ctrl_info->queue_memory_base);

	return 0;
}

static void pqi_init_operational_queues(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	u16 next_iq_id = PQI_MIN_OPERATIONAL_QUEUE_ID;
	u16 next_oq_id = PQI_MIN_OPERATIONAL_QUEUE_ID;

	/*
	 * Initialize the backpointers to the controller structure in
	 * each operational queue group structure.
	 */
	for (i = 0; i < ctrl_info->num_queue_groups; i++)
		ctrl_info->queue_groups[i].ctrl_info = ctrl_info;

	/*
	 * Assign IDs to all operational queues.  Note that the IDs
	 * assigned to operational IQs are independent of the IDs
	 * assigned to operational OQs.
	 */
	ctrl_info->event_queue.oq_id = next_oq_id++;
	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		ctrl_info->queue_groups[i].iq_id[RAID_PATH] = next_iq_id++;
		ctrl_info->queue_groups[i].iq_id[AIO_PATH] = next_iq_id++;
		ctrl_info->queue_groups[i].oq_id = next_oq_id++;
	}

	/*
	 * Assign MSI-X table entry indexes to all queues.  Note that the
	 * interrupt for the event queue is shared with the first queue group.
	 */
	ctrl_info->event_queue.int_msg_num = 0;
	for (i = 0; i < ctrl_info->num_queue_groups; i++)
		ctrl_info->queue_groups[i].int_msg_num = i;

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		spin_lock_init(&ctrl_info->queue_groups[i].submit_lock[0]);
		spin_lock_init(&ctrl_info->queue_groups[i].submit_lock[1]);
		INIT_LIST_HEAD(&ctrl_info->queue_groups[i].request_list[0]);
		INIT_LIST_HEAD(&ctrl_info->queue_groups[i].request_list[1]);
	}
}

static int pqi_alloc_admin_queues(struct pqi_ctrl_info *ctrl_info)
{
	size_t alloc_length;
	struct pqi_admin_queues_aligned *admin_queues_aligned;
	struct pqi_admin_queues *admin_queues;

	alloc_length = sizeof(struct pqi_admin_queues_aligned) +
		PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT;

	ctrl_info->admin_queue_memory_base =
		dma_zalloc_coherent(&ctrl_info->pci_dev->dev,
			alloc_length,
			&ctrl_info->admin_queue_memory_base_dma_handle,
			GFP_KERNEL);

	if (!ctrl_info->admin_queue_memory_base)
		return -ENOMEM;

	ctrl_info->admin_queue_memory_length = alloc_length;

	admin_queues = &ctrl_info->admin_queues;
	admin_queues_aligned = PTR_ALIGN(ctrl_info->admin_queue_memory_base,
		PQI_QUEUE_ELEMENT_ARRAY_ALIGNMENT);
	admin_queues->iq_element_array =
		&admin_queues_aligned->iq_element_array;
	admin_queues->oq_element_array =
		&admin_queues_aligned->oq_element_array;
	admin_queues->iq_ci = &admin_queues_aligned->iq_ci;
	admin_queues->oq_pi = &admin_queues_aligned->oq_pi;

	admin_queues->iq_element_array_bus_addr =
		ctrl_info->admin_queue_memory_base_dma_handle +
		(admin_queues->iq_element_array -
		ctrl_info->admin_queue_memory_base);
	admin_queues->oq_element_array_bus_addr =
		ctrl_info->admin_queue_memory_base_dma_handle +
		(admin_queues->oq_element_array -
		ctrl_info->admin_queue_memory_base);
	admin_queues->iq_ci_bus_addr =
		ctrl_info->admin_queue_memory_base_dma_handle +
		((void *)admin_queues->iq_ci -
		ctrl_info->admin_queue_memory_base);
	admin_queues->oq_pi_bus_addr =
		ctrl_info->admin_queue_memory_base_dma_handle +
		((void *)admin_queues->oq_pi -
		ctrl_info->admin_queue_memory_base);

	return 0;
}

#define PQI_ADMIN_QUEUE_CREATE_TIMEOUT_JIFFIES		HZ
#define PQI_ADMIN_QUEUE_CREATE_POLL_INTERVAL_MSECS	1

static int pqi_create_admin_queues(struct pqi_ctrl_info *ctrl_info)
{
	struct pqi_device_registers __iomem *pqi_registers;
	struct pqi_admin_queues *admin_queues;
	unsigned long timeout;
	u8 status;
	u32 reg;

	pqi_registers = ctrl_info->pqi_registers;
	admin_queues = &ctrl_info->admin_queues;

	writeq((u64)admin_queues->iq_element_array_bus_addr,
		&pqi_registers->admin_iq_element_array_addr);
	writeq((u64)admin_queues->oq_element_array_bus_addr,
		&pqi_registers->admin_oq_element_array_addr);
	writeq((u64)admin_queues->iq_ci_bus_addr,
		&pqi_registers->admin_iq_ci_addr);
	writeq((u64)admin_queues->oq_pi_bus_addr,
		&pqi_registers->admin_oq_pi_addr);

	reg = PQI_ADMIN_IQ_NUM_ELEMENTS |
		(PQI_ADMIN_OQ_NUM_ELEMENTS) << 8 |
		(admin_queues->int_msg_num << 16);
	writel(reg, &pqi_registers->admin_iq_num_elements);
	writel(PQI_CREATE_ADMIN_QUEUE_PAIR,
		&pqi_registers->function_and_status_code);

	timeout = PQI_ADMIN_QUEUE_CREATE_TIMEOUT_JIFFIES + jiffies;
	while (1) {
		status = readb(&pqi_registers->function_and_status_code);
		if (status == PQI_STATUS_IDLE)
			break;
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		msleep(PQI_ADMIN_QUEUE_CREATE_POLL_INTERVAL_MSECS);
	}

	/*
	 * The offset registers are not initialized to the correct
	 * offsets until *after* the create admin queue pair command
	 * completes successfully.
	 */
	admin_queues->iq_pi = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		readq(&pqi_registers->admin_iq_pi_offset);
	admin_queues->oq_ci = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		readq(&pqi_registers->admin_oq_ci_offset);

	return 0;
}

static void pqi_submit_admin_request(struct pqi_ctrl_info *ctrl_info,
	struct pqi_general_admin_request *request)
{
	struct pqi_admin_queues *admin_queues;
	void *next_element;
	pqi_index_t iq_pi;

	admin_queues = &ctrl_info->admin_queues;
	iq_pi = admin_queues->iq_pi_copy;

	next_element = admin_queues->iq_element_array +
		(iq_pi * PQI_ADMIN_IQ_ELEMENT_LENGTH);

	memcpy(next_element, request, sizeof(*request));

	iq_pi = (iq_pi + 1) % PQI_ADMIN_IQ_NUM_ELEMENTS;
	admin_queues->iq_pi_copy = iq_pi;

	/*
	 * This write notifies the controller that an IU is available to be
	 * processed.
	 */
	writel(iq_pi, admin_queues->iq_pi);
}

#define PQI_ADMIN_REQUEST_TIMEOUT_SECS	60

static int pqi_poll_for_admin_response(struct pqi_ctrl_info *ctrl_info,
	struct pqi_general_admin_response *response)
{
	struct pqi_admin_queues *admin_queues;
	pqi_index_t oq_pi;
	pqi_index_t oq_ci;
	unsigned long timeout;

	admin_queues = &ctrl_info->admin_queues;
	oq_ci = admin_queues->oq_ci_copy;

	timeout = (PQI_ADMIN_REQUEST_TIMEOUT_SECS * HZ) + jiffies;

	while (1) {
		oq_pi = *admin_queues->oq_pi;
		if (oq_pi != oq_ci)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(&ctrl_info->pci_dev->dev,
				"timed out waiting for admin response\n");
			return -ETIMEDOUT;
		}
		if (!sis_is_firmware_running(ctrl_info))
			return -ENXIO;
		usleep_range(1000, 2000);
	}

	memcpy(response, admin_queues->oq_element_array +
		(oq_ci * PQI_ADMIN_OQ_ELEMENT_LENGTH), sizeof(*response));

	oq_ci = (oq_ci + 1) % PQI_ADMIN_OQ_NUM_ELEMENTS;
	admin_queues->oq_ci_copy = oq_ci;
	writel(oq_ci, admin_queues->oq_ci);

	return 0;
}

static void pqi_start_io(struct pqi_ctrl_info *ctrl_info,
	struct pqi_queue_group *queue_group, enum pqi_io_path path,
	struct pqi_io_request *io_request)
{
	struct pqi_io_request *next;
	void *next_element;
	pqi_index_t iq_pi;
	pqi_index_t iq_ci;
	size_t iu_length;
	unsigned long flags;
	unsigned int num_elements_needed;
	unsigned int num_elements_to_end_of_queue;
	size_t copy_count;
	struct pqi_iu_header *request;

	spin_lock_irqsave(&queue_group->submit_lock[path], flags);

	if (io_request) {
		io_request->queue_group = queue_group;
		list_add_tail(&io_request->request_list_entry,
			&queue_group->request_list[path]);
	}

	iq_pi = queue_group->iq_pi_copy[path];

	list_for_each_entry_safe(io_request, next,
		&queue_group->request_list[path], request_list_entry) {

		request = io_request->iu;

		iu_length = get_unaligned_le16(&request->iu_length) +
			PQI_REQUEST_HEADER_LENGTH;
		num_elements_needed =
			DIV_ROUND_UP(iu_length,
				PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

		iq_ci = *queue_group->iq_ci[path];

		if (num_elements_needed > pqi_num_elements_free(iq_pi, iq_ci,
			ctrl_info->num_elements_per_iq))
			break;

		put_unaligned_le16(queue_group->oq_id,
			&request->response_queue_id);

		next_element = queue_group->iq_element_array[path] +
			(iq_pi * PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

		num_elements_to_end_of_queue =
			ctrl_info->num_elements_per_iq - iq_pi;

		if (num_elements_needed <= num_elements_to_end_of_queue) {
			memcpy(next_element, request, iu_length);
		} else {
			copy_count = num_elements_to_end_of_queue *
				PQI_OPERATIONAL_IQ_ELEMENT_LENGTH;
			memcpy(next_element, request, copy_count);
			memcpy(queue_group->iq_element_array[path],
				(u8 *)request + copy_count,
				iu_length - copy_count);
		}

		iq_pi = (iq_pi + num_elements_needed) %
			ctrl_info->num_elements_per_iq;

		list_del(&io_request->request_list_entry);
	}

	if (iq_pi != queue_group->iq_pi_copy[path]) {
		queue_group->iq_pi_copy[path] = iq_pi;
		/*
		 * This write notifies the controller that one or more IUs are
		 * available to be processed.
		 */
		writel(iq_pi, queue_group->iq_pi[path]);
	}

	spin_unlock_irqrestore(&queue_group->submit_lock[path], flags);
}

#define PQI_WAIT_FOR_COMPLETION_IO_TIMEOUT_SECS		10

static int pqi_wait_for_completion_io(struct pqi_ctrl_info *ctrl_info,
	struct completion *wait)
{
	int rc;

	while (1) {
		if (wait_for_completion_io_timeout(wait,
			PQI_WAIT_FOR_COMPLETION_IO_TIMEOUT_SECS * HZ)) {
			rc = 0;
			break;
		}

		pqi_check_ctrl_health(ctrl_info);
		if (pqi_ctrl_offline(ctrl_info)) {
			rc = -ENXIO;
			break;
		}
	}

	return rc;
}

static void pqi_raid_synchronous_complete(struct pqi_io_request *io_request,
	void *context)
{
	struct completion *waiting = context;

	complete(waiting);
}

static int pqi_submit_raid_request_synchronous_with_io_request(
	struct pqi_ctrl_info *ctrl_info, struct pqi_io_request *io_request,
	unsigned long timeout_msecs)
{
	int rc = 0;
	DECLARE_COMPLETION_ONSTACK(wait);

	io_request->io_complete_callback = pqi_raid_synchronous_complete;
	io_request->context = &wait;

	pqi_start_io(ctrl_info,
		&ctrl_info->queue_groups[PQI_DEFAULT_QUEUE_GROUP], RAID_PATH,
		io_request);

	if (timeout_msecs == NO_TIMEOUT) {
		pqi_wait_for_completion_io(ctrl_info, &wait);
	} else {
		if (!wait_for_completion_io_timeout(&wait,
			msecs_to_jiffies(timeout_msecs))) {
			dev_warn(&ctrl_info->pci_dev->dev,
				"command timed out\n");
			rc = -ETIMEDOUT;
		}
	}

	return rc;
}

static int pqi_submit_raid_request_synchronous(struct pqi_ctrl_info *ctrl_info,
	struct pqi_iu_header *request, unsigned int flags,
	struct pqi_raid_error_info *error_info, unsigned long timeout_msecs)
{
	int rc;
	struct pqi_io_request *io_request;
	unsigned long start_jiffies;
	unsigned long msecs_blocked;
	size_t iu_length;

	/*
	 * Note that specifying PQI_SYNC_FLAGS_INTERRUPTABLE and a timeout value
	 * are mutually exclusive.
	 */

	if (flags & PQI_SYNC_FLAGS_INTERRUPTABLE) {
		if (down_interruptible(&ctrl_info->sync_request_sem))
			return -ERESTARTSYS;
	} else {
		if (timeout_msecs == NO_TIMEOUT) {
			down(&ctrl_info->sync_request_sem);
		} else {
			start_jiffies = jiffies;
			if (down_timeout(&ctrl_info->sync_request_sem,
				msecs_to_jiffies(timeout_msecs)))
				return -ETIMEDOUT;
			msecs_blocked =
				jiffies_to_msecs(jiffies - start_jiffies);
			if (msecs_blocked >= timeout_msecs)
				return -ETIMEDOUT;
			timeout_msecs -= msecs_blocked;
		}
	}

	pqi_ctrl_busy(ctrl_info);
	timeout_msecs = pqi_wait_if_ctrl_blocked(ctrl_info, timeout_msecs);
	if (timeout_msecs == 0) {
		rc = -ETIMEDOUT;
		goto out;
	}

	if (pqi_ctrl_offline(ctrl_info)) {
		rc = -ENXIO;
		goto out;
	}

	io_request = pqi_alloc_io_request(ctrl_info);

	put_unaligned_le16(io_request->index,
		&(((struct pqi_raid_path_request *)request)->request_id));

	if (request->iu_type == PQI_REQUEST_IU_RAID_PATH_IO)
		((struct pqi_raid_path_request *)request)->error_index =
			((struct pqi_raid_path_request *)request)->request_id;

	iu_length = get_unaligned_le16(&request->iu_length) +
		PQI_REQUEST_HEADER_LENGTH;
	memcpy(io_request->iu, request, iu_length);

	rc = pqi_submit_raid_request_synchronous_with_io_request(ctrl_info,
		io_request, timeout_msecs);

	if (error_info) {
		if (io_request->error_info)
			memcpy(error_info, io_request->error_info,
				sizeof(*error_info));
		else
			memset(error_info, 0, sizeof(*error_info));
	} else if (rc == 0 && io_request->error_info) {
		u8 scsi_status;
		struct pqi_raid_error_info *raid_error_info;

		raid_error_info = io_request->error_info;
		scsi_status = raid_error_info->status;

		if (scsi_status == SAM_STAT_CHECK_CONDITION &&
			raid_error_info->data_out_result ==
			PQI_DATA_IN_OUT_UNDERFLOW)
			scsi_status = SAM_STAT_GOOD;

		if (scsi_status != SAM_STAT_GOOD)
			rc = -EIO;
	}

	pqi_free_io_request(io_request);

out:
	pqi_ctrl_unbusy(ctrl_info);
	up(&ctrl_info->sync_request_sem);

	return rc;
}

static int pqi_validate_admin_response(
	struct pqi_general_admin_response *response, u8 expected_function_code)
{
	if (response->header.iu_type != PQI_RESPONSE_IU_GENERAL_ADMIN)
		return -EINVAL;

	if (get_unaligned_le16(&response->header.iu_length) !=
		PQI_GENERAL_ADMIN_IU_LENGTH)
		return -EINVAL;

	if (response->function_code != expected_function_code)
		return -EINVAL;

	if (response->status != PQI_GENERAL_ADMIN_STATUS_SUCCESS)
		return -EINVAL;

	return 0;
}

static int pqi_submit_admin_request_synchronous(
	struct pqi_ctrl_info *ctrl_info,
	struct pqi_general_admin_request *request,
	struct pqi_general_admin_response *response)
{
	int rc;

	pqi_submit_admin_request(ctrl_info, request);

	rc = pqi_poll_for_admin_response(ctrl_info, response);

	if (rc == 0)
		rc = pqi_validate_admin_response(response,
			request->function_code);

	return rc;
}

static int pqi_report_device_capability(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct pqi_general_admin_request request;
	struct pqi_general_admin_response response;
	struct pqi_device_capability *capability;
	struct pqi_iu_layer_descriptor *sop_iu_layer_descriptor;

	capability = kmalloc(sizeof(*capability), GFP_KERNEL);
	if (!capability)
		return -ENOMEM;

	memset(&request, 0, sizeof(request));

	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code =
		PQI_GENERAL_ADMIN_FUNCTION_REPORT_DEVICE_CAPABILITY;
	put_unaligned_le32(sizeof(*capability),
		&request.data.report_device_capability.buffer_length);

	rc = pqi_map_single(ctrl_info->pci_dev,
		&request.data.report_device_capability.sg_descriptor,
		capability, sizeof(*capability),
		PCI_DMA_FROMDEVICE);
	if (rc)
		goto out;

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);

	pqi_pci_unmap(ctrl_info->pci_dev,
		&request.data.report_device_capability.sg_descriptor, 1,
		PCI_DMA_FROMDEVICE);

	if (rc)
		goto out;

	if (response.status != PQI_GENERAL_ADMIN_STATUS_SUCCESS) {
		rc = -EIO;
		goto out;
	}

	ctrl_info->max_inbound_queues =
		get_unaligned_le16(&capability->max_inbound_queues);
	ctrl_info->max_elements_per_iq =
		get_unaligned_le16(&capability->max_elements_per_iq);
	ctrl_info->max_iq_element_length =
		get_unaligned_le16(&capability->max_iq_element_length)
		* 16;
	ctrl_info->max_outbound_queues =
		get_unaligned_le16(&capability->max_outbound_queues);
	ctrl_info->max_elements_per_oq =
		get_unaligned_le16(&capability->max_elements_per_oq);
	ctrl_info->max_oq_element_length =
		get_unaligned_le16(&capability->max_oq_element_length)
		* 16;

	sop_iu_layer_descriptor =
		&capability->iu_layer_descriptors[PQI_PROTOCOL_SOP];

	ctrl_info->max_inbound_iu_length_per_firmware =
		get_unaligned_le16(
			&sop_iu_layer_descriptor->max_inbound_iu_length);
	ctrl_info->inbound_spanning_supported =
		sop_iu_layer_descriptor->inbound_spanning_supported;
	ctrl_info->outbound_spanning_supported =
		sop_iu_layer_descriptor->outbound_spanning_supported;

out:
	kfree(capability);

	return rc;
}

static int pqi_validate_device_capability(struct pqi_ctrl_info *ctrl_info)
{
	if (ctrl_info->max_iq_element_length <
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH) {
		dev_err(&ctrl_info->pci_dev->dev,
			"max. inbound queue element length of %d is less than the required length of %d\n",
			ctrl_info->max_iq_element_length,
			PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);
		return -EINVAL;
	}

	if (ctrl_info->max_oq_element_length <
		PQI_OPERATIONAL_OQ_ELEMENT_LENGTH) {
		dev_err(&ctrl_info->pci_dev->dev,
			"max. outbound queue element length of %d is less than the required length of %d\n",
			ctrl_info->max_oq_element_length,
			PQI_OPERATIONAL_OQ_ELEMENT_LENGTH);
		return -EINVAL;
	}

	if (ctrl_info->max_inbound_iu_length_per_firmware <
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH) {
		dev_err(&ctrl_info->pci_dev->dev,
			"max. inbound IU length of %u is less than the min. required length of %d\n",
			ctrl_info->max_inbound_iu_length_per_firmware,
			PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);
		return -EINVAL;
	}

	if (!ctrl_info->inbound_spanning_supported) {
		dev_err(&ctrl_info->pci_dev->dev,
			"the controller does not support inbound spanning\n");
		return -EINVAL;
	}

	if (ctrl_info->outbound_spanning_supported) {
		dev_err(&ctrl_info->pci_dev->dev,
			"the controller supports outbound spanning but this driver does not\n");
		return -EINVAL;
	}

	return 0;
}

static int pqi_delete_operational_queue(struct pqi_ctrl_info *ctrl_info,
	bool inbound_queue, u16 queue_id)
{
	struct pqi_general_admin_request request;
	struct pqi_general_admin_response response;

	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	if (inbound_queue)
		request.function_code =
			PQI_GENERAL_ADMIN_FUNCTION_DELETE_IQ;
	else
		request.function_code =
			PQI_GENERAL_ADMIN_FUNCTION_DELETE_OQ;
	put_unaligned_le16(queue_id,
		&request.data.delete_operational_queue.queue_id);

	return pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
}

static int pqi_create_event_queue(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct pqi_event_queue *event_queue;
	struct pqi_general_admin_request request;
	struct pqi_general_admin_response response;

	event_queue = &ctrl_info->event_queue;

	/*
	 * Create OQ (Outbound Queue - device to host queue) to dedicate
	 * to events.
	 */
	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code = PQI_GENERAL_ADMIN_FUNCTION_CREATE_OQ;
	put_unaligned_le16(event_queue->oq_id,
		&request.data.create_operational_oq.queue_id);
	put_unaligned_le64((u64)event_queue->oq_element_array_bus_addr,
		&request.data.create_operational_oq.element_array_addr);
	put_unaligned_le64((u64)event_queue->oq_pi_bus_addr,
		&request.data.create_operational_oq.pi_addr);
	put_unaligned_le16(PQI_NUM_EVENT_QUEUE_ELEMENTS,
		&request.data.create_operational_oq.num_elements);
	put_unaligned_le16(PQI_EVENT_OQ_ELEMENT_LENGTH / 16,
		&request.data.create_operational_oq.element_length);
	request.data.create_operational_oq.queue_protocol = PQI_PROTOCOL_SOP;
	put_unaligned_le16(event_queue->int_msg_num,
		&request.data.create_operational_oq.int_msg_num);

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
	if (rc)
		return rc;

	event_queue->oq_ci = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		get_unaligned_le64(
			&response.data.create_operational_oq.oq_ci_offset);

	return 0;
}

static int pqi_create_queue_group(struct pqi_ctrl_info *ctrl_info,
	unsigned int group_number)
{
	int rc;
	struct pqi_queue_group *queue_group;
	struct pqi_general_admin_request request;
	struct pqi_general_admin_response response;

	queue_group = &ctrl_info->queue_groups[group_number];

	/*
	 * Create IQ (Inbound Queue - host to device queue) for
	 * RAID path.
	 */
	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code = PQI_GENERAL_ADMIN_FUNCTION_CREATE_IQ;
	put_unaligned_le16(queue_group->iq_id[RAID_PATH],
		&request.data.create_operational_iq.queue_id);
	put_unaligned_le64(
		(u64)queue_group->iq_element_array_bus_addr[RAID_PATH],
		&request.data.create_operational_iq.element_array_addr);
	put_unaligned_le64((u64)queue_group->iq_ci_bus_addr[RAID_PATH],
		&request.data.create_operational_iq.ci_addr);
	put_unaligned_le16(ctrl_info->num_elements_per_iq,
		&request.data.create_operational_iq.num_elements);
	put_unaligned_le16(PQI_OPERATIONAL_IQ_ELEMENT_LENGTH / 16,
		&request.data.create_operational_iq.element_length);
	request.data.create_operational_iq.queue_protocol = PQI_PROTOCOL_SOP;

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating inbound RAID queue\n");
		return rc;
	}

	queue_group->iq_pi[RAID_PATH] = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		get_unaligned_le64(
			&response.data.create_operational_iq.iq_pi_offset);

	/*
	 * Create IQ (Inbound Queue - host to device queue) for
	 * Advanced I/O (AIO) path.
	 */
	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code = PQI_GENERAL_ADMIN_FUNCTION_CREATE_IQ;
	put_unaligned_le16(queue_group->iq_id[AIO_PATH],
		&request.data.create_operational_iq.queue_id);
	put_unaligned_le64((u64)queue_group->
		iq_element_array_bus_addr[AIO_PATH],
		&request.data.create_operational_iq.element_array_addr);
	put_unaligned_le64((u64)queue_group->iq_ci_bus_addr[AIO_PATH],
		&request.data.create_operational_iq.ci_addr);
	put_unaligned_le16(ctrl_info->num_elements_per_iq,
		&request.data.create_operational_iq.num_elements);
	put_unaligned_le16(PQI_OPERATIONAL_IQ_ELEMENT_LENGTH / 16,
		&request.data.create_operational_iq.element_length);
	request.data.create_operational_iq.queue_protocol = PQI_PROTOCOL_SOP;

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating inbound AIO queue\n");
		goto delete_inbound_queue_raid;
	}

	queue_group->iq_pi[AIO_PATH] = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		get_unaligned_le64(
			&response.data.create_operational_iq.iq_pi_offset);

	/*
	 * Designate the 2nd IQ as the AIO path.  By default, all IQs are
	 * assumed to be for RAID path I/O unless we change the queue's
	 * property.
	 */
	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code = PQI_GENERAL_ADMIN_FUNCTION_CHANGE_IQ_PROPERTY;
	put_unaligned_le16(queue_group->iq_id[AIO_PATH],
		&request.data.change_operational_iq_properties.queue_id);
	put_unaligned_le32(PQI_IQ_PROPERTY_IS_AIO_QUEUE,
		&request.data.change_operational_iq_properties.vendor_specific);

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error changing queue property\n");
		goto delete_inbound_queue_aio;
	}

	/*
	 * Create OQ (Outbound Queue - device to host queue).
	 */
	memset(&request, 0, sizeof(request));
	request.header.iu_type = PQI_REQUEST_IU_GENERAL_ADMIN;
	put_unaligned_le16(PQI_GENERAL_ADMIN_IU_LENGTH,
		&request.header.iu_length);
	request.function_code = PQI_GENERAL_ADMIN_FUNCTION_CREATE_OQ;
	put_unaligned_le16(queue_group->oq_id,
		&request.data.create_operational_oq.queue_id);
	put_unaligned_le64((u64)queue_group->oq_element_array_bus_addr,
		&request.data.create_operational_oq.element_array_addr);
	put_unaligned_le64((u64)queue_group->oq_pi_bus_addr,
		&request.data.create_operational_oq.pi_addr);
	put_unaligned_le16(ctrl_info->num_elements_per_oq,
		&request.data.create_operational_oq.num_elements);
	put_unaligned_le16(PQI_OPERATIONAL_OQ_ELEMENT_LENGTH / 16,
		&request.data.create_operational_oq.element_length);
	request.data.create_operational_oq.queue_protocol = PQI_PROTOCOL_SOP;
	put_unaligned_le16(queue_group->int_msg_num,
		&request.data.create_operational_oq.int_msg_num);

	rc = pqi_submit_admin_request_synchronous(ctrl_info, &request,
		&response);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating outbound queue\n");
		goto delete_inbound_queue_aio;
	}

	queue_group->oq_ci = ctrl_info->iomem_base +
		PQI_DEVICE_REGISTERS_OFFSET +
		get_unaligned_le64(
			&response.data.create_operational_oq.oq_ci_offset);

	return 0;

delete_inbound_queue_aio:
	pqi_delete_operational_queue(ctrl_info, true,
		queue_group->iq_id[AIO_PATH]);

delete_inbound_queue_raid:
	pqi_delete_operational_queue(ctrl_info, true,
		queue_group->iq_id[RAID_PATH]);

	return rc;
}

static int pqi_create_queues(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	unsigned int i;

	rc = pqi_create_event_queue(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating event queue\n");
		return rc;
	}

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		rc = pqi_create_queue_group(ctrl_info, i);
		if (rc) {
			dev_err(&ctrl_info->pci_dev->dev,
				"error creating queue group number %u/%u\n",
				i, ctrl_info->num_queue_groups);
			return rc;
		}
	}

	return 0;
}

#define PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH	\
	(offsetof(struct pqi_event_config, descriptors) + \
	(PQI_MAX_EVENT_DESCRIPTORS * sizeof(struct pqi_event_descriptor)))

static int pqi_configure_events(struct pqi_ctrl_info *ctrl_info,
	bool enable_events)
{
	int rc;
	unsigned int i;
	struct pqi_event_config *event_config;
	struct pqi_event_descriptor *event_descriptor;
	struct pqi_general_management_request request;

	event_config = kmalloc(PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH,
		GFP_KERNEL);
	if (!event_config)
		return -ENOMEM;

	memset(&request, 0, sizeof(request));

	request.header.iu_type = PQI_REQUEST_IU_REPORT_VENDOR_EVENT_CONFIG;
	put_unaligned_le16(offsetof(struct pqi_general_management_request,
		data.report_event_configuration.sg_descriptors[1]) -
		PQI_REQUEST_HEADER_LENGTH, &request.header.iu_length);
	put_unaligned_le32(PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH,
		&request.data.report_event_configuration.buffer_length);

	rc = pqi_map_single(ctrl_info->pci_dev,
		request.data.report_event_configuration.sg_descriptors,
		event_config, PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH,
		PCI_DMA_FROMDEVICE);
	if (rc)
		goto out;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header,
		0, NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev,
		request.data.report_event_configuration.sg_descriptors, 1,
		PCI_DMA_FROMDEVICE);

	if (rc)
		goto out;

	for (i = 0; i < event_config->num_event_descriptors; i++) {
		event_descriptor = &event_config->descriptors[i];
		if (enable_events &&
			pqi_is_supported_event(event_descriptor->event_type))
			put_unaligned_le16(ctrl_info->event_queue.oq_id,
					&event_descriptor->oq_id);
		else
			put_unaligned_le16(0, &event_descriptor->oq_id);
	}

	memset(&request, 0, sizeof(request));

	request.header.iu_type = PQI_REQUEST_IU_SET_VENDOR_EVENT_CONFIG;
	put_unaligned_le16(offsetof(struct pqi_general_management_request,
		data.report_event_configuration.sg_descriptors[1]) -
		PQI_REQUEST_HEADER_LENGTH, &request.header.iu_length);
	put_unaligned_le32(PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH,
		&request.data.report_event_configuration.buffer_length);

	rc = pqi_map_single(ctrl_info->pci_dev,
		request.data.report_event_configuration.sg_descriptors,
		event_config, PQI_REPORT_EVENT_CONFIG_BUFFER_LENGTH,
		PCI_DMA_TODEVICE);
	if (rc)
		goto out;

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header, 0,
		NULL, NO_TIMEOUT);

	pqi_pci_unmap(ctrl_info->pci_dev,
		request.data.report_event_configuration.sg_descriptors, 1,
		PCI_DMA_TODEVICE);

out:
	kfree(event_config);

	return rc;
}

static inline int pqi_enable_events(struct pqi_ctrl_info *ctrl_info)
{
	return pqi_configure_events(ctrl_info, true);
}

static inline int pqi_disable_events(struct pqi_ctrl_info *ctrl_info)
{
	return pqi_configure_events(ctrl_info, false);
}

static void pqi_free_all_io_requests(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	struct device *dev;
	size_t sg_chain_buffer_length;
	struct pqi_io_request *io_request;

	if (!ctrl_info->io_request_pool)
		return;

	dev = &ctrl_info->pci_dev->dev;
	sg_chain_buffer_length = ctrl_info->sg_chain_buffer_length;
	io_request = ctrl_info->io_request_pool;

	for (i = 0; i < ctrl_info->max_io_slots; i++) {
		kfree(io_request->iu);
		if (!io_request->sg_chain_buffer)
			break;
		dma_free_coherent(dev, sg_chain_buffer_length,
			io_request->sg_chain_buffer,
			io_request->sg_chain_buffer_dma_handle);
		io_request++;
	}

	kfree(ctrl_info->io_request_pool);
	ctrl_info->io_request_pool = NULL;
}

static inline int pqi_alloc_error_buffer(struct pqi_ctrl_info *ctrl_info)
{
	ctrl_info->error_buffer = dma_zalloc_coherent(&ctrl_info->pci_dev->dev,
		ctrl_info->error_buffer_length,
		&ctrl_info->error_buffer_dma_handle, GFP_KERNEL);

	if (!ctrl_info->error_buffer)
		return -ENOMEM;

	return 0;
}

static int pqi_alloc_io_resources(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	void *sg_chain_buffer;
	size_t sg_chain_buffer_length;
	dma_addr_t sg_chain_buffer_dma_handle;
	struct device *dev;
	struct pqi_io_request *io_request;

	ctrl_info->io_request_pool = kzalloc(ctrl_info->max_io_slots *
		sizeof(ctrl_info->io_request_pool[0]), GFP_KERNEL);

	if (!ctrl_info->io_request_pool) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to allocate I/O request pool\n");
		goto error;
	}

	dev = &ctrl_info->pci_dev->dev;
	sg_chain_buffer_length = ctrl_info->sg_chain_buffer_length;
	io_request = ctrl_info->io_request_pool;

	for (i = 0; i < ctrl_info->max_io_slots; i++) {
		io_request->iu =
			kmalloc(ctrl_info->max_inbound_iu_length, GFP_KERNEL);

		if (!io_request->iu) {
			dev_err(&ctrl_info->pci_dev->dev,
				"failed to allocate IU buffers\n");
			goto error;
		}

		sg_chain_buffer = dma_alloc_coherent(dev,
			sg_chain_buffer_length, &sg_chain_buffer_dma_handle,
			GFP_KERNEL);

		if (!sg_chain_buffer) {
			dev_err(&ctrl_info->pci_dev->dev,
				"failed to allocate PQI scatter-gather chain buffers\n");
			goto error;
		}

		io_request->index = i;
		io_request->sg_chain_buffer = sg_chain_buffer;
		io_request->sg_chain_buffer_dma_handle =
			sg_chain_buffer_dma_handle;
		io_request++;
	}

	return 0;

error:
	pqi_free_all_io_requests(ctrl_info);

	return -ENOMEM;
}

/*
 * Calculate required resources that are sized based on max. outstanding
 * requests and max. transfer size.
 */

static void pqi_calculate_io_resources(struct pqi_ctrl_info *ctrl_info)
{
	u32 max_transfer_size;
	u32 max_sg_entries;

	ctrl_info->scsi_ml_can_queue =
		ctrl_info->max_outstanding_requests - PQI_RESERVED_IO_SLOTS;
	ctrl_info->max_io_slots = ctrl_info->max_outstanding_requests;

	ctrl_info->error_buffer_length =
		ctrl_info->max_io_slots * PQI_ERROR_BUFFER_ELEMENT_LENGTH;

	if (reset_devices)
		max_transfer_size = min(ctrl_info->max_transfer_size,
			PQI_MAX_TRANSFER_SIZE_KDUMP);
	else
		max_transfer_size = min(ctrl_info->max_transfer_size,
			PQI_MAX_TRANSFER_SIZE);

	max_sg_entries = max_transfer_size / PAGE_SIZE;

	/* +1 to cover when the buffer is not page-aligned. */
	max_sg_entries++;

	max_sg_entries = min(ctrl_info->max_sg_entries, max_sg_entries);

	max_transfer_size = (max_sg_entries - 1) * PAGE_SIZE;

	ctrl_info->sg_chain_buffer_length =
		(max_sg_entries * sizeof(struct pqi_sg_descriptor)) +
		PQI_EXTRA_SGL_MEMORY;
	ctrl_info->sg_tablesize = max_sg_entries;
	ctrl_info->max_sectors = max_transfer_size / 512;
}

static void pqi_calculate_queue_resources(struct pqi_ctrl_info *ctrl_info)
{
	int num_queue_groups;
	u16 num_elements_per_iq;
	u16 num_elements_per_oq;

	if (reset_devices) {
		num_queue_groups = 1;
	} else {
		int num_cpus;
		int max_queue_groups;

		max_queue_groups = min(ctrl_info->max_inbound_queues / 2,
			ctrl_info->max_outbound_queues - 1);
		max_queue_groups = min(max_queue_groups, PQI_MAX_QUEUE_GROUPS);

		num_cpus = num_online_cpus();
		num_queue_groups = min(num_cpus, ctrl_info->max_msix_vectors);
		num_queue_groups = min(num_queue_groups, max_queue_groups);
	}

	ctrl_info->num_queue_groups = num_queue_groups;
	ctrl_info->max_hw_queue_index = num_queue_groups - 1;

	/*
	 * Make sure that the max. inbound IU length is an even multiple
	 * of our inbound element length.
	 */
	ctrl_info->max_inbound_iu_length =
		(ctrl_info->max_inbound_iu_length_per_firmware /
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH) *
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH;

	num_elements_per_iq =
		(ctrl_info->max_inbound_iu_length /
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

	/* Add one because one element in each queue is unusable. */
	num_elements_per_iq++;

	num_elements_per_iq = min(num_elements_per_iq,
		ctrl_info->max_elements_per_iq);

	num_elements_per_oq = ((num_elements_per_iq - 1) * 2) + 1;
	num_elements_per_oq = min(num_elements_per_oq,
		ctrl_info->max_elements_per_oq);

	ctrl_info->num_elements_per_iq = num_elements_per_iq;
	ctrl_info->num_elements_per_oq = num_elements_per_oq;

	ctrl_info->max_sg_per_iu =
		((ctrl_info->max_inbound_iu_length -
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH) /
		sizeof(struct pqi_sg_descriptor)) +
		PQI_MAX_EMBEDDED_SG_DESCRIPTORS;
}

static inline void pqi_set_sg_descriptor(
	struct pqi_sg_descriptor *sg_descriptor, struct scatterlist *sg)
{
	u64 address = (u64)sg_dma_address(sg);
	unsigned int length = sg_dma_len(sg);

	put_unaligned_le64(address, &sg_descriptor->address);
	put_unaligned_le32(length, &sg_descriptor->length);
	put_unaligned_le32(0, &sg_descriptor->flags);
}

static int pqi_build_raid_sg_list(struct pqi_ctrl_info *ctrl_info,
	struct pqi_raid_path_request *request, struct scsi_cmnd *scmd,
	struct pqi_io_request *io_request)
{
	int i;
	u16 iu_length;
	int sg_count;
	bool chained;
	unsigned int num_sg_in_iu;
	unsigned int max_sg_per_iu;
	struct scatterlist *sg;
	struct pqi_sg_descriptor *sg_descriptor;

	sg_count = scsi_dma_map(scmd);
	if (sg_count < 0)
		return sg_count;

	iu_length = offsetof(struct pqi_raid_path_request, sg_descriptors) -
		PQI_REQUEST_HEADER_LENGTH;

	if (sg_count == 0)
		goto out;

	sg = scsi_sglist(scmd);
	sg_descriptor = request->sg_descriptors;
	max_sg_per_iu = ctrl_info->max_sg_per_iu - 1;
	chained = false;
	num_sg_in_iu = 0;
	i = 0;

	while (1) {
		pqi_set_sg_descriptor(sg_descriptor, sg);
		if (!chained)
			num_sg_in_iu++;
		i++;
		if (i == sg_count)
			break;
		sg_descriptor++;
		if (i == max_sg_per_iu) {
			put_unaligned_le64(
				(u64)io_request->sg_chain_buffer_dma_handle,
				&sg_descriptor->address);
			put_unaligned_le32((sg_count - num_sg_in_iu)
				* sizeof(*sg_descriptor),
				&sg_descriptor->length);
			put_unaligned_le32(CISS_SG_CHAIN,
				&sg_descriptor->flags);
			chained = true;
			num_sg_in_iu++;
			sg_descriptor = io_request->sg_chain_buffer;
		}
		sg = sg_next(sg);
	}

	put_unaligned_le32(CISS_SG_LAST, &sg_descriptor->flags);
	request->partial = chained;
	iu_length += num_sg_in_iu * sizeof(*sg_descriptor);

out:
	put_unaligned_le16(iu_length, &request->header.iu_length);

	return 0;
}

static int pqi_build_aio_sg_list(struct pqi_ctrl_info *ctrl_info,
	struct pqi_aio_path_request *request, struct scsi_cmnd *scmd,
	struct pqi_io_request *io_request)
{
	int i;
	u16 iu_length;
	int sg_count;
	bool chained;
	unsigned int num_sg_in_iu;
	unsigned int max_sg_per_iu;
	struct scatterlist *sg;
	struct pqi_sg_descriptor *sg_descriptor;

	sg_count = scsi_dma_map(scmd);
	if (sg_count < 0)
		return sg_count;

	iu_length = offsetof(struct pqi_aio_path_request, sg_descriptors) -
		PQI_REQUEST_HEADER_LENGTH;
	num_sg_in_iu = 0;

	if (sg_count == 0)
		goto out;

	sg = scsi_sglist(scmd);
	sg_descriptor = request->sg_descriptors;
	max_sg_per_iu = ctrl_info->max_sg_per_iu - 1;
	chained = false;
	i = 0;

	while (1) {
		pqi_set_sg_descriptor(sg_descriptor, sg);
		if (!chained)
			num_sg_in_iu++;
		i++;
		if (i == sg_count)
			break;
		sg_descriptor++;
		if (i == max_sg_per_iu) {
			put_unaligned_le64(
				(u64)io_request->sg_chain_buffer_dma_handle,
				&sg_descriptor->address);
			put_unaligned_le32((sg_count - num_sg_in_iu)
				* sizeof(*sg_descriptor),
				&sg_descriptor->length);
			put_unaligned_le32(CISS_SG_CHAIN,
				&sg_descriptor->flags);
			chained = true;
			num_sg_in_iu++;
			sg_descriptor = io_request->sg_chain_buffer;
		}
		sg = sg_next(sg);
	}

	put_unaligned_le32(CISS_SG_LAST, &sg_descriptor->flags);
	request->partial = chained;
	iu_length += num_sg_in_iu * sizeof(*sg_descriptor);

out:
	put_unaligned_le16(iu_length, &request->header.iu_length);
	request->num_sg_descriptors = num_sg_in_iu;

	return 0;
}

static void pqi_raid_io_complete(struct pqi_io_request *io_request,
	void *context)
{
	struct scsi_cmnd *scmd;

	scmd = io_request->scmd;
	pqi_free_io_request(io_request);
	scsi_dma_unmap(scmd);
	pqi_scsi_done(scmd);
}

static int pqi_raid_submit_scsi_cmd_with_io_request(
	struct pqi_ctrl_info *ctrl_info, struct pqi_io_request *io_request,
	struct pqi_scsi_dev *device, struct scsi_cmnd *scmd,
	struct pqi_queue_group *queue_group)
{
	int rc;
	size_t cdb_length;
	struct pqi_raid_path_request *request;

	io_request->io_complete_callback = pqi_raid_io_complete;
	io_request->scmd = scmd;

	request = io_request->iu;
	memset(request, 0,
		offsetof(struct pqi_raid_path_request, sg_descriptors));

	request->header.iu_type = PQI_REQUEST_IU_RAID_PATH_IO;
	put_unaligned_le32(scsi_bufflen(scmd), &request->buffer_length);
	request->task_attribute = SOP_TASK_ATTRIBUTE_SIMPLE;
	put_unaligned_le16(io_request->index, &request->request_id);
	request->error_index = request->request_id;
	memcpy(request->lun_number, device->scsi3addr,
		sizeof(request->lun_number));

	cdb_length = min_t(size_t, scmd->cmd_len, sizeof(request->cdb));
	memcpy(request->cdb, scmd->cmnd, cdb_length);

	switch (cdb_length) {
	case 6:
	case 10:
	case 12:
	case 16:
		/* No bytes in the Additional CDB bytes field */
		request->additional_cdb_bytes_usage =
			SOP_ADDITIONAL_CDB_BYTES_0;
		break;
	case 20:
		/* 4 bytes in the Additional cdb field */
		request->additional_cdb_bytes_usage =
			SOP_ADDITIONAL_CDB_BYTES_4;
		break;
	case 24:
		/* 8 bytes in the Additional cdb field */
		request->additional_cdb_bytes_usage =
			SOP_ADDITIONAL_CDB_BYTES_8;
		break;
	case 28:
		/* 12 bytes in the Additional cdb field */
		request->additional_cdb_bytes_usage =
			SOP_ADDITIONAL_CDB_BYTES_12;
		break;
	case 32:
	default:
		/* 16 bytes in the Additional cdb field */
		request->additional_cdb_bytes_usage =
			SOP_ADDITIONAL_CDB_BYTES_16;
		break;
	}

	switch (scmd->sc_data_direction) {
	case DMA_TO_DEVICE:
		request->data_direction = SOP_READ_FLAG;
		break;
	case DMA_FROM_DEVICE:
		request->data_direction = SOP_WRITE_FLAG;
		break;
	case DMA_NONE:
		request->data_direction = SOP_NO_DIRECTION_FLAG;
		break;
	case DMA_BIDIRECTIONAL:
		request->data_direction = SOP_BIDIRECTIONAL;
		break;
	default:
		dev_err(&ctrl_info->pci_dev->dev,
			"unknown data direction: %d\n",
			scmd->sc_data_direction);
		break;
	}

	rc = pqi_build_raid_sg_list(ctrl_info, request, scmd, io_request);
	if (rc) {
		pqi_free_io_request(io_request);
		return SCSI_MLQUEUE_HOST_BUSY;
	}

	pqi_start_io(ctrl_info, queue_group, RAID_PATH, io_request);

	return 0;
}

static inline int pqi_raid_submit_scsi_cmd(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device, struct scsi_cmnd *scmd,
	struct pqi_queue_group *queue_group)
{
	struct pqi_io_request *io_request;

	io_request = pqi_alloc_io_request(ctrl_info);

	return pqi_raid_submit_scsi_cmd_with_io_request(ctrl_info, io_request,
		device, scmd, queue_group);
}

static inline void pqi_schedule_bypass_retry(struct pqi_ctrl_info *ctrl_info)
{
	if (!pqi_ctrl_blocked(ctrl_info))
		schedule_work(&ctrl_info->raid_bypass_retry_work);
}

static bool pqi_raid_bypass_retry_needed(struct pqi_io_request *io_request)
{
	struct scsi_cmnd *scmd;
	struct pqi_scsi_dev *device;
	struct pqi_ctrl_info *ctrl_info;

	if (!io_request->raid_bypass)
		return false;

	scmd = io_request->scmd;
	if ((scmd->result & 0xff) == SAM_STAT_GOOD)
		return false;
	if (host_byte(scmd->result) == DID_NO_CONNECT)
		return false;

	device = scmd->device->hostdata;
	if (pqi_device_offline(device))
		return false;

	ctrl_info = shost_to_hba(scmd->device->host);
	if (pqi_ctrl_offline(ctrl_info))
		return false;

	return true;
}

static inline void pqi_add_to_raid_bypass_retry_list(
	struct pqi_ctrl_info *ctrl_info,
	struct pqi_io_request *io_request, bool at_head)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl_info->raid_bypass_retry_list_lock, flags);
	if (at_head)
		list_add(&io_request->request_list_entry,
			&ctrl_info->raid_bypass_retry_list);
	else
		list_add_tail(&io_request->request_list_entry,
			&ctrl_info->raid_bypass_retry_list);
	spin_unlock_irqrestore(&ctrl_info->raid_bypass_retry_list_lock, flags);
}

static void pqi_queued_raid_bypass_complete(struct pqi_io_request *io_request,
	void *context)
{
	struct scsi_cmnd *scmd;

	scmd = io_request->scmd;
	pqi_free_io_request(io_request);
	pqi_scsi_done(scmd);
}

static void pqi_queue_raid_bypass_retry(struct pqi_io_request *io_request)
{
	struct scsi_cmnd *scmd;
	struct pqi_ctrl_info *ctrl_info;

	io_request->io_complete_callback = pqi_queued_raid_bypass_complete;
	scmd = io_request->scmd;
	scmd->result = 0;
	ctrl_info = shost_to_hba(scmd->device->host);

	pqi_add_to_raid_bypass_retry_list(ctrl_info, io_request, false);
	pqi_schedule_bypass_retry(ctrl_info);
}

static int pqi_retry_raid_bypass(struct pqi_io_request *io_request)
{
	struct scsi_cmnd *scmd;
	struct pqi_scsi_dev *device;
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_queue_group *queue_group;

	scmd = io_request->scmd;
	device = scmd->device->hostdata;
	if (pqi_device_in_reset(device)) {
		pqi_free_io_request(io_request);
		set_host_byte(scmd, DID_RESET);
		pqi_scsi_done(scmd);
		return 0;
	}

	ctrl_info = shost_to_hba(scmd->device->host);
	queue_group = io_request->queue_group;

	pqi_reinit_io_request(io_request);

	return pqi_raid_submit_scsi_cmd_with_io_request(ctrl_info, io_request,
		device, scmd, queue_group);
}

static inline struct pqi_io_request *pqi_next_queued_raid_bypass_request(
	struct pqi_ctrl_info *ctrl_info)
{
	unsigned long flags;
	struct pqi_io_request *io_request;

	spin_lock_irqsave(&ctrl_info->raid_bypass_retry_list_lock, flags);
	io_request = list_first_entry_or_null(
		&ctrl_info->raid_bypass_retry_list,
		struct pqi_io_request, request_list_entry);
	if (io_request)
		list_del(&io_request->request_list_entry);
	spin_unlock_irqrestore(&ctrl_info->raid_bypass_retry_list_lock, flags);

	return io_request;
}

static void pqi_retry_raid_bypass_requests(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct pqi_io_request *io_request;

	pqi_ctrl_busy(ctrl_info);

	while (1) {
		if (pqi_ctrl_blocked(ctrl_info))
			break;
		io_request = pqi_next_queued_raid_bypass_request(ctrl_info);
		if (!io_request)
			break;
		rc = pqi_retry_raid_bypass(io_request);
		if (rc) {
			pqi_add_to_raid_bypass_retry_list(ctrl_info, io_request,
				true);
			pqi_schedule_bypass_retry(ctrl_info);
			break;
		}
	}

	pqi_ctrl_unbusy(ctrl_info);
}

static void pqi_raid_bypass_retry_worker(struct work_struct *work)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = container_of(work, struct pqi_ctrl_info,
		raid_bypass_retry_work);
	pqi_retry_raid_bypass_requests(ctrl_info);
}

static void pqi_clear_all_queued_raid_bypass_retries(
	struct pqi_ctrl_info *ctrl_info)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl_info->raid_bypass_retry_list_lock, flags);
	INIT_LIST_HEAD(&ctrl_info->raid_bypass_retry_list);
	spin_unlock_irqrestore(&ctrl_info->raid_bypass_retry_list_lock, flags);
}

static void pqi_aio_io_complete(struct pqi_io_request *io_request,
	void *context)
{
	struct scsi_cmnd *scmd;

	scmd = io_request->scmd;
	scsi_dma_unmap(scmd);
	if (io_request->status == -EAGAIN)
		set_host_byte(scmd, DID_IMM_RETRY);
	else if (pqi_raid_bypass_retry_needed(io_request)) {
		pqi_queue_raid_bypass_retry(io_request);
		return;
	}
	pqi_free_io_request(io_request);
	pqi_scsi_done(scmd);
}

static inline int pqi_aio_submit_scsi_cmd(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device, struct scsi_cmnd *scmd,
	struct pqi_queue_group *queue_group)
{
	return pqi_aio_submit_io(ctrl_info, scmd, device->aio_handle,
		scmd->cmnd, scmd->cmd_len, queue_group, NULL, false);
}

static int pqi_aio_submit_io(struct pqi_ctrl_info *ctrl_info,
	struct scsi_cmnd *scmd, u32 aio_handle, u8 *cdb,
	unsigned int cdb_length, struct pqi_queue_group *queue_group,
	struct pqi_encryption_info *encryption_info, bool raid_bypass)
{
	int rc;
	struct pqi_io_request *io_request;
	struct pqi_aio_path_request *request;

	io_request = pqi_alloc_io_request(ctrl_info);
	io_request->io_complete_callback = pqi_aio_io_complete;
	io_request->scmd = scmd;
	io_request->raid_bypass = raid_bypass;

	request = io_request->iu;
	memset(request, 0,
		offsetof(struct pqi_raid_path_request, sg_descriptors));

	request->header.iu_type = PQI_REQUEST_IU_AIO_PATH_IO;
	put_unaligned_le32(aio_handle, &request->nexus_id);
	put_unaligned_le32(scsi_bufflen(scmd), &request->buffer_length);
	request->task_attribute = SOP_TASK_ATTRIBUTE_SIMPLE;
	put_unaligned_le16(io_request->index, &request->request_id);
	request->error_index = request->request_id;
	if (cdb_length > sizeof(request->cdb))
		cdb_length = sizeof(request->cdb);
	request->cdb_length = cdb_length;
	memcpy(request->cdb, cdb, cdb_length);

	switch (scmd->sc_data_direction) {
	case DMA_TO_DEVICE:
		request->data_direction = SOP_READ_FLAG;
		break;
	case DMA_FROM_DEVICE:
		request->data_direction = SOP_WRITE_FLAG;
		break;
	case DMA_NONE:
		request->data_direction = SOP_NO_DIRECTION_FLAG;
		break;
	case DMA_BIDIRECTIONAL:
		request->data_direction = SOP_BIDIRECTIONAL;
		break;
	default:
		dev_err(&ctrl_info->pci_dev->dev,
			"unknown data direction: %d\n",
			scmd->sc_data_direction);
		break;
	}

	if (encryption_info) {
		request->encryption_enable = true;
		put_unaligned_le16(encryption_info->data_encryption_key_index,
			&request->data_encryption_key_index);
		put_unaligned_le32(encryption_info->encrypt_tweak_lower,
			&request->encrypt_tweak_lower);
		put_unaligned_le32(encryption_info->encrypt_tweak_upper,
			&request->encrypt_tweak_upper);
	}

	rc = pqi_build_aio_sg_list(ctrl_info, request, scmd, io_request);
	if (rc) {
		pqi_free_io_request(io_request);
		return SCSI_MLQUEUE_HOST_BUSY;
	}

	pqi_start_io(ctrl_info, queue_group, AIO_PATH, io_request);

	return 0;
}

static inline u16 pqi_get_hw_queue(struct pqi_ctrl_info *ctrl_info,
	struct scsi_cmnd *scmd)
{
	u16 hw_queue;

	hw_queue = blk_mq_unique_tag_to_hwq(blk_mq_unique_tag(scmd->request));
	if (hw_queue > ctrl_info->max_hw_queue_index)
		hw_queue = 0;

	return hw_queue;
}

/*
 * This function gets called just before we hand the completed SCSI request
 * back to the SML.
 */

void pqi_prep_for_scsi_done(struct scsi_cmnd *scmd)
{
	struct pqi_scsi_dev *device;

	device = scmd->device->hostdata;
	atomic_dec(&device->scsi_cmds_outstanding);
}

static int pqi_scsi_queue_command(struct Scsi_Host *shost,
	struct scsi_cmnd *scmd)
{
	int rc;
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_scsi_dev *device;
	u16 hw_queue;
	struct pqi_queue_group *queue_group;
	bool raid_bypassed;

	device = scmd->device->hostdata;
	ctrl_info = shost_to_hba(shost);

	atomic_inc(&device->scsi_cmds_outstanding);

	if (pqi_ctrl_offline(ctrl_info)) {
		set_host_byte(scmd, DID_NO_CONNECT);
		pqi_scsi_done(scmd);
		return 0;
	}

	pqi_ctrl_busy(ctrl_info);
	if (pqi_ctrl_blocked(ctrl_info) || pqi_device_in_reset(device)) {
		rc = SCSI_MLQUEUE_HOST_BUSY;
		goto out;
	}

	/*
	 * This is necessary because the SML doesn't zero out this field during
	 * error recovery.
	 */
	scmd->result = 0;

	hw_queue = pqi_get_hw_queue(ctrl_info, scmd);
	queue_group = &ctrl_info->queue_groups[hw_queue];

	if (pqi_is_logical_device(device)) {
		raid_bypassed = false;
		if (device->raid_bypass_enabled &&
				!blk_rq_is_passthrough(scmd->request)) {
			rc = pqi_raid_bypass_submit_scsi_cmd(ctrl_info, device,
				scmd, queue_group);
			if (rc == 0 || rc == SCSI_MLQUEUE_HOST_BUSY)
				raid_bypassed = true;
		}
		if (!raid_bypassed)
			rc = pqi_raid_submit_scsi_cmd(ctrl_info, device, scmd,
				queue_group);
	} else {
		if (device->aio_enabled)
			rc = pqi_aio_submit_scsi_cmd(ctrl_info, device, scmd,
				queue_group);
		else
			rc = pqi_raid_submit_scsi_cmd(ctrl_info, device, scmd,
				queue_group);
	}

out:
	pqi_ctrl_unbusy(ctrl_info);
	if (rc)
		atomic_dec(&device->scsi_cmds_outstanding);

	return rc;
}

static int pqi_wait_until_queued_io_drained(struct pqi_ctrl_info *ctrl_info,
	struct pqi_queue_group *queue_group)
{
	unsigned int path;
	unsigned long flags;
	bool list_is_empty;

	for (path = 0; path < 2; path++) {
		while (1) {
			spin_lock_irqsave(
				&queue_group->submit_lock[path], flags);
			list_is_empty =
				list_empty(&queue_group->request_list[path]);
			spin_unlock_irqrestore(
				&queue_group->submit_lock[path], flags);
			if (list_is_empty)
				break;
			pqi_check_ctrl_health(ctrl_info);
			if (pqi_ctrl_offline(ctrl_info))
				return -ENXIO;
			usleep_range(1000, 2000);
		}
	}

	return 0;
}

static int pqi_wait_until_inbound_queues_empty(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	unsigned int i;
	unsigned int path;
	struct pqi_queue_group *queue_group;
	pqi_index_t iq_pi;
	pqi_index_t iq_ci;

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		queue_group = &ctrl_info->queue_groups[i];

		rc = pqi_wait_until_queued_io_drained(ctrl_info, queue_group);
		if (rc)
			return rc;

		for (path = 0; path < 2; path++) {
			iq_pi = queue_group->iq_pi_copy[path];

			while (1) {
				iq_ci = *queue_group->iq_ci[path];
				if (iq_ci == iq_pi)
					break;
				pqi_check_ctrl_health(ctrl_info);
				if (pqi_ctrl_offline(ctrl_info))
					return -ENXIO;
				usleep_range(1000, 2000);
			}
		}
	}

	return 0;
}

static void pqi_fail_io_queued_for_device(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	unsigned int i;
	unsigned int path;
	struct pqi_queue_group *queue_group;
	unsigned long flags;
	struct pqi_io_request *io_request;
	struct pqi_io_request *next;
	struct scsi_cmnd *scmd;
	struct pqi_scsi_dev *scsi_device;

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		queue_group = &ctrl_info->queue_groups[i];

		for (path = 0; path < 2; path++) {
			spin_lock_irqsave(
				&queue_group->submit_lock[path], flags);

			list_for_each_entry_safe(io_request, next,
				&queue_group->request_list[path],
				request_list_entry) {
				scmd = io_request->scmd;
				if (!scmd)
					continue;

				scsi_device = scmd->device->hostdata;
				if (scsi_device != device)
					continue;

				list_del(&io_request->request_list_entry);
				set_host_byte(scmd, DID_RESET);
				pqi_scsi_done(scmd);
			}

			spin_unlock_irqrestore(
				&queue_group->submit_lock[path], flags);
		}
	}
}

static int pqi_device_wait_for_pending_io(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	while (atomic_read(&device->scsi_cmds_outstanding)) {
		pqi_check_ctrl_health(ctrl_info);
		if (pqi_ctrl_offline(ctrl_info))
			return -ENXIO;
		usleep_range(1000, 2000);
	}

	return 0;
}

static int pqi_ctrl_wait_for_pending_io(struct pqi_ctrl_info *ctrl_info)
{
	bool io_pending;
	unsigned long flags;
	struct pqi_scsi_dev *device;

	while (1) {
		io_pending = false;

		spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);
		list_for_each_entry(device, &ctrl_info->scsi_device_list,
			scsi_device_list_entry) {
			if (atomic_read(&device->scsi_cmds_outstanding)) {
				io_pending = true;
				break;
			}
		}
		spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock,
					flags);

		if (!io_pending)
			break;

		pqi_check_ctrl_health(ctrl_info);
		if (pqi_ctrl_offline(ctrl_info))
			return -ENXIO;

		usleep_range(1000, 2000);
	}

	return 0;
}

static void pqi_lun_reset_complete(struct pqi_io_request *io_request,
	void *context)
{
	struct completion *waiting = context;

	complete(waiting);
}

#define PQI_LUN_RESET_TIMEOUT_SECS	10

static int pqi_wait_for_lun_reset_completion(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device, struct completion *wait)
{
	int rc;

	while (1) {
		if (wait_for_completion_io_timeout(wait,
			PQI_LUN_RESET_TIMEOUT_SECS * HZ)) {
			rc = 0;
			break;
		}

		pqi_check_ctrl_health(ctrl_info);
		if (pqi_ctrl_offline(ctrl_info)) {
			rc = -ENXIO;
			break;
		}
	}

	return rc;
}

static int pqi_lun_reset(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;
	struct pqi_io_request *io_request;
	DECLARE_COMPLETION_ONSTACK(wait);
	struct pqi_task_management_request *request;

	io_request = pqi_alloc_io_request(ctrl_info);
	io_request->io_complete_callback = pqi_lun_reset_complete;
	io_request->context = &wait;

	request = io_request->iu;
	memset(request, 0, sizeof(*request));

	request->header.iu_type = PQI_REQUEST_IU_TASK_MANAGEMENT;
	put_unaligned_le16(sizeof(*request) - PQI_REQUEST_HEADER_LENGTH,
		&request->header.iu_length);
	put_unaligned_le16(io_request->index, &request->request_id);
	memcpy(request->lun_number, device->scsi3addr,
		sizeof(request->lun_number));
	request->task_management_function = SOP_TASK_MANAGEMENT_LUN_RESET;

	pqi_start_io(ctrl_info,
		&ctrl_info->queue_groups[PQI_DEFAULT_QUEUE_GROUP], RAID_PATH,
		io_request);

	rc = pqi_wait_for_lun_reset_completion(ctrl_info, device, &wait);
	if (rc == 0)
		rc = io_request->status;

	pqi_free_io_request(io_request);

	return rc;
}

/* Performs a reset at the LUN level. */

static int pqi_device_reset(struct pqi_ctrl_info *ctrl_info,
	struct pqi_scsi_dev *device)
{
	int rc;

	rc = pqi_lun_reset(ctrl_info, device);
	if (rc == 0)
		rc = pqi_device_wait_for_pending_io(ctrl_info, device);

	return rc == 0 ? SUCCESS : FAILED;
}

static int pqi_eh_device_reset_handler(struct scsi_cmnd *scmd)
{
	int rc;
	struct Scsi_Host *shost;
	struct pqi_ctrl_info *ctrl_info;
	struct pqi_scsi_dev *device;

	shost = scmd->device->host;
	ctrl_info = shost_to_hba(shost);
	device = scmd->device->hostdata;

	dev_err(&ctrl_info->pci_dev->dev,
		"resetting scsi %d:%d:%d:%d\n",
		shost->host_no, device->bus, device->target, device->lun);

	pqi_check_ctrl_health(ctrl_info);
	if (pqi_ctrl_offline(ctrl_info)) {
		rc = FAILED;
		goto out;
	}

	mutex_lock(&ctrl_info->lun_reset_mutex);

	pqi_ctrl_block_requests(ctrl_info);
	pqi_ctrl_wait_until_quiesced(ctrl_info);
	pqi_fail_io_queued_for_device(ctrl_info, device);
	rc = pqi_wait_until_inbound_queues_empty(ctrl_info);
	pqi_device_reset_start(device);
	pqi_ctrl_unblock_requests(ctrl_info);

	if (rc)
		rc = FAILED;
	else
		rc = pqi_device_reset(ctrl_info, device);

	pqi_device_reset_done(device);

	mutex_unlock(&ctrl_info->lun_reset_mutex);

out:
	dev_err(&ctrl_info->pci_dev->dev,
		"reset of scsi %d:%d:%d:%d: %s\n",
		shost->host_no, device->bus, device->target, device->lun,
		rc == SUCCESS ? "SUCCESS" : "FAILED");

	return rc;
}

static int pqi_slave_alloc(struct scsi_device *sdev)
{
	struct pqi_scsi_dev *device;
	unsigned long flags;
	struct pqi_ctrl_info *ctrl_info;
	struct scsi_target *starget;
	struct sas_rphy *rphy;

	ctrl_info = shost_to_hba(sdev->host);

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

	if (sdev_channel(sdev) == PQI_PHYSICAL_DEVICE_BUS) {
		starget = scsi_target(sdev);
		rphy = target_to_rphy(starget);
		device = pqi_find_device_by_sas_rphy(ctrl_info, rphy);
		if (device) {
			device->target = sdev_id(sdev);
			device->lun = sdev->lun;
			device->target_lun_valid = true;
		}
	} else {
		device = pqi_find_scsi_dev(ctrl_info, sdev_channel(sdev),
			sdev_id(sdev), sdev->lun);
	}

	if (device) {
		sdev->hostdata = device;
		device->sdev = sdev;
		if (device->queue_depth) {
			device->advertised_queue_depth = device->queue_depth;
			scsi_change_queue_depth(sdev,
				device->advertised_queue_depth);
		}
	}

	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	return 0;
}

static int pqi_map_queues(struct Scsi_Host *shost)
{
	struct pqi_ctrl_info *ctrl_info = shost_to_hba(shost);

	return blk_mq_pci_map_queues(&shost->tag_set, ctrl_info->pci_dev);
}

static int pqi_getpciinfo_ioctl(struct pqi_ctrl_info *ctrl_info,
	void __user *arg)
{
	struct pci_dev *pci_dev;
	u32 subsystem_vendor;
	u32 subsystem_device;
	cciss_pci_info_struct pciinfo;

	if (!arg)
		return -EINVAL;

	pci_dev = ctrl_info->pci_dev;

	pciinfo.domain = pci_domain_nr(pci_dev->bus);
	pciinfo.bus = pci_dev->bus->number;
	pciinfo.dev_fn = pci_dev->devfn;
	subsystem_vendor = pci_dev->subsystem_vendor;
	subsystem_device = pci_dev->subsystem_device;
	pciinfo.board_id = ((subsystem_device << 16) & 0xffff0000) |
		subsystem_vendor;

	if (copy_to_user(arg, &pciinfo, sizeof(pciinfo)))
		return -EFAULT;

	return 0;
}

static int pqi_getdrivver_ioctl(void __user *arg)
{
	u32 version;

	if (!arg)
		return -EINVAL;

	version = (DRIVER_MAJOR << 28) | (DRIVER_MINOR << 24) |
		(DRIVER_RELEASE << 16) | DRIVER_REVISION;

	if (copy_to_user(arg, &version, sizeof(version)))
		return -EFAULT;

	return 0;
}

struct ciss_error_info {
	u8	scsi_status;
	int	command_status;
	size_t	sense_data_length;
};

static void pqi_error_info_to_ciss(struct pqi_raid_error_info *pqi_error_info,
	struct ciss_error_info *ciss_error_info)
{
	int ciss_cmd_status;
	size_t sense_data_length;

	switch (pqi_error_info->data_out_result) {
	case PQI_DATA_IN_OUT_GOOD:
		ciss_cmd_status = CISS_CMD_STATUS_SUCCESS;
		break;
	case PQI_DATA_IN_OUT_UNDERFLOW:
		ciss_cmd_status = CISS_CMD_STATUS_DATA_UNDERRUN;
		break;
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW:
		ciss_cmd_status = CISS_CMD_STATUS_DATA_OVERRUN;
		break;
	case PQI_DATA_IN_OUT_PROTOCOL_ERROR:
	case PQI_DATA_IN_OUT_BUFFER_ERROR:
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW_DESCRIPTOR_AREA:
	case PQI_DATA_IN_OUT_BUFFER_OVERFLOW_BRIDGE:
	case PQI_DATA_IN_OUT_ERROR:
		ciss_cmd_status = CISS_CMD_STATUS_PROTOCOL_ERROR;
		break;
	case PQI_DATA_IN_OUT_HARDWARE_ERROR:
	case PQI_DATA_IN_OUT_PCIE_FABRIC_ERROR:
	case PQI_DATA_IN_OUT_PCIE_COMPLETION_TIMEOUT:
	case PQI_DATA_IN_OUT_PCIE_COMPLETER_ABORT_RECEIVED:
	case PQI_DATA_IN_OUT_PCIE_UNSUPPORTED_REQUEST_RECEIVED:
	case PQI_DATA_IN_OUT_PCIE_ECRC_CHECK_FAILED:
	case PQI_DATA_IN_OUT_PCIE_UNSUPPORTED_REQUEST:
	case PQI_DATA_IN_OUT_PCIE_ACS_VIOLATION:
	case PQI_DATA_IN_OUT_PCIE_TLP_PREFIX_BLOCKED:
	case PQI_DATA_IN_OUT_PCIE_POISONED_MEMORY_READ:
		ciss_cmd_status = CISS_CMD_STATUS_HARDWARE_ERROR;
		break;
	case PQI_DATA_IN_OUT_UNSOLICITED_ABORT:
		ciss_cmd_status = CISS_CMD_STATUS_UNSOLICITED_ABORT;
		break;
	case PQI_DATA_IN_OUT_ABORTED:
		ciss_cmd_status = CISS_CMD_STATUS_ABORTED;
		break;
	case PQI_DATA_IN_OUT_TIMEOUT:
		ciss_cmd_status = CISS_CMD_STATUS_TIMEOUT;
		break;
	default:
		ciss_cmd_status = CISS_CMD_STATUS_TARGET_STATUS;
		break;
	}

	sense_data_length =
		get_unaligned_le16(&pqi_error_info->sense_data_length);
	if (sense_data_length == 0)
		sense_data_length =
		get_unaligned_le16(&pqi_error_info->response_data_length);
	if (sense_data_length)
		if (sense_data_length > sizeof(pqi_error_info->data))
			sense_data_length = sizeof(pqi_error_info->data);

	ciss_error_info->scsi_status = pqi_error_info->status;
	ciss_error_info->command_status = ciss_cmd_status;
	ciss_error_info->sense_data_length = sense_data_length;
}

static int pqi_passthru_ioctl(struct pqi_ctrl_info *ctrl_info, void __user *arg)
{
	int rc;
	char *kernel_buffer = NULL;
	u16 iu_length;
	size_t sense_data_length;
	IOCTL_Command_struct iocommand;
	struct pqi_raid_path_request request;
	struct pqi_raid_error_info pqi_error_info;
	struct ciss_error_info ciss_error_info;

	if (pqi_ctrl_offline(ctrl_info))
		return -ENXIO;
	if (!arg)
		return -EINVAL;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (copy_from_user(&iocommand, arg, sizeof(iocommand)))
		return -EFAULT;
	if (iocommand.buf_size < 1 &&
		iocommand.Request.Type.Direction != XFER_NONE)
		return -EINVAL;
	if (iocommand.Request.CDBLen > sizeof(request.cdb))
		return -EINVAL;
	if (iocommand.Request.Type.Type != TYPE_CMD)
		return -EINVAL;

	switch (iocommand.Request.Type.Direction) {
	case XFER_NONE:
	case XFER_WRITE:
	case XFER_READ:
	case XFER_READ | XFER_WRITE:
		break;
	default:
		return -EINVAL;
	}

	if (iocommand.buf_size > 0) {
		kernel_buffer = kmalloc(iocommand.buf_size, GFP_KERNEL);
		if (!kernel_buffer)
			return -ENOMEM;
		if (iocommand.Request.Type.Direction & XFER_WRITE) {
			if (copy_from_user(kernel_buffer, iocommand.buf,
				iocommand.buf_size)) {
				rc = -EFAULT;
				goto out;
			}
		} else {
			memset(kernel_buffer, 0, iocommand.buf_size);
		}
	}

	memset(&request, 0, sizeof(request));

	request.header.iu_type = PQI_REQUEST_IU_RAID_PATH_IO;
	iu_length = offsetof(struct pqi_raid_path_request, sg_descriptors) -
		PQI_REQUEST_HEADER_LENGTH;
	memcpy(request.lun_number, iocommand.LUN_info.LunAddrBytes,
		sizeof(request.lun_number));
	memcpy(request.cdb, iocommand.Request.CDB, iocommand.Request.CDBLen);
	request.additional_cdb_bytes_usage = SOP_ADDITIONAL_CDB_BYTES_0;

	switch (iocommand.Request.Type.Direction) {
	case XFER_NONE:
		request.data_direction = SOP_NO_DIRECTION_FLAG;
		break;
	case XFER_WRITE:
		request.data_direction = SOP_WRITE_FLAG;
		break;
	case XFER_READ:
		request.data_direction = SOP_READ_FLAG;
		break;
	case XFER_READ | XFER_WRITE:
		request.data_direction = SOP_BIDIRECTIONAL;
		break;
	}

	request.task_attribute = SOP_TASK_ATTRIBUTE_SIMPLE;

	if (iocommand.buf_size > 0) {
		put_unaligned_le32(iocommand.buf_size, &request.buffer_length);

		rc = pqi_map_single(ctrl_info->pci_dev,
			&request.sg_descriptors[0], kernel_buffer,
			iocommand.buf_size, PCI_DMA_BIDIRECTIONAL);
		if (rc)
			goto out;

		iu_length += sizeof(request.sg_descriptors[0]);
	}

	put_unaligned_le16(iu_length, &request.header.iu_length);

	rc = pqi_submit_raid_request_synchronous(ctrl_info, &request.header,
		PQI_SYNC_FLAGS_INTERRUPTABLE, &pqi_error_info, NO_TIMEOUT);

	if (iocommand.buf_size > 0)
		pqi_pci_unmap(ctrl_info->pci_dev, request.sg_descriptors, 1,
			PCI_DMA_BIDIRECTIONAL);

	memset(&iocommand.error_info, 0, sizeof(iocommand.error_info));

	if (rc == 0) {
		pqi_error_info_to_ciss(&pqi_error_info, &ciss_error_info);
		iocommand.error_info.ScsiStatus = ciss_error_info.scsi_status;
		iocommand.error_info.CommandStatus =
			ciss_error_info.command_status;
		sense_data_length = ciss_error_info.sense_data_length;
		if (sense_data_length) {
			if (sense_data_length >
				sizeof(iocommand.error_info.SenseInfo))
				sense_data_length =
					sizeof(iocommand.error_info.SenseInfo);
			memcpy(iocommand.error_info.SenseInfo,
				pqi_error_info.data, sense_data_length);
			iocommand.error_info.SenseLen = sense_data_length;
		}
	}

	if (copy_to_user(arg, &iocommand, sizeof(iocommand))) {
		rc = -EFAULT;
		goto out;
	}

	if (rc == 0 && iocommand.buf_size > 0 &&
		(iocommand.Request.Type.Direction & XFER_READ)) {
		if (copy_to_user(iocommand.buf, kernel_buffer,
			iocommand.buf_size)) {
			rc = -EFAULT;
		}
	}

out:
	kfree(kernel_buffer);

	return rc;
}

static int pqi_ioctl(struct scsi_device *sdev, int cmd, void __user *arg)
{
	int rc;
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = shost_to_hba(sdev->host);

	switch (cmd) {
	case CCISS_DEREGDISK:
	case CCISS_REGNEWDISK:
	case CCISS_REGNEWD:
		rc = pqi_scan_scsi_devices(ctrl_info);
		break;
	case CCISS_GETPCIINFO:
		rc = pqi_getpciinfo_ioctl(ctrl_info, arg);
		break;
	case CCISS_GETDRIVVER:
		rc = pqi_getdrivver_ioctl(arg);
		break;
	case CCISS_PASSTHRU:
		rc = pqi_passthru_ioctl(ctrl_info, arg);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static ssize_t pqi_version_show(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	ssize_t count = 0;
	struct Scsi_Host *shost;
	struct pqi_ctrl_info *ctrl_info;

	shost = class_to_shost(dev);
	ctrl_info = shost_to_hba(shost);

	count += snprintf(buffer + count, PAGE_SIZE - count,
		"  driver: %s\n", DRIVER_VERSION BUILD_TIMESTAMP);

	count += snprintf(buffer + count, PAGE_SIZE - count,
		"firmware: %s\n", ctrl_info->firmware_version);

	return count;
}

static ssize_t pqi_host_rescan_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	struct Scsi_Host *shost = class_to_shost(dev);

	pqi_scan_start(shost);

	return count;
}

static ssize_t pqi_lockup_action_show(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	int count = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pqi_lockup_actions); i++) {
		if (pqi_lockup_actions[i].action == pqi_lockup_action)
			count += snprintf(buffer + count, PAGE_SIZE - count,
				"[%s] ", pqi_lockup_actions[i].name);
		else
			count += snprintf(buffer + count, PAGE_SIZE - count,
				"%s ", pqi_lockup_actions[i].name);
	}

	count += snprintf(buffer + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t pqi_lockup_action_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	unsigned int i;
	char *action_name;
	char action_name_buffer[32];

	strlcpy(action_name_buffer, buffer, sizeof(action_name_buffer));
	action_name = strstrip(action_name_buffer);

	for (i = 0; i < ARRAY_SIZE(pqi_lockup_actions); i++) {
		if (strcmp(action_name, pqi_lockup_actions[i].name) == 0) {
			pqi_lockup_action = pqi_lockup_actions[i].action;
			return count;
		}
	}

	return -EINVAL;
}

static DEVICE_ATTR(version, 0444, pqi_version_show, NULL);
static DEVICE_ATTR(rescan, 0200, NULL, pqi_host_rescan_store);
static DEVICE_ATTR(lockup_action, 0644,
	pqi_lockup_action_show, pqi_lockup_action_store);

static struct device_attribute *pqi_shost_attrs[] = {
	&dev_attr_version,
	&dev_attr_rescan,
	&dev_attr_lockup_action,
	NULL
};

static ssize_t pqi_sas_address_show(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct pqi_ctrl_info *ctrl_info;
	struct scsi_device *sdev;
	struct pqi_scsi_dev *device;
	unsigned long flags;
	u64 sas_address;

	sdev = to_scsi_device(dev);
	ctrl_info = shost_to_hba(sdev->host);

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

	device = sdev->hostdata;
	if (pqi_is_logical_device(device)) {
		spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock,
			flags);
		return -ENODEV;
	}
	sas_address = device->sas_address;

	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	return snprintf(buffer, PAGE_SIZE, "0x%016llx\n", sas_address);
}

static ssize_t pqi_ssd_smart_path_enabled_show(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct pqi_ctrl_info *ctrl_info;
	struct scsi_device *sdev;
	struct pqi_scsi_dev *device;
	unsigned long flags;

	sdev = to_scsi_device(dev);
	ctrl_info = shost_to_hba(sdev->host);

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

	device = sdev->hostdata;
	buffer[0] = device->raid_bypass_enabled ? '1' : '0';
	buffer[1] = '\n';
	buffer[2] = '\0';

	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	return 2;
}

static ssize_t pqi_raid_level_show(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct pqi_ctrl_info *ctrl_info;
	struct scsi_device *sdev;
	struct pqi_scsi_dev *device;
	unsigned long flags;
	char *raid_level;

	sdev = to_scsi_device(dev);
	ctrl_info = shost_to_hba(sdev->host);

	spin_lock_irqsave(&ctrl_info->scsi_device_list_lock, flags);

	device = sdev->hostdata;

	if (pqi_is_logical_device(device))
		raid_level = pqi_raid_level_to_string(device->raid_level);
	else
		raid_level = "N/A";

	spin_unlock_irqrestore(&ctrl_info->scsi_device_list_lock, flags);

	return snprintf(buffer, PAGE_SIZE, "%s\n", raid_level);
}

static DEVICE_ATTR(sas_address, 0444, pqi_sas_address_show, NULL);
static DEVICE_ATTR(ssd_smart_path_enabled, 0444,
	pqi_ssd_smart_path_enabled_show, NULL);
static DEVICE_ATTR(raid_level, 0444, pqi_raid_level_show, NULL);

static struct device_attribute *pqi_sdev_attrs[] = {
	&dev_attr_sas_address,
	&dev_attr_ssd_smart_path_enabled,
	&dev_attr_raid_level,
	NULL
};

static struct scsi_host_template pqi_driver_template = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME_SHORT,
	.proc_name = DRIVER_NAME_SHORT,
	.queuecommand = pqi_scsi_queue_command,
	.scan_start = pqi_scan_start,
	.scan_finished = pqi_scan_finished,
	.this_id = -1,
	.use_clustering = ENABLE_CLUSTERING,
	.eh_device_reset_handler = pqi_eh_device_reset_handler,
	.ioctl = pqi_ioctl,
	.slave_alloc = pqi_slave_alloc,
	.map_queues = pqi_map_queues,
	.sdev_attrs = pqi_sdev_attrs,
	.shost_attrs = pqi_shost_attrs,
};

static int pqi_register_scsi(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct Scsi_Host *shost;

	shost = scsi_host_alloc(&pqi_driver_template, sizeof(ctrl_info));
	if (!shost) {
		dev_err(&ctrl_info->pci_dev->dev,
			"scsi_host_alloc failed for controller %u\n",
			ctrl_info->ctrl_id);
		return -ENOMEM;
	}

	shost->io_port = 0;
	shost->n_io_port = 0;
	shost->this_id = -1;
	shost->max_channel = PQI_MAX_BUS;
	shost->max_cmd_len = MAX_COMMAND_SIZE;
	shost->max_lun = ~0;
	shost->max_id = ~0;
	shost->max_sectors = ctrl_info->max_sectors;
	shost->can_queue = ctrl_info->scsi_ml_can_queue;
	shost->cmd_per_lun = shost->can_queue;
	shost->sg_tablesize = ctrl_info->sg_tablesize;
	shost->transportt = pqi_sas_transport_template;
	shost->irq = pci_irq_vector(ctrl_info->pci_dev, 0);
	shost->unique_id = shost->irq;
	shost->nr_hw_queues = ctrl_info->num_queue_groups;
	shost->hostdata[0] = (unsigned long)ctrl_info;

	rc = scsi_add_host(shost, &ctrl_info->pci_dev->dev);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"scsi_add_host failed for controller %u\n",
			ctrl_info->ctrl_id);
		goto free_host;
	}

	rc = pqi_add_sas_host(shost, ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"add SAS host failed for controller %u\n",
			ctrl_info->ctrl_id);
		goto remove_host;
	}

	ctrl_info->scsi_host = shost;

	return 0;

remove_host:
	scsi_remove_host(shost);
free_host:
	scsi_host_put(shost);

	return rc;
}

static void pqi_unregister_scsi(struct pqi_ctrl_info *ctrl_info)
{
	struct Scsi_Host *shost;

	pqi_delete_sas_host(ctrl_info);

	shost = ctrl_info->scsi_host;
	if (!shost)
		return;

	scsi_remove_host(shost);
	scsi_host_put(shost);
}

static int pqi_wait_for_pqi_reset_completion(struct pqi_ctrl_info *ctrl_info)
{
	int rc = 0;
	struct pqi_device_registers __iomem *pqi_registers;
	unsigned long timeout;
	unsigned int timeout_msecs;
	union pqi_reset_register reset_reg;

	pqi_registers = ctrl_info->pqi_registers;
	timeout_msecs = readw(&pqi_registers->max_reset_timeout) * 100;
	timeout = msecs_to_jiffies(timeout_msecs) + jiffies;

	while (1) {
		msleep(PQI_RESET_POLL_INTERVAL_MSECS);
		reset_reg.all_bits = readl(&pqi_registers->device_reset);
		if (reset_reg.bits.reset_action == PQI_RESET_ACTION_COMPLETED)
			break;
		pqi_check_ctrl_health(ctrl_info);
		if (pqi_ctrl_offline(ctrl_info)) {
			rc = -ENXIO;
			break;
		}
		if (time_after(jiffies, timeout)) {
			rc = -ETIMEDOUT;
			break;
		}
	}

	return rc;
}

static int pqi_reset(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	union pqi_reset_register reset_reg;

	if (ctrl_info->pqi_reset_quiesce_supported) {
		rc = sis_pqi_reset_quiesce(ctrl_info);
		if (rc) {
			dev_err(&ctrl_info->pci_dev->dev,
				"PQI reset failed during quiesce with error %d\n",
				rc);
			return rc;
		}
	}

	reset_reg.all_bits = 0;
	reset_reg.bits.reset_type = PQI_RESET_TYPE_HARD_RESET;
	reset_reg.bits.reset_action = PQI_RESET_ACTION_RESET;

	writel(reset_reg.all_bits, &ctrl_info->pqi_registers->device_reset);

	rc = pqi_wait_for_pqi_reset_completion(ctrl_info);
	if (rc)
		dev_err(&ctrl_info->pci_dev->dev,
			"PQI reset failed with error %d\n", rc);

	return rc;
}

static int pqi_get_ctrl_firmware_version(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	struct bmic_identify_controller *identify;

	identify = kmalloc(sizeof(*identify), GFP_KERNEL);
	if (!identify)
		return -ENOMEM;

	rc = pqi_identify_controller(ctrl_info, identify);
	if (rc)
		goto out;

	memcpy(ctrl_info->firmware_version, identify->firmware_version,
		sizeof(identify->firmware_version));
	ctrl_info->firmware_version[sizeof(identify->firmware_version)] = '\0';
	snprintf(ctrl_info->firmware_version +
		strlen(ctrl_info->firmware_version),
		sizeof(ctrl_info->firmware_version),
		"-%u", get_unaligned_le16(&identify->firmware_build_number));

out:
	kfree(identify);

	return rc;
}

static int pqi_process_config_table(struct pqi_ctrl_info *ctrl_info)
{
	u32 table_length;
	u32 section_offset;
	void __iomem *table_iomem_addr;
	struct pqi_config_table *config_table;
	struct pqi_config_table_section_header *section;

	table_length = ctrl_info->config_table_length;

	config_table = kmalloc(table_length, GFP_KERNEL);
	if (!config_table) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to allocate memory for PQI configuration table\n");
		return -ENOMEM;
	}

	/*
	 * Copy the config table contents from I/O memory space into the
	 * temporary buffer.
	 */
	table_iomem_addr = ctrl_info->iomem_base +
		ctrl_info->config_table_offset;
	memcpy_fromio(config_table, table_iomem_addr, table_length);

	section_offset =
		get_unaligned_le32(&config_table->first_section_offset);

	while (section_offset) {
		section = (void *)config_table + section_offset;

		switch (get_unaligned_le16(&section->section_id)) {
		case PQI_CONFIG_TABLE_SECTION_HEARTBEAT:
			if (pqi_disable_heartbeat)
				dev_warn(&ctrl_info->pci_dev->dev,
				"heartbeat disabled by module parameter\n");
			else
				ctrl_info->heartbeat_counter =
					table_iomem_addr +
					section_offset +
					offsetof(
					struct pqi_config_table_heartbeat,
						heartbeat_counter);
			break;
		}

		section_offset =
			get_unaligned_le16(&section->next_section_offset);
	}

	kfree(config_table);

	return 0;
}

/* Switches the controller from PQI mode back into SIS mode. */

static int pqi_revert_to_sis_mode(struct pqi_ctrl_info *ctrl_info)
{
	int rc;

	pqi_change_irq_mode(ctrl_info, IRQ_MODE_NONE);
	rc = pqi_reset(ctrl_info);
	if (rc)
		return rc;
	rc = sis_reenable_sis_mode(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"re-enabling SIS mode failed with error %d\n", rc);
		return rc;
	}
	pqi_save_ctrl_mode(ctrl_info, SIS_MODE);

	return 0;
}

/*
 * If the controller isn't already in SIS mode, this function forces it into
 * SIS mode.
 */

static int pqi_force_sis_mode(struct pqi_ctrl_info *ctrl_info)
{
	if (!sis_is_firmware_running(ctrl_info))
		return -ENXIO;

	if (pqi_get_ctrl_mode(ctrl_info) == SIS_MODE)
		return 0;

	if (sis_is_kernel_up(ctrl_info)) {
		pqi_save_ctrl_mode(ctrl_info, SIS_MODE);
		return 0;
	}

	return pqi_revert_to_sis_mode(ctrl_info);
}

static int pqi_ctrl_init(struct pqi_ctrl_info *ctrl_info)
{
	int rc;

	rc = pqi_force_sis_mode(ctrl_info);
	if (rc)
		return rc;

	/*
	 * Wait until the controller is ready to start accepting SIS
	 * commands.
	 */
	rc = sis_wait_for_ctrl_ready(ctrl_info);
	if (rc)
		return rc;

	/*
	 * Get the controller properties.  This allows us to determine
	 * whether or not it supports PQI mode.
	 */
	rc = sis_get_ctrl_properties(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error obtaining controller properties\n");
		return rc;
	}

	rc = sis_get_pqi_capabilities(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error obtaining controller capabilities\n");
		return rc;
	}

	if (reset_devices) {
		if (ctrl_info->max_outstanding_requests >
			PQI_MAX_OUTSTANDING_REQUESTS_KDUMP)
			ctrl_info->max_outstanding_requests =
					PQI_MAX_OUTSTANDING_REQUESTS_KDUMP;
	} else {
		if (ctrl_info->max_outstanding_requests >
			PQI_MAX_OUTSTANDING_REQUESTS)
			ctrl_info->max_outstanding_requests =
					PQI_MAX_OUTSTANDING_REQUESTS;
	}

	pqi_calculate_io_resources(ctrl_info);

	rc = pqi_alloc_error_buffer(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to allocate PQI error buffer\n");
		return rc;
	}

	/*
	 * If the function we are about to call succeeds, the
	 * controller will transition from legacy SIS mode
	 * into PQI mode.
	 */
	rc = sis_init_base_struct_addr(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error initializing PQI mode\n");
		return rc;
	}

	/* Wait for the controller to complete the SIS -> PQI transition. */
	rc = pqi_wait_for_pqi_mode_ready(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"transition to PQI mode failed\n");
		return rc;
	}

	/* From here on, we are running in PQI mode. */
	ctrl_info->pqi_mode_enabled = true;
	pqi_save_ctrl_mode(ctrl_info, PQI_MODE);

	rc = pqi_process_config_table(ctrl_info);
	if (rc)
		return rc;

	rc = pqi_alloc_admin_queues(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to allocate admin queues\n");
		return rc;
	}

	rc = pqi_create_admin_queues(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating admin queues\n");
		return rc;
	}

	rc = pqi_report_device_capability(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"obtaining device capability failed\n");
		return rc;
	}

	rc = pqi_validate_device_capability(ctrl_info);
	if (rc)
		return rc;

	pqi_calculate_queue_resources(ctrl_info);

	rc = pqi_enable_msix_interrupts(ctrl_info);
	if (rc)
		return rc;

	if (ctrl_info->num_msix_vectors_enabled < ctrl_info->num_queue_groups) {
		ctrl_info->max_msix_vectors =
			ctrl_info->num_msix_vectors_enabled;
		pqi_calculate_queue_resources(ctrl_info);
	}

	rc = pqi_alloc_io_resources(ctrl_info);
	if (rc)
		return rc;

	rc = pqi_alloc_operational_queues(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to allocate operational queues\n");
		return rc;
	}

	pqi_init_operational_queues(ctrl_info);

	rc = pqi_request_irqs(ctrl_info);
	if (rc)
		return rc;

	rc = pqi_create_queues(ctrl_info);
	if (rc)
		return rc;

	pqi_change_irq_mode(ctrl_info, IRQ_MODE_MSIX);

	ctrl_info->controller_online = true;
	pqi_start_heartbeat_timer(ctrl_info);

	rc = pqi_enable_events(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error enabling events\n");
		return rc;
	}

	/* Register with the SCSI subsystem. */
	rc = pqi_register_scsi(ctrl_info);
	if (rc)
		return rc;

	rc = pqi_get_ctrl_firmware_version(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error obtaining firmware version\n");
		return rc;
	}

	rc = pqi_write_driver_version_to_host_wellness(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error updating host wellness\n");
		return rc;
	}

	pqi_schedule_update_time_worker(ctrl_info);

	pqi_scan_scsi_devices(ctrl_info);

	return 0;
}

static void pqi_reinit_queues(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	struct pqi_admin_queues *admin_queues;
	struct pqi_event_queue *event_queue;

	admin_queues = &ctrl_info->admin_queues;
	admin_queues->iq_pi_copy = 0;
	admin_queues->oq_ci_copy = 0;
	*admin_queues->oq_pi = 0;

	for (i = 0; i < ctrl_info->num_queue_groups; i++) {
		ctrl_info->queue_groups[i].iq_pi_copy[RAID_PATH] = 0;
		ctrl_info->queue_groups[i].iq_pi_copy[AIO_PATH] = 0;
		ctrl_info->queue_groups[i].oq_ci_copy = 0;

		*ctrl_info->queue_groups[i].iq_ci[RAID_PATH] = 0;
		*ctrl_info->queue_groups[i].iq_ci[AIO_PATH] = 0;
		*ctrl_info->queue_groups[i].oq_pi = 0;
	}

	event_queue = &ctrl_info->event_queue;
	*event_queue->oq_pi = 0;
	event_queue->oq_ci_copy = 0;
}

static int pqi_ctrl_init_resume(struct pqi_ctrl_info *ctrl_info)
{
	int rc;

	rc = pqi_force_sis_mode(ctrl_info);
	if (rc)
		return rc;

	/*
	 * Wait until the controller is ready to start accepting SIS
	 * commands.
	 */
	rc = sis_wait_for_ctrl_ready_resume(ctrl_info);
	if (rc)
		return rc;

	/*
	 * If the function we are about to call succeeds, the
	 * controller will transition from legacy SIS mode
	 * into PQI mode.
	 */
	rc = sis_init_base_struct_addr(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error initializing PQI mode\n");
		return rc;
	}

	/* Wait for the controller to complete the SIS -> PQI transition. */
	rc = pqi_wait_for_pqi_mode_ready(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"transition to PQI mode failed\n");
		return rc;
	}

	/* From here on, we are running in PQI mode. */
	ctrl_info->pqi_mode_enabled = true;
	pqi_save_ctrl_mode(ctrl_info, PQI_MODE);

	pqi_reinit_queues(ctrl_info);

	rc = pqi_create_admin_queues(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error creating admin queues\n");
		return rc;
	}

	rc = pqi_create_queues(ctrl_info);
	if (rc)
		return rc;

	pqi_change_irq_mode(ctrl_info, IRQ_MODE_MSIX);

	ctrl_info->controller_online = true;
	pqi_start_heartbeat_timer(ctrl_info);
	pqi_ctrl_unblock_requests(ctrl_info);

	rc = pqi_enable_events(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error enabling events\n");
		return rc;
	}

	rc = pqi_write_driver_version_to_host_wellness(ctrl_info);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"error updating host wellness\n");
		return rc;
	}

	pqi_schedule_update_time_worker(ctrl_info);

	pqi_scan_scsi_devices(ctrl_info);

	return 0;
}

static inline int pqi_set_pcie_completion_timeout(struct pci_dev *pci_dev,
	u16 timeout)
{
	return pcie_capability_clear_and_set_word(pci_dev, PCI_EXP_DEVCTL2,
		PCI_EXP_DEVCTL2_COMP_TIMEOUT, timeout);
}

static int pqi_pci_init(struct pqi_ctrl_info *ctrl_info)
{
	int rc;
	u64 mask;

	rc = pci_enable_device(ctrl_info->pci_dev);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to enable PCI device\n");
		return rc;
	}

	if (sizeof(dma_addr_t) > 4)
		mask = DMA_BIT_MASK(64);
	else
		mask = DMA_BIT_MASK(32);

	rc = dma_set_mask(&ctrl_info->pci_dev->dev, mask);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev, "failed to set DMA mask\n");
		goto disable_device;
	}

	rc = pci_request_regions(ctrl_info->pci_dev, DRIVER_NAME_SHORT);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to obtain PCI resources\n");
		goto disable_device;
	}

	ctrl_info->iomem_base = ioremap_nocache(pci_resource_start(
		ctrl_info->pci_dev, 0),
		sizeof(struct pqi_ctrl_registers));
	if (!ctrl_info->iomem_base) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to map memory for controller registers\n");
		rc = -ENOMEM;
		goto release_regions;
	}

#define PCI_EXP_COMP_TIMEOUT_65_TO_210_MS		0x6

	/* Increase the PCIe completion timeout. */
	rc = pqi_set_pcie_completion_timeout(ctrl_info->pci_dev,
		PCI_EXP_COMP_TIMEOUT_65_TO_210_MS);
	if (rc) {
		dev_err(&ctrl_info->pci_dev->dev,
			"failed to set PCIe completion timeout\n");
		goto release_regions;
	}

	/* Enable bus mastering. */
	pci_set_master(ctrl_info->pci_dev);

	ctrl_info->registers = ctrl_info->iomem_base;
	ctrl_info->pqi_registers = &ctrl_info->registers->pqi_registers;

	pci_set_drvdata(ctrl_info->pci_dev, ctrl_info);

	return 0;

release_regions:
	pci_release_regions(ctrl_info->pci_dev);
disable_device:
	pci_disable_device(ctrl_info->pci_dev);

	return rc;
}

static void pqi_cleanup_pci_init(struct pqi_ctrl_info *ctrl_info)
{
	iounmap(ctrl_info->iomem_base);
	pci_release_regions(ctrl_info->pci_dev);
	if (pci_is_enabled(ctrl_info->pci_dev))
		pci_disable_device(ctrl_info->pci_dev);
	pci_set_drvdata(ctrl_info->pci_dev, NULL);
}

static struct pqi_ctrl_info *pqi_alloc_ctrl_info(int numa_node)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = kzalloc_node(sizeof(struct pqi_ctrl_info),
			GFP_KERNEL, numa_node);
	if (!ctrl_info)
		return NULL;

	mutex_init(&ctrl_info->scan_mutex);
	mutex_init(&ctrl_info->lun_reset_mutex);

	INIT_LIST_HEAD(&ctrl_info->scsi_device_list);
	spin_lock_init(&ctrl_info->scsi_device_list_lock);

	INIT_WORK(&ctrl_info->event_work, pqi_event_worker);
	atomic_set(&ctrl_info->num_interrupts, 0);

	INIT_DELAYED_WORK(&ctrl_info->rescan_work, pqi_rescan_worker);
	INIT_DELAYED_WORK(&ctrl_info->update_time_work, pqi_update_time_worker);

	timer_setup(&ctrl_info->heartbeat_timer, pqi_heartbeat_timer_handler, 0);
	INIT_WORK(&ctrl_info->ctrl_offline_work, pqi_ctrl_offline_worker);

	sema_init(&ctrl_info->sync_request_sem,
		PQI_RESERVED_IO_SLOTS_SYNCHRONOUS_REQUESTS);
	init_waitqueue_head(&ctrl_info->block_requests_wait);

	INIT_LIST_HEAD(&ctrl_info->raid_bypass_retry_list);
	spin_lock_init(&ctrl_info->raid_bypass_retry_list_lock);
	INIT_WORK(&ctrl_info->raid_bypass_retry_work,
		pqi_raid_bypass_retry_worker);

	ctrl_info->ctrl_id = atomic_inc_return(&pqi_controller_count) - 1;
	ctrl_info->irq_mode = IRQ_MODE_NONE;
	ctrl_info->max_msix_vectors = PQI_MAX_MSIX_VECTORS;

	return ctrl_info;
}

static inline void pqi_free_ctrl_info(struct pqi_ctrl_info *ctrl_info)
{
	kfree(ctrl_info);
}

static void pqi_free_interrupts(struct pqi_ctrl_info *ctrl_info)
{
	pqi_free_irqs(ctrl_info);
	pqi_disable_msix_interrupts(ctrl_info);
}

static void pqi_free_ctrl_resources(struct pqi_ctrl_info *ctrl_info)
{
	pqi_stop_heartbeat_timer(ctrl_info);
	pqi_free_interrupts(ctrl_info);
	if (ctrl_info->queue_memory_base)
		dma_free_coherent(&ctrl_info->pci_dev->dev,
			ctrl_info->queue_memory_length,
			ctrl_info->queue_memory_base,
			ctrl_info->queue_memory_base_dma_handle);
	if (ctrl_info->admin_queue_memory_base)
		dma_free_coherent(&ctrl_info->pci_dev->dev,
			ctrl_info->admin_queue_memory_length,
			ctrl_info->admin_queue_memory_base,
			ctrl_info->admin_queue_memory_base_dma_handle);
	pqi_free_all_io_requests(ctrl_info);
	if (ctrl_info->error_buffer)
		dma_free_coherent(&ctrl_info->pci_dev->dev,
			ctrl_info->error_buffer_length,
			ctrl_info->error_buffer,
			ctrl_info->error_buffer_dma_handle);
	if (ctrl_info->iomem_base)
		pqi_cleanup_pci_init(ctrl_info);
	pqi_free_ctrl_info(ctrl_info);
}

static void pqi_remove_ctrl(struct pqi_ctrl_info *ctrl_info)
{
	pqi_cancel_rescan_worker(ctrl_info);
	pqi_cancel_update_time_worker(ctrl_info);
	pqi_remove_all_scsi_devices(ctrl_info);
	pqi_unregister_scsi(ctrl_info);
	if (ctrl_info->pqi_mode_enabled)
		pqi_revert_to_sis_mode(ctrl_info);
	pqi_free_ctrl_resources(ctrl_info);
}

static void pqi_perform_lockup_action(void)
{
	switch (pqi_lockup_action) {
	case PANIC:
		panic("FATAL: Smart Family Controller lockup detected");
		break;
	case REBOOT:
		emergency_restart();
		break;
	case NONE:
	default:
		break;
	}
}

static struct pqi_raid_error_info pqi_ctrl_offline_raid_error_info = {
	.data_out_result = PQI_DATA_IN_OUT_HARDWARE_ERROR,
	.status = SAM_STAT_CHECK_CONDITION,
};

static void pqi_fail_all_outstanding_requests(struct pqi_ctrl_info *ctrl_info)
{
	unsigned int i;
	struct pqi_io_request *io_request;
	struct scsi_cmnd *scmd;

	for (i = 0; i < ctrl_info->max_io_slots; i++) {
		io_request = &ctrl_info->io_request_pool[i];
		if (atomic_read(&io_request->refcount) == 0)
			continue;

		scmd = io_request->scmd;
		if (scmd) {
			set_host_byte(scmd, DID_NO_CONNECT);
		} else {
			io_request->status = -ENXIO;
			io_request->error_info =
				&pqi_ctrl_offline_raid_error_info;
		}

		io_request->io_complete_callback(io_request,
			io_request->context);
	}
}

static void pqi_take_ctrl_offline_deferred(struct pqi_ctrl_info *ctrl_info)
{
	pqi_perform_lockup_action();
	pqi_stop_heartbeat_timer(ctrl_info);
	pqi_free_interrupts(ctrl_info);
	pqi_cancel_rescan_worker(ctrl_info);
	pqi_cancel_update_time_worker(ctrl_info);
	pqi_ctrl_wait_until_quiesced(ctrl_info);
	pqi_fail_all_outstanding_requests(ctrl_info);
	pqi_clear_all_queued_raid_bypass_retries(ctrl_info);
	pqi_ctrl_unblock_requests(ctrl_info);
}

static void pqi_ctrl_offline_worker(struct work_struct *work)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = container_of(work, struct pqi_ctrl_info, ctrl_offline_work);
	pqi_take_ctrl_offline_deferred(ctrl_info);
}

static void pqi_take_ctrl_offline(struct pqi_ctrl_info *ctrl_info)
{
	if (!ctrl_info->controller_online)
		return;

	ctrl_info->controller_online = false;
	ctrl_info->pqi_mode_enabled = false;
	pqi_ctrl_block_requests(ctrl_info);
	if (!pqi_disable_ctrl_shutdown)
		sis_shutdown_ctrl(ctrl_info);
	pci_disable_device(ctrl_info->pci_dev);
	dev_err(&ctrl_info->pci_dev->dev, "controller offline\n");
	schedule_work(&ctrl_info->ctrl_offline_work);
}

static void pqi_print_ctrl_info(struct pci_dev *pci_dev,
	const struct pci_device_id *id)
{
	char *ctrl_description;

	if (id->driver_data)
		ctrl_description = (char *)id->driver_data;
	else
		ctrl_description = "Microsemi Smart Family Controller";

	dev_info(&pci_dev->dev, "%s found\n", ctrl_description);
}

static int pqi_pci_probe(struct pci_dev *pci_dev,
	const struct pci_device_id *id)
{
	int rc;
	int node;
	struct pqi_ctrl_info *ctrl_info;

	pqi_print_ctrl_info(pci_dev, id);

	if (pqi_disable_device_id_wildcards &&
		id->subvendor == PCI_ANY_ID &&
		id->subdevice == PCI_ANY_ID) {
		dev_warn(&pci_dev->dev,
			"controller not probed because device ID wildcards are disabled\n");
		return -ENODEV;
	}

	if (id->subvendor == PCI_ANY_ID || id->subdevice == PCI_ANY_ID)
		dev_warn(&pci_dev->dev,
			"controller device ID matched using wildcards\n");

	node = dev_to_node(&pci_dev->dev);
	if (node == NUMA_NO_NODE)
		set_dev_node(&pci_dev->dev, 0);

	ctrl_info = pqi_alloc_ctrl_info(node);
	if (!ctrl_info) {
		dev_err(&pci_dev->dev,
			"failed to allocate controller info block\n");
		return -ENOMEM;
	}

	ctrl_info->pci_dev = pci_dev;

	rc = pqi_pci_init(ctrl_info);
	if (rc)
		goto error;

	rc = pqi_ctrl_init(ctrl_info);
	if (rc)
		goto error;

	return 0;

error:
	pqi_remove_ctrl(ctrl_info);

	return rc;
}

static void pqi_pci_remove(struct pci_dev *pci_dev)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = pci_get_drvdata(pci_dev);
	if (!ctrl_info)
		return;

	pqi_remove_ctrl(ctrl_info);
}

static void pqi_shutdown(struct pci_dev *pci_dev)
{
	int rc;
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = pci_get_drvdata(pci_dev);
	if (!ctrl_info)
		goto error;

	/*
	 * Write all data in the controller's battery-backed cache to
	 * storage.
	 */
	rc = pqi_flush_cache(ctrl_info, SHUTDOWN);
	pqi_reset(ctrl_info);
	if (rc == 0)
		return;

error:
	dev_warn(&pci_dev->dev,
		"unable to flush controller cache\n");
}

static void pqi_process_lockup_action_param(void)
{
	unsigned int i;

	if (!pqi_lockup_action_param)
		return;

	for (i = 0; i < ARRAY_SIZE(pqi_lockup_actions); i++) {
		if (strcmp(pqi_lockup_action_param,
			pqi_lockup_actions[i].name) == 0) {
			pqi_lockup_action = pqi_lockup_actions[i].action;
			return;
		}
	}

	pr_warn("%s: invalid lockup action setting \"%s\" - supported settings: none, reboot, panic\n",
		DRIVER_NAME_SHORT, pqi_lockup_action_param);
}

static void pqi_process_module_params(void)
{
	pqi_process_lockup_action_param();
}

static __maybe_unused int pqi_suspend(struct pci_dev *pci_dev, pm_message_t state)
{
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = pci_get_drvdata(pci_dev);

	pqi_disable_events(ctrl_info);
	pqi_cancel_update_time_worker(ctrl_info);
	pqi_cancel_rescan_worker(ctrl_info);
	pqi_wait_until_scan_finished(ctrl_info);
	pqi_wait_until_lun_reset_finished(ctrl_info);
	pqi_flush_cache(ctrl_info, SUSPEND);
	pqi_ctrl_block_requests(ctrl_info);
	pqi_ctrl_wait_until_quiesced(ctrl_info);
	pqi_wait_until_inbound_queues_empty(ctrl_info);
	pqi_ctrl_wait_for_pending_io(ctrl_info);
	pqi_stop_heartbeat_timer(ctrl_info);

	if (state.event == PM_EVENT_FREEZE)
		return 0;

	pci_save_state(pci_dev);
	pci_set_power_state(pci_dev, pci_choose_state(pci_dev, state));

	ctrl_info->controller_online = false;
	ctrl_info->pqi_mode_enabled = false;

	return 0;
}

static __maybe_unused int pqi_resume(struct pci_dev *pci_dev)
{
	int rc;
	struct pqi_ctrl_info *ctrl_info;

	ctrl_info = pci_get_drvdata(pci_dev);

	if (pci_dev->current_state != PCI_D0) {
		ctrl_info->max_hw_queue_index = 0;
		pqi_free_interrupts(ctrl_info);
		pqi_change_irq_mode(ctrl_info, IRQ_MODE_INTX);
		rc = request_irq(pci_irq_vector(pci_dev, 0), pqi_irq_handler,
			IRQF_SHARED, DRIVER_NAME_SHORT,
			&ctrl_info->queue_groups[0]);
		if (rc) {
			dev_err(&ctrl_info->pci_dev->dev,
				"irq %u init failed with error %d\n",
				pci_dev->irq, rc);
			return rc;
		}
		pqi_start_heartbeat_timer(ctrl_info);
		pqi_ctrl_unblock_requests(ctrl_info);
		return 0;
	}

	pci_set_power_state(pci_dev, PCI_D0);
	pci_restore_state(pci_dev);

	return pqi_ctrl_init_resume(ctrl_info);
}

/* Define the PCI IDs for the controllers that we support. */
static const struct pci_device_id pqi_pci_id_table[] = {
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       0x152d, 0x8a22)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       0x152d, 0x8a23)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       0x152d, 0x8a24)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       0x152d, 0x8a36)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       0x152d, 0x8a37)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0110)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0608)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0800)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0801)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0802)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0803)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0804)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0805)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0806)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0807)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0900)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0901)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0902)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0903)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0904)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0905)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0906)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0907)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x0908)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x090a)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1200)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1201)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1202)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1280)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1281)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1300)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1301)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1302)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1303)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_ADAPTEC2, 0x1380)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_DELL, 0x1fe0)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0600)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0601)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0602)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0603)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0609)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0650)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0651)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0652)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0653)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0654)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0655)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0700)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x0701)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x1001)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x1100)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_VENDOR_ID_HP, 0x1101)
	},
	{
		PCI_DEVICE_SUB(PCI_VENDOR_ID_ADAPTEC2, 0x028f,
			       PCI_ANY_ID, PCI_ANY_ID)
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, pqi_pci_id_table);

static struct pci_driver pqi_pci_driver = {
	.name = DRIVER_NAME_SHORT,
	.id_table = pqi_pci_id_table,
	.probe = pqi_pci_probe,
	.remove = pqi_pci_remove,
	.shutdown = pqi_shutdown,
#if defined(CONFIG_PM)
	.suspend = pqi_suspend,
	.resume = pqi_resume,
#endif
};

static int __init pqi_init(void)
{
	int rc;

	pr_info(DRIVER_NAME "\n");

	pqi_sas_transport_template =
		sas_attach_transport(&pqi_sas_transport_functions);
	if (!pqi_sas_transport_template)
		return -ENODEV;

	pqi_process_module_params();

	rc = pci_register_driver(&pqi_pci_driver);
	if (rc)
		sas_release_transport(pqi_sas_transport_template);

	return rc;
}

static void __exit pqi_cleanup(void)
{
	pci_unregister_driver(&pqi_pci_driver);
	sas_release_transport(pqi_sas_transport_template);
}

module_init(pqi_init);
module_exit(pqi_cleanup);

static void __attribute__((unused)) verify_structures(void)
{
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_host_to_ctrl_doorbell) != 0x20);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_interrupt_mask) != 0x34);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_ctrl_to_host_doorbell) != 0x9c);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_ctrl_to_host_doorbell_clear) != 0xa0);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_driver_scratch) != 0xb0);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_firmware_status) != 0xbc);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		sis_mailbox) != 0x1000);
	BUILD_BUG_ON(offsetof(struct pqi_ctrl_registers,
		pqi_registers) != 0x4000);

	BUILD_BUG_ON(offsetof(struct pqi_iu_header,
		iu_type) != 0x0);
	BUILD_BUG_ON(offsetof(struct pqi_iu_header,
		iu_length) != 0x2);
	BUILD_BUG_ON(offsetof(struct pqi_iu_header,
		response_queue_id) != 0x4);
	BUILD_BUG_ON(offsetof(struct pqi_iu_header,
		work_area) != 0x6);
	BUILD_BUG_ON(sizeof(struct pqi_iu_header) != 0x8);

	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		status) != 0x0);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		service_response) != 0x1);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		data_present) != 0x2);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		reserved) != 0x3);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		residual_count) != 0x4);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		data_length) != 0x8);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		reserved1) != 0xa);
	BUILD_BUG_ON(offsetof(struct pqi_aio_error_info,
		data) != 0xc);
	BUILD_BUG_ON(sizeof(struct pqi_aio_error_info) != 0x10c);

	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		data_in_result) != 0x0);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		data_out_result) != 0x1);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		reserved) != 0x2);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		status) != 0x5);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		status_qualifier) != 0x6);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		sense_data_length) != 0x8);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		response_data_length) != 0xa);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		data_in_transferred) != 0xc);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		data_out_transferred) != 0x10);
	BUILD_BUG_ON(offsetof(struct pqi_raid_error_info,
		data) != 0x14);
	BUILD_BUG_ON(sizeof(struct pqi_raid_error_info) != 0x114);

	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		signature) != 0x0);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		function_and_status_code) != 0x8);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		max_admin_iq_elements) != 0x10);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		max_admin_oq_elements) != 0x11);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_iq_element_length) != 0x12);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_oq_element_length) != 0x13);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		max_reset_timeout) != 0x14);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		legacy_intx_status) != 0x18);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		legacy_intx_mask_set) != 0x1c);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		legacy_intx_mask_clear) != 0x20);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		device_status) != 0x40);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_iq_pi_offset) != 0x48);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_oq_ci_offset) != 0x50);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_iq_element_array_addr) != 0x58);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_oq_element_array_addr) != 0x60);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_iq_ci_addr) != 0x68);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_oq_pi_addr) != 0x70);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_iq_num_elements) != 0x78);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_oq_num_elements) != 0x79);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		admin_queue_int_msg_num) != 0x7a);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		device_error) != 0x80);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		error_details) != 0x88);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		device_reset) != 0x90);
	BUILD_BUG_ON(offsetof(struct pqi_device_registers,
		power_action) != 0x94);
	BUILD_BUG_ON(sizeof(struct pqi_device_registers) != 0x100);

	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		header.work_area) != 6);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		function_code) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.report_device_capability.buffer_length) != 44);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.report_device_capability.sg_descriptor) != 48);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.queue_id) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.element_array_addr) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.ci_addr) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.num_elements) != 32);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.element_length) != 34);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_iq.queue_protocol) != 36);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.queue_id) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.element_array_addr) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.pi_addr) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.num_elements) != 32);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.element_length) != 34);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.queue_protocol) != 36);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.int_msg_num) != 40);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.coalescing_count) != 42);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.min_coalescing_time) != 44);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.create_operational_oq.max_coalescing_time) != 48);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_request,
		data.delete_operational_queue.queue_id) != 12);
	BUILD_BUG_ON(sizeof(struct pqi_general_admin_request) != 64);
	BUILD_BUG_ON(FIELD_SIZEOF(struct pqi_general_admin_request,
		data.create_operational_iq) != 64 - 11);
	BUILD_BUG_ON(FIELD_SIZEOF(struct pqi_general_admin_request,
		data.create_operational_oq) != 64 - 11);
	BUILD_BUG_ON(FIELD_SIZEOF(struct pqi_general_admin_request,
		data.delete_operational_queue) != 64 - 11);

	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		header.work_area) != 6);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		function_code) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		status) != 11);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		data.create_operational_iq.status_descriptor) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		data.create_operational_iq.iq_pi_offset) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		data.create_operational_oq.status_descriptor) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_admin_response,
		data.create_operational_oq.oq_ci_offset) != 16);
	BUILD_BUG_ON(sizeof(struct pqi_general_admin_response) != 64);

	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		header.response_queue_id) != 4);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		header.work_area) != 6);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		nexus_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		buffer_length) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		lun_number) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		protocol_specific) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		error_index) != 27);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		cdb) != 32);
	BUILD_BUG_ON(offsetof(struct pqi_raid_path_request,
		sg_descriptors) != 64);
	BUILD_BUG_ON(sizeof(struct pqi_raid_path_request) !=
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		header.response_queue_id) != 4);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		header.work_area) != 6);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		nexus_id) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		buffer_length) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		data_encryption_key_index) != 22);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		encrypt_tweak_lower) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		encrypt_tweak_upper) != 28);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		cdb) != 32);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		error_index) != 48);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		num_sg_descriptors) != 50);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		cdb_length) != 51);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		lun_number) != 52);
	BUILD_BUG_ON(offsetof(struct pqi_aio_path_request,
		sg_descriptors) != 64);
	BUILD_BUG_ON(sizeof(struct pqi_aio_path_request) !=
		PQI_OPERATIONAL_IQ_ELEMENT_LENGTH);

	BUILD_BUG_ON(offsetof(struct pqi_io_response,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_io_response,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_io_response,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_io_response,
		error_index) != 10);

	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		header.response_queue_id) != 4);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		data.report_event_configuration.buffer_length) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		data.report_event_configuration.sg_descriptors) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		data.set_event_configuration.global_event_oq_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		data.set_event_configuration.buffer_length) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_general_management_request,
		data.set_event_configuration.sg_descriptors) != 16);

	BUILD_BUG_ON(offsetof(struct pqi_iu_layer_descriptor,
		max_inbound_iu_length) != 6);
	BUILD_BUG_ON(offsetof(struct pqi_iu_layer_descriptor,
		max_outbound_iu_length) != 14);
	BUILD_BUG_ON(sizeof(struct pqi_iu_layer_descriptor) != 16);

	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		data_length) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		iq_arbitration_priority_support_bitmask) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		maximum_aw_a) != 9);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		maximum_aw_b) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		maximum_aw_c) != 11);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_inbound_queues) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_elements_per_iq) != 18);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_iq_element_length) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		min_iq_element_length) != 26);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_outbound_queues) != 30);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_elements_per_oq) != 32);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		intr_coalescing_time_granularity) != 34);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		max_oq_element_length) != 36);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		min_oq_element_length) != 38);
	BUILD_BUG_ON(offsetof(struct pqi_device_capability,
		iu_layer_descriptors) != 64);
	BUILD_BUG_ON(sizeof(struct pqi_device_capability) != 576);

	BUILD_BUG_ON(offsetof(struct pqi_event_descriptor,
		event_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_event_descriptor,
		oq_id) != 2);
	BUILD_BUG_ON(sizeof(struct pqi_event_descriptor) != 4);

	BUILD_BUG_ON(offsetof(struct pqi_event_config,
		num_event_descriptors) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_event_config,
		descriptors) != 4);

	BUILD_BUG_ON(PQI_NUM_SUPPORTED_EVENTS !=
		ARRAY_SIZE(pqi_supported_event_types));

	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		event_type) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		event_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		additional_event_id) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_event_response,
		data) != 16);
	BUILD_BUG_ON(sizeof(struct pqi_event_response) != 32);

	BUILD_BUG_ON(offsetof(struct pqi_event_acknowledge_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_event_acknowledge_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_event_acknowledge_request,
		event_type) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_event_acknowledge_request,
		event_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_event_acknowledge_request,
		additional_event_id) != 12);
	BUILD_BUG_ON(sizeof(struct pqi_event_acknowledge_request) != 16);

	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		nexus_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		lun_number) != 16);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		protocol_specific) != 24);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		outbound_queue_id_to_manage) != 26);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		request_id_to_manage) != 28);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_request,
		task_management_function) != 30);
	BUILD_BUG_ON(sizeof(struct pqi_task_management_request) != 32);

	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		header.iu_type) != 0);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		header.iu_length) != 2);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		request_id) != 8);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		nexus_id) != 10);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		additional_response_info) != 12);
	BUILD_BUG_ON(offsetof(struct pqi_task_management_response,
		response_code) != 15);
	BUILD_BUG_ON(sizeof(struct pqi_task_management_response) != 16);

	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		configured_logical_drive_count) != 0);
	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		configuration_signature) != 1);
	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		firmware_version) != 5);
	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		extended_logical_unit_count) != 154);
	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		firmware_build_number) != 190);
	BUILD_BUG_ON(offsetof(struct bmic_identify_controller,
		controller_mode) != 292);

	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		phys_bay_in_box) != 115);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		device_type) != 120);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		redundant_path_present_map) != 1736);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		active_path_number) != 1738);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		alternate_paths_phys_connector) != 1739);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		alternate_paths_phys_box_on_port) != 1755);
	BUILD_BUG_ON(offsetof(struct bmic_identify_physical_device,
		current_queue_depth_limit) != 1796);
	BUILD_BUG_ON(sizeof(struct bmic_identify_physical_device) != 2560);

	BUILD_BUG_ON(PQI_ADMIN_IQ_NUM_ELEMENTS > 255);
	BUILD_BUG_ON(PQI_ADMIN_OQ_NUM_ELEMENTS > 255);
	BUILD_BUG_ON(PQI_ADMIN_IQ_ELEMENT_LENGTH %
		PQI_QUEUE_ELEMENT_LENGTH_ALIGNMENT != 0);
	BUILD_BUG_ON(PQI_ADMIN_OQ_ELEMENT_LENGTH %
		PQI_QUEUE_ELEMENT_LENGTH_ALIGNMENT != 0);
	BUILD_BUG_ON(PQI_OPERATIONAL_IQ_ELEMENT_LENGTH > 1048560);
	BUILD_BUG_ON(PQI_OPERATIONAL_IQ_ELEMENT_LENGTH %
		PQI_QUEUE_ELEMENT_LENGTH_ALIGNMENT != 0);
	BUILD_BUG_ON(PQI_OPERATIONAL_OQ_ELEMENT_LENGTH > 1048560);
	BUILD_BUG_ON(PQI_OPERATIONAL_OQ_ELEMENT_LENGTH %
		PQI_QUEUE_ELEMENT_LENGTH_ALIGNMENT != 0);

	BUILD_BUG_ON(PQI_RESERVED_IO_SLOTS >= PQI_MAX_OUTSTANDING_REQUESTS);
	BUILD_BUG_ON(PQI_RESERVED_IO_SLOTS >=
		PQI_MAX_OUTSTANDING_REQUESTS_KDUMP);
}
