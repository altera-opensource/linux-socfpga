// SPDX-License-Identifier: GPL-2.0+
// Copyright 2017 IBM Corp.
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>
#include <linux/uaccess.h>
#include <uapi/misc/ocxl.h>
#include "ocxl_internal.h"


#define OCXL_NUM_MINORS 256 /* Total to reserve */

static dev_t ocxl_dev;
static struct class *ocxl_class;
static struct mutex minors_idr_lock;
static struct idr minors_idr;

static struct ocxl_afu *find_and_get_afu(dev_t devno)
{
	struct ocxl_afu *afu;
	int afu_minor;

	afu_minor = MINOR(devno);
	/*
	 * We don't declare an RCU critical section here, as our AFU
	 * is protected by a reference counter on the device. By the time the
	 * minor number of a device is removed from the idr, the ref count of
	 * the device is already at 0, so no user API will access that AFU and
	 * this function can't return it.
	 */
	afu = idr_find(&minors_idr, afu_minor);
	if (afu)
		ocxl_afu_get(afu);
	return afu;
}

static int allocate_afu_minor(struct ocxl_afu *afu)
{
	int minor;

	mutex_lock(&minors_idr_lock);
	minor = idr_alloc(&minors_idr, afu, 0, OCXL_NUM_MINORS, GFP_KERNEL);
	mutex_unlock(&minors_idr_lock);
	return minor;
}

static void free_afu_minor(struct ocxl_afu *afu)
{
	mutex_lock(&minors_idr_lock);
	idr_remove(&minors_idr, MINOR(afu->dev.devt));
	mutex_unlock(&minors_idr_lock);
}

static int afu_open(struct inode *inode, struct file *file)
{
	struct ocxl_afu *afu;
	struct ocxl_context *ctx;
	int rc;

	pr_debug("%s for device %x\n", __func__, inode->i_rdev);

	afu = find_and_get_afu(inode->i_rdev);
	if (!afu)
		return -ENODEV;

	ctx = ocxl_context_alloc();
	if (!ctx) {
		rc = -ENOMEM;
		goto put_afu;
	}

	rc = ocxl_context_init(ctx, afu, inode->i_mapping);
	if (rc)
		goto put_afu;
	file->private_data = ctx;
	ocxl_afu_put(afu);
	return 0;

put_afu:
	ocxl_afu_put(afu);
	return rc;
}

static long afu_ioctl_attach(struct ocxl_context *ctx,
			struct ocxl_ioctl_attach __user *uarg)
{
	struct ocxl_ioctl_attach arg;
	u64 amr = 0;
	int rc;

	pr_debug("%s for context %d\n", __func__, ctx->pasid);

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	/* Make sure reserved fields are not set for forward compatibility */
	if (arg.reserved1 || arg.reserved2 || arg.reserved3)
		return -EINVAL;

	amr = arg.amr & mfspr(SPRN_UAMOR);
	rc = ocxl_context_attach(ctx, amr);
	return rc;
}

#define CMD_STR(x) (x == OCXL_IOCTL_ATTACH ? "ATTACH" :			\
			x == OCXL_IOCTL_IRQ_ALLOC ? "IRQ_ALLOC" :	\
			x == OCXL_IOCTL_IRQ_FREE ? "IRQ_FREE" :		\
			x == OCXL_IOCTL_IRQ_SET_FD ? "IRQ_SET_FD" :	\
			"UNKNOWN")

static long afu_ioctl(struct file *file, unsigned int cmd,
		unsigned long args)
{
	struct ocxl_context *ctx = file->private_data;
	struct ocxl_ioctl_irq_fd irq_fd;
	u64 irq_offset;
	long rc;

	pr_debug("%s for context %d, command %s\n", __func__, ctx->pasid,
		CMD_STR(cmd));

	if (ctx->status == CLOSED)
		return -EIO;

	switch (cmd) {
	case OCXL_IOCTL_ATTACH:
		rc = afu_ioctl_attach(ctx,
				(struct ocxl_ioctl_attach __user *) args);
		break;

	case OCXL_IOCTL_IRQ_ALLOC:
		rc = ocxl_afu_irq_alloc(ctx, &irq_offset);
		if (!rc) {
			rc = copy_to_user((u64 __user *) args, &irq_offset,
					sizeof(irq_offset));
			if (rc)
				ocxl_afu_irq_free(ctx, irq_offset);
		}
		break;

	case OCXL_IOCTL_IRQ_FREE:
		rc = copy_from_user(&irq_offset, (u64 __user *) args,
				sizeof(irq_offset));
		if (rc)
			return -EFAULT;
		rc = ocxl_afu_irq_free(ctx, irq_offset);
		break;

	case OCXL_IOCTL_IRQ_SET_FD:
		rc = copy_from_user(&irq_fd, (u64 __user *) args,
				sizeof(irq_fd));
		if (rc)
			return -EFAULT;
		if (irq_fd.reserved)
			return -EINVAL;
		rc = ocxl_afu_irq_set_fd(ctx, irq_fd.irq_offset,
					irq_fd.eventfd);
		break;

	default:
		rc = -EINVAL;
	}
	return rc;
}

static long afu_compat_ioctl(struct file *file, unsigned int cmd,
			unsigned long args)
{
	return afu_ioctl(file, cmd, args);
}

static int afu_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ocxl_context *ctx = file->private_data;

	pr_debug("%s for context %d\n", __func__, ctx->pasid);
	return ocxl_context_mmap(ctx, vma);
}

static bool has_xsl_error(struct ocxl_context *ctx)
{
	bool ret;

	mutex_lock(&ctx->xsl_error_lock);
	ret = !!ctx->xsl_error.addr;
	mutex_unlock(&ctx->xsl_error_lock);

	return ret;
}

/*
 * Are there any events pending on the AFU
 * ctx: The AFU context
 * Returns: true if there are events pending
 */
static bool afu_events_pending(struct ocxl_context *ctx)
{
	if (has_xsl_error(ctx))
		return true;
	return false;
}

static unsigned int afu_poll(struct file *file, struct poll_table_struct *wait)
{
	struct ocxl_context *ctx = file->private_data;
	unsigned int mask = 0;
	bool closed;

	pr_debug("%s for context %d\n", __func__, ctx->pasid);

	poll_wait(file, &ctx->events_wq, wait);

	mutex_lock(&ctx->status_mutex);
	closed = (ctx->status == CLOSED);
	mutex_unlock(&ctx->status_mutex);

	if (afu_events_pending(ctx))
		mask = POLLIN | POLLRDNORM;
	else if (closed)
		mask = POLLERR;

	return mask;
}

/*
 * Populate the supplied buffer with a single XSL error
 * ctx:	The AFU context to report the error from
 * header: the event header to populate
 * buf: The buffer to write the body into (should be at least
 *      AFU_EVENT_BODY_XSL_ERROR_SIZE)
 * Return: the amount of buffer that was populated
 */
static ssize_t append_xsl_error(struct ocxl_context *ctx,
				struct ocxl_kernel_event_header *header,
				char __user *buf)
{
	struct ocxl_kernel_event_xsl_fault_error body;

	memset(&body, 0, sizeof(body));

	mutex_lock(&ctx->xsl_error_lock);
	if (!ctx->xsl_error.addr) {
		mutex_unlock(&ctx->xsl_error_lock);
		return 0;
	}

	body.addr = ctx->xsl_error.addr;
	body.dsisr = ctx->xsl_error.dsisr;
	body.count = ctx->xsl_error.count;

	ctx->xsl_error.addr = 0;
	ctx->xsl_error.dsisr = 0;
	ctx->xsl_error.count = 0;

	mutex_unlock(&ctx->xsl_error_lock);

	header->type = OCXL_AFU_EVENT_XSL_FAULT_ERROR;

	if (copy_to_user(buf, &body, sizeof(body)))
		return -EFAULT;

	return sizeof(body);
}

#define AFU_EVENT_BODY_MAX_SIZE sizeof(struct ocxl_kernel_event_xsl_fault_error)

/*
 * Reports events on the AFU
 * Format:
 *	Header (struct ocxl_kernel_event_header)
 *	Body (struct ocxl_kernel_event_*)
 *	Header...
 */
static ssize_t afu_read(struct file *file, char __user *buf, size_t count,
			loff_t *off)
{
	struct ocxl_context *ctx = file->private_data;
	struct ocxl_kernel_event_header header;
	ssize_t rc;
	size_t used = 0;
	DEFINE_WAIT(event_wait);

	memset(&header, 0, sizeof(header));

	/* Require offset to be 0 */
	if (*off != 0)
		return -EINVAL;

	if (count < (sizeof(struct ocxl_kernel_event_header) +
			AFU_EVENT_BODY_MAX_SIZE))
		return -EINVAL;

	for (;;) {
		prepare_to_wait(&ctx->events_wq, &event_wait,
				TASK_INTERRUPTIBLE);

		if (afu_events_pending(ctx))
			break;

		if (ctx->status == CLOSED)
			break;

		if (file->f_flags & O_NONBLOCK) {
			finish_wait(&ctx->events_wq, &event_wait);
			return -EAGAIN;
		}

		if (signal_pending(current)) {
			finish_wait(&ctx->events_wq, &event_wait);
			return -ERESTARTSYS;
		}

		schedule();
	}

	finish_wait(&ctx->events_wq, &event_wait);

	if (has_xsl_error(ctx)) {
		used = append_xsl_error(ctx, &header, buf + sizeof(header));
		if (used < 0)
			return used;
	}

	if (!afu_events_pending(ctx))
		header.flags |= OCXL_KERNEL_EVENT_FLAG_LAST;

	if (copy_to_user(buf, &header, sizeof(header)))
		return -EFAULT;

	used += sizeof(header);

	rc = (ssize_t) used;
	return rc;
}

static int afu_release(struct inode *inode, struct file *file)
{
	struct ocxl_context *ctx = file->private_data;
	int rc;

	pr_debug("%s for device %x\n", __func__, inode->i_rdev);
	rc = ocxl_context_detach(ctx);
	mutex_lock(&ctx->mapping_lock);
	ctx->mapping = NULL;
	mutex_unlock(&ctx->mapping_lock);
	wake_up_all(&ctx->events_wq);
	if (rc != -EBUSY)
		ocxl_context_free(ctx);
	return 0;
}

static const struct file_operations ocxl_afu_fops = {
	.owner		= THIS_MODULE,
	.open           = afu_open,
	.unlocked_ioctl = afu_ioctl,
	.compat_ioctl   = afu_compat_ioctl,
	.mmap           = afu_mmap,
	.poll           = afu_poll,
	.read           = afu_read,
	.release        = afu_release,
};

int ocxl_create_cdev(struct ocxl_afu *afu)
{
	int rc;

	cdev_init(&afu->cdev, &ocxl_afu_fops);
	rc = cdev_add(&afu->cdev, afu->dev.devt, 1);
	if (rc) {
		dev_err(&afu->dev, "Unable to add afu char device: %d\n", rc);
		return rc;
	}
	return 0;
}

void ocxl_destroy_cdev(struct ocxl_afu *afu)
{
	cdev_del(&afu->cdev);
}

int ocxl_register_afu(struct ocxl_afu *afu)
{
	int minor;

	minor = allocate_afu_minor(afu);
	if (minor < 0)
		return minor;
	afu->dev.devt = MKDEV(MAJOR(ocxl_dev), minor);
	afu->dev.class = ocxl_class;
	return device_register(&afu->dev);
}

void ocxl_unregister_afu(struct ocxl_afu *afu)
{
	free_afu_minor(afu);
}

static char *ocxl_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "ocxl/%s", dev_name(dev));
}

int ocxl_file_init(void)
{
	int rc;

	mutex_init(&minors_idr_lock);
	idr_init(&minors_idr);

	rc = alloc_chrdev_region(&ocxl_dev, 0, OCXL_NUM_MINORS, "ocxl");
	if (rc) {
		pr_err("Unable to allocate ocxl major number: %d\n", rc);
		return rc;
	}

	ocxl_class = class_create(THIS_MODULE, "ocxl");
	if (IS_ERR(ocxl_class)) {
		pr_err("Unable to create ocxl class\n");
		unregister_chrdev_region(ocxl_dev, OCXL_NUM_MINORS);
		return PTR_ERR(ocxl_class);
	}

	ocxl_class->devnode = ocxl_devnode;
	return 0;
}

void ocxl_file_exit(void)
{
	class_destroy(ocxl_class);
	unregister_chrdev_region(ocxl_dev, OCXL_NUM_MINORS);
	idr_destroy(&minors_idr);
}
