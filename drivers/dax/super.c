/*
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/pagemap.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/magic.h>
#include <linux/genhd.h>
#include <linux/cdev.h>
#include <linux/hash.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/dax.h>
#include <linux/fs.h>

static dev_t dax_devt;
DEFINE_STATIC_SRCU(dax_srcu);
static struct vfsmount *dax_mnt;
static DEFINE_IDA(dax_minor_ida);
static struct kmem_cache *dax_cache __read_mostly;
static struct super_block *dax_superblock __read_mostly;

#define DAX_HASH_SIZE (PAGE_SIZE / sizeof(struct hlist_head))
static struct hlist_head dax_host_list[DAX_HASH_SIZE];
static DEFINE_SPINLOCK(dax_host_lock);

int dax_read_lock(void)
{
	return srcu_read_lock(&dax_srcu);
}
EXPORT_SYMBOL_GPL(dax_read_lock);

void dax_read_unlock(int id)
{
	srcu_read_unlock(&dax_srcu, id);
}
EXPORT_SYMBOL_GPL(dax_read_unlock);

#ifdef CONFIG_BLOCK
#include <linux/blkdev.h>

int bdev_dax_pgoff(struct block_device *bdev, sector_t sector, size_t size,
		pgoff_t *pgoff)
{
	phys_addr_t phys_off = (get_start_sect(bdev) + sector) * 512;

	if (pgoff)
		*pgoff = PHYS_PFN(phys_off);
	if (phys_off % PAGE_SIZE || size % PAGE_SIZE)
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL(bdev_dax_pgoff);

#if IS_ENABLED(CONFIG_FS_DAX)
struct dax_device *fs_dax_get_by_bdev(struct block_device *bdev)
{
	if (!blk_queue_dax(bdev->bd_queue))
		return NULL;
	return fs_dax_get_by_host(bdev->bd_disk->disk_name);
}
EXPORT_SYMBOL_GPL(fs_dax_get_by_bdev);
#endif

/**
 * __bdev_dax_supported() - Check if the device supports dax for filesystem
 * @sb: The superblock of the device
 * @blocksize: The block size of the device
 *
 * This is a library function for filesystems to check if the block device
 * can be mounted with dax option.
 *
 * Return: negative errno if unsupported, 0 if supported.
 */
int __bdev_dax_supported(struct super_block *sb, int blocksize)
{
	struct block_device *bdev = sb->s_bdev;
	struct dax_device *dax_dev;
	pgoff_t pgoff;
	int err, id;
	void *kaddr;
	pfn_t pfn;
	long len;

	if (blocksize != PAGE_SIZE) {
		pr_debug("VFS (%s): error: unsupported blocksize for dax\n",
				sb->s_id);
		return -EINVAL;
	}

	err = bdev_dax_pgoff(bdev, 0, PAGE_SIZE, &pgoff);
	if (err) {
		pr_debug("VFS (%s): error: unaligned partition for dax\n",
				sb->s_id);
		return err;
	}

	dax_dev = dax_get_by_host(bdev->bd_disk->disk_name);
	if (!dax_dev) {
		pr_debug("VFS (%s): error: device does not support dax\n",
				sb->s_id);
		return -EOPNOTSUPP;
	}

	id = dax_read_lock();
	len = dax_direct_access(dax_dev, pgoff, 1, &kaddr, &pfn);
	dax_read_unlock(id);

	put_dax(dax_dev);

	if (len < 1) {
		pr_debug("VFS (%s): error: dax access failed (%ld)\n",
				sb->s_id, len);
		return len < 0 ? len : -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__bdev_dax_supported);
#endif

enum dax_device_flags {
	/* !alive + rcu grace period == no new operations / mappings */
	DAXDEV_ALIVE,
	/* gate whether dax_flush() calls the low level flush routine */
	DAXDEV_WRITE_CACHE,
};

/**
 * struct dax_device - anchor object for dax services
 * @inode: core vfs
 * @cdev: optional character interface for "device dax"
 * @host: optional name for lookups where the device path is not available
 * @private: dax driver private data
 * @flags: state and boolean properties
 */
struct dax_device {
	struct hlist_node list;
	struct inode inode;
	struct cdev cdev;
	const char *host;
	void *private;
	unsigned long flags;
	const struct dax_operations *ops;
};

static ssize_t write_cache_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dax_device *dax_dev = dax_get_by_host(dev_name(dev));
	ssize_t rc;

	WARN_ON_ONCE(!dax_dev);
	if (!dax_dev)
		return -ENXIO;

	rc = sprintf(buf, "%d\n", !!test_bit(DAXDEV_WRITE_CACHE,
				&dax_dev->flags));
	put_dax(dax_dev);
	return rc;
}

static ssize_t write_cache_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	bool write_cache;
	int rc = strtobool(buf, &write_cache);
	struct dax_device *dax_dev = dax_get_by_host(dev_name(dev));

	WARN_ON_ONCE(!dax_dev);
	if (!dax_dev)
		return -ENXIO;

	if (rc)
		len = rc;
	else if (write_cache)
		set_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags);
	else
		clear_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags);

	put_dax(dax_dev);
	return len;
}
static DEVICE_ATTR_RW(write_cache);

static umode_t dax_visible(struct kobject *kobj, struct attribute *a, int n)
{
	struct device *dev = container_of(kobj, typeof(*dev), kobj);
	struct dax_device *dax_dev = dax_get_by_host(dev_name(dev));

	WARN_ON_ONCE(!dax_dev);
	if (!dax_dev)
		return 0;

#ifndef CONFIG_ARCH_HAS_PMEM_API
	if (a == &dev_attr_write_cache.attr)
		return 0;
#endif
	return a->mode;
}

static struct attribute *dax_attributes[] = {
	&dev_attr_write_cache.attr,
	NULL,
};

struct attribute_group dax_attribute_group = {
	.name = "dax",
	.attrs = dax_attributes,
	.is_visible = dax_visible,
};
EXPORT_SYMBOL_GPL(dax_attribute_group);

/**
 * dax_direct_access() - translate a device pgoff to an absolute pfn
 * @dax_dev: a dax_device instance representing the logical memory range
 * @pgoff: offset in pages from the start of the device to translate
 * @nr_pages: number of consecutive pages caller can handle relative to @pfn
 * @kaddr: output parameter that returns a virtual address mapping of pfn
 * @pfn: output parameter that returns an absolute pfn translation of @pgoff
 *
 * Return: negative errno if an error occurs, otherwise the number of
 * pages accessible at the device relative @pgoff.
 */
long dax_direct_access(struct dax_device *dax_dev, pgoff_t pgoff, long nr_pages,
		void **kaddr, pfn_t *pfn)
{
	long avail;

	/*
	 * The device driver is allowed to sleep, in order to make the
	 * memory directly accessible.
	 */
	might_sleep();

	if (!dax_dev)
		return -EOPNOTSUPP;

	if (!dax_alive(dax_dev))
		return -ENXIO;

	if (nr_pages < 0)
		return nr_pages;

	avail = dax_dev->ops->direct_access(dax_dev, pgoff, nr_pages,
			kaddr, pfn);
	if (!avail)
		return -ERANGE;
	return min(avail, nr_pages);
}
EXPORT_SYMBOL_GPL(dax_direct_access);

size_t dax_copy_from_iter(struct dax_device *dax_dev, pgoff_t pgoff, void *addr,
		size_t bytes, struct iov_iter *i)
{
	if (!dax_alive(dax_dev))
		return 0;

	return dax_dev->ops->copy_from_iter(dax_dev, pgoff, addr, bytes, i);
}
EXPORT_SYMBOL_GPL(dax_copy_from_iter);

#ifdef CONFIG_ARCH_HAS_PMEM_API
void arch_wb_cache_pmem(void *addr, size_t size);
void dax_flush(struct dax_device *dax_dev, void *addr, size_t size)
{
	if (unlikely(!test_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags)))
		return;

	arch_wb_cache_pmem(addr, size);
}
#else
void dax_flush(struct dax_device *dax_dev, void *addr, size_t size)
{
}
#endif
EXPORT_SYMBOL_GPL(dax_flush);

void dax_write_cache(struct dax_device *dax_dev, bool wc)
{
	if (wc)
		set_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags);
	else
		clear_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags);
}
EXPORT_SYMBOL_GPL(dax_write_cache);

bool dax_write_cache_enabled(struct dax_device *dax_dev)
{
	return test_bit(DAXDEV_WRITE_CACHE, &dax_dev->flags);
}
EXPORT_SYMBOL_GPL(dax_write_cache_enabled);

bool dax_alive(struct dax_device *dax_dev)
{
	lockdep_assert_held(&dax_srcu);
	return test_bit(DAXDEV_ALIVE, &dax_dev->flags);
}
EXPORT_SYMBOL_GPL(dax_alive);

static int dax_host_hash(const char *host)
{
	return hashlen_hash(hashlen_string("DAX", host)) % DAX_HASH_SIZE;
}

/*
 * Note, rcu is not protecting the liveness of dax_dev, rcu is ensuring
 * that any fault handlers or operations that might have seen
 * dax_alive(), have completed.  Any operations that start after
 * synchronize_srcu() has run will abort upon seeing !dax_alive().
 */
void kill_dax(struct dax_device *dax_dev)
{
	if (!dax_dev)
		return;

	clear_bit(DAXDEV_ALIVE, &dax_dev->flags);

	synchronize_srcu(&dax_srcu);

	spin_lock(&dax_host_lock);
	hlist_del_init(&dax_dev->list);
	spin_unlock(&dax_host_lock);

	dax_dev->private = NULL;
}
EXPORT_SYMBOL_GPL(kill_dax);

static struct inode *dax_alloc_inode(struct super_block *sb)
{
	struct dax_device *dax_dev;
	struct inode *inode;

	dax_dev = kmem_cache_alloc(dax_cache, GFP_KERNEL);
	if (!dax_dev)
		return NULL;

	inode = &dax_dev->inode;
	inode->i_rdev = 0;
	return inode;
}

static struct dax_device *to_dax_dev(struct inode *inode)
{
	return container_of(inode, struct dax_device, inode);
}

static void dax_i_callback(struct rcu_head *head)
{
	struct inode *inode = container_of(head, struct inode, i_rcu);
	struct dax_device *dax_dev = to_dax_dev(inode);

	kfree(dax_dev->host);
	dax_dev->host = NULL;
	if (inode->i_rdev)
		ida_simple_remove(&dax_minor_ida, MINOR(inode->i_rdev));
	kmem_cache_free(dax_cache, dax_dev);
}

static void dax_destroy_inode(struct inode *inode)
{
	struct dax_device *dax_dev = to_dax_dev(inode);

	WARN_ONCE(test_bit(DAXDEV_ALIVE, &dax_dev->flags),
			"kill_dax() must be called before final iput()\n");
	call_rcu(&inode->i_rcu, dax_i_callback);
}

static const struct super_operations dax_sops = {
	.statfs = simple_statfs,
	.alloc_inode = dax_alloc_inode,
	.destroy_inode = dax_destroy_inode,
	.drop_inode = generic_delete_inode,
};

static struct dentry *dax_mount(struct file_system_type *fs_type,
		int flags, const char *dev_name, void *data)
{
	return mount_pseudo(fs_type, "dax:", &dax_sops, NULL, DAXFS_MAGIC);
}

static struct file_system_type dax_fs_type = {
	.name = "dax",
	.mount = dax_mount,
	.kill_sb = kill_anon_super,
};

static int dax_test(struct inode *inode, void *data)
{
	dev_t devt = *(dev_t *) data;

	return inode->i_rdev == devt;
}

static int dax_set(struct inode *inode, void *data)
{
	dev_t devt = *(dev_t *) data;

	inode->i_rdev = devt;
	return 0;
}

static struct dax_device *dax_dev_get(dev_t devt)
{
	struct dax_device *dax_dev;
	struct inode *inode;

	inode = iget5_locked(dax_superblock, hash_32(devt + DAXFS_MAGIC, 31),
			dax_test, dax_set, &devt);

	if (!inode)
		return NULL;

	dax_dev = to_dax_dev(inode);
	if (inode->i_state & I_NEW) {
		set_bit(DAXDEV_ALIVE, &dax_dev->flags);
		inode->i_cdev = &dax_dev->cdev;
		inode->i_mode = S_IFCHR;
		inode->i_flags = S_DAX;
		mapping_set_gfp_mask(&inode->i_data, GFP_USER);
		unlock_new_inode(inode);
	}

	return dax_dev;
}

static void dax_add_host(struct dax_device *dax_dev, const char *host)
{
	int hash;

	/*
	 * Unconditionally init dax_dev since it's coming from a
	 * non-zeroed slab cache
	 */
	INIT_HLIST_NODE(&dax_dev->list);
	dax_dev->host = host;
	if (!host)
		return;

	hash = dax_host_hash(host);
	spin_lock(&dax_host_lock);
	hlist_add_head(&dax_dev->list, &dax_host_list[hash]);
	spin_unlock(&dax_host_lock);
}

struct dax_device *alloc_dax(void *private, const char *__host,
		const struct dax_operations *ops)
{
	struct dax_device *dax_dev;
	const char *host;
	dev_t devt;
	int minor;

	host = kstrdup(__host, GFP_KERNEL);
	if (__host && !host)
		return NULL;

	minor = ida_simple_get(&dax_minor_ida, 0, MINORMASK+1, GFP_KERNEL);
	if (minor < 0)
		goto err_minor;

	devt = MKDEV(MAJOR(dax_devt), minor);
	dax_dev = dax_dev_get(devt);
	if (!dax_dev)
		goto err_dev;

	dax_add_host(dax_dev, host);
	dax_dev->ops = ops;
	dax_dev->private = private;
	return dax_dev;

 err_dev:
	ida_simple_remove(&dax_minor_ida, minor);
 err_minor:
	kfree(host);
	return NULL;
}
EXPORT_SYMBOL_GPL(alloc_dax);

void put_dax(struct dax_device *dax_dev)
{
	if (!dax_dev)
		return;
	iput(&dax_dev->inode);
}
EXPORT_SYMBOL_GPL(put_dax);

/**
 * dax_get_by_host() - temporary lookup mechanism for filesystem-dax
 * @host: alternate name for the device registered by a dax driver
 */
struct dax_device *dax_get_by_host(const char *host)
{
	struct dax_device *dax_dev, *found = NULL;
	int hash, id;

	if (!host)
		return NULL;

	hash = dax_host_hash(host);

	id = dax_read_lock();
	spin_lock(&dax_host_lock);
	hlist_for_each_entry(dax_dev, &dax_host_list[hash], list) {
		if (!dax_alive(dax_dev)
				|| strcmp(host, dax_dev->host) != 0)
			continue;

		if (igrab(&dax_dev->inode))
			found = dax_dev;
		break;
	}
	spin_unlock(&dax_host_lock);
	dax_read_unlock(id);

	return found;
}
EXPORT_SYMBOL_GPL(dax_get_by_host);

/**
 * inode_dax: convert a public inode into its dax_dev
 * @inode: An inode with i_cdev pointing to a dax_dev
 *
 * Note this is not equivalent to to_dax_dev() which is for private
 * internal use where we know the inode filesystem type == dax_fs_type.
 */
struct dax_device *inode_dax(struct inode *inode)
{
	struct cdev *cdev = inode->i_cdev;

	return container_of(cdev, struct dax_device, cdev);
}
EXPORT_SYMBOL_GPL(inode_dax);

struct inode *dax_inode(struct dax_device *dax_dev)
{
	return &dax_dev->inode;
}
EXPORT_SYMBOL_GPL(dax_inode);

void *dax_get_private(struct dax_device *dax_dev)
{
	return dax_dev->private;
}
EXPORT_SYMBOL_GPL(dax_get_private);

static void init_once(void *_dax_dev)
{
	struct dax_device *dax_dev = _dax_dev;
	struct inode *inode = &dax_dev->inode;

	memset(dax_dev, 0, sizeof(*dax_dev));
	inode_init_once(inode);
}

static int __dax_fs_init(void)
{
	int rc;

	dax_cache = kmem_cache_create("dax_cache", sizeof(struct dax_device), 0,
			(SLAB_HWCACHE_ALIGN|SLAB_RECLAIM_ACCOUNT|
			 SLAB_MEM_SPREAD|SLAB_ACCOUNT),
			init_once);
	if (!dax_cache)
		return -ENOMEM;

	rc = register_filesystem(&dax_fs_type);
	if (rc)
		goto err_register_fs;

	dax_mnt = kern_mount(&dax_fs_type);
	if (IS_ERR(dax_mnt)) {
		rc = PTR_ERR(dax_mnt);
		goto err_mount;
	}
	dax_superblock = dax_mnt->mnt_sb;

	return 0;

 err_mount:
	unregister_filesystem(&dax_fs_type);
 err_register_fs:
	kmem_cache_destroy(dax_cache);

	return rc;
}

static void __dax_fs_exit(void)
{
	kern_unmount(dax_mnt);
	unregister_filesystem(&dax_fs_type);
	kmem_cache_destroy(dax_cache);
}

static int __init dax_fs_init(void)
{
	int rc;

	rc = __dax_fs_init();
	if (rc)
		return rc;

	rc = alloc_chrdev_region(&dax_devt, 0, MINORMASK+1, "dax");
	if (rc)
		__dax_fs_exit();
	return rc;
}

static void __exit dax_fs_exit(void)
{
	unregister_chrdev_region(dax_devt, MINORMASK+1);
	ida_destroy(&dax_minor_ida);
	__dax_fs_exit();
}

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
subsys_initcall(dax_fs_init);
module_exit(dax_fs_exit);
