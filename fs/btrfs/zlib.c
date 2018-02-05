/*
 * Copyright (C) 2008 Oracle.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 021110-1307, USA.
 *
 * Based on jffs2 zlib code:
 * Copyright © 2001-2007 Red Hat, Inc.
 * Created by David Woodhouse <dwmw2@infradead.org>
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/zlib.h>
#include <linux/zutil.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/bio.h>
#include <linux/refcount.h>
#include "compression.h"

struct workspace {
	z_stream strm;
	char *buf;
	struct list_head list;
	int level;
};

static void zlib_free_workspace(struct list_head *ws)
{
	struct workspace *workspace = list_entry(ws, struct workspace, list);

	kvfree(workspace->strm.workspace);
	kfree(workspace->buf);
	kfree(workspace);
}

static struct list_head *zlib_alloc_workspace(void)
{
	struct workspace *workspace;
	int workspacesize;

	workspace = kzalloc(sizeof(*workspace), GFP_KERNEL);
	if (!workspace)
		return ERR_PTR(-ENOMEM);

	workspacesize = max(zlib_deflate_workspacesize(MAX_WBITS, MAX_MEM_LEVEL),
			zlib_inflate_workspacesize());
	workspace->strm.workspace = kvmalloc(workspacesize, GFP_KERNEL);
	workspace->buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!workspace->strm.workspace || !workspace->buf)
		goto fail;

	INIT_LIST_HEAD(&workspace->list);

	return &workspace->list;
fail:
	zlib_free_workspace(&workspace->list);
	return ERR_PTR(-ENOMEM);
}

static int zlib_compress_pages(struct list_head *ws,
			       struct address_space *mapping,
			       u64 start,
			       struct page **pages,
			       unsigned long *out_pages,
			       unsigned long *total_in,
			       unsigned long *total_out)
{
	struct workspace *workspace = list_entry(ws, struct workspace, list);
	int ret;
	char *data_in;
	char *cpage_out;
	int nr_pages = 0;
	struct page *in_page = NULL;
	struct page *out_page = NULL;
	unsigned long bytes_left;
	unsigned long len = *total_out;
	unsigned long nr_dest_pages = *out_pages;
	const unsigned long max_out = nr_dest_pages * PAGE_SIZE;

	*out_pages = 0;
	*total_out = 0;
	*total_in = 0;

	if (Z_OK != zlib_deflateInit(&workspace->strm, workspace->level)) {
		pr_warn("BTRFS: deflateInit failed\n");
		ret = -EIO;
		goto out;
	}

	workspace->strm.total_in = 0;
	workspace->strm.total_out = 0;

	in_page = find_get_page(mapping, start >> PAGE_SHIFT);
	data_in = kmap(in_page);

	out_page = alloc_page(GFP_NOFS | __GFP_HIGHMEM);
	if (out_page == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	cpage_out = kmap(out_page);
	pages[0] = out_page;
	nr_pages = 1;

	workspace->strm.next_in = data_in;
	workspace->strm.next_out = cpage_out;
	workspace->strm.avail_out = PAGE_SIZE;
	workspace->strm.avail_in = min(len, PAGE_SIZE);

	while (workspace->strm.total_in < len) {
		ret = zlib_deflate(&workspace->strm, Z_SYNC_FLUSH);
		if (ret != Z_OK) {
			pr_debug("BTRFS: deflate in loop returned %d\n",
			       ret);
			zlib_deflateEnd(&workspace->strm);
			ret = -EIO;
			goto out;
		}

		/* we're making it bigger, give up */
		if (workspace->strm.total_in > 8192 &&
		    workspace->strm.total_in <
		    workspace->strm.total_out) {
			ret = -E2BIG;
			goto out;
		}
		/* we need another page for writing out.  Test this
		 * before the total_in so we will pull in a new page for
		 * the stream end if required
		 */
		if (workspace->strm.avail_out == 0) {
			kunmap(out_page);
			if (nr_pages == nr_dest_pages) {
				out_page = NULL;
				ret = -E2BIG;
				goto out;
			}
			out_page = alloc_page(GFP_NOFS | __GFP_HIGHMEM);
			if (out_page == NULL) {
				ret = -ENOMEM;
				goto out;
			}
			cpage_out = kmap(out_page);
			pages[nr_pages] = out_page;
			nr_pages++;
			workspace->strm.avail_out = PAGE_SIZE;
			workspace->strm.next_out = cpage_out;
		}
		/* we're all done */
		if (workspace->strm.total_in >= len)
			break;

		/* we've read in a full page, get a new one */
		if (workspace->strm.avail_in == 0) {
			if (workspace->strm.total_out > max_out)
				break;

			bytes_left = len - workspace->strm.total_in;
			kunmap(in_page);
			put_page(in_page);

			start += PAGE_SIZE;
			in_page = find_get_page(mapping,
						start >> PAGE_SHIFT);
			data_in = kmap(in_page);
			workspace->strm.avail_in = min(bytes_left,
							   PAGE_SIZE);
			workspace->strm.next_in = data_in;
		}
	}
	workspace->strm.avail_in = 0;
	ret = zlib_deflate(&workspace->strm, Z_FINISH);
	zlib_deflateEnd(&workspace->strm);

	if (ret != Z_STREAM_END) {
		ret = -EIO;
		goto out;
	}

	if (workspace->strm.total_out >= workspace->strm.total_in) {
		ret = -E2BIG;
		goto out;
	}

	ret = 0;
	*total_out = workspace->strm.total_out;
	*total_in = workspace->strm.total_in;
out:
	*out_pages = nr_pages;
	if (out_page)
		kunmap(out_page);

	if (in_page) {
		kunmap(in_page);
		put_page(in_page);
	}
	return ret;
}

static int zlib_decompress_bio(struct list_head *ws, struct compressed_bio *cb)
{
	struct workspace *workspace = list_entry(ws, struct workspace, list);
	int ret = 0, ret2;
	int wbits = MAX_WBITS;
	char *data_in;
	size_t total_out = 0;
	unsigned long page_in_index = 0;
	size_t srclen = cb->compressed_len;
	unsigned long total_pages_in = DIV_ROUND_UP(srclen, PAGE_SIZE);
	unsigned long buf_start;
	struct page **pages_in = cb->compressed_pages;
	u64 disk_start = cb->start;
	struct bio *orig_bio = cb->orig_bio;

	data_in = kmap(pages_in[page_in_index]);
	workspace->strm.next_in = data_in;
	workspace->strm.avail_in = min_t(size_t, srclen, PAGE_SIZE);
	workspace->strm.total_in = 0;

	workspace->strm.total_out = 0;
	workspace->strm.next_out = workspace->buf;
	workspace->strm.avail_out = PAGE_SIZE;

	/* If it's deflate, and it's got no preset dictionary, then
	   we can tell zlib to skip the adler32 check. */
	if (srclen > 2 && !(data_in[1] & PRESET_DICT) &&
	    ((data_in[0] & 0x0f) == Z_DEFLATED) &&
	    !(((data_in[0]<<8) + data_in[1]) % 31)) {

		wbits = -((data_in[0] >> 4) + 8);
		workspace->strm.next_in += 2;
		workspace->strm.avail_in -= 2;
	}

	if (Z_OK != zlib_inflateInit2(&workspace->strm, wbits)) {
		pr_warn("BTRFS: inflateInit failed\n");
		kunmap(pages_in[page_in_index]);
		return -EIO;
	}
	while (workspace->strm.total_in < srclen) {
		ret = zlib_inflate(&workspace->strm, Z_NO_FLUSH);
		if (ret != Z_OK && ret != Z_STREAM_END)
			break;

		buf_start = total_out;
		total_out = workspace->strm.total_out;

		/* we didn't make progress in this inflate call, we're done */
		if (buf_start == total_out)
			break;

		ret2 = btrfs_decompress_buf2page(workspace->buf, buf_start,
						 total_out, disk_start,
						 orig_bio);
		if (ret2 == 0) {
			ret = 0;
			goto done;
		}

		workspace->strm.next_out = workspace->buf;
		workspace->strm.avail_out = PAGE_SIZE;

		if (workspace->strm.avail_in == 0) {
			unsigned long tmp;
			kunmap(pages_in[page_in_index]);
			page_in_index++;
			if (page_in_index >= total_pages_in) {
				data_in = NULL;
				break;
			}
			data_in = kmap(pages_in[page_in_index]);
			workspace->strm.next_in = data_in;
			tmp = srclen - workspace->strm.total_in;
			workspace->strm.avail_in = min(tmp,
							   PAGE_SIZE);
		}
	}
	if (ret != Z_STREAM_END)
		ret = -EIO;
	else
		ret = 0;
done:
	zlib_inflateEnd(&workspace->strm);
	if (data_in)
		kunmap(pages_in[page_in_index]);
	if (!ret)
		zero_fill_bio(orig_bio);
	return ret;
}

static int zlib_decompress(struct list_head *ws, unsigned char *data_in,
			   struct page *dest_page,
			   unsigned long start_byte,
			   size_t srclen, size_t destlen)
{
	struct workspace *workspace = list_entry(ws, struct workspace, list);
	int ret = 0;
	int wbits = MAX_WBITS;
	unsigned long bytes_left;
	unsigned long total_out = 0;
	unsigned long pg_offset = 0;
	char *kaddr;

	destlen = min_t(unsigned long, destlen, PAGE_SIZE);
	bytes_left = destlen;

	workspace->strm.next_in = data_in;
	workspace->strm.avail_in = srclen;
	workspace->strm.total_in = 0;

	workspace->strm.next_out = workspace->buf;
	workspace->strm.avail_out = PAGE_SIZE;
	workspace->strm.total_out = 0;
	/* If it's deflate, and it's got no preset dictionary, then
	   we can tell zlib to skip the adler32 check. */
	if (srclen > 2 && !(data_in[1] & PRESET_DICT) &&
	    ((data_in[0] & 0x0f) == Z_DEFLATED) &&
	    !(((data_in[0]<<8) + data_in[1]) % 31)) {

		wbits = -((data_in[0] >> 4) + 8);
		workspace->strm.next_in += 2;
		workspace->strm.avail_in -= 2;
	}

	if (Z_OK != zlib_inflateInit2(&workspace->strm, wbits)) {
		pr_warn("BTRFS: inflateInit failed\n");
		return -EIO;
	}

	while (bytes_left > 0) {
		unsigned long buf_start;
		unsigned long buf_offset;
		unsigned long bytes;

		ret = zlib_inflate(&workspace->strm, Z_NO_FLUSH);
		if (ret != Z_OK && ret != Z_STREAM_END)
			break;

		buf_start = total_out;
		total_out = workspace->strm.total_out;

		if (total_out == buf_start) {
			ret = -EIO;
			break;
		}

		if (total_out <= start_byte)
			goto next;

		if (total_out > start_byte && buf_start < start_byte)
			buf_offset = start_byte - buf_start;
		else
			buf_offset = 0;

		bytes = min(PAGE_SIZE - pg_offset,
			    PAGE_SIZE - buf_offset);
		bytes = min(bytes, bytes_left);

		kaddr = kmap_atomic(dest_page);
		memcpy(kaddr + pg_offset, workspace->buf + buf_offset, bytes);
		kunmap_atomic(kaddr);

		pg_offset += bytes;
		bytes_left -= bytes;
next:
		workspace->strm.next_out = workspace->buf;
		workspace->strm.avail_out = PAGE_SIZE;
	}

	if (ret != Z_STREAM_END && bytes_left != 0)
		ret = -EIO;
	else
		ret = 0;

	zlib_inflateEnd(&workspace->strm);

	/*
	 * this should only happen if zlib returned fewer bytes than we
	 * expected.  btrfs_get_block is responsible for zeroing from the
	 * end of the inline extent (destlen) to the end of the page
	 */
	if (pg_offset < destlen) {
		kaddr = kmap_atomic(dest_page);
		memset(kaddr + pg_offset, 0, destlen - pg_offset);
		kunmap_atomic(kaddr);
	}
	return ret;
}

static void zlib_set_level(struct list_head *ws, unsigned int type)
{
	struct workspace *workspace = list_entry(ws, struct workspace, list);
	unsigned level = (type & 0xF0) >> 4;

	if (level > 9)
		level = 9;

	workspace->level = level > 0 ? level : 3;
}

const struct btrfs_compress_op btrfs_zlib_compress = {
	.alloc_workspace	= zlib_alloc_workspace,
	.free_workspace		= zlib_free_workspace,
	.compress_pages		= zlib_compress_pages,
	.decompress_bio		= zlib_decompress_bio,
	.decompress		= zlib_decompress,
	.set_level              = zlib_set_level,
};
