/*
 * Copyright (C) 2017 Oracle.  All rights reserved.
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
 */

#include <linux/types.h>
#include "btrfs-tests.h"
#include "../ctree.h"

static void free_extent_map_tree(struct extent_map_tree *em_tree)
{
	struct extent_map *em;
	struct rb_node *node;

	while (!RB_EMPTY_ROOT(&em_tree->map)) {
		node = rb_first(&em_tree->map);
		em = rb_entry(node, struct extent_map, rb_node);
		remove_extent_mapping(em_tree, em);

#ifdef CONFIG_BTRFS_DEBUG
		if (refcount_read(&em->refs) != 1) {
			test_msg(
"em leak: em (start 0x%llx len 0x%llx block_start 0x%llx block_len 0x%llx) refs %d\n",
				 em->start, em->len, em->block_start,
				 em->block_len, refcount_read(&em->refs));

			refcount_set(&em->refs, 1);
		}
#endif
		free_extent_map(em);
	}
}

/*
 * Test scenario:
 *
 * Suppose that no extent map has been loaded into memory yet, there is a file
 * extent [0, 16K), followed by another file extent [16K, 20K), two dio reads
 * are entering btrfs_get_extent() concurrently, t1 is reading [8K, 16K), t2 is
 * reading [0, 8K)
 *
 *     t1                            t2
 *  btrfs_get_extent()              btrfs_get_extent()
 *    -> lookup_extent_mapping()      ->lookup_extent_mapping()
 *    -> add_extent_mapping(0, 16K)
 *    -> return em
 *                                    ->add_extent_mapping(0, 16K)
 *                                    -> #handle -EEXIST
 */
static void test_case_1(struct extent_map_tree *em_tree)
{
	struct extent_map *em;
	u64 start = 0;
	u64 len = SZ_8K;
	int ret;

	em = alloc_extent_map();
	if (!em)
		/* Skip the test on error. */
		return;

	/* Add [0, 16K) */
	em->start = 0;
	em->len = SZ_16K;
	em->block_start = 0;
	em->block_len = SZ_16K;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	/* Add [16K, 20K) following [0, 16K)  */
	em = alloc_extent_map();
	if (!em)
		goto out;

	em->start = SZ_16K;
	em->len = SZ_4K;
	em->block_start = SZ_32K; /* avoid merging */
	em->block_len = SZ_4K;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	em = alloc_extent_map();
	if (!em)
		goto out;

	/* Add [0, 8K), should return [0, 16K) instead. */
	em->start = start;
	em->len = len;
	em->block_start = start;
	em->block_len = len;
	ret = btrfs_add_extent_mapping(em_tree, &em, em->start, em->len);
	if (ret)
		test_msg("case1 [%llu %llu]: ret %d\n", start, start + len, ret);
	if (em &&
	    (em->start != 0 || extent_map_end(em) != SZ_16K ||
	     em->block_start != 0 || em->block_len != SZ_16K))
		test_msg(
"case1 [%llu %llu]: ret %d return a wrong em (start %llu len %llu block_start %llu block_len %llu\n",
			 start, start + len, ret, em->start, em->len,
			 em->block_start, em->block_len);
	free_extent_map(em);
out:
	/* free memory */
	free_extent_map_tree(em_tree);
}

/*
 * Test scenario:
 *
 * Reading the inline ending up with EEXIST, ie. read an inline
 * extent and discard page cache and read it again.
 */
static void test_case_2(struct extent_map_tree *em_tree)
{
	struct extent_map *em;
	int ret;

	em = alloc_extent_map();
	if (!em)
		/* Skip the test on error. */
		return;

	/* Add [0, 1K) */
	em->start = 0;
	em->len = SZ_1K;
	em->block_start = EXTENT_MAP_INLINE;
	em->block_len = (u64)-1;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	/* Add [4K, 4K) following [0, 1K)  */
	em = alloc_extent_map();
	if (!em)
		goto out;

	em->start = SZ_4K;
	em->len = SZ_4K;
	em->block_start = SZ_4K;
	em->block_len = SZ_4K;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	em = alloc_extent_map();
	if (!em)
		goto out;

	/* Add [0, 1K) */
	em->start = 0;
	em->len = SZ_1K;
	em->block_start = EXTENT_MAP_INLINE;
	em->block_len = (u64)-1;
	ret = btrfs_add_extent_mapping(em_tree, &em, em->start, em->len);
	if (ret)
		test_msg("case2 [0 1K]: ret %d\n", ret);
	if (em &&
	    (em->start != 0 || extent_map_end(em) != SZ_1K ||
	     em->block_start != EXTENT_MAP_INLINE || em->block_len != (u64)-1))
		test_msg(
"case2 [0 1K]: ret %d return a wrong em (start %llu len %llu block_start %llu block_len %llu\n",
			 ret, em->start, em->len, em->block_start,
			 em->block_len);
	free_extent_map(em);
out:
	/* free memory */
	free_extent_map_tree(em_tree);
}

static void __test_case_3(struct extent_map_tree *em_tree, u64 start)
{
	struct extent_map *em;
	u64 len = SZ_4K;
	int ret;

	em = alloc_extent_map();
	if (!em)
		/* Skip this test on error. */
		return;

	/* Add [4K, 8K) */
	em->start = SZ_4K;
	em->len = SZ_4K;
	em->block_start = SZ_4K;
	em->block_len = SZ_4K;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	em = alloc_extent_map();
	if (!em)
		goto out;

	/* Add [0, 16K) */
	em->start = 0;
	em->len = SZ_16K;
	em->block_start = 0;
	em->block_len = SZ_16K;
	ret = btrfs_add_extent_mapping(em_tree, &em, start, len);
	if (ret)
		test_msg("case3 [0x%llx 0x%llx): ret %d\n",
			 start, start + len, ret);
	/*
	 * Since bytes within em are contiguous, em->block_start is identical to
	 * em->start.
	 */
	if (em &&
	    (start < em->start || start + len > extent_map_end(em) ||
	     em->start != em->block_start || em->len != em->block_len))
		test_msg(
"case3 [0x%llx 0x%llx): ret %d em (start 0x%llx len 0x%llx block_start 0x%llx block_len 0x%llx)\n",
			 start, start + len, ret, em->start, em->len,
			 em->block_start, em->block_len);
	free_extent_map(em);
out:
	/* free memory */
	free_extent_map_tree(em_tree);
}

/*
 * Test scenario:
 *
 * Suppose that no extent map has been loaded into memory yet.
 * There is a file extent [0, 16K), two jobs are running concurrently
 * against it, t1 is buffered writing to [4K, 8K) and t2 is doing dio
 * read from [0, 4K) or [8K, 12K) or [12K, 16K).
 *
 * t1 goes ahead of t2 and adds em [4K, 8K) into tree.
 *
 *         t1                       t2
 *  cow_file_range()	     btrfs_get_extent()
 *                            -> lookup_extent_mapping()
 *   -> add_extent_mapping()
 *                            -> add_extent_mapping()
 */
static void test_case_3(struct extent_map_tree *em_tree)
{
	__test_case_3(em_tree, 0);
	__test_case_3(em_tree, SZ_8K);
	__test_case_3(em_tree, (12 * 1024ULL));
}

static void __test_case_4(struct extent_map_tree *em_tree, u64 start)
{
	struct extent_map *em;
	u64 len = SZ_4K;
	int ret;

	em = alloc_extent_map();
	if (!em)
		/* Skip this test on error. */
		return;

	/* Add [0K, 8K) */
	em->start = 0;
	em->len = SZ_8K;
	em->block_start = 0;
	em->block_len = SZ_8K;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	em = alloc_extent_map();
	if (!em)
		goto out;

	/* Add [8K, 24K) */
	em->start = SZ_8K;
	em->len = 24 * 1024ULL;
	em->block_start = SZ_16K; /* avoid merging */
	em->block_len = 24 * 1024ULL;
	ret = add_extent_mapping(em_tree, em, 0);
	ASSERT(ret == 0);
	free_extent_map(em);

	em = alloc_extent_map();
	if (!em)
		goto out;
	/* Add [0K, 32K) */
	em->start = 0;
	em->len = SZ_32K;
	em->block_start = 0;
	em->block_len = SZ_32K;
	ret = btrfs_add_extent_mapping(em_tree, &em, start, len);
	if (ret)
		test_msg("case4 [0x%llx 0x%llx): ret %d\n",
			 start, len, ret);
	if (em &&
	    (start < em->start || start + len > extent_map_end(em)))
		test_msg(
"case4 [0x%llx 0x%llx): ret %d, added wrong em (start 0x%llx len 0x%llx block_start 0x%llx block_len 0x%llx)\n",
			 start, len, ret, em->start, em->len, em->block_start,
			 em->block_len);
	free_extent_map(em);
out:
	/* free memory */
	free_extent_map_tree(em_tree);
}

/*
 * Test scenario:
 *
 * Suppose that no extent map has been loaded into memory yet.
 * There is a file extent [0, 32K), two jobs are running concurrently
 * against it, t1 is doing dio write to [8K, 32K) and t2 is doing dio
 * read from [0, 4K) or [4K, 8K).
 *
 * t1 goes ahead of t2 and splits em [0, 32K) to em [0K, 8K) and [8K 32K).
 *
 *         t1                                t2
 *  btrfs_get_blocks_direct()	       btrfs_get_blocks_direct()
 *   -> btrfs_get_extent()              -> btrfs_get_extent()
 *       -> lookup_extent_mapping()
 *       -> add_extent_mapping()            -> lookup_extent_mapping()
 *          # load [0, 32K)
 *   -> btrfs_new_extent_direct()
 *       -> btrfs_drop_extent_cache()
 *          # split [0, 32K)
 *       -> add_extent_mapping()
 *          # add [8K, 32K)
 *                                          -> add_extent_mapping()
 *                                             # handle -EEXIST when adding
 *                                             # [0, 32K)
 */
static void test_case_4(struct extent_map_tree *em_tree)
{
	__test_case_4(em_tree, 0);
	__test_case_4(em_tree, SZ_4K);
}

int btrfs_test_extent_map()
{
	struct extent_map_tree *em_tree;

	test_msg("Running extent_map tests\n");

	em_tree = kzalloc(sizeof(*em_tree), GFP_KERNEL);
	if (!em_tree)
		/* Skip the test on error. */
		return 0;

	extent_map_tree_init(em_tree);

	test_case_1(em_tree);
	test_case_2(em_tree);
	test_case_3(em_tree);
	test_case_4(em_tree);

	kfree(em_tree);
	return 0;
}
