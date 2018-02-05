#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
# Runs bitmap infrastructure tests using test_bitmap kernel module
if ! /sbin/modprobe -q -n test_bitmap; then
	echo "bitmap: [SKIP]"
	exit 77
fi

if /sbin/modprobe -q test_bitmap; then
	/sbin/modprobe -q -r test_bitmap
	echo "bitmap: ok"
else
	echo "bitmap: [FAIL]"
	exit 1
fi
