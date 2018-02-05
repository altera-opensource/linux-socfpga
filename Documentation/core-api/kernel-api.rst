====================
The Linux Kernel API
====================


List Management Functions
=========================

.. kernel-doc:: include/linux/list.h
   :internal:

Basic C Library Functions
=========================

When writing drivers, you cannot in general use routines which are from
the C Library. Some of the functions have been found generally useful
and they are listed below. The behaviour of these functions may vary
slightly from those defined by ANSI, and these deviations are noted in
the text.

String Conversions
------------------

.. kernel-doc:: lib/vsprintf.c
   :export:

.. kernel-doc:: include/linux/kernel.h
   :functions: kstrtol

.. kernel-doc:: include/linux/kernel.h
   :functions: kstrtoul

.. kernel-doc:: lib/kstrtox.c
   :export:

String Manipulation
-------------------

.. kernel-doc:: lib/string.c
   :export:

Bit Operations
--------------

.. kernel-doc:: arch/x86/include/asm/bitops.h
   :internal:

Basic Kernel Library Functions
==============================

The Linux kernel provides more basic utility functions.

Bitmap Operations
-----------------

.. kernel-doc:: lib/bitmap.c
   :doc: bitmap introduction

.. kernel-doc:: include/linux/bitmap.h
   :doc: declare bitmap

.. kernel-doc:: include/linux/bitmap.h
   :doc: bitmap overview

.. kernel-doc:: include/linux/bitmap.h
   :doc: bitmap bitops

.. kernel-doc:: lib/bitmap.c
   :export:

.. kernel-doc:: lib/bitmap.c
   :internal:

.. kernel-doc:: include/linux/bitmap.h
   :internal:

Command-line Parsing
--------------------

.. kernel-doc:: lib/cmdline.c
   :export:

CRC Functions
-------------

.. kernel-doc:: lib/crc4.c
   :export:

.. kernel-doc:: lib/crc7.c
   :export:

.. kernel-doc:: lib/crc8.c
   :export:

.. kernel-doc:: lib/crc16.c
   :export:

.. kernel-doc:: lib/crc32.c

.. kernel-doc:: lib/crc-ccitt.c
   :export:

.. kernel-doc:: lib/crc-itu-t.c
   :export:

idr/ida Functions
-----------------

.. kernel-doc:: include/linux/idr.h
   :doc: idr sync

.. kernel-doc:: lib/idr.c
   :doc: IDA description

.. kernel-doc:: lib/idr.c
   :export:

Math Functions in Linux
=======================

Base 2 log and power Functions
------------------------------

.. kernel-doc:: include/linux/log2.h
   :internal:

Division Functions
------------------

.. kernel-doc:: include/asm-generic/div64.h
   :functions: do_div

.. kernel-doc:: include/linux/math64.h
   :internal:

.. kernel-doc:: lib/div64.c
   :functions: div_s64_rem div64_u64_rem div64_u64 div64_s64

.. kernel-doc:: lib/gcd.c
   :export:

Sorting
-------

.. kernel-doc:: lib/sort.c
   :export:

.. kernel-doc:: lib/list_sort.c
   :export:

UUID/GUID
---------

.. kernel-doc:: lib/uuid.c
   :export:

Memory Management in Linux
==========================

The Slab Cache
--------------

.. kernel-doc:: include/linux/slab.h
   :internal:

.. kernel-doc:: mm/slab.c
   :export:

.. kernel-doc:: mm/util.c
   :export:

User Space Memory Access
------------------------

.. kernel-doc:: arch/x86/include/asm/uaccess.h
   :internal:

.. kernel-doc:: arch/x86/lib/usercopy_32.c
   :export:

More Memory Management Functions
--------------------------------

.. kernel-doc:: mm/readahead.c
   :export:

.. kernel-doc:: mm/filemap.c
   :export:

.. kernel-doc:: mm/memory.c
   :export:

.. kernel-doc:: mm/vmalloc.c
   :export:

.. kernel-doc:: mm/page_alloc.c
   :internal:

.. kernel-doc:: mm/mempool.c
   :export:

.. kernel-doc:: mm/dmapool.c
   :export:

.. kernel-doc:: mm/page-writeback.c
   :export:

.. kernel-doc:: mm/truncate.c
   :export:

Kernel IPC facilities
=====================

IPC utilities
-------------

.. kernel-doc:: ipc/util.c
   :internal:

FIFO Buffer
===========

kfifo interface
---------------

.. kernel-doc:: include/linux/kfifo.h
   :internal:

relay interface support
=======================

Relay interface support is designed to provide an efficient mechanism
for tools and facilities to relay large amounts of data from kernel
space to user space.

relay interface
---------------

.. kernel-doc:: kernel/relay.c
   :export:

.. kernel-doc:: kernel/relay.c
   :internal:

Module Support
==============

Module Loading
--------------

.. kernel-doc:: kernel/kmod.c
   :export:

Inter Module support
--------------------

Refer to the file kernel/module.c for more information.

Hardware Interfaces
===================

Interrupt Handling
------------------

.. kernel-doc:: kernel/irq/manage.c
   :export:

DMA Channels
------------

.. kernel-doc:: kernel/dma.c
   :export:

Resources Management
--------------------

.. kernel-doc:: kernel/resource.c
   :internal:

.. kernel-doc:: kernel/resource.c
   :export:

MTRR Handling
-------------

.. kernel-doc:: arch/x86/kernel/cpu/mtrr/main.c
   :export:

Security Framework
==================

.. kernel-doc:: security/security.c
   :internal:

.. kernel-doc:: security/inode.c
   :export:

Audit Interfaces
================

.. kernel-doc:: kernel/audit.c
   :export:

.. kernel-doc:: kernel/auditsc.c
   :internal:

.. kernel-doc:: kernel/auditfilter.c
   :internal:

Accounting Framework
====================

.. kernel-doc:: kernel/acct.c
   :internal:

Block Devices
=============

.. kernel-doc:: block/blk-core.c
   :export:

.. kernel-doc:: block/blk-core.c
   :internal:

.. kernel-doc:: block/blk-map.c
   :export:

.. kernel-doc:: block/blk-sysfs.c
   :internal:

.. kernel-doc:: block/blk-settings.c
   :export:

.. kernel-doc:: block/blk-exec.c
   :export:

.. kernel-doc:: block/blk-flush.c
   :export:

.. kernel-doc:: block/blk-lib.c
   :export:

.. kernel-doc:: block/blk-tag.c
   :export:

.. kernel-doc:: block/blk-tag.c
   :internal:

.. kernel-doc:: block/blk-integrity.c
   :export:

.. kernel-doc:: kernel/trace/blktrace.c
   :internal:

.. kernel-doc:: block/genhd.c
   :internal:

.. kernel-doc:: block/genhd.c
   :export:

Char devices
============

.. kernel-doc:: fs/char_dev.c
   :export:

Clock Framework
===============

The clock framework defines programming interfaces to support software
management of the system clock tree. This framework is widely used with
System-On-Chip (SOC) platforms to support power management and various
devices which may need custom clock rates. Note that these "clocks"
don't relate to timekeeping or real time clocks (RTCs), each of which
have separate frameworks. These :c:type:`struct clk <clk>`
instances may be used to manage for example a 96 MHz signal that is used
to shift bits into and out of peripherals or busses, or otherwise
trigger synchronous state machine transitions in system hardware.

Power management is supported by explicit software clock gating: unused
clocks are disabled, so the system doesn't waste power changing the
state of transistors that aren't in active use. On some systems this may
be backed by hardware clock gating, where clocks are gated without being
disabled in software. Sections of chips that are powered but not clocked
may be able to retain their last state. This low power state is often
called a *retention mode*. This mode still incurs leakage currents,
especially with finer circuit geometries, but for CMOS circuits power is
mostly used by clocked state changes.

Power-aware drivers only enable their clocks when the device they manage
is in active use. Also, system sleep states often differ according to
which clock domains are active: while a "standby" state may allow wakeup
from several active domains, a "mem" (suspend-to-RAM) state may require
a more wholesale shutdown of clocks derived from higher speed PLLs and
oscillators, limiting the number of possible wakeup event sources. A
driver's suspend method may need to be aware of system-specific clock
constraints on the target sleep state.

Some platforms support programmable clock generators. These can be used
by external chips of various kinds, such as other CPUs, multimedia
codecs, and devices with strict requirements for interface clocking.

.. kernel-doc:: include/linux/clk.h
   :internal:

Synchronization Primitives
==========================

Read-Copy Update (RCU)
----------------------

.. kernel-doc:: include/linux/rcupdate.h

.. kernel-doc:: include/linux/rcupdate_wait.h

.. kernel-doc:: include/linux/rcutree.h

.. kernel-doc:: kernel/rcu/tree.c

.. kernel-doc:: kernel/rcu/tree_plugin.h

.. kernel-doc:: kernel/rcu/tree_exp.h

.. kernel-doc:: kernel/rcu/update.c

.. kernel-doc:: include/linux/srcu.h

.. kernel-doc:: kernel/rcu/srcutree.c

.. kernel-doc:: include/linux/rculist_bl.h

.. kernel-doc:: include/linux/rculist.h

.. kernel-doc:: include/linux/rculist_nulls.h

.. kernel-doc:: include/linux/rcu_sync.h

.. kernel-doc:: kernel/rcu/sync.c

