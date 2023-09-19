# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Altera device drivers.
#

obj-$(CONFIG_NET_ALTERA_ETH) += altera_eth.o
altera_eth-objs := altera_utils.o intel_fpga_tod.o altera_eth_dma.o \
		   altera_sgdma.o altera_msgdma.o \
		   altera_msgdma_prefetcher.o
ifeq ($(CONFIG_ALTERA_TSE),y)
	obj-$(CONFIG_ALTERA_TSE) += altera_tse.o
	altera_tse-objs := altera_tse_main.o altera_tse_ethtool.o
endif

