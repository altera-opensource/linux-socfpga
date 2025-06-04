# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the Altera device drivers.
#
ifeq ($(CONFIG_NET_ALTERA_ETH),y)
	obj-$(CONFIG_NET_ALTERA_ETH) += altera_eth.o
	altera_eth-objs := altera_eth_dma.o \
			   altera_sgdma.o altera_msgdma.o \
			   altera_msgdma_prefetcher.o
endif

ifeq ($(CONFIG_INTEL_FPGA_UTIL),y)
	obj-$(CONFIG_INTEL_FPGA_UTIL) += altera_utils.o
endif

ifeq ($(CONFIG_SSGDMA_SOC_DMA),y)
        obj-$(CONFIG_SSGDMA_SOC_DMA) += ssgdma_soc_dma.o
        ssgdma_soc_dma-objs := intel_ssgdma_soc_main.o \
                                        intel_ssgdma_common.o \
                                        intel_ssgdma_hal.o \
                                        intel_ssgdma_dmac.o
	ssgdma_soc_dma-objs := $(addprefix ssgdma/, $(ssgdma_soc_dma-objs))
endif

ifeq ($(CONFIG_INTEL_FPGA_FREQ_CTRL), y)
	obj-$(CONFIG_INTEL_FPGA_FREQ_CTRL) += intel_frequency_ctrl_driver.o
	
        ifeq ($(CONFIG_DEBUG_FS),y)
		intel_frequency_ctrl_driver-objs := intel_freq_ctrl_zl30733_debugfs.o
	endif
	intel_frequency_ctrl_driver-objs += intel_freq_ctrl_zl30793_spi.o \
				intel_freq_ctrl_zl30793.o \
				intel_freq_ctrl_zl30733_i2c.o \
				intel_freq_ctrl_common_spi.o \
				intel_freq_ctrl_common_i2c.o \

	intel_frequency_ctrl_driver-objs := $(addprefix dpll/, $(intel_frequency_ctrl_driver-objs))
endif

ifeq ($(CONFIG_INTEL_FPGA_TOD), y)
	obj-$(CONFIG_INTEL_FPGA_TOD)  += intel_freq_control.o \
					 intel_fpga_tod.o
endif

ifeq ($(CONFIG_ALTERA_TSE),y)
	obj-$(CONFIG_ALTERA_TSE) += altera_tse.o
	altera_tse-objs := altera_tse_main.o altera_tse_ethtool.o
endif
ifeq ($(CONFIG_INTEL_FPGA_QSE_LL),y)
	obj-$(CONFIG_INTEL_FPGA_QSE_LL) += intel_fpga_qse_ll.o
	intel_fpga_qse_ll-objs := intel_fpga_qse_ll_main.o intel_fpga_qse_ll_ethtool.o
endif

ifeq ($(CONFIG_INTEL_FPGA_ETILE),y)
	obj-$(CONFIG_INTEL_FPGA_ETILE) += intel_fpga_etile.o
	intel_fpga_etile-objs := intel_fpga_etile_main.o intel_fpga_etile_fec.o \
				 intel_fpga_etile_ethtool.o
endif

ifeq ($(CONFIG_INTEL_FPGA_HSSISS),y)
        obj-$(CONFIG_INTEL_FPGA_HSSISS) += intel_fpga_hssi.o
        ifeq ($(CONFIG_DEBUG_FS),y)
                intel_fpga_hssi-objs := intel_fpga_hssigl_gts_driver.o \
					intel_fpga_hssiss.o \
					intel_fpga_hssi_driver.o \
					intel_fpga_hssiss_debug.o \
					intel_fpga_hssigl_driver.o \
					intel_fpga_eth_hssi_itf.o
        else
                intel_fpga_hssi-objs := intel_fpga_hssigl_gts_driver.o \
					intel_fpga_hssiss.o \
					intel_fpga_hssi_driver.o \
					intel_fpga_hssigl_driver.o \
					intel_fpga_eth_hssi_itf.o
        endif
endif

ifeq ($(CONFIG_INTEL_FPGA_HSSI_XTILE),y)
	obj-$(CONFIG_INTEL_FPGA_HSSI_XTILE) += intel_fpga_hssi_xtile.o
        ifeq ($(CONFIG_DEBUG_FS),y)
                intel_fpga_hssi_xtile-objs := intel_fpga_eth_debug.o
	endif
        intel_fpga_hssi_xtile-objs += intel_fpga_eth_main.o           \
				      intel_fpga_hssi_etile_fec.o     \
				      intel_fpga_etile_driver.o       \
                                      intel_fpga_hssi_etile_ethtool.o \
				      intel_fpga_ftile_driver.o		\
				      intel_fpga_hssi_ftile_fec.o	\
				      intel_fpga_hssi_ftile_ethtool.o	\

endif

ifeq ($(CONFIG_INTEL_FPGA_GTS_TILE), y)
	obj-$(CONFIG_INTEL_FPGA_GTS_TILE) += intel_fpga_gts_tile.o
        ifeq ($(CONFIG_DEBUG_FS),y)
                intel_fpga_gts_tile-objs := intel_fpga_eth_debug.o
	endif
        intel_fpga_gts_tile-objs += intel_fpga_eth_main.o           \
				    intel_fpga_gts_ethtool.o 	    \
				    intel_fpga_gts_driver.o	    \
				    intel_fpga_etile_driver.o       \
				    intel_fpga_hssi_etile_fec.o     \
                                    intel_fpga_hssi_etile_ethtool.o \
				    intel_fpga_ftile_driver.o       \
				    intel_fpga_hssi_ftile_fec.o	    \
				    intel_fpga_hssi_ftile_ethtool.o
endif
