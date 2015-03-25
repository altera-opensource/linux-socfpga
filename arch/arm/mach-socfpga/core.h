/*
 * Copyright 2012 Pavel Machek <pavel@denx.de>
 * Copyright (C) 2012 Altera Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __MACH_CORE_H
#define __MACH_CORE_H

#define SOCFPGA_RSTMGR_CTRL	0x04
#define SOCFPGA_RSTMGR_MODPERRST	0x14
#define SOCFPGA_RSTMGR_BRGMODRST	0x1c

#define SOCFPGA_A10_RSTMGR_CTRL		0xC
#define SOCFPGA_A10_RSTMGR_PER0MODRST	0x24
#define SOCFPGA_A10_RSTMGR_PER1MODRST	0x28
#define SOCFPGA_A10_RSTMGR_BRGMODRST	0x2C

/* System Manager bits */
#define RSTMGR_CTRL_SWCOLDRSTREQ	0x1	/* Cold Reset */
#define RSTMGR_CTRL_SWWARMRSTREQ	0x2	/* Warm Reset */

extern void socfpga_secondary_startup(void);
#define SOCFPGA_ID_DEFAULT		0x1
#define SOCFPGA_REVISION_DEFAULT	0x1

#define SOCFPGA_RSTMGR_CTRL	0x04
#define SOCFPGA_RSTMGR_MODPERRST	0x14
#define SOCFPGA_RSTMGR_BRGMODRST	0x1c

/* A10 Sysmgr EMAC control registers */
#define SOCFPGA_A10_SYSMGR_EMAC0_CTRL	0x44
#define SOCFPGA_A10_SYSMGR_EMAC1_CTRL	0x48
#define SOCFPGA_A10_SYSMGR_EMAC2_CTRL	0x4c

/* System Manager bits */
#define RSTMGR_CTRL_SWCOLDRSTREQ	0x1	/* Cold Reset */
#define RSTMGR_CTRL_SWWARMRSTREQ	0x2	/* Warm Reset */
/*MPU Module Reset Register */
 #define RSTMGR_MPUMODRST_CPU0	0x1	/*CPU0 Reset*/
 #define RSTMGR_MPUMODRST_CPU1	0x2	/*CPU1 Reset*/
 #define RSTMGR_MPUMODRST_WDS		0x4	/*Watchdog Reset*/
 #define RSTMGR_MPUMODRST_SCUPER	0x8	/*SCU and periphs reset*/
 #define RSTMGR_MPUMODRST_L2		0x10	/*L2 Cache reset*/

/* Peripheral Module Reset Register bits */

/*
 * EMAC0 and EMAC1 reset bits in permodrst in C5/A5 and per0modrst
 * in Arria 10 are the same bit locations.
 */
#define RSTMGR_PERMODRST_EMAC0	0x1
#define RSTMGR_PERMODRST_EMAC1	0x2
#define RSTMGR_PER0MODRST_A10_EMAC2 0x4

#define RSTMGR_PER0MODRST_A10_EMAC0_ECC BIT(8)
#define RSTMGR_PER0MODRST_A10_EMAC1_ECC BIT(9)
#define RSTMGR_PER0MODRST_A10_EMAC2_ECC BIT(10)

#define SYSMGR_SILICON_ID1_OFFSET 0x0
#define SYSMGR_SILICON_ID1_REV_SHIFT 0
#define SYSMGR_SILICON_ID1_REV_MASK 0x0000FFFF
#define SYSMGR_SILICON_ID1_ID_SHIFT 16
#define SYSMGR_SILICON_ID1_ID_MASK 0xFFFF0000
#define SYSMGR_EMACGRP_CTRL_OFFSET 0x60
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII 0x0
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RGMII 0x1
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RMII 0x2
#define SYSMGR_EMACGRP_CTRL_PHYSEL_WIDTH 2

#define SYSMGR_EMACGRP_CTRL_PHYSEL_MASK 0x00000003

extern void __iomem *socfpga_scu_base_addr;
extern void __iomem *sys_manager_base_addr;
extern void __iomem *rst_manager_base_addr;

extern void socfpga_init_clocks(void);

extern void v7_secondary_startup(void);
extern struct smp_operations socfpga_smp_ops;
extern char secondary_trampoline, secondary_trampoline_end;

extern struct dw_mci_board sdmmc_platform_data;
extern unsigned long cpu1start_addr;

#define SOCFPGA_SCU_VIRT_BASE   0xfee00000

/* Clock manager defines */
#define SOCFPGA_ENABLE_PLL_REG	0xA0

#endif /* __CORE_H */
