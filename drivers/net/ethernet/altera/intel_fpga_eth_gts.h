/* SPDX-License-Identifier: GPL-2.0 */
/* Altera FPGA SM Ethernet MAC driver
 * Copyright (C) 2024, 2025 Altera Corporation. All rights reserved.
 *
 * Contributors:
 *   Preetam Narayan
*/

#ifndef __INTEL_FPGA_SM_ETH_H__
#define __INTEL_FPGA_SM_ETH_H__

#define INTEL_FPGA_SM_ETH_RESOURCE_NAME "intel_fpga_gts"

#define INTEL_FPGA_BYTE_ALIGN   8
#define INTEL_FPGA_WORD_ALIGN   32

#define MOD_PARAM_PERM  0644

/* Flow Control defines */
#define FLOW_OFF        0
#define FLOW_RX         1
#define FLOW_TX         2
#define FLOW_ON         (FLOW_TX | FLOW_RX)

/* 0x50108: IP Soft Reset Register eth_reset */
#define ETH_EIO_SYS_RST                                         BIT(0)
#define ETH_SOFT_TX_RST                                         BIT(1)
#define ETH_SOFT_RX_RST                                         BIT(2)

/* 0x50074: Configuration of TX Statistics Counters */
#define ETH_TX_CNTR_CFG_RST_ALL                                 BIT(0)
#define ETH_TX_CNTR_CFG_RST_PARITY_ERR                          BIT(1)

/* 0x50078: Configuration of RX Statistics Counters */
#define ETH_RX_CNTR_CFG_RST_ALL                                 BIT(0)
#define ETH_RX_CNTR_CFG_RST_PARITY_ERR                          BIT(1)

#define ETH_ASSERT_PMA_SER_LBK_EN    0x6A340
#define ETH_ASSERT_PMA_SER_LBK_DIS   0x0A040
#define ETH_DEASSERT_PMA_SER_LBK_EN  0x62340
#define ETH_DEASSERT_PMA_SER_LBK_DIS 0x02040
#define ETH_SOFT_RST_ACK  BIT(0)
#define ETH_SOFT_RX_RESET BIT(2)
#define ETH_ASSERT_PMA_SER_LBK_ACK GENMASK(15, 14)
#define ETH_DEASSERT_PMA_SER_LBK_ACK ETH_ASSERT_PMA_SER_LBK_ACK
#define ETH_DEASSERT_PMA_SER_LBK_DONE 0
#define ETH_ENABLE_FEC_LOOPBACK	    BIT(1)
#define ETH_ENABLE_XCVRIF_LOOPBACK	    BIT(0)
#define ETH_ENABLE_MAC_LOOPBACK             GENMASK(3, 2)
#define ETH_ENABLE_NEAREND_PAR_PMA_LOOPBACK BIT(22)
#define ETH_ENABLE_NEAREND_PCS_LOOPBACK     GENMASK(18, 16)
#define ETH_ENABLE_FAREND_PCS_LOOPBACK      GENMASK(15, 13)

#define ETH_DISABLE_FEC_LOOPBACK	    BIT(1)
#define ETH_DISABLE_XCVRIF_LOOPBACK	    BIT(0)
#define ETH_DISABLE_MAC_LOOPBACK	    GENMASK(3, 2)
#define ETH_DISBLE_NEAREND_PAR_PMA_LOOPBACK BIT(22)
#define ETH_DISABLE_NEAREND_PCS_LOOPBACK    GENMASK(18, 16)
#define ETH_DISABLE_FAREND_PCS_LOOPBACK     GENMASK(15, 13)

#define ETH_FREEZE_RX_MAC_STATS	BIT(2)
#define ETH_FREEZE_TX_MAC_STATS	BIT(2)
#define ETH_DEFREEZE_RX_MAC_STATS	BIT(2)
#define ETH_DEFREEZE_TX_MAC_STATS	BIT(2)

#define ETH_RX_MAC_REMOTE_FAULT		BIT(1)
#define ETH_RX_MAC_LOCAL_FAULT		BIT(0)

#define ETH_TX_MAC_ENABLE_S_ADDR_EN     BIT(3)
#define ETH_TX_MAC_DISABLE_TXMAC        BIT(2)
#define ETH_RX_MAC_CRC_FORWARD          BIT(0)

/* Flow Control Feature Configuration */
#define ETH_TX_EN_STD_FLOW_CTRL                                 BIT(0)
#define ETH_RX_EN_STD_FLOW_CTRL                                 BIT(0)

struct intel_fpga_gts_eth_softip_csr {
	u32 gui_option; //0x100
	u32 qhip_scratch; //0x104
	u32 eth_reset;
	u32 eth_reset_status; //0x10C
	u32 phy_tx_pll_locked; //0x110
	u32 phy_eiofreq_locked; //0x114
	u32 pcs_status; //0x118
	u32 pcs_control; //0x11C
	u32 link_fault_status; //0x120
	u32 res1[1];
	u32 clk_tx_khz; //0x128
	u32 clk_rx_khz; //0x12C
	u32 clk_pll_khz; //0x130
	u32 clk_tx_div_khz; //0x134
	u32 clk_rec_div64_khz; //0x138
	u32 clk_rec_div_khz; //0x13C
	u32 res2[4];
	u32 status_signals; //0x150
};

 #define eth_soft_csroffs(a) (offsetof(struct intel_fpga_gts_eth_softip_csr, a))

struct intel_fpga_gts_eth_softip_ptp {
	u32 ptp_tx_tam_adjust; //0x800
	u32 ptp_rx_tam_adjust; //0x804
	u32 res1[2];
	u32 ptp_ref_lane; //0x80C
	u32 ptp_dr_cfg; //0x810
	u32 ptp_tx_user_cfg_status; //0x814
	u32 ptp_rx_user_cfg_status; //0x818
	u32 ptp_uim_tam_snapshot; //0x81C
	u32 ptp_tx_uim_tam_info0; //0x820
	u32 ptp_tx_uim_tam_info1; //0x824
	u32 ptp_rx_uim_tam_info0; //0x828
	u32 ptp_rx_uim_tam_info1; //0x82C
	u32 ptp_status; //0x830
	u32 res2[3];
	u32 ptp_status2; //0x840
	u32 res3[43];
	u32 ptp_tx_lane_calc_data_constdelay; //0x8F0
	u32 ptp_rx_lane_calc_data_constdelay; //0x8F4
	u32 res4[3];
	u32 ptp_tx_lane0_calc_data_offset; //0x900
	u32 ptp_rx_lane0_calc_data_offset; //0x904
	u32 ptp_tx_lane0_calc_data_time; //0x908
	u32 ptp_rx_lane0_calc_data_time; //0x90C
	u32 ptp_tx_lane0_calc_data_wiredelay; //0x910
	u32 ptp_rx_lane0_calc_data_wiredelay; //0x914
};

#define eth_softip_ptp_csroffs(a) (offsetof(struct intel_fpga_gts_eth_softip_ptp, a))

struct intel_fpga_gts_pcs_fec {
	u32 res1[0x3BFF];                  // 0x51000 - 0x60000 (reserved)
	u32 config_ctrl;		   // 0x60000
	u32 res2[2];
	u32 tx_pld_conf;		   // 0x60010
	u32 res3[0xD];
	u32 phy_ehip_pcs_modes;            // 0x60048
	u32 res4[0x9];                     // 0x6004C - 0x6006F (reserved)
	u32 xus_timer_window;              // 0x60070
	u32 ber_invalid_count;             // 0x60074
	u32 err_inj;                       // 0x60078
	u32 res5;                          // 0x6007C (reserved)
	u32 phy_frame_error;               // 0x60080
	u32 phy_rxpcs_status;              // 0x60084
	u32 am_lock;                       // 0x60088
	u32 res6;                          // 0x6008C (reserved)
	u32 ber_count;                     // 0x60090
	u32 res7[0x18];                    // 0x60094 - 0x600F3 (reserved)
	u32 err_block_cnt;                 // 0x600F4
	u32 res8[0x3FC2];                  // 0x600F8 - 0x6FFFF (reserved)
	u32 rsfec_tx_top;                  // 0x70000
	u32 res9[2];                       // 0x70004 - 0x7000B (reserved)
	u32 rsfec_lane_cfg0;               // 0x7000C
	u32 res10[6];                       // 0x70010 - 0x70027 (reserved)
	u32 rsfec_err_inj_tx;              // 0x70028
	u32 res11[76];                      // 0x7002C - 0x70203 (reserved)
	u32 rsfec_lane_tx_stat;            // 0x70204
	u32 rsfec_lane_tx_hold;            // 0x70208
	u32 res12;                         // 0x7020C (reserved)
	u32 rsfec_lane_rx_stat;            // 0x70210
	u32 rsfec_lane_rx_hold;            // 0x70214
	u32 res13[6];                      // 0x70218 - 0x7022F (reserved)
	u32 rsfec_cw_pos_rx;               // 0x70230
	u32 res14;                         // 0x70234 (reserved)
	u32 rsfec_err_val_tx;              // 0x70238
	u32 rsfec_corr_cw_cnt_lo;          // 0x7023C
	u32 rsfec_corr_cw_cnt_hi;          // 0x70240
	u32 rsfec_uncorr_cw_cnt_lo;        // 0x70244
	u32 rsfec_uncorr_cw_cnt_hi;        // 0x70248
	u32 rsfec_corr_syms_cnt_lo;        // 0x7024C
	u32 rsfec_corr_syms_cnt_hi;        // 0x70250
	u32 rsfec_corr_0s_cnt_lo;          // 0x70254
	u32 rsfec_corr_0s_cnt_hi;          // 0x70258
	u32 rsfec_corr_1s_cnt_lo;          // 0x7025C
	u32 rsfec_corr_1s_cnt_hi;          // 0x70260
	u32 rsfec_corr_cwbin_cnt_0_1;      // 0x70264
	u32 rsfec_corr_cwbin_cnt_2_3;      // 0x70268
	u32 rsfec_corr_cwbin_cnt_4_5;      // 0x7026C
	u32 rsfec_corr_cwbin_cnt_6_7;      // 0x70270
	u32 rsfec_corr_cwbin_cnt_8_9;      // 0x70274
	u32 rsfec_corr_cwbin_cnt_10_11;    // 0x70278
	u32 rsfec_debug_cfg;               // 0x7027C
};

#define eth_hardip_pcsfec_csroffs(a) (offsetof(struct intel_fpga_gts_pcs_fec, a))

struct intel_fpga_gts_hardip_ptp {
	u32 res1[0x40020 / 4 - 1]; // Reserved fields up to offset 0x40020
	u32 ptp_clk_mux;            // 0x40020
	u32 cfg_tx_ptp0_p2p0;       // 0x40024
	u32 cfg_tx_ptp0_p2p1;       // 0x40028
	u32 cfg_tx_ptp0_p2p2;       // 0x4002C
	u32 cfg_tx_ptp0_p2p3;       // 0x40030
	u32 cfg_tx_ptp0_p2p4;       // 0x40034
	u32 cfg_tx_ptp0_p2p5;       // 0x40038
	u32 cfg_tx_ptp0_p2p6;       // 0x4003C
	u32 cfg_tx_ptp0_p2p7;       // 0x40040
	u32 cfg_tx_ptp0_p2p8;       // 0x40044
	u32 cfg_tx_ptp0_p2p9;       // 0x40048
	u32 cfg_tx_ptp0_p2p10;      // 0x4004C
	u32 cfg_tx_ptp0_p2p11;      // 0x40050
	u32 cfg_tx_ptp0_p2p12;      // 0x40054
	u32 cfg_tx_ptp0_p2p13;      // 0x40058
	u32 cfg_tx_ptp0_p2p14;      // 0x4005C
	u32 cfg_tx_ptp0_p2p15;      // 0x40060
	u32 cfg_tx_ptp0_p2p16;      // 0x40064
	u32 cfg_tx_ptp0_p2p17;      // 0x40068
	u32 cfg_tx_ptp0_p2p18;      // 0x4006C
	u32 cfg_tx_ptp0_p2p19;      // 0x40070
	u32 cfg_tx_ptp0_p2p20;      // 0x40074
	u32 cfg_tx_ptp0_p2p21;      // 0x40078
	u32 cfg_tx_ptp0_p2p22;      // 0x4007C
	u32 cfg_tx_ptp0_p2p23;      // 0x40080
	u32 cfg_tx_ptp0_p2p24;      // 0x40084
	u32 cfg_tx_ptp0_p2p25;      // 0x40088
	u32 cfg_tx_ptp0_p2p26;      // 0x4008C
	u32 cfg_tx_ptp0_p2p27;      // 0x40090
	u32 cfg_tx_ptp0_p2p28;      // 0x40094
	u32 cfg_tx_ptp0_p2p29;      // 0x40098
	u32 cfg_tx_ptp0_p2p30;      // 0x4009C
	u32 cfg_tx_ptp0_p2p31;      // 0x400A0
	u32 cfg_tx_ptp0_p2p32;      // 0x400A4
	u32 cfg_tx_ptp0_p2p33;      // 0x400A8
	u32 cfg_tx_ptp0_p2p34;      // 0x400AC
	u32 cfg_tx_ptp0_p2p35;      // 0x400B0
	u32 cfg_tx_ptp0_p2p36;      // 0x400B4
	u32 cfg_tx_ptp0_p2p37;      // 0x400B8
	u32 cfg_tx_ptp0_p2p38;      // 0x400BC
	u32 cfg_tx_ptp0_p2p39;      // 0x400C0
	u32 cfg_tx_ptp0_p2p40;      // 0x400C4
	u32 cfg_tx_ptp0_p2p41;      // 0x400C8
	u32 cfg_tx_ptp0_p2p42;      // 0x400CC
	u32 cfg_tx_ptp0_p2p43;      // 0x400D0
	u32 cfg_tx_ptp0_p2p44;      // 0x400D4
	u32 cfg_tx_ptp0_p2p45;      // 0x400D8
	u32 cfg_tx_ptp0_p2p46;      // 0x400DC
	u32 cfg_tx_ptp0_p2p47;      // 0x400E0
	u32 cfg_tx_ptp0_p2p48;      // 0x400E4
	u32 cfg_tx_ptp0_p2p49;      // 0x400E8
	u32 cfg_tx_ptp0_p2p50;      // 0x400EC
	u32 cfg_tx_ptp0_p2p51;      // 0x400F0
	u32 cfg_tx_ptp0_p2p52;      // 0x400F4
	u32 cfg_tx_ptp0_p2p53;      // 0x400F8
	u32 cfg_tx_ptp0_p2p54;      // 0x400FC
	u32 cfg_tx_ptp0_p2p55;      // 0x40100
	u32 cfg_tx_ptp0_p2p56;      // 0x40104
	u32 cfg_tx_ptp0_p2p57;      // 0x40108
	u32 cfg_tx_ptp0_p2p58;      // 0x4010C
	u32 cfg_tx_ptp0_p2p59;      // 0x40110
	u32 cfg_tx_ptp0_p2p60;      // 0x40114
	u32 cfg_tx_ptp0_p2p61;      // 0x40118
	u32 cfg_tx_ptp0_p2p62;      // 0x4011C
	u32 cfg_tx_ptp0_p2p63;      // 0x40120
	u32 cfg_tx_ptp0_p2p64;      // 0x40124
	u32 cfg_tx_ptp0_p2p65;      // 0x40128
	u32 cfg_tx_ptp0_p2p66;      // 0x4012C
	u32 cfg_tx_ptp0_p2p67;      // 0x40130
	u32 cfg_tx_ptp0_p2p68;      // 0x40134
	u32 cfg_tx_ptp0_p2p69;      // 0x40138
	u32 cfg_tx_ptp0_p2p70;      // 0x4013C
	u32 cfg_tx_ptp0_p2p71;      // 0x40140
	u32 cfg_tx_ptp0_p2p72;      // 0x40144
	u32 cfg_tx_ptp0_p2p73;      // 0x40148
	u32 cfg_tx_ptp0_p2p74;      // 0x4014C
	u32 cfg_tx_ptp0_p2p75;      // 0x40150
	u32 cfg_tx_ptp0_p2p76;      // 0x40154
	u32 cfg_tx_ptp0_p2p77;      // 0x40158
	u32 cfg_tx_ptp0_p2p78;      // 0x4015C
	u32 cfg_tx_ptp0_p2p79;      // 0x40160
	u32 cfg_tx_ptp0_p2p80;      // 0x40164
	u32 cfg_tx_ptp0_p2p81;      // 0x40168
	u32 cfg_tx_ptp0_p2p82;      // 0x4016C
	u32 cfg_tx_ptp0_p2p83;      // 0x40170
	u32 cfg_tx_ptp0_p2p84;      // 0x40174
	u32 cfg_tx_ptp0_p2p85;      // 0x40178
	u32 cfg_tx_ptp0_p2p86;      // 0x4017C
	u32 cfg_tx_ptp0_p2p87;      // 0x40180
	u32 cfg_tx_ptp0_p2p88;      // 0x40184
	u32 cfg_tx_ptp0_p2p89;      // 0x40188
	u32 cfg_tx_ptp0_p2p90;      // 0x4018C
	u32 cfg_tx_ptp0_p2p91;      // 0x40190
	u32 cfg_tx_ptp0_p2p92;      // 0x40194
	u32 cfg_tx_ptp0_p2p93;      // 0x40198
	u32 cfg_tx_ptp0_p2p94;      // 0x4019C
	u32 cfg_tx_ptp0_p2p95;      // 0x401A0
	u32 cfg_tx_ptp0_p2p96;      // 0x401A4
	u32 cfg_tx_ptp0_p2p97;      // 0x401A8
	u32 cfg_tx_ptp0_p2p98;      // 0x401AC
	u32 cfg_tx_ptp0_p2p99;      // 0x401B0
	u32 cfg_tx_ptp0_p2p100;     // 0x401B4
	u32 cfg_tx_ptp0_p2p101;     // 0x401B8
	u32 cfg_tx_ptp0_p2p102;     // 0x401BC
	u32 cfg_tx_ptp0_p2p103;     // 0x401C0
	u32 cfg_tx_ptp0_p2p104;     // 0x401C4
	u32 cfg_tx_ptp0_p2p105;     // 0x401C8
	u32 cfg_tx_ptp0_p2p106;     // 0x401CC
	u32 cfg_tx_ptp0_p2p107;     // 0x401D0
	u32 cfg_tx_ptp0_p2p108;     // 0x401D4
	u32 cfg_tx_ptp0_p2p109;     // 0x401D8
	u32 cfg_tx_ptp0_p2p110;     // 0x401DC
	u32 cfg_tx_ptp0_p2p111;     // 0x401E0
	u32 cfg_tx_ptp0_p2p112;     // 0x401E4
	u32 cfg_tx_ptp0_p2p113;     // 0x401E8
	u32 cfg_tx_ptp0_p2p114;     // 0x401EC
	u32 cfg_tx_ptp0_p2p115;     // 0x401F0
	u32 cfg_tx_ptp0_p2p116;     // 0x401F4
	u32 cfg_tx_ptp0_p2p117;     // 0x401F8
	u32 cfg_tx_ptp0_p2p118;     // 0x401FC
	u32 cfg_tx_ptp0_p2p119;     // 0x40200
	u32 cfg_tx_ptp0_p2p120;     // 0x40204
	u32 cfg_tx_ptp0_p2p121;     // 0x40208
	u32 cfg_tx_ptp0_p2p122;     // 0x4020C
	u32 cfg_tx_ptp0_p2p123;     // 0x40210
	u32 cfg_tx_ptp0_p2p124;     // 0x40214
	u32 cfg_tx_ptp0_p2p125;     // 0x40218
	u32 cfg_tx_ptp0_p2p126;     // 0x4021C
	u32 cfg_tx_ptp0_p2p127;     // 0x40220
	u32 cfg_tx_ptp0_asm0;       // 0x40224
	u32 cfg_tx_ptp0_asm1;	    //0x40228
	u32 cfg_tx_ptp0_asm2; //0x4022C
	u32 cfg_tx_ptp0_asm3; //0x40230
	u32 cfg_tx_ptp0_asm4; //0x40234
	u32 cfg_tx_ptp0_asm5; //0x40238
	u32 cfg_tx_ptp0_asm6; //0x4023C
	u32 cfg_tx_ptp0_asm7; //0x40240
	u32 cfg_tx_ptp0_asm8; //0x40244
	u32 cfg_tx_ptp0_asm9; //0x40248
	u32 cfg_tx_ptp0_asm10; //0x4024C
	u32 cfg_tx_ptp0_asm11; //0x40250
	u32 cfg_tx_ptp0_asm12; //0x40254
	u32 cfg_tx_ptp0_asm13; //0x40258
	u32 cfg_tx_ptp0_asm14; //0x4025C
	u32 cfg_tx_ptp0_asm15; //0x40260
	u32 cfg_tx_ptp0_asm16; //0x40264
	u32 cfg_tx_ptp0_asm17; //0x40268
	u32 cfg_tx_ptp0_asm18; //0x4026C
	u32 cfg_tx_ptp0_asm19; //0x40270
	u32 cfg_tx_ptp0_asm20; //0x40274
	u32 cfg_tx_ptp0_asm21; //0x40278
	u32 cfg_tx_ptp0_asm22; //0x4027C
	u32 cfg_tx_ptp0_asm23; //0x40280
	u32 cfg_tx_ptp0_asm24; //0x40284
	u32 cfg_tx_ptp0_asm25; //0x40288
	u32 cfg_tx_ptp0_asm26; //0x4028C
	u32 cfg_tx_ptp0_asm27; //0x40290
	u32 cfg_tx_ptp0_asm28; //0x40294
	u32 cfg_tx_ptp0_asm29; //0x40298
	u32 cfg_tx_ptp0_asm30; //0x4029C
	u32 cfg_tx_ptp0_asm31; //0x402A0
	u32 cfg_tx_ptp0_asm32; //0x402A4
	u32 cfg_tx_ptp0_asm33; //0x402A8
	u32 cfg_tx_ptp0_asm34; //0x402AC
	u32 cfg_tx_ptp0_asm35; //0x402B0
	u32 cfg_tx_ptp0_asm36; //0x402B4
	u32 cfg_tx_ptp0_asm37; //0x402B8
	u32 cfg_tx_ptp0_asm38; //0x402BC
	u32 cfg_tx_ptp0_asm39; //0x402C0
	u32 cfg_tx_ptp0_asm40; //0x402C4
	u32 cfg_tx_ptp0_asm41; //0x402C8
	u32 cfg_tx_ptp0_asm42; //0x402CC
	u32 cfg_tx_ptp0_asm43; //0x402D0
	u32 cfg_tx_ptp0_asm44; //0x402D4
	u32 cfg_tx_ptp0_asm45; //0x402D8
	u32 cfg_tx_ptp0_asm46; //0x402DC
	u32 cfg_tx_ptp0_asm47; //0x402E0
	u32 cfg_tx_ptp0_asm48; //0x402E4
	u32 cfg_tx_ptp0_asm49; //0x402E8
	u32 cfg_tx_ptp0_asm50; //0x402EC
	u32 cfg_tx_ptp0_asm51; //0x402F0
	u32 cfg_tx_ptp0_asm52; //0x402F4
	u32 cfg_tx_ptp0_asm53; //0x402F8
	u32 cfg_tx_ptp0_asm54; //0x402FC
	u32 cfg_tx_ptp0_asm55; //0x40300
	u32 cfg_tx_ptp0_asm56; //0x40304
	u32 cfg_tx_ptp0_asm57; //0x40308
	u32 cfg_tx_ptp0_asm58; //0x4030C
	u32 cfg_tx_ptp0_asm59; //0x40310
	u32 cfg_tx_ptp0_asm60; //0x40314
	u32 cfg_tx_ptp0_asm61; //0x40318
	u32 cfg_tx_ptp0_asm62; //0x4031C
	u32 cfg_tx_ptp0_asm63; //0x40320
	u32 cfg_tx_ptp0_asm64; //0x40324
	u32 cfg_tx_ptp0_asm65; //0x40328
	u32 cfg_tx_ptp0_asm66; //0x4032C
	u32 cfg_tx_ptp0_asm67; //0x40330
	u32 cfg_tx_ptp0_asm68; //0x40334
	u32 cfg_tx_ptp0_asm69; //0x40338
	u32 cfg_tx_ptp0_asm70; //0x4033C
	u32 cfg_tx_ptp0_asm71; //0x40340
	u32 cfg_tx_ptp0_asm72; //0x40344
	u32 cfg_tx_ptp0_asm73; //0x40348
	u32 cfg_tx_ptp0_asm74; //0x4034C
	u32 cfg_tx_ptp0_asm75; //0x40350
	u32 cfg_tx_ptp0_asm76; //0x40354
	u32 cfg_tx_ptp0_asm77; //0x40358
	u32 cfg_tx_ptp0_asm78; //0x4035C
	u32 cfg_tx_ptp0_asm79; //0x40360
	u32 cfg_tx_ptp0_asm80; //0x40364
	u32 cfg_tx_ptp0_asm81; //0x40368
	u32 cfg_tx_ptp0_asm82; //0x4036C
	u32 cfg_tx_ptp0_asm83; //0x40370
	u32 cfg_tx_ptp0_asm84; //0x40374
	u32 cfg_tx_ptp0_asm85; //0x40378
	u32 cfg_tx_ptp0_asm86; //0x4037C
	u32 cfg_tx_ptp0_asm87; //0x40380
	u32 cfg_tx_ptp0_asm88; //0x40384
	u32 cfg_tx_ptp0_asm89; //0x40388
	u32 cfg_tx_ptp0_asm90; //0x4038C
	u32 cfg_tx_ptp0_asm91; //0x40390
	u32 cfg_tx_ptp0_asm92; //0x40394
	u32 cfg_tx_ptp0_asm93; //0x40398
	u32 cfg_tx_ptp0_asm94; //0x4039C
	u32 cfg_tx_ptp0_asm95; //0x403A0
	u32 cfg_tx_ptp0_asm96; //0x403A4
	u32 cfg_tx_ptp0_asm97; //0x403A8
	u32 cfg_tx_ptp0_asm98; //0x403AC
	u32 cfg_tx_ptp0_asm99; //0x403B0
	u32 cfg_tx_ptp0_asm100; //0x403B4
	u32 cfg_tx_ptp0_asm101; //0x403B8
	u32 cfg_tx_ptp0_asm102; //0x403BC
	u32 cfg_tx_ptp0_asm103; //0x403C0
	u32 cfg_tx_ptp0_asm104; //0x403C4
	u32 cfg_tx_ptp0_asm105; //0x403C8
	u32 cfg_tx_ptp0_asm106; //0x403CC
	u32 cfg_tx_ptp0_asm107; //0x403D0
	u32 cfg_tx_ptp0_asm108; //0x403D4
	u32 cfg_tx_ptp0_asm109; //0x403D8
	u32 cfg_tx_ptp0_asm110; //0x403DC
	u32 cfg_tx_ptp0_asm111; //0x403E0
	u32 cfg_tx_ptp0_asm112; //0x403E4
	u32 cfg_tx_ptp0_asm113; //0x403E8
	u32 cfg_tx_ptp0_asm114; //0x403EC
	u32 cfg_tx_ptp0_asm115; //0x403F0
	u32 cfg_tx_ptp0_asm116; //0x403F4
	u32 cfg_tx_ptp0_asm117; //0x403F8
	u32 cfg_tx_ptp0_asm118; //0x403FC
	u32 cfg_tx_ptp0_asm119; //0x40400
	u32 cfg_tx_ptp0_asm120; //0x40404
	u32 cfg_tx_ptp0_asm121; //0x40408
	u32 cfg_tx_ptp0_asm122; //0x4040C
	u32 cfg_tx_ptp0_asm123; //0x40410
	u32 cfg_tx_ptp0_asm124; //0x40414
	u32 cfg_tx_ptp0_asm125; //0x40418
	u32 cfg_tx_ptp0_asm126; //0x4041C
	u32 cfg_tx_ptp0_asm127; //0x40420
	u32 res2[(0x407F8 - 0x40424) / 4 - 1]; // Reserved fields up to offset 0x407F8
	u32 ptp_rx_mux;              // 0x407F8
};

 #define eth_hardip_ptp_csroffs(a) (offsetof(struct intel_fpga_gts_hardip_ptp, a))

struct intel_fpga_gts_hardip_emac {
	u32 link_fault_config; //0x50000
	u32 ipg_col_rem; //0x50004
	u32 max_tx_size_config; //0x50008
	u32 txmac_control; //0x5000C
	u32 txmac_ehip_cfg; //0x50010
	u32 txmac_saddrl; //0x50014
	u32 txmac_saddrh; //0x50018
	u32 max_rx_size_config; //0x5001C
	u32 mac_crc_config; //0x50020
	u32 rxmac_control; //0x50024
	u32 rxmac_ehip_cfg; //0x50028
	u32 tx_pause_en; //0x5002C
	u32 tx_pause_request; //0x50030
	u32 retransmit_xoff_holdoff_en; //0x50034
	u32 retransmit_xoff_holdoff_quanta; //0x50038
	u32 tx_pause_quanta; //0x5003C
	u32 tx_xof_en_tx_pause_qnumber; //0x50040
	u32 cfg_retransmit_holdoff_en; //0x50044
	u32 cfg_retransmit_holdoff_quanta; //0x50048
	u32 tx_pfc_daddrl; //0x5004C
	u32 tx_pfc_daddrh; //0x50050
	u32 tx_pfc_saddrl; //0x50054
	u32 tx_pfc_saddrh; //0x50058
	u32 txsfc_ehip_cfg; //0x5005C
	u32 rx_pause_enable; //0x50060
	u32 rx_pause_fwd; //0x50064
	u32 rx_pause_daddrl; //0x50068
	u32 rx_pause_daddrh; //0x5006C
	u32 rxsfc_ehip_cfg; //0x50070
	u32 cntr_tx_config; //0x50074
	u32 cntr_rx_config; //0x50078
	u32 res1[2];
	u32 pfc_pause_quanta_0; //0x50084
	u32 pfc_pause_quanta_1; //0x50088
	u32 pfc_pause_quanta_2; //0x5008C
	u32 pfc_pause_quanta_3; //0x50090
	u32 pfc_pause_quanta_4; //0x50094
	u32 pfc_pause_quanta_5; //0x50098
	u32 pfc_pause_quanta_6; //0x5009C
	u32 pfc_pause_quanta_7; //0x500A0
	u32 pfc_holdoff_quanta_0; //0x500A4
	u32 pfc_holdoff_quanta_1; //0x500A8
	u32 pfc_holdoff_quanta_2; //0x500AC
	u32 pfc_holdoff_quanta_3; //0x500B0
	u32 pfc_holdoff_quanta_4; //0x500B4
	u32 pfc_holdoff_quanta_5; //0x500B8
	u32 pfc_holdoff_quanta_6; //0x500BC
	u32 pfc_holdoff_quanta_7; //0x500C0
	u32 res2[7];
	u32 tx_ptp_extra_latency; //0x500E0
	u32 tx_ptp_ui; //0x500E4
	u32 res3[1];
	u32 tx_ptp_phy_lane_num; //0x500EC
	u32 tx_ptp_ap_filter; //0x500F0
	u32 res4[2];
	u32 rx_ptp_extra_latency; //0x500FC
	u32 rx_ptp_ui; //0x50100
	u32 res5[1];
	u32 rx_ptp_phy_lane_num; //0x50108
	u32 rx_ptp_ap_filter; //0x5010C
	u32 res6[0x3A];
	u32 rx_pkt_n_ts_rx_ctr; //0x501F8
	u32 res7[1];
	u32 cntr_tx_fragments_lo; //0x50200
	u32 res8[1];
	u32 cntr_tx_jabbers_lo; //0x50208
	u32 res9[1];
	u32 cntr_tx_fcs_lo; //0x50210
	u32 res10[1];
	u32 cntr_tx_fcs_err_okpkt_lo; //0x50218
	u32 rev11[1];
	u32 cntr_tx_mcast_data_err_lo; //0x50220
	u32 res12[1];
	u32 cntr_tx_bcast_data_err_lo; //0x50228
	u32 res13[1];
	u32 cntr_tx_ucast_data_err_lo; //0x50230
	u32 res14[1];
	u32 cntr_tx_mcast_ctrl_err_lo; //0x50238
	u32 res15[1];
	u32 cntr_tx_bcast_ctrl_err_lo; //0x50240
	u32 res16[1];
	u32 cntr_tx_ucast_ctrl_err_lo; //0x50248
	u32 res17[1];
	u32 cntr_tx_pause_err_lo; //0x50250
	u32 res18[1];
	u32 cntr_tx_64b_lo; //0x50258
	u32 cntr_tx_64b_hi; //0x5025C
	u32 cntr_tx_65to127b_lo; //0x50260
	u32 cntr_tx_65to127b_hi; //0x50264
	u32 cntr_tx_128to255b_lo; //0x50268
	u32 cntr_tx_128to255b_hi; //0x5026C
	u32 cntr_tx_256to511b_lo; //0x50270
	u32 cntr_tx_256to511b_hi; //0x50274
	u32 cntr_tx_512to1023b_lo; //0x50278
	u32 cntr_tx_512to1023b_hi; //0x5027C
	u32 cntr_tx_1024to1518b_lo; //0x50280
	u32 cntr_tx_1024to1518b_hi; //0x50284
	u32 cntr_tx_1519tomaxb_lo; //0x50288
	u32 cntr_tx_1519tomaxb_hi; //0x5028C
	u32 cntr_tx_oversize_lo; //0x50290
	u32 res19[1];
	u32 cntr_tx_mcast_data_ok_lo; //0x50298
	u32 cntr_tx_mcast_data_ok_hi; //0x5029C
	u32 cntr_tx_bcast_data_ok_lo; //0x502A0
	u32 cntr_tx_bcast_data_ok_hi; //0x502A4
	u32 cntr_tx_ucast_data_ok_lo; //0x502A8
	u32 cntr_tx_ucast_data_ok_hi; //0x502AC
	u32 cntr_tx_mcast_ctrl_lo; //0x502B0
	u32 cntr_tx_mcast_ctrl_hi; //0x502B4
	u32 cntr_tx_bcast_ctrl_lo; //0x502B8
	u32 cntr_tx_bcast_ctrl_hi; //0x502BC
	u32 cntr_tx_ucast_ctrl_lo; //0x502C0
	u32 cntr_tx_ucast_ctrl_hi; //0x502C4
	u32 cntr_tx_pause_lo; //0x502C8
	u32 cntr_tx_pause_hi; //0x502CC
	u32 cntr_tx_runt_lo; //0x502D0
	u32 res20[1];
	u32 cntr_tx_st_lo; //0x502D8
	u32 cntr_tx_st_hi; //0x502DC
	u32 cntr_tx_lenerr_lo; //0x502E0
	u32 res21[1];
	u32 cntr_tx_pfc_err_lo; //0x502E8
	u32 res22[1];
	u32 cntr_tx_pfc_lo; //0x502F0
	u32 cntr_tx_pfc_hi; //0x502F4
	u32 cntr_tx_payloadoctetsok_lo; //0x502F8
	u32 cntr_tx_payloadoctetsok_hi; //0x502FC
	u32 cntr_tx_octetsok_lo; //0x50300
	u32 cntr_tx_octetsok_hi; //0x50304
	u32 cntr_tx_malformed_lo; //0x50308
	u32 res23[1];
	u32 cntr_tx_dropped_lo; //0x50310
	u32 res24[1];
	u32 cntr_tx_badlt_lo; //0x50318
	u32 res25[1];
	u32 cntr_tx_total_ptp_pkts; //0x50320
	u32 cntr_tx_total_1step_ptp_pkts; //0x50324
	u32 cntr_tx_total_2step_ptp_pkts; //0x50328
	u32 cntr_tx_total_v1_ptp_pkts; //0x5032C
	u32 cntr_tx_total_v2_ptp_pkts; //0x50330
	u32 cntr_rx_fragments_lo; //0x50334
	u32 res26[1];
	u32 cntr_rx_jabbers_lo; //0x5033C
	u32 res27[1];
	u32 cntr_rx_fcs_lo; //0x50344
	u32 res28[1];
	u32 cntr_rx_fcs_err_okpkt_lo; //0x5034C
	u32 res29[1];
	u32 cntr_rx_mcast_data_err_lo; //0x50354
	u32 res30[1];
	u32 cntr_rx_bcast_data_err_lo; //0x5035C
	u32 res31[1];
	u32 cntr_rx_ucast_data_err_lo; //0x50364
	u32 res32[1];
	u32 cntr_rx_mcast_ctrl_err_lo; //0x5036C
	u32 res33[1];
	u32 cntr_rx_bcast_ctrl_err_lo; //0x50374
	u32 res34[1];
	u32 cntr_rx_ucast_ctrl_err_lo; //0x5037C
	u32 res35[1];
	u32 cntr_rx_pause_err_lo; //0x50384
	u32 res36[1];
	u32 cntr_rx_64b_lo; //0x5038C
	u32 cntr_rx_64b_hi; //0x50390
	u32 cntr_rx_65to127b_lo; //0x50394
	u32 cntr_rx_65to127b_hi; //0x50398
	u32 cntr_rx_128to255b_lo; //0x5039C
	u32 cntr_rx_128to255b_hi; //0x503A0
	u32 cntr_rx_256to511b_lo; //0x503A4
	u32 cntr_rx_256to511b_hi; //0x503A8
	u32 cntr_rx_512to1023b_lo; //0x503AC
	u32 cntr_rx_512to1023b_hi; //0x503B0
	u32 cntr_rx_1024to1518b_lo; //0x503B4
	u32 cntr_rx_1024to1518b_hi; //0x503B8
	u32 cntr_rx_1519tomaxb_lo; //0x503BC
	u32 cntr_rx_1519tomaxb_hi; //0x503C0
	u32 cntr_rx_oversize_lo; //0x503C4
	u32 res37[1];
	u32 cntr_rx_mcast_data_ok_lo; //0x503CC
	u32 cntr_rx_mcast_data_ok_hi; //0x503D0
	u32 cntr_rx_bcast_data_ok_lo; //0x503D4
	u32 cntr_rx_bcast_data_ok_hi; //0x503D8
	u32 cntr_rx_ucast_data_ok_lo; //0x503DC
	u32 cntr_rx_ucast_data_ok_hi; //0x503E0
	u32 cntr_rx_mcast_ctrl_lo; //0x503E4
	u32 cntr_rx_mcast_ctrl_hi; //0x503E8
	u32 cntr_rx_bcast_ctrl_lo; //0x503EC
	u32 cntr_rx_bcast_ctrl_hi; //0x503F0
	u32 cntr_rx_ucast_ctrl_lo; //0x503F4
	u32 cntr_rx_ucast_ctrl_hi; //0x503F8
	u32 cntr_rx_pause_lo; //0x503FC
	u32 cntr_rx_pause_hi; //0x50400
	u32 cntr_rx_runt_lo; //0x50404
	u32 res38[1];
	u32 cntr_rx_st_lo; //0x5040C
	u32 cntr_rx_st_hi; //0x50410
	u32 cntr_rx_lenerr_lo; //0x50414
	u32 res39[1];
	u32 cntr_rx_pfc_err_lo; //0x5041C
	u32 res40[1];
	u32 cntr_rx_pfc_lo; //0x50424
	u32 cntr_rx_pfc_hi; //0x50428
	u32 cntr_rx_payloadoctetsok_lo; //0x5042C
	u32 cntr_rx_payloadoctetsok_hi; //0x50430
	u32 cntr_rx_octetsok_lo; //0x50434
	u32 cntr_rx_octetsok_hi; //0x50438
	u32 cntr_rx_malformed_lo; //0x5043C
	u32 res41[1];
	u32 cntr_rx_dropped_lo; //0x50444
	u32 res42[1];
	u32 cntr_rx_badlt_lo; //0x5044C
	u32 res43[1];
	u32 cntr_rx_total_ptp_ts; //0x50454
	u32 tx_ptp_cf_overflow; //0x50458
	u32 tx_ptp_tam_lo_pl_0; //0x5045C
	u32 tx_ptp_tam_med_pl_0; //0x50460
	u32 tx_ptp_tam_hi_pl_0; //0x50464
	u32 tx_ptp_tam_adj_pl_0; //0x50468
	u32 res44[0x3C];
	u32 tx_ts_ss_lo; //0x5055C
	u32 tx_ts_ss_mid; //0x50560
	u32 tx_ts_ss_hi; //0x50564
	u32 tx_vl_ss; //0x50568
	u32 rx_ptp_tam_lo_pl_0; //0x5056C
	u32 rx_ptp_tam_med_pl_0; //0x50570
	u32 rx_ptp_tam_hi_pl_0; //0x50574
	u32 rx_ptp_tam_adj_pl_0; //0x50578
	u32 res45[0x3C];
	u32 rx_ts_ss_lo; //0x5066C
	u32 rx_ts_ss_mid; //0x50670
	u32 rx_ts_ss_hi; //0x50674
};

#define eth_hardip_emac_csroffs(a) (offsetof(struct intel_fpga_gts_hardip_emac, a))

struct intel_fpga_gts_hardip_xcvr_pma {
	u32 res0[7];
	u32 sm_xcvrif_debug1; //0x80020
	u32 sm_xcvrif_reg_9; //0x80024
	u32 res1[4];
	u32 xcvrif_stat_0; //0x8003C
	u32 xcvrif_stat_hold_1; //0x80040
	u32 res2[2];
	u32 xcvrif_stat_3; //0x8004C
	u32 xcvrif_stat_hold_4; //0x80050
};

 #define eth_hardip_xcvr_pma_csroffs(a) (offsetof(struct intel_fpga_gts_hardip_xcvr_pma, a))

struct intel_fpga_gts_softip_pma_phy {
	u32 gui_option; //0x800
	u32 phy_tx_pll_locked; //0x810
	u32 phy_rx_cdr_locked; //0x814
	u32 phy_reset; //0x808
	u32 phy_reset_status; //0x80C
	u32 src_ctrl; //0x818
	u32 src_user_reg0; //0x10018
	u32 phy_scratch; //0x804
};

struct intel_fpga_gts_hardip_pma {
	u32 res1[0x1D0];
	u32 SRDS_IP_SYNTH_MED_reg_16; //0x90740
	u32 SRDS_IP_SYNTH_MED_reg_17; //0x90744
	u32 res2[0x3E];
	u32 SRDS_IP_SYNTH_SLOW_reg_16; //0x90840
	u32 SRDS_IP_SYNTH_SLOW_reg_17; //0x90844
	u32 res3[0x53];
	u32 SRDS_IP_SYNTH_FAST_reg_37; //0x90994
	u32 SRDS_IP_SYNTH_FAST_reg_38; //0x90998
	u32 res4[0x29F];
	u32 SRDS_IP_LANE_reg_7; //0x91418
	u32 res5;
	u32 SRDS_IP_LANE_reg_9; //0x91420
	u32 res6;
	u32 SRDS_IP_LANE_reg_11; //0x91428
	u32 res7[0x62];
	u32 SRDS_IP_LANE_reg_110; //0x915B4
	u32 res8[0x66];
	u32 SRDS_IP_LANE_reg_213; //0x91750
	u32 res9[0x70];
	u32 SRDS_IP_LANE_RXEQ_reg_5; //0x91914
	u32 res10[0xA8];
	u32 SRDS_IP_LANE_RXEQ_reg_174; //0x91BB8
	u32 res11[0x911];
	u32 SRDS_IP_PLLLCSLOW_DIV0; //0x94000
	u32 res12[0x2];
	u32 SRDS_IP_PLLLCSLOW_FRAC_LOCK0; //0x9400C
	u32 res13[0x3C];
	u32 SRDS_IP_PLLLCMED_DIV0; //0x94100
	u32 res14[0x2];
	u32 SRDS_IP_PLLLCMED_FRAC_LOCK0; //0x9410C
	u32 res15[0x3C];
	u32 SRDS_IP_PLLLCFAST_DIV0; //0x94200
	u32 res16[0x2];
	u32 SRDS_IP_PLLLCFAST_FRAC_LOCK0; //0x9420C
	u32 res17[0xD83];
	u32 SRDS_IP_IF_debug; //0x9781C
	u32 res18[4];
	u32 SRDS_IP_IF_TX1; //0x97830
	u32 res19[0x3202];
	u32 SCMNG_PM_LINK_MNG_SIDE_CPI_REGS; //0xA403C
	u32 SCMNG_PM_PHY_SIDE_CPI_REGS; //0xA4040
	u32 res20[0x3EF];
	u32 GTS_Physical_LANE_Number; //0xA5000
	u32 res21[0x10];
	u32 pre_pma_lblk; //0xA5044
};

#define eth_hardip_pma_csroffs(a) (offsetof(struct intel_fpga_gts_hardip_pma, a))

#define SYS_PLL_LOCKED    BIT(8)
#define RX_CDR_LOCKED0    BIT(7)
#define RX_CDR_LOCKED1    BIT(6)

#define RX_CDR_LOCKED(port) \
	((BUG((port) > 1)), ((port) == 0 ? RX_CDR_LOCKED0 : RX_CDR_LOCKED1))

#define TX_PLL_LOCKED1(x)   (((x) & BIT(5)))
#define TX_PLL_LOCKED0(x)   (((x) & BIT(4)))
#define TX_PLL_LOCKED(x, port) \
	((BUG((port) > 1)), ((port) == 0 ? TX_PLL_LOCKED0(x) : TX_PLL_LOCKED1(x)))

#define TX_LANE_STABLE1(x)  (((x) & BIT(3)))
#define TX_LANE_STABLE0(x)  (((x) & BIT(2)))
#define TX_LANE_STABLE(x, port) \
	((BUG((port) > 1)), ((port) == 0 ? TX_LANE_STABLE0(x) : TX_LANE_STABLE1(x)))

#define RX_PCS_READY0(x)    (((x) & BIT(0)))
#define RX_PCS_READY1(x)    (((x) & BIT(1)))
#define RX_PCS_READY(x, port) \
	((BUG((port) > 1)), ((port) == 0 ? RX_PCS_READY0(x) : RX_PCS_READY1(x)))

#define TX_FIFO_DEPTH0(x)   (((x) & GENMASK(7, 0)))
#define TX_FIFO_DEPTH1(x)   ((((x) & GENMASK(15, 8)) >> 8))
#define TX_FIFO_DEPTH(port, x) \
	((BUG((port) > 1)), ((port) == 0 ? TX_FIFO_DEPTH0(x) : TX_FIFO_DEPTH1(x)))

#define RX_FIFO_DEPTH0(x) (((x) & GENMASK(23, 16)) >> 16)
#define RX_FIFO_DEPTH1(x) (((x) & GENMASK(31, 24)) >> 24)
#define RX_FIFO_DEPTH(port, x) \
	((BUG((port) > 1)), ((port) == 0 ? RX_FIFO_DEPTH0(x) : RX_FIFO_DEPTH1(x)))

struct intel_fpga_userspace_reg {
	u32 control_reg;
	u32 error_reg;
	u32 status_reg;
	u32 fifo_status_reg;
};

#define eth_userspace_csroffs(a) (offsetof(struct intel_fpga_userspace_reg, a))

#endif
