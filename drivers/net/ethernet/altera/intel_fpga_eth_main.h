
#ifndef __INTEL_FPGA_ETH_MAIN_H__
#define __INTEL_FPGA_ETH_MAIN_H__

#include <linux/bitops.h>
#include <linux/if_vlan.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timer.h>
#include <linux/phylink.h>
#include "intel_fpga_tod.h"
#include "altera_eth_dma.h"
#include "altera_msgdma.h"
#include "altera_msgdmahw.h"
#include "altera_utils.h"
#include "altera_msgdma_prefetcher.h"
#include "altera_msgdmahw_prefetcher.h"
#include "altera_sgdma.h"

#define INTEL_FPGA_XTILE_ETH_RESOURCE_NAME "intel_fpga_eth"

#define INTEL_FPGA_RET_SUCCESS                          0
/* Flow Control defines */
#define FLOW_OFF        0
#define FLOW_RX         1
#define FLOW_TX         2
#define FLOW_ON         (FLOW_TX | FLOW_RX)

#define INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR              1000000
#define INTEL_FPGA_XTILE_ETH_RESOURCE_NAME "intel_fpga_eth"

#define INTEL_FPGA_BYTE_ALIGN   8
#define INTEL_FPGA_WORD_ALIGN   32

#define MOD_PARAM_PERM  0644

typedef struct {

        const char *fec_type;
        const char *ptp_accu_mode;
        struct net_device *dev;
        struct device     *device;
        struct phylink    *phylink;
        struct xtile_spec_ops *spec_ops;
        struct platform_device *pdev_hssi;
        struct intel_fpga_rx_fifo __iomem *rx_fifo;
        struct intel_fpga_rx_fifo __iomem *tx_fifo;

        u32 tile_chan;
        u32 hssi_port;
        u32 tx_irq;
        u32 rx_irq;
        u32 max_mtu;
        u32 tx_fifo_depth;
        u32 rx_fifo_depth;
        u32 rx_fifo_almost_full;
        u32 rx_fifo_almost_empty;
        u32 rxdma_buffer_size;
        u32 flow_ctrl;
        u32 pause;
        u32 msg_enable;
        u32 link_speed;
        u32 tx_pma_delay_ns;
        u32 rx_pma_delay_ns;
        u32 tx_pma_delay_fns;
        u32 rx_pma_delay_fns;
        u32 rsfec_cw_pos_rx;
        u32 tx_external_phy_delay_ns;
        u32 rx_external_phy_delay_ns;
        u32 ptp_tx_ref_pl;
        u32 ptp_tx_routing_adj;
        u32 ptp_rx_routing_adj;

        u8 duplex;
        u8 qsfp_lane;
        bool autoneg;
        bool ptp_enable;
        bool curr_link_state;
        bool cable_unplugged;

        spinlock_t tx_lock;
        spinlock_t mac_cfg_lock;
        spinlock_t rxdma_irq_lock;

        struct napi_struct napi;
        struct delayed_work dwork;
	struct work_struct  ui_worker;
        struct timer_list fec_timer;
        struct altera_dma_private dma_priv;
        struct phylink_config phylink_config;
        struct intel_fpga_tod_private *ptp_priv;

        phy_interface_t phy_iface;

} intel_fpga_xtile_eth_private;

/* RX FIFO Address Space
 */
struct intel_fpga_rx_fifo {
	u32 fill_level;					//0x00
	u32 reserved;					//0x04
	u32 almost_full_threshold;			//0x08
	u32 almost_empty_threshold;			//0x0C
	u32 cut_through_threshold;			//0x10
	u32 drop_on_error;				//0x14
};

#define rx_fifo_csroffs(a)	(offsetof(struct intel_fpga_rx_fifo, a))

#define tx_fifo_csroffs(a)	(offsetof(struct intel_fpga_rx_fifo, a))

#endif
