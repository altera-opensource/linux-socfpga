
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
#include "intel_freq_control.h"
#include "intel_fpga_hssiss.h"
#define INTEL_FPGA_XTILE_ETH_RESOURCE_NAME "intel_fpga_eth"

#define INTEL_FPGA_RET_SUCCESS                          0
/* Flow Control defines */
#define FLOW_OFF        0
#define FLOW_RX         1
#define FLOW_TX         2
#define FLOW_ON         (FLOW_TX | FLOW_RX)

/* TX Flow Control */
#define MAC_PAUSEFRAME_QUANTA					0xFFFF

#define INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR              1000000
#define INTEL_FPGA_XTILE_ETH_RESOURCE_NAME "intel_fpga_eth"

#define INTEL_FPGA_BYTE_ALIGN   8
#define INTEL_FPGA_WORD_ALIGN   32

#define MOD_PARAM_PERM  0644

typedef enum {
	ETH_LINK_STATE_RESET = 0,
	ETH_LINK_STATE_START,
	ETH_LINK_STATE_STOP,
	ETH_LINK_STATE_RUN
} xtile_eth_link_state;

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
        u32 ptp_tx_routing_adj;
        u32 ptp_rx_routing_adj;
        u32 pma_lanes_used;
        u16 pma_type;
        u8  eth_rate;

        u8 duplex;
        u8 qsfp_lane;
        bool autoneg;
        bool ptp_enable;
        xtile_eth_link_state link_state;
        bool cable_unplugged;
        bool ui_enable;
        bool monitor_thread_enable;
        bool napi_state;
	bool netque_state;
	bool tx_irq_enabled;
	bool rx_irq_enabled;
	u64 irq_tx_enable_cntr;
	u64 irq_rx_enable_cntr;
	u64 irq_tx_disable_cntr;
	u64 irq_rx_disable_cntr;

	rwlock_t wr_lock;
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
        hssi_eth_port_attr hssi_port_x_attr;

        phy_interface_t phy_iface;
	u32 ptp_clockcleaner_enable;

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

// Function Prototypes
int etile_init_mac(intel_fpga_xtile_eth_private *priv);
void etile_get_stats64(struct net_device *dev,
		       struct rtnl_link_stats64 *storage);
void etile_update_mac_addr(intel_fpga_xtile_eth_private *priv);
int etile_ehip_reset(intel_fpga_xtile_eth_private *priv,
			bool tx, bool rx, bool sys);
int etile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv);
void intel_fpga_etile_set_ethtool_ops(struct net_device *netdev);



extern int ftile_ehip_reset(intel_fpga_xtile_eth_private *priv,
                        bool tx_reset, bool rx_reset, bool sys_reset);
extern int ftile_ehip_deassert_reset(intel_fpga_xtile_eth_private *priv);
extern int ftile_init(intel_fpga_xtile_eth_private *priv);
extern int ftile_uninit(intel_fpga_xtile_eth_private *priv);
extern int ftile_start(intel_fpga_xtile_eth_private *priv);
extern int ftile_stop(intel_fpga_xtile_eth_private *priv);
extern int ftile_run_check(intel_fpga_xtile_eth_private *priv);
extern void ftile_update_mac_addr(intel_fpga_xtile_eth_private *priv);
extern bool ftile_get_link_fault_status(intel_fpga_xtile_eth_private *priv);
extern void intel_fpga_ftile_set_ethtool_ops(struct net_device *dev);
extern void ftile_get_stats64(struct net_device *dev,
			      struct rtnl_link_stats64 *storage);

int xtile_check_counter_complete(intel_fpga_xtile_eth_private *priv,
                                 u32 regbank,
                                 size_t offs,
                                 u8 bit_mask,
                                 bool set_bit,
                                 int align);
#endif
