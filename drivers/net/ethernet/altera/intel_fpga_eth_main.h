
#ifndef __INTEL_FPGA_ETH_MAIN_H__
#define __INTEL_FPGA_ETH_MAIN_H__

#include "altera_eth_dma.h"
#include "altera_msgdma_prefetcher.h"

#ifdef CONFIG_INTEL_FPGA_HSSI_ETILE
#include "intel_fpga_eth_etile.h"
#endif

/* To be changed */

#define INTEL_FPGA_ETILE_ETH_RESOURCE_NAME "intel_fpga_etile"

/* To be changed */

#define INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR              1000000

typedef struct intel_fpga_etile_eth_private intel_fpga_xtile_eth_private;

int fec_init(struct platform_device *pdev, struct intel_fpga_etile_eth_private *priv);
void intel_fpga_xtile_set_ethtool_ops(struct net_device *dev);

void init_qsfp_ctrl_space(intel_fpga_xtile_eth_private *priv);
void deinit_qsfp_ctrl_space(intel_fpga_xtile_eth_private *priv);

struct qsfp_ops {
        void (*qsfp_link_up)(intel_fpga_xtile_eth_private*);
        void (*qsfp_link_down)(intel_fpga_xtile_eth_private*);
};

struct xtile_spec_ops {
	const struct qsfp_ops *qsfp_ops;
	const struct altera_dmaops *dma_ops;
	void (*pma_digi_reset)(intel_fpga_xtile_eth_private*, bool, bool);
	void (*pma_ana_reset)(intel_fpga_xtile_eth_private*);
};

#endif
