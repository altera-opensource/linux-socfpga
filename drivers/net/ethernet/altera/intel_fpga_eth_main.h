
#include "altera_eth_dma.h"
#include "altera_msgdma_prefetcher.h"

#ifdef CONFIG_INTEL_FPGA_ETILE
#include "intel_fpga_eth_etile.h"
#endif

/* To be changed */

#define INTEL_FPGA_ETILE_ETH_RESOURCE_NAME "intel_fpga_etile"

/* To be changed */

#define INTEL_FPGA_XTILE_SW_RESET_WATCHDOG_CNTR              1000000

int fec_init(struct platform_device *pdev, struct intel_fpga_etile_eth_private *priv);
void intel_fpga_xtile_set_ethtool_ops(struct net_device *dev);
int xtile_check_counter_complete(struct intel_fpga_etile_eth_private *priv, u32 regbank,
                                 size_t offs, u8 bit_mask, bool set_bit,
                                 int align);

