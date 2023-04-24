#include "intel_fpga_eth_main.h"

struct xtile_spec_ops {
	const struct altera_dmaops *dma_ops;

	void (*link_up)(intel_fpga_xtile_eth_private*);
	void (*link_down)(intel_fpga_xtile_eth_private*);
	int (*ehip_reset_deassert)(intel_fpga_xtile_eth_private*);
	int (*ehip_reset)(intel_fpga_xtile_eth_private*, bool tx, bool rx, bool sys);

        struct mac_ops {
                void (*update_mac)(intel_fpga_xtile_eth_private *);
                int (*init_mac)(intel_fpga_xtile_eth_private *);
        } mac_ops;

	void (*net_stats)(struct net_device *, struct rtnl_link_stats64 *);

	struct eth_ops {
		void (*eth_reg_callback) (struct net_device *netdev);
	} eth_ops;

	void (*ptp_init)(intel_fpga_xtile_eth_private *);
};


