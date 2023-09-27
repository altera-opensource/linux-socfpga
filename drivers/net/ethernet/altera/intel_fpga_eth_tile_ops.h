struct xtile_spec_ops {
	const struct altera_dmaops *dma_ops;

	struct {
		int (*reset)(intel_fpga_xtile_eth_private*, bool tx, bool rx, bool sys);
		int (*deassert_reset)(intel_fpga_xtile_eth_private*);
		int (*init)(intel_fpga_xtile_eth_private*);
		int (*uninit)(intel_fpga_xtile_eth_private*);
		int (*start)(intel_fpga_xtile_eth_private*);
		int (*stop)(intel_fpga_xtile_eth_private*);
		int (*run_check)(intel_fpga_xtile_eth_private*);
		bool (*link_fault_status)(intel_fpga_xtile_eth_private*);
		void (*update_mac_addr)(intel_fpga_xtile_eth_private*);
		void (*net_stats)(struct net_device *, struct rtnl_link_stats64 *);
		 void (*reg_ethtool_ops) (struct net_device *netdev);
	} tile;

	bool (*link_check)(intel_fpga_xtile_eth_private*);
};


