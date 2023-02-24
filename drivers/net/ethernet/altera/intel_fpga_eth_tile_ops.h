

struct xtile_spec_ops {
	const struct altera_dmaops *dma_ops;
        
	struct qsfp_ops {
                void (*qsfp_init)(intel_fpga_xtile_eth_private*);
                void (*qsfp_link_up)(intel_fpga_xtile_eth_private*);
                void (*qsfp_link_down)(intel_fpga_xtile_eth_private*);
                void (*qsfp_deinit)(intel_fpga_xtile_eth_private*);
        } qsfp_ops;

        struct reset_ops {
                void (*pma_ana_reset)(intel_fpga_xtile_eth_private*);
                void (*pma_digi_reset)(intel_fpga_xtile_eth_private*,
					bool, bool);
        } reset_ops;

        struct mac_ops {
                void (*update_mac)(intel_fpga_xtile_eth_private *);
                int (*init_mac)(intel_fpga_xtile_eth_private *);
        } mac_ops;

        struct stat_ops {
                void (*net_stats)(struct net_device *,
                                        struct rtnl_link_stats64 *);
        } stat_ops;

	struct eth_ops {
		void (*eth_reg_callback) (struct net_device *netdev);
	} eth_ops;
};
