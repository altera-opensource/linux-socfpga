// SPDX-License-Identifier: GPL-2.0-only
/* Altera Triple-Speed Ethernet (TSE) variant for the Intel FPGA xtile
 * pluggable Ethernet driver framework.
 *
 * This file implements the variant-specific bits (MAC register init,
 * MDIO bus, phylink wiring, multicast filtering, link state callbacks).
 *
 * Copyright (C) 2026 Altera Corporation.
 *
 * The MAC register layout, CSR accessors and MAC statistics handling are
 * derived from the in-tree Altera Triple-Speed Ethernet MAC driver
 * (drivers/net/ethernet/altera/altera_tse*),
 * Copyright (C) 2008-2014 Altera Corporation, with original contributors:
 *   Dalon Westergreen, Thomas Chou, Ian Abbott, Yuriy Kozlov, Tobias Klauser,
 *   Andriy Smolskyy, Roman Bulgakov, Dmytro Mytarchuk, Matthew Gerlach.
 *   Original driver contributed by SLS; major updates by GlobalLogic.
 *
 * The PCS bring-up (regmap -> mdio-regmap -> Lynx PCS) follows the current
 * upstream altera_tse conversion.
 *
 * Contributors:
 *   Mahesh Vaidya
 *   Krishna Kumar S R
 *   Preetam Narayan
 *   Lubana.Badakar
 */

#include <linux/atomic.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/mdio/mdio-regmap.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/pcs-lynx.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "altera_eth_dma.h"
#include "altera_tse.h"
#include "altera_utils.h"
#include "intel_fpga_eth_main.h"
#include "intel_fpga_eth_xtile_tse.h"

/* phy_addr sentinel: scan the local MDIO bus for the first responsive PHY */
#define TSE_POLL_PHY		(-1)

/* Watchdog count for command_config SW_RESET self-clear */
#ifndef ALTERA_TSE_SW_RESET_WATCHDOG_CNTR
#define ALTERA_TSE_SW_RESET_WATCHDOG_CNTR	10000
#endif

/* Per-driver instance counter used to disambiguate MDIO bus IDs when
 * multiple TSE instances are present in the same system.
 */
static atomic_t tse_instance_count = ATOMIC_INIT(~0);

/*
 * Small helpers
 */

static inline struct intel_fpga_tse_private *
tse_tilepriv(intel_fpga_xtile_eth_private *priv)
{
	return priv->intel_fpga_tile_private;
}

/*
 * MDIO bus
 *
 * The TSE MAC contains a Clause 22 only MDIO master. PHY accesses are
 * performed by writing the PHY address into mdio_phy1_addr, then
 * reading/writing the 32 registers in MDIO Space 1 (dword offsets
 * 0xA0..0xBF in the MAC CSR space). Each register access on the AVMM side
 * blocks until the MDIO transaction completes, so MDIO callbacks MUST NOT
 *  be invoked from atomic context.
 *
 * NOTE: MDIO Space 0 (dword 0x80..0x9F) is occupied by the integrated PCS
 * register file in MAC+PCS variants. We use Space 1 for the external PHY.
 *
 * Clause 45 support
 * -----------------
 * The TSE MDIO master is Clause 22 only on the wire (no MDIO Manageable
 * Device with the C45 ST=00 frame format). To reach the MMDs of a C45
 * PHYs, we use the standard 802.3 Clause 22 MMD-indirect mechanism via
 * registers MII_MMD_CTRL (0x0D) and MII_MMD_DATA (0x0E):
 *
 *   1. C22 write devad to MII_MMD_CTRL    (function = Address, top 2 bits 00)
 *   2. C22 write reg  to MII_MMD_DATA     (16-bit C45 register pointer)
 *   3. C22 write devad | NOINCR to MII_MMD_CTRL (function = Data, top 2 = 01)
 *   4. C22 read or write MII_MMD_DATA     (actual C45 register access)
 *
 */

static int intel_fpga_tse_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct net_device *ndev = bus->priv;
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	/* C22 read. The framework dispatches Clause 45 accesses to read_c45
	 * via a separate callback path, so we never see C45 addresses here.
	 */
	csrwr32((mii_id & 0x1f), tp->mac_dev, tse_csroffs(mdio_phy1_addr));
	return csrrd32(tp->mac_dev, tse_csroffs(mdio_phy1) + regnum * 4) & 0xffff;
}

static int intel_fpga_tse_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				     u16 value)
{
	struct net_device *ndev = bus->priv;
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	csrwr32((mii_id & 0x1f), tp->mac_dev, tse_csroffs(mdio_phy1_addr));
	csrwr32(value, tp->mac_dev, tse_csroffs(mdio_phy1) + regnum * 4);
	return 0;
}

/* C22 MMD-indirect address phase.
 *
 * Performs the three C22 writes that point the MMD at the requested
 * (devad, regnum) tuple, leaving the C45 register at MII_MMD_DATA ready
 * for a single C22 read or write. The MDIO bus lock is held by the
 * kernel framework while the read_c45 / write_c45 callback is running,
 * so the four C22 transactions issued here are serialized against any
 * other access on this bus.
 *
 * We re-set mdio_phy1_addr at the start because some C22 transactions
 * elsewhere on the bus may have changed it; doing so makes each MMD
 * sequence self-contained.
 */
static void tse_mmd_indirect_addr(struct intel_fpga_tse_private *tp,
				  int phyad, int devad, u16 regnum)
{
	/* Latch the PHY address for all subsequent transactions */
	csrwr32(phyad & 0x1f, tp->mac_dev, tse_csroffs(mdio_phy1_addr));

	/* Step 1: function = Address, devad */
	csrwr32(devad & 0x1f, tp->mac_dev,
		tse_csroffs(mdio_phy1) + MII_MMD_CTRL * 4);

	/* Step 2: write the C45 register offset into MII_MMD_DATA */
	csrwr32(regnum, tp->mac_dev,
		tse_csroffs(mdio_phy1) + MII_MMD_DATA * 4);

	/* Step 3: function = Data (no post-increment) | devad */
	csrwr32(MII_MMD_CTRL_NOINCR | (devad & 0x1f), tp->mac_dev,
		tse_csroffs(mdio_phy1) + MII_MMD_CTRL * 4);
}

static int intel_fpga_tse_mdio_read_c45(struct mii_bus *bus, int phyad,
					int devad, int regnum)
{
	struct net_device *ndev = bus->priv;
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (devad < 0 || devad > 31 || regnum < 0 || regnum > 0xffff)
		return -EINVAL;

	tse_mmd_indirect_addr(tp, phyad, devad, (u16)regnum);

	/* Step 4: read the data from MII_MMD_DATA. */
	return csrrd32(tp->mac_dev,
		       tse_csroffs(mdio_phy1) + MII_MMD_DATA * 4) & 0xffff;
}

static int intel_fpga_tse_mdio_write_c45(struct mii_bus *bus, int phyad,
					 int devad, int regnum, u16 value)
{
	struct net_device *ndev = bus->priv;
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (devad < 0 || devad > 31 || regnum < 0 || regnum > 0xffff)
		return -EINVAL;

	tse_mmd_indirect_addr(tp, phyad, devad, (u16)regnum);
	csrwr32(value, tp->mac_dev, tse_csroffs(mdio_phy1) + MII_MMD_DATA * 4);
	return 0;
}

static int intel_fpga_tse_mdio_create(struct net_device *ndev, unsigned int id)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	struct device_node *mdio_node = NULL;
	struct device_node *child = NULL;
	struct mii_bus *mdio;
	int ret;

	for_each_child_of_node(priv->device->of_node, child) {
		if (of_device_is_compatible(child, "altr,tse-mdio")) {
			mdio_node = child;
			break;
		}
	}

	if (!mdio_node) {
		netdev_dbg(ndev, "no MDIO subnode in device tree\n");
		return 0;
	}

	mdio = mdiobus_alloc();
	if (!mdio) {
		dev_err(priv->device, "mdiobus_alloc failed\n");
		ret = -ENOMEM;
		goto put_node;
	}

	mdio->name      = INTEL_FPGA_TSE_RESOURCE_NAME;
	mdio->read      = &intel_fpga_tse_mdio_read;
	mdio->write     = &intel_fpga_tse_mdio_write;
	mdio->read_c45  = &intel_fpga_tse_mdio_read_c45;
	mdio->write_c45 = &intel_fpga_tse_mdio_write_c45;

	/* C22 + C45-via-MMD-indirect: the bus is wire-level Clause 22 only,
	 * but we transparently translate C45 accesses via the MMD-indirect
	 * dance in read_c45 / write_c45 above.
	 */

	snprintf(mdio->id, MII_BUS_ID_SIZE, "%s-%u", mdio->name, id);
	mdio->priv   = ndev;
	mdio->parent = priv->device;

	ret = of_mdiobus_register(mdio, mdio_node);
	if (ret) {
		dev_err(priv->device, "of_mdiobus_register %s failed: %d\n",
			mdio->id, ret);
		goto free_mdio;
	}

	of_node_put(mdio_node);
	dev_info(priv->device, "MDIO bus %s registered (C22 + C45-via-MMD)\n",
		 mdio->id);

	tp->mdio = mdio;
	return 0;

free_mdio:
	mdiobus_free(mdio);
put_node:
	of_node_put(mdio_node);
	return ret;
}

static void intel_fpga_tse_mdio_destroy(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (!tp->mdio)
		return;

	netdev_info(ndev, "MDIO bus %s removed\n", tp->mdio->id);
	mdiobus_unregister(tp->mdio);
	mdiobus_free(tp->mdio);
	tp->mdio = NULL;
}

static int intel_fpga_tse_phy_get_addr_mdio_create(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	struct device_node *np = priv->device->of_node;
	u32 phy_addr;
	int ret;

	ret = of_get_phy_mode(np, &priv->phy_iface);
	if (ret) {
		/* No phy-mode property => no PHY connection desired. */
		return 0;
	}

	/* phy_addr is signed (TSE_POLL_PHY/-1 => auto-poll), so read into a u32
	 * temp to match of_property_read_u32()'s output type, then assign.
	 */
	if (of_property_read_u32(np, "phy-addr", &phy_addr))
		tp->phy_addr = TSE_POLL_PHY;
	else
		tp->phy_addr = phy_addr;

	if (!(tp->phy_addr == TSE_POLL_PHY ||
	      (tp->phy_addr >= 0 && tp->phy_addr < PHY_MAX_ADDR))) {
		dev_err(priv->device, "invalid phy-addr %d\n", tp->phy_addr);
		return -ENODEV;
	}

	return intel_fpga_tse_mdio_create(ndev,
					  atomic_add_return(1, &tse_instance_count));
}

/*
 * MAC initialization, reset, enable/disable
 *
 * All routines that mutate command_config take priv->mac_cfg_lock.  Callers
 * already holding the lock should call the "_locked" variants directly.
 *
 */

void intel_fpga_tse_update_mac_addr(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	const u8 *addr = priv->dev->dev_addr;
	u32 msb, lsb;

	/* mac_0 = [addr3 addr2 addr1 addr0] (reversed byte order in the
	 * low 32 bits), mac_1 = [_ _ addr5 addr4].
	 */
	msb = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	lsb = ((addr[5] << 8) | addr[4]) & 0xffff;

	csrwr32(msb, tp->mac_dev, tse_csroffs(mac_addr_0));
	csrwr32(lsb, tp->mac_dev, tse_csroffs(mac_addr_1));
}

/* Caller must hold priv->mac_cfg_lock */
static int tse_reset_mac_locked(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	int counter;
	u32 dat;

	dat = csrrd32(tp->mac_dev, tse_csroffs(command_config));
	dat &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);
	dat |= MAC_CMDCFG_SW_RESET | MAC_CMDCFG_CNT_RESET;
	csrwr32(dat, tp->mac_dev, tse_csroffs(command_config));

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_clear(tp->mac_dev, tse_csroffs(command_config),
				     MAC_CMDCFG_SW_RESET))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		/* SW reset did not self-clear; force-clear and report timeout.
		 * This can legitimately happen when no line clocks are present
		 * (e.g., cable unplugged).
		 */
		dat = csrrd32(tp->mac_dev, tse_csroffs(command_config));
		dat &= ~MAC_CMDCFG_SW_RESET;
		csrwr32(dat, tp->mac_dev, tse_csroffs(command_config));
		return -ETIMEDOUT;
	}
	return 0;
}

/* tile.reset callback */
int intel_fpga_tse_reset(intel_fpga_xtile_eth_private *priv,
			 bool tx, bool rx, bool sys)
{
	int ret;

	spin_lock(&priv->mac_cfg_lock);
	ret = tse_reset_mac_locked(priv);
	spin_unlock(&priv->mac_cfg_lock);
	return ret;
}

/* Caller must hold priv->mac_cfg_lock */
static int tse_init_mac_locked(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	u32 cmd, frm_length;

	/* --- FIFO buffer thresholds --- */
	csrwr32(priv->rx_fifo_depth - ALTERA_TSE_RX_SECTION_EMPTY,
		tp->mac_dev, tse_csroffs(rx_section_empty));
	csrwr32(ALTERA_TSE_RX_SECTION_FULL,  tp->mac_dev, tse_csroffs(rx_section_full));
	csrwr32(ALTERA_TSE_RX_ALMOST_EMPTY,  tp->mac_dev, tse_csroffs(rx_almost_empty));
	csrwr32(ALTERA_TSE_RX_ALMOST_FULL,   tp->mac_dev, tse_csroffs(rx_almost_full));

	csrwr32(priv->tx_fifo_depth - ALTERA_TSE_TX_SECTION_EMPTY,
		tp->mac_dev, tse_csroffs(tx_section_empty));
	csrwr32(ALTERA_TSE_TX_SECTION_FULL,  tp->mac_dev, tse_csroffs(tx_section_full));
	csrwr32(ALTERA_TSE_TX_ALMOST_EMPTY,  tp->mac_dev, tse_csroffs(tx_almost_empty));
	csrwr32(ALTERA_TSE_TX_ALMOST_FULL,   tp->mac_dev, tse_csroffs(tx_almost_full));

	/* --- MAC address --- */
	intel_fpga_tse_update_mac_addr(priv);

	/* --- Frame length, IPG, pause quanta --- */
	frm_length = ETH_HLEN + priv->dev->mtu + ETH_FCS_LEN;
	csrwr32(frm_length, tp->mac_dev, tse_csroffs(frm_length));
	csrwr32(ALTERA_TSE_TX_IPG_LENGTH, tp->mac_dev, tse_csroffs(tx_ipg_length));
	csrwr32(ALTERA_TSE_PAUSE_QUANTA,  tp->mac_dev, tse_csroffs(pause_quanta));

	/* RX_SHIFT16 on receive path to 16-bit-align IP payloads in skb data;
	 * TX_SHIFT16 left clear since the core hands us 32-bit-aligned frames.
	 * OMIT_CRC left clear: MAC computes/inserts FCS on Tx.
	 */
	tse_set_bit(tp->mac_dev, tse_csroffs(rx_cmd_stat),
		    ALTERA_TSE_RX_CMD_STAT_RX_SHIFT16);
	tse_clear_bit(tp->mac_dev, tse_csroffs(tx_cmd_stat),
		      ALTERA_TSE_TX_CMD_STAT_TX_SHIFT16 |
		      ALTERA_TSE_TX_CMD_STAT_OMIT_CRC);

	/* --- command_config: feature bits, leave TX/RX disabled --- */
	cmd  = csrrd32(tp->mac_dev, tse_csroffs(command_config));
	cmd &= ~MAC_CMDCFG_PAD_EN;	/* keep padding bytes on receive */
	cmd &= ~MAC_CMDCFG_CRC_FWD;	/* strip FCS before delivering Rx */
	cmd |=  MAC_CMDCFG_RX_ERR_DISC;	/* drop frames with CRC errors */
	cmd |=  MAC_CMDCFG_CNTL_FRM_ENA;
	cmd &= ~MAC_CMDCFG_TX_ENA;
	cmd &= ~MAC_CMDCFG_RX_ENA;

	/* Speed/duplex bits initialised to 0; phylink mac_link_up will set
	 * ETH_SPEED / ENA_10 / HD_ENA according to the negotiated link.
	 * (Note: in SGMII the PCS handles the wire-rate adaptation by byte
	 * replication; these MAC-side bits do not change the SGMII signaling.
	 * They affect the eth_mode and set_1000/set_10 output strobes, which
	 * may be used by board glue logic.)
	 */
	cmd &= ~MAC_CMDCFG_HD_ENA;
	cmd &= ~MAC_CMDCFG_ETH_SPEED;
	cmd &= ~MAC_CMDCFG_ENA_10;

	csrwr32(cmd, tp->mac_dev, tse_csroffs(command_config));

	if (netif_msg_hw(priv))
		dev_dbg(priv->device,
			"TSE: command_config post-init = 0x%08x\n", cmd);

	return 0;
}

/* tile.deassert_reset callback: reset MAC then load defaults.
 * Called once from xtile_open before DMA/IRQ are wired up.
 */
int intel_fpga_tse_deassert_reset(intel_fpga_xtile_eth_private *priv)
{
	int ret;

	spin_lock(&priv->mac_cfg_lock);
	ret = tse_reset_mac_locked(priv);
	/* tse_reset_mac_locked may legitimately time out if line clocks are
	 * not yet present. Log but continue: register init still proceeds.
	 */
	if (ret)
		dev_dbg(priv->device, "MAC SW reset timed out (%d); continuing\n",
			ret);

	ret = tse_init_mac_locked(priv);
	spin_unlock(&priv->mac_cfg_lock);
	if (ret)
		netdev_err(priv->dev, "tse_init_mac_locked failed: %d\n", ret);
	return ret;
}

/* Enable / disable MAC datapath. Caller must hold priv->mac_cfg_lock. */
static void tse_set_mac_locked(intel_fpga_xtile_eth_private *priv, bool enable)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	u32 v;

	v = csrrd32(tp->mac_dev, tse_csroffs(command_config));
	if (enable)
		v |=  (MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);
	else
		v &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);
	csrwr32(v, tp->mac_dev, tse_csroffs(command_config));
}

/* tile.start callback: invoked by the link monitor when link goes up. */
int intel_fpga_tse_start(intel_fpga_xtile_eth_private *priv)
{
	spin_lock(&priv->mac_cfg_lock);
	tse_set_mac_locked(priv, true);
	spin_unlock(&priv->mac_cfg_lock);
	return 0;
}

/* tile.stop callback: invoked by the link monitor when link goes down. */
int intel_fpga_tse_stop(intel_fpga_xtile_eth_private *priv)
{
	spin_lock(&priv->mac_cfg_lock);
	tse_set_mac_locked(priv, false);
	spin_unlock(&priv->mac_cfg_lock);
	return 0;
}

/*
 * Multicast / Rx filter
 */

static void tse_set_mcfilter(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	struct netdev_hw_addr *ha;
	int i;

	for (i = 0; i < 64; i++)
		csrwr32(0, tp->mac_dev, tse_csroffs(hash_table) + i * 4);

	netdev_for_each_mc_addr(ha, ndev) {
		unsigned int hash = 0;
		int mac_octet;

		for (mac_octet = 5; mac_octet >= 0; mac_octet--) {
			unsigned char xor_bit = 0;
			unsigned char octet = ha->addr[mac_octet];
			unsigned int bitshift;

			for (bitshift = 0; bitshift < 8; bitshift++)
				xor_bit ^= ((octet >> bitshift) & 0x01);

			hash = (hash << 1) | xor_bit;
		}
		csrwr32(1, tp->mac_dev, tse_csroffs(hash_table) + hash * 4);
	}
}

static void tse_set_mcfilterall(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	int i;

	for (i = 0; i < 64; i++)
		csrwr32(1, tp->mac_dev, tse_csroffs(hash_table) + i * 4);
}

void intel_fpga_tse_set_rx_mode_hashfilter(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	spin_lock(&priv->mac_cfg_lock);

	if (ndev->flags & IFF_PROMISC)
		tse_set_bit(tp->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);
	else
		tse_clear_bit(tp->mac_dev, tse_csroffs(command_config),
			      MAC_CMDCFG_PROMIS_EN);

	if (ndev->flags & IFF_ALLMULTI)
		tse_set_mcfilterall(ndev);
	else
		tse_set_mcfilter(ndev);

	spin_unlock(&priv->mac_cfg_lock);
}

void intel_fpga_tse_set_rx_mode(struct net_device *ndev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	spin_lock(&priv->mac_cfg_lock);

	if ((ndev->flags & IFF_PROMISC) || (ndev->flags & IFF_ALLMULTI) ||
	    !netdev_mc_empty(ndev) || !netdev_uc_empty(ndev))
		tse_set_bit(tp->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);
	else
		tse_clear_bit(tp->mac_dev, tse_csroffs(command_config),
			      MAC_CMDCFG_PROMIS_EN);

	spin_unlock(&priv->mac_cfg_lock);
}

/* tile.set_rx_mode entry point: dispatches to the hash-filter or plain
 * implementation based on the DT-derived hash_filter flag. Called by the
 * core's xtile_set_rx_mode through priv->spec_ops->tile.set_rx_mode.
 */
void intel_fpga_tse_dispatch_set_rx_mode(struct net_device *dev)
{
	intel_fpga_xtile_eth_private *priv = netdev_priv(dev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (tp->hash_filter)
		intel_fpga_tse_set_rx_mode_hashfilter(dev);
	else
		intel_fpga_tse_set_rx_mode(dev);
}

/*
 * Phylink wiring
 *
 * The TSE PCS (small TSE with 1000BASE-X/SGMII PCS) is a memory-mapped instance
 * of the Lynx PCS, so it is driven by the Lynx PCS driver
 * (drivers/net/pcs/pcs-lynx.c) over an mdio-regmap shim. We create the
 * phylink_pcs handle via lynx_pcs_create_mdiodev() (see tse_pcs_phylink_create)
 * and return it from mac_select_pcs.
 *
 * We do NOT enable TX_ENA / RX_ENA in mac_link_up because the xtile
 * framework's link monitor calls tile.start (which enables the MAC) on
 * the link-up edge. mac_link_up only updates the speed/duplex bits.
 *
 * The link_up flag in tile_priv is maintained from these callbacks and
 * read by intel_fpga_tse_get_link_fault_status to feed the monitor.
 */

static void tse_mac_config(struct phylink_config *config, unsigned int mode,
			   const struct phylink_link_state *state)
{
	/* Nothing to do here.
	 *
	 * Interface-mode setup (SGMII vs 1000BASE-X), link timer, IF_MODE and
	 * BMCR autoneg-enable are handled by lynx_pcs_config() in
	 * drivers/net/pcs/pcs-lynx.c, invoked by phylink via the phylink_pcs
	 * handle returned from tse_mac_select_pcs().
	 *
	 * Resolved speed / duplex / pause are programmed in tse_mac_link_up().
	 */
}

static void tse_mac_link_down(struct phylink_config *config,
			      unsigned int mode, phy_interface_t interface)
{
	struct net_device *ndev = to_net_dev(config->dev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	/* WRITE_ONCE paired with READ_ONCE in intel_fpga_tse_get_link_fault_status
	 * which is read from the eth_monitor_link_status workqueue context.
	 */
	WRITE_ONCE(tp->link_up, false);

	/* The actual MAC disable (clearing TX_ENA / RX_ENA) is performed by
	 * intel_fpga_tse_stop, invoked by the framework's link monitor
	 * (eth_monitor_link_status) on the next poll once link_fault_status
	 * reports the link as down.
	 */
}

static void tse_mac_link_up(struct phylink_config *config, struct phy_device *phy,
			    unsigned int mode, phy_interface_t interface,
			    int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct net_device *ndev = to_net_dev(config->dev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	u32 ctrl;

	/* Update MAC speed/duplex select bits in command_config.
	 *
	 * Per User Guide Section SGMII init sequence and Section
	 * (1000BASE-X init sequence), the Note states:
	 *
	 *   "If 1000BASE-X/SGMII PCS is initialized, set the ETH_SPEED (bit 3)
	 *    and ENA_10 (bit 25) in command_config register to 0. If half
	 *    duplex is reported in the PHY/PCS status register, set the
	 *    HD_ENA (bit 10) to 1 in command_config register."
	 *
	 * Rationale : the SGMII PCS sits between the MAC's fixed-rate
	 * GMII interface and the 1.25 Gbps SGMII serial link, and
	 * performs byte replication (10x at 10 Mbps, 100x at 100 Mbps) to
	 * adapt the line rate. The MAC therefore runs at gigabit GMII clock
	 * regardless of the negotiated copper speed, and ETH_SPEED/ENA_10
	 * (which configure the MAC's own clocking and IFG generation) must
	 * remain at 0.
	 *
	 */
	spin_lock(&priv->mac_cfg_lock);

	ctrl  = csrrd32(tp->mac_dev, tse_csroffs(command_config));
	ctrl &= ~(MAC_CMDCFG_ENA_10 | MAC_CMDCFG_ETH_SPEED | MAC_CMDCFG_HD_ENA);

	if (duplex == DUPLEX_HALF)
		ctrl |= MAC_CMDCFG_HD_ENA;

	if (interface == PHY_INTERFACE_MODE_SGMII ||
	    interface == PHY_INTERFACE_MODE_1000BASEX) {
		/* SGMII / 1000BASE-X: leave ETH_SPEED and ENA_10 at 0
		 * The PCS handles rate adaptation; the MAC stays in
		 * gigabit GMII clocking.
		 */
	} else {
		/* Non-SGMII (e.g. RGMII direct-to-copper). Apply the
		 * upstream-style speed select. Our current variant doesn't
		 * advertise these modes, but the code is left in for forward
		 * compatibility if a future board adds an RGMII variant.
		 */
		if (speed == SPEED_1000)
			ctrl |= MAC_CMDCFG_ETH_SPEED;
		else if (speed == SPEED_10)
			ctrl |= MAC_CMDCFG_ENA_10;
		/* SPEED_100 => both ETH_SPEED and ENA_10 left clear */
	}

	csrwr32(ctrl, tp->mac_dev, tse_csroffs(command_config));

	spin_unlock(&priv->mac_cfg_lock);

	/* Cache resolved speed/duplex on priv so the core's link-up log
	 * message (phy_speed_to_str / phy_duplex_to_str) reports correctly.
	 */
	priv->link_speed = speed;
	priv->duplex     = duplex;

	WRITE_ONCE(tp->link_up, true);
}

static struct phylink_pcs *tse_mac_select_pcs(struct phylink_config *config,
					      phy_interface_t interface)
{
	struct net_device *ndev = to_net_dev(config->dev);
	intel_fpga_xtile_eth_private *priv = netdev_priv(ndev);
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (interface == PHY_INTERFACE_MODE_SGMII ||
	    interface == PHY_INTERFACE_MODE_1000BASEX)
		return tp->pcs;
	return NULL;
}

static const struct phylink_mac_ops intel_fpga_tse_phylink_ops = {
	.mac_select_pcs = tse_mac_select_pcs,
	.mac_config	= tse_mac_config,
	.mac_link_down	= tse_mac_link_down,
	.mac_link_up	= tse_mac_link_up,
};

/* tile.link_fault_status: returns true when phylink reports link up.
 * The xtile framework's eth_monitor_link_status will then call tile.start
 * (which enables TX_ENA/RX_ENA) and run eth_link_up to wake the Tx queues
 * and the NAPI poll loop.
 */
bool intel_fpga_tse_get_link_fault_status(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	return READ_ONCE(tp->link_up);
}

static int tse_phylink_connect(intel_fpga_xtile_eth_private *priv)
{
	if (priv->phy_iface == PHY_INTERFACE_MODE_NA)
		return 0;	/* no PHY (e.g. serial-only test) */

	return phylink_of_phy_connect(priv->phylink, priv->device->of_node, 0);
}

static void tse_pcs_destroy_action(void *pcs)
{
	lynx_pcs_destroy(pcs);
}

/*
 * tile.init / tile.uninit
 *
 * Called from xtile_open / xtile_shutdown (the ndo_open / ndo_stop path).
 * MAC register defaults were loaded earlier from tile.deassert_reset.
 * Here we connect the PHY through phylink and start its state machine.
 */

/* Create the PCS and phylink instances (called from tile.check_dts_param).
 *
 * The Altera TSE PCS is a memory-mapped instance of the Lynx PCS, so we wrap
 * the PCS register window in a regmap, expose it as an MDIO bus via mdio-regmap,
 * and let the Lynx PCS driver manage it. The regmap and the shim MDIO bus are
 * devm-managed on the *platform* device (priv->device == &pdev->dev), which is
 * valid at probe time.
 *
 */
static int tse_pcs_phylink_create(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	struct net_device *ndev = priv->dev;
	struct regmap_config pcs_rc = {};
	struct mdio_regmap_config mrc = {};
	struct regmap *pcs_regmap;
	struct mii_bus *pcs_bus;
	int ret;

	/* PCS register access width: 4-byte (32-bit AVMM) or 2-byte stride.
	 * 16-bit register values in both cases; reg_shift converts the MDIO
	 * register index into the byte offset within the PCS window.
	 */
	if (tp->pcs_reg_width == 4) {
		pcs_rc.reg_bits  = 32;
		pcs_rc.val_bits  = 16;
		pcs_rc.reg_shift = REGMAP_UPSHIFT(2);
	} else {
		pcs_rc.reg_bits  = 16;
		pcs_rc.val_bits  = 16;
		pcs_rc.reg_shift = REGMAP_UPSHIFT(1);
	}

	pcs_regmap = devm_regmap_init_mmio(priv->device, tp->pcs_base, &pcs_rc);
	if (IS_ERR(pcs_regmap)) {
		ret = PTR_ERR(pcs_regmap);
		dev_err(priv->device, "PCS regmap init failed: %d\n", ret);
		return ret;
	}

	/* Expose the PCS regmap as an MDIO bus so the Lynx PCS (an MDIO-attached
	 * driver) can reach the memory-mapped PCS. The bus only answers at
	 * valid_addr (0), and autoscan=false means it does not auto-probe; the
	 * PCS mdiodev is created explicitly at addr 0 below.
	 */
	mrc.regmap     = pcs_regmap;
	mrc.parent     = priv->device;
	mrc.valid_addr = 0x0;
	mrc.autoscan   = false;
	/* Use the resolved platform-device name (ndev->name is still "eth%d"
	 * here, pre-register_netdev) so the MDIO bus id is unique per instance.
	 */
	snprintf(mrc.name, MII_BUS_ID_SIZE, "%s-pcs-mii", dev_name(priv->device));

	pcs_bus = devm_mdio_regmap_register(priv->device, &mrc);
	if (IS_ERR(pcs_bus)) {
		ret = PTR_ERR(pcs_bus);
		dev_err(priv->device, "PCS mdio-regmap register failed: %d\n", ret);
		return ret;
	}

	tp->pcs = lynx_pcs_create_mdiodev(pcs_bus, 0);
	if (IS_ERR(tp->pcs)) {
		ret = PTR_ERR(tp->pcs);
		tp->pcs = NULL;
		dev_err(priv->device, "lynx_pcs_create_mdiodev failed: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(priv->device, tse_pcs_destroy_action, tp->pcs);
	if (ret) {
		tp->pcs = NULL;   /* _or_reset already called lynx_pcs_destroy */
		return ret;
	}

	/* Phylink. */
	priv->phylink_config.dev	      = &ndev->dev;
	priv->phylink_config.type	      = PHYLINK_NETDEV;
	priv->phylink_config.mac_capabilities =
		MAC_SYM_PAUSE | MAC_10 | MAC_100 | MAC_1000FD;

	__set_bit(PHY_INTERFACE_MODE_SGMII,
		  priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_1000BASEX,
		  priv->phylink_config.supported_interfaces);
	/* The small TSE MAC supports MII/GMII/RGMII too. Advertise them only if
	 * the board really wires them up by setting phy-mode in DT accordingly.
	 * Conservative defaults here.
	 */
	__set_bit(PHY_INTERFACE_MODE_MII,
		  priv->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_GMII,
		  priv->phylink_config.supported_interfaces);
	phy_interface_set_rgmii(priv->phylink_config.supported_interfaces);

	priv->phylink = phylink_create(&priv->phylink_config,
				       of_fwnode_handle(priv->device->of_node),
				       priv->phy_iface,
				       &intel_fpga_tse_phylink_ops);
	if (IS_ERR(priv->phylink)) {
		ret = PTR_ERR(priv->phylink);
		priv->phylink = NULL;
		dev_err(priv->device, "phylink_create failed: %d\n", ret);
		tp->pcs = NULL;
		return ret;
	}

	return 0;
}

int intel_fpga_tse_init(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);
	int ret;

	/* PCS + phylink were created at probe (tile.check_dts_param). Here we
	 * only (re)connect the PHY and start the phylink state machine.
	 */

	/* Refresh primary MAC address (it may have been changed by userspace
	 * while the interface was down) and clear stats.
	 */
	spin_lock(&priv->mac_cfg_lock);
	intel_fpga_tse_update_mac_addr(priv);
	tse_set_bit(tp->mac_dev, tse_csroffs(command_config),
		    MAC_CMDCFG_CNT_RESET);
	spin_unlock(&priv->mac_cfg_lock);

	WRITE_ONCE(tp->link_up, false);

	ret = tse_phylink_connect(priv);
	if (ret)
		return ret;

	if (priv->phylink)
		phylink_start(priv->phylink);

	return 0;
}

int intel_fpga_tse_uninit(intel_fpga_xtile_eth_private *priv)
{
	struct intel_fpga_tse_private *tp = tse_tilepriv(priv);

	if (priv->phylink) {
		phylink_stop(priv->phylink);
		phylink_disconnect_phy(priv->phylink);
	}

	WRITE_ONCE(tp->link_up, false);

	/* Belt-and-braces: ensure MAC datapath is disabled. */
	intel_fpga_tse_stop(priv);

	/* SW-reset the MAC so the link partner sees link-down.
	 *
	 * On the TSE IP, asserting SW_RESET in command_config also resets the
	 * embedded PCS. The PCS's SGMII output toward the PHY goes invalid,
	 * the PHY loses host-port carrier, and the copper link drops — which
	 * is what the link partner needs to see
	 *
	 * On the next ifup, tile.deassert_reset performs another SW reset and
	 * full register initialisation, so the MAC ends up correctly set up
	 * for the new session.
	 */
	spin_lock(&priv->mac_cfg_lock);
	tse_reset_mac_locked(priv);
	spin_unlock(&priv->mac_cfg_lock);

	return 0;
}

/*
 * tile.check_dts_param / tile.init / tile.uninit / tile.remove
 *
 * Called from the core's probe (intel_fpga_xtile_probe). All one-time setup --
 * including PCS and phylink creation -- lives in tile.check_dts_param. The PCS
 * is the Lynx PCS driven over an mdio-regmap shim, and both the regmap and the
 * shim MDIO bus are devm-managed on the platform device (priv->device), which
 * is valid even though the core runs check_dts_param BEFORE register_netdev().
 * phylink_create() only kzalloc()s, so it is safe pre-register too.
 *
 * tile.check_dts_param responsibilities (probe-time, runs once):
 *   - Allocate the variant-private (intel_fpga_tse_private).
 *   - Map the MAC and PCS register windows.
 *   - Read FIFO depths, hash-filter and PCS-reg-width DT properties.
 *   - Read megacore_revision (logging).
 *   - Set ndev->mem_start / mem_end (informational).
 *   - Create the local MDIO bus and discover the PHY.
 *   - Create the PCS (regmap -> mdio-regmap -> lynx) and phylink.
 *   (rx-mode is served by the tile.set_rx_mode dispatcher registered in
 *    tse_data — intel_fpga_tse_dispatch_set_rx_mode; nothing is installed on
 *    the netdev here, and netdev_ops is not swapped.)
 *
 * tile.init responsibilities (runs on every ndo_open):
 *   - Refresh MAC address, CNT_RESET, clear link_up.
 *   - phylink_connect + phylink_start.
 *
 * tile.uninit responsibilities (runs on every ndo_stop):
 *   - phylink_stop + phylink_disconnect_phy; MAC SW reset to drop the link.
 *
 * tile.remove responsibilities (runs once at unbind):
 *   - phylink_destroy, lynx_pcs_destroy, mdiobus_unregister/free. tp, mac_dev,
 *     pcs_base, the PCS regmap and the shim MDIO bus are devm-managed.
 */

/* Helper: map a named "reg" resource on the platform device. */
static int tse_map_resource(struct platform_device *pdev, const char *name,
			    void __iomem **out)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(&pdev->dev, "missing resource '%s'\n", name);
		return -ENODEV;
	}
	*out = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(*out))
		return PTR_ERR(*out);
	return 0;
}

bool intel_fpga_tse_check_dts_param(intel_fpga_xtile_eth_private *priv)
{
	struct platform_device *pdev = to_platform_device(priv->device);
	struct net_device *ndev = priv->dev;
	struct intel_fpga_tse_private *tp;
	int ret;

	/* Allocate the variant private and publish it on priv before any
	 * code path that might read it back via tse_tilepriv().
	 */
	tp = devm_kzalloc(&pdev->dev, sizeof(*tp), GFP_KERNEL);
	if (!tp)
		return false;
	priv->intel_fpga_tile_private = tp;

	/* --- Map MAC register window --- */
	ret = tse_map_resource(pdev, "control_port",
			       (void __iomem **)&tp->mac_dev);
	if (ret) {
		netdev_err(ndev, "cannot map 'control_port' MAC CSR space: %d\n",
			   ret);
		return false;
	}

	/* --- Set Memory Info --- */
	ndev->mem_start = (unsigned long)tp->mac_dev;
	ndev->mem_end   = ndev->mem_start + sizeof(struct altera_tse_mac) - 1;

	/* --- Map PCS register window ---
	 * Two valid topologies:
	 *  a) PCS exposed as its own AVMM slave with reg name "pcs". Reg width
	 *     is reported in DT via "altr,pcs-reg-width" (default 4).
	 *  b) PCS aliased into the MAC CSR space at the MDIO Space 0 offset.
	 *     Reg width is always 4 in this case (32-bit AVMM).
	 */
	{
		struct resource *r = platform_get_resource_byname(pdev,
								  IORESOURCE_MEM,
								  "pcs");

		if (r) {
			tp->pcs_base = devm_ioremap_resource(&pdev->dev, r);
			if (IS_ERR(tp->pcs_base)) {
				ret = PTR_ERR(tp->pcs_base);
				netdev_err(ndev, "cannot map 'pcs': %d\n", ret);
				return false;
			}
			if (of_property_read_u32(pdev->dev.of_node,
						 "altr,pcs-reg-width",
						 &tp->pcs_reg_width))
				tp->pcs_reg_width = 4;
		} else {
			tp->pcs_base = (void __iomem *)((uintptr_t)tp->mac_dev +
						tse_csroffs(mdio_phy0));
			tp->pcs_reg_width = 4;
		}
	}

	/* --- FIFO depths --- */
	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
				 &priv->rx_fifo_depth)) {
		netdev_err(ndev, "rx-fifo-depth missing in DT\n");
		return false;
	}
	if (of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
				 &priv->tx_fifo_depth)) {
		netdev_err(ndev, "tx-fifo-depth missing in DT\n");
		return false;
	}

	/* Multicast hash filter present? Selects the ndo_set_rx_mode variant. */
	tp->hash_filter = of_property_read_bool(pdev->dev.of_node,
						"altr,has-hash-multicast-filter");

	/* MAC revision (for logging only). Safe to read here: tp->mac_dev was
	 * just ioremapped above.
	 */
	tp->revision = ioread32(&tp->mac_dev->megacore_revision);

	/* --- MDIO Creation --- */
	ret = intel_fpga_tse_phy_get_addr_mdio_create(ndev);
	if (ret) {
		netdev_err(ndev, "MDIO/PHY discovery failed: %d\n", ret);
		return false;
	}

	/* --- PCS + phylink ---
	 * Built on priv->device (&pdev->dev): the PCS regmap and the shim MDIO
	 * bus are devm there, so this runs at probe rather than the first ifup.
	 * priv->phy_iface was resolved by the MDIO/PHY discovery above.
	 */
	ret = tse_pcs_phylink_create(priv);
	if (ret) {
		dev_err(priv->device, "PCS/phylink setup failed: %d\n", ret);
		return false;
	}

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "Altera TSE %d.%d (PCS %d-bit)\n",
			 (tp->revision >> 8) & 0xff, tp->revision & 0xff,
			 tp->pcs_reg_width * 8);

	return true;
}

int intel_fpga_tse_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	intel_fpga_xtile_eth_private *priv;
	struct intel_fpga_tse_private *tp;

	if (!ndev)
		return 0;

	priv = netdev_priv(ndev);
	tp = tse_tilepriv(priv);

	if (priv->phylink) {
		phylink_destroy(priv->phylink);
		priv->phylink = NULL;
	}

	if (tp->pcs)
		tp->pcs = NULL;

	intel_fpga_tse_mdio_destroy(ndev);

	/* tp, pcs_base, mac_dev, the PCS regmap and the shim MDIO bus are all
	 * devm-managed; only phylink and the Lynx PCS need explicit teardown.
	 */
	return 0;
}
