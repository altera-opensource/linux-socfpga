/*******************************************************************************
 *
 * Intel 10 Gigabit PCI Express Linux driver
 * Copyright(c) 2017 Oracle and/or its affiliates. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Linux NICS <linux.nics@intel.com>
 * e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 ******************************************************************************/

#include "ixgbe.h"
#include <net/xfrm.h>
#include <crypto/aead.h>

/**
 * ixgbe_ipsec_set_tx_sa - set the Tx SA registers
 * @hw: hw specific details
 * @idx: register index to write
 * @key: key byte array
 * @salt: salt bytes
 **/
static void ixgbe_ipsec_set_tx_sa(struct ixgbe_hw *hw, u16 idx,
				  u32 key[], u32 salt)
{
	u32 reg;
	int i;

	for (i = 0; i < 4; i++)
		IXGBE_WRITE_REG(hw, IXGBE_IPSTXKEY(i), cpu_to_be32(key[3 - i]));
	IXGBE_WRITE_REG(hw, IXGBE_IPSTXSALT, cpu_to_be32(salt));
	IXGBE_WRITE_FLUSH(hw);

	reg = IXGBE_READ_REG(hw, IXGBE_IPSTXIDX);
	reg &= IXGBE_RXTXIDX_IPS_EN;
	reg |= idx << IXGBE_RXTXIDX_IDX_SHIFT | IXGBE_RXTXIDX_WRITE;
	IXGBE_WRITE_REG(hw, IXGBE_IPSTXIDX, reg);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_ipsec_set_rx_item - set an Rx table item
 * @hw: hw specific details
 * @idx: register index to write
 * @tbl: table selector
 *
 * Trigger the device to store into a particular Rx table the
 * data that has already been loaded into the input register
 **/
static void ixgbe_ipsec_set_rx_item(struct ixgbe_hw *hw, u16 idx,
				    enum ixgbe_ipsec_tbl_sel tbl)
{
	u32 reg;

	reg = IXGBE_READ_REG(hw, IXGBE_IPSRXIDX);
	reg &= IXGBE_RXTXIDX_IPS_EN;
	reg |= tbl << IXGBE_RXIDX_TBL_SHIFT |
	       idx << IXGBE_RXTXIDX_IDX_SHIFT |
	       IXGBE_RXTXIDX_WRITE;
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXIDX, reg);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_ipsec_set_rx_sa - set up the register bits to save SA info
 * @hw: hw specific details
 * @idx: register index to write
 * @spi: security parameter index
 * @key: key byte array
 * @salt: salt bytes
 * @mode: rx decrypt control bits
 * @ip_idx: index into IP table for related IP address
 **/
static void ixgbe_ipsec_set_rx_sa(struct ixgbe_hw *hw, u16 idx, __be32 spi,
				  u32 key[], u32 salt, u32 mode, u32 ip_idx)
{
	int i;

	/* store the SPI (in bigendian) and IPidx */
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXSPI, cpu_to_le32(spi));
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXIPIDX, ip_idx);
	IXGBE_WRITE_FLUSH(hw);

	ixgbe_ipsec_set_rx_item(hw, idx, ips_rx_spi_tbl);

	/* store the key, salt, and mode */
	for (i = 0; i < 4; i++)
		IXGBE_WRITE_REG(hw, IXGBE_IPSRXKEY(i), cpu_to_be32(key[3 - i]));
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXSALT, cpu_to_be32(salt));
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXMOD, mode);
	IXGBE_WRITE_FLUSH(hw);

	ixgbe_ipsec_set_rx_item(hw, idx, ips_rx_key_tbl);
}

/**
 * ixgbe_ipsec_set_rx_ip - set up the register bits to save SA IP addr info
 * @hw: hw specific details
 * @idx: register index to write
 * @addr: IP address byte array
 **/
static void ixgbe_ipsec_set_rx_ip(struct ixgbe_hw *hw, u16 idx, __be32 addr[])
{
	int i;

	/* store the ip address */
	for (i = 0; i < 4; i++)
		IXGBE_WRITE_REG(hw, IXGBE_IPSRXIPADDR(i), cpu_to_le32(addr[i]));
	IXGBE_WRITE_FLUSH(hw);

	ixgbe_ipsec_set_rx_item(hw, idx, ips_rx_ip_tbl);
}

/**
 * ixgbe_ipsec_clear_hw_tables - because some tables don't get cleared on reset
 * @adapter: board private structure
 **/
static void ixgbe_ipsec_clear_hw_tables(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 buf[4] = {0, 0, 0, 0};
	u16 idx;

	/* disable Rx and Tx SA lookup */
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXIDX, 0);
	IXGBE_WRITE_REG(hw, IXGBE_IPSTXIDX, 0);

	/* scrub the tables - split the loops for the max of the IP table */
	for (idx = 0; idx < IXGBE_IPSEC_MAX_RX_IP_COUNT; idx++) {
		ixgbe_ipsec_set_tx_sa(hw, idx, buf, 0);
		ixgbe_ipsec_set_rx_sa(hw, idx, 0, buf, 0, 0, 0);
		ixgbe_ipsec_set_rx_ip(hw, idx, (__be32 *)buf);
	}
	for (; idx < IXGBE_IPSEC_MAX_SA_COUNT; idx++) {
		ixgbe_ipsec_set_tx_sa(hw, idx, buf, 0);
		ixgbe_ipsec_set_rx_sa(hw, idx, 0, buf, 0, 0, 0);
	}

	ipsec->num_rx_sa = 0;
	ipsec->num_tx_sa = 0;
}

/**
 * ixgbe_ipsec_stop_data
 * @adapter: board private structure
 **/
static void ixgbe_ipsec_stop_data(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	bool link = adapter->link_up;
	u32 t_rdy, r_rdy;
	u32 limit;
	u32 reg;

	/* halt data paths */
	reg = IXGBE_READ_REG(hw, IXGBE_SECTXCTRL);
	reg |= IXGBE_SECTXCTRL_TX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, reg);

	reg = IXGBE_READ_REG(hw, IXGBE_SECRXCTRL);
	reg |= IXGBE_SECRXCTRL_RX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, reg);

	IXGBE_WRITE_FLUSH(hw);

	/* If the tx fifo doesn't have link, but still has data,
	 * we can't clear the tx sec block.  Set the MAC loopback
	 * before block clear
	 */
	if (!link) {
		reg = IXGBE_READ_REG(hw, IXGBE_MACC);
		reg |= IXGBE_MACC_FLU;
		IXGBE_WRITE_REG(hw, IXGBE_MACC, reg);

		reg = IXGBE_READ_REG(hw, IXGBE_HLREG0);
		reg |= IXGBE_HLREG0_LPBK;
		IXGBE_WRITE_REG(hw, IXGBE_HLREG0, reg);

		IXGBE_WRITE_FLUSH(hw);
		mdelay(3);
	}

	/* wait for the paths to empty */
	limit = 20;
	do {
		mdelay(10);
		t_rdy = IXGBE_READ_REG(hw, IXGBE_SECTXSTAT) &
			IXGBE_SECTXSTAT_SECTX_RDY;
		r_rdy = IXGBE_READ_REG(hw, IXGBE_SECRXSTAT) &
			IXGBE_SECRXSTAT_SECRX_RDY;
	} while (!t_rdy && !r_rdy && limit--);

	/* undo loopback if we played with it earlier */
	if (!link) {
		reg = IXGBE_READ_REG(hw, IXGBE_MACC);
		reg &= ~IXGBE_MACC_FLU;
		IXGBE_WRITE_REG(hw, IXGBE_MACC, reg);

		reg = IXGBE_READ_REG(hw, IXGBE_HLREG0);
		reg &= ~IXGBE_HLREG0_LPBK;
		IXGBE_WRITE_REG(hw, IXGBE_HLREG0, reg);

		IXGBE_WRITE_FLUSH(hw);
	}
}

/**
 * ixgbe_ipsec_stop_engine
 * @adapter: board private structure
 **/
static void ixgbe_ipsec_stop_engine(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 reg;

	ixgbe_ipsec_stop_data(adapter);

	/* disable Rx and Tx SA lookup */
	IXGBE_WRITE_REG(hw, IXGBE_IPSTXIDX, 0);
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXIDX, 0);

	/* disable the Rx and Tx engines and full packet store-n-forward */
	reg = IXGBE_READ_REG(hw, IXGBE_SECTXCTRL);
	reg |= IXGBE_SECTXCTRL_SECTX_DIS;
	reg &= ~IXGBE_SECTXCTRL_STORE_FORWARD;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, reg);

	reg = IXGBE_READ_REG(hw, IXGBE_SECRXCTRL);
	reg |= IXGBE_SECRXCTRL_SECRX_DIS;
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, reg);

	/* restore the "tx security buffer almost full threshold" to 0x250 */
	IXGBE_WRITE_REG(hw, IXGBE_SECTXBUFFAF, 0x250);

	/* Set minimum IFG between packets back to the default 0x1 */
	reg = IXGBE_READ_REG(hw, IXGBE_SECTXMINIFG);
	reg = (reg & 0xfffffff0) | 0x1;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXMINIFG, reg);

	/* final set for normal (no ipsec offload) processing */
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, IXGBE_SECTXCTRL_SECTX_DIS);
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, IXGBE_SECRXCTRL_SECRX_DIS);

	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_ipsec_start_engine
 * @adapter: board private structure
 *
 * NOTE: this increases power consumption whether being used or not
 **/
static void ixgbe_ipsec_start_engine(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 reg;

	ixgbe_ipsec_stop_data(adapter);

	/* Set minimum IFG between packets to 3 */
	reg = IXGBE_READ_REG(hw, IXGBE_SECTXMINIFG);
	reg = (reg & 0xfffffff0) | 0x3;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXMINIFG, reg);

	/* Set "tx security buffer almost full threshold" to 0x15 so that the
	 * almost full indication is generated only after buffer contains at
	 * least an entire jumbo packet.
	 */
	reg = IXGBE_READ_REG(hw, IXGBE_SECTXBUFFAF);
	reg = (reg & 0xfffffc00) | 0x15;
	IXGBE_WRITE_REG(hw, IXGBE_SECTXBUFFAF, reg);

	/* restart the data paths by clearing the DISABLE bits */
	IXGBE_WRITE_REG(hw, IXGBE_SECRXCTRL, 0);
	IXGBE_WRITE_REG(hw, IXGBE_SECTXCTRL, IXGBE_SECTXCTRL_STORE_FORWARD);

	/* enable Rx and Tx SA lookup */
	IXGBE_WRITE_REG(hw, IXGBE_IPSTXIDX, IXGBE_RXTXIDX_IPS_EN);
	IXGBE_WRITE_REG(hw, IXGBE_IPSRXIDX, IXGBE_RXTXIDX_IPS_EN);

	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_ipsec_restore - restore the ipsec HW settings after a reset
 * @adapter: board private structure
 **/
void ixgbe_ipsec_restore(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct ixgbe_hw *hw = &adapter->hw;
	int i;

	if (!(adapter->flags2 & IXGBE_FLAG2_IPSEC_ENABLED))
		return;

	/* clean up and restart the engine */
	ixgbe_ipsec_stop_engine(adapter);
	ixgbe_ipsec_clear_hw_tables(adapter);
	ixgbe_ipsec_start_engine(adapter);

	/* reload the IP addrs */
	for (i = 0; i < IXGBE_IPSEC_MAX_RX_IP_COUNT; i++) {
		struct rx_ip_sa *ipsa = &ipsec->ip_tbl[i];

		if (ipsa->used)
			ixgbe_ipsec_set_rx_ip(hw, i, ipsa->ipaddr);
	}

	/* reload the Rx and Tx keys */
	for (i = 0; i < IXGBE_IPSEC_MAX_SA_COUNT; i++) {
		struct rx_sa *rsa = &ipsec->rx_tbl[i];
		struct tx_sa *tsa = &ipsec->tx_tbl[i];

		if (rsa->used)
			ixgbe_ipsec_set_rx_sa(hw, i, rsa->xs->id.spi,
					      rsa->key, rsa->salt,
					      rsa->mode, rsa->iptbl_ind);

		if (tsa->used)
			ixgbe_ipsec_set_tx_sa(hw, i, tsa->key, tsa->salt);
	}
}

/**
 * ixgbe_ipsec_find_empty_idx - find the first unused security parameter index
 * @ipsec: pointer to ipsec struct
 * @rxtable: true if we need to look in the Rx table
 *
 * Returns the first unused index in either the Rx or Tx SA table
 **/
static int ixgbe_ipsec_find_empty_idx(struct ixgbe_ipsec *ipsec, bool rxtable)
{
	u32 i;

	if (rxtable) {
		if (ipsec->num_rx_sa == IXGBE_IPSEC_MAX_SA_COUNT)
			return -ENOSPC;

		/* search rx sa table */
		for (i = 0; i < IXGBE_IPSEC_MAX_SA_COUNT; i++) {
			if (!ipsec->rx_tbl[i].used)
				return i;
		}
	} else {
		if (ipsec->num_tx_sa == IXGBE_IPSEC_MAX_SA_COUNT)
			return -ENOSPC;

		/* search tx sa table */
		for (i = 0; i < IXGBE_IPSEC_MAX_SA_COUNT; i++) {
			if (!ipsec->tx_tbl[i].used)
				return i;
		}
	}

	return -ENOSPC;
}

/**
 * ixgbe_ipsec_find_rx_state - find the state that matches
 * @ipsec: pointer to ipsec struct
 * @daddr: inbound address to match
 * @proto: protocol to match
 * @spi: SPI to match
 * @ip4: true if using an ipv4 address
 *
 * Returns a pointer to the matching SA state information
 **/
static struct xfrm_state *ixgbe_ipsec_find_rx_state(struct ixgbe_ipsec *ipsec,
						    __be32 *daddr, u8 proto,
						    __be32 spi, bool ip4)
{
	struct rx_sa *rsa;
	struct xfrm_state *ret = NULL;

	rcu_read_lock();
	hash_for_each_possible_rcu(ipsec->rx_sa_list, rsa, hlist, spi)
		if (spi == rsa->xs->id.spi &&
		    ((ip4 && *daddr == rsa->xs->id.daddr.a4) ||
		      (!ip4 && !memcmp(daddr, &rsa->xs->id.daddr.a6,
				       sizeof(rsa->xs->id.daddr.a6)))) &&
		    proto == rsa->xs->id.proto) {
			ret = rsa->xs;
			xfrm_state_hold(ret);
			break;
		}
	rcu_read_unlock();
	return ret;
}

/**
 * ixgbe_ipsec_parse_proto_keys - find the key and salt based on the protocol
 * @xs: pointer to xfrm_state struct
 * @mykey: pointer to key array to populate
 * @mysalt: pointer to salt value to populate
 *
 * This copies the protocol keys and salt to our own data tables.  The
 * 82599 family only supports the one algorithm.
 **/
static int ixgbe_ipsec_parse_proto_keys(struct xfrm_state *xs,
					u32 *mykey, u32 *mysalt)
{
	struct net_device *dev = xs->xso.dev;
	unsigned char *key_data;
	char *alg_name = NULL;
	const char aes_gcm_name[] = "rfc4106(gcm(aes))";
	int key_len;

	if (xs->aead) {
		key_data = &xs->aead->alg_key[0];
		key_len = xs->aead->alg_key_len;
		alg_name = xs->aead->alg_name;
	} else {
		netdev_err(dev, "Unsupported IPsec algorithm\n");
		return -EINVAL;
	}

	if (strcmp(alg_name, aes_gcm_name)) {
		netdev_err(dev, "Unsupported IPsec algorithm - please use %s\n",
			   aes_gcm_name);
		return -EINVAL;
	}

	/* The key bytes come down in a bigendian array of bytes, so
	 * we don't need to do any byteswapping.
	 * 160 accounts for 16 byte key and 4 byte salt
	 */
	if (key_len == 160) {
		*mysalt = ((u32 *)key_data)[4];
	} else if (key_len != 128) {
		netdev_err(dev, "IPsec hw offload only supports keys up to 128 bits with a 32 bit salt\n");
		return -EINVAL;
	} else {
		netdev_info(dev, "IPsec hw offload parameters missing 32 bit salt value\n");
		*mysalt = 0;
	}
	memcpy(mykey, key_data, 16);

	return 0;
}

/**
 * ixgbe_ipsec_add_sa - program device with a security association
 * @xs: pointer to transformer state struct
 **/
static int ixgbe_ipsec_add_sa(struct xfrm_state *xs)
{
	struct net_device *dev = xs->xso.dev;
	struct ixgbe_adapter *adapter = netdev_priv(dev);
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct ixgbe_hw *hw = &adapter->hw;
	int checked, match, first;
	u16 sa_idx;
	int ret;
	int i;

	if (xs->id.proto != IPPROTO_ESP && xs->id.proto != IPPROTO_AH) {
		netdev_err(dev, "Unsupported protocol 0x%04x for ipsec offload\n",
			   xs->id.proto);
		return -EINVAL;
	}

	if (xs->xso.flags & XFRM_OFFLOAD_INBOUND) {
		struct rx_sa rsa;

		if (xs->calg) {
			netdev_err(dev, "Compression offload not supported\n");
			return -EINVAL;
		}

		/* find the first unused index */
		ret = ixgbe_ipsec_find_empty_idx(ipsec, true);
		if (ret < 0) {
			netdev_err(dev, "No space for SA in Rx table!\n");
			return ret;
		}
		sa_idx = (u16)ret;

		memset(&rsa, 0, sizeof(rsa));
		rsa.used = true;
		rsa.xs = xs;

		if (rsa.xs->id.proto & IPPROTO_ESP)
			rsa.decrypt = xs->ealg || xs->aead;

		/* get the key and salt */
		ret = ixgbe_ipsec_parse_proto_keys(xs, rsa.key, &rsa.salt);
		if (ret) {
			netdev_err(dev, "Failed to get key data for Rx SA table\n");
			return ret;
		}

		/* get ip for rx sa table */
		if (xs->props.family == AF_INET6)
			memcpy(rsa.ipaddr, &xs->id.daddr.a6, 16);
		else
			memcpy(&rsa.ipaddr[3], &xs->id.daddr.a4, 4);

		/* The HW does not have a 1:1 mapping from keys to IP addrs, so
		 * check for a matching IP addr entry in the table.  If the addr
		 * already exists, use it; else find an unused slot and add the
		 * addr.  If one does not exist and there are no unused table
		 * entries, fail the request.
		 */

		/* Find an existing match or first not used, and stop looking
		 * after we've checked all we know we have.
		 */
		checked = 0;
		match = -1;
		first = -1;
		for (i = 0;
		     i < IXGBE_IPSEC_MAX_RX_IP_COUNT &&
		     (checked < ipsec->num_rx_sa || first < 0);
		     i++) {
			if (ipsec->ip_tbl[i].used) {
				if (!memcmp(ipsec->ip_tbl[i].ipaddr,
					    rsa.ipaddr, sizeof(rsa.ipaddr))) {
					match = i;
					break;
				}
				checked++;
			} else if (first < 0) {
				first = i;  /* track the first empty seen */
			}
		}

		if (ipsec->num_rx_sa == 0)
			first = 0;

		if (match >= 0) {
			/* addrs are the same, we should use this one */
			rsa.iptbl_ind = match;
			ipsec->ip_tbl[match].ref_cnt++;

		} else if (first >= 0) {
			/* no matches, but here's an empty slot */
			rsa.iptbl_ind = first;

			memcpy(ipsec->ip_tbl[first].ipaddr,
			       rsa.ipaddr, sizeof(rsa.ipaddr));
			ipsec->ip_tbl[first].ref_cnt = 1;
			ipsec->ip_tbl[first].used = true;

			ixgbe_ipsec_set_rx_ip(hw, rsa.iptbl_ind, rsa.ipaddr);

		} else {
			/* no match and no empty slot */
			netdev_err(dev, "No space for SA in Rx IP SA table\n");
			memset(&rsa, 0, sizeof(rsa));
			return -ENOSPC;
		}

		rsa.mode = IXGBE_RXMOD_VALID;
		if (rsa.xs->id.proto & IPPROTO_ESP)
			rsa.mode |= IXGBE_RXMOD_PROTO_ESP;
		if (rsa.decrypt)
			rsa.mode |= IXGBE_RXMOD_DECRYPT;
		if (rsa.xs->props.family == AF_INET6)
			rsa.mode |= IXGBE_RXMOD_IPV6;

		/* the preparations worked, so save the info */
		memcpy(&ipsec->rx_tbl[sa_idx], &rsa, sizeof(rsa));

		ixgbe_ipsec_set_rx_sa(hw, sa_idx, rsa.xs->id.spi, rsa.key,
				      rsa.salt, rsa.mode, rsa.iptbl_ind);
		xs->xso.offload_handle = sa_idx + IXGBE_IPSEC_BASE_RX_INDEX;

		ipsec->num_rx_sa++;

		/* hash the new entry for faster search in Rx path */
		hash_add_rcu(ipsec->rx_sa_list, &ipsec->rx_tbl[sa_idx].hlist,
			     rsa.xs->id.spi);
	} else {
		struct tx_sa tsa;

		/* find the first unused index */
		ret = ixgbe_ipsec_find_empty_idx(ipsec, false);
		if (ret < 0) {
			netdev_err(dev, "No space for SA in Tx table\n");
			return ret;
		}
		sa_idx = (u16)ret;

		memset(&tsa, 0, sizeof(tsa));
		tsa.used = true;
		tsa.xs = xs;

		if (xs->id.proto & IPPROTO_ESP)
			tsa.encrypt = xs->ealg || xs->aead;

		ret = ixgbe_ipsec_parse_proto_keys(xs, tsa.key, &tsa.salt);
		if (ret) {
			netdev_err(dev, "Failed to get key data for Tx SA table\n");
			memset(&tsa, 0, sizeof(tsa));
			return ret;
		}

		/* the preparations worked, so save the info */
		memcpy(&ipsec->tx_tbl[sa_idx], &tsa, sizeof(tsa));

		ixgbe_ipsec_set_tx_sa(hw, sa_idx, tsa.key, tsa.salt);

		xs->xso.offload_handle = sa_idx + IXGBE_IPSEC_BASE_TX_INDEX;

		ipsec->num_tx_sa++;
	}

	/* enable the engine if not already warmed up */
	if (!(adapter->flags2 & IXGBE_FLAG2_IPSEC_ENABLED)) {
		ixgbe_ipsec_start_engine(adapter);
		adapter->flags2 |= IXGBE_FLAG2_IPSEC_ENABLED;
	}

	return 0;
}

/**
 * ixgbe_ipsec_del_sa - clear out this specific SA
 * @xs: pointer to transformer state struct
 **/
static void ixgbe_ipsec_del_sa(struct xfrm_state *xs)
{
	struct net_device *dev = xs->xso.dev;
	struct ixgbe_adapter *adapter = netdev_priv(dev);
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 zerobuf[4] = {0, 0, 0, 0};
	u16 sa_idx;

	if (xs->xso.flags & XFRM_OFFLOAD_INBOUND) {
		struct rx_sa *rsa;
		u8 ipi;

		sa_idx = xs->xso.offload_handle - IXGBE_IPSEC_BASE_RX_INDEX;
		rsa = &ipsec->rx_tbl[sa_idx];

		if (!rsa->used) {
			netdev_err(dev, "Invalid Rx SA selected sa_idx=%d offload_handle=%lu\n",
				   sa_idx, xs->xso.offload_handle);
			return;
		}

		ixgbe_ipsec_set_rx_sa(hw, sa_idx, 0, zerobuf, 0, 0, 0);
		hash_del_rcu(&rsa->hlist);

		/* if the IP table entry is referenced by only this SA,
		 * i.e. ref_cnt is only 1, clear the IP table entry as well
		 */
		ipi = rsa->iptbl_ind;
		if (ipsec->ip_tbl[ipi].ref_cnt > 0) {
			ipsec->ip_tbl[ipi].ref_cnt--;

			if (!ipsec->ip_tbl[ipi].ref_cnt) {
				memset(&ipsec->ip_tbl[ipi], 0,
				       sizeof(struct rx_ip_sa));
				ixgbe_ipsec_set_rx_ip(hw, ipi, zerobuf);
			}
		}

		memset(rsa, 0, sizeof(struct rx_sa));
		ipsec->num_rx_sa--;
	} else {
		sa_idx = xs->xso.offload_handle - IXGBE_IPSEC_BASE_TX_INDEX;

		if (!ipsec->tx_tbl[sa_idx].used) {
			netdev_err(dev, "Invalid Tx SA selected sa_idx=%d offload_handle=%lu\n",
				   sa_idx, xs->xso.offload_handle);
			return;
		}

		ixgbe_ipsec_set_tx_sa(hw, sa_idx, zerobuf, 0);
		memset(&ipsec->tx_tbl[sa_idx], 0, sizeof(struct tx_sa));
		ipsec->num_tx_sa--;
	}

	/* if there are no SAs left, stop the engine to save energy */
	if (ipsec->num_rx_sa == 0 && ipsec->num_tx_sa == 0) {
		adapter->flags2 &= ~IXGBE_FLAG2_IPSEC_ENABLED;
		ixgbe_ipsec_stop_engine(adapter);
	}
}

/**
 * ixgbe_ipsec_offload_ok - can this packet use the xfrm hw offload
 * @skb: current data packet
 * @xs: pointer to transformer state struct
 **/
static bool ixgbe_ipsec_offload_ok(struct sk_buff *skb, struct xfrm_state *xs)
{
	if (xs->props.family == AF_INET) {
		/* Offload with IPv4 options is not supported yet */
		if (ip_hdr(skb)->ihl != 5)
			return false;
	} else {
		/* Offload with IPv6 extension headers is not support yet */
		if (ipv6_ext_hdr(ipv6_hdr(skb)->nexthdr))
			return false;
	}

	return true;
}

/**
 * ixgbe_ipsec_free - called by xfrm garbage collections
 * @xs: pointer to transformer state struct
 *
 * We don't have any garbage to collect, so we shouldn't bother
 * implementing this function, but the XFRM code doesn't check for
 * existence before calling the API callback.
 **/
static void ixgbe_ipsec_free(struct xfrm_state *xs)
{
}

static const struct xfrmdev_ops ixgbe_xfrmdev_ops = {
	.xdo_dev_state_add = ixgbe_ipsec_add_sa,
	.xdo_dev_state_delete = ixgbe_ipsec_del_sa,
	.xdo_dev_offload_ok = ixgbe_ipsec_offload_ok,
	.xdo_dev_state_free = ixgbe_ipsec_free,
};

/**
 * ixgbe_ipsec_tx - setup Tx flags for ipsec offload
 * @tx_ring: outgoing context
 * @first: current data packet
 * @itd: ipsec Tx data for later use in building context descriptor
 **/
int ixgbe_ipsec_tx(struct ixgbe_ring *tx_ring,
		   struct ixgbe_tx_buffer *first,
		   struct ixgbe_ipsec_tx_data *itd)
{
	struct ixgbe_adapter *adapter = netdev_priv(tx_ring->netdev);
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct xfrm_state *xs;
	struct tx_sa *tsa;

	if (unlikely(!first->skb->sp->len)) {
		netdev_err(tx_ring->netdev, "%s: no xfrm state len = %d\n",
			   __func__, first->skb->sp->len);
		return 0;
	}

	xs = xfrm_input_state(first->skb);
	if (unlikely(!xs)) {
		netdev_err(tx_ring->netdev, "%s: no xfrm_input_state() xs = %p\n",
			   __func__, xs);
		return 0;
	}

	itd->sa_idx = xs->xso.offload_handle - IXGBE_IPSEC_BASE_TX_INDEX;
	if (unlikely(itd->sa_idx > IXGBE_IPSEC_MAX_SA_COUNT)) {
		netdev_err(tx_ring->netdev, "%s: bad sa_idx=%d handle=%lu\n",
			   __func__, itd->sa_idx, xs->xso.offload_handle);
		return 0;
	}

	tsa = &ipsec->tx_tbl[itd->sa_idx];
	if (unlikely(!tsa->used)) {
		netdev_err(tx_ring->netdev, "%s: unused sa_idx=%d\n",
			   __func__, itd->sa_idx);
		return 0;
	}

	first->tx_flags |= IXGBE_TX_FLAGS_IPSEC | IXGBE_TX_FLAGS_CC;

	itd->flags = 0;
	if (xs->id.proto == IPPROTO_ESP) {
		itd->flags |= IXGBE_ADVTXD_TUCMD_IPSEC_TYPE_ESP |
			      IXGBE_ADVTXD_TUCMD_L4T_TCP;
		if (first->protocol == htons(ETH_P_IP))
			itd->flags |= IXGBE_ADVTXD_TUCMD_IPV4;
		itd->trailer_len = xs->props.trailer_len;
	}
	if (tsa->encrypt)
		itd->flags |= IXGBE_ADVTXD_TUCMD_IPSEC_ENCRYPT_EN;

	return 1;
}

/**
 * ixgbe_ipsec_rx - decode ipsec bits from Rx descriptor
 * @rx_ring: receiving ring
 * @rx_desc: receive data descriptor
 * @skb: current data packet
 *
 * Determine if there was an ipsec encapsulation noticed, and if so set up
 * the resulting status for later in the receive stack.
 **/
void ixgbe_ipsec_rx(struct ixgbe_ring *rx_ring,
		    union ixgbe_adv_rx_desc *rx_desc,
		    struct sk_buff *skb)
{
	struct ixgbe_adapter *adapter = netdev_priv(rx_ring->netdev);
	__le16 pkt_info = rx_desc->wb.lower.lo_dword.hs_rss.pkt_info;
	__le16 ipsec_pkt_types = cpu_to_le16(IXGBE_RXDADV_PKTTYPE_IPSEC_AH |
					     IXGBE_RXDADV_PKTTYPE_IPSEC_ESP);
	struct ixgbe_ipsec *ipsec = adapter->ipsec;
	struct xfrm_offload *xo = NULL;
	struct xfrm_state *xs = NULL;
	struct ipv6hdr *ip6 = NULL;
	struct iphdr *ip4 = NULL;
	void *daddr;
	__be32 spi;
	u8 *c_hdr;
	u8 proto;

	/* Find the ip and crypto headers in the data.
	 * We can assume no vlan header in the way, b/c the
	 * hw won't recognize the IPsec packet and anyway the
	 * currently vlan device doesn't support xfrm offload.
	 */
	if (pkt_info & cpu_to_le16(IXGBE_RXDADV_PKTTYPE_IPV4)) {
		ip4 = (struct iphdr *)(skb->data + ETH_HLEN);
		daddr = &ip4->daddr;
		c_hdr = (u8 *)ip4 + ip4->ihl * 4;
	} else if (pkt_info & cpu_to_le16(IXGBE_RXDADV_PKTTYPE_IPV6)) {
		ip6 = (struct ipv6hdr *)(skb->data + ETH_HLEN);
		daddr = &ip6->daddr;
		c_hdr = (u8 *)ip6 + sizeof(struct ipv6hdr);
	} else {
		return;
	}

	switch (pkt_info & ipsec_pkt_types) {
	case cpu_to_le16(IXGBE_RXDADV_PKTTYPE_IPSEC_AH):
		spi = ((struct ip_auth_hdr *)c_hdr)->spi;
		proto = IPPROTO_AH;
		break;
	case cpu_to_le16(IXGBE_RXDADV_PKTTYPE_IPSEC_ESP):
		spi = ((struct ip_esp_hdr *)c_hdr)->spi;
		proto = IPPROTO_ESP;
		break;
	default:
		return;
	}

	xs = ixgbe_ipsec_find_rx_state(ipsec, daddr, proto, spi, !!ip4);
	if (unlikely(!xs))
		return;

	skb->sp = secpath_dup(skb->sp);
	if (unlikely(!skb->sp))
		return;

	skb->sp->xvec[skb->sp->len++] = xs;
	skb->sp->olen++;
	xo = xfrm_offload(skb);
	xo->flags = CRYPTO_DONE;
	xo->status = CRYPTO_SUCCESS;

	adapter->rx_ipsec++;
}

/**
 * ixgbe_init_ipsec_offload - initialize security registers for IPSec operation
 * @adapter: board private structure
 **/
void ixgbe_init_ipsec_offload(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ipsec *ipsec;
	size_t size;

	if (adapter->hw.mac.type == ixgbe_mac_82598EB)
		return;

	ipsec = kzalloc(sizeof(*ipsec), GFP_KERNEL);
	if (!ipsec)
		goto err1;
	hash_init(ipsec->rx_sa_list);

	size = sizeof(struct rx_sa) * IXGBE_IPSEC_MAX_SA_COUNT;
	ipsec->rx_tbl = kzalloc(size, GFP_KERNEL);
	if (!ipsec->rx_tbl)
		goto err2;

	size = sizeof(struct tx_sa) * IXGBE_IPSEC_MAX_SA_COUNT;
	ipsec->tx_tbl = kzalloc(size, GFP_KERNEL);
	if (!ipsec->tx_tbl)
		goto err2;

	size = sizeof(struct rx_ip_sa) * IXGBE_IPSEC_MAX_RX_IP_COUNT;
	ipsec->ip_tbl = kzalloc(size, GFP_KERNEL);
	if (!ipsec->ip_tbl)
		goto err2;

	ipsec->num_rx_sa = 0;
	ipsec->num_tx_sa = 0;

	adapter->ipsec = ipsec;
	ixgbe_ipsec_stop_engine(adapter);
	ixgbe_ipsec_clear_hw_tables(adapter);

	adapter->netdev->xfrmdev_ops = &ixgbe_xfrmdev_ops;
	adapter->netdev->features |= NETIF_F_HW_ESP;
	adapter->netdev->hw_enc_features |= NETIF_F_HW_ESP;

	return;

err2:
	kfree(ipsec->ip_tbl);
	kfree(ipsec->rx_tbl);
	kfree(ipsec->tx_tbl);
err1:
	kfree(adapter->ipsec);
	netdev_err(adapter->netdev, "Unable to allocate memory for SA tables");
}

/**
 * ixgbe_stop_ipsec_offload - tear down the ipsec offload
 * @adapter: board private structure
 **/
void ixgbe_stop_ipsec_offload(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ipsec *ipsec = adapter->ipsec;

	adapter->ipsec = NULL;
	if (ipsec) {
		kfree(ipsec->ip_tbl);
		kfree(ipsec->rx_tbl);
		kfree(ipsec->tx_tbl);
		kfree(ipsec);
	}
}
