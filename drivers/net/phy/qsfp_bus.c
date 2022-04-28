// SPDX-License-Identifier: GPL-2.0-only
#include <linux/export.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/phylink.h>
#include <linux/property.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/qsfp.h>



struct qsfp_quirk {
	const char *vendor;
	const char *part;
	void (*modes)(const struct qsfp_eeprom_id *id, unsigned long *modes);
};

/**
 * struct qsfp_bus - internal representation of a qsfp bus
 */
struct qsfp_bus {
	/* private: */
	struct kref kref;
	struct list_head node;
	struct fwnode_handle *fwnode;

	const struct qsfp_socket_ops *socket_ops;
	struct device *qsfp_dev;
	struct qsfp *qsfp;
	const struct qsfp_quirk *qsfp_quirk;

	const struct qsfp_upstream_ops *upstream_ops;
	void *upstream;
	struct phy_device *phydev;

	bool registered;
	bool started;
};

static void qsfp_quirk_2500basex(const struct qsfp_eeprom_id *id,
				 unsigned long *modes)
{
	phylink_set(modes, 2500baseX_Full);
}

static void qsfp_quirk_ubnt_uf_instant(const struct qsfp_eeprom_id *id,
				       unsigned long *modes)
{
	/* Ubiquiti U-Fiber Instant module claims that support all transceiver
	 * types including 10G Ethernet which is not truth. So clear all claimed
	 * modes and set only one mode which module supports: 1000baseX_Full.
	 */
	phylink_zero(modes);
	phylink_set(modes, 1000baseX_Full);
}

static const struct qsfp_quirk qsfp_quirks[] = {
	{
		// Alcatel Lucent G-010S-P can operate at 2500base-X, but
		// incorrectly report 2500MBd NRZ in their EEPROM
		.vendor = "ALCATELLUCENT",
		.part = "G010SP",
		.modes = qsfp_quirk_2500basex,
	},
	{
		// Alcatel Lucent G-010S-A can operate at 2500base-X, but
		// report 3.2GBd NRZ in their EEPROM
		.vendor = "ALCATELLUCENT",
		.part = "3FE46541AA",
		.modes = qsfp_quirk_2500basex,
	},
	{
		// Huawei MA5671A can operate at 2500base-X, but report 1.2GBd
		// NRZ in their EEPROM
		.vendor = "HUAWEI",
		.part = "MA5671A",
		.modes = qsfp_quirk_2500basex,
	},
	{
		.vendor = "UBNT",
		.part = "UF-INSTANT",
		.modes = qsfp_quirk_ubnt_uf_instant,
	},
};

static size_t qsfp_strlen(const char *str, size_t maxlen)
{
	size_t size, i;

	/* Trailing characters should be filled with space chars */
	for (i = 0, size = 0; i < maxlen; i++)
		if (str[i] != ' ')
			size = i + 1;

	return size;
}

static bool qsfp_match(const char *qs, const char *str, size_t len)
{
	if (!qs)
		return true;
	if (strlen(qs) != len)
		return false;
	return !strncmp(qs, str, len);
}

static const struct qsfp_quirk *
qsfp_lookup_quirk(const struct qsfp_eeprom_id *id)
{
	const struct qsfp_quirk *q;
	unsigned int i;
	size_t vs, ps;

	vs = qsfp_strlen(id->base.etile_qsfp_vendor_name,
			 ARRAY_SIZE(id->base.etile_qsfp_vendor_name));
	ps = qsfp_strlen(id->base.etile_qsfp_vendor_pn,
			 ARRAY_SIZE(id->base.etile_qsfp_vendor_pn));

	for (i = 0, q = qsfp_quirks; i < ARRAY_SIZE(qsfp_quirks); i++, q++)
		if (qsfp_match(q->vendor, id->base.etile_qsfp_vendor_name,
			       vs) &&
		    qsfp_match(q->part, id->base.etile_qsfp_vendor_pn, ps))
			return q;

	return NULL;
}

/**
 * qsfp_parse_port() - Parse the EEPROM base ID, setting the port type
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @id: a pointer to the module's &struct qsfp_eeprom_id
 * @support: optional pointer to an array of unsigned long for the
 *   ethtool support mask
 *
 * Parse the EEPROM identification given in @id, and return one of
 * %PORT_TP, %PORT_FIBRE or %PORT_OTHER. If @support is non-%NULL,
 * also set the ethtool %ETHTOOL_LINK_MODE_xxx_BIT corresponding with
 * the connector type.
 *
 * If the port type is not known, returns %PORT_OTHER.
 */

int qsfp_parse_port(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id,
		    unsigned long *support)
{
	int port;

	/* port is the physical connector, set this from the connector field. */
	switch (id->base.etile_qsfp_connector_type) {
	case SFF8024_QSFP_DD_CONNECTOR_SC:
	case SFF8024_QSFP_DD_CONNECTOR_FIBERJACK:
	case SFF8024_QSFP_DD_CONNECTOR_LC:
	case SFF8024_QSFP_DD_CONNECTOR_MT_RJ:
	case SFF8024_QSFP_DD_CONNECTOR_MU:
	case SFF8024_QSFP_DD_CONNECTOR_OPTICAL_PIGTAIL:
	case SFF8024_QSFP_DD_CONNECTOR_MPO_1X12:
	case SFF8024_QSFP_DD_CONNECTOR_MPO_2X16:
		port = PORT_FIBRE;
		break;

	case SFF8024_QSFP_DD_CONNECTOR_RJ45:
		port = PORT_TP;
		break;

	case SFF8024_QSFP_DD_CONNECTOR_COPPER_PIGTAIL:
		port = PORT_DA;
		break;

	case SFF8024_QSFP_DD_CONNECTOR_UNSPEC:
		{
			port = PORT_TP;
			break;
		}
		fallthrough;
	case SFF8024_QSFP_DD_CONNECTOR_SG: /* guess */
	case SFF8024_QSFP_DD_CONNECTOR_HSSDC_II:
	case SFF8024_QSFP_DD_CONNECTOR_NOSEPARATE:
	case SFF8024_QSFP_DD_CONNECTOR_MXC_2X16:
		port = PORT_OTHER;
		break;
	default:
		dev_warn(bus->qsfp_dev, "qsfp: unknown connector id 0x%02x\n",
			 id->base.etile_qsfp_connector_type);
		port = PORT_OTHER;
		break;
	}

	if (support) {
		switch (port) {
		case PORT_FIBRE:
			phylink_set(support, FIBRE);
			break;

		case PORT_TP:
			phylink_set(support, TP);
			break;
		}
	}

	return port;
}
EXPORT_SYMBOL_GPL(qsfp_parse_port);

/**
 * qsfp_may_have_phy() - indicate whether the module may have a PHY
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @id: a pointer to the module's &struct qsfp_eeprom_id
 *
 * Parse the EEPROM identification given in @id, and return whether
 * this module may have a PHY.
 */

bool qsfp_may_have_phy(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id)
{
	if (id->base.etile_qsfp_identifier != SFF8024_ID_QSFP_DD_INF_8628) {
		switch (id->base.etile_qsfp_spec_compliance_1[0]) {
		case SFF8636_QSFP_DD_ECC_100GBASE_CR4:
		case SFF8636_QSFP_DD_ECC_CAUI4:

			return true;
		}
	}

	return false;
}
EXPORT_SYMBOL_GPL(qsfp_may_have_phy);

/**
 * qsfp_parse_support() - Parse the eeprom id for supported link modes
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @id: a pointer to the module's &struct qsfp_eeprom_id
 * @support: pointer to an array of unsigned long for the ethtool support mask
 *
 * Parse the EEPROM identification information and derive the supported
 * ethtool link modes for the module.
 */

void qsfp_parse_support(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id,
			unsigned long *support)
{
	unsigned int etile_qsfp_br_nom, etile_qsfp_br_max, etile_qsfp_br_min;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(modes) = {
		0,
	};

	/* Decode the bitrate information to MBd */
	etile_qsfp_br_min = 0;
	etile_qsfp_br_nom = 0;
	etile_qsfp_br_max = 0;
	if (id->base.etile_qsfp_br_nom) {
		if (id->base.etile_qsfp_br_nom != 255) {
			etile_qsfp_br_nom = id->base.etile_qsfp_br_nom * 100;
			etile_qsfp_br_min = etile_qsfp_br_nom -
					    id->base.etile_qsfp_br_nom *
						    id->base.etile_qsfp_br_min;
			etile_qsfp_br_max = etile_qsfp_br_nom +
					    id->base.etile_qsfp_br_nom *
						    id->base.etile_qsfp_br_max;
		} else if (id->base.etile_qsfp_br_max) {
			etile_qsfp_br_nom = 250 * id->base.etile_qsfp_br_max;
			etile_qsfp_br_max = etile_qsfp_br_nom +
					    etile_qsfp_br_nom *
						    id->base.etile_qsfp_br_min /
						    100;
			etile_qsfp_br_min = etile_qsfp_br_nom -
					    etile_qsfp_br_nom *
						    id->base.etile_qsfp_br_min /
						    100;
		}

		/* When using passive cables, in case neither BR,min nor BR,max
		 * are specified, set etile_qsfp_br_min to 0 as the nominal value is then
		 * used as the maximum.
		 */
	}

	/* Set ethtool support from the compliance fields. */
	if (id->base.etile_qsfp_spec_compliance_1)
		phylink_set(modes, 10000baseSR_Full);
	if (id->base.etile_qsfp_spec_compliance_1)
		phylink_set(modes, 10000baseLR_Full);

	switch (id->base.etile_qsfp_spec_compliance_1[0]) {
	case SFF8636_QSFP_DD_ECC_100GBASE_CR4:
		break;
	case SFF8636_QSFP_DD_ECC_CAUI4:
		phylink_set(modes, 100000baseSR4_Full);
		phylink_set(modes, 25000baseSR_Full);
		break;

	default:
		dev_warn(bus->qsfp_dev,
			 "Unknown/unsupported extended compliance code: 0x%02x\n",
			 id->base.etile_qsfp_spec_compliance_1[0]);
		break;
	}

	/* If we haven't discovered any modes that this module supports, try
	 * the bitrate to determine supported modes. Some BiDi modules (eg,
	 * 1310nm/1550nm) are not 1000BASE-BX compliant due to the differing
	 * wavelengths, so do not set any transceiver bits.
	 */
	if (bitmap_empty(modes, __ETHTOOL_LINK_MODE_MASK_NBITS)) {
		/* If the bit rate allows 1000baseX */
		if (etile_qsfp_br_nom && etile_qsfp_br_min <= 1300 &&
		    etile_qsfp_br_max >= 1200)
			phylink_set(modes, 1000baseX_Full);
	}

	if (bus->qsfp_quirk)
		bus->qsfp_quirk->modes(id, modes);

	bitmap_or(support, support, modes, __ETHTOOL_LINK_MODE_MASK_NBITS);

	phylink_set(support, Autoneg);
	phylink_set(support, Pause);
	phylink_set(support, Asym_Pause);
}
EXPORT_SYMBOL_GPL(qsfp_parse_support);

/**
 * qsfp_select_interface() - Select appropriate phy_interface_t mode
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @link_modes: ethtool link modes mask
 *
 * Derive the phy_interface_t mode for the qsfp module from the link
 * modes mask.
 */
phy_interface_t qsfp_select_interface(struct qsfp_bus *bus,
				      unsigned long *link_modes)
{
	if (phylink_test(link_modes, 10000baseCR_Full) ||
	    phylink_test(link_modes, 10000baseSR_Full) ||
	    phylink_test(link_modes, 10000baseLR_Full) ||
	    phylink_test(link_modes, 10000baseLRM_Full) ||
	    phylink_test(link_modes, 10000baseER_Full) ||
	    phylink_test(link_modes, 10000baseT_Full))
		return PHY_INTERFACE_MODE_10GBASER;

	if (phylink_test(link_modes, 2500baseX_Full))
		return PHY_INTERFACE_MODE_2500BASEX;

	if (phylink_test(link_modes, 1000baseT_Half) ||
	    phylink_test(link_modes, 1000baseT_Full))
		return PHY_INTERFACE_MODE_SGMII;

	if (phylink_test(link_modes, 1000baseX_Full))
		return PHY_INTERFACE_MODE_1000BASEX;

	dev_warn(bus->qsfp_dev, "Unable to ascertain link mode\n");

	return PHY_INTERFACE_MODE_NA;
}
EXPORT_SYMBOL_GPL(qsfp_select_interface);

static LIST_HEAD(qsfp_buses);
static DEFINE_MUTEX(qsfp_mutex);

static const struct qsfp_upstream_ops *
qsfp_get_upstream_ops(struct qsfp_bus *bus)
{
	return bus->registered ? bus->upstream_ops : NULL;
}

static struct qsfp_bus *qsfp_bus_get(struct fwnode_handle *fwnode)
{
	struct qsfp_bus *qsfp, *new, *found = NULL;

	new = kzalloc(sizeof(*new), GFP_KERNEL);

	mutex_lock(&qsfp_mutex);

	list_for_each_entry(qsfp, &qsfp_buses, node) {
		if (qsfp->fwnode == fwnode) {
			kref_get(&qsfp->kref);
			found = qsfp;
			break;
		}
	}

	if (!found && new) {
		kref_init(&new->kref);
		new->fwnode = fwnode;
		list_add(&new->node, &qsfp_buses);
		found = new;
		new = NULL;
	}

	mutex_unlock(&qsfp_mutex);

	kfree(new);

	return found;
}

static void qsfp_bus_release(struct kref *kref)
{
	struct qsfp_bus *bus = container_of(kref, struct qsfp_bus, kref);

	list_del(&bus->node);
	mutex_unlock(&qsfp_mutex);
	kfree(bus);
}

/**
 * qsfp_bus_put() - put a reference on the &struct qsfp_bus
 * @bus: the &struct qsfp_bus found via qsfp_bus_find_fwnode()
 *
 * Put a reference on the &struct qsfp_bus and free the underlying structure
 * if this was the last reference.
 */
void qsfp_bus_put(struct qsfp_bus *bus)
{
	if (bus)
		kref_put_mutex(&bus->kref, qsfp_bus_release, &qsfp_mutex);
}
EXPORT_SYMBOL_GPL(qsfp_bus_put);

static int qsfp_register_bus(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = bus->upstream_ops;
	int ret;

	pr_info("qsfp register bus\n");

	if (ops) {
		if (ops->link_down)
			ops->link_down(bus->upstream);
		if (ops->connect_phy /*&& bus->phydev*/) {
			ret = ops->connect_phy(bus->upstream, bus->phydev);
			if (ret)
				return ret;
		}
	}
	bus->registered = true;

	bus->socket_ops->attach(bus->qsfp);
	if (bus->started)
		bus->socket_ops->start(bus->qsfp);
	bus->upstream_ops->attach(bus->upstream, bus);
	return 0;
}

static void qsfp_unregister_bus(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = bus->upstream_ops;

	if (bus->registered) {
		bus->upstream_ops->detach(bus->upstream, bus);
		if (bus->started)
			bus->socket_ops->stop(bus->qsfp);
		bus->socket_ops->detach(bus->qsfp);
		if (bus->phydev && ops && ops->disconnect_phy)
			ops->disconnect_phy(bus->upstream);
	}
	bus->registered = false;
}

/**
 * qsfp_get_module_info() - Get the ethtool_modinfo for a qsfp module
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @modinfo: a &struct ethtool_modinfo
 *
 * Fill in the type and eeprom_len parameters in @modinfo for a module on
 * the qsfp bus specified by @bus.
 *
 * Returns 0 on success or a negative errno number.
 */
int qsfp_get_module_info(struct qsfp_bus *bus, struct ethtool_modinfo *modinfo)
{
	return bus->socket_ops->module_info(bus->qsfp, modinfo);
}
EXPORT_SYMBOL_GPL(qsfp_get_module_info);

/**
 * qsfp_get_module_eeprom() - Read the qsfp module EEPROM
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 * @ee: a &struct ethtool_eeprom
 * @data: buffer to contain the EEPROM data (must be at least @ee->len bytes)
 *
 * Read the EEPROM as specified by the supplied @ee. See the documentation
 * for &struct ethtool_eeprom for the region to be read.
 *
 * Returns 0 on success or a negative errno number.
 */
int qsfp_get_module_eeprom(struct qsfp_bus *bus, struct ethtool_eeprom *ee,
			   u8 *data)
{
	return bus->socket_ops->module_eeprom(bus->qsfp, ee, data);
}
EXPORT_SYMBOL_GPL(qsfp_get_module_eeprom);

/**
 * qsfp_upstream_start() - Inform the qsfp that the network device is up
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 *
 * Inform the qsfp socket that the network device is now up, so that the
 * module can be enabled by allowing TX_DISABLE to be deasserted. This
 * should be called from the network device driver's &struct net_device_ops
 * ndo_open() method.
 */
void qsfp_upstream_start(struct qsfp_bus *bus)
{
	if (bus->registered)
		bus->socket_ops->start(bus->qsfp);
	bus->started = true;
}
EXPORT_SYMBOL_GPL(qsfp_upstream_start);

/**
 * qsfp_upstream_stop() - Inform the qsfp that the network device is down
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 *
 * Inform the qsfp socket that the network device is now up, so that the
 * module can be disabled by asserting TX_DISABLE, disabling the laser
 * in optical modules. This should be called from the network device
 * driver's &struct net_device_ops ndo_stop() method.
 */
void qsfp_upstream_stop(struct qsfp_bus *bus)
{
	if (bus->registered)
		bus->socket_ops->stop(bus->qsfp);
	bus->started = false;
}
EXPORT_SYMBOL_GPL(qsfp_upstream_stop);

static void qsfp_upstream_clear(struct qsfp_bus *bus)
{
	bus->upstream_ops = NULL;
	bus->upstream = NULL;
}

/**
 * qsfp_bus_find_fwnode() - parse and locate the qsfp bus from fwnode
 * @fwnode: firmware node for the parent device (MAC or PHY)
 *
 * Parse the parent device's firmware node for a qsfp bus, and locate
 * the qsfp_bus structure, incrementing its reference count.  This must
 * be put via qsfp_bus_put() when done.
 *
 * Returns:
 *	- on success, a pointer to the qsfp_bus structure,
 *	- %NULL if no qsfp is specified,
 *	- on failure, an error pointer value:
 *	- corresponding to the errors detailed for
 *	fwnode_property_get_reference_args().
 *	- %-ENOMEM if we failed to allocate the bus.
 *	- an error from the upstream's connect_phy() method.
 */
struct qsfp_bus *qsfp_bus_find_fwnode(struct fwnode_handle *fwnode)
{
	struct fwnode_reference_args ref;
	struct qsfp_bus *bus;
	int ret;

	ret = fwnode_property_get_reference_args(fwnode, "qsfp", NULL, 0, 0,
						 &ref);
	if (ret == -ENOENT)
		return NULL;
	else if (ret < 0)
		return ERR_PTR(ret);

	bus = qsfp_bus_get(ref.fwnode);
	fwnode_handle_put(ref.fwnode);
	if (!bus)
		return ERR_PTR(-ENOMEM);

	return bus;
}
EXPORT_SYMBOL_GPL(qsfp_bus_find_fwnode);

/**
 * qsfp_bus_add_upstream() - parse and register the neighbouring device
 * @bus: the &struct qsfp_bus found via qsfp_bus_find_fwnode()
 * @upstream: the upstream private data
 * @ops: the upstream's &struct qsfp_upstream_ops
 *
 * Add upstream driver for the qsfp bus, and if the bus is complete, register
 * the qsfp bus using qsfp_register_upstream().  This takes a reference on the
 * bus, so it is safe to put the bus after this call.
 *
 * Returns:
 *	- on success, a pointer to the qsfp_bus structure,
 *	- %NULL if no qsfp is specified,
 *	- on failure, an error pointer value:
 *	- corresponding to the errors detailed for
 *	fwnode_property_get_reference_args().
 *	- %-ENOMEM if we failed to allocate the bus.
 *	- an error from the upstream's connect_phy() method.
 */
int qsfp_bus_add_upstream(struct qsfp_bus *bus, void *upstream,
			  const struct qsfp_upstream_ops *ops)
{
	int ret;

	/* If no bus, return success */
	if (!bus)
		return 0;

	rtnl_lock();
	kref_get(&bus->kref);
	bus->upstream_ops = ops;
	bus->upstream = upstream;

	if (bus->qsfp) {
		ret = qsfp_register_bus(bus);
		if (ret)
			qsfp_upstream_clear(bus);
	} else {
		ret = 0;
	}
	rtnl_unlock();

	if (ret)
		qsfp_bus_put(bus);

	return ret;
}
EXPORT_SYMBOL_GPL(qsfp_bus_add_upstream);

/**
 * qsfp_bus_del_upstream() - Delete a qsfp bus
 * @bus: a pointer to the &struct qsfp_bus structure for the qsfp module
 *
 * Delete a previously registered upstream connection for the qsfp
 * module. @bus should have been added by qsfp_bus_add_upstream().
 */
void qsfp_bus_del_upstream(struct qsfp_bus *bus)
{
	if (bus) {
		rtnl_lock();
		if (bus->qsfp)
			qsfp_unregister_bus(bus);
		qsfp_upstream_clear(bus);
		rtnl_unlock();

		qsfp_bus_put(bus);
	}
}
EXPORT_SYMBOL_GPL(qsfp_bus_del_upstream);

/* Socket driver entry points */
int qsfp_add_phy(struct qsfp_bus *bus, struct phy_device *phydev)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);
	int ret = 0;

	if (ops && ops->connect_phy)
		ret = ops->connect_phy(bus->upstream, phydev);

	if (ret == 0)
		bus->phydev = phydev;

	return ret;
}
EXPORT_SYMBOL_GPL(qsfp_add_phy);

void qsfp_remove_phy(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);

	if (ops && ops->disconnect_phy)
		ops->disconnect_phy(bus->upstream);
	bus->phydev = NULL;
}
EXPORT_SYMBOL_GPL(qsfp_remove_phy);

void qsfp_link_up(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);

	if (ops && ops->link_up)
		ops->link_up(bus->upstream);
}
EXPORT_SYMBOL_GPL(qsfp_link_up);

void qsfp_link_down(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);

	if (ops && ops->link_down)
		ops->link_down(bus->upstream);
}
EXPORT_SYMBOL_GPL(qsfp_link_down);

int qsfp_module_insert(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);
	int ret = 0;

	pr_info("Tessolve module insert\n");

	bus->qsfp_quirk = qsfp_lookup_quirk(id);

	if (ops && ops->module_insert)
		ret = ops->module_insert(bus->upstream, id);

	return ret;
}
EXPORT_SYMBOL_GPL(qsfp_module_insert);

void qsfp_module_remove(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);

	if (ops && ops->module_remove)
		ops->module_remove(bus->upstream);

	bus->qsfp_quirk = NULL;
}
EXPORT_SYMBOL_GPL(qsfp_module_remove);

int qsfp_module_start(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);
	int ret = 0;

	if (ops && ops->module_start)
		ret = ops->module_start(bus->upstream);

	return ret;
}
EXPORT_SYMBOL_GPL(qsfp_module_start);

void qsfp_module_stop(struct qsfp_bus *bus)
{
	const struct qsfp_upstream_ops *ops = qsfp_get_upstream_ops(bus);

	if (ops && ops->module_stop)
		ops->module_stop(bus->upstream);
}
EXPORT_SYMBOL_GPL(qsfp_module_stop);

static void qsfp_socket_clear(struct qsfp_bus *bus)
{
	bus->qsfp_dev = NULL;
	bus->qsfp = NULL;
	bus->socket_ops = NULL;
}

struct qsfp_bus *qsfp_register_socket(struct device *dev, struct qsfp *qsfp,
				      const struct qsfp_socket_ops *ops)
{
	struct qsfp_bus *bus = qsfp_bus_get(dev->fwnode);
	int ret = 0;

	if (bus) {
		rtnl_lock();
		bus->qsfp_dev = dev;
		bus->qsfp = qsfp;
		bus->socket_ops = ops;

		if (bus->upstream_ops) {
			ret = qsfp_register_bus(bus);
			if (ret)
				qsfp_socket_clear(bus);
		}
		rtnl_unlock();
	}

	if (ret) {
		qsfp_bus_put(bus);
		bus = NULL;
	}

	return bus;
}
EXPORT_SYMBOL_GPL(qsfp_register_socket);

void qsfp_unregister_socket(struct qsfp_bus *bus)
{
	rtnl_lock();
	if (bus->upstream_ops)
		qsfp_unregister_bus(bus);
	qsfp_socket_clear(bus);
	rtnl_unlock();

	qsfp_bus_put(bus);
}
EXPORT_SYMBOL_GPL(qsfp_unregister_socket);
