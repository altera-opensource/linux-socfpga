/* SPDX-License-Identifier: GPL-2.0 */
/* Intel QSFP_MODULE_DRIVER
 * Copyright (C) 2020-2021 Intel Corporation. All rights reserved.
 *
 * Contributors:
 * Malku
 * Deepak
 * Original driver contributed by Intel.
 */
#ifndef LINUX_QSFP_H
#define LINUX_QSFP_H

#include <linux/phy.h>
#include <linux/ethtool.h>

struct qsfp;
struct qsfp_eeprom_base {
	u8 etile_qsfp_identifier;					//0x00 00
	u8 etile_qsfp_revision;						//0x01 01
	u8 etile_qsfp_status;						//0x02 02
	u8 etile_qsfp_interrupt_flags[19];			//0x03 03
	u8 etile_qsfp_device_monitors[12];			//0x16 22
	u8 etile_qsfp_channel_monitors[48];			//0x22 34
	u8 RESERVED_0[4];							//0x52 82
	u8 etile_qsfp_control[13];					//0x56 86
	u8 RESERVED_1;								//0x63 99
	u8 etile_qsfp_device_channel_masks[5];		//0x64 100
	u8 etile_qsfp_vendor_specific[2];			//0x69 105
	u8 RESERVED_2;								//0x6b 107
	u8 etile_device_properties_1[3];			//0x6c 108
	u8 etile_pci_express[2];					//0x6f 111
	u8 etile_device_properties_2[2];			//0x71 113
	u8 RESERVED_3[4];							//0x74 115
	u8 etile_qsfp_password_change[4];			//0x77 119
	u8 etile_qsfp_password_entry_area[4];		//0x7b 123
	u8 etile_qsfp_page_select_byte;				//0x7f 127
	u8 etile_qsfp_identifier_1;					//0x80 128
	u8 etile_qsfp_ext_identifier;				//0x81 129
	u8 etile_qsfp_connector_type;				//0x82 130
	u8 etile_qsfp_spec_compliance_1[8];			//0x83 131
	u8 etile_qsfp_encoding;						//0x8b 139
	u8 etile_qsfp_br_nom;						//0x8c 140
	u8 etile_qsfp_ext_compliance;				//0x8d 141
	u8 etile_qsfp_link_lenghth_1;				//0x8e 142
	u8 etile_qsfp_link_lenghth_2;				//0x8f 143
	u8 etile_qsfp_link_lenghth_3;				//0x90 144
	u8 etile_qsfp_link_lenghth_4;				//0x91 145
	u8 etile_qsfp_link_lenghth_5;				//0x92 146
	u8 etile_qsfp_device_technology;			//0x93 147
	char etile_qsfp_vendor_name[16];			//0x94 148
	u8 etile_qsfp_extended_module;				//0xa4 164
	char etile_qsfp_vendor_oui[3];				//0xa5 165
	char etile_qsfp_vendor_pn[16];				//0xa8 168
	char etile_qsfp_vendor_rev[2];				//0xb8 184
	u8 etile_qsfp_wavelength_copper[2];			//0xba 186
	u8 etile_qsfp_wavelength_tolerance[2];		//0xbc 188
	u8 etile_qsfp_max_case_temp[1];				//0xbe 190
	u8 etile_qsfp_cc_base;						//0xbf 191
	u8 etile_qsfp_ext_spec_compliance;			//0xc0 192
	u8 etile_qsfp_options_1;					//0xc1 193
	u8 etile_qsfp_options_2;					//0xc2 194
	u8 etile_qsfp_options_3;					//0xc3 195
	char etile_qsfp_vendor_serial_number[16];	//0xc4 196
	char etile_qsfp_vendor_date_code[8];		//0xd4 212
	u8 etile_qsfp_diag_monitor;					//0xdc 220
	u8 etile_qsfp_enhanced_options;				//0xdd 221
	u8 etile_qsfp_br_nom_1;						//0xde 222
	u8 etile_qsfp_cc_ext;						//0xdf 223
	u8 etile_qsfp_venodor_specific_id[32];		//0xe0 224
	u8 etile_qsfp_br_max;						//TBD
	u8 etile_qsfp_br_min;						//TBD

} __packed;


	/**
	 * struct qsfp_eeprom_id - raw qsfp module identification information
	 * @base: base qsfp module identification structure
	 * @ext: extended qsfp module identification structure
	 *
	 * See the SFF-8472 specification and related documents for the definition
	 * of these structure members. This can be obtained from
	 * https://www.snia.org/technology-communities/sff/specifications
	 */
struct qsfp_eeprom_id {
	struct qsfp_eeprom_base base;

} __packed;



enum {

	SFF8024_ID_QSFP_DD_INF_8628	= 0x18,
	SFF8024_QSFP_DD_ENCODING_UNSPEC	= 0x00,
	SFF8024_QSFP_DD_ENCODING_8B10B	= 0x01,
	SFF8024_QSFP_DD_ENCODING_4B5B	= 0x02,
	SFF8024_QSFP_DD_ENCODING_NRZ	= 0x03,
	SFF8024_QSFP_DD_ENCODING_8436_SONET	= 0x04,
	SFF8024_QSFP_DD_ENCODING_8436_64B66B	= 0x05,
	SFF8024_QSFP_DD_ENCODING_8436_MANCHESTER	= 0x06,
	SFF8024_QSFP_DD_ENCODING_256B257B	= 0x07,
	SFF8024_QSFP_DD_ENCODING_PAM4	= 0x08,
	SFF8024_ID_QSFP_28	= 0x11,

	SFF8024_QSFP_DD_CONNECTOR_UNSPEC = 0x00,
	SFF8024_QSFP_DD_CONNECTOR_SC = 0x01,
	SFF8024_QSFP_DD_CONNECTOR_FIBRE_CHANNEL_STYLE1 = 0x02,
	SFF8024_QSFP_DD_CONNECTOR_FIBRE_CHANNEL_STYLE2 = 0x03,
	SFF8024_QSFP_DD_CONNECTOR_BNC_TNC = 0x04,
	SFF8024_QSFP_DD_CONNECTOR_FIBRE_CHANNEL_COAX_HEADERS = 0x05,
	SFF8024_QSFP_DD_CONNECTOR_FIBERJACK = 0x06,
	SFF8024_QSFP_DD_CONNECTOR_LC = 0x07,
	SFF8024_QSFP_DD_CONNECTOR_MT_RJ = 0x08,
	SFF8024_QSFP_DD_CONNECTOR_MU = 0x09,
	SFF8024_QSFP_DD_CONNECTOR_SG = 0x0a,
	SFF8024_QSFP_DD_CONNECTOR_OPTICAL_PIGTAIL = 0x0b,
	SFF8024_QSFP_DD_CONNECTOR_MPO_1X12 = 0x0c,
	SFF8024_QSFP_DD_CONNECTOR_MPO_2X16 = 0x0d,
	SFF8024_QSFP_DD_CONNECTOR_HSSDC_II = 0x20,
	SFF8024_QSFP_DD_CONNECTOR_COPPER_PIGTAIL = 0x21,
	SFF8024_QSFP_DD_CONNECTOR_RJ45 = 0x22,
	SFF8024_QSFP_DD_CONNECTOR_NOSEPARATE = 0x23,
	SFF8024_QSFP_DD_CONNECTOR_MXC_2X16 = 0x24,
	SFF8024_QSFP_DD_CONNECTOR_CS_OPTICAL_CONNECTOR = 0x25,
	SFF8024_QSFP_DD_CONNECTOR_SN_OPICAL_CONNECTOR = 0x26,
	SFF8024_QSFP_DD_CONNECTOR_MPO_2X12 = 0x27,
	SFF8024_QSFP_DD_CONNECTOR_MXC_1X16 = 0x28,

	SFF8636_QSFP_DD_ECC_100GBASE_CR4 = 1,
	SFF8636_QSFP_DD_ECC_CAUI4 = 0,

	SFF8636_QSFP_DD_ECC_LAUI2_C2M = 3,
	SFF8636_QSFP_DD_ECC_50GAUI2_C2M = 2,
	SFF8636_QSFP_DD_ECC_50GAUI1_C2M = 1,
	SFF8636_QSFP_DD_ECC_CDAUI8_C2M = 0,

};

/* qsfp EEPROM registers */
enum {
	QSFP_PHYS_ID	= 0x00,
	QSFP_STATUS	= 0x01,
	QSFP_STATUS_1	= 0x02,
	QSFP_RX_TX_LOSS	= 0x03,
	QSFP_TX_FAULT	= 0x04,
	QSFP_DEVICE_MONITORS	= 0x16,
	QSFP_CHANNEL_MONITORS	= 0x22,
	QSFP_CONTROL	= 0x56,
	QSFP_DEVICE_CHANNEL_MASKS	= 0x64,
	QSFP_VENDOR_SPECIFIC	= 0x69,
	QSFP_DEVICE_PROPERTIES	= 0x6C,
	QSFP_PCI_EXPRESS	= 0x6F,
	QSFP_DEVICE_PROPERTIES_1	= 0x71,
	QSFP_PASSWORD_CHANGE_AREA	= 0x77,
	QSFP_PASSWORD_ENTRY_AREA	= 0x7B,
	QSFP_PAGE_SELECT_BYTE	= 0x7F,
	QSFP_IDENTIFIER	= 0x80,
	QSFP_EXT_IDENTIFIER	= 0x81,
	QSFP_CONNECTOR	= 0x82,
	QSFP_COMPLIANCE	= 0x83,
	QSFP_ENCODING	= 0x8b,
	QSFP_BR_NOMINAL	= 0x8C,
	QSFP_EXT_RATE_SELECT	= 0x8d,
	QSFP_LINK_LEN_SMF	= 0x8e,
	QSFP_LINK_LEN_OM3_50	= 0x8f,
	QSFP_LINK_LEN_OM2_50	= 0x90,
	QSFP_LINK_LEN_OM2_62_5	= 0x91,
	QSFP_LINK_LEN_COPPER_1M	= 0x92,
	QSFP_DEVICE_TECHNOLOGY	= 0x93,
	QSFP_VENDOR_NAME	= 0x94,
	QSFP_VENDOR_OUI	= 0xa5,
	QSFP_VENDOR_PN	= 0xa8,
	QSFP_VENDOR_REV	= 0xb8,
	QSFP_MAX_CASE_TEMP	= 0xbe,
	QSFP_CC_BASE	= 0xbf,
	QSFP_OPTIONS	= 0xC3,
	QSFP_VENDOR_SN	= 0xC4,
	QSFP_DATECODE	= 0xD4,
	QSFP_DIAGMON	= 0xDc,
	QSFP_ENHOPTS	= 0xDD,
	QSFP_CC_EXT	= 0xDf,

			QSFP_TX4_LOS_CHANNEL_4 = BIT(7),
			QSFP_TX3_LOS_CHANNEL_3 = BIT(6),
			QSFP_TX2_LOS_CHANNEL_2 = BIT(5),
			QSFP_TX1_LOS_CHANNEL_1 = BIT(4),
			QSFP_RX4_LOS_CHANNEL_4 = BIT(3),
			QSFP_RX3_LOS_CHANNEL_3 = BIT(2),
			QSFP_RX2_LOS_CHANNEL_2 = BIT(1),
			QSFP_RX1_LOS_CHANNEL_1 = BIT(0),

	QSFP_OPTIONS_TX_INPUT_ADAPTIVE	= BIT(20),
	QSFP_OPTIONS_TX_INPUT_EQUALIZATION	= BIT(19),
	QSFP_OPTIONS_TX_INPUT_EQUALIZATION_FIXED	= BIT(18),
	QSFP_OPTIONS_RX_OUTPUT_EMPHASIS	= BIT(17),
	QSFP_OPTIONS_RX_OUTPUT_APLITUDE	= BIT(16),
	QSFP_OPTIONS_TX_CDR_CONTROL	= BIT(15),
	QSFP_OPTIONS_RX_CDR_CONTROL	= BIT(14),
	QSFP_OPTIONS_TX_CDR_LOSS	= BIT(13),
	QSFP_OPTIONS_RX_CDR_LOSS	= BIT(12),
	QSFP_OPTIONS_RX_SQUELCH_DISABLE	= BIT(11),
	QSFP_OPTIONS_RX_OUTPUT_DISBALE	= BIT(10),
	QSFP_OPTIONS_TX_SQUELCH_DISABLE	= BIT(9),
	QSFP_OPTIONS_TX_OUTPUT_DISBALE	= BIT(8),
	QSFP_OPTIONS_MEMORY_PAGE_2	= BIT(7),
	QSFP_OPTIONS_MEMORY_PAGE_1	= BIT(6),
	QSFP_OPTIONS_RATE_SELECT	= BIT(5),
	QSFP_OPTIONS_TX_DISABLE	= BIT(4),
	QSFP_OPTIONS_TX_FAULT	= BIT(3),
	QSFP_OPTIONS_TX_SQUELCH_IMPLEMENTED	= BIT(2),
	QSFP_OPTIONS_TX_LOSS_SIGNAL	= BIT(1),
	QSFP_OPTIONS_PAGES	= BIT(0),



	QSFP_DIAGMON_DDM	= BIT(6),
	QSFP_DIAGMON_INT_CAL	= BIT(5),
	QSFP_DIAGMON_EXT_CAL	= BIT(4),
	QSFP_DIAGMON_RXPWR_AVG	= BIT(3),
	QSFP_DIAGMON_ADDRMODE	= BIT(2),

	QSFP_DIAGMON_RX_OPTICAL_POWER_MONITOR	= BIT(4),
	QSFP_DIAGMON_RX_OPTICAL_POWER_MEASUREMENT_TYPE	= BIT(3),
	QSFP_DIAGMON_TX_OPTICAL_POWER_MONITOR	= BIT(2),
	QSFP_DIAGMON_TX_BIAS_MONITOR_IMPLEMENTED	= BIT(1),
	QSFP_DIAGMON_RESERVED	= BIT(0),

	QSFP_ENHOPTS_USER_DEFINED	= BIT(7),
	QSFP_ENHOPTS_VENDOR_SPECIFIC	= BIT(6),
	QSFP_ENHOPTS_INTERNAL_3_VOLTS	= BIT(5),
	QSFP_ENHOPTS_POWER_CHANGE_COMPLETE	= BIT(4),
	QSFP_ENHOPTS_RX_RATE_SELECT	= BIT(3),
	QSFP_ENHOPTS_APPLICATION_SELECTED	= BIT(2),

	QSFP_SFF8472_COMPLIANCE_NONE = 0x00,
	QSFP_SFF8472_COMPLIANCE_REV9_3 = 0x01,
	QSFP_SFF8472_COMPLIANCE_REV9_5 = 0x02,
	QSFP_SFF8472_COMPLIANCE_REV10_2 = 0x03,
	QSFP_SFF8472_COMPLIANCE_REV10_4 = 0x04,
	QSFP_SFF8472_COMPLIANCE_REV11_0 = 0x05,
	QSFP_SFF8472_COMPLIANCE_REV11_3 = 0x06,
	QSFP_SFF8472_COMPLIANCE_REV11_4 = 0x07,
	QsSFP_SFF8472_COMPLIANCE_REV12_0 = 0x08,

	QSFP_EXT_STATUS = 0x76,

};

struct fwnode_handle;
struct ethtool_eeprom;
struct ethtool_modinfo;
struct qsfp_bus;

	/**
	 * struct qsfp_upstream_ops - upstream operations structure
	 * @attach: called when the qsfp socket driver is bound to the upstream
	 *   (mandatory).
	 * @detach: called when the qsfp socket driver is unbound from the upstream
	 *   (mandatory).
	 * @module_insert: called after a module has been detected to determine
	 *   whether the module is supported for the upstream device.
	 * @module_remove: called after the module has been removed.
	 * @module_start: called after the PHY probe step
	 * @module_stop: called before the PHY is removed
	 * @link_down: called when the link is non-operational for whatever
	 *   reason.
	 * @link_up: called when the link is operational.
	 * @connect_phy: called when an I2C accessible PHY has been detected
	 *   on the module.
	 * @disconnect_phy: called when a module with an I2C accessible PHY has
	 *   been removed.
	 */
struct qsfp_upstream_ops {
	void (*attach)(void *priv, struct qsfp_bus *bus);
	void (*detach)(void *priv, struct qsfp_bus *bus);
	int (*module_insert)(void *priv, const struct qsfp_eeprom_id *id);
	void (*module_remove)(void *priv);
	int (*module_start)(void *priv);
	void (*module_stop)(void *priv);
	void (*link_down)(void *priv);
	void (*link_up)(void *priv);
	int (*connect_phy)(void *priv, struct phy_device *phydev);
	void (*disconnect_phy)(void *priv);
};

struct qsfp_socket_ops {
	void (*attach)(struct qsfp *qsfp);
	void (*detach)(struct qsfp *qsfp);
	void (*start)(struct qsfp *qsfp);
	void (*stop)(struct qsfp *qsfp);
	int (*module_info)(struct qsfp *qsfp, struct ethtool_modinfo *modinfo);
	int (*module_eeprom)(struct qsfp *qsfp, struct ethtool_eeprom *ee,
			     u8 *data);
};

int qsfp_add_phy(struct qsfp_bus *bus, struct phy_device *phydev);
void qsfp_remove_phy(struct qsfp_bus *bus);
void qsfp_link_up(struct qsfp_bus *bus);
void qsfp_link_down(struct qsfp_bus *bus);
int qsfp_module_insert(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id);
void qsfp_module_remove(struct qsfp_bus *bus);
int qsfp_module_start(struct qsfp_bus *bus);
void qsfp_module_stop(struct qsfp_bus *bus);
int qsfp_link_configure(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id);
struct qsfp_bus *qsfp_register_socket(struct device *dev, struct qsfp *qsfp,
				      const struct qsfp_socket_ops *ops);
void qsfp_unregister_socket(struct qsfp_bus *bus);

int get_cable_attach(struct qsfp *old);
int get_channel_info(struct qsfp *old);

#if IS_ENABLED(CONFIG_QSFP)
int qsfp_parse_port(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id,
		    unsigned long *support);
bool qsfp_may_have_phy(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id);
void qsfp_parse_support(struct qsfp_bus *bus, const struct qsfp_eeprom_id *id,
			unsigned long *support);
phy_interface_t qsfp_select_interface(struct qsfp_bus *bus,
				      unsigned long *link_modes);

int qsfp_get_module_info(struct qsfp_bus *bus, struct ethtool_modinfo *modinfo);
int qsfp_get_module_eeprom(struct qsfp_bus *bus, struct ethtool_eeprom *ee,
			   u8 *data);
void qsfp_upstream_start(struct qsfp_bus *bus);
void qsfp_upstream_stop(struct qsfp_bus *bus);
void qsfp_bus_put(struct qsfp_bus *bus);
struct qsfp_bus *qsfp_bus_find_fwnode(struct fwnode_handle *fwnode);
int qsfp_bus_add_upstream(struct qsfp_bus *bus, void *upstream,
			  const struct qsfp_upstream_ops *ops);
void qsfp_bus_del_upstream(struct qsfp_bus *bus);
#else

static inline int qsfp_parse_port(struct qsfp_bus *bus,
				  const struct qsfp_eeprom_id *id,
				  unsigned long *support)
{
	return PORT_OTHER;
}

static inline bool qsfp_may_have_phy(struct qsfp_bus *bus,
				     const struct qsfp_eeprom_id *id)
{
	return false;
}

static inline void qsfp_parse_support(struct qsfp_bus *bus,
				      const struct qsfp_eeprom_id *id,
				      unsigned long *support)
{
}

static inline phy_interface_t qsfp_select_interface(struct qsfp_bus *bus,
						    unsigned long *link_modes)
{
	return PHY_INTERFACE_MODE_NA;
}

static inline int qsfp_get_module_info(struct qsfp_bus *bus,
				       struct ethtool_modinfo *modinfo)
{
	return -EOPNOTSUPP;
}

static inline int qsfp_get_module_eeprom(struct qsfp_bus *bus,
					 struct ethtool_eeprom *ee, u8 *data)
{
	return -EOPNOTSUPP;
}

static inline void qsfp_upstream_start(struct qsfp_bus *bus)
{
}

static inline void qsfp_upstream_stop(struct qsfp_bus *bus)
{
}

static inline void qsfp_bus_put(struct qsfp_bus *bus)
{
}

static inline struct qsfp_bus *
qsfp_bus_find_fwnode(struct fwnode_handle *fwnode)
{
	return NULL;
}

static inline int qsfp_bus_add_upstream(struct qsfp_bus *bus, void *upstream,
					const struct qsfp_upstream_ops *ops)
{
	return 0;
}

static inline void qsfp_bus_del_upstream(struct qsfp_bus *bus)
{
}

#endif
#endif /* LINUX_QSFP_H */
