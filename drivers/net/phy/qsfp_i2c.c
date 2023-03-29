// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mdio/mdio-i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/qsfp.h>

#include "swphy.h"

static struct qsfp *qsfp;

enum {
	GPIO_MODULE_PRESENT,
	GPIO_MODULE_INTERRUPT,
	GPIO_MODULE_INIT_MODE,
	GPIO_MODULE_RESET,
	GPIO_MODULE_SELECT,
	GPIO_MAX,

	QSFP_F_PRESENT = BIT(GPIO_MODULE_PRESENT),
	QSFP_INTERRUPT = BIT(GPIO_MODULE_INTERRUPT),
	QSFP_INIT = BIT(GPIO_MODULE_INIT_MODE),
	QSFP_RESET = BIT(GPIO_MODULE_RESET),
	QSFP_SELECT = BIT(GPIO_MODULE_SELECT),

	QSFP_E_INSERT = 0,
	QSFP_E_REMOVE,
	QSFP_E_DEV_ATTACH,
	QSFP_E_DEV_DETACH,
	QSFP_E_DEV_DOWN,
	QSFP_E_DEV_UP,
	QSFP_E_TX_FAULT,
	QSFP_E_TX_CLEAR,
	QSFP_E_TX_LOS,
	QSFP_E_RX_LOS,
	QSFP_E_TIMEOUT,

	QSFP_MOD_EMPTY = 0,
	QSFP_MOD_ERROR,
	QSFP_MOD_PROBE,
	QSFP_MOD_WAITDEV,
	QSFP_MOD_HPOWER,
	QSFP_MOD_WAITPWR,
	QSFP_MOD_PRESENT,

	QSFP_DEV_DETACHED = 0,
	QSFP_DEV_DOWN,
	QSFP_DEV_UP,

	QSFP_S_DOWN = 0,
	QSFP_S_FAIL,
	QSFP_S_WAIT,
	QSFP_S_INIT,
	QSFP_S_INIT_PHY,
	QSFP_S_INIT_TX_FAULT,
	QSFP_S_WAIT_LOS,
	QSFP_S_LINK_UP,
	QSFP_S_TX_FAULT,
	QSFP_S_REINIT,
	QSFP_S_TX_DISABLE,
};

static const char *const mod_state_strings[] = {
	[QSFP_MOD_EMPTY] = "empty",
	[QSFP_MOD_ERROR] = "error",
	[QSFP_MOD_PROBE] = "probe",
	[QSFP_MOD_WAITDEV] = "waitdev",
	[QSFP_MOD_HPOWER] = "hpower",
	[QSFP_MOD_WAITPWR] = "waitpwr",
	[QSFP_MOD_PRESENT] = "present",
};

static const char *const dev_state_strings[] = {
	[QSFP_DEV_DETACHED] = "detached",
	[QSFP_DEV_DOWN] = "down",
	[QSFP_DEV_UP] = "up",
};

static const char *const event_strings[] = {
	[QSFP_E_INSERT] = "insert",
	[QSFP_E_REMOVE] = "remove",
	[QSFP_E_DEV_ATTACH] = "dev_attach",
	[QSFP_E_DEV_DETACH] = "dev_detach",
	[QSFP_E_DEV_DOWN] = "dev_down",
	[QSFP_E_DEV_UP] = "dev_up",
	[QSFP_E_TX_FAULT] = "tx_fault",
	[QSFP_E_TX_CLEAR] = "tx_clear",
	[QSFP_E_TX_LOS] = "tx_los",
	[QSFP_E_RX_LOS] = "rx_los",
	[QSFP_E_TIMEOUT] = "timeout",
};

static const char *const sm_state_strings[] = {
	[QSFP_S_DOWN] = "down",
	[QSFP_S_FAIL] = "fail",
	[QSFP_S_WAIT] = "wait",
	[QSFP_S_INIT] = "init",
	[QSFP_S_INIT_PHY] = "init_phy",
	[QSFP_S_INIT_TX_FAULT] = "init_tx_fault",
	[QSFP_S_WAIT_LOS] = "wait_los",
	[QSFP_S_LINK_UP] = "link_up",
	[QSFP_S_TX_FAULT] = "tx_fault",
	[QSFP_S_REINIT] = "reinit",
	[QSFP_S_TX_DISABLE] = "rx_disable",
};

static const char *const gpio_of_names[] = {
	"qsfpdd_modprsn",
	"qsfpdd_intn",
	"qsfpdd_initmode",
	"qsfpdd_resetn",
	"qsfpdd_modseln",
};

static const enum gpiod_flags gpio_flags[] = {
	GPIOD_IN, GPIOD_IN, GPIOD_ASIS, GPIOD_ASIS, GPIOD_ASIS,
};

/* t_start_up (SFF-8431) or t_init (SFF-8472) is the time required for a
 * non-cooled module to initialise its laser safety circuitry. We wait
 * an initial T_WAIT period before we check the tx fault to give any PHY
 * on board (for a copper qsfp) time to initialise.
 */
#define T_START_UP msecs_to_jiffies(300)

/* qsfp modules appear to always have their PHY configured for bus address
 * 0x56 (which with mdio-i2c, translates to a PHY address of 22).
 */
#define QSFP_PHY_ADDR 22

struct sff_data {
	unsigned int gpios;
	bool (*module_supported)(const struct qsfp_eeprom_id *id);
};

struct qsfp {
	struct device *dev;
	struct i2c_adapter *i2c;
	struct mii_bus *i2c_mii;
	struct qsfp_bus *qsfp_bus;
	struct phy_device *mod_phy;
	const struct sff_data *type;
	size_t i2c_block_size;
	u32 max_power_mw;
	unsigned int module_revision;
	unsigned int module_present;
	unsigned int tx_rx_loss_info;
	unsigned int options_state;
	int channel_number;
	unsigned int sm_probe_phy_err;
	unsigned char vender_name[16];
	unsigned char sn_number[16];
	unsigned char part_number[16];

	unsigned int (*get_state)(struct qsfp *qsfp);
	void (*set_state)(struct qsfp *qsfp, unsigned int state);
	int (*read)(struct qsfp *qsfp, bool a2, u8 dev_addr, void *buf, size_t len);
	int (*write)(struct qsfp *qsfp, bool a2, u8 dev_addr, void *buf, size_t len);

	struct gpio_desc *gpio[GPIO_MAX];
	int gpio_irq[GPIO_MAX];

	bool need_poll;

	struct mutex st_mutex; /* Protects state */
	unsigned int state_soft_mask;
	unsigned int state;
	struct delayed_work poll;
	struct delayed_work timeout;
	struct mutex sm_mutex; /* Protects state machine */
	unsigned char sm_mod_state;
	unsigned char sm_mod_tries_init;
	unsigned char sm_mod_tries;
	unsigned char sm_dev_state;
	unsigned short sm_state;
	unsigned char sm_fault_retries;
	unsigned char sm_phy_retries;

	struct qsfp_eeprom_id id;
	unsigned int module_power_mw;
	unsigned int module_t_start_up;
	int channel_info;
};

static bool qsfp_module_supported(const struct qsfp_eeprom_id *id)
{
	if (id->base.etile_qsfp_identifier == SFF8024_ID_QSFP ||
	    id->base.etile_qsfp_identifier == SFF8024_ID_QSFP_PLUS ||
		id->base.etile_qsfp_identifier == SFF8024_ID_QSFP_28)
		return true;

	/* qsfp GPON module Ubiquiti U-Fiber Instant has in its EEPROM stored
	 * phys id SFF instead of qsfp. Therefore mark this module explicitly
	 * as supported based on vendor name and pn match.
	 */

	if (id->base.etile_qsfp_identifier == SFF8024_ID_QSFP_DD_INF_8628 &&
	    id->base.etile_qsfp_ext_identifier == QSFP_EXT_IDENTIFIER &&
	    !memcmp(id->base.etile_qsfp_vendor_name, "UBNT            ", 16) &&
	    !memcmp(id->base.etile_qsfp_vendor_pn, "UF-INSTANT      ", 16))
		return true;

	return false;
}

static const struct sff_data qsfp_data = {
	.gpios = QSFP_F_PRESENT | QSFP_INTERRUPT | QSFP_INIT | QSFP_RESET |
		 QSFP_SELECT,
	.module_supported = qsfp_module_supported,
};

static const struct of_device_id qsfp_of_match[] = {
	{
		.compatible = "sff,qsfp-multi-channel",
		.data = &qsfp_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, qsfp_of_match);

static unsigned long poll_jiffies;

/*static */
unsigned int qsfp_gpio_get_state(struct qsfp *qsfp)
{
	unsigned int i, state, v;

	for (i = state = 0; i < GPIO_MAX; i++) {
		if (gpio_flags[i] != GPIOD_IN || !qsfp->gpio[i])
			continue;

		v = gpiod_get_value_cansleep(qsfp->gpio[i]);
		if (v)
			state |= BIT(i);
	}

	return state;
}
EXPORT_SYMBOL(qsfp_gpio_get_state);

/*static*/
unsigned int sff_gpio_get_state(struct qsfp *qsfp)
{
	int ret;

	ret = qsfp_gpio_get_state(qsfp) | QSFP_F_PRESENT;
	return ret;
}
EXPORT_SYMBOL(sff_gpio_get_state);

void qsfp_gpio_set_state(struct qsfp *qsfp, unsigned int state)
{
	if (state & QSFP_F_PRESENT) {
		/* If the module is present, drive the signals */
		gpiod_direction_output(qsfp->gpio[GPIO_MODULE_SELECT],
				       state & QSFP_SELECT);

	} else {
		/* Otherwise, let them float to the pull-ups */

		gpiod_direction_input(qsfp->gpio[GPIO_MODULE_PRESENT]);
	}
}
EXPORT_SYMBOL(qsfp_gpio_set_state);

static int qsfp_i2c_read(struct qsfp *qsfp, bool a2, u8 dev_addr, void *buf,
			 size_t len)
{
	struct i2c_msg msgs[2];
	u8 bus_addr = a2 ? 0x51 : 0x50;
	size_t block_size = qsfp->i2c_block_size;
	size_t this_len;
	int ret;

	msgs[0].addr = bus_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &dev_addr;
	msgs[1].addr = bus_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	while (len) {
		this_len = len;
		if (this_len > block_size)
			this_len = block_size;

		msgs[1].len = this_len;

		ret = i2c_transfer(qsfp->i2c, msgs, ARRAY_SIZE(msgs));
		if (ret < 0)
			return ret;

		if (ret != ARRAY_SIZE(msgs))
			break;

		msgs[1].buf += this_len;
		dev_addr += this_len;
		len -= this_len;
	}

	return msgs[1].buf - (u8 *)buf;
}

static int qsfp_i2c_write(struct qsfp *qsfp, bool a2, u8 dev_addr, void *buf,
			  size_t len)
{
	struct i2c_msg msgs[1];
	u8 bus_addr = a2 ? 0x51 : 0x50;
	int ret;

	msgs[0].addr = bus_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1 + len;
	msgs[0].buf = kmalloc(1 + len, GFP_KERNEL);
	if (!msgs[0].buf)
		return -ENOMEM;

	msgs[0].buf[0] = dev_addr;
	memcpy(&msgs[0].buf[1], buf, len);

	ret = i2c_transfer(qsfp->i2c, msgs, ARRAY_SIZE(msgs));

	kfree(msgs[0].buf);

	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? len : 0;
}

static int qsfp_i2c_configure(struct qsfp *qsfp, struct i2c_adapter *i2c)
{
	struct mii_bus *i2c_mii;
	int ret;

	if (!i2c_check_functionality(i2c, I2C_FUNC_I2C))
		return -EINVAL;

	qsfp->i2c = i2c;
	qsfp->read = qsfp_i2c_read;
	qsfp->write = qsfp_i2c_write;

	i2c_mii = mdio_i2c_alloc(qsfp->dev, i2c);
	if (IS_ERR(i2c_mii))
		return PTR_ERR(i2c_mii);
	i2c_mii->name = "QSFP I2C Bus";
	i2c_mii->phy_mask = ~0;

	ret = mdiobus_register(i2c_mii);
	if (ret < 0) {
		;
		mdiobus_free(i2c_mii);
		return ret;
	}

	qsfp->i2c_mii = i2c_mii;

	return 0;
}

/* Interface */

static int qsfp_read(struct qsfp *qsfp, bool a2, u8 addr, void *buf, size_t len)
{
	return qsfp->read(qsfp, a2, addr, buf, len);
}

static int qsfp_write(struct qsfp *qsfp, bool a2, u8 addr, void *buf,
		      size_t len)
{
	return qsfp->write(qsfp, a2, addr, buf, len);
}

unsigned int qsfp_get_state(struct qsfp *qsfp)
{
	unsigned int state = qsfp->get_state(qsfp);
	return state;

} EXPORT_SYMBOL(qsfp_get_state);

void qsfp_set_state(struct qsfp *qsfp, unsigned int state)
{
	qsfp->set_state(qsfp, state);

} EXPORT_SYMBOL(qsfp_set_state);

static unsigned int qsfp_check(void *buf, size_t len)
{
	u8 *p, check;

	for (p = buf, check = 0; len; p++, len--)
		check += *p;

	return check;
}

static int qsfp_sm_probe_phy(struct qsfp *qsfp, bool is_c45)
{
	struct phy_device *phy;
	int err;

	phy = get_phy_device(qsfp->i2c_mii, QSFP_PHY_ADDR, is_c45);

	if (phy == ERR_PTR(-ENODEV))
		return PTR_ERR(phy);

	if (IS_ERR(phy)) {
		dev_err(qsfp->dev, "mdiobus scan returned %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}

	err = phy_device_register(phy);
	if (err) {
		phy_device_free(phy);
		dev_err(qsfp->dev, "phy_device_register failed: %d\n", err);
		return err;
	}

	err = qsfp_add_phy(qsfp->qsfp_bus, phy);
	if (err) {
		phy_device_remove(phy);
		phy_device_free(phy);
		dev_err(qsfp->dev, "qsfp_add_phy failed: %d\n", err);
		return err;
	}
	qsfp->mod_phy = phy;

	return 0;
}

struct phy_device *gsfp_mod_phy(struct qsfp *old)
{
	return qsfp->mod_phy;
}
EXPORT_SYMBOL(gsfp_mod_phy);

static int qsfp_sm_probe_for_phy(struct qsfp *qsfp)
{
	int err = 0;

	switch (qsfp->id.base.etile_qsfp_spec_compliance_1[0]) {
	case SFF8636_QSFP_DD_ECC_LAUI2_C2M:
	case SFF8636_QSFP_DD_ECC_50GAUI2_C2M:
	case SFF8636_QSFP_DD_ECC_50GAUI1_C2M:
	case SFF8636_QSFP_DD_ECC_CDAUI8_C2M:
		err = qsfp_sm_probe_phy(qsfp, true);
		break;

	default:
	if (qsfp->id.base.etile_qsfp_spec_compliance_1[2] & SFF8024_QSFP_e1000_base_t) {
		err = qsfp_sm_probe_phy(qsfp, false);
		break;
		}
	}
	qsfp->sm_probe_phy_err = err;

	return qsfp->sm_probe_phy_err;
}

int gsfp_probe_phy(struct qsfp *old)
{
	return qsfp->sm_probe_phy_err;
}
EXPORT_SYMBOL(gsfp_probe_phy);

static int qsfp_module_parse_power(struct qsfp *qsfp)
{
	u32 power_mw = 1000;

	if (power_mw > qsfp->max_power_mw) {
		/* Module power specification exceeds the allowed maximum. */
		if (qsfp->id.base.etile_qsfp_spec_compliance_1[0] ==
			    SFF8636_QSFP_DD_ECC_100GBASE_CR4 &&
		    !(qsfp->id.base.etile_qsfp_diag_monitor &
		      QSFP_DIAGMON_DDM)) {
			/* The module appears not to implement bus address
			 * 0xa2, so assume that the module powers up in the
			 * indicated mode.
			 */
			dev_err(qsfp->dev,
				"Host does not support %u.%uW modules, module left in power mode\n",
				power_mw / 1000, (power_mw / 100) % 10);
			return -EINVAL;
		}
	}

	/* If the module requires a higher power mode, but also requires
	 * an address change sequence, warn the user that the module may
	 * not be functional.
	 */
	if (qsfp->id.base.etile_qsfp_diag_monitor & QSFP_DIAGMON_ADDRMODE &&
	    power_mw > 1000) {
		dev_warn(qsfp->dev,
			 "Address Change Sequence not supported but module requires %u.%uW, module may not be functional\n",
			power_mw / 1000, (power_mw / 100) % 10);
		return 0;
	}

	qsfp->module_power_mw = power_mw;

	return 0;
}

static bool qsfp_id_needs_byte_io(struct qsfp *qsfp, void *buf, size_t len)
{
	size_t i, block_size = qsfp->i2c_block_size;

	/* Already using byte IO */
	if (block_size == 1)
		return false;

	for (i = 1; i < len; i += block_size) {
		if (memchr_inv(buf + i, '\0', min(block_size - 1, len - i)))
			return false;
	}
	return true;
}

int get_module_revision(struct qsfp *qsfp)
{
	int ret;
	u8 buf[16] = {0};
	char buf_16[16] = {'\0'};

		ret = qsfp_read(qsfp, false, QSFP_VENDOR_NAME, buf_16, 16);
		buf_16[15] = '\0';
		strcpy(qsfp->vender_name, buf_16);
		ret = qsfp_read(qsfp, false, QSFP_VENDOR_PN, buf_16, 16);
		buf_16[15] = '\0';
		strcpy(qsfp->part_number, buf_16);
		ret = qsfp_read(qsfp, false, QSFP_VENDOR_SN, buf_16, 16);
		buf_16[15] = '\0';
		strcpy(qsfp->sn_number, buf_16);
		ret = qsfp_read(qsfp, false, QSFP_STATUS, buf, 1);
		qsfp->module_revision = buf[0];
		ret = qsfp_get_state(qsfp) & QSFP_F_PRESENT;
		qsfp->module_present = ret;
		ret = qsfp_read(qsfp, false, QSFP_RX_TX_LOSS, buf, 1);
		qsfp->tx_rx_loss_info = buf[0];

		ret = qsfp_sm_probe_for_phy(qsfp);
		ret = qsfp_read(qsfp, false, QSFP_OPTIONS, buf, 1);
		qsfp->options_state = buf[0] & QSFP_OPTIONS_TX_FAULT;

		/***************************************************/

		qsfp->state_soft_mask = 0;
		if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_DISABLE)
			qsfp->state_soft_mask |= QSFP_OPTIONS_TX_DISABLE;
		if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_FAULT)
			qsfp->state_soft_mask |= QSFP_OPTIONS_TX_FAULT;
		if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_LOSS_SIGNAL)
			qsfp->state_soft_mask |= QSFP_OPTIONS_TX_LOSS_SIGNAL;

	return 0;
}

int soft_mask(struct qsfp *old)
{
	return qsfp->state_soft_mask;
}
EXPORT_SYMBOL(soft_mask);

int get_qsfp_options(struct qsfp *old)
{
	return qsfp->options_state;
}
EXPORT_SYMBOL(get_qsfp_options);

int get_cable_attach(struct qsfp *old)
{
	return qsfp->module_present;
}
EXPORT_SYMBOL(get_cable_attach);

void get_cable_info(char *vender_name)
{
	strcpy(vender_name, qsfp->vender_name);
}
EXPORT_SYMBOL(get_cable_info);

int get_tx_rx_loss(struct qsfp *old)
{
	return qsfp->tx_rx_loss_info;
}
EXPORT_SYMBOL(get_tx_rx_loss);

static int qsfp_select_eeprom_page(struct qsfp *qsfp)
{
	int err;
	u8 buf[16];
	u8 i = 0;
	int ret;

	err = qsfp_write(qsfp, false, QSFP_PAGE_SELECT_BYTE, &i, 1);

	ret = qsfp_read(qsfp, false, QSFP_PAGE_SELECT_BYTE, buf, 1);

	return 0;
}

static int qsfp_cotsworks_fixup_check(struct qsfp *qsfp,
				      struct qsfp_eeprom_id *id)
{
	u8 check;
	int err;

	err = qsfp_write(qsfp, false, QSFP_PAGE_SELECT_BYTE, (u8 *)0x2, 1);

	if (id->base.etile_qsfp_identifier != SFF8024_ID_QSFP_DD_INF_8628 ||
	    id->base.etile_qsfp_ext_identifier != QSFP_EXT_IDENTIFIER ||
	    id->base.etile_qsfp_connector_type !=
		    SFF8024_QSFP_DD_CONNECTOR_LC) {
		dev_warn(qsfp->dev,
			 "Rewriting fiber module EEPROM with corrected values\n");
		id->base.etile_qsfp_identifier = SFF8024_ID_QSFP_DD_INF_8628;
		id->base.etile_qsfp_ext_identifier = QSFP_EXT_IDENTIFIER;
		id->base.etile_qsfp_connector_type =
			SFF8024_QSFP_DD_CONNECTOR_LC;
		err = qsfp_write(qsfp, false, QSFP_PHYS_ID, &id->base, 3);
		if (err != 3) {
			dev_err(qsfp->dev,
				"Failed to rewrite module EEPROM: %d\n", err);
			return err;
		}

		/* Cotsworks modules have been found to require a delay between write operations. */
		mdelay(50);

		/* Update base structure checksum */
		check = qsfp_check(&id->base, sizeof(id->base) - 1);
		err = qsfp_write(qsfp, false, QSFP_CC_BASE, &check, 1);
		if (err != 1) {
			dev_err(qsfp->dev,
				"Failed to update base structure checksum in fiber module EEPROM: %d\n",
				err);
			return err;
		}
	}
	return 0;
}

static int qsfp_sm_mod_probe(struct qsfp *qsfp, bool report)
{
	/* qsfp module inserted - read I2C data */
	struct qsfp_eeprom_id id;
	bool cotsworks_sfbg;
	bool cotsworks;

	int ret;

	/* Some qsfp modules and also some Linux I2C drivers do not like reads
	 * longer than 16 bytes, so read the EEPROM in chunks of 16 bytes at
	 * a time.
	 */

	qsfp->i2c_block_size = 16;

	qsfp_select_eeprom_page(qsfp);

	ret = qsfp_read(qsfp, false, 0x0, &id.base, sizeof(id.base));
	if (ret < 0) {
		if (report)
			dev_err(qsfp->dev, "failed to read EEPROM: %d\n", ret);
				return -EAGAIN;
	}

	if (ret != sizeof(id.base)) {
		dev_err(qsfp->dev, "EEPROM short read: %d\n", ret);

		return -EAGAIN;
	}

	/* Some qsfp modules (e.g. Nokia 3FE46541AA) lock up if read from
	 * address 0x51 is just one byte at a time. Also SFF-8472 requires
	 * that EEPROM supports atomic 16bit read operation for diagnostic
	 * fields, so do not switch to one byte reading at a time unless it
	 * is really required and we have no other option.
	 */
	if (qsfp_id_needs_byte_io(qsfp, &id.base, sizeof(id.base))) {
		dev_info(qsfp->dev,
			 "Detected broken RTL8672/RTL9601C emulated EEPROM\n");

		dev_info(qsfp->dev,
			 "Switching to reading EEPROM to one byte at a time\n");

		qsfp->i2c_block_size = 1;

		ret = qsfp_read(qsfp, false, 0, &id.base, sizeof(id.base));
		if (ret < 0) {
			if (report) {
				dev_err(qsfp->dev,
					"failed to read EEPROM: %d\n", ret);
			}

			return -EAGAIN;
		}

		if (ret != sizeof(id.base)) {
			dev_err(qsfp->dev, "EEPROM short read: %d\n", ret);

			return -EAGAIN;
		}
	}

	/* Cotsworks do not seem to update the checksums when they
	 * do the final programming with the final module part number,
	 * serial number and date code.
	 */
	cotsworks =
		!memcmp(id.base.etile_qsfp_vendor_name, "COTSWORKS       ", 16);
	cotsworks_sfbg = !memcmp(id.base.etile_qsfp_vendor_pn, "SFBG", 4);

	/* Cotsworks SFF module EEPROM do not always have valid etile_qsfp_identifier,
	 * etile_qsfp_ext_identifier, and connector bytes.  Rewrite SFF EEPROM bytes if
	 * Cotsworks PN matches and bytes are not correct.
	 */
	if (cotsworks && cotsworks_sfbg) {
		ret = qsfp_cotsworks_fixup_check(qsfp, &id);

		if (ret < 0)
			return ret;
	}

	qsfp->id = id;
	dev_info(qsfp->dev, "module %.*s %.*s rev %.*x sn %.*s dc %.*s\n",
		 (int)sizeof(id.base.etile_qsfp_vendor_name),
		 id.base.etile_qsfp_vendor_name,
		 (int)sizeof(id.base.etile_qsfp_vendor_pn),
		 id.base.etile_qsfp_vendor_pn,
		 (int)sizeof(id.base.etile_qsfp_revision),
		 id.base.etile_qsfp_revision,
		 (int)sizeof(id.base.etile_qsfp_vendor_serial_number),
		 id.base.etile_qsfp_vendor_serial_number,
		 (int)sizeof(id.base.etile_qsfp_vendor_date_code),
		 id.base.etile_qsfp_vendor_date_code);

	/* Check whether we support this module */
	if (!qsfp->type->module_supported(&id)) {
		dev_err(qsfp->dev,
			"module is not supported - phys id 0x%02x 0x%02x\n",
			qsfp->id.base.etile_qsfp_identifier_1,
			qsfp->id.base.etile_qsfp_ext_identifier);
			return -EINVAL;
	}

	/* Parse the module power requirement */
	ret = qsfp_module_parse_power(qsfp);
	if (ret < 0)
		return ret;

	qsfp->module_t_start_up = T_START_UP;

	return 0;
}

static void qsfp_timeout(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, timeout.work);
	int ret;
	u8 buf[16] = {0};

	rtnl_lock();
	ret = qsfp_read(qsfp, false, QSFP_RX_TX_LOSS, buf, 1);
	rtnl_unlock();
}

static irqreturn_t qsfp_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static void qsfp_poll(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, poll.work);

	get_module_revision(qsfp);

	if (qsfp->state_soft_mask &
		    (QSFP_OPTIONS_TX_LOSS_SIGNAL | QSFP_OPTIONS_TX_FAULT) ||
	    qsfp->need_poll)
		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);
}

static struct qsfp *qsfp_alloc(struct device *dev)
{
	struct qsfp *qsfp;

	qsfp = kzalloc(sizeof(*qsfp), GFP_KERNEL);
	if (!qsfp)
		return ERR_PTR(-ENOMEM);

	qsfp->dev = dev;

	mutex_init(&qsfp->sm_mutex);
	mutex_init(&qsfp->st_mutex);

	INIT_DELAYED_WORK(&qsfp->poll, qsfp_poll);

	INIT_DELAYED_WORK(&qsfp->timeout, qsfp_timeout);

	return qsfp;
}

static void qsfp_cleanup(void *data)
{
	struct qsfp *qsfp = data;

	cancel_delayed_work_sync(&qsfp->poll);
	cancel_delayed_work_sync(&qsfp->timeout);
	if (qsfp->i2c_mii) {
		mdiobus_unregister(qsfp->i2c_mii);
		mdiobus_free(qsfp->i2c_mii);
	}
	if (qsfp->i2c)
		i2c_put_adapter(qsfp->i2c);
	kfree(qsfp);
}

static int qsfp_probe(struct platform_device *pdev)
{
	const struct sff_data *sff;
	struct i2c_adapter *i2c;
	char *qsfp_irq_name;
	int err, i;

	qsfp = qsfp_alloc(&pdev->dev);

	if (IS_ERR(qsfp))
		return PTR_ERR(qsfp);

	platform_set_drvdata(pdev, qsfp);

	err = devm_add_action(qsfp->dev, qsfp_cleanup, qsfp);
	if (err < 0)
		return err;

	qsfp->type = &qsfp_data;
	sff = qsfp->type;

	if (pdev->dev.of_node) {
		struct device_node *node = pdev->dev.of_node;
		const struct of_device_id *id;
		struct device_node *np;

		id = of_match_node(qsfp_of_match, node);
		if (WARN_ON(!id))
			return -EINVAL;

		qsfp->type = id->data;
		sff = qsfp->type;

		np = of_parse_phandle(node, "i2c-bus", 0);

		if (!np) {
			dev_err(qsfp->dev, "missing 'i2c-bus' property\n");
			return -ENODEV;
		}

		i2c = of_find_i2c_adapter_by_node(np);
		of_node_put(np);
	} else if (has_acpi_companion(&pdev->dev)) {
		struct acpi_device *adev = ACPI_COMPANION(&pdev->dev);
		struct fwnode_handle *fw = acpi_fwnode_handle(adev);
		struct fwnode_reference_args args;
		struct acpi_handle *acpi_handle;
		int ret;

		ret = acpi_node_get_property_reference(fw, "i2c-bus", 0, &args);
		if (ret || !is_acpi_device_node(args.fwnode)) {
			dev_err(&pdev->dev, "missing 'i2c-bus' property\n");
			return -ENODEV;
		}

		acpi_handle = ACPI_HANDLE_FWNODE
		(args.fwnode);
		i2c = i2c_acpi_find_adapter_by_handle(acpi_handle);

	} else {
		return -EINVAL;
	}

	if (!i2c)
		return -EPROBE_DEFER;

	err = qsfp_i2c_configure(qsfp, i2c);
	if (err < 0) {
		i2c_put_adapter(i2c);

		return err;
	}
	for (i = 0; i < GPIO_MAX; i++)
		if (sff->gpios & BIT(i)) {
			qsfp->gpio[i] = devm_gpiod_get_optional
			(qsfp->dev, gpio_of_names[i], gpio_flags[i]);

			if (IS_ERR(qsfp->gpio[i]))
				return PTR_ERR(qsfp->gpio[i]);
		}

	qsfp->get_state = qsfp_gpio_get_state;
	qsfp->set_state = qsfp_gpio_set_state;

	/* Modules that have no detect signal are always present */
	if (!(qsfp->gpio[GPIO_MODULE_PRESENT]))
		qsfp->get_state = sff_gpio_get_state;

	device_property_read_u32(&pdev->dev, "maximum-power-milliwatt",
				 &qsfp->max_power_mw);
	if (!qsfp->max_power_mw)
		qsfp->max_power_mw = 1000;

	qsfp_set_state(qsfp, qsfp->state | QSFP_SELECT | QSFP_F_PRESENT);

	qsfp->state = qsfp_get_state(qsfp) | QSFP_OPTIONS_TX_DISABLE;

	if (qsfp->state & QSFP_F_PRESENT) {
		qsfp->state |= QSFP_SELECT;

		qsfp->state = qsfp_get_state(qsfp) | QSFP_OPTIONS_TX_DISABLE;

		qsfp_set_state(qsfp, qsfp->state);
		rtnl_lock();

		rtnl_unlock();
	} else {
		pr_info("qsfp is not present\n");
	}

	qsfp->gpio_irq[GPIO_MODULE_INTERRUPT] =
		gpiod_to_irq(qsfp->gpio[GPIO_MODULE_INTERRUPT]);
	if (qsfp->gpio_irq[GPIO_MODULE_INTERRUPT] < 0) {
		qsfp->gpio_irq[GPIO_MODULE_INTERRUPT] = 0;
		qsfp->need_poll = true;
	}

	qsfp_irq_name = devm_kasprintf(qsfp->dev, GFP_KERNEL, "%s-%s",
				       dev_name(qsfp->dev),
				       gpio_of_names[GPIO_MODULE_INTERRUPT]);

	if (!qsfp_irq_name)
		return -ENOMEM;

	err = devm_request_threaded_irq(qsfp->dev, qsfp->gpio_irq[GPIO_MODULE_INTERRUPT],
					NULL, qsfp_irq,
					IRQF_ONESHOT |
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					qsfp_irq_name, qsfp);

	if (err) {
		qsfp->gpio_irq[GPIO_MODULE_INTERRUPT] = 0;
		qsfp->need_poll = true;
	}

	if (qsfp->need_poll)
		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);

	/* We could have an issue in cases no Tx disable pin is available or
	 * wired as modules using a laser as their light source will continue to
	 * be active when the fiber is removed. This could be a safety issue and
	 * we should at least warn the user about that.
	 */
	if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_DISABLE)
		dev_warn(qsfp->dev, "No tx_disable pin: qsfp modules will always be emitting.\n");

	qsfp_sm_mod_probe(qsfp, 1);

	return 0;
}

static int qsfp_remove(struct platform_device *pdev)

{
	struct qsfp *qsfp = platform_get_drvdata(pdev);

	return 0;
}

static struct platform_driver qsfp_driver = {
	.probe = qsfp_probe,
	.remove = qsfp_remove,
	.driver = {
		.name = "sff,qsfp-multi-channel",
		.owner = THIS_MODULE,
		.of_match_table = qsfp_of_match,
	},
};

static int qsfp_init(void)
{
	poll_jiffies = msecs_to_jiffies(5000);

	return platform_driver_register(&qsfp_driver);
}
module_init(qsfp_init);

static void qsfp_exit(void)
{
	platform_driver_unregister(&qsfp_driver);
}
module_exit(qsfp_exit);

MODULE_ALIAS("platform:qsfp");
MODULE_AUTHOR("Malku, Deepak Nagaraju");
MODULE_LICENSE("GPL v2");
