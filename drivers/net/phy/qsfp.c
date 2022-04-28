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

static u8 previous_state = 1;
static struct qsfp *qsfp;
static void qsfp_sm_event(struct qsfp *qsfp, unsigned int event);
static void get_module_revision(struct qsfp *qsfp);

#define QSFP_TX_CHANNEL_4 0x4
#define QSFP_TX_CHANNEL_3 0x3
#define QSFP_TX_CHANNEL_2 0x2
#define QSFP_TX_CHANNEL_1 0x1
#define QSFP_RX_CHANNEL_4 0x4
#define QSFP_RX_CHANNEL_3 0x3
#define QSFP_RX_CHANNEL_2 0x2
#define QSFP_RX_CHANNEL_1 0x1

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
	[QSFP_MOD_EMPTY] = "empty",	[QSFP_MOD_ERROR] = "error",
	[QSFP_MOD_PROBE] = "probe",	[QSFP_MOD_WAITDEV] = "waitdev",
	[QSFP_MOD_HPOWER] = "hpower",	[QSFP_MOD_WAITPWR] = "waitpwr",
	[QSFP_MOD_PRESENT] = "present",
};

static const char *mod_state_to_str(unsigned short mod_state)
{
	if (mod_state >= ARRAY_SIZE(mod_state_strings))
		return "Unknown module state";
	return mod_state_strings[mod_state];
}

static const char *const dev_state_strings[] = {
	[QSFP_DEV_DETACHED] = "detached",
	[QSFP_DEV_DOWN] = "down",
	[QSFP_DEV_UP] = "up",
};

static const char *dev_state_to_str(unsigned short dev_state)
{
	if (dev_state >= ARRAY_SIZE(dev_state_strings))
		return "Unknown device state";
	return dev_state_strings[dev_state];
}

static const char *const event_strings[] = {
	[QSFP_E_INSERT] = "insert",	    [QSFP_E_REMOVE] = "remove",
	[QSFP_E_DEV_ATTACH] = "dev_attach", [QSFP_E_DEV_DETACH] = "dev_detach",
	[QSFP_E_DEV_DOWN] = "dev_down",	    [QSFP_E_DEV_UP] = "dev_up",
	[QSFP_E_TX_FAULT] = "tx_fault",	    [QSFP_E_TX_CLEAR] = "tx_clear",
	[QSFP_E_TX_LOS] = "tx_los",	    [QSFP_E_RX_LOS] = "rx_los",
	[QSFP_E_TIMEOUT] = "timeout",
};

static const char *event_to_str(unsigned short event)
{
	if (event >= ARRAY_SIZE(event_strings))
		return "Unknown event";
	return event_strings[event];
}

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

static const char *sm_state_to_str(unsigned short sm_state)
{
	if (sm_state >= ARRAY_SIZE(sm_state_strings))
		return "Unknown state";
	return sm_state_strings[sm_state];
}

static const char *const gpio_of_names[] = {
	"qsfpdd_modprsn", "qsfpdd_intn",    "qsfpdd_initmode",
	"qsfpdd_resetn",  "qsfpdd_modseln",
};

static const enum gpiod_flags gpio_flags[] = {
	GPIOD_IN, GPIOD_IN, GPIOD_ASIS, GPIOD_ASIS, GPIOD_ASIS,
};

/* t_start_up (SFF-8431) or t_init (SFF-8472) is the time required for a
 * non-cooled module to initialise its laser safety circuitry. We wait
 * an initial T_WAIT period before we check the tx fault to give any PHY
 * on board (for a copper qsfp) time to initialise.
 */
#define T_WAIT msecs_to_jiffies(50)
#define T_START_UP msecs_to_jiffies(300)
#define T_START_UP_BAD_GPON msecs_to_jiffies(60000)

/* t_reset is the time required to assert the TX_DISABLE signal to reset
 * an indicated TX_FAULT.
 */
#define T_RESET_US 10
#define T_FAULT_RECOVER msecs_to_jiffies(1000)

/* N_FAULT_INIT is the number of recovery attempts at module initialisation
 * time. If the TX_FAULT signal is not deasserted after this number of
 * attempts at clearing it, we decide that the module is faulty.
 * N_FAULT is the same but after the module has initialised.
 */
#define N_FAULT_INIT 5
#define N_FAULT 5

/* T_PHY_RETRY is the time interval between attempts to probe the PHY.
 * R_PHY_RETRY is the number of attempts.
 */
#define T_PHY_RETRY msecs_to_jiffies(50)
#define R_PHY_RETRY 12

/* qsfp module presence detection is poor: the three MOD DEF signals are
 * the same length on the PCB, which means it's possible for MOD DEF 0 to
 * connect before the I2C bus on MOD DEF 1/2.
 *
 * The SFF-8472 specifies t_serial ("Time from power on until module is
 * ready for data transmission over the two wire serial bus.") as 300ms.
 */
#define T_SERIAL msecs_to_jiffies(300)
#define T_HPOWER_LEVEL msecs_to_jiffies(300)
#define T_PROBE_RETRY_INIT msecs_to_jiffies(100)
#define R_PROBE_RETRY_INIT 10
#define T_PROBE_RETRY_SLOW msecs_to_jiffies(5000)
#define R_PROBE_RETRY_SLOW 12

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
	int channel_number;
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
};

static bool qsfp_module_supported(const struct qsfp_eeprom_id *id)
{
	if (id->base.etile_qsfp_identifier == SFF8024_ID_QSFP_28)
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
		.compatible = "sff,qsfp",
		.data = &qsfp_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, qsfp_of_match);

static unsigned long poll_jiffies;

static unsigned int qsfp_gpio_get_state(struct qsfp *qsfp)
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

static unsigned int sff_gpio_get_state(struct qsfp *qsfp)
{
	return qsfp_gpio_get_state(qsfp) | QSFP_F_PRESENT;
}

static void qsfp_gpio_set_state(struct qsfp *qsfp, unsigned int state)
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

void qsfp_reset(struct qsfp *qsfp, unsigned int value)
{
	unsigned int state = qsfp_gpio_get_state(qsfp);

	if (value)
		qsfp_gpio_set_state(qsfp, state | QSFP_RESET);
	else
		qsfp_gpio_set_state(qsfp, state & (~QSFP_RESET));
}
EXPORT_SYMBOL(qsfp_reset);

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

static void qsfp_soft_start_poll(struct qsfp *qsfp)
{
	u8 status;
	//const struct qsfp_eeprom_id *id = &qsfp->id;
	qsfp_read(qsfp, false, QSFP_OPTIONS, &status, sizeof(status));

	qsfp->state_soft_mask = 0;

	if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_DISABLE)
		qsfp->state_soft_mask |= QSFP_OPTIONS_TX_DISABLE;
	if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_FAULT)
		qsfp->state_soft_mask |= QSFP_OPTIONS_TX_FAULT;
	if (qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_LOSS_SIGNAL)
		qsfp->state_soft_mask |= QSFP_OPTIONS_TX_LOSS_SIGNAL;

	if (qsfp->state_soft_mask &
		    (QSFP_OPTIONS_TX_LOSS_SIGNAL | QSFP_OPTIONS_TX_FAULT) &&
	    !qsfp->need_poll)

		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);
}

static void qsfp_soft_stop_poll(struct qsfp *qsfp)
{
	qsfp->state_soft_mask = 0;
}

static unsigned int qsfp_get_state(struct qsfp *qsfp)
{
	unsigned int state = qsfp->get_state(qsfp);

	return state;
}

static void qsfp_set_state(struct qsfp *qsfp, unsigned int state)
{
	qsfp->set_state(qsfp, state);
}

static unsigned int qsfp_check(void *buf, size_t len)
{
	u8 *p, check;

	for (p = buf, check = 0; len; p++, len--)
		check += *p;

	return check;
}

/* Helpers */
static void qsfp_module_tx_disable(struct qsfp *qsfp)
{
	dev_dbg(qsfp->dev, "tx disable %u -> %u\n",
		qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_DISABLE ?
			1 :
			      0,
		1);
	qsfp->id.base.etile_qsfp_options_3 |= QSFP_OPTIONS_TX_DISABLE;
	qsfp_set_state(qsfp, qsfp->state);
}

static void qsfp_module_tx_enable(struct qsfp *qsfp)
{
	dev_dbg(qsfp->dev, "tx disable %u -> %u\n",
		qsfp->id.base.etile_qsfp_options_3 & QSFP_OPTIONS_TX_DISABLE ?
			1 :
			      0,
		0);
	qsfp->id.base.etile_qsfp_options_3 &= ~QSFP_OPTIONS_TX_DISABLE;
	qsfp_set_state(qsfp, qsfp->state);
}

static void qsfp_module_tx_fault_reset(struct qsfp *qsfp)
{
	unsigned int state = qsfp->id.base.etile_qsfp_options_3;

	int ret;

	ret = qsfp_read(qsfp, false, QSFP_OPTIONS, &state, sizeof(state));

	if (state & QSFP_OPTIONS_TX_DISABLE)
		return;

	qsfp_set_state(qsfp, state | QSFP_OPTIONS_TX_DISABLE);

	udelay(T_RESET_US);

	qsfp_set_state(qsfp, state);
}

/* qsfp state machine */
static void qsfp_sm_set_timer(struct qsfp *qsfp, unsigned int timeout)
{
	if (timeout) {
		mod_delayed_work(system_power_efficient_wq, &qsfp->timeout,
				 timeout);
	} else {
		cancel_delayed_work(&qsfp->timeout);
	}
}

static void qsfp_sm_next(struct qsfp *qsfp, unsigned int state,
			 unsigned int timeout)
{
	qsfp->sm_state = state;
	qsfp_sm_set_timer(qsfp, timeout);
}

static void qsfp_sm_mod_next(struct qsfp *qsfp, unsigned int state,
			     unsigned int timeout)
{
	qsfp->sm_mod_state = state;
	qsfp_sm_set_timer(qsfp, timeout);
}

static void qsfp_sm_phy_detach(struct qsfp *qsfp)
{
	qsfp_remove_phy(qsfp->qsfp_bus);
	phy_device_remove(qsfp->mod_phy);
	phy_device_free(qsfp->mod_phy);
	qsfp->mod_phy = NULL;
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

static void qsfp_sm_link_up(struct qsfp *qsfp)
{
	qsfp_link_up(qsfp->qsfp_bus);
	qsfp_sm_next(qsfp, QSFP_S_LINK_UP, 0);
}

static void qsfp_sm_link_down(struct qsfp *qsfp)
{
	qsfp_link_down(qsfp->qsfp_bus);
}

static void qsfp_sm_link_check_los(struct qsfp *qsfp)
{
	bool los = false;

	if (los)
		qsfp_sm_next(qsfp, QSFP_S_WAIT_LOS, 0);
	else

		qsfp_sm_link_up(qsfp);
}

static bool qsfp_los_event_active(struct qsfp *qsfp, unsigned int event)
{
	int ret;
	u8 buf[16];

	ret = qsfp_read(qsfp, false, QSFP_PHYS_ID, buf, 1);

	return 0;
}

static bool qsfp_los_event_inactive(struct qsfp *qsfp, unsigned int event)
{
	return 0;
}

static void qsfp_sm_fault(struct qsfp *qsfp, unsigned int next_state, bool warn)
{
	if (qsfp->sm_fault_retries && !--qsfp->sm_fault_retries) {
		dev_err(qsfp->dev,
			"module persistently indicates fault, disabling\n");
		qsfp_sm_next(qsfp, QSFP_S_TX_DISABLE, 0);
	} else {
		if (warn)
			dev_err(qsfp->dev, "module transmit fault indicated\n");

		qsfp_sm_next(qsfp, next_state, T_FAULT_RECOVER);
	}
}

/* Probe a qsfp for a PHY device if the module supports copper - the PHY
 * normally sits at I2C bus address 0x56, and may either be a clause 22
 * or clause 45 PHY.
 *
 * Clause 22 copper qsfp modules normally operate in Cisco SGMII mode with
 * negotiation enabled, but some may be in 1000base-X - which is for the
 * PHY driver to determine.
 *
 * Clause 45 copper qsfp+ modules (10G) appear to switch their interface
 * mode according to the negotiated line speed.
 */
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

		err = qsfp_sm_probe_phy(qsfp, false);
		break;
	}
	return err;
}

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

static int qsfp_sm_mod_hpower(struct qsfp *qsfp, bool enable)
{
	u8 val;
	int err;

	static void *gpio_reg;
	u32 gpio_reg_val;

	gpio_reg = ioremap(0x82000020, 4);
	gpio_reg_val = readl(gpio_reg);
	gpio_reg_val = gpio_reg_val & 0xfffffffe;
	writel(gpio_reg_val, gpio_reg);

	qsfp_set_state(qsfp, qsfp->state & QSFP_INIT);

	err = qsfp_read(qsfp, false, QSFP_EXT_STATUS, &val, sizeof(val));
	if (err != sizeof(val)) {
		dev_err(qsfp->dev, "Failed to read EEPROM: %d\n", err);
		return -EAGAIN;
	}

	/* DM7052 reports as a high power module, responds to reads (with
	 * all bytes 0xff) at 0x51 but does not accept writes.  In any case,
	 * if the bit is already set, we're already in high power mode.
	 */
	if (!!(val & BIT(0)) == enable)
		return 0;

	if (enable)
		val |= BIT(0);
	else
		val &= ~BIT(0);

	err = qsfp_write(qsfp, false, QSFP_EXT_STATUS, &val, sizeof(val));
	if (err != sizeof(val)) {
		dev_err(qsfp->dev, "Failed to write EEPROM: %d\n", err);
		return -EAGAIN;
	}

	if (enable)
		dev_info(qsfp->dev, "Module switched to %u.%uW power level\n",
			 qsfp->module_power_mw / 1000,
			 (qsfp->module_power_mw / 100) % 10);

	return 0;
}

/* GPON modules based on Realtek RTL8672 and RTL9601C chips (e.g. V-SOL
 * V2801F, CarlitoxxPro CPGOS03-0490, Ubiquiti U-Fiber Instant, ...) do
 * not support multibyte reads from the EEPROM. Each multi-byte read
 * operation returns just one byte of EEPROM followed by zeros. There is
 * no way to identify which modules are using Realtek RTL8672 and RTL9601C
 * chips. Moreover every OEM of V-SOL V2801F module puts its own vendor
 * name and vendor id into EEPROM, so there is even no way to detect if
 * module is V-SOL V2801F. Therefore check for those zeros in the read
 * data and then based on check switch to reading EEPROM to one byte
 * at a time.
 */
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

static void get_module_revision(struct qsfp *old)
{
	int ret;
	u8 buf[16];
	char buf_16[16];

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
}

int get_cable_attach(struct qsfp *old)
{
	return qsfp->module_present;
}
EXPORT_SYMBOL(get_cable_attach);

int get_channel_info(struct qsfp *old)
{
	return qsfp->channel_number;
}
EXPORT_SYMBOL(get_channel_info);

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

static int qsfp_status_indicators(struct qsfp *qsfp)
{
	int ret;
	u8 buf[16];
	bool flag = false;
	static unsigned int prv_buf, prv_buf_rx;

	qsfp->state = qsfp_gpio_get_state(qsfp);
	if (qsfp->state & QSFP_F_PRESENT) {
		ret = qsfp_read(qsfp, false, QSFP_STATUS, buf, 1);
		qsfp->module_revision = buf[0];
		if (qsfp->module_revision == 0x0) {
			qsfp->module_present = TRUE;
			qsfp->channel_number = -EOPNOTSUPP;
		}

		if (qsfp->module_revision == 0x7 || qsfp->module_revision == 0x8) {
			ret = qsfp_read(qsfp, false, QSFP_OPTIONS, buf, 1);

			if (buf[0] & QSFP_OPTIONS_TX_LOSS_SIGNAL) {
				ret = qsfp_read(qsfp, false, QSFP_RX_TX_LOSS, buf, 1);

			if (prv_buf != buf[0]) {
				prv_buf = buf[0];
				flag = true;
			}
			if (flag) {
				qsfp->module_present = TRUE;
				qsfp->channel_number = buf[0];
			} else {
				qsfp->module_present = TRUE;
				qsfp->channel_number = buf[0];
			}
			} else {
				ret = qsfp_read(qsfp, false, QSFP_RX_TX_LOSS, buf, 1);
				buf[0] = buf[0] & 0xf;
			if (prv_buf_rx != buf[0]) {
				prv_buf_rx = buf[0];
				flag = true;
				}
			if (flag) {
				qsfp->module_present = TRUE;
				qsfp->channel_number = buf[0];
			} else {
				qsfp->module_present = TRUE;
				qsfp->channel_number = buf[0];
			}
		}
	}

	} else {
		qsfp->module_present = -EINVAL;
		qsfp->channel_number = -EOPNOTSUPP;
		}

	return 0;
}

static int qsfp_state_indicators(struct qsfp *qsfp)
{
	int ret;
	u8 buf[16];
	bool flag = false;

	qsfp->state = qsfp_gpio_get_state(qsfp);
	qsfp->state &= QSFP_F_PRESENT;
	if (previous_state != qsfp->state) {
		previous_state = (qsfp->state);
		flag = true;
	}

	if (flag) {
		if (qsfp->state & QSFP_F_PRESENT) {
			ret = qsfp_read(qsfp, false, QSFP_PHYS_ID, buf, 1);

			get_module_revision(qsfp);
			pr_info("module: %s pn: %s sn: %s rev: %x\n", qsfp->vender_name,
				qsfp->part_number, qsfp->sn_number, qsfp->module_revision);

		} else {
			qsfp->module_present = -EINVAL;
			qsfp->channel_number = -EOPNOTSUPP;
		}
	}

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

	if (!memcmp(id.base.etile_qsfp_vendor_name, "YAMAICHI   ", 16) &&
	    !memcmp(id.base.etile_qsfp_vendor_pn, "3FE46541AA      ", 16))
		qsfp->module_t_start_up = T_START_UP_BAD_GPON;
	else
		qsfp->module_t_start_up = T_START_UP;

	return 0;
}

static void qsfp_sm_mod_remove(struct qsfp *qsfp)
{
	if (qsfp->sm_mod_state > QSFP_MOD_WAITDEV)
		qsfp_module_remove(qsfp->qsfp_bus);

	memset(&qsfp->id, 0, sizeof(qsfp->id));
	qsfp->module_power_mw = 0;

	dev_info(qsfp->dev, "module removed\n");
}

/* This state machine tracks the upstream's state */
static void qsfp_sm_device(struct qsfp *qsfp, unsigned int event)
{
	switch (qsfp->sm_dev_state) {
	default:
		if (event == QSFP_E_DEV_ATTACH)
			qsfp->sm_dev_state = QSFP_DEV_DOWN;
		break;
	case QSFP_DEV_DOWN:

		if (event == QSFP_E_DEV_DETACH)
			qsfp->sm_dev_state = QSFP_DEV_DETACHED;

		else if (event == QSFP_E_DEV_UP)
			qsfp->sm_dev_state = QSFP_DEV_UP;
		break;

	case QSFP_DEV_UP:

		if (event == QSFP_E_DEV_DETACH)
			qsfp->sm_dev_state = QSFP_DEV_DETACHED;
		else if (event == QSFP_E_DEV_DOWN)
			qsfp->sm_dev_state = QSFP_DEV_DOWN;
		break;
	}
}

/* This state machine tracks the insert/remove state of the module, probes
 * the on-board EEPROM, and sets up the power level.
 */
static void qsfp_sm_module(struct qsfp *qsfp, unsigned int event)
{
	int err;

	/* Handle remove event globally, it resets this state machine */
	if (event == QSFP_E_REMOVE) {
		if (qsfp->sm_mod_state > QSFP_MOD_PROBE)
			qsfp_sm_mod_remove(qsfp);
		qsfp_sm_mod_next(qsfp, QSFP_MOD_EMPTY, 0);
		return;
	}

	/* Handle device detach globally */
	if (qsfp->sm_dev_state < QSFP_DEV_DOWN &&
	    qsfp->sm_mod_state > QSFP_MOD_WAITDEV) {
		if (qsfp->module_power_mw > 1000 &&
		    qsfp->sm_mod_state > QSFP_MOD_HPOWER)
			qsfp_sm_mod_hpower(qsfp, false);
		qsfp_sm_mod_next(qsfp, QSFP_MOD_WAITDEV, 0);
		return;
	}

	switch (qsfp->sm_mod_state) {
	default:
		if (event == QSFP_E_INSERT) {
			qsfp_sm_mod_next(qsfp, QSFP_MOD_PROBE, T_SERIAL);
			qsfp->sm_mod_tries_init = R_PROBE_RETRY_INIT;
			qsfp->sm_mod_tries = R_PROBE_RETRY_SLOW;
		}
		break;

	case QSFP_MOD_PROBE:
		/* Wait for T_PROBE_INIT to time out */
		if (event != QSFP_E_TIMEOUT)
			break;

		err = qsfp_sm_mod_probe(qsfp, qsfp->sm_mod_tries == 1);
		if (err == -EAGAIN) {
			if (qsfp->sm_mod_tries_init &&
			    --qsfp->sm_mod_tries_init) {
				qsfp_sm_set_timer(qsfp, T_PROBE_RETRY_INIT);
				break;
			} else if (qsfp->sm_mod_tries && --qsfp->sm_mod_tries) {
				if (qsfp->sm_mod_tries == R_PROBE_RETRY_SLOW - 1)
					dev_warn(qsfp->dev, "please wait, module slow to respond\n");
				qsfp_sm_set_timer(qsfp, T_PROBE_RETRY_SLOW);
				break;
			}
		}
		if (err < 0) {
			qsfp_sm_mod_next(qsfp, QSFP_MOD_ERROR, 0);
			break;
		}

		fallthrough;
	case QSFP_MOD_WAITDEV:

		/* Ensure that the device is attached before proceeding */
		if (qsfp->sm_dev_state < QSFP_DEV_DOWN)
			break;

		/* Report the module insertion to the upstream device */
		qsfp->id.base.etile_qsfp_identifier = 0x11; //
		err = qsfp_module_insert(qsfp->qsfp_bus, &qsfp->id);

		if (err < 0) {
			qsfp_sm_mod_next(qsfp, QSFP_MOD_ERROR, 0);
			break;
		}

		/* If this is a power level 1 module, we are done */
		if (qsfp->module_power_mw <= 1000)
			goto insert;

		qsfp_sm_mod_next(qsfp, QSFP_MOD_HPOWER, 0);
		fallthrough;
	case QSFP_MOD_HPOWER:
		/* Enable high power mode */
		err = qsfp_sm_mod_hpower(qsfp, true);
		if (err < 0) {
			if (err != -EAGAIN) {
				qsfp_module_remove(qsfp->qsfp_bus);
				qsfp_sm_mod_next(qsfp, QSFP_MOD_ERROR, 0);
			} else {
				qsfp_sm_set_timer(qsfp, T_PROBE_RETRY_INIT);
			}
			break;
		}

		qsfp_sm_mod_next(qsfp, QSFP_MOD_WAITPWR, T_HPOWER_LEVEL);
		break;

	case QSFP_MOD_WAITPWR:

		/* Wait for T_HPOWER_LEVEL to time out */
		if (event != QSFP_E_TIMEOUT)
			break;

insert:

		qsfp_sm_mod_next(qsfp, QSFP_MOD_PRESENT, 0);
		break;

	case QSFP_MOD_PRESENT:
	case QSFP_MOD_ERROR:
		break;
	}
}

static void qsfp_sm_main(struct qsfp *qsfp, unsigned int event)
{
	unsigned long timeout;
	int ret;

	/* Some events are global */
	if (qsfp->sm_state != QSFP_S_DOWN &&
	    (qsfp->sm_mod_state != QSFP_MOD_PRESENT ||
	     qsfp->sm_dev_state != QSFP_DEV_UP)) {
		if (qsfp->sm_state == QSFP_S_LINK_UP &&
		    qsfp->sm_dev_state == QSFP_DEV_UP)
			qsfp_sm_link_down(qsfp);
		if (qsfp->sm_state > QSFP_S_INIT)
			qsfp_module_stop(qsfp->qsfp_bus);
		if (qsfp->mod_phy)
			qsfp_sm_phy_detach(qsfp);
		qsfp_module_tx_disable(qsfp);
		qsfp_soft_stop_poll(qsfp);
		qsfp_sm_next(qsfp, QSFP_S_DOWN, 0);
		return;
	}

	/* The main state machine */
	switch (qsfp->sm_state) {
	case QSFP_S_DOWN:

		if (qsfp->sm_mod_state != QSFP_MOD_PRESENT ||
		    qsfp->sm_dev_state != QSFP_DEV_UP)
			break;

		if (!(qsfp->id.base.etile_qsfp_diag_monitor &
		      QSFP_DIAGMON_ADDRMODE))
			qsfp_soft_start_poll(qsfp);

		qsfp_module_tx_enable(qsfp);

		/* Initialise the fault clearance retries */
		qsfp->sm_fault_retries = N_FAULT_INIT;

		/* We need to check the TX_FAULT state, which is not defined
		 * while TX_DISABLE is asserted. The earliest we want to do
		 * anything (such as probe for a PHY) is 50ms.
		 */
		qsfp_sm_next(qsfp, QSFP_S_WAIT, T_WAIT);
		break;

	case QSFP_S_WAIT:

		if (event != QSFP_E_TIMEOUT)

			break;

		if (qsfp->id.base.etile_qsfp_options_3 &
		    QSFP_OPTIONS_TX_FAULT) {
			/* Wait up to t_init (SFF-8472) or t_start_up (SFF-8431)
			 * from the TX_DISABLE deassertion for the module to
			 * initialise, which is indicated by TX_FAULT
			 * deasserting.
			 */
			timeout = qsfp->module_t_start_up;
			if (timeout > T_WAIT)
				timeout -= T_WAIT;
			else
				timeout = 1;

			qsfp_sm_next(qsfp, QSFP_S_INIT, timeout);
		} else {
			/* TX_FAULT is not asserted, assume the module has
			 * finished initialising.
			 */
			goto init_done;
		}
		break;

	case QSFP_S_INIT:

		if (event == QSFP_E_TIMEOUT &&
		    qsfp->id.base.etile_qsfp_options_3 &
			    QSFP_OPTIONS_TX_FAULT) {
			/* TX_FAULT is still asserted after t_init
			 * or t_start_up, so assume there is a fault.
			 */
			qsfp_sm_fault(qsfp, QSFP_S_INIT_TX_FAULT,
				      qsfp->sm_fault_retries == N_FAULT_INIT);
		} else if (event == QSFP_E_TIMEOUT ||
			   event == QSFP_E_TX_CLEAR) {
init_done:

			qsfp->sm_phy_retries = R_PHY_RETRY;
			goto phy_probe;
		}
		break;

	case QSFP_S_INIT_PHY:

		if (event != QSFP_E_TIMEOUT)
			break;
phy_probe:
		/* TX_FAULT deasserted or we timed out with TX_FAULT
		 * clear.  Probe for the PHY and check the LOS state.
		 */
		ret = qsfp_sm_probe_for_phy(qsfp);
		if (ret == -ENODEV) {
			if (--qsfp->sm_phy_retries) {
				qsfp_sm_next(qsfp, QSFP_S_INIT_PHY,
					     T_PHY_RETRY);
					dev_info(qsfp->dev, "PHY detected\n");
				break;
			}
		} else if (ret) {
			qsfp_sm_next(qsfp, QSFP_S_FAIL, 0);
			break;
		}
		if (qsfp_module_start(qsfp->qsfp_bus)) {
			qsfp_sm_next(qsfp, QSFP_S_FAIL, 0);
			break;
		}
		qsfp_sm_link_check_los(qsfp);

		/* Reset the fault retry count */
		qsfp->sm_fault_retries = N_FAULT;
		break;

	case QSFP_S_INIT_TX_FAULT:

		if (event == QSFP_E_TIMEOUT) {
			qsfp_module_tx_fault_reset(qsfp);
			qsfp_sm_next(qsfp, QSFP_S_INIT,
				     qsfp->module_t_start_up);
		}

		break;

	case QSFP_S_WAIT_LOS:

		if (event == QSFP_E_TX_FAULT)
			qsfp_sm_fault(qsfp, QSFP_S_TX_FAULT, true);
		else if (qsfp_los_event_inactive(qsfp, event))
			qsfp_sm_link_up(qsfp);

		break;

	case QSFP_S_LINK_UP:

		if (event == QSFP_E_TX_FAULT) {
			qsfp_sm_link_down(qsfp);
			qsfp_sm_fault(qsfp, QSFP_S_TX_FAULT, true);
		} else if (qsfp_los_event_active(qsfp, event)) {
			qsfp_sm_link_down(qsfp);
			qsfp_sm_next(qsfp, QSFP_S_WAIT_LOS, 0);
		}

		break;

	case QSFP_S_TX_FAULT:

		if (event == QSFP_E_TIMEOUT) {
			qsfp_module_tx_fault_reset(qsfp);
			qsfp_sm_next(qsfp, QSFP_S_REINIT,
				     qsfp->module_t_start_up);
		}

		break;

	case QSFP_S_REINIT:

		if (event == QSFP_E_TIMEOUT &&
		    qsfp->id.base.etile_qsfp_options_3 &
			    QSFP_OPTIONS_TX_FAULT) {
			qsfp_sm_fault(qsfp, QSFP_S_TX_FAULT, false);
		} else if (event == QSFP_E_TIMEOUT ||
			   event == QSFP_E_TX_CLEAR) {
			dev_info(qsfp->dev,
				 "module transmit fault recovered\n");
			qsfp_sm_link_check_los(qsfp);
		};

		break;

	case QSFP_S_TX_DISABLE:

		break;
	}
}

static void qsfp_sm_event(struct qsfp *qsfp, unsigned int event)
{
	mutex_lock(&qsfp->sm_mutex);

	dev_dbg(qsfp->dev, "SM: enter %s:%s:%s event %s\n",
		mod_state_to_str(qsfp->sm_mod_state),
		dev_state_to_str(qsfp->sm_dev_state),
		sm_state_to_str(qsfp->sm_state), event_to_str(event));

	qsfp_sm_device(qsfp, event);
	qsfp_sm_module(qsfp, event);
	qsfp_sm_main(qsfp, event);

	dev_dbg(qsfp->dev, "SM: exit %s:%s:%s\n",
		mod_state_to_str(qsfp->sm_mod_state),
		dev_state_to_str(qsfp->sm_dev_state),
		sm_state_to_str(qsfp->sm_state));

	mutex_unlock(&qsfp->sm_mutex);
}

void qsfp_attach(struct qsfp *old)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_ATTACH);
}
EXPORT_SYMBOL(qsfp_attach);

void qsfp_detach(struct qsfp *old)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_DETACH);
}
EXPORT_SYMBOL(qsfp_detach);

void qsfp_start(struct qsfp *old)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_UP);
}
EXPORT_SYMBOL(qsfp_start);

void qsfp_stop(struct qsfp *old)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_DOWN);
}
EXPORT_SYMBOL(qsfp_stop);

static int qsfp_module_info(struct qsfp *qsfp, struct ethtool_modinfo *modinfo)
{
	/* locking... and check module is present */

	if (qsfp->id.base.etile_qsfp_spec_compliance_1[0] &&
	    !(qsfp->id.base.etile_qsfp_diag_monitor & QSFP_DIAGMON_ADDRMODE)) {
		modinfo->type = ETH_MODULE_SFF_8472;
		modinfo->eeprom_len = ETH_MODULE_SFF_8472_LEN;
	} else {
		modinfo->type = ETH_MODULE_SFF_8079;
		modinfo->eeprom_len = ETH_MODULE_SFF_8079_LEN;
	}
	return 0;
}

static int qsfp_module_eeprom(struct qsfp *qsfp, struct ethtool_eeprom *ee,
			      u8 *data)
{
	unsigned int first, last, len;
	int ret;

	if (ee->len == 0)
		return -EINVAL;

	first = ee->offset;
	last = ee->offset + ee->len;
	if (first < ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8079_LEN);
		len -= first;

		ret = qsfp_read(qsfp, false, first, data, len);
		if (ret < 0)
			return ret;

		first += len;
		data += len;
	}
	if (first < ETH_MODULE_SFF_8472_LEN && last > ETH_MODULE_SFF_8079_LEN) {
		len = min_t(unsigned int, last, ETH_MODULE_SFF_8472_LEN);
		len -= first;
		first -= ETH_MODULE_SFF_8079_LEN;

		ret = qsfp_read(qsfp, false, first, data, len);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static const struct qsfp_socket_ops qsfp_module_ops = {
	.attach = qsfp_attach,
	.detach = qsfp_detach,
	.start = qsfp_start,
	.stop = qsfp_stop,
	.module_info = qsfp_module_info,
	.module_eeprom = qsfp_module_eeprom,
};

static void qsfp_timeout(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, timeout.work);

	rtnl_lock();
	qsfp_sm_event(qsfp, QSFP_E_TIMEOUT);
	rtnl_unlock();
}

static void qsfp_check_state(struct qsfp *qsfp)
{
	unsigned int state, i, changed;

	mutex_lock(&qsfp->st_mutex);
	state = qsfp_get_state(qsfp);

	changed = state ^ qsfp->state;
	changed &= QSFP_F_PRESENT | QSFP_OPTIONS_TX_LOSS_SIGNAL |
		   QSFP_OPTIONS_TX_FAULT;

	for (i = 0; i < GPIO_MAX; i++)
		if (changed & BIT(i))
			dev_dbg(qsfp->dev, "%s %u -> %u\n", gpio_of_names[i],
				!!(qsfp->state & BIT(i)), !!(state & BIT(i)));

	qsfp->state = state;

	rtnl_lock();
	if (changed & QSFP_F_PRESENT)
		qsfp_sm_event(qsfp, state & QSFP_F_PRESENT ? QSFP_E_INSERT :
								   QSFP_E_REMOVE);

	if (changed & QSFP_OPTIONS_TX_FAULT)
		qsfp_sm_event(qsfp, state & QSFP_OPTIONS_TX_FAULT ?
					    QSFP_E_TX_FAULT :
						  QSFP_E_TX_CLEAR);

	if (changed & QSFP_OPTIONS_TX_LOSS_SIGNAL)
		qsfp_sm_event(qsfp, state & QSFP_OPTIONS_TX_LOSS_SIGNAL ?
					    QSFP_E_TX_LOS :
						  QSFP_E_RX_LOS);

	rtnl_unlock();
	mutex_unlock(&qsfp->st_mutex);
}

static irqreturn_t qsfp_irq(int irq, void *data)
{
	struct qsfp *qsfp = data;

	qsfp_check_state(qsfp);

	return IRQ_HANDLED;
}

static void qsfp_poll(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, poll.work);

	qsfp_check_state(qsfp);

	qsfp_state_indicators(qsfp);
	qsfp_status_indicators(qsfp);

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
	static void *reg_addr;
	u32 reg_value;

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

		reg_addr = ioremap(0xffd11000, 0x40);
		reg_addr += 0x28;
		reg_value = readl(reg_addr);
		//bring out from reset by writing to bit 8//
		reg_value &= 0xfffffeff;
		writel(reg_value, reg_addr);

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
	qsfp_reset(qsfp, 0);

	for (i = 0; i < GPIO_MAX; i++)
		if (sff->gpios & BIT(i)) {
			qsfp->gpio[i] = devm_gpiod_get_optional
			(qsfp->dev, gpio_of_names[i], gpio_flags[i]);

			if (IS_ERR(qsfp->gpio[i]))
				return PTR_ERR(qsfp->gpio[i]);
		}

	qsfp->get_state = qsfp_gpio_get_state;
	qsfp->set_state = qsfp_gpio_set_state;

	qsfp_reset(qsfp, 1);

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
		qsfp_sm_event(qsfp, QSFP_E_INSERT);

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

	qsfp->qsfp_bus =
		qsfp_register_socket(qsfp->dev, qsfp, &qsfp_module_ops);
	if (!qsfp->qsfp_bus)
		return -ENOMEM;

	get_module_revision(qsfp);

	return 0;
}

static int qsfp_remove(struct platform_device *pdev)

{
	struct qsfp *qsfp = platform_get_drvdata(pdev);

	qsfp_unregister_socket(qsfp->qsfp_bus);

	rtnl_lock();
	qsfp_sm_event(qsfp, QSFP_E_REMOVE);
	rtnl_unlock();

	return 0;
}

static struct platform_driver qsfp_driver = {
	.probe = qsfp_probe,
	.remove = qsfp_remove,
	.driver = {
		.name = "sff,qsfp",
		.owner = THIS_MODULE,
		.of_match_table = qsfp_of_match,
	},
};

static int qsfp_init(void)
{
	poll_jiffies = msecs_to_jiffies(1000);

	return platform_driver_register(&qsfp_driver);
}
module_init(qsfp_init);

static void qsfp_exit(void)
{
	platform_driver_unregister(&qsfp_driver);
}
module_exit(qsfp_exit);

MODULE_ALIAS("platform:qsfp");
MODULE_AUTHOR("Malku Deepak");
MODULE_LICENSE("GPL v2");
