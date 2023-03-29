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

struct qsfp *qsfp_temp;

static void qsfp_sm_event(struct qsfp *qsfp, unsigned int event);

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

static const struct of_device_id qsfp_of_match[] = {
	{
		.compatible = "sff,qsfp",

	},
	{},
};
MODULE_DEVICE_TABLE(of, qsfp_of_match);

static unsigned long poll_jiffies;

static void qsfp_soft_start_poll(struct qsfp *qsfp)
{
	u8 status;

	status = soft_mask(qsfp_temp);

	if (status &
		    (QSFP_OPTIONS_TX_LOSS_SIGNAL | QSFP_OPTIONS_TX_FAULT) &&
	    !qsfp->need_poll)

		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);
}

static void qsfp_soft_stop_poll(struct qsfp *qsfp)
{
	qsfp->state_soft_mask = 0;
}

static void qsfp_module_tx_fault_reset(struct qsfp *qsfp)
{
	u8 buf[16] = {0};

	buf[0] = get_qsfp_options(qsfp_temp);
	if (buf[0] & QSFP_OPTIONS_TX_DISABLE)
		return;
}

/* qsfp state machine */
static void qsfp_sm_set_timer(struct qsfp *qsfp, unsigned int timeout)
{
	if (timeout)
		mod_delayed_work(system_power_efficient_wq, &qsfp->timeout,
				 timeout);
	else
		cancel_delayed_work(&qsfp->timeout);
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
	u8 buf[16] = {0};

	 buf[0] = get_tx_rx_loss(qsfp_temp);

	if ((buf[0] & 0xf0) || (buf[0] & 0x0f))
		return 1;

	return 0;
}

static bool qsfp_los_event_inactive(struct qsfp *qsfp, unsigned int event)
{
	u8 buf[16] = {0};

	buf[0] = get_tx_rx_loss(qsfp_temp);

	if (!(buf[0] & (0xff)))
		return 1;

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
	int err;

	qsfp->mod_phy = gsfp_mod_phy(qsfp_temp);

	err = gsfp_probe_phy(qsfp_temp);

	return err = gsfp_probe_phy(qsfp_temp);
}

static int qsfp_sm_mod_probe(struct qsfp *qsfp, bool report)
{
	qsfp->module_t_start_up = T_START_UP;

	return 0;
}

static void qsfp_sm_mod_remove(struct qsfp *qsfp)
{
	if (qsfp->sm_mod_state > QSFP_MOD_WAITDEV)
		qsfp_module_remove(qsfp->qsfp_bus);

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

	/* modules have been found to require a delay between read and write operations.
	 *Hence adding delay which will keep the i2C bus idle for few seconds
	 */

	mdelay(50);

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
		    qsfp->sm_mod_state > QSFP_MOD_HPOWER){
			//qsfp_sm_mod_hpower(qsfp, false);
			}
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

		qsfp_sm_mod_next(qsfp, QSFP_MOD_WAITDEV, 0);

		fallthrough;
	case QSFP_MOD_WAITDEV:
		/* Ensure that the device is attached before proceeding */
		if (qsfp->sm_dev_state < QSFP_DEV_DOWN)
			break;

		/* Report the module insertion to the upstream device */
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
		    qsfp->sm_dev_state == QSFP_DEV_UP){
			qsfp_sm_link_down(qsfp);
			}
		if (qsfp->sm_state > QSFP_S_INIT)
			qsfp_module_stop(qsfp->qsfp_bus);
		if (qsfp->mod_phy)
			qsfp_sm_phy_detach(qsfp);
		qsfp_soft_stop_poll(qsfp);
		qsfp_sm_next(qsfp, QSFP_S_DOWN, 0);
		return;
	}

	/* The main state machine */
	switch (qsfp->sm_state) {
	case QSFP_S_DOWN:
		if (qsfp->sm_mod_state != QSFP_MOD_PRESENT ||
		    qsfp->sm_dev_state != QSFP_DEV_UP){
			break;
			}

			qsfp_soft_start_poll(qsfp);

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
		if (get_qsfp_options(qsfp_temp)) {
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
		if (event == QSFP_E_TIMEOUT && get_qsfp_options(qsfp_temp)) {
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
				break;
			} else {
				/*earlier ported the driver wrongly from SFP*/
				dev_info(qsfp->dev, "no PHY detected\n");
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

		if (event == QSFP_E_TIMEOUT && get_qsfp_options(qsfp_temp)) {
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

void qsfp_attach(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_ATTACH);
}
EXPORT_SYMBOL(qsfp_attach);

void qsfp_detach(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_DETACH);
}
EXPORT_SYMBOL(qsfp_detach);

void qsfp_start(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_UP);
}
EXPORT_SYMBOL(qsfp_start);

void qsfp_stop(struct qsfp *qsfp)
{
	qsfp_sm_event(qsfp, QSFP_E_DEV_DOWN);
}
EXPORT_SYMBOL(qsfp_stop);

static const struct qsfp_socket_ops qsfp_module_ops = {
	.attach = qsfp_attach,
	.detach = qsfp_detach,
	.start = qsfp_start,
	.stop = qsfp_stop,

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
	int qsfp_module_info_testing;
	int qsfp_rx_tx_channel_info;
	char qsfp_vendor_info[16];
	int ret;
	static unsigned int prv_buf;
	bool flag = false;
	unsigned int state;

	qsfp_module_info_testing = get_cable_attach(qsfp_temp);
	get_cable_info((char *)(qsfp_vendor_info));
	qsfp_rx_tx_channel_info = get_tx_rx_loss(qsfp_temp);

	if (get_qsfp_options(qsfp_temp)) {
		qsfp_rx_tx_channel_info = get_tx_rx_loss(qsfp_temp);
			if (prv_buf != qsfp_rx_tx_channel_info) {
				prv_buf = qsfp_rx_tx_channel_info;
				flag = true;
			}
		if (flag) {
			qsfp_sm_event(qsfp, state & QSFP_OPTIONS_TX_LOSS_SIGNAL ?
					QSFP_E_TX_LOS : QSFP_E_RX_LOS);
	}
	}

	ret = sff_gpio_get_state(qsfp);

	rtnl_lock();
	qsfp_sm_event(qsfp, QSFP_E_INSERT);
	rtnl_unlock();
}

static void qsfp_poll(struct work_struct *work)
{
	struct qsfp *qsfp = container_of(work, struct qsfp, poll.work);

	qsfp_check_state(qsfp);

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
	int err;

	struct qsfp *qsfp;

	qsfp = qsfp_alloc(&pdev->dev);

	if (IS_ERR(qsfp))
		return PTR_ERR(qsfp);

	platform_set_drvdata(pdev, qsfp);

	err = devm_add_action(qsfp->dev, qsfp_cleanup, qsfp);
	if (err < 0)
		return err;

	qsfp->qsfp_bus =
		qsfp_register_socket(qsfp->dev, qsfp, &qsfp_module_ops);
	if (!qsfp->qsfp_bus)
		return -ENOMEM;
	qsfp->need_poll = true;

	if (qsfp->need_poll)
		mod_delayed_work(system_wq, &qsfp->poll, poll_jiffies);

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
