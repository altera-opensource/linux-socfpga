// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright(c) 2015-17 Intel Corporation.

/*
 * Cadence SoundWire Master module
 * Used by Master driver
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw.h>
#include "bus.h"
#include "cadence_master.h"

#define CDNS_MCP_CONFIG				0x0

#define CDNS_MCP_CONFIG_MCMD_RETRY		GENMASK(27, 24)
#define CDNS_MCP_CONFIG_MPREQ_DELAY		GENMASK(20, 16)
#define CDNS_MCP_CONFIG_MMASTER			BIT(7)
#define CDNS_MCP_CONFIG_BUS_REL			BIT(6)
#define CDNS_MCP_CONFIG_SNIFFER			BIT(5)
#define CDNS_MCP_CONFIG_SSPMOD			BIT(4)
#define CDNS_MCP_CONFIG_CMD			BIT(3)
#define CDNS_MCP_CONFIG_OP			GENMASK(2, 0)
#define CDNS_MCP_CONFIG_OP_NORMAL		0

#define CDNS_MCP_CONTROL			0x4

#define CDNS_MCP_CONTROL_RST_DELAY		GENMASK(10, 8)
#define CDNS_MCP_CONTROL_CMD_RST		BIT(7)
#define CDNS_MCP_CONTROL_SOFT_RST		BIT(6)
#define CDNS_MCP_CONTROL_SW_RST			BIT(5)
#define CDNS_MCP_CONTROL_HW_RST			BIT(4)
#define CDNS_MCP_CONTROL_CLK_PAUSE		BIT(3)
#define CDNS_MCP_CONTROL_CLK_STOP_CLR		BIT(2)
#define CDNS_MCP_CONTROL_CMD_ACCEPT		BIT(1)
#define CDNS_MCP_CONTROL_BLOCK_WAKEUP		BIT(0)


#define CDNS_MCP_CMDCTRL			0x8
#define CDNS_MCP_SSPSTAT			0xC
#define CDNS_MCP_FRAME_SHAPE			0x10
#define CDNS_MCP_FRAME_SHAPE_INIT		0x14

#define CDNS_MCP_CONFIG_UPDATE			0x18
#define CDNS_MCP_CONFIG_UPDATE_BIT		BIT(0)

#define CDNS_MCP_PHYCTRL			0x1C
#define CDNS_MCP_SSP_CTRL0			0x20
#define CDNS_MCP_SSP_CTRL1			0x28
#define CDNS_MCP_CLK_CTRL0			0x30
#define CDNS_MCP_CLK_CTRL1			0x38

#define CDNS_MCP_STAT				0x40

#define CDNS_MCP_STAT_ACTIVE_BANK		BIT(20)
#define CDNS_MCP_STAT_CLK_STOP			BIT(16)

#define CDNS_MCP_INTSTAT			0x44
#define CDNS_MCP_INTMASK			0x48

#define CDNS_MCP_INT_IRQ			BIT(31)
#define CDNS_MCP_INT_WAKEUP			BIT(16)
#define CDNS_MCP_INT_SLAVE_RSVD			BIT(15)
#define CDNS_MCP_INT_SLAVE_ALERT		BIT(14)
#define CDNS_MCP_INT_SLAVE_ATTACH		BIT(13)
#define CDNS_MCP_INT_SLAVE_NATTACH		BIT(12)
#define CDNS_MCP_INT_SLAVE_MASK			GENMASK(15, 12)
#define CDNS_MCP_INT_DPINT			BIT(11)
#define CDNS_MCP_INT_CTRL_CLASH			BIT(10)
#define CDNS_MCP_INT_DATA_CLASH			BIT(9)
#define CDNS_MCP_INT_CMD_ERR			BIT(7)
#define CDNS_MCP_INT_RX_WL			BIT(2)
#define CDNS_MCP_INT_TXE			BIT(1)

#define CDNS_MCP_INTSET				0x4C

#define CDNS_SDW_SLAVE_STAT			0x50
#define CDNS_MCP_SLAVE_STAT_MASK		BIT(1, 0)

#define CDNS_MCP_SLAVE_INTSTAT0			0x54
#define CDNS_MCP_SLAVE_INTSTAT1			0x58
#define CDNS_MCP_SLAVE_INTSTAT_NPRESENT		BIT(0)
#define CDNS_MCP_SLAVE_INTSTAT_ATTACHED		BIT(1)
#define CDNS_MCP_SLAVE_INTSTAT_ALERT		BIT(2)
#define CDNS_MCP_SLAVE_INTSTAT_RESERVED		BIT(3)
#define CDNS_MCP_SLAVE_STATUS_BITS		GENMASK(3, 0)
#define CDNS_MCP_SLAVE_STATUS_NUM		4

#define CDNS_MCP_SLAVE_INTMASK0			0x5C
#define CDNS_MCP_SLAVE_INTMASK1			0x60

#define CDNS_MCP_SLAVE_INTMASK0_MASK		GENMASK(30, 0)
#define CDNS_MCP_SLAVE_INTMASK1_MASK		GENMASK(16, 0)

#define CDNS_MCP_PORT_INTSTAT			0x64
#define CDNS_MCP_PDI_STAT			0x6C

#define CDNS_MCP_FIFOLEVEL			0x78
#define CDNS_MCP_FIFOSTAT			0x7C
#define CDNS_MCP_RX_FIFO_AVAIL			GENMASK(5, 0)

#define CDNS_MCP_CMD_BASE			0x80
#define CDNS_MCP_RESP_BASE			0x80
#define CDNS_MCP_CMD_LEN			0x20
#define CDNS_MCP_CMD_WORD_LEN			0x4

#define CDNS_MCP_CMD_SSP_TAG			BIT(31)
#define CDNS_MCP_CMD_COMMAND			GENMASK(30, 28)
#define CDNS_MCP_CMD_DEV_ADDR			GENMASK(27, 24)
#define CDNS_MCP_CMD_REG_ADDR_H			GENMASK(23, 16)
#define CDNS_MCP_CMD_REG_ADDR_L			GENMASK(15, 8)
#define CDNS_MCP_CMD_REG_DATA			GENMASK(7, 0)

#define CDNS_MCP_CMD_READ			2
#define CDNS_MCP_CMD_WRITE			3

#define CDNS_MCP_RESP_RDATA			GENMASK(15, 8)
#define CDNS_MCP_RESP_ACK			BIT(0)
#define CDNS_MCP_RESP_NACK			BIT(1)

#define CDNS_DP_SIZE				128

#define CDNS_DPN_B0_CONFIG(n)			(0x100 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B0_CH_EN(n)			(0x104 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B0_SAMPLE_CTRL(n)		(0x108 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B0_OFFSET_CTRL(n)		(0x10C + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B0_HCTRL(n)			(0x110 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B0_ASYNC_CTRL(n)		(0x114 + CDNS_DP_SIZE * (n))

#define CDNS_DPN_B1_CONFIG(n)			(0x118 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B1_CH_EN(n)			(0x11C + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B1_SAMPLE_CTRL(n)		(0x120 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B1_OFFSET_CTRL(n)		(0x124 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B1_HCTRL(n)			(0x128 + CDNS_DP_SIZE * (n))
#define CDNS_DPN_B1_ASYNC_CTRL(n)		(0x12C + CDNS_DP_SIZE * (n))

#define CDNS_DPN_CONFIG_BPM			BIT(18)
#define CDNS_DPN_CONFIG_BGC			GENMASK(17, 16)
#define CDNS_DPN_CONFIG_WL			GENMASK(12, 8)
#define CDNS_DPN_CONFIG_PORT_DAT		GENMASK(3, 2)
#define CDNS_DPN_CONFIG_PORT_FLOW		GENMASK(1, 0)

#define CDNS_DPN_SAMPLE_CTRL_SI			GENMASK(15, 0)

#define CDNS_DPN_OFFSET_CTRL_1			GENMASK(7, 0)
#define CDNS_DPN_OFFSET_CTRL_2			GENMASK(15, 8)

#define CDNS_DPN_HCTRL_HSTOP			GENMASK(3, 0)
#define CDNS_DPN_HCTRL_HSTART			GENMASK(7, 4)
#define CDNS_DPN_HCTRL_LCTRL			GENMASK(10, 8)

#define CDNS_PORTCTRL				0x130
#define CDNS_PORTCTRL_DIRN			BIT(7)
#define CDNS_PORTCTRL_BANK_INVERT		BIT(8)

#define CDNS_PORT_OFFSET			0x80

#define CDNS_PDI_CONFIG(n)			(0x1100 + (n) * 16)

#define CDNS_PDI_CONFIG_SOFT_RESET		BIT(24)
#define CDNS_PDI_CONFIG_CHANNEL			GENMASK(15, 8)
#define CDNS_PDI_CONFIG_PORT			GENMASK(4, 0)

/* Driver defaults */

#define CDNS_DEFAULT_CLK_DIVIDER		0
#define CDNS_DEFAULT_FRAME_SHAPE		0x30
#define CDNS_DEFAULT_SSP_INTERVAL		0x18
#define CDNS_TX_TIMEOUT				2000

#define CDNS_PCM_PDI_OFFSET			0x2
#define CDNS_PDM_PDI_OFFSET			0x6

#define CDNS_SCP_RX_FIFOLEVEL			0x2

/*
 * register accessor helpers
 */
static inline u32 cdns_readl(struct sdw_cdns *cdns, int offset)
{
	return readl(cdns->registers + offset);
}

static inline void cdns_writel(struct sdw_cdns *cdns, int offset, u32 value)
{
	writel(value, cdns->registers + offset);
}

static inline void cdns_updatel(struct sdw_cdns *cdns,
				int offset, u32 mask, u32 val)
{
	u32 tmp;

	tmp = cdns_readl(cdns, offset);
	tmp = (tmp & ~mask) | val;
	cdns_writel(cdns, offset, tmp);
}

static int cdns_clear_bit(struct sdw_cdns *cdns, int offset, u32 value)
{
	int timeout = 10;
	u32 reg_read;

	writel(value, cdns->registers + offset);

	/* Wait for bit to be self cleared */
	do {
		reg_read = readl(cdns->registers + offset);
		if ((reg_read & value) == 0)
			return 0;

		timeout--;
		udelay(50);
	} while (timeout != 0);

	return -EAGAIN;
}

/*
 * IO Calls
 */
static enum sdw_command_response cdns_fill_msg_resp(
			struct sdw_cdns *cdns,
			struct sdw_msg *msg, int count, int offset)
{
	int nack = 0, no_ack = 0;
	int i;

	/* check message response */
	for (i = 0; i < count; i++) {
		if (!(cdns->response_buf[i] & CDNS_MCP_RESP_ACK)) {
			no_ack = 1;
			dev_dbg(cdns->dev, "Msg Ack not received\n");
			if (cdns->response_buf[i] & CDNS_MCP_RESP_NACK) {
				nack = 1;
				dev_err(cdns->dev, "Msg NACK received\n");
			}
		}
	}

	if (nack) {
		dev_err(cdns->dev, "Msg NACKed for Slave %d\n", msg->dev_num);
		return SDW_CMD_FAIL;
	} else if (no_ack) {
		dev_dbg(cdns->dev, "Msg ignored for Slave %d\n", msg->dev_num);
		return SDW_CMD_IGNORED;
	}

	/* fill response */
	for (i = 0; i < count; i++)
		msg->buf[i + offset] = cdns->response_buf[i] >>
				SDW_REG_SHIFT(CDNS_MCP_RESP_RDATA);

	return SDW_CMD_OK;
}

static enum sdw_command_response
_cdns_xfer_msg(struct sdw_cdns *cdns, struct sdw_msg *msg, int cmd,
				int offset, int count, bool defer)
{
	unsigned long time;
	u32 base, i, data;
	u16 addr;

	/* Program the watermark level for RX FIFO */
	if (cdns->msg_count != count) {
		cdns_writel(cdns, CDNS_MCP_FIFOLEVEL, count);
		cdns->msg_count = count;
	}

	base = CDNS_MCP_CMD_BASE;
	addr = msg->addr;

	for (i = 0; i < count; i++) {
		data = msg->dev_num << SDW_REG_SHIFT(CDNS_MCP_CMD_DEV_ADDR);
		data |= cmd << SDW_REG_SHIFT(CDNS_MCP_CMD_COMMAND);
		data |= addr++  << SDW_REG_SHIFT(CDNS_MCP_CMD_REG_ADDR_L);

		if (msg->flags == SDW_MSG_FLAG_WRITE)
			data |= msg->buf[i + offset];

		data |= msg->ssp_sync << SDW_REG_SHIFT(CDNS_MCP_CMD_SSP_TAG);
		cdns_writel(cdns, base, data);
		base += CDNS_MCP_CMD_WORD_LEN;
	}

	if (defer)
		return SDW_CMD_OK;

	/* wait for timeout or response */
	time = wait_for_completion_timeout(&cdns->tx_complete,
				msecs_to_jiffies(CDNS_TX_TIMEOUT));
	if (!time) {
		dev_err(cdns->dev, "IO transfer timed out\n");
		msg->len = 0;
		return SDW_CMD_TIMEOUT;
	}

	return cdns_fill_msg_resp(cdns, msg, count, offset);
}

static enum sdw_command_response cdns_program_scp_addr(
			struct sdw_cdns *cdns, struct sdw_msg *msg)
{
	int nack = 0, no_ack = 0;
	unsigned long time;
	u32 data[2], base;
	int i;

	/* Program the watermark level for RX FIFO */
	if (cdns->msg_count != CDNS_SCP_RX_FIFOLEVEL) {
		cdns_writel(cdns, CDNS_MCP_FIFOLEVEL, CDNS_SCP_RX_FIFOLEVEL);
		cdns->msg_count = CDNS_SCP_RX_FIFOLEVEL;
	}

	data[0] = msg->dev_num << SDW_REG_SHIFT(CDNS_MCP_CMD_DEV_ADDR);
	data[0] |= 0x3 << SDW_REG_SHIFT(CDNS_MCP_CMD_COMMAND);
	data[1] = data[0];

	data[0] |= SDW_SCP_ADDRPAGE1 << SDW_REG_SHIFT(CDNS_MCP_CMD_REG_ADDR_L);
	data[1] |= SDW_SCP_ADDRPAGE2 << SDW_REG_SHIFT(CDNS_MCP_CMD_REG_ADDR_L);

	data[0] |= msg->addr_page1;
	data[1] |= msg->addr_page2;

	base = CDNS_MCP_CMD_BASE;
	cdns_writel(cdns, base, data[0]);
	base += CDNS_MCP_CMD_WORD_LEN;
	cdns_writel(cdns, base, data[1]);

	time = wait_for_completion_timeout(&cdns->tx_complete,
				msecs_to_jiffies(CDNS_TX_TIMEOUT));
	if (!time) {
		dev_err(cdns->dev, "SCP Msg trf timed out\n");
		msg->len = 0;
		return SDW_CMD_TIMEOUT;
	}

	/* check response the writes */
	for (i = 0; i < 2; i++) {
		if (!(cdns->response_buf[i] & CDNS_MCP_RESP_ACK)) {
			no_ack = 1;
			dev_err(cdns->dev, "Program SCP Ack not received");
			if (cdns->response_buf[i] & CDNS_MCP_RESP_NACK) {
				nack = 1;
				dev_err(cdns->dev, "Program SCP NACK received");
			}
		}
	}

	/* For NACK, NO ack, don't return err if we are in Broadcast mode */
	if (nack) {
		dev_err(cdns->dev,
			"SCP_addrpage NACKed for Slave %d", msg->dev_num);
		return SDW_CMD_FAIL;
	} else if (no_ack) {
		dev_dbg(cdns->dev,
			"SCP_addrpage ignored for Slave %d", msg->dev_num);
		return SDW_CMD_IGNORED;
	}

	return SDW_CMD_OK;
}

static int cdns_prep_msg(struct sdw_cdns *cdns, struct sdw_msg *msg, int *cmd)
{
	int ret;

	if (msg->page) {
		ret = cdns_program_scp_addr(cdns, msg);
		if (ret) {
			msg->len = 0;
			return ret;
		}
	}

	switch (msg->flags) {
	case SDW_MSG_FLAG_READ:
		*cmd = CDNS_MCP_CMD_READ;
		break;

	case SDW_MSG_FLAG_WRITE:
		*cmd = CDNS_MCP_CMD_WRITE;
		break;

	default:
		dev_err(cdns->dev, "Invalid msg cmd: %d\n", msg->flags);
		return -EINVAL;
	}

	return 0;
}

static enum sdw_command_response
cdns_xfer_msg(struct sdw_bus *bus, struct sdw_msg *msg)
{
	struct sdw_cdns *cdns = bus_to_cdns(bus);
	int cmd = 0, ret, i;

	ret = cdns_prep_msg(cdns, msg, &cmd);
	if (ret)
		return SDW_CMD_FAIL_OTHER;

	for (i = 0; i < msg->len / CDNS_MCP_CMD_LEN; i++) {
		ret = _cdns_xfer_msg(cdns, msg, cmd, i * CDNS_MCP_CMD_LEN,
				CDNS_MCP_CMD_LEN, false);
		if (ret < 0)
			goto exit;
	}

	if (!(msg->len % CDNS_MCP_CMD_LEN))
		goto exit;

	ret = _cdns_xfer_msg(cdns, msg, cmd, i * CDNS_MCP_CMD_LEN,
			msg->len % CDNS_MCP_CMD_LEN, false);

exit:
	return ret;
}

static enum sdw_command_response
cdns_xfer_msg_defer(struct sdw_bus *bus,
		struct sdw_msg *msg, struct sdw_defer *defer)
{
	struct sdw_cdns *cdns = bus_to_cdns(bus);
	int cmd = 0, ret;

	/* for defer only 1 message is supported */
	if (msg->len > 1)
		return -ENOTSUPP;

	ret = cdns_prep_msg(cdns, msg, &cmd);
	if (ret)
		return SDW_CMD_FAIL_OTHER;

	cdns->defer = defer;
	cdns->defer->length = msg->len;

	return _cdns_xfer_msg(cdns, msg, cmd, 0, msg->len, true);
}

static enum sdw_command_response
cdns_reset_page_addr(struct sdw_bus *bus, unsigned int dev_num)
{
	struct sdw_cdns *cdns = bus_to_cdns(bus);
	struct sdw_msg msg;

	/* Create dummy message with valid device number */
	memset(&msg, 0, sizeof(msg));
	msg.dev_num = dev_num;

	return cdns_program_scp_addr(cdns, &msg);
}

/*
 * IRQ handling
 */

static void cdns_read_response(struct sdw_cdns *cdns)
{
	u32 num_resp, cmd_base;
	int i;

	num_resp = cdns_readl(cdns, CDNS_MCP_FIFOSTAT);
	num_resp &= CDNS_MCP_RX_FIFO_AVAIL;

	cmd_base = CDNS_MCP_CMD_BASE;

	for (i = 0; i < num_resp; i++) {
		cdns->response_buf[i] = cdns_readl(cdns, cmd_base);
		cmd_base += CDNS_MCP_CMD_WORD_LEN;
	}
}

static int cdns_update_slave_status(struct sdw_cdns *cdns,
					u32 slave0, u32 slave1)
{
	enum sdw_slave_status status[SDW_MAX_DEVICES + 1];
	bool is_slave = false;
	u64 slave, mask;
	int i, set_status;

	/* combine the two status */
	slave = ((u64)slave1 << 32) | slave0;
	memset(status, 0, sizeof(status));

	for (i = 0; i <= SDW_MAX_DEVICES; i++) {
		mask = (slave >> (i * CDNS_MCP_SLAVE_STATUS_NUM)) &
				CDNS_MCP_SLAVE_STATUS_BITS;
		if (!mask)
			continue;

		is_slave = true;
		set_status = 0;

		if (mask & CDNS_MCP_SLAVE_INTSTAT_RESERVED) {
			status[i] = SDW_SLAVE_RESERVED;
			set_status++;
		}

		if (mask & CDNS_MCP_SLAVE_INTSTAT_ATTACHED) {
			status[i] = SDW_SLAVE_ATTACHED;
			set_status++;
		}

		if (mask & CDNS_MCP_SLAVE_INTSTAT_ALERT) {
			status[i] = SDW_SLAVE_ALERT;
			set_status++;
		}

		if (mask & CDNS_MCP_SLAVE_INTSTAT_NPRESENT) {
			status[i] = SDW_SLAVE_UNATTACHED;
			set_status++;
		}

		/* first check if Slave reported multiple status */
		if (set_status > 1) {
			dev_warn(cdns->dev,
					"Slave reported multiple Status: %d\n",
					status[i]);
			/*
			 * TODO: we need to reread the status here by
			 * issuing a PING cmd
			 */
		}
	}

	if (is_slave)
		return sdw_handle_slave_status(&cdns->bus, status);

	return 0;
}

/**
 * sdw_cdns_irq() - Cadence interrupt handler
 * @irq: irq number
 * @dev_id: irq context
 */
irqreturn_t sdw_cdns_irq(int irq, void *dev_id)
{
	struct sdw_cdns *cdns = dev_id;
	u32 int_status;
	int ret = IRQ_HANDLED;

	/* Check if the link is up */
	if (!cdns->link_up)
		return IRQ_NONE;

	int_status = cdns_readl(cdns, CDNS_MCP_INTSTAT);

	if (!(int_status & CDNS_MCP_INT_IRQ))
		return IRQ_NONE;

	if (int_status & CDNS_MCP_INT_RX_WL) {
		cdns_read_response(cdns);

		if (cdns->defer) {
			cdns_fill_msg_resp(cdns, cdns->defer->msg,
					cdns->defer->length, 0);
			complete(&cdns->defer->complete);
			cdns->defer = NULL;
		} else
			complete(&cdns->tx_complete);
	}

	if (int_status & CDNS_MCP_INT_CTRL_CLASH) {

		/* Slave is driving bit slot during control word */
		dev_err_ratelimited(cdns->dev, "Bus clash for control word\n");
		int_status |= CDNS_MCP_INT_CTRL_CLASH;
	}

	if (int_status & CDNS_MCP_INT_DATA_CLASH) {
		/*
		 * Multiple slaves trying to drive bit slot, or issue with
		 * ownership of data bits or Slave gone bonkers
		 */
		dev_err_ratelimited(cdns->dev, "Bus clash for data word\n");
		int_status |= CDNS_MCP_INT_DATA_CLASH;
	}

	if (int_status & CDNS_MCP_INT_SLAVE_MASK) {
		/* Mask the Slave interrupt and wake thread */
		cdns_updatel(cdns, CDNS_MCP_INTMASK,
				CDNS_MCP_INT_SLAVE_MASK, 0);

		int_status &= ~CDNS_MCP_INT_SLAVE_MASK;
		ret = IRQ_WAKE_THREAD;
	}

	cdns_writel(cdns, CDNS_MCP_INTSTAT, int_status);
	return ret;
}
EXPORT_SYMBOL(sdw_cdns_irq);

/**
 * sdw_cdns_thread() - Cadence irq thread handler
 * @irq: irq number
 * @dev_id: irq context
 */
irqreturn_t sdw_cdns_thread(int irq, void *dev_id)
{
	struct sdw_cdns *cdns = dev_id;
	u32 slave0, slave1;

	dev_dbg(cdns->dev, "Slave status change\n");

	slave0 = cdns_readl(cdns, CDNS_MCP_SLAVE_INTSTAT0);
	slave1 = cdns_readl(cdns, CDNS_MCP_SLAVE_INTSTAT1);

	cdns_update_slave_status(cdns, slave0, slave1);
	cdns_writel(cdns, CDNS_MCP_SLAVE_INTSTAT0, slave0);
	cdns_writel(cdns, CDNS_MCP_SLAVE_INTSTAT1, slave1);

	/* clear and unmask Slave interrupt now */
	cdns_writel(cdns, CDNS_MCP_INTSTAT, CDNS_MCP_INT_SLAVE_MASK);
	cdns_updatel(cdns, CDNS_MCP_INTMASK,
			CDNS_MCP_INT_SLAVE_MASK, CDNS_MCP_INT_SLAVE_MASK);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(sdw_cdns_thread);

/*
 * init routines
 */
static int _cdns_enable_interrupt(struct sdw_cdns *cdns)
{
	u32 mask;

	cdns_writel(cdns, CDNS_MCP_SLAVE_INTMASK0,
				CDNS_MCP_SLAVE_INTMASK0_MASK);
	cdns_writel(cdns, CDNS_MCP_SLAVE_INTMASK1,
				CDNS_MCP_SLAVE_INTMASK1_MASK);

	mask = CDNS_MCP_INT_SLAVE_RSVD | CDNS_MCP_INT_SLAVE_ALERT |
		CDNS_MCP_INT_SLAVE_ATTACH | CDNS_MCP_INT_SLAVE_NATTACH |
		CDNS_MCP_INT_CTRL_CLASH | CDNS_MCP_INT_DATA_CLASH |
		CDNS_MCP_INT_RX_WL | CDNS_MCP_INT_IRQ | CDNS_MCP_INT_DPINT;

	cdns_writel(cdns, CDNS_MCP_INTMASK, mask);

	return 0;
}

/**
 * sdw_cdns_enable_interrupt() - Enable SDW interrupts and update config
 * @cdns: Cadence instance
 */
int sdw_cdns_enable_interrupt(struct sdw_cdns *cdns)
{
	int ret;

	_cdns_enable_interrupt(cdns);
	ret = cdns_clear_bit(cdns, CDNS_MCP_CONFIG_UPDATE,
			CDNS_MCP_CONFIG_UPDATE_BIT);
	if (ret < 0)
		dev_err(cdns->dev, "Config update timedout");

	return ret;
}
EXPORT_SYMBOL(sdw_cdns_enable_interrupt);

/**
 * sdw_cdns_init() - Cadence initialization
 * @cdns: Cadence instance
 */
int sdw_cdns_init(struct sdw_cdns *cdns)
{
	u32 val;
	int ret;

	/* Exit clock stop */
	ret = cdns_clear_bit(cdns, CDNS_MCP_CONTROL,
			CDNS_MCP_CONTROL_CLK_STOP_CLR);
	if (ret < 0) {
		dev_err(cdns->dev, "Couldn't exit from clock stop\n");
		return ret;
	}

	/* Set clock divider */
	val = cdns_readl(cdns, CDNS_MCP_CLK_CTRL0);
	val |= CDNS_DEFAULT_CLK_DIVIDER;
	cdns_writel(cdns, CDNS_MCP_CLK_CTRL0, val);

	/* Set the default frame shape */
	cdns_writel(cdns, CDNS_MCP_FRAME_SHAPE_INIT, CDNS_DEFAULT_FRAME_SHAPE);

	/* Set SSP interval to default value */
	cdns_writel(cdns, CDNS_MCP_SSP_CTRL0, CDNS_DEFAULT_SSP_INTERVAL);
	cdns_writel(cdns, CDNS_MCP_SSP_CTRL1, CDNS_DEFAULT_SSP_INTERVAL);

	/* Set cmd accept mode */
	cdns_updatel(cdns, CDNS_MCP_CONTROL, CDNS_MCP_CONTROL_CMD_ACCEPT,
					CDNS_MCP_CONTROL_CMD_ACCEPT);

	/* Configure mcp config */
	val = cdns_readl(cdns, CDNS_MCP_CONFIG);

	/* Set Max cmd retry to 15 */
	val |= CDNS_MCP_CONFIG_MCMD_RETRY;

	/* Set frame delay between PREQ and ping frame to 15 frames */
	val |= 0xF << SDW_REG_SHIFT(CDNS_MCP_CONFIG_MPREQ_DELAY);

	/* Disable auto bus release */
	val &= ~CDNS_MCP_CONFIG_BUS_REL;

	/* Disable sniffer mode */
	val &= ~CDNS_MCP_CONFIG_SNIFFER;

	/* Set cmd mode for Tx and Rx cmds */
	val &= ~CDNS_MCP_CONFIG_CMD;

	/* Set operation to normal */
	val &= ~CDNS_MCP_CONFIG_OP;
	val |= CDNS_MCP_CONFIG_OP_NORMAL;

	cdns_writel(cdns, CDNS_MCP_CONFIG, val);

	return 0;
}
EXPORT_SYMBOL(sdw_cdns_init);

struct sdw_master_ops sdw_cdns_master_ops = {
	.read_prop = sdw_master_read_prop,
	.xfer_msg = cdns_xfer_msg,
	.xfer_msg_defer = cdns_xfer_msg_defer,
	.reset_page_addr = cdns_reset_page_addr,
};
EXPORT_SYMBOL(sdw_cdns_master_ops);

/**
 * sdw_cdns_probe() - Cadence probe routine
 * @cdns: Cadence instance
 */
int sdw_cdns_probe(struct sdw_cdns *cdns)
{
	init_completion(&cdns->tx_complete);

	return 0;
}
EXPORT_SYMBOL(sdw_cdns_probe);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Cadence Soundwire Library");
