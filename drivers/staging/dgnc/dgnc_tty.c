// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2003 Digi International (www.digi.com)
 *	Scott H Kilau <Scott_Kilau at digi dot com>
 */

/*
 * This file implements the tty driver functionality for the
 * Neo and ClassicBoard PCI based product lines.
 */

#include <linux/kernel.h>
#include <linux/sched/signal.h>	/* For jiffies, task states, etc. */
#include <linux/interrupt.h>	/* For tasklet and interrupt structs/defines */
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/delay.h>	/* For udelay */
#include <linux/uaccess.h>	/* For copy_from_user/copy_to_user */
#include <linux/pci.h>
#include "dgnc_driver.h"
#include "dgnc_tty.h"
#include "dgnc_cls.h"

/* Default transparent print information. */

static const struct digi_t dgnc_digi_init = {
	.digi_flags =	DIGI_COOK,	/* Flags */
	.digi_maxcps =	100,		/* Max CPS */
	.digi_maxchar =	50,		/* Max chars in print queue */
	.digi_bufsize =	100,		/* Printer buffer size */
	.digi_onlen =	4,		/* size of printer on string */
	.digi_offlen =	4,		/* size of printer off string */
	.digi_onstr =	"\033[5i",	/* ANSI printer on string ] */
	.digi_offstr =	"\033[4i",	/* ANSI printer off string ] */
	.digi_term =	"ansi"		/* default terminal type */
};

static int dgnc_tty_open(struct tty_struct *tty, struct file *file);
static void dgnc_tty_close(struct tty_struct *tty, struct file *file);
static int dgnc_block_til_ready(struct tty_struct *tty, struct file *file,
				struct channel_t *ch);
static int dgnc_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
			  unsigned long arg);
static int dgnc_tty_digigeta(struct tty_struct *tty,
			     struct digi_t __user *retinfo);
static int dgnc_tty_digiseta(struct tty_struct *tty,
			     struct digi_t __user *new_info);
static int dgnc_tty_write_room(struct tty_struct *tty);
static int dgnc_tty_put_char(struct tty_struct *tty, unsigned char c);
static int dgnc_tty_chars_in_buffer(struct tty_struct *tty);
static void dgnc_tty_start(struct tty_struct *tty);
static void dgnc_tty_stop(struct tty_struct *tty);
static void dgnc_tty_throttle(struct tty_struct *tty);
static void dgnc_tty_unthrottle(struct tty_struct *tty);
static void dgnc_tty_flush_chars(struct tty_struct *tty);
static void dgnc_tty_flush_buffer(struct tty_struct *tty);
static void dgnc_tty_hangup(struct tty_struct *tty);
static int dgnc_set_modem_info(struct channel_t *ch, unsigned int command,
			       unsigned int __user *value);
static int dgnc_get_modem_info(struct channel_t *ch,
			       unsigned int __user *value);
static int dgnc_tty_tiocmget(struct tty_struct *tty);
static int dgnc_tty_tiocmset(struct tty_struct *tty, unsigned int set,
			     unsigned int clear);
static int dgnc_tty_send_break(struct tty_struct *tty, int msec);
static void dgnc_tty_wait_until_sent(struct tty_struct *tty, int timeout);
static int dgnc_tty_write(struct tty_struct *tty, const unsigned char *buf,
			  int count);
static void dgnc_tty_set_termios(struct tty_struct *tty,
				 struct ktermios *old_termios);
static void dgnc_tty_send_xchar(struct tty_struct *tty, char ch);
static void dgnc_set_signal_low(struct channel_t *ch, const unsigned char line);
static void dgnc_wake_up_unit(struct un_t *unit);

static const struct tty_operations dgnc_tty_ops = {
	.open = dgnc_tty_open,
	.close = dgnc_tty_close,
	.write = dgnc_tty_write,
	.write_room = dgnc_tty_write_room,
	.flush_buffer = dgnc_tty_flush_buffer,
	.chars_in_buffer = dgnc_tty_chars_in_buffer,
	.flush_chars = dgnc_tty_flush_chars,
	.ioctl = dgnc_tty_ioctl,
	.set_termios = dgnc_tty_set_termios,
	.stop = dgnc_tty_stop,
	.start = dgnc_tty_start,
	.throttle = dgnc_tty_throttle,
	.unthrottle = dgnc_tty_unthrottle,
	.hangup = dgnc_tty_hangup,
	.put_char = dgnc_tty_put_char,
	.tiocmget = dgnc_tty_tiocmget,
	.tiocmset = dgnc_tty_tiocmset,
	.break_ctl = dgnc_tty_send_break,
	.wait_until_sent = dgnc_tty_wait_until_sent,
	.send_xchar = dgnc_tty_send_xchar
};

/* TTY Initialization/Cleanup Functions */

static struct tty_driver *dgnc_tty_create(char *serial_name, uint maxports,
					  int major, int minor)
{
	int rc;
	struct tty_driver *drv;

	drv = tty_alloc_driver(maxports,
			       TTY_DRIVER_REAL_RAW |
			       TTY_DRIVER_DYNAMIC_DEV |
			       TTY_DRIVER_HARDWARE_BREAK);
	if (IS_ERR(drv))
		return drv;

	drv->name = serial_name;
	drv->name_base = 0;
	drv->major = major;
	drv->minor_start = minor;
	drv->type = TTY_DRIVER_TYPE_SERIAL;
	drv->subtype = SERIAL_TYPE_NORMAL;
	drv->init_termios = tty_std_termios;
	drv->init_termios.c_cflag = (B9600 | CS8 | CREAD | HUPCL | CLOCAL);
	drv->init_termios.c_ispeed = 9600;
	drv->init_termios.c_ospeed = 9600;
	drv->driver_name = DRVSTR;
	/*
	 * Entry points for driver.  Called by the kernel from
	 * tty_io.c and n_tty.c.
	 */
	tty_set_operations(drv, &dgnc_tty_ops);
	rc = tty_register_driver(drv);
	if (rc < 0) {
		put_tty_driver(drv);
		return ERR_PTR(rc);
	}
	return drv;
}

static void dgnc_tty_free(struct tty_driver *drv)
{
	tty_unregister_driver(drv);
	put_tty_driver(drv);
}

/**
 * dgnc_tty_register() - Init the tty subsystem for this board.
 */
int dgnc_tty_register(struct dgnc_board *brd)
{
	int rc;

	snprintf(brd->serial_name, MAXTTYNAMELEN, "tty_dgnc_%d_",
		 brd->boardnum);

	brd->serial_driver = dgnc_tty_create(brd->serial_name,
					     brd->maxports, 0, 0);
	if (IS_ERR(brd->serial_driver)) {
		rc = PTR_ERR(brd->serial_driver);
		dev_dbg(&brd->pdev->dev, "Can't register tty device (%d)\n",
			rc);
		return rc;
	}

	snprintf(brd->print_name, MAXTTYNAMELEN, "pr_dgnc_%d_", brd->boardnum);
	brd->print_driver = dgnc_tty_create(brd->print_name, brd->maxports,
					    0x80,
					    brd->serial_driver->major);
	if (IS_ERR(brd->print_driver)) {
		rc = PTR_ERR(brd->print_driver);
		dev_dbg(&brd->pdev->dev,
			"Can't register Transparent Print device(%d)\n", rc);
		dgnc_tty_free(brd->serial_driver);
		return rc;
	}
	return 0;
}

void dgnc_tty_unregister(struct dgnc_board *brd)
{
	dgnc_tty_free(brd->print_driver);
	dgnc_tty_free(brd->serial_driver);
}

/**
 * dgnc_tty_init() - Initialize the tty subsystem.
 *
 * Called once per board after board has been downloaded and initialized.
 */
int dgnc_tty_init(struct dgnc_board *brd)
{
	int i;
	int rc;
	void __iomem *vaddr;
	struct channel_t *ch;

	if (!brd)
		return -ENXIO;

	/* Initialize board structure elements. */

	vaddr = brd->re_map_membase;

	brd->nasync = brd->maxports;

	for (i = 0; i < brd->nasync; i++) {
		brd->channels[i] = kzalloc(sizeof(*brd->channels[i]),
					   GFP_KERNEL);
		if (!brd->channels[i]) {
			rc = -ENOMEM;
			goto err_free_channels;
		}
	}

	ch = brd->channels[0];
	vaddr = brd->re_map_membase;

	/* Set up channel variables */
	for (i = 0; i < brd->nasync; i++, ch = brd->channels[i]) {
		spin_lock_init(&ch->ch_lock);

		ch->ch_tun.un_ch = ch;
		ch->ch_tun.un_type = DGNC_SERIAL;
		ch->ch_tun.un_dev = i;

		ch->ch_pun.un_ch = ch;
		ch->ch_pun.un_type = DGNC_PRINT;
		ch->ch_pun.un_dev = i + 128;

		ch->ch_cls_uart = vaddr + (brd->bd_uart_offset * i);

		ch->ch_bd = brd;
		ch->ch_portnum = i;
		ch->ch_digi = dgnc_digi_init;

		/* .25 second delay */
		ch->ch_close_delay = 250;

		init_waitqueue_head(&ch->ch_flags_wait);
		init_waitqueue_head(&ch->ch_tun.un_flags_wait);
		init_waitqueue_head(&ch->ch_pun.un_flags_wait);

		{
			struct device *classp;

			classp = tty_register_device(brd->serial_driver, i,
						     &ch->ch_bd->pdev->dev);
			ch->ch_tun.un_sysfs = classp;

			classp = tty_register_device(brd->print_driver, i,
						     &ch->ch_bd->pdev->dev);
			ch->ch_pun.un_sysfs = classp;
		}
	}

	return 0;

err_free_channels:
	for (i = i - 1; i >= 0; --i) {
		kfree(brd->channels[i]);
		brd->channels[i] = NULL;
	}

	return rc;
}

/**
 * dgnc_cleanup_tty() - Cleanup driver.
 *
 * Uninitialize the TTY portion of this driver.  Free all memory and
 * resources.
 */
void dgnc_cleanup_tty(struct dgnc_board *brd)
{
	int i = 0;

	for (i = 0; i < brd->nasync; i++)
		tty_unregister_device(brd->serial_driver, i);

	tty_unregister_driver(brd->serial_driver);

	for (i = 0; i < brd->nasync; i++)
		tty_unregister_device(brd->print_driver, i);

	tty_unregister_driver(brd->print_driver);

	put_tty_driver(brd->serial_driver);
	put_tty_driver(brd->print_driver);
}

/**
 * dgnc_wmove() - Write data to transmit queue.
 * @ch: Pointer to channel structure.
 * @buf: Pointer to characters to be moved.
 * @n: Number of characters to move.
 */
static void dgnc_wmove(struct channel_t *ch, char *buf, uint n)
{
	int	remain;
	uint	head;

	if (!ch)
		return;

	head = ch->ch_w_head & WQUEUEMASK;

	/*
	 * If the write wraps over the top of the circular buffer,
	 * move the portion up to the wrap point, and reset the
	 * pointers to the bottom.
	 */
	remain = WQUEUESIZE - head;

	if (n >= remain) {
		n -= remain;
		memcpy(ch->ch_wqueue + head, buf, remain);
		head = 0;
		buf += remain;
	}

	if (n > 0) {
		/* Move rest of data. */
		remain = n;
		memcpy(ch->ch_wqueue + head, buf, remain);
		head += remain;
	}

	head &= WQUEUEMASK;
	ch->ch_w_head = head;
}

/**
 * dgnc_input() - Process received data.
 * @ch: Pointer to channel structure.
 */
void dgnc_input(struct channel_t *ch)
{
	struct dgnc_board *bd;
	struct tty_struct *tp;
	struct tty_ldisc *ld = NULL;
	uint	rmask;
	ushort	head;
	ushort	tail;
	int	data_len;
	unsigned long flags;
	int flip_len;
	int len = 0;
	int n = 0;
	int s = 0;
	int i = 0;

	if (!ch)
		return;

	tp = ch->ch_tun.un_tty;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	rmask = RQUEUEMASK;
	head = ch->ch_r_head & rmask;
	tail = ch->ch_r_tail & rmask;
	data_len = (head - tail) & rmask;

	if (data_len == 0)
		goto exit_unlock;

	/*
	 * If the device is not open, or CREAD is off,
	 * flush input data and return immediately.
	 */
	if (!tp ||
	    !(ch->ch_tun.un_flags & UN_ISOPEN) ||
	    !C_CREAD(tp) ||
	    (ch->ch_tun.un_flags & UN_CLOSING)) {
		ch->ch_r_head = tail;

		/* Force queue flow control to be released, if needed */
		dgnc_check_queue_flow_control(ch);

		goto exit_unlock;
	}

	if (ch->ch_flags & CH_FORCED_STOPI)
		goto exit_unlock;

	flip_len = TTY_FLIPBUF_SIZE;

	len = min(data_len, flip_len);
	len = min(len, (N_TTY_BUF_SIZE - 1));

	ld = tty_ldisc_ref(tp);
	if (!ld) {
		len = 0;
	} else {
		if (!ld->ops->receive_buf) {
			ch->ch_r_head = ch->ch_r_tail;
			len = 0;
		}
	}

	if (len <= 0)
		goto exit_unlock;

	/*
	 * The tty layer in the kernel has changed in 2.6.16+.
	 *
	 * The flip buffers in the tty structure are no longer exposed,
	 * and probably will be going away eventually.
	 *
	 * If we are completely raw, we don't need to go through a lot
	 * of the tty layers that exist.
	 * In this case, we take the shortest and fastest route we
	 * can to relay the data to the user.
	 *
	 * On the other hand, if we are not raw, we need to go through
	 * the new 2.6.16+ tty layer, which has its API more well defined.
	 */
	len = tty_buffer_request_room(tp->port, len);
	n = len;

	/*
	 * n now contains the most amount of data we can copy,
	 * bounded either by how much the Linux tty layer can handle,
	 * or the amount of data the card actually has pending...
	 */
	while (n) {
		unsigned char *ch_pos = ch->ch_equeue + tail;

		s = ((head >= tail) ? head : RQUEUESIZE) - tail;
		s = min(s, n);

		if (s <= 0)
			break;

		/*
		 * If conditions are such that ld needs to see all
		 * UART errors, we will have to walk each character
		 * and error byte and send them to the buffer one at
		 * a time.
		 */
		if (I_PARMRK(tp) || I_BRKINT(tp) || I_INPCK(tp)) {
			for (i = 0; i < s; i++) {
				unsigned char ch = *(ch_pos + i);
				char flag = TTY_NORMAL;

				if (ch & UART_LSR_BI)
					flag = TTY_BREAK;
				else if (ch & UART_LSR_PE)
					flag = TTY_PARITY;
				else if (ch & UART_LSR_FE)
					flag = TTY_FRAME;

				tty_insert_flip_char(tp->port, ch, flag);
			}
		} else {
			tty_insert_flip_string(tp->port, ch_pos, s);
		}

		tail += s;
		n -= s;
		/* Flip queue if needed */
		tail &= rmask;
	}

	ch->ch_r_tail = tail & rmask;
	ch->ch_e_tail = tail & rmask;
	dgnc_check_queue_flow_control(ch);
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	/* Tell the tty layer its okay to "eat" the data now */
	tty_flip_buffer_push(tp->port);

	if (ld)
		tty_ldisc_deref(ld);
	return;

exit_unlock:
	spin_unlock_irqrestore(&ch->ch_lock, flags);
	if (ld)
		tty_ldisc_deref(ld);
}

/**
 * dgnc_carrier()
 *
 * Determines when CARRIER changes state and takes appropriate
 * action.
 */
void dgnc_carrier(struct channel_t *ch)
{
	int virt_carrier = 0;
	int phys_carrier = 0;

	if (!ch)
		return;

	if (ch->ch_mistat & UART_MSR_DCD)
		phys_carrier = 1;

	if (ch->ch_digi.digi_flags & DIGI_FORCEDCD)
		virt_carrier = 1;

	if (ch->ch_c_cflag & CLOCAL)
		virt_carrier = 1;

	/* Test for a VIRTUAL carrier transition to HIGH. */

	if (((ch->ch_flags & CH_FCAR) == 0) && (virt_carrier == 1)) {
		/*
		 * When carrier rises, wake any threads waiting
		 * for carrier in the open routine.
		 */
		if (waitqueue_active(&ch->ch_flags_wait))
			wake_up_interruptible(&ch->ch_flags_wait);
	}

	/* Test for a PHYSICAL carrier transition to HIGH. */

	if (((ch->ch_flags & CH_CD) == 0) && (phys_carrier == 1)) {
		/*
		 * When carrier rises, wake any threads waiting
		 * for carrier in the open routine.
		 */
		if (waitqueue_active(&ch->ch_flags_wait))
			wake_up_interruptible(&ch->ch_flags_wait);
	}

	/*
	 *  Test for a PHYSICAL transition to low, so long as we aren't
	 *  currently ignoring physical transitions (which is what "virtual
	 *  carrier" indicates).
	 *
	 *  The transition of the virtual carrier to low really doesn't
	 *  matter... it really only means "ignore carrier state", not
	 *  "make pretend that carrier is there".
	 */
	if ((virt_carrier == 0) && ((ch->ch_flags & CH_CD) != 0) &&
	    (phys_carrier == 0)) {
		/*
		 *   When carrier drops:
		 *
		 *   Drop carrier on all open units.
		 *
		 *   Flush queues, waking up any task waiting in the
		 *   line discipline.
		 *
		 *   Send a hangup to the control terminal.
		 *
		 *   Enable all select calls.
		 */
		if (waitqueue_active(&ch->ch_flags_wait))
			wake_up_interruptible(&ch->ch_flags_wait);

		if (ch->ch_tun.un_open_count > 0)
			tty_hangup(ch->ch_tun.un_tty);

		if (ch->ch_pun.un_open_count > 0)
			tty_hangup(ch->ch_pun.un_tty);
	}

	/*  Make sure that our cached values reflect the current reality. */

	if (virt_carrier == 1)
		ch->ch_flags |= CH_FCAR;
	else
		ch->ch_flags &= ~CH_FCAR;

	if (phys_carrier == 1)
		ch->ch_flags |= CH_CD;
	else
		ch->ch_flags &= ~CH_CD;
}

/*  Assign the custom baud rate to the channel structure */
static void dgnc_set_custom_speed(struct channel_t *ch, uint newrate)
{
	int testdiv;
	int testrate_high;
	int testrate_low;
	int deltahigh;
	int deltalow;

	if (newrate <= 0) {
		ch->ch_custom_speed = 0;
		return;
	}

	/*
	 *  Since the divisor is stored in a 16-bit integer, we make sure
	 *  we don't allow any rates smaller than a 16-bit integer would allow.
	 *  And of course, rates above the dividend won't fly.
	 */
	if (newrate && newrate < ((ch->ch_bd->bd_dividend / 0xFFFF) + 1))
		newrate = (ch->ch_bd->bd_dividend / 0xFFFF) + 1;

	if (newrate && newrate > ch->ch_bd->bd_dividend)
		newrate = ch->ch_bd->bd_dividend;

	if (newrate > 0) {
		testdiv = ch->ch_bd->bd_dividend / newrate;

		/*
		 *  If we try to figure out what rate the board would use
		 *  with the test divisor, it will be either equal or higher
		 *  than the requested baud rate.  If we then determine the
		 *  rate with a divisor one higher, we will get the next lower
		 *  supported rate below the requested.
		 */
		testrate_high = ch->ch_bd->bd_dividend / testdiv;
		testrate_low  = ch->ch_bd->bd_dividend / (testdiv + 1);

		/*
		 *  If the rate for the requested divisor is correct, just
		 *  use it and be done.
		 */
		if (testrate_high != newrate) {
			/*
			 *  Otherwise, pick the rate that is closer
			 *  (i.e. whichever rate has a smaller delta).
			 */
			deltahigh = testrate_high - newrate;
			deltalow = newrate - testrate_low;

			if (deltahigh < deltalow)
				newrate = testrate_high;
			else
				newrate = testrate_low;
		}
	}

	ch->ch_custom_speed = newrate;
}

void dgnc_check_queue_flow_control(struct channel_t *ch)
{
	int qleft;

	qleft = ch->ch_r_tail - ch->ch_r_head - 1;
	if (qleft < 0)
		qleft += RQUEUEMASK + 1;

	/*
	 * Check to see if we should enforce flow control on our queue because
	 * the ld (or user) isn't reading data out of our queue fast enuf.
	 *
	 * NOTE: This is done based on what the current flow control of the
	 * port is set for.
	 *
	 * 1) HWFLOW (RTS) - Turn off the UART's Receive interrupt.
	 *	This will cause the UART's FIFO to back up, and force
	 *	the RTS signal to be dropped.
	 * 2) SWFLOW (IXOFF) - Keep trying to send a stop character to
	 *	the other side, in hopes it will stop sending data to us.
	 * 3) NONE - Nothing we can do.  We will simply drop any extra data
	 *	that gets sent into us when the queue fills up.
	 */
	if (qleft < 256) {
		/* HWFLOW */
		if (ch->ch_digi.digi_flags & CTSPACE ||
		    ch->ch_c_cflag & CRTSCTS) {
			if (!(ch->ch_flags & CH_RECEIVER_OFF)) {
				ch->ch_bd->bd_ops->disable_receiver(ch);
				ch->ch_flags |= (CH_RECEIVER_OFF);
			}
		}
		/* SWFLOW */
		else if (ch->ch_c_iflag & IXOFF) {
			if (ch->ch_stops_sent <= MAX_STOPS_SENT) {
				ch->ch_bd->bd_ops->send_stop_character(ch);
				ch->ch_stops_sent++;
			}
		}
	}

	/*
	 * Check to see if we should unenforce flow control because
	 * ld (or user) finally read enuf data out of our queue.
	 *
	 * NOTE: This is done based on what the current flow control of the
	 * port is set for.
	 *
	 * 1) HWFLOW (RTS) - Turn back on the UART's Receive interrupt.
	 *	This will cause the UART's FIFO to raise RTS back up,
	 *	which will allow the other side to start sending data again.
	 * 2) SWFLOW (IXOFF) - Send a start character to
	 *	the other side, so it will start sending data to us again.
	 * 3) NONE - Do nothing. Since we didn't do anything to turn off the
	 *	other side, we don't need to do anything now.
	 */
	if (qleft > (RQUEUESIZE / 2)) {
		/* HWFLOW */
		if (ch->ch_digi.digi_flags & RTSPACE ||
		    ch->ch_c_cflag & CRTSCTS) {
			if (ch->ch_flags & CH_RECEIVER_OFF) {
				ch->ch_bd->bd_ops->enable_receiver(ch);
				ch->ch_flags &= ~(CH_RECEIVER_OFF);
			}
		}
		/* SWFLOW */
		else if (ch->ch_c_iflag & IXOFF && ch->ch_stops_sent) {
			ch->ch_stops_sent = 0;
			ch->ch_bd->bd_ops->send_start_character(ch);
		}
	}
}

static void dgnc_set_signal_low(struct channel_t *ch, const unsigned char sig)
{
	ch->ch_mostat &= ~(sig);
	ch->ch_bd->bd_ops->assert_modem_signals(ch);
}

void dgnc_wakeup_writes(struct channel_t *ch)
{
	int qlen = 0;
	unsigned long flags;

	if (!ch)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	/* If channel now has space, wake up anyone waiting on the condition. */

	qlen = ch->ch_w_head - ch->ch_w_tail;
	if (qlen < 0)
		qlen += WQUEUESIZE;

	if (qlen >= (WQUEUESIZE - 256)) {
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return;
	}

	if (ch->ch_tun.un_flags & UN_ISOPEN) {
		tty_wakeup(ch->ch_tun.un_tty);

		/*
		 * If unit is set to wait until empty, check to make sure
		 * the queue AND FIFO are both empty.
		 */
		if (ch->ch_tun.un_flags & UN_EMPTY) {
			if ((qlen == 0) &&
			    (ch->ch_bd->bd_ops->get_uart_bytes_left(ch) == 0)) {
				ch->ch_tun.un_flags &= ~(UN_EMPTY);

				/*
				 * If RTS Toggle mode is on, whenever
				 * the queue and UART is empty, keep RTS low.
				 */
				if (ch->ch_digi.digi_flags & DIGI_RTS_TOGGLE)
					dgnc_set_signal_low(ch, UART_MCR_RTS);

				/*
				 * If DTR Toggle mode is on, whenever
				 * the queue and UART is empty, keep DTR low.
				 */
				if (ch->ch_digi.digi_flags & DIGI_DTR_TOGGLE)
					dgnc_set_signal_low(ch, UART_MCR_DTR);
			}
		}

		wake_up_interruptible(&ch->ch_tun.un_flags_wait);
	}

	if (ch->ch_pun.un_flags & UN_ISOPEN) {
		tty_wakeup(ch->ch_pun.un_tty);

		/*
		 * If unit is set to wait until empty, check to make sure
		 * the queue AND FIFO are both empty.
		 */
		if (ch->ch_pun.un_flags & UN_EMPTY) {
			if ((qlen == 0) &&
			    (ch->ch_bd->bd_ops->get_uart_bytes_left(ch) == 0))
				ch->ch_pun.un_flags &= ~(UN_EMPTY);
		}

		wake_up_interruptible(&ch->ch_pun.un_flags_wait);
	}

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

static struct dgnc_board *find_board_by_major(unsigned int major)
{
	int i;

	for (i = 0; i < MAXBOARDS; i++) {
		struct dgnc_board *brd = dgnc_board[i];

		if (!brd)
			return NULL;

		if (major == brd->serial_driver->major ||
		    major == brd->print_driver->major)
			return brd;
	}

	return NULL;
}

/* TTY Entry points and helper functions */

static int dgnc_tty_open(struct tty_struct *tty, struct file *file)
{
	struct dgnc_board	*brd;
	struct channel_t *ch;
	struct un_t	*un;
	uint		major = 0;
	uint		minor = 0;
	int		rc = 0;
	unsigned long flags;

	rc = 0;

	major = MAJOR(tty_devnum(tty));
	minor = MINOR(tty_devnum(tty));

	if (major > 255)
		return -ENXIO;

	brd = find_board_by_major(major);
	if (!brd)
		return -ENXIO;

	rc = wait_event_interruptible(brd->state_wait,
				      (brd->state & BOARD_READY));
	if (rc)
		return rc;

	spin_lock_irqsave(&brd->bd_lock, flags);

	if (PORT_NUM(minor) >= brd->nasync) {
		rc = -ENXIO;
		goto err_brd_unlock;
	}

	ch = brd->channels[PORT_NUM(minor)];
	if (!ch) {
		rc = -ENXIO;
		goto err_brd_unlock;
	}

	spin_unlock_irqrestore(&brd->bd_lock, flags);

	spin_lock_irqsave(&ch->ch_lock, flags);

	/* Figure out our type */
	if (!IS_PRINT(minor)) {
		un = &brd->channels[PORT_NUM(minor)]->ch_tun;
		un->un_type = DGNC_SERIAL;
	} else if (IS_PRINT(minor)) {
		un = &brd->channels[PORT_NUM(minor)]->ch_pun;
		un->un_type = DGNC_PRINT;
	} else {
		rc = -ENXIO;
		goto err_ch_unlock;
	}

	/*
	 * If the port is still in a previous open, and in a state
	 * where we simply cannot safely keep going, wait until the
	 * state clears.
	 */
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	rc = wait_event_interruptible(ch->ch_flags_wait,
				      ((ch->ch_flags & CH_OPENING) == 0));
	/* If ret is non-zero, user ctrl-c'ed us */
	if (rc)
		return -EINTR;

	/*
	 * If either unit is in the middle of the fragile part of close,
	 * we just cannot touch the channel safely.
	 * Go to sleep, knowing that when the channel can be
	 * touched safely, the close routine will signal the
	 * ch_flags_wait to wake us back up.
	 */
	rc = wait_event_interruptible(
				ch->ch_flags_wait,
				(((ch->ch_tun.un_flags |
				ch->ch_pun.un_flags) & UN_CLOSING) == 0));
	/* If ret is non-zero, user ctrl-c'ed us */
	if (rc)
		return -EINTR;

	spin_lock_irqsave(&ch->ch_lock, flags);

	tty->driver_data = un;

	/* Initialize tty's */

	if (!(un->un_flags & UN_ISOPEN)) {
		un->un_tty = tty;

		/* Maybe do something here to the TTY struct as well? */
	}

	/*
	 * Allocate channel buffers for read/write/error.
	 * Set flag, so we don't get trounced on.
	 */
	ch->ch_flags |= (CH_OPENING);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	if (!ch->ch_rqueue)
		ch->ch_rqueue = kzalloc(RQUEUESIZE, GFP_KERNEL);
	if (!ch->ch_equeue)
		ch->ch_equeue = kzalloc(EQUEUESIZE, GFP_KERNEL);
	if (!ch->ch_wqueue)
		ch->ch_wqueue = kzalloc(WQUEUESIZE, GFP_KERNEL);

	if (!ch->ch_rqueue || !ch->ch_equeue || !ch->ch_wqueue) {
		kfree(ch->ch_rqueue);
		kfree(ch->ch_equeue);
		kfree(ch->ch_wqueue);
		return -ENOMEM;
	}

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags &= ~(CH_OPENING);
	wake_up_interruptible(&ch->ch_flags_wait);

	/* Initialize if neither terminal or printer is open. */

	if (!((ch->ch_tun.un_flags | ch->ch_pun.un_flags) & UN_ISOPEN)) {
		/* Flush input queues. */
		ch->ch_r_head = 0;
		ch->ch_r_tail = 0;
		ch->ch_e_head = 0;
		ch->ch_e_tail = 0;
		ch->ch_w_head = 0;
		ch->ch_w_tail = 0;

		brd->bd_ops->flush_uart_write(ch);
		brd->bd_ops->flush_uart_read(ch);

		ch->ch_flags = 0;
		ch->ch_cached_lsr = 0;
		ch->ch_stop_sending_break = 0;
		ch->ch_stops_sent = 0;

		ch->ch_c_cflag   = tty->termios.c_cflag;
		ch->ch_c_iflag   = tty->termios.c_iflag;
		ch->ch_c_oflag   = tty->termios.c_oflag;
		ch->ch_c_lflag   = tty->termios.c_lflag;
		ch->ch_startc = tty->termios.c_cc[VSTART];
		ch->ch_stopc  = tty->termios.c_cc[VSTOP];

		/*
		 * Bring up RTS and DTR...
		 * Also handle RTS or DTR toggle if set.
		 */
		if (!(ch->ch_digi.digi_flags & DIGI_RTS_TOGGLE))
			ch->ch_mostat |= (UART_MCR_RTS);
		if (!(ch->ch_digi.digi_flags & DIGI_DTR_TOGGLE))
			ch->ch_mostat |= (UART_MCR_DTR);

		/* Tell UART to init itself */
		brd->bd_ops->uart_init(ch);
	}

	brd->bd_ops->param(tty);

	dgnc_carrier(ch);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	rc = dgnc_block_til_ready(tty, file, ch);

	spin_lock_irqsave(&ch->ch_lock, flags);
	ch->ch_open_count++;
	un->un_open_count++;
	un->un_flags |= (UN_ISOPEN);
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return rc;

err_brd_unlock:
	spin_unlock_irqrestore(&brd->bd_lock, flags);

	return rc;
err_ch_unlock:
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return rc;
}

/* Wait for DCD, if needed. */
static int dgnc_block_til_ready(struct tty_struct *tty,
				struct file *file,
				struct channel_t *ch)
{
	int rc = 0;
	struct un_t *un = tty->driver_data;
	unsigned long flags;
	uint	old_flags = 0;
	int	sleep_on_un_flags = 0;

	if (!file)
		return -ENXIO;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_wopen++;

	while (1) {
		sleep_on_un_flags = 0;

		if (ch->ch_bd->state == BOARD_FAILED) {
			rc = -ENXIO;
			break;
		}

		if (tty_hung_up_p(file)) {
			rc = -EAGAIN;
			break;
		}

		/*
		 * If either unit is in the middle of the fragile part of close,
		 * we just cannot touch the channel safely.
		 * Go back to sleep, knowing that when the channel can be
		 * touched safely, the close routine will signal the
		 * ch_wait_flags to wake us back up.
		 */
		if (!((ch->ch_tun.un_flags |
		    ch->ch_pun.un_flags) &
		    UN_CLOSING)) {
			/*
			 * Our conditions to leave cleanly and happily:
			 * 1) NONBLOCKING on the tty is set.
			 * 2) CLOCAL is set.
			 * 3) DCD (fake or real) is active.
			 */

			if (file->f_flags & O_NONBLOCK)
				break;

			if (tty_io_error(tty)) {
				rc = -EIO;
				break;
			}

			if (ch->ch_flags & CH_CD)
				break;

			if (ch->ch_flags & CH_FCAR)
				break;
		} else {
			sleep_on_un_flags = 1;
		}

		/*
		 * If there is a signal pending, the user probably
		 * interrupted (ctrl-c) us.
		 */
		if (signal_pending(current)) {
			rc = -ERESTARTSYS;
			break;
		}

		if (sleep_on_un_flags)
			old_flags = ch->ch_tun.un_flags | ch->ch_pun.un_flags;
		else
			old_flags = ch->ch_flags;

		/*
		 * Let go of channel lock before calling schedule.
		 * Our poller will get any FEP events and wake us up when DCD
		 * eventually goes active.
		 */

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		/*
		 * Wait for something in the flags to change
		 * from the current value.
		 */
		if (sleep_on_un_flags)
			rc = wait_event_interruptible
				(un->un_flags_wait,
				 (old_flags != (ch->ch_tun.un_flags |
						ch->ch_pun.un_flags)));
		else
			rc = wait_event_interruptible(
					ch->ch_flags_wait,
					(old_flags != ch->ch_flags));

		/*
		 * We got woken up for some reason.
		 * Before looping around, grab our channel lock.
		 */
		spin_lock_irqsave(&ch->ch_lock, flags);
	}

	ch->ch_wopen--;

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return rc;
}

/* Hangup the port.  Like a close, but don't wait for output to drain. */
static void dgnc_tty_hangup(struct tty_struct *tty)
{
	if (!tty)
		return;

	/* flush the transmit queues */
	dgnc_tty_flush_buffer(tty);
}

static void dgnc_tty_close(struct tty_struct *tty, struct file *file)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	/*
	 * Determine if this is the last close or not - and if we agree about
	 * which type of close it is with the Line Discipline
	 */
	if ((tty->count == 1) && (un->un_open_count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  un_open_count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		dev_dbg(tty->dev,
			"tty->count is 1, un open count is %d\n",
			un->un_open_count);
		un->un_open_count = 1;
	}

	if (un->un_open_count)
		un->un_open_count--;
	else
		dev_dbg(tty->dev,
			"bad serial port open count of %d\n",
			un->un_open_count);

	ch->ch_open_count--;

	if (ch->ch_open_count && un->un_open_count) {
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return;
	}

	/* OK, its the last close on the unit */
	un->un_flags |= UN_CLOSING;

	tty->closing = 1;

	/*
	 * Only officially close channel if count is 0 and
	 * DIGI_PRINTER bit is not set.
	 */
	if ((ch->ch_open_count == 0) &&
	    !(ch->ch_digi.digi_flags & DIGI_PRINTER)) {
		ch->ch_flags &= ~(CH_STOPI | CH_FORCED_STOPI);

		/* turn off print device when closing print device. */

		if ((un->un_type == DGNC_PRINT) && (ch->ch_flags & CH_PRON)) {
			dgnc_wmove(ch, ch->ch_digi.digi_offstr,
				   (int)ch->ch_digi.digi_offlen);
			ch->ch_flags &= ~CH_PRON;
		}

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		/* wait for output to drain */
		/* This will also return if we take an interrupt */

		bd->bd_ops->drain(tty, 0);

		dgnc_tty_flush_buffer(tty);
		tty_ldisc_flush(tty);

		spin_lock_irqsave(&ch->ch_lock, flags);

		tty->closing = 0;

		/* If we have HUPCL set, lower DTR and RTS */

		if (ch->ch_c_cflag & HUPCL) {
			/* Drop RTS/DTR */
			ch->ch_mostat &= ~(UART_MCR_DTR | UART_MCR_RTS);
			bd->bd_ops->assert_modem_signals(ch);

			/*
			 * Go to sleep to ensure RTS/DTR
			 * have been dropped for modems to see it.
			 */
			if (ch->ch_close_delay) {
				spin_unlock_irqrestore(&ch->ch_lock,
						       flags);
				msleep_interruptible(ch->ch_close_delay);
				spin_lock_irqsave(&ch->ch_lock, flags);
			}
		}

		ch->ch_old_baud = 0;

		/* Turn off UART interrupts for this port */
		ch->ch_bd->bd_ops->uart_off(ch);
	} else {
		/* turn off print device when closing print device. */

		if ((un->un_type == DGNC_PRINT) && (ch->ch_flags & CH_PRON)) {
			dgnc_wmove(ch, ch->ch_digi.digi_offstr,
				   (int)ch->ch_digi.digi_offlen);
			ch->ch_flags &= ~CH_PRON;
		}
	}

	un->un_tty = NULL;
	un->un_flags &= ~(UN_ISOPEN | UN_CLOSING);

	wake_up_interruptible(&ch->ch_flags_wait);
	wake_up_interruptible(&un->un_flags_wait);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

/*
 * Return number of characters that have not been transmitted yet.
 *
 * This routine is used by the line discipline to determine if there
 * is data waiting to be transmitted/drained/flushed or not.
 */
static int dgnc_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct channel_t *ch = NULL;
	struct un_t *un = NULL;
	ushort thead;
	ushort ttail;
	uint tmask;
	uint chars;
	unsigned long flags;

	if (!tty)
		return 0;

	un = tty->driver_data;
	if (!un)
		return 0;

	ch = un->un_ch;
	if (!ch)
		return 0;

	spin_lock_irqsave(&ch->ch_lock, flags);

	tmask = WQUEUEMASK;
	thead = ch->ch_w_head & tmask;
	ttail = ch->ch_w_tail & tmask;

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	if (ttail == thead)
		chars = 0;
	else if (thead > ttail)
		chars = thead - ttail;
	else
		chars = thead - ttail + WQUEUESIZE;

	return chars;
}

/*
 * Reduces bytes_available to the max number of characters
 * that can be sent currently given the maxcps value, and
 * returns the new bytes_available.  This only affects printer
 * output.
 */
static int dgnc_maxcps_room(struct channel_t *ch, int bytes_available)
{
	int rc = bytes_available;

	if (ch->ch_digi.digi_maxcps > 0 && ch->ch_digi.digi_bufsize > 0) {
		int cps_limit = 0;
		unsigned long current_time = jiffies;
		unsigned long buffer_time = current_time +
			(HZ * ch->ch_digi.digi_bufsize) /
			ch->ch_digi.digi_maxcps;

		if (ch->ch_cpstime < current_time) {
			/* buffer is empty */
			ch->ch_cpstime = current_time;	/* reset ch_cpstime */
			cps_limit = ch->ch_digi.digi_bufsize;
		} else if (ch->ch_cpstime < buffer_time) {
			/* still room in the buffer */
			cps_limit = ((buffer_time - ch->ch_cpstime) *
					ch->ch_digi.digi_maxcps) / HZ;
		} else {
			/* no room in the buffer */
			cps_limit = 0;
		}

		rc = min(cps_limit, bytes_available);
	}

	return rc;
}

/* Return room available in Tx buffer */
static int dgnc_tty_write_room(struct tty_struct *tty)
{
	struct channel_t *ch = NULL;
	struct un_t *un = NULL;
	ushort head;
	ushort tail;
	ushort tmask;
	int room = 0;
	unsigned long flags;

	if (!tty)
		return 0;

	un = tty->driver_data;
	if (!un)
		return 0;

	ch = un->un_ch;
	if (!ch)
		return 0;

	spin_lock_irqsave(&ch->ch_lock, flags);

	tmask = WQUEUEMASK;
	head = (ch->ch_w_head) & tmask;
	tail = (ch->ch_w_tail) & tmask;

	room = tail - head - 1;
	if (room < 0)
		room += WQUEUESIZE;

	/* Limit printer to maxcps */
	if (un->un_type != DGNC_PRINT)
		room = dgnc_maxcps_room(ch, room);

	/*
	 * If we are printer device, leave room for
	 * possibly both the on and off strings.
	 */
	if (un->un_type == DGNC_PRINT) {
		if (!(ch->ch_flags & CH_PRON))
			room -= ch->ch_digi.digi_onlen;
		room -= ch->ch_digi.digi_offlen;
	} else {
		if (ch->ch_flags & CH_PRON)
			room -= ch->ch_digi.digi_offlen;
	}

	if (room < 0)
		room = 0;

	spin_unlock_irqrestore(&ch->ch_lock, flags);
	return room;
}

/*
 * Put a character into ch->ch_buf
 * Used by the line discipline for OPOST processing
 */
static int dgnc_tty_put_char(struct tty_struct *tty, unsigned char c)
{
	dgnc_tty_write(tty, &c, 1);
	return 1;
}

/*
 * Take data from the user or kernel and send it out to the FEP.
 * In here exists all the Transparent Print magic as well.
 */
static int dgnc_tty_write(struct tty_struct *tty,
			  const unsigned char *buf, int count)
{
	struct channel_t *ch = NULL;
	struct un_t *un = NULL;
	int bufcount = 0, n = 0;
	unsigned long flags;
	ushort head;
	ushort tail;
	ushort tmask;
	uint remain;

	if (!tty)
		return 0;

	un = tty->driver_data;
	if (!un)
		return 0;

	ch = un->un_ch;
	if (!ch)
		return 0;

	if (!count)
		return 0;

	/*
	 * Store original amount of characters passed in.
	 * This helps to figure out if we should ask the FEP
	 * to send us an event when it has more space available.
	 */

	spin_lock_irqsave(&ch->ch_lock, flags);

	tmask = WQUEUEMASK;
	head = (ch->ch_w_head) & tmask;
	tail = (ch->ch_w_tail) & tmask;

	bufcount = tail - head - 1;
	if (bufcount < 0)
		bufcount += WQUEUESIZE;

	/*
	 * Limit printer output to maxcps overall, with bursts allowed
	 * up to bufsize characters.
	 */
	if (un->un_type != DGNC_PRINT)
		bufcount = dgnc_maxcps_room(ch, bufcount);

	count = min(count, bufcount);
	if (count <= 0)
		goto exit_retry;

	/*
	 * Output the printer ON string, if we are in terminal mode, but
	 * need to be in printer mode.
	 */
	if ((un->un_type == DGNC_PRINT) && !(ch->ch_flags & CH_PRON)) {
		dgnc_wmove(ch, ch->ch_digi.digi_onstr,
			   (int)ch->ch_digi.digi_onlen);
		head = (ch->ch_w_head) & tmask;
		ch->ch_flags |= CH_PRON;
	}

	/*
	 * On the other hand, output the printer OFF string, if we are
	 * currently in printer mode, but need to output to the terminal.
	 */
	if ((un->un_type != DGNC_PRINT) && (ch->ch_flags & CH_PRON)) {
		dgnc_wmove(ch, ch->ch_digi.digi_offstr,
			   (int)ch->ch_digi.digi_offlen);
		head = (ch->ch_w_head) & tmask;
		ch->ch_flags &= ~CH_PRON;
	}

	n = count;

	/*
	 * If the write wraps over the top of the circular buffer,
	 * move the portion up to the wrap point, and reset the
	 * pointers to the bottom.
	 */
	remain = WQUEUESIZE - head;

	if (n >= remain) {
		n -= remain;
		memcpy(ch->ch_wqueue + head, buf, remain);
		head = 0;
		buf += remain;
	}

	if (n > 0) {
		/* Move rest of data. */
		remain = n;
		memcpy(ch->ch_wqueue + head, buf, remain);
		head += remain;
	}

	if (count) {
		head &= tmask;
		ch->ch_w_head = head;
	}

	/* Update printer buffer empty time. */
	if ((un->un_type == DGNC_PRINT) && (ch->ch_digi.digi_maxcps > 0) &&
	    (ch->ch_digi.digi_bufsize > 0)) {
		ch->ch_cpstime += (HZ * count) / ch->ch_digi.digi_maxcps;
	}

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	if (count)
		ch->ch_bd->bd_ops->copy_data_from_queue_to_uart(ch);

	return count;

exit_retry:
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return 0;
}

/* Return modem signals to ld. */
static int dgnc_tty_tiocmget(struct tty_struct *tty)
{
	struct channel_t *ch;
	struct un_t *un;
	int rc;
	unsigned char mstat = 0;
	unsigned long flags;

	if (!tty)
		return -EIO;

	un = tty->driver_data;
	if (!un)
		return -EIO;

	ch = un->un_ch;
	if (!ch)
		return -EIO;

	spin_lock_irqsave(&ch->ch_lock, flags);

	mstat = ch->ch_mostat | ch->ch_mistat;

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	rc = 0;

	if (mstat & UART_MCR_DTR)
		rc |= TIOCM_DTR;
	if (mstat & UART_MCR_RTS)
		rc |= TIOCM_RTS;
	if (mstat & UART_MSR_CTS)
		rc |= TIOCM_CTS;
	if (mstat & UART_MSR_DSR)
		rc |= TIOCM_DSR;
	if (mstat & UART_MSR_RI)
		rc |= TIOCM_RI;
	if (mstat & UART_MSR_DCD)
		rc |= TIOCM_CD;

	return rc;
}

/* Set modem signals, called by ld. */
static int dgnc_tty_tiocmset(struct tty_struct *tty,
			     unsigned int set, unsigned int clear)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return -EIO;

	un = tty->driver_data;
	if (!un)
		return -EIO;

	ch = un->un_ch;
	if (!ch)
		return -EIO;

	bd = ch->ch_bd;
	if (!bd)
		return -EIO;

	spin_lock_irqsave(&ch->ch_lock, flags);

	if (set & TIOCM_RTS)
		ch->ch_mostat |= UART_MCR_RTS;

	if (set & TIOCM_DTR)
		ch->ch_mostat |= UART_MCR_DTR;

	if (clear & TIOCM_RTS)
		ch->ch_mostat &= ~(UART_MCR_RTS);

	if (clear & TIOCM_DTR)
		ch->ch_mostat &= ~(UART_MCR_DTR);

	bd->bd_ops->assert_modem_signals(ch);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return 0;
}

/* Send a Break, called by ld. */
static int dgnc_tty_send_break(struct tty_struct *tty, int msec)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return -EIO;

	un = tty->driver_data;
	if (!un)
		return -EIO;

	ch = un->un_ch;
	if (!ch)
		return -EIO;

	bd = ch->ch_bd;
	if (!bd)
		return -EIO;

	if (msec < 0)
		msec = 0xFFFF;

	spin_lock_irqsave(&ch->ch_lock, flags);

	bd->bd_ops->send_break(ch, msec);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return 0;
}

/* wait until data has been transmitted, called by ld. */
static void dgnc_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	bd->bd_ops->drain(tty, 0);
}

/* send a high priority character, called by ld. */
static void dgnc_tty_send_xchar(struct tty_struct *tty, char c)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);
	bd->bd_ops->send_immediate_char(ch, c);
	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

/* Return modem signals to ld. */
static inline int dgnc_get_mstat(struct channel_t *ch)
{
	unsigned char mstat;
	unsigned long flags;
	int rc;

	if (!ch)
		return -ENXIO;

	spin_lock_irqsave(&ch->ch_lock, flags);

	mstat = ch->ch_mostat | ch->ch_mistat;

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	rc = 0;

	if (mstat & UART_MCR_DTR)
		rc |= TIOCM_DTR;
	if (mstat & UART_MCR_RTS)
		rc |= TIOCM_RTS;
	if (mstat & UART_MSR_CTS)
		rc |= TIOCM_CTS;
	if (mstat & UART_MSR_DSR)
		rc |= TIOCM_DSR;
	if (mstat & UART_MSR_RI)
		rc |= TIOCM_RI;
	if (mstat & UART_MSR_DCD)
		rc |= TIOCM_CD;

	return rc;
}

/* Return modem signals to ld. */
static int dgnc_get_modem_info(struct channel_t *ch,
			       unsigned int  __user *value)
{
	return put_user(dgnc_get_mstat(ch), value);
}

/* Set modem signals, called by ld. */
static int dgnc_set_modem_info(struct channel_t *ch,
			       unsigned int command,
			       unsigned int __user *value)
{
	int rc;
	unsigned int arg = 0;
	unsigned long flags;

	rc = get_user(arg, value);
	if (rc)
		return rc;

	switch (command) {
	case TIOCMBIS:
		if (arg & TIOCM_RTS)
			ch->ch_mostat |= UART_MCR_RTS;

		if (arg & TIOCM_DTR)
			ch->ch_mostat |= UART_MCR_DTR;

		break;

	case TIOCMBIC:
		if (arg & TIOCM_RTS)
			ch->ch_mostat &= ~(UART_MCR_RTS);

		if (arg & TIOCM_DTR)
			ch->ch_mostat &= ~(UART_MCR_DTR);

		break;

	case TIOCMSET:

		if (arg & TIOCM_RTS)
			ch->ch_mostat |= UART_MCR_RTS;
		else
			ch->ch_mostat &= ~(UART_MCR_RTS);

		if (arg & TIOCM_DTR)
			ch->ch_mostat |= UART_MCR_DTR;
		else
			ch->ch_mostat &= ~(UART_MCR_DTR);

		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_bd->bd_ops->assert_modem_signals(ch);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return 0;
}

/* Ioctl to get the information for ditty. */
static int dgnc_tty_digigeta(struct tty_struct *tty,
			     struct digi_t __user *retinfo)
{
	struct channel_t *ch;
	struct un_t *un;
	struct digi_t tmp;
	unsigned long flags;

	if (!retinfo)
		return -EFAULT;

	if (!tty)
		return -EFAULT;

	un = tty->driver_data;
	if (!un)
		return -EFAULT;

	ch = un->un_ch;
	if (!ch)
		return -EFAULT;

	memset(&tmp, 0, sizeof(tmp));

	spin_lock_irqsave(&ch->ch_lock, flags);
	memcpy(&tmp, &ch->ch_digi, sizeof(tmp));
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	if (copy_to_user(retinfo, &tmp, sizeof(*retinfo)))
		return -EFAULT;

	return 0;
}

/* Ioctl to set the information for ditty. */
static int dgnc_tty_digiseta(struct tty_struct *tty,
			     struct digi_t __user *new_info)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	struct digi_t new_digi;
	unsigned long flags;

	if (!tty)
		return -EFAULT;

	un = tty->driver_data;
	if (!un)
		return -EFAULT;

	ch = un->un_ch;
	if (!ch)
		return -EFAULT;

	bd = ch->ch_bd;
	if (!bd)
		return -EFAULT;

	if (copy_from_user(&new_digi, new_info, sizeof(new_digi)))
		return -EFAULT;

	spin_lock_irqsave(&ch->ch_lock, flags);

	/* Handle transitions to and from RTS Toggle. */

	if (!(ch->ch_digi.digi_flags & DIGI_RTS_TOGGLE) &&
	    (new_digi.digi_flags & DIGI_RTS_TOGGLE))
		ch->ch_mostat &= ~(UART_MCR_RTS);
	if ((ch->ch_digi.digi_flags & DIGI_RTS_TOGGLE) &&
	    !(new_digi.digi_flags & DIGI_RTS_TOGGLE))
		ch->ch_mostat |= (UART_MCR_RTS);

	/* Handle transitions to and from DTR Toggle. */

	if (!(ch->ch_digi.digi_flags & DIGI_DTR_TOGGLE) &&
	    (new_digi.digi_flags & DIGI_DTR_TOGGLE))
		ch->ch_mostat &= ~(UART_MCR_DTR);
	if ((ch->ch_digi.digi_flags & DIGI_DTR_TOGGLE) &&
	    !(new_digi.digi_flags & DIGI_DTR_TOGGLE))
		ch->ch_mostat |= (UART_MCR_DTR);

	memcpy(&ch->ch_digi, &new_digi, sizeof(new_digi));

	if (ch->ch_digi.digi_maxcps < 1)
		ch->ch_digi.digi_maxcps = 1;

	if (ch->ch_digi.digi_maxcps > 10000)
		ch->ch_digi.digi_maxcps = 10000;

	if (ch->ch_digi.digi_bufsize < 10)
		ch->ch_digi.digi_bufsize = 10;

	if (ch->ch_digi.digi_maxchar < 1)
		ch->ch_digi.digi_maxchar = 1;

	if (ch->ch_digi.digi_maxchar > ch->ch_digi.digi_bufsize)
		ch->ch_digi.digi_maxchar = ch->ch_digi.digi_bufsize;

	if (ch->ch_digi.digi_onlen > DIGI_PLEN)
		ch->ch_digi.digi_onlen = DIGI_PLEN;

	if (ch->ch_digi.digi_offlen > DIGI_PLEN)
		ch->ch_digi.digi_offlen = DIGI_PLEN;

	bd->bd_ops->param(tty);

	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return 0;
}

static void dgnc_tty_set_termios(struct tty_struct *tty,
				 struct ktermios *old_termios)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_c_cflag   = tty->termios.c_cflag;
	ch->ch_c_iflag   = tty->termios.c_iflag;
	ch->ch_c_oflag   = tty->termios.c_oflag;
	ch->ch_c_lflag   = tty->termios.c_lflag;
	ch->ch_startc = tty->termios.c_cc[VSTART];
	ch->ch_stopc  = tty->termios.c_cc[VSTOP];

	bd->bd_ops->param(tty);
	dgnc_carrier(ch);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

static void dgnc_tty_throttle(struct tty_struct *tty)
{
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags |= (CH_FORCED_STOPI);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

static void dgnc_tty_unthrottle(struct tty_struct *tty)
{
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags &= ~(CH_FORCED_STOPI);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

static void dgnc_tty_start(struct tty_struct *tty)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags &= ~(CH_FORCED_STOP);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

static void dgnc_tty_stop(struct tty_struct *tty)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags |= (CH_FORCED_STOP);

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

/*
 * Flush the cook buffer
 *
 * Note to self, and any other poor souls who venture here:
 *
 * flush in this case DOES NOT mean dispose of the data.
 * instead, it means "stop buffering and send it if you
 * haven't already."  Just guess how I figured that out...   SRW 2-Jun-98
 *
 * It is also always called in interrupt context - JAR 8-Sept-99
 */
static void dgnc_tty_flush_chars(struct tty_struct *tty)
{
	struct dgnc_board *bd;
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	bd = ch->ch_bd;
	if (!bd)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	/* Do something maybe here */

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

/* Flush Tx buffer (make in == out) */
static void dgnc_tty_flush_buffer(struct tty_struct *tty)
{
	struct channel_t *ch;
	struct un_t *un;
	unsigned long flags;

	if (!tty)
		return;

	un = tty->driver_data;
	if (!un)
		return;

	ch = un->un_ch;
	if (!ch)
		return;

	spin_lock_irqsave(&ch->ch_lock, flags);

	ch->ch_flags &= ~CH_STOP;

	/* Flush our write queue */
	ch->ch_w_head = ch->ch_w_tail;

	/* Flush UARTs transmit FIFO */
	ch->ch_bd->bd_ops->flush_uart_write(ch);

	if (ch->ch_tun.un_flags & (UN_LOW | UN_EMPTY)) {
		ch->ch_tun.un_flags &= ~(UN_LOW | UN_EMPTY);
		wake_up_interruptible(&ch->ch_tun.un_flags_wait);
	}
	if (ch->ch_pun.un_flags & (UN_LOW | UN_EMPTY)) {
		ch->ch_pun.un_flags &= ~(UN_LOW | UN_EMPTY);
		wake_up_interruptible(&ch->ch_pun.un_flags_wait);
	}

	spin_unlock_irqrestore(&ch->ch_lock, flags);
}

/* Wakes up processes waiting in the unit's (teminal/printer) wait queue */
static void dgnc_wake_up_unit(struct un_t *unit)
{
	unit->un_flags &= ~(UN_LOW | UN_EMPTY);
	wake_up_interruptible(&unit->un_flags_wait);
}

/* The IOCTL function and all of its helpers */

/* The usual assortment of ioctl's */
static int dgnc_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
			  unsigned long arg)
{
	struct dgnc_board *bd;
	struct board_ops *ch_bd_ops;
	struct channel_t *ch;
	struct un_t *un;
	int rc;
	unsigned long flags;
	void __user *uarg = (void __user *)arg;

	if (!tty)
		return -ENODEV;

	un = tty->driver_data;
	if (!un)
		return -ENODEV;

	ch = un->un_ch;
	if (!ch)
		return -ENODEV;

	bd = ch->ch_bd;
	if (!bd)
		return -ENODEV;

	ch_bd_ops = bd->bd_ops;

	spin_lock_irqsave(&ch->ch_lock, flags);

	if (un->un_open_count <= 0) {
		rc = -EIO;
		goto err_unlock;
	}

	switch (cmd) {
	/* Here are all the standard ioctl's that we MUST implement */

	case TCSBRK:
		/*
		 * TCSBRK is SVID version: non-zero arg --> no break
		 * this behaviour is exploited by tcdrain().
		 *
		 * According to POSIX.1 spec (7.2.2.1.2) breaks should be
		 * between 0.25 and 0.5 seconds so we'll ask for something
		 * in the middle: 0.375 seconds.
		 */
		rc = tty_check_change(tty);
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		if (rc)
			return rc;

		rc = ch_bd_ops->drain(tty, 0);
		if (rc)
			return -EINTR;

		spin_lock_irqsave(&ch->ch_lock, flags);

		if (((cmd == TCSBRK) && (!arg)) || (cmd == TCSBRKP))
			ch_bd_ops->send_break(ch, 250);

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return 0;

	case TCSBRKP:
		/*
		 * support for POSIX tcsendbreak()
		 * According to POSIX.1 spec (7.2.2.1.2) breaks should be
		 * between 0.25 and 0.5 seconds so we'll ask for something
		 * in the middle: 0.375 seconds.
		 */
		rc = tty_check_change(tty);
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		if (rc)
			return rc;

		rc = ch_bd_ops->drain(tty, 0);
		if (rc)
			return -EINTR;

		spin_lock_irqsave(&ch->ch_lock, flags);

		ch_bd_ops->send_break(ch, 250);

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return 0;

	case TIOCSBRK:
		rc = tty_check_change(tty);
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		if (rc)
			return rc;

		rc = ch_bd_ops->drain(tty, 0);
		if (rc)
			return -EINTR;

		spin_lock_irqsave(&ch->ch_lock, flags);

		ch_bd_ops->send_break(ch, 250);

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return 0;

	case TIOCCBRK:
		/* Do Nothing */
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return 0;

	case TIOCGSOFTCAR:

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return put_user(C_CLOCAL(tty) ? 1 : 0,
				(unsigned long __user *)arg);

	case TIOCSSOFTCAR:

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		rc = get_user(arg, (unsigned long __user *)arg);
		if (rc)
			return rc;

		spin_lock_irqsave(&ch->ch_lock, flags);
		tty->termios.c_cflag = ((tty->termios.c_cflag & ~CLOCAL) |
				       (arg ? CLOCAL : 0));
		ch_bd_ops->param(tty);
		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return 0;

	case TIOCMGET:
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return dgnc_get_modem_info(ch, uarg);

	case TIOCMBIS:
	case TIOCMBIC:
	case TIOCMSET:
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return dgnc_set_modem_info(ch, cmd, uarg);

		/* Here are any additional ioctl's that we want to implement */

	case TCFLSH:
		/*
		 * The linux tty driver doesn't have a flush
		 * input routine for the driver, assuming all backed
		 * up data is in the line disc. buffers.  However,
		 * we all know that's not the case.  Here, we
		 * act on the ioctl, but then lie and say we didn't
		 * so the line discipline will process the flush
		 * also.
		 */
		rc = tty_check_change(tty);
		if (rc)
			goto err_unlock;

		if ((arg == TCIFLUSH) || (arg == TCIOFLUSH)) {
			ch->ch_r_head = ch->ch_r_tail;
			ch_bd_ops->flush_uart_read(ch);
			/* Force queue flow control to be released, if needed */
			dgnc_check_queue_flow_control(ch);
		}

		if ((arg == TCOFLUSH) || (arg == TCIOFLUSH)) {
			if (!(un->un_type == DGNC_PRINT)) {
				ch->ch_w_head = ch->ch_w_tail;
				ch_bd_ops->flush_uart_write(ch);

				if (ch->ch_tun.un_flags & (UN_LOW | UN_EMPTY))
					dgnc_wake_up_unit(&ch->ch_tun);

				if (ch->ch_pun.un_flags & (UN_LOW | UN_EMPTY))
					dgnc_wake_up_unit(&ch->ch_pun);
			}
		}

		/* pretend we didn't recognize this IOCTL */
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return -ENOIOCTLCMD;
	case TCSETSF:
	case TCSETSW:
		/*
		 * The linux tty driver doesn't have a flush
		 * input routine for the driver, assuming all backed
		 * up data is in the line disc. buffers.  However,
		 * we all know that's not the case.  Here, we
		 * act on the ioctl, but then lie and say we didn't
		 * so the line discipline will process the flush
		 * also.
		 */
		if (cmd == TCSETSF) {
			/* flush rx */
			ch->ch_flags &= ~CH_STOP;
			ch->ch_r_head = ch->ch_r_tail;
			ch_bd_ops->flush_uart_read(ch);
			/* Force queue flow control to be released, if needed */
			dgnc_check_queue_flow_control(ch);
		}

		/* now wait for all the output to drain */
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		rc = ch_bd_ops->drain(tty, 0);
		if (rc)
			return -EINTR;

		/* pretend we didn't recognize this */
		return -ENOIOCTLCMD;

	case TCSETAW:

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		rc = ch_bd_ops->drain(tty, 0);
		if (rc)
			return -EINTR;

		/* pretend we didn't recognize this */
		return -ENOIOCTLCMD;

	case TCXONC:
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		/* Make the ld do it */
		return -ENOIOCTLCMD;

	case DIGI_GETA:
		/* get information for ditty */
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return dgnc_tty_digigeta(tty, uarg);

	case DIGI_SETAW:
	case DIGI_SETAF:

		/* set information for ditty */
		if (cmd == (DIGI_SETAW)) {
			spin_unlock_irqrestore(&ch->ch_lock, flags);
			rc = ch_bd_ops->drain(tty, 0);
			if (rc)
				return -EINTR;

			spin_lock_irqsave(&ch->ch_lock, flags);
		} else {
			tty_ldisc_flush(tty);
		}
		/* fall thru */

	case DIGI_SETA:
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return dgnc_tty_digiseta(tty, uarg);

	case DIGI_LOOPBACK:
		{
			uint loopback = 0;
			/*
			 * Let go of locks when accessing user space,
			 * could sleep
			 */
			spin_unlock_irqrestore(&ch->ch_lock, flags);
			rc = get_user(loopback, (unsigned int __user *)arg);
			if (rc)
				return rc;
			spin_lock_irqsave(&ch->ch_lock, flags);

			/* Enable/disable internal loopback for this port */
			if (loopback)
				ch->ch_flags |= CH_LOOPBACK;
			else
				ch->ch_flags &= ~(CH_LOOPBACK);

			ch_bd_ops->param(tty);
			spin_unlock_irqrestore(&ch->ch_lock, flags);
			return 0;
		}

	case DIGI_GETCUSTOMBAUD:
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return put_user(ch->ch_custom_speed,
				(unsigned int __user *)arg);

	case DIGI_SETCUSTOMBAUD:
	{
		int new_rate;

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		rc = get_user(new_rate, (int __user *)arg);
		if (rc)
			return rc;
		spin_lock_irqsave(&ch->ch_lock, flags);
		dgnc_set_custom_speed(ch, new_rate);
		ch_bd_ops->param(tty);
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return 0;
	}

	/*
	 * This ioctl allows insertion of a character into the front
	 * of any pending data to be transmitted.
	 *
	 * This ioctl is to satisfy the "Send Character Immediate"
	 * call that the RealPort protocol spec requires.
	 */
	case DIGI_REALPORT_SENDIMMEDIATE:
	{
		unsigned char c;

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		rc = get_user(c, (unsigned char __user *)arg);
		if (rc)
			return rc;
		spin_lock_irqsave(&ch->ch_lock, flags);
		ch_bd_ops->send_immediate_char(ch, c);
		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return 0;
	}

	/*
	 * This ioctl returns all the current counts for the port.
	 *
	 * This ioctl is to satisfy the "Line Error Counters"
	 * call that the RealPort protocol spec requires.
	 */
	case DIGI_REALPORT_GETCOUNTERS:
	{
		struct digi_getcounter buf;

		buf.norun = ch->ch_err_overrun;
		buf.noflow = 0;		/* The driver doesn't keep this stat */
		buf.nframe = ch->ch_err_frame;
		buf.nparity = ch->ch_err_parity;
		buf.nbreak = ch->ch_err_break;
		buf.rbytes = ch->ch_rxcount;
		buf.tbytes = ch->ch_txcount;

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		if (copy_to_user(uarg, &buf, sizeof(buf)))
			return -EFAULT;

		return 0;
	}

	/*
	 * This ioctl returns all current events.
	 *
	 * This ioctl is to satisfy the "Event Reporting"
	 * call that the RealPort protocol spec requires.
	 */
	case DIGI_REALPORT_GETEVENTS:
	{
		unsigned int events = 0;

		/* NOTE: MORE EVENTS NEEDS TO BE ADDED HERE */
		if (ch->ch_flags & CH_BREAK_SENDING)
			events |= EV_TXB;
		if ((ch->ch_flags & CH_STOP) ||
		    (ch->ch_flags & CH_FORCED_STOP))
			events |= (EV_OPU | EV_OPS);

		if ((ch->ch_flags & CH_STOPI) ||
		    (ch->ch_flags & CH_FORCED_STOPI))
			events |= (EV_IPU | EV_IPS);

		spin_unlock_irqrestore(&ch->ch_lock, flags);
		return put_user(events, (unsigned int __user *)arg);
	}

	/*
	 * This ioctl returns TOUT and TIN counters based
	 * upon the values passed in by the RealPort Server.
	 * It also passes back whether the UART Transmitter is
	 * empty as well.
	 */
	case DIGI_REALPORT_GETBUFFERS:
	{
		struct digi_getbuffer buf;
		int tdist;
		int count;

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		if (copy_from_user(&buf, uarg, sizeof(buf)))
			return -EFAULT;

		spin_lock_irqsave(&ch->ch_lock, flags);

		/* Figure out how much data is in our RX and TX queues. */

		buf.rxbuf = (ch->ch_r_head - ch->ch_r_tail) & RQUEUEMASK;
		buf.txbuf = (ch->ch_w_head - ch->ch_w_tail) & WQUEUEMASK;

		/*
		 * Is the UART empty?
		 * Add that value to whats in our TX queue.
		 */

		count = buf.txbuf + ch_bd_ops->get_uart_bytes_left(ch);

		/*
		 * Figure out how much data the RealPort Server believes should
		 * be in our TX queue.
		 */
		tdist = (buf.tx_in - buf.tx_out) & 0xffff;

		/*
		 * If we have more data than the RealPort Server believes we
		 * should have, reduce our count to its amount.
		 *
		 * This count difference CAN happen because the Linux LD can
		 * insert more characters into our queue for OPOST processing
		 * that the RealPort Server doesn't know about.
		 */
		if (buf.txbuf > tdist)
			buf.txbuf = tdist;

		/* Report whether our queue and UART TX are completely empty. */

		if (count)
			buf.txdone = 0;
		else
			buf.txdone = 1;

		spin_unlock_irqrestore(&ch->ch_lock, flags);

		if (copy_to_user(uarg, &buf, sizeof(buf)))
			return -EFAULT;

		return 0;
	}
	default:
		spin_unlock_irqrestore(&ch->ch_lock, flags);

		return -ENOIOCTLCMD;
	}
err_unlock:
	spin_unlock_irqrestore(&ch->ch_lock, flags);

	return rc;
}
