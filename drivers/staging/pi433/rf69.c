/*
 * abstraction of the spi interface of HopeRf rf69 radio module
 *
 * Copyright (C) 2016 Wolf-Entwicklungen
 *	Marcus Wolf <linux@wolf-entwicklungen.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* enable prosa debug info */
#undef DEBUG
/* enable print of values on reg access */
#undef DEBUG_VALUES
/* enable print of values on fifo access */
#undef DEBUG_FIFO_ACCESS

#include <linux/types.h>
#include <linux/spi/spi.h>

#include "rf69.h"
#include "rf69_registers.h"

#define F_OSC	  32000000 /* in Hz */
#define FIFO_SIZE 66	   /* in byte */

/*-------------------------------------------------------------------------*/

static u8 rf69_read_reg(struct spi_device *spi, u8 addr)
{
	int retval;

	retval = spi_w8r8(spi, addr);

#ifdef DEBUG_VALUES
	if (retval < 0)
		/* should never happen, since we already checked,
		 * that module is connected. Therefore no error
		 * handling, just an optional error message...
		 */
		dev_dbg(&spi->dev, "read 0x%x FAILED\n", addr);
	else
		dev_dbg(&spi->dev, "read 0x%x from reg 0x%x\n", retval, addr);
#endif

	return retval;
}

static int rf69_write_reg(struct spi_device *spi, u8 addr, u8 value)
{
	int retval;
	char buffer[2];

	buffer[0] = addr | WRITE_BIT;
	buffer[1] = value;

	retval = spi_write(spi, &buffer, 2);

#ifdef DEBUG_VALUES
	if (retval < 0)
		/* should never happen, since we already checked,
		 * that module is connected. Therefore no error
		 * handling, just an optional error message...
		 */
		dev_dbg(&spi->dev, "write 0x%x to 0x%x FAILED\n", value, addr);
	else
		dev_dbg(&spi->dev, "wrote 0x%x to reg 0x%x\n", value, addr);
#endif

	return retval;
}

/*-------------------------------------------------------------------------*/

static int rf69_set_bit(struct spi_device *spi, u8 reg, u8 mask)
{
	u8 tmp;

	tmp = rf69_read_reg(spi, reg);
	tmp = tmp | mask;
	return rf69_write_reg(spi, reg, tmp);
}

static int rf69_clear_bit(struct spi_device *spi, u8 reg, u8 mask)
{
	u8 tmp;

	tmp = rf69_read_reg(spi, reg);
	tmp = tmp & ~mask;
	return rf69_write_reg(spi, reg, tmp);
}

static inline int rf69_read_mod_write(struct spi_device *spi, u8 reg, u8 mask, u8 value)
{
	u8 tmp;

	tmp = rf69_read_reg(spi, reg);
	tmp = (tmp & ~mask) | value;
	return rf69_write_reg(spi, reg, tmp);
}

/*-------------------------------------------------------------------------*/

int rf69_set_mode(struct spi_device *spi, enum mode mode)
{
	switch (mode) {
	case transmit:
		return rf69_read_mod_write(spi, REG_OPMODE, MASK_OPMODE_MODE, OPMODE_MODE_TRANSMIT);
	case receive:
		return rf69_read_mod_write(spi, REG_OPMODE, MASK_OPMODE_MODE, OPMODE_MODE_RECEIVE);
	case synthesizer:
		return rf69_read_mod_write(spi, REG_OPMODE, MASK_OPMODE_MODE, OPMODE_MODE_SYNTHESIZER);
	case standby:
		return rf69_read_mod_write(spi, REG_OPMODE, MASK_OPMODE_MODE, OPMODE_MODE_STANDBY);
	case mode_sleep:
		return rf69_read_mod_write(spi, REG_OPMODE, MASK_OPMODE_MODE, OPMODE_MODE_SLEEP);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	// we are using packet mode, so this check is not really needed
	// but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
	//while (_mode == RF69_MODE_SLEEP && (READ_REG(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
}

int rf69_set_data_mode(struct spi_device *spi, u8 data_mode)
{
	return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODE, data_mode);
}

int rf69_set_modulation(struct spi_device *spi, enum modulation modulation)
{
	switch (modulation) {
	case OOK:
		return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_TYPE, DATAMODUL_MODULATION_TYPE_OOK);
	case FSK:
		return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_TYPE, DATAMODUL_MODULATION_TYPE_FSK);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

static enum modulation rf69_get_modulation(struct spi_device *spi)
{
	u8 currentValue;

	currentValue = rf69_read_reg(spi, REG_DATAMODUL);

	switch (currentValue & MASK_DATAMODUL_MODULATION_TYPE) {
	case DATAMODUL_MODULATION_TYPE_OOK:
		return OOK;
	case DATAMODUL_MODULATION_TYPE_FSK:
		return FSK;
	default:
		return UNDEF;
	}
}

int rf69_set_modulation_shaping(struct spi_device *spi,
				enum mod_shaping mod_shaping)
{
	switch (rf69_get_modulation(spi)) {
	case FSK:
		switch (mod_shaping) {
		case SHAPING_OFF:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_NONE);
		case SHAPING_1_0:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_1_0);
		case SHAPING_0_5:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_0_5);
		case SHAPING_0_3:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_0_3);
		default:
			dev_dbg(&spi->dev, "set: illegal input param");
			return -EINVAL;
		}
	case OOK:
		switch (mod_shaping) {
		case SHAPING_OFF:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_NONE);
		case SHAPING_BR:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_BR);
		case SHAPING_2BR:
			return rf69_read_mod_write(spi, REG_DATAMODUL, MASK_DATAMODUL_MODULATION_SHAPE, DATAMODUL_MODULATION_SHAPE_2BR);
		default:
			dev_dbg(&spi->dev, "set: illegal input param");
			return -EINVAL;
		}
	default:
		dev_dbg(&spi->dev, "set: modulation undefined");
		return -EINVAL;
	}
}

int rf69_set_bit_rate(struct spi_device *spi, u16 bitRate)
{
	int retval;
	u32 bitRate_min;
	u32 bitRate_reg;
	u8 msb;
	u8 lsb;

	// check input value
	bitRate_min = F_OSC / 8388608; // 8388608 = 2^23;
	if (bitRate < bitRate_min) {
		dev_dbg(&spi->dev, "setBitRate: illegal input param");
		return -EINVAL;
	}

	// calculate reg settings
	bitRate_reg = (F_OSC / bitRate);

	msb = (bitRate_reg & 0xff00) >> 8;
	lsb = (bitRate_reg & 0xff);

	// transmit to RF 69
	retval = rf69_write_reg(spi, REG_BITRATE_MSB, msb);
	if (retval)
		return retval;
	retval = rf69_write_reg(spi, REG_BITRATE_LSB, lsb);
	if (retval)
		return retval;

	return 0;
}

int rf69_set_deviation(struct spi_device *spi, u32 deviation)
{
	int retval;
	u64 f_reg;
	u64 f_step;
	u8 msb;
	u8 lsb;
	u64 factor = 1000000; // to improve precision of calculation

	// TODO: Dependency to bitrate
	if (deviation < 600 || deviation > 500000) {
		dev_dbg(&spi->dev, "set_deviation: illegal input param");
		return -EINVAL;
	}

	// calculat f step
	f_step = F_OSC * factor;
	do_div(f_step, 524288); //  524288 = 2^19

	// calculate register settings
	f_reg = deviation * factor;
	do_div(f_reg, f_step);

	msb = (f_reg & 0xff00) >> 8;
	lsb = (f_reg & 0xff);

	// check msb
	if (msb & ~FDEVMASB_MASK) {
		dev_dbg(&spi->dev, "set_deviation: err in calc of msb");
		return -EINVAL;
	}

	// write to chip
	retval = rf69_write_reg(spi, REG_FDEV_MSB, msb);
	if (retval)
		return retval;
	retval = rf69_write_reg(spi, REG_FDEV_LSB, lsb);
	if (retval)
		return retval;

	return 0;
}

int rf69_set_frequency(struct spi_device *spi, u32 frequency)
{
	int retval;
	u32 f_max;
	u64 f_reg;
	u64 f_step;
	u8 msb;
	u8 mid;
	u8 lsb;
	u64 factor = 1000000; // to improve precision of calculation

	// calculat f step
	f_step = F_OSC * factor;
	do_div(f_step, 524288); //  524288 = 2^19

	// check input value
	f_max = div_u64(f_step * 8388608, factor);
	if (frequency > f_max) {
		dev_dbg(&spi->dev, "setFrequency: illegal input param");
		return -EINVAL;
	}

	// calculate reg settings
	f_reg = frequency * factor;
	do_div(f_reg, f_step);

	msb = (f_reg & 0xff0000) >> 16;
	mid = (f_reg & 0xff00)   >>  8;
	lsb = (f_reg & 0xff);

	// write to chip
	retval = rf69_write_reg(spi, REG_FRF_MSB, msb);
	if (retval)
		return retval;
	retval = rf69_write_reg(spi, REG_FRF_MID, mid);
	if (retval)
		return retval;
	retval = rf69_write_reg(spi, REG_FRF_LSB, lsb);
	if (retval)
		return retval;

	return 0;
}

int rf69_enable_amplifier(struct spi_device *spi, u8 amplifier_mask)
{
	return rf69_set_bit(spi, REG_PALEVEL, amplifier_mask);
}

int rf69_disable_amplifier(struct spi_device *spi, u8 amplifier_mask)
{
	return rf69_clear_bit(spi, REG_PALEVEL, amplifier_mask);
}

int rf69_set_output_power_level(struct spi_device *spi, u8 powerLevel)
{
	// TODO: Dependency to PA0,1,2 setting
	powerLevel += 18;

	// check input value
	if (powerLevel > 0x1f) {
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	// write value
	return rf69_read_mod_write(spi, REG_PALEVEL, MASK_PALEVEL_OUTPUT_POWER, powerLevel);
}

int rf69_set_pa_ramp(struct spi_device *spi, enum paRamp paRamp)
{
	switch (paRamp) {
	case ramp3400:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_3400);
	case ramp2000:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_2000);
	case ramp1000:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_1000);
	case ramp500:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_500);
	case ramp250:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_250);
	case ramp125:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_125);
	case ramp100:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_100);
	case ramp62:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_62);
	case ramp50:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_50);
	case ramp40:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_40);
	case ramp31:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_31);
	case ramp25:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_25);
	case ramp20:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_20);
	case ramp15:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_15);
	case ramp12:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_12);
	case ramp10:
		return rf69_write_reg(spi, REG_PARAMP, PARAMP_10);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_antenna_impedance(struct spi_device *spi, enum antennaImpedance antennaImpedance)
{
	switch (antennaImpedance) {
	case fiftyOhm:
		return rf69_clear_bit(spi, REG_LNA, MASK_LNA_ZIN);
	case twohundretOhm:
		return rf69_set_bit(spi, REG_LNA, MASK_LNA_ZIN);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_lna_gain(struct spi_device *spi, enum lnaGain lnaGain)
{
	switch (lnaGain) {
	case automatic:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_AUTO);
	case max:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX);
	case max_minus_6:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX_MINUS_6);
	case max_minus_12:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX_MINUS_12);
	case max_minus_24:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX_MINUS_24);
	case max_minus_36:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX_MINUS_36);
	case max_minus_48:
		return rf69_read_mod_write(spi, REG_LNA, MASK_LNA_GAIN, LNA_GAIN_MAX_MINUS_48);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

static int rf69_set_bandwidth_intern(struct spi_device *spi, u8 reg,
				     enum mantisse mantisse, u8 exponent)
{
	u8 newValue;

	// check value for mantisse and exponent
	if (exponent > 7) {
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	if ((mantisse != mantisse16) &&
	    (mantisse != mantisse20) &&
	    (mantisse != mantisse24)) {
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	// read old value
	newValue = rf69_read_reg(spi, reg);

	// "delete" mantisse and exponent = just keep the DCC setting
	newValue = newValue & MASK_BW_DCC_FREQ;

	// add new mantisse
	switch (mantisse) {
	case mantisse16:
		newValue = newValue | BW_MANT_16;
		break;
	case mantisse20:
		newValue = newValue | BW_MANT_20;
		break;
	case mantisse24:
		newValue = newValue | BW_MANT_24;
		break;
	}

	// add new exponent
	newValue = newValue | exponent;

	// write back
	return rf69_write_reg(spi, reg, newValue);
}

int rf69_set_bandwidth(struct spi_device *spi, enum mantisse mantisse, u8 exponent)
{
	return rf69_set_bandwidth_intern(spi, REG_RXBW, mantisse, exponent);
}

int rf69_set_bandwidth_during_afc(struct spi_device *spi, enum mantisse mantisse, u8 exponent)
{
	return rf69_set_bandwidth_intern(spi, REG_AFCBW, mantisse, exponent);
}

int rf69_set_ook_threshold_dec(struct spi_device *spi, enum thresholdDecrement thresholdDecrement)
{
	switch (thresholdDecrement) {
	case dec_every8th:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_EVERY_8TH);
	case dec_every4th:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_EVERY_4TH);
	case dec_every2nd:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_EVERY_2ND);
	case dec_once:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_ONCE);
	case dec_twice:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_TWICE);
	case dec_4times:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_4_TIMES);
	case dec_8times:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_8_TIMES);
	case dec_16times:
		return rf69_read_mod_write(spi, REG_OOKPEAK, MASK_OOKPEAK_THRESDEC, OOKPEAK_THRESHDEC_16_TIMES);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_dio_mapping(struct spi_device *spi, u8 DIONumber, u8 value)
{
	u8 mask;
	u8 shift;
	u8 regaddr;
	u8 regValue;

	switch (DIONumber) {
	case 0:
		mask = MASK_DIO0; shift = SHIFT_DIO0; regaddr = REG_DIOMAPPING1;
		break;
	case 1:
		mask = MASK_DIO1; shift = SHIFT_DIO1; regaddr = REG_DIOMAPPING1;
		break;
	case 2:
		mask = MASK_DIO2; shift = SHIFT_DIO2; regaddr = REG_DIOMAPPING1;
		break;
	case 3:
		mask = MASK_DIO3; shift = SHIFT_DIO3; regaddr = REG_DIOMAPPING1;
		break;
	case 4:
		mask = MASK_DIO4; shift = SHIFT_DIO4; regaddr = REG_DIOMAPPING2;
		break;
	case 5:
		mask = MASK_DIO5; shift = SHIFT_DIO5; regaddr = REG_DIOMAPPING2;
		break;
	default:
	dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	// read reg
	regValue = rf69_read_reg(spi, regaddr);
	// delete old value
	regValue = regValue & ~mask;
	// add new value
	regValue = regValue | value << shift;
	// write back
	return rf69_write_reg(spi, regaddr, regValue);
}

bool rf69_get_flag(struct spi_device *spi, enum flag flag)
{
	switch (flag) {
	case modeSwitchCompleted:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_MODE_READY);
	case readyToReceive:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_RX_READY);
	case readyToSend:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_TX_READY);
	case pllLocked:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_PLL_LOCK);
	case rssiExceededThreshold:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_RSSI);
	case timeout:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_TIMEOUT);
	case automode:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_AUTOMODE);
	case syncAddressMatch:
		return (rf69_read_reg(spi, REG_IRQFLAGS1) & MASK_IRQFLAGS1_SYNC_ADDRESS_MATCH);
	case fifo_full:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_FIFO_FULL);
/*	case fifo_not_empty:
 *		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_FIFO_NOT_EMPTY); */
	case fifo_empty:
		return !(rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_FIFO_NOT_EMPTY);
	case fifo_level_below_threshold:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_FIFO_LEVEL);
	case fifo_overrun:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_FIFO_OVERRUN);
	case packetSent:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_PACKET_SENT);
	case payload_ready:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_PAYLOAD_READY);
	case crcOk:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_CRC_OK);
	case batteryLow:
		return (rf69_read_reg(spi, REG_IRQFLAGS2) & MASK_IRQFLAGS2_LOW_BAT);
	default:			 return false;
	}
}

int rf69_set_rssi_threshold(struct spi_device *spi, u8 threshold)
{
	/* no value check needed - u8 exactly matches register size */

	return rf69_write_reg(spi, REG_RSSITHRESH, threshold);
}

int rf69_set_preamble_length(struct spi_device *spi, u16 preambleLength)
{
	int retval;
	u8 msb, lsb;

	/* no value check needed - u16 exactly matches register size */

	/* calculate reg settings */
	msb = (preambleLength & 0xff00) >> 8;
	lsb = (preambleLength & 0xff);

	/* transmit to chip */
	retval = rf69_write_reg(spi, REG_PREAMBLE_MSB, msb);
	if (retval)
		return retval;
	retval = rf69_write_reg(spi, REG_PREAMBLE_LSB, lsb);

	return retval;
}

int rf69_enable_sync(struct spi_device *spi)
{
	return rf69_set_bit(spi, REG_SYNC_CONFIG, MASK_SYNC_CONFIG_SYNC_ON);
}

int rf69_disable_sync(struct spi_device *spi)
{
	return rf69_clear_bit(spi, REG_SYNC_CONFIG, MASK_SYNC_CONFIG_SYNC_ON);
}

int rf69_set_fifo_fill_condition(struct spi_device *spi, enum fifo_fill_condition fifo_fill_condition)
{
	switch (fifo_fill_condition) {
	case always:
		return rf69_set_bit(spi, REG_SYNC_CONFIG, MASK_SYNC_CONFIG_FIFO_FILL_CONDITION);
	case afterSyncInterrupt:
		return rf69_clear_bit(spi, REG_SYNC_CONFIG, MASK_SYNC_CONFIG_FIFO_FILL_CONDITION);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_sync_size(struct spi_device *spi, u8 syncSize)
{
	// check input value
	if (syncSize > 0x07) {
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	// write value
	return rf69_read_mod_write(spi, REG_SYNC_CONFIG, MASK_SYNC_CONFIG_SYNC_SIZE, (syncSize << 3));
}

int rf69_set_sync_values(struct spi_device *spi, u8 syncValues[8])
{
	int retval = 0;

	retval += rf69_write_reg(spi, REG_SYNCVALUE1, syncValues[0]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE2, syncValues[1]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE3, syncValues[2]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE4, syncValues[3]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE5, syncValues[4]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE6, syncValues[5]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE7, syncValues[6]);
	retval += rf69_write_reg(spi, REG_SYNCVALUE8, syncValues[7]);

	return retval;
}

int rf69_set_packet_format(struct spi_device *spi, enum packetFormat packetFormat)
{
	switch (packetFormat) {
	case packetLengthVar:
		return rf69_set_bit(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_PAKET_FORMAT_VARIABLE);
	case packetLengthFix:
		return rf69_clear_bit(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_PAKET_FORMAT_VARIABLE);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_enable_crc(struct spi_device *spi)
{
	return rf69_set_bit(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_CRC_ON);
}

int rf69_disable_crc(struct spi_device *spi)
{
	return rf69_clear_bit(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_CRC_ON);
}

int rf69_set_adressFiltering(struct spi_device *spi, enum addressFiltering addressFiltering)
{
	switch (addressFiltering) {
	case filteringOff:
		return rf69_read_mod_write(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_ADDRESSFILTERING, PACKETCONFIG1_ADDRESSFILTERING_OFF);
	case nodeAddress:
		return rf69_read_mod_write(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_ADDRESSFILTERING, PACKETCONFIG1_ADDRESSFILTERING_NODE);
	case nodeOrBroadcastAddress:
		return rf69_read_mod_write(spi, REG_PACKETCONFIG1, MASK_PACKETCONFIG1_ADDRESSFILTERING, PACKETCONFIG1_ADDRESSFILTERING_NODEBROADCAST);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_payload_length(struct spi_device *spi, u8 payload_length)
{
	return rf69_write_reg(spi, REG_PAYLOAD_LENGTH, payload_length);
}

int rf69_set_node_address(struct spi_device *spi, u8 nodeAddress)
{
	return rf69_write_reg(spi, REG_NODEADRS, nodeAddress);
}

int rf69_set_broadcast_address(struct spi_device *spi, u8 broadcastAddress)
{
	return rf69_write_reg(spi, REG_BROADCASTADRS, broadcastAddress);
}

int rf69_set_tx_start_condition(struct spi_device *spi, enum txStartCondition txStartCondition)
{
	switch (txStartCondition) {
	case fifo_level:
		return rf69_clear_bit(spi, REG_FIFO_THRESH, MASK_FIFO_THRESH_TXSTART);
	case fifo_not_empty:
		return rf69_set_bit(spi, REG_FIFO_THRESH, MASK_FIFO_THRESH_TXSTART);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

int rf69_set_fifo_threshold(struct spi_device *spi, u8 threshold)
{
	int retval;

	/* check input value */
	if (threshold & 0x80) {
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}

	/* write value */
	retval = rf69_read_mod_write(spi, REG_FIFO_THRESH, MASK_FIFO_THRESH_VALUE, threshold);
	if (retval)
		return retval;

	/* access the fifo to activate new threshold
	 * retval (mis-) used as buffer here
	 */
	return rf69_read_fifo(spi, (u8 *)&retval, 1);
}

int rf69_set_dagc(struct spi_device *spi, enum dagc dagc)
{
	switch (dagc) {
	case normalMode:
		return rf69_write_reg(spi, REG_TESTDAGC, DAGC_NORMAL);
	case improve:
		return rf69_write_reg(spi, REG_TESTDAGC, DAGC_IMPROVED_LOWBETA0);
	case improve4LowModulationIndex:
		return rf69_write_reg(spi, REG_TESTDAGC, DAGC_IMPROVED_LOWBETA1);
	default:
		dev_dbg(&spi->dev, "set: illegal input param");
		return -EINVAL;
	}
}

/*-------------------------------------------------------------------------*/

int rf69_read_fifo(struct spi_device *spi, u8 *buffer, unsigned int size)
{
#ifdef DEBUG_FIFO_ACCESS
	int i;
#endif
	struct spi_transfer transfer;
	u8 local_buffer[FIFO_SIZE + 1];
	int retval;

	if (size > FIFO_SIZE) {
		dev_dbg(&spi->dev, "read fifo: passed in buffer bigger then internal buffer\n");
		return -EMSGSIZE;
	}

	/* prepare a bidirectional transfer */
	local_buffer[0] = REG_FIFO;
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = local_buffer;
	transfer.rx_buf = local_buffer;
	transfer.len	= size + 1;

	retval = spi_sync_transfer(spi, &transfer, 1);

#ifdef DEBUG_FIFO_ACCESS
	for (i = 0; i < size; i++)
		dev_dbg(&spi->dev, "%d - 0x%x\n", i, local_buffer[i + 1]);
#endif

	memcpy(buffer, &local_buffer[1], size);

	return retval;
}

int rf69_write_fifo(struct spi_device *spi, u8 *buffer, unsigned int size)
{
#ifdef DEBUG_FIFO_ACCESS
	int i;
#endif
	char spi_address = REG_FIFO | WRITE_BIT;
	u8 local_buffer[FIFO_SIZE + 1];

	if (size > FIFO_SIZE) {
		dev_dbg(&spi->dev, "read fifo: passed in buffer bigger then internal buffer\n");
		return -EMSGSIZE;
	}

	local_buffer[0] = spi_address;
	memcpy(&local_buffer[1], buffer, size);

#ifdef DEBUG_FIFO_ACCESS
	for (i = 0; i < size; i++)
		dev_dbg(&spi->dev, "0x%x\n", buffer[i]);
#endif

	return spi_write(spi, local_buffer, size + 1);
}

