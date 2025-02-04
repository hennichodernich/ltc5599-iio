// SPDX-License-Identifier: GPL-2.0-only
/*
 * LTC5599 quadrature modulator driver.
 *
 * Copyright 2025 Henning Paul
 *  Author: Henning Paul <hnch@gmx.net>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

// 0x00
#define LTC5599_FREQ_REG 0x00
#define LTC5599_FREQ_VALUE(n) (n & 0x7F)
#define LTC5599_FREQ_MASK 0x7F
#define LTC5599_IQ_PHASEBAL_EXT_SIGN_BIT (1<<7)

// 0x01
#define LTC5599_GAIN_REG 0x01
#define LTC5599_GAIN_VALUE(n) (n & 0x1F)
#define LTC5599_GAIN_MASK 0x1F
#define LTC5599_QDISABLE_BIT (1<<5)
#define LTC5599_AGCTRL_BIT (1<<6)
#define LTC5599_TEMPUPDT_BIT (1<<7)

//0x02
#define LTC5599_OFFSI_REG 0x02
#define LTC5599_OFFSQ_REG 0x03
#define LTC5599_OFFS_VALUE(n) (n & 0xFF) 

//0x04
#define LTC5599_IQ_GAINRAT_REG 0x04
#define LTC5599_IQ_GAINRAT_VALUE(n) (n & 0xFF)

//0x05
#define LTC5599_IQ_PHASEBAL_REG 0x05
#define LTC5599_IQ_PHASEBAL_FINE_VALUE(n) ((n) & 0x1F)
#define LTC5599_IQ_PHASEBAL_FINE_MASK 0x1F
#define LTC5599_IQ_PHASEBAL_EXT_SHIFT 5
#define LTC5599_IQ_PHASEBAL_EXT_VALUE(n) (((n) & 0x07) << LTC5599_IQ_PHASEBAL_EXT_SHIFT)
#define LTC5599_IQ_PHASEBAL_EXT_MASK (0x07 << LTC5599_IQ_PHASEBAL_EXT_SHIFT)

//0x06
#define LTC5599_LOMATCH_OVR_REG 0x06

//0x07
#define LTC5599_TEMPCORR_OVR_REG 0x07

//0x08
#define LTC5599_MODE_REG 0x08
#define LTC5599_RESET_BIT (1<<3)

#define LTC5599_READ_OPERATION 0x01

/**
 * struct ltc5599_chip_info - chip specific information
 * @channels:		Channel specification
 */
struct ltc5599_chip_info {
	const struct iio_chan_spec *channels;
};

/**
 * struct ltc5599 - driver instance specific data
 * @spi:		the SPI device for this driver instance
 * @chip_info:		chip model specific constants, available modes etc
 * @data:		spi transfer buffers
 */
struct ltc5599 {
	struct spi_device		*spi;
	const struct ltc5599_chip_info	*chip_info;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__u8 data[2] ____cacheline_aligned;
	__u8 shadowregs[32];
};

enum ltc5599_type {
	ID_LTC5599,
};


static int spi_read_while_write(struct spi_device *spi, const void *txbuf, void *rxbuf, unsigned n_trx)
{
	int			status;
	struct spi_message	message;
	struct spi_transfer	x;

	if (!n_trx)
		return 0;

	spi_message_init(&message);
	memset(&x, 0, sizeof(x));

	x.len = n_trx;
	x.tx_buf = txbuf;
	x.rx_buf = rxbuf;
	spi_message_add_tail(&x, &message);

	status = spi_sync(spi, &message);

	return status;
}

static int ltc5599_write(struct iio_dev *indio_dev, u8 addr,	u8 val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	st->data[0] = ((addr & 0x7F) << 1) & (~LTC5599_READ_OPERATION);
	st->data[1] = val & 0xFF;
	ret = spi_read_while_write(st->spi, st->data, NULL, 2);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ltc5599_read(struct iio_dev *indio_dev, u8 addr, u8 *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	u8 tmp[2];

	mutex_lock(&indio_dev->mlock);
	st->data[0] = ((addr & 0x7F) << 1) | LTC5599_READ_OPERATION;
	st->data[1] = 0xFF;
	ret = spi_read_while_write(st->spi, st->data, tmp, 2);
	if (ret < 0)
		goto out_unlock;

	*val = tmp[1];

out_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ltc5599_write_freq(struct iio_dev *indio_dev, unsigned int val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	uint8_t tmp = st->shadowregs[LTC5599_FREQ_REG];

	tmp = (tmp & ~LTC5599_FREQ_MASK) | LTC5599_FREQ_VALUE(val);

	ret = ltc5599_write(indio_dev, LTC5599_FREQ_REG, tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_FREQ_REG] = tmp;
	return 0;
}

static int ltc5599_read_freq(struct iio_dev *indio_dev, unsigned int *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	u8 tmp;
	int ret;

	ret = ltc5599_read(indio_dev, LTC5599_FREQ_REG, &tmp);
	if (ret)
		return ret;

	st->shadowregs[LTC5599_FREQ_REG] = tmp;
	*val = st->shadowregs[LTC5599_FREQ_REG] & LTC5599_FREQ_MASK;
	return 0;
}

static int ltc5599_write_gain(struct iio_dev *indio_dev, unsigned int val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	uint8_t tmp = st->shadowregs[LTC5599_GAIN_REG];

	tmp = (tmp & ~LTC5599_GAIN_MASK) | LTC5599_GAIN_VALUE(val);

	ret = ltc5599_write(indio_dev, LTC5599_GAIN_REG, tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_GAIN_REG] = tmp;
	return 0;
}

static int ltc5599_read_gain(struct iio_dev *indio_dev, unsigned int *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	u8 tmp;
	int ret;

	ret = ltc5599_read(indio_dev, LTC5599_GAIN_REG, &tmp);
	if (ret)
		return ret;

	st->shadowregs[LTC5599_GAIN_REG] = tmp;
	*val = st->shadowregs[LTC5599_GAIN_REG] & LTC5599_GAIN_MASK;
	return 0;
}

static int ltc5599_write_offset(struct iio_dev *indio_dev, unsigned int chan, int val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	
	if (chan > 1)
		return -EINVAL;
	
	if (val > 127)
		val = 127;
	if (val < -127)
		val = -127;

	val += 128;

	u8 tmp = LTC5599_OFFS_VALUE(val);

	ret = ltc5599_write(indio_dev, LTC5599_OFFSI_REG+chan, tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_OFFSI_REG+chan] = tmp;
	return 0;
}

static int ltc5599_read_offset(struct iio_dev *indio_dev, unsigned int chan, int *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	u8 tmp;

	int ret;

	if (chan > 1)
		return -EINVAL;

	ret = ltc5599_read(indio_dev, LTC5599_OFFSI_REG+chan, &tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_OFFSI_REG+chan] = tmp;
	*val = st->shadowregs[LTC5599_OFFSI_REG+chan];
	*val -= 128;

	return 0;
}

static int ltc5599_write_iqgainratio(struct iio_dev *indio_dev, int val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	uint8_t tmp = LTC5599_IQ_GAINRAT_VALUE(val);

       	tmp ^= 0x80;

	ret = ltc5599_write(indio_dev, LTC5599_IQ_GAINRAT_REG, tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_IQ_GAINRAT_REG] = tmp;
	return 0;
}

static int ltc5599_read_iqgainratio(struct iio_dev *indio_dev, int *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	u8 tmp;
	int ret;

	ret = ltc5599_read(indio_dev, LTC5599_IQ_GAINRAT_REG, &tmp);
	if (ret)
		return ret;

	st->shadowregs[LTC5599_IQ_GAINRAT_REG] = tmp;
	*val = st->shadowregs[LTC5599_IQ_GAINRAT_REG];
       	*val -= 128;
	return 0;
}

static int ltc5599_write_iqphasebalance(struct iio_dev *indio_dev, int val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret, coarse;
	uint8_t tmp1;
	uint8_t tmp2 = st->shadowregs[LTC5599_FREQ_REG];
	
	if (val < -16)
		tmp2 &= ~LTC5599_IQ_PHASEBAL_EXT_SIGN_BIT;
	else
		tmp2 |= LTC5599_IQ_PHASEBAL_EXT_SIGN_BIT;
	
	if (val>0)
        	coarse = (val+16) / 32;
        else
        	coarse = (15-val) / 32;

	tmp1 = LTC5599_IQ_PHASEBAL_EXT_VALUE(coarse) | LTC5599_IQ_PHASEBAL_FINE_VALUE((val & 0x1F) ^ 0x10);

	ret = ltc5599_write(indio_dev, LTC5599_IQ_PHASEBAL_REG, tmp1);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_IQ_PHASEBAL_REG] = tmp1;
	
	ret = ltc5599_write(indio_dev, LTC5599_FREQ_REG, tmp2);
	if (ret)
		return ret;

	st->shadowregs[LTC5599_FREQ_REG] = tmp2;
	return 0;
}

static int ltc5599_read_iqphasebalance(struct iio_dev *indio_dev, int *val)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	u8 tmp;
	int ret, multiplier, coarse;

	ret = ltc5599_read(indio_dev, LTC5599_FREQ_REG, &tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_FREQ_REG] = tmp;

	if (tmp & LTC5599_IQ_PHASEBAL_EXT_SIGN_BIT)
		multiplier = 1;
	else
		multiplier = -1;

	ret = ltc5599_read(indio_dev, LTC5599_IQ_PHASEBAL_REG, &tmp);
	if (ret)
		return ret;
	st->shadowregs[LTC5599_IQ_PHASEBAL_REG] = tmp;

	coarse = (tmp & LTC5599_IQ_PHASEBAL_EXT_MASK) >> LTC5599_IQ_PHASEBAL_EXT_SHIFT;

	*val = (tmp & LTC5599_IQ_PHASEBAL_FINE_MASK) - 16;
	*val += multiplier * coarse * 32;

	return 0;
}

static int ltc5599_fill_shadowregs(struct iio_dev *indio_dev)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	memset(st->shadowregs, 0x00, 32);
	st->shadowregs[LTC5599_FREQ_REG] 	= 0x2E;
	st->shadowregs[LTC5599_GAIN_REG] 	= 0x84;
	st->shadowregs[LTC5599_OFFSI_REG] 	= 0x80;
	st->shadowregs[LTC5599_OFFSQ_REG] 	= 0x80;
	st->shadowregs[LTC5599_IQ_GAINRAT_REG] 	= 0x80;
	st->shadowregs[LTC5599_IQ_PHASEBAL_REG] = 0x10;
	st->shadowregs[LTC5599_LOMATCH_OVR_REG] = 0x50;
	st->shadowregs[LTC5599_TEMPCORR_OVR_REG] = 0x06;
	st->shadowregs[LTC5599_MODE_REG] 	= 0x00;

	return 0;
}

static int ltc5599_init_registers(struct iio_dev *indio_dev)
{
	struct ltc5599 *st = iio_priv(indio_dev);

	return 0;
}

static unsigned int freq_to_ctrl_word(unsigned int freq_in_khz)
{
	if (freq_in_khz > 1249100)
        	return 1;
	else if (freq_in_khz > 1248600)
        	return 2;
	else if (freq_in_khz > 1238100)
        	return 3;
	else if (freq_in_khz > 1214100)
        	return 4;
	else if (freq_in_khz > 1191200)
        	return 5;
	else if (freq_in_khz > 1165600)
        	return 6;
	else if (freq_in_khz > 1141000)
        	return 7;
	else if (freq_in_khz > 1120600)
        	return 8;
	else if (freq_in_khz > 1100500)
        	return 9;
	else if (freq_in_khz > 1069500)
        	return 10;
	else if (freq_in_khz > 1039599)
        	return 11;
	else if (freq_in_khz > 1023100)
        	return 12;
	else if (freq_in_khz > 1007100)
        	return 13;
	else if (freq_in_khz > 988300)
        	return 14;
	else if (freq_in_khz > 961800)
        	return 15;
	else if (freq_in_khz > 941300)
        	return 16;
	else if (freq_in_khz > 921500)
        	return 17;
	else if (freq_in_khz > 895200)
        	return 18;
	else if (freq_in_khz > 877600)
        	return 19;
	else if (freq_in_khz > 863600)
        	return 20;
	else if (freq_in_khz > 843200)
        	return 21;
	else if (freq_in_khz > 826900)
        	return 22;
	else if (freq_in_khz > 807000)
        	return 23;
	else if (freq_in_khz > 792300)
        	return 24;
	else if (freq_in_khz > 772200)
        	return 25;
	else if (freq_in_khz > 752700)
        	return 26;
	else if (freq_in_khz > 734000)
        	return 27;
	else if (freq_in_khz > 724200)
        	return 28;
	else if (freq_in_khz > 704600)
        	return 29;
	else if (freq_in_khz > 688700)
        	return 30;
	else if (freq_in_khz > 673200)
        	return 31;
	else if (freq_in_khz > 655200)
        	return 32;
	else if (freq_in_khz > 638100)
        	return 33;
	else if (freq_in_khz > 624600)
        	return 34;
	else if (freq_in_khz > 611900)
        	return 35;
	else if (freq_in_khz > 598400)
        	return 36;
	else if (freq_in_khz > 585100)
        	return 37;
	else if (freq_in_khz > 573900)
        	return 38;
	else if (freq_in_khz > 563100)
        	return 39;
	else if (freq_in_khz > 548100)
        	return 40;
	else if (freq_in_khz > 538100)
        	return 41;
	else if (freq_in_khz > 529100)
        	return 42;
	else if (freq_in_khz > 518500)
        	return 43;
	else if (freq_in_khz > 507000)
        	return 44;
	else if (freq_in_khz > 497700)
        	return 45;
	else if (freq_in_khz > 488000)
        	return 46;
	else if (freq_in_khz > 471500)
        	return 47;
	else if (freq_in_khz > 457700)
        	return 48;
	else if (freq_in_khz > 448700)
        	return 49;
	else if (freq_in_khz > 437400)
        	return 50;
	else if (freq_in_khz > 426600)
        	return 51;
	else if (freq_in_khz > 417500)
        	return 52;
	else if (freq_in_khz > 407500)
        	return 53;
	else if (freq_in_khz > 398000)
        	return 54;
	else if (freq_in_khz > 390100)
        	return 55;
	else if (freq_in_khz > 382800)
        	return 56;
	else if (freq_in_khz > 376600)
        	return 57;
	else if (freq_in_khz > 369800)
        	return 58;
	else if (freq_in_khz > 353100)
        	return 59;
	else if (freq_in_khz > 339000)
        	return 60;
	else if (freq_in_khz > 332600)
        	return 61;
	else if (freq_in_khz > 327200)
        	return 62;
	else if (freq_in_khz > 320600)
        	return 63;
	else if (freq_in_khz > 313700)
        	return 64;
	else if (freq_in_khz > 309100)
        	return 65;
	else if (freq_in_khz > 304500)
        	return 66;
	else if (freq_in_khz > 288100)
        	return 67;
	else if (freq_in_khz > 278300)
        	return 68;
	else if (freq_in_khz > 274200)
        	return 69;
	else if (freq_in_khz > 270300)
        	return 70;
	else if (freq_in_khz > 266000)
        	return 71;
	else if (freq_in_khz > 261899)
        	return 72;
	else if (freq_in_khz > 258200)
        	return 73;
	else if (freq_in_khz > 254100)
        	return 74;
	else if (freq_in_khz > 243600)
        	return 75;
	else if (freq_in_khz > 233800)
        	return 76;
	else if (freq_in_khz > 230800)
        	return 77;
	else if (freq_in_khz > 228000)
        	return 78;
	else if (freq_in_khz > 220200)
        	return 79;
	else if (freq_in_khz > 212600)
        	return 80;
	else if (freq_in_khz > 210000)
        	return 81;
	else if (freq_in_khz > 207600)
        	return 82;
	else if (freq_in_khz > 202100)
        	return 83;
	else if (freq_in_khz > 196200)
        	return 84;
	else if (freq_in_khz > 193700)
        	return 85;
	else if (freq_in_khz > 191200)
        	return 86;
	else if (freq_in_khz > 186600)
        	return 87;
	else if (freq_in_khz > 182000)
        	return 88;
	else if (freq_in_khz > 179400)
        	return 89;
	else if (freq_in_khz > 176000)
        	return 90;
	else if (freq_in_khz > 170100)
        	return 91;
	else if (freq_in_khz > 165000)
        	return 92;
	else if (freq_in_khz > 162500)
        	return 93;
	else if (freq_in_khz > 160000)
        	return 94;
	else if (freq_in_khz > 156700)
        	return 95;
	else if (freq_in_khz > 153600)
        	return 96;
	else if (freq_in_khz > 151100)
        	return 97;
	else if (freq_in_khz > 148600)
        	return 98;
	else if (freq_in_khz > 142500)
        	return 99;
	else if (freq_in_khz > 139600)
        	return 100;
	else if (freq_in_khz > 136500)
        	return 101;
	else if (freq_in_khz > 134300)
        	return 102;
	else if (freq_in_khz > 131200)
        	return 103;
	else if (freq_in_khz > 128100)
        	return 104;
	else if (freq_in_khz > 126000)
        	return 105;
	else if (freq_in_khz > 123800)
        	return 106;
	else if (freq_in_khz > 121300)
        	return 107;
	else if (freq_in_khz > 118300)
        	return 108;
	else if (freq_in_khz > 115700)
        	return 109;
	else if (freq_in_khz > 113500)
        	return 110;
	else if (freq_in_khz > 111300)
        	return 111;
	else if (freq_in_khz > 109500)
        	return 112;
	else if (freq_in_khz > 107600)
        	return 113;
	else if (freq_in_khz > 105600)
        	return 114;
	else if (freq_in_khz > 103000)
        	return 115;
	else if (freq_in_khz > 100300)
        	return 116;
	else if (freq_in_khz > 98500)
        	return 117;
	else if (freq_in_khz > 96600)
        	return 118;
	else if (freq_in_khz > 94700)
        	return 119;
	else if (freq_in_khz > 93000)
        	return 120;
	else 
        	return 121;
}


static int ltc5599_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	int ret;
	unsigned int tmp;
	long long tmp2;

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		ret = ltc5599_read_offset(indio_dev, chan->address, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		ret = ltc5599_read_freq(indio_dev, &tmp);
		if (ret)
			return ret;
		tmp2 = -553LL * (int)tmp * (int)tmp * (int)tmp;
		tmp2 += 198810LL * (int)tmp * (int)tmp;
		tmp2 += -26120002LL * (int)tmp;
		tmp2 += 1319492809LL;
		*val = (int)tmp2;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ltc5599_read_gain(indio_dev, &tmp);
		if (ret)
			return ret;
		*val = -1 * (int)tmp;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW:
		ret = ltc5599_read_iqgainratio(indio_dev, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		ret = ltc5599_read_iqphasebalance(indio_dev, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ltc5599_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct ltc5599 *st = iio_priv(indio_dev);
	int ret;
	unsigned int tmp;

	switch (info) {
	case IIO_CHAN_INFO_OFFSET:
		if ((val < -127) || (val > 127))
			return -EINVAL;
		ret = ltc5599_write_offset(indio_dev, chan->address, val);
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if ((val < 30000000) || (val > 1300000000))
			return -EINVAL;
		tmp = freq_to_ctrl_word(val/1000);

		ret = ltc5599_write_freq(indio_dev, tmp);
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val > 0)
			return -EINVAL;
		tmp = -val;
		if (tmp > 19)
			tmp = 19;

		ret = ltc5599_write_gain(indio_dev, tmp);
		break;
	case IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW:
		if ((val < -127) || (val > 127))
			return -EINVAL;
		ret = ltc5599_write_iqgainratio(indio_dev, val);
		break;
	case IIO_CHAN_INFO_PHASE:
		if ((val < -240) || (val > 239))
			return -EINVAL;
		ret = ltc5599_write_iqphasebalance(indio_dev, val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info ltc5599_info = {
	.read_raw = ltc5599_read_raw,
	.write_raw = ltc5599_write_raw,
};

#define LTC5599_CHANNEL(chan) {				\
	.type = IIO_ALTVOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.address = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_OFFSET),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW) | BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_FREQUENCY) | BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
}

static const struct iio_chan_spec ltc5599_channels[] = { \
	LTC5599_CHANNEL(0), \
	LTC5599_CHANNEL(1), \
};

static const struct ltc5599_chip_info ltc5599_chip_info[] = {
	[ID_LTC5599] = {
		.channels = ltc5599_channels,
	},
};

static int ltc5599_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct ltc5599 *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->chip_info = &ltc5599_chip_info[id->driver_data];
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = id->name;
	indio_dev->info = &ltc5599_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = 2;

	ltc5599_fill_shadowregs(indio_dev);
	ltc5599_init_registers(indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	return 0;
}

static void ltc5599_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
}

static const struct spi_device_id ltc5599_spi_ids[] = {
	{ "ltc5599", ID_LTC5599 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc5599_spi_ids);

static struct spi_driver ltc5599_spi_driver = {
	.driver = {
		.name = "ltc5599",
	},
	.probe = ltc5599_spi_probe,
	.remove = ltc5599_spi_remove,
	.id_table = ltc5599_spi_ids,
};
module_spi_driver(ltc5599_spi_driver);

MODULE_AUTHOR("Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("Analog Devices LTC5599 quadrature modulator");
MODULE_LICENSE("GPL v2");
