// SPDX-License-Identifier: GPL-2.0-or-later
/* Driver for TI CC1120 802.15.4 Wireless-PAN Networking controller
 *
 * Copyright (C) 2014 Varka Bhadram <varkab@cdac.in>
 *		      Md.Jamal Mohiuddin <mjmohiuddin@cdac.in>
 *		      P Sowjanya <sowjanyap@cdac.in>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/of_gpio.h>
#include <linux/ieee802154.h>
#include <linux/crc-ccitt.h>
#include <asm/unaligned.h>

#include <net/mac802154.h>
#include <net/cfg802154.h>
#include "cc1120.h"


struct cc1120_platform_data {
#if 0
	struct gpio_desc *interrrupt;
	struct gpio_desc *tcxo;
	struct gpio_desc *hgm;
	struct gpio_desc *reset;
#endif
	int pin_interrupt;
	int pin_tcxo;
	int pin_hgm;
	int pin_rst;
};

/* Driver private information */
struct cc1120_private {
	struct spi_device 	*spi;			/* SPI device structure */
	struct ieee802154_hw 	*hw;		/* IEEE-802.15.4 device */
	u8 					*buf;			/* SPI TX/Rx data buffer */
	struct mutex 		buffer_mutex;	/* SPI buffer mutex */
	bool 				is_tx;			/* Flag for sync b/w Tx and Rx */
	bool 				amplified;		/* Flag for CC2591 */
	int 				fifo_pin;		/* FIFO GPIO pin number */
	struct work_struct 	fifop_irqwork;	/* Workqueue for FIFOP */
	spinlock_t 			lock;			/* Lock for is_tx*/
	struct completion 	tx_complete;	/* Work completion for Tx */
	cc1200_rf_registers_set_t *p_rf_settings;	/* rf settings */
};

static const cc1120_reg_settings_t g_cc1120_reg_settings_groups[] =
{
    {CC112X_IOCFG3 , 0x06},         // IOCFG3              GPIO3 IO Pin Configuration
    {CC112X_IOCFG2 , 0x58},         // IOCFG2              GPIO2 IO Pin Configuration
    {CC112X_IOCFG1 , 0xB0},         // IOCFG1              GPIO1 IO Pin Configuration
    {CC112X_IOCFG0 , 0x59},         // IOCFG0              GPIO0 IO Pin Configuration
    {CC112X_SYNC_CFG1 , 0x08},      // SYNC_CFG1           Sync Word Detection Configuration Reg. 1
    {CC112X_DEVIATION_M , 0x99},    // DEVIATION_M         Frequency Deviation Configuration
    {CC112X_MODCFG_DEV_E , 0x0D},   // MODCFG_DEV_E        Modulation Format and Frequency Deviation Configur..
    {CC112X_DCFILT_CFG , 0x15},     // DCFILT_CFG          Digital DC Removal Configuration
    {CC112X_PREAMBLE_CFG1 , 0x18},  // PREAMBLE_CFG1       Preamble Length Configuration Reg. 1
    {CC112X_FREQ_IF_CFG , 0x3A},    // FREQ_IF_CFG         RX Mixer Frequency Configuration
    {CC112X_IQIC , 0x00},           // IQIC                Digital Image Channel Compensation Configuration
    {CC112X_CHAN_BW , 0x02},        // CHAN_BW             Channel Filter Configuration
    {CC112X_MDMCFG0 , 0x05},        // MDMCFG0             General Modem Parameter Configuration Reg. 0
    {CC112X_SYMBOL_RATE2 , 0x99},   // SYMBOL_RATE2        Symbol Rate Configuration Exponent and Mantissa [1..
    {CC112X_SYMBOL_RATE1 , 0x99},   // SYMBOL_RATE1        Symbol Rate Configuration Mantissa [15:8]
    {CC112X_SYMBOL_RATE0 , 0x99},   // SYMBOL_RATE0        Symbol Rate Configuration Mantissa [7:0]
    {CC112X_AGC_REF , 0x3C},        // AGC_REF             AGC Reference Level Configuration
    {CC112X_AGC_CS_THR , 0xEF},     // AGC_CS_THR          Carrier Sense Threshold Configuration
    {CC112X_AGC_CFG1 , 0xA9},       // AGC_CFG1            Automatic Gain Control Configuration Reg. 1
    {CC112X_AGC_CFG0 , 0xC0},       // AGC_CFG0            Automatic Gain Control Configuration Reg. 0
    {CC112X_FIFO_CFG , 0x00},       // FIFO_CFG            FIFO Configuration
    {CC112X_FS_CFG , 0x12},         // FS_CFG              Frequency Synthesizer Configuration
    {CC112X_PKT_CFG0 , 0x20},       // PKT_CFG0            Variable packet length mode. Packet length configured by the first byte received after sync word
    {CC112X_PA_CFG2 , 0x77},        // PA_CFG2             Power Amplifier Configuration Reg. 2
    {CC112X_PA_CFG0 , 0x79},        // PA_CFG0             Power Amplifier Configuration Reg. 0
    {CC112X_PKT_LEN , 0xFF},        // PKT_LEN             Packet Length Configuration
    {CC112X_IF_MIX_CFG , 0x00},     // IF_MIX_CFG          IF Mix Configuration
    {CC112X_TOC_CFG , 0x0A},        // TOC_CFG             Timing Offset Correction Configuration
    {CC112X_FREQ2 , 0x72},          // FREQ2               Frequency Configuration [23:16]
    {CC112X_FREQ1 , 0x60},          // FREQ1               Frequency Configuration [15:8]
    {CC112X_FS_DIG1 , 0x00},        // FS_DIG1             Frequency Synthesizer Digital Reg. 1
    {CC112X_FS_DIG0 , 0x5F},        // FS_DIG0             Frequency Synthesizer Digital Reg. 0
    {CC112X_FS_CAL1 , 0x40},        // FS_CAL1             Frequency Synthesizer Calibration Reg. 1
    {CC112X_FS_CAL0 , 0x0E},        // FS_CAL0             Frequency Synthesizer Calibration Reg. 0
    {CC112X_FS_DIVTWO , 0x03},      // FS_DIVTWO           Frequency Synthesizer Divide by 2
    {CC112X_FS_DSM0 , 0x33},        // FS_DSM0             FS Digital Synthesizer Module Configuration Reg. 0
    {CC112X_FS_DVC0 , 0x17},        // FS_DVC0             Frequency Synthesizer Divider Chain Configuration ..
    {CC112X_FS_PFD , 0x50},         // FS_PFD              Frequency Synthesizer Phase Frequency Detector Con..
    {CC112X_FS_PRE , 0x6E},         // FS_PRE              Frequency Synthesizer Prescaler Configuration
    {CC112X_FS_REG_DIV_CML , 0x14}, // FS_REG_DIV_CML      Frequency Synthesizer Divider Regulator Configurat..
    {CC112X_FS_SPARE , 0xAC},       // FS_SPARE            Frequency Synthesizer Spare
    {CC112X_FS_VCO0 , 0xB4},        // FS_VCO0             FS Voltage Controlled Oscillator Configuration Reg..
    {CC112X_XOSC5 , 0x0E},          // XOSC5               Crystal Oscillator Configuration Reg. 5
    {CC112X_XOSC1 , 0x03},          // XOSC1               Crystal Oscillator Configuration Reg. 1
};


static cc1200_rf_registers_set_t g_cc1120_rf_settings_group[] =
{
    {
        .chan_center_freq0 = 915000,
        .channel_spacing = 2000, /* 200 KHz */
        .p_rf_settings = g_cc1120_reg_settings_groups,
    },
};

static inline void dev_dump(void * p_buffer, uint32_t len)
{
    uint8_t *p = (uint8_t *)p_buffer;
    uint32_t index;
    for ( index = 0; index <  len; index++)
    {
        printk(KERN_CONT "%02X ", p[index]);
    }
}

/* Generic Functions */
static int cc1120_cmd_strobe(struct cc1120_private *priv, u8 cmd)
{
	struct spi_transfer xfer[1] = { };
	int ret = 0;

	priv->buf[0] = cmd;

	xfer[0].tx_buf = priv->buf;
	xfer[0].rx_buf = priv->buf;
	xfer[0].len = 1;
	mutex_lock(&priv->buffer_mutex);

	ret = spi_sync_transfer(priv->spi, xfer, ARRAY_SIZE(xfer));
	if(ret < 0)
	{
		dev_err(&priv->spi->dev,"spi_sync_transfer err %d\n", ret);
	}

	mutex_unlock(&priv->buffer_mutex);

	return ret;
}

/******************************************************************************
 * @fn      cc1120_get_status(void)
 *
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
 *          status of the radio and the number of free bytes in the TX FIFO.
 *
 *          Status byte:
 *          --------------------------------------------------------------------------------
 *          |          |            |                                                      |
 *          | CHIP_RDY | STATE[2:0] | 						Reserved				 	   |
 *          |          |            |                                                      |
 *          --------------------------------------------------------------------------------
 */
static int cc1120_get_status(struct cc1120_private *priv, u8 *status)
{
	int ret = 0;

	ret = cc1120_cmd_strobe(priv, CC112X_SNOP);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev,"spi_sync_transfer err %d\n", ret);
	}
	else if(!ret)
	{
		*status = priv->buf[0];
		dev_vdbg(&priv->spi->dev,"cc1120_get_status status = %02x\n",*status);
	}

	return ret;
}

static int cc1120_wr_reg(struct cc1120_private *priv, u16 addr, u8 *data, u8 len)
{
	int 	err_code = 0;
	uint8_t tx1buff[2] = {0 ,0};
	uint8_t heard_len = 0;
	uint8_t exten_cmd = (uint8_t)(addr >> 8);
	uint8_t base_addr = (uint8_t)(addr & 0xff);

	struct spi_transfer xfer[2] = { };

	tx1buff[0] &= 0x7F;
	tx1buff[0] |= 0x40;

	if(!exten_cmd)
	{
		heard_len = 1;
		tx1buff[0] |= base_addr;
	}
	else
	{
		heard_len = 2;
		tx1buff[0] |= exten_cmd;
		tx1buff[1] |= base_addr;
	}

	xfer[0].tx_buf = tx1buff;
	xfer[0].len = heard_len;

	xfer[1].tx_buf = data;
	xfer[1].len = len;

	mutex_lock(&priv->buffer_mutex);

	err_code = spi_sync_transfer(priv->spi, xfer, ARRAY_SIZE(xfer));
	if(err_code < 0)
	{
		dev_err(&priv->spi->dev,
		"spi_sync_transfer err %d\n", err_code);
	}

	mutex_unlock(&priv->buffer_mutex);

	return err_code;
}

static int cc1120_rd_reg(struct cc1120_private *priv, u16 addr, u8 *data, u8 len)
{
	int 	err_code = 0;
	uint8_t tx1buff[2] = {0,0};
	uint8_t heard_len = 0;
	uint8_t exten_cmd = (uint8_t)(addr >> 8);
	uint8_t base_addr = (uint8_t)(addr & 0xff);

	struct spi_transfer xfer[2] = { };

	tx1buff[0] |= 0xC0;
	if(!exten_cmd)
	{
		heard_len = 1;
		tx1buff[0] |= base_addr;
	}
	else
	{
		heard_len = 2;
		tx1buff[0] |= exten_cmd;
		tx1buff[1] |= base_addr;
	}

	xfer[0].tx_buf = tx1buff;
	xfer[0].len = heard_len;

	xfer[1].rx_buf = data;
	xfer[1].len = len;

	mutex_lock(&priv->buffer_mutex);

	err_code = spi_sync_transfer(priv->spi, xfer, ARRAY_SIZE(xfer));
	if(err_code < 0)
	{
		dev_err(&priv->spi->dev,
		"spi_sync_transfer err %d\n", err_code);
	}

	mutex_unlock(&priv->buffer_mutex);

	return err_code;
}

static int cc1120_write_txfifo(struct cc1120_private *priv, u8 *data, u8 len)
{
	/**
	 * Reference to cc1120 demo
	 */
	int ret = 0;

	ret = cc1120_wr_reg(priv,CC112X_BURST_TXFIFO, data, len);
	return ret;
}

static int cc1120_read_rxfifo(struct cc1120_private *priv, u8 *data, u8 len)
{
	/**
	 * Reference to cc1120 demo
	 */
	int ret = 0;

	ret = cc1120_rd_reg(priv,CC112X_BURST_RXFIFO, data, len);
	return 0;
}

/**
 * @brief cc1120_hw_init -- Configure cc1120 registers
 */
static int cc1120_hw_init(struct cc1120_private *priv)
{
	int ret = 0;
	uint8_t i = 0;
	uint8_t test_value = 0;

	dev_dbg(&priv->spi->dev, "cc1120_hw_init be called\n");

	for(i = 0; i < ARRAY_SIZE(g_cc1120_reg_settings_groups); i++ )
	{
		ret = cc1120_wr_reg(priv, priv->p_rf_settings->p_rf_settings[i].addr, \
							&priv->p_rf_settings->p_rf_settings[i].data, 1);

		dev_info(&priv->spi->dev, "Write 0x%02x to reg 0x%04x ,ret = %d\n",priv->p_rf_settings->p_rf_settings[i].data,
																		   priv->p_rf_settings->p_rf_settings[i].addr, ret);
		if(ret < 0)
		{
			dev_err(&priv->spi->dev,"cc1120_init err %d\n", ret);
			return ret;
		}
	}

	/* test spi function */
	cc1120_rd_reg(priv , priv->p_rf_settings->p_rf_settings[1].addr, &test_value, 1);
	if(priv->p_rf_settings->p_rf_settings[1].data != test_value )
	{
		dev_err(&priv->spi->dev,"cc1120_init err, Check the SPI interface function\n");
		return -EIO;
	}

	ret = cc1120_cmd_strobe(priv, CC112X_SIDLE);

	dev_dbg(&priv->spi->dev, "cc1120_hw_init finish,ret = %d\n",ret);

	return 0;
}

/**
 * @brief cc1120_rf_calibrate -- Rf_calibrate
 */
static int cc1120_rf_calibrate(struct cc1120_private *priv)
{
	int ret = 0;

	/**
	 * Reference to module demo
	 */
	#define VCDAC_START_OFFSET 2
	#define FS_VCO2_INDEX 0
	#define FS_VCO4_INDEX 1
	#define FS_CHP_INDEX 2

	uint8_t original_fs_cal2 = 0;
    uint8_t cal_results_for_vcdac_start_high[3];
    uint8_t cal_results_for_vcdac_start_mid[3];
    uint8_t marcstate;
    uint8_t write_byte;

	uint8_t rties = 0;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    write_byte = 0x00;
    cc1120_wr_reg(priv,CC112X_FS_VCO2, &write_byte, 1);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc1120_rd_reg(priv,CC112X_FS_CAL2, &original_fs_cal2, 1);
	write_byte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc1120_wr_reg(priv,CC112X_FS_CAL2, &write_byte, 1);

	// 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    cc1120_cmd_strobe(priv,CC112X_SCAL);

	do {
        cc1120_rd_reg(priv,CC112X_MARCSTATE, &marcstate, 1);
		dev_warn(&priv->spi->dev,"CC112X_MARCSTATE != 0x41\n");
		rties++;
		usleep_range(1000,2000);
    } while ((marcstate != 0x41) && (rties));
	if(rties == 0)
	{
		dev_err(&priv->spi->dev,"CC112X_MARCSTATE != 0x41,overtime\n");
		return -EINVAL;
	}

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
    cc1120_rd_reg(priv,CC112X_FS_VCO2,
                     &cal_results_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc1120_rd_reg(priv,CC112X_FS_VCO4,
                     &cal_results_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc1120_rd_reg(priv,CC112X_FS_CHP,
                     &cal_results_for_vcdac_start_high[FS_CHP_INDEX], 1);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    write_byte = 0x00;
    cc1120_wr_reg(priv,CC112X_FS_VCO2, &write_byte, 1);

	// 6) Continue with mid VCDAC (original VCDAC_START):
    write_byte = original_fs_cal2;
    cc1120_wr_reg(priv,CC112X_FS_CAL2, &write_byte, 1);

	// 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    cc1120_cmd_strobe(priv,CC112X_SCAL);

	rties = 0;
	do {
        cc1120_rd_reg(priv,CC112X_MARCSTATE, &marcstate, 1);
		dev_warn(&priv->spi->dev,"CC112X_MARCSTATE != 0x41\n");
		rties++;
		usleep_range(1000,2000);
    } while ((marcstate != 0x41) && (rties));
	if(rties == 0)
	{
		dev_err(&priv->spi->dev,"CC112X_MARCSTATE != 0x41,overtime\n");
		return -EINVAL;
	}

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    cc1120_rd_reg(priv,CC112X_FS_VCO2,
                     &cal_results_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc1120_rd_reg(priv,CC112X_FS_VCO4,
                     &cal_results_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc1120_rd_reg(priv,CC112X_FS_CHP,
                     &cal_results_for_vcdac_start_mid[FS_CHP_INDEX], 1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (cal_results_for_vcdac_start_high[FS_VCO2_INDEX] >
            cal_results_for_vcdac_start_mid[FS_VCO2_INDEX])
	{
        write_byte = cal_results_for_vcdac_start_high[FS_VCO2_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_VCO2, &write_byte, 1);
        write_byte = cal_results_for_vcdac_start_high[FS_VCO4_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_VCO4, &write_byte, 1);
        write_byte = cal_results_for_vcdac_start_high[FS_CHP_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_CHP, &write_byte, 1);
    }
	else
	{
        write_byte = cal_results_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_VCO2, &write_byte, 1);
        write_byte = cal_results_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_VCO4, &write_byte, 1);
        write_byte = cal_results_for_vcdac_start_mid[FS_CHP_INDEX];
        cc1120_wr_reg(priv,CC112X_FS_CHP, &write_byte, 1);
    }
	return ret;
}

/**
 * @brief cc1120_start -- Flush the RX && TX FIFO,and start RX
 */
static int cc1120_start(struct ieee802154_hw *hw)
{
	int ret = 0;
	uint8_t reg_value = 0;

	struct cc1120_private *priv = hw->priv;

	/* set tx/rx off mode */
	cc1120_rd_reg(priv,CC112X_RFEND_CFG1,&reg_value,1);
	reg_value |= RXOFF_MODE;
	cc1120_wr_reg(priv,CC112X_RFEND_CFG1,&reg_value,1);

	cc1120_rd_reg(priv,CC112X_RFEND_CFG0,&reg_value,1);
	reg_value |= TXOFF_MODE;
	cc1120_wr_reg(priv,CC112X_RFEND_CFG0,&reg_value,1);

	ret = cc1120_cmd_strobe(priv, CC112X_SIDLE);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SIDLE err %d\n", ret);
		return ret;
	}

	ret = cc1120_cmd_strobe(priv, CC112X_SFRX);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SFRX err %d\n", ret);
		return ret;
	}

	ret = cc1120_cmd_strobe(priv, CC112X_SFTX);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SFTX err %d\n", ret);
		return ret;
	}

	ret = cc1120_cmd_strobe(priv, CC112X_SRX);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SRX err %d\n", ret);
		return ret;
	}

	dev_info(&priv->spi->dev,"cc1120_start_debug and start rx %d\n", ret);

	return ret;
}

/**
 * @brief cc1120_stop -- Enter SLEEP mode when CSn is de-asserted
 */
static void cc1120_stop(struct ieee802154_hw *hw)
{
	/**
	 * Reference to Zephyr demo
	 */
	cc1120_cmd_strobe(hw->priv, CC112X_SPWD);
}

/**
 * @brief cc1120_tx -- Enable TX
 */
static int cc1120_tx(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	int ret = 0;
	struct cc1120_private *priv = hw->priv;
	unsigned long flags;
	uint8_t tx_fifo_num = 0;
	uint8_t tx_buff[128] = {0};

	dev_info(&priv->spi->dev,"cc1120_tx , tx_len = 0x%02x\n",skb->len);

	tx_buff[0] = (uint8_t)skb->len;
	memcpy(&tx_buff[1],skb->data,skb->len);

	dev_dump(tx_buff, (skb->len + 1));

	ret = cc1120_cmd_strobe(hw->priv, CC112X_SIDLE);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SIDLE err %d\n", ret);
		goto out;
	}

	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 1;
	spin_unlock_irqrestore(&priv->lock, flags);

	ret = cc1120_write_txfifo(hw->priv, tx_buff, (skb->len + 1));
	if(ret)
	{
		dev_err(&priv->spi->dev,"write_txfifo CC1120_PHY_HDR_LEN  err %d\n", ret);
		goto out;
	}

	cc1120_rd_reg(priv, CC112X_NUM_TXBYTES, &tx_fifo_num, 1);

	if(tx_fifo_num != (skb->len + 1))
	{
		dev_err(&priv->spi->dev,"txfifo check len err indicate %d ,actually %d \n", tx_fifo_num,(skb->len+1));
		goto out;
	}

	/* Start tx */
	ret = cc1120_cmd_strobe(hw->priv, CC112X_STX);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_STX err, Cannot start transmission%d\n", ret);
		goto out;
	}

	/* wait previous one done */
	ret = wait_for_completion_interruptible(&priv->tx_complete);
	if (ret < 0)
		goto out;

	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 0;
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;

out:
	dev_err(&priv->spi->dev,"cc1120_tx fail,flush txfifo and turn rx on\n\n");
	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 0;
	spin_unlock_irqrestore(&priv->lock, flags);

	cc1120_cmd_strobe(priv, CC112X_SFTX);
	cc1120_cmd_strobe(priv, CC112X_SRX);

	return ret;
}

static int cc1120_tx_debug(struct cc1120_private *priv, struct sk_buff *skb)
{
	/**
	 * Reference to Zephyr demo
	 */
	int ret = 0;
	unsigned long flags;
	uint8_t tx_fifo_num = 0;
	uint8_t tx_buff[128] = {0};

	dev_info(&priv->spi->dev,"cc1120_tx_debug , tx_len = 0x%02x\n",skb->len);

	tx_buff[0] = (uint8_t)skb->len;
	memcpy(&tx_buff[1],skb->data,skb->len);

	dev_dump(tx_buff, (skb->len + 1));

	ret = cc1120_cmd_strobe(priv, CC112X_SIDLE);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_SIDLE err %d\n", ret);
		goto out;
	}

	ret = cc1120_write_txfifo(priv, tx_buff, (skb->len + 1) );
	if(ret)
	{
		dev_err(&priv->spi->dev,"write_txfifo err %d\n", ret);
		goto out;
	}

	cc1120_rd_reg(priv, CC112X_NUM_TXBYTES, &tx_fifo_num, 1);

	if(tx_fifo_num != (skb->len + 1))
	{
		dev_err(&priv->spi->dev,"txfifo check len err indicate %d ,actually %d \n", tx_fifo_num,(skb->len+1));
	}

	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 1;
	spin_unlock_irqrestore(&priv->lock, flags);

	/* Start tx */
	ret = cc1120_cmd_strobe(priv, CC112X_STX);
	if(ret)
	{
		dev_err(&priv->spi->dev,"cmd_CC112X_STX err, Cannot start transmission%d\n", ret);
		goto out;
	}

	/* wait previous one done */
	ret = wait_for_completion_interruptible(&priv->tx_complete);
	if (ret < 0)
		goto out;

out:
	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 0;
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

/**
 * @brief cc1120_rx -- Enable RX,and read rxfifo
 */
static int cc1120_rx(struct cc1120_private *priv)
{

	#define RX_FIFO_ERROR        0x11	//MARCSTATE - MARC State 10001

	int ret = 0;
	uint8_t lqi = 0;
	struct sk_buff *skb;
	uint8_t rxbyte = 0;
	bool crc_ok;
	uint8_t rssi = 0;
	uint8_t marcstate = 0;
	uint8_t rx_buff[128] = {0};

	cc1120_rd_reg(priv, CC112X_NUM_RXBYTES, &rxbyte , 1);
	if(!rxbyte)
	{
		dev_err(&priv->spi->dev, "cc1120_rx no rx data\n");
		ret = -EINVAL;
		goto flush;
	}

	cc1120_rd_reg(priv, CC112X_MARCSTATE, &marcstate , 1);
	if((marcstate & 0x1f) == RX_FIFO_ERROR)
	{
		dev_err(&priv->spi->dev, "marcstate RX_FIFO_ERROR err\n");
		ret = -EINVAL;
		goto flush;
	}

	/**
	 * RX-fifo data format
	 *
	 * | payload-len | payload | RSSI | CRC-Flag + lqi |
	 * |     1B   	 |   nB    |   1  |        1B      |
	 *
	 */

	/* Read rx data */
	if (cc1120_read_rxfifo(priv,rx_buff, rxbyte))
	{
		dev_err(&priv->spi->dev, "frame reception failed\n");
		ret = -EINVAL;
		goto flush;
	}

	skb = dev_alloc_skb(rx_buff[0]);
	if (!skb)
	{
		dev_err(&priv->spi->dev, "dev_alloc_skb err\n");
		ret = -ENOMEM;
		goto flush;
	}

	/* extract valid data */
	memcpy(skb_put(skb,rx_buff[0]), &rx_buff[1], rx_buff[0]);

	dev_info(&priv->spi->dev, "Recv %d data:\n",rxbyte);
	dev_dump(rx_buff, rxbyte);

	/* Check FSC */
	crc_ok = rx_buff[rxbyte - 1] & BIT(7);
	if (!crc_ok)
	{
		dev_err(&priv->spi->dev, "cc1120_rx CRC check failed\n");
		kfree_skb(skb);
		ret = -EINVAL;
		goto flush;
	}

	rssi = rx_buff[rxbyte - 2];

	/* To calculate LQI, the lower 7 bits of the last byte (the
	 * correlation value provided by the radio) must be scaled to
	 * the range 0-255. According to section 20.6, the correlation
	 * value ranges from 50-110. Ideally this would be calibrated
	 * per hardware design, but we use roughly the datasheet values
	 * to get close enough while avoiding floating point.
	 * PKT_CFG1.APPEND_STATUS = 1 （default）
	 */
	lqi = rx_buff[rxbyte - 1] & 0x7f;

	ieee802154_rx_irqsafe(priv->hw, skb, lqi);

	dev_info(&priv->spi->dev, "RXFIFO: rxbyte = %d , rssi = 0x%02x , lqi = 0x%02x\n\n", rxbyte , rssi ,lqi);

	return 0;

flush:
	dev_err(&priv->spi->dev, "cc1120_rx err to flush\n\n");
	cc1120_cmd_strobe(priv, CC112X_SIDLE);
	cc1120_cmd_strobe(priv, CC112X_SFRX);
	/* Get back to rx */
	cc1120_cmd_strobe(priv, CC112X_SRX);

	return ret;
}

/* Energy Detection -- RSSI */
static int cc1120_ed(struct ieee802154_hw *hw, u8 *level)
{
	uint16_t rssi = 0;
	uint8_t reg_value = 0;
	struct cc1120_private *priv = hw->priv;
	uint8_t temp;

	cc1120_rd_reg(priv, CC112X_RSSI0, &reg_value, 1);
	if(!(reg_value & 0x01))
	{
		dev_err(&priv->spi->dev, "RSSI is not valid\n");
		return -EINVAL;
	}

	temp = (reg_value >> 3);
	temp &= 0x0f;

	cc1120_rd_reg(priv, CC112X_RSSI1, &reg_value, 1);
	rssi |= reg_value;
	rssi = rssi << 4;
	rssi |= temp;

	dev_info(&priv->spi->dev,"RSSI = 0x%04x\n",rssi);

	*level = rssi/16;

	return 0;
}

/* See Section 9.12 - RF programming
 *
 * The given formula in datasheet cannot be simply applied here, where CPU
 * limits us to unsigned integers of 32 bits. Instead, "slicing" it to
 * parts that fits in such limit is a solution which is applied below.
 *
 * The original formula being (freqoff is neglegted):
 * Freq = ( RF * Lo_Div * 2^16 ) / Xtal
 *
 * RF and Xtal are, from here, expressed in KHz.
 *
 * It first calculates the targeted RF with given ChanCenterFreq0, channel
 * spacing and the channel number.
 *
 * The calculation will slice the targeted RF by multiple of 10:
 * 10^n where n is in [5, 3]. The rest, below 1000, is taken at once.
 * Let's take the 434000 KHz RF for instance:
 * it will be "sliced" in 3 parts: 400000, 30000, 4000.
 * Or the 169406 KHz RF, 4 parts: 100000, 60000, 9000, 406.
 *
 * This permits also to play with Xtal to keep the result big enough to avoid
 * losing precision. A factor - growing as much as Xtal decrease -  is then
 * applied to get to the proper result. Which one is rounded to the nearest
 * integer, again to get a bit better precision.
 *
 * In the end, this algorithm below works for all the supported bands by CC1200.
 * User does not need to pass anything extra besides the nominal settings: no
 * pre-computed part or else.
 */
static uint32_t rf_evaluate_freq_setting(struct cc1120_private *priv, uint32_t chan)
{
	/**
	 * Reference to Zephyr demo
	 */
	uint32_t xtal = 1;	//or 32MHz
	uint32_t mult_10 = 100000U;
	uint32_t factor = 1U;
	uint32_t freq = 0U;
	uint32_t rf, lo_div;
	uint8_t reg_value = 0;

	rf = priv->p_rf_settings->chan_center_freq0 + ((chan * (uint32_t)priv->p_rf_settings->channel_spacing) / 10U);

	cc1120_rd_reg(priv, CC112X_FS_CFG, &reg_value, 1);
	lo_div = (reg_value & 0x0f) << 1;

	dev_info(&priv->spi->dev,"Calculating freq for %u KHz RF (%u)\n", rf, lo_div);

	while (rf > 0)
	{
		uint32_t hz, freq_tmp, rst;

		if (rf < 1000) {
			hz = rf;
		} else {
			hz = rf / mult_10;
			hz *= mult_10;
		}

		if (hz < 1000) {
			freq_tmp = (hz * lo_div * 65536U) / xtal;
		} else {
			freq_tmp = ((hz * lo_div) / xtal) * 65536U;
		}

		rst = freq_tmp % factor;
		freq_tmp /= factor;

		if (factor > 1 && (rst/(factor/10U)) > 5) {
			freq_tmp++;
		}

		freq += freq_tmp;

		factor *= 10U;
		mult_10 /= 10U;
		xtal /= 10U;
		rf -= hz;
	}

	return freq;
}

static int cc1120_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	/* Unlike usual 15.4 chips, cc1200 is closer to a bare metal radio modem
	 * and thus does not provide any means to select a channel directly, but
	 * requires instead that one calculates and configures the actual
	 * targeted frequency for the requested channel.
	 *
	 * See rf_evaluate_freq_setting() above.
	 */
	struct cc1120_private *priv = hw->priv;
	uint32_t freq = rf_evaluate_freq_setting(priv, channel);
	int ret = 0;
	uint8_t wr_value = 0;

	wr_value = (freq >> 16) & 0xff;
	ret = cc1120_wr_reg(priv, CC112X_FREQ2, &wr_value, 1);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev, "write CC112X_FREQ2 err %d\n",ret);
		ret = -EINVAL;
		return ret;
	}

	wr_value = (freq >> 8) & 0xff;
	ret = cc1120_wr_reg(priv, CC112X_FREQ1, &wr_value, 1);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev, "write CC112X_FREQ1 err %d\n",ret);
		ret = -EINVAL;
		return ret;
	}

	wr_value = freq & 0xff;
	ret = cc1120_wr_reg(priv, CC112X_FREQ0, &wr_value, 1);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev, "write CC112X_FREQ0 err %d\n",ret);
		ret = -EINVAL;
		return ret;
	}

	ret = cc1120_rf_calibrate(priv);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev, "cc1120_rf_calibrate err %d\n",ret);
	}

	return ret;
}

/* Range: -8 ~ 16 */
static const s32 cc1120_powers[] =
{
	1600, 1400, 1000, 800, 400, 0 , -200, -400, -800,
};

static const s32 cc1120_cca_ed_levels[] =
{
	1000, 700, 300, 0, -300, -700,-1000,
};

static int cc1120_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
	struct cc1120_private *priv = hw->priv;
	int ret = 0;
	uint8_t reg_value;
	uint8_t i = 0;

	dev_info(&priv->spi->dev, "cc1120_set_txpower be called\n");

	for(i = 0; i < ARRAY_SIZE(cc1120_powers);i++)
	{
		if(mbm == cc1120_powers[i])
			break;
	}

	if(i >=  ARRAY_SIZE(cc1120_powers))
	{
		dev_err(&priv->spi->dev, "cc1120_set_txpower err ,invalid parameter\n");
		return -EINVAL;
	}

	s32 dbm = mbm/100;
	/**
	 * Output Power = (PA POWER RAMP + 1)/2 - 18
	 */
	s32 t_dbm = (dbm + 18) * 2 - 1;

	reg_value = (uint8_t)t_dbm;

	reg_value |= 0x40;

	ret = cc1120_wr_reg(priv, CC112X_PA_CFG2, &reg_value, 1);
	dev_info(&priv->spi->dev, "cc1120_set_txpower dbm = %d, reg_val = 0x%02x , ret = %d\n",dbm,reg_value,ret);

	return ret;
}

static int cc1120_set_cca_mode(struct ieee802154_hw *hw,const struct wpan_phy_cca *cca)
{
	int ret = 0;
	uint8_t reg_value = 0x10;
	struct cc1120_private *priv = hw->priv;

	/* Indicates clear channel when RSSI is below threshold and ETSI LBT requirements are met*/
	ret = cc1120_wr_reg(priv, CC112X_PKT_CFG2, &reg_value, 1);
	if(ret < 0)
	{
		dev_err(&priv->spi->dev, "cc1120_set_cca_mode err\n");
	}

	return ret;
}

static int cc1120_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	int ret = 0;
	uint8_t reg_value;
	struct cc1120_private *priv = hw->priv;
	uint8_t i = 0;

	for(i = 0; i < ARRAY_SIZE(cc1120_cca_ed_levels);i++)
	{
		if(mbm == cc1120_cca_ed_levels[i])
			break;
	}

	if(i >=  ARRAY_SIZE(cc1120_cca_ed_levels))
	{
		dev_err(&priv->spi->dev, "cc1120_set_cca_ed_level err\n");
		return -EINVAL;
	}

	s32 dbm = mbm/100;
	s8 t_dbm = (s8)dbm;

	reg_value = (~t_dbm)+1;

	/* Indicates clear channel when RSSI is below threshold */
	ret = cc1120_wr_reg(priv, CC112X_AGC_CS_THR, &reg_value, 1);
	dev_info(&priv->spi->dev, "cc1120_set_cca_ed_level dbm = %d, reg_val = 0x%02x , ret = %d\n",t_dbm,reg_value,ret);

	return ret;
}

static const struct ieee802154_ops cc1120_ops = {
	.owner = THIS_MODULE,
	.start = cc1120_start,
	.stop = cc1120_stop,
	.xmit_sync = cc1120_tx,
	.ed = cc1120_ed,
	.set_channel = cc1120_set_channel,
	.set_txpower = cc1120_set_txpower,
	.set_cca_mode = cc1120_set_cca_mode,
	.set_cca_ed_level = cc1120_set_cca_ed_level,
};

static int cc1120_ieee802154_register(struct cc1120_private *priv)
{
	int ret = -ENOMEM;

	priv->hw = ieee802154_alloc_hw(sizeof(*priv), &cc1120_ops);
	if (!priv->hw)
	{
		goto err_ret;
	}

	priv->hw->priv = priv;
	priv->hw->parent = &priv->spi->dev;
	priv->hw->extra_tx_headroom = 0;

	ieee802154_random_extended_addr(&priv->hw->phy->perm_extended_addr);
	//priv->hw->phy->perm_extended_addr = 0x04030201004b1200;

	/* We use 902~928MHz support only channels 1-10 */
	priv->hw->phy->supported.channels[0] = 0x7Fe;

	/*take care of this flag*/
	//priv->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM;

	priv->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER | WPAN_PHY_FLAG_CCA_ED_LEVEL | WPAN_PHY_FLAG_CCA_MODE;

	priv->hw->phy->supported.tx_powers = cc1120_powers;
	priv->hw->phy->supported.tx_powers_size = ARRAY_SIZE(cc1120_powers);
	priv->hw->phy->transmit_power = priv->hw->phy->supported.tx_powers[4];


	priv->hw->phy->supported.cca_ed_levels = cc1120_cca_ed_levels;
	priv->hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(cc1120_cca_ed_levels);
	priv->hw->phy->cca_ed_level = priv->hw->phy->supported.cca_ed_levels[6];

	priv->hw->phy->cca.mode = NL802154_CCA_ENERGY;

	priv->hw->phy->supported.min_frame_retries = 1;
	priv->hw->phy->supported.max_frame_retries = 5;

	/* use for LBT */
	priv->hw->phy->supported.min_csma_backoffs = 0;
	priv->hw->phy->supported.max_csma_backoffs = 3;

	priv->hw->phy->current_channel = 1;

	ret = ieee802154_register_hw(priv->hw);
	if (ret)
		goto err_free_device;

	return 0;

err_free_device:
	dev_info(&priv->spi->dev, "ieee802154_register_hw err , ret = %d\n",ret);
	ieee802154_free_hw(priv->hw);
err_ret:
	return ret;

	return 0;
}

static void cc1120_fifop_irqwork(struct work_struct *work)
{
	struct 	cc1120_private *priv = container_of(work, struct cc1120_private, fifop_irqwork);

	dev_info(&priv->spi->dev, "cc1120_fifop_irqwork be called , is_tx = %d\n",priv->is_tx);

	if(!priv->is_tx)
	{
		dev_info(&priv->spi->dev, "rx_done\n");
		cc1120_rx(priv);
	}
	else
	{
		dev_info(&priv->spi->dev, "tx_done\n\n");
		complete(&priv->tx_complete);
	}
}

static irqreturn_t cc1120_isr(int irq, void *data)
{
	struct cc1120_private *priv = data;

	schedule_work(&priv->fifop_irqwork);

	return IRQ_HANDLED;
}

static int cc1120_get_platform_data(struct spi_device *spi, struct cc1120_platform_data *pdata)
{
	struct device_node *np = spi->dev.of_node;
	struct cc1120_private *priv = spi_get_drvdata(spi);

	if (!np)
	{
		struct cc1120_platform_data *spi_pdata = spi->dev.platform_data;

		if (!spi_pdata)
			return -ENOENT;
		*pdata = *spi_pdata;
		priv->fifo_pin = pdata->pin_interrupt;
		return 0;
	}

	/* Get gpios */
	pdata->pin_interrupt = of_get_named_gpio(np,"dio3-gpios",0);
	if(!pdata->pin_interrupt)
	{
		dev_warn(&spi->dev, "Unable to get pin_dio3");
	}
	else
	{
		priv->fifo_pin = pdata->pin_interrupt;
	}
	pdata->pin_hgm = of_get_named_gpio(np,"hgm-gpios",0);
	if(!pdata->pin_hgm)
	{
		dev_warn(&spi->dev, "Unable to get pin_hgm");
	}
	pdata->pin_rst = of_get_named_gpio(np,"reset-gpios",0);
	if(!pdata->pin_rst)
	{
		dev_warn(&spi->dev, "Unable to get pin_rst");
	}
	pdata->pin_tcxo = of_get_named_gpio(np,"tcxo-gpios",0);
	if(!pdata->pin_tcxo)
	{
		dev_warn(&spi->dev, "Unable to get pin_tcxo");
	}

	return 0;
}

static ssize_t cc1120_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cc1120_private *priv = dev_get_drvdata(dev);
	uint8_t tx_buf[1] = {};
	uint8_t rx_buf[1] = {};
	dev_info(dev, "cc1120_debug_show v2.6\n");

	cc1120_rd_reg(priv, CC112X_IOCFG3, rx_buf,1);
	dev_info(dev,"read 0x%02x value 0x%02x before\n",CC112X_IOCFG3,rx_buf[0]);

	tx_buf[0] = 0x88;
	cc1120_rd_reg(priv, CC112X_SYNC3, rx_buf,1);
	dev_info(dev,"read 0x%02x value 0x%02x before\n",CC112X_SYNC3,rx_buf[0]);
	cc1120_wr_reg(priv, CC112X_SYNC3, tx_buf,1);
	cc1120_rd_reg(priv, CC112X_SYNC3, rx_buf,1);
	dev_info(dev,"read 0x%02x value 0x%02x after\n",CC112X_SYNC3,rx_buf[0]);
	return 0;
}

static ssize_t cc1120_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cc1120_private *priv = dev_get_drvdata(dev);

	uint8_t tx_buf[1] = {};
	uint8_t rx_buf[1] = {};
	dev_info(dev, "cc1120_debug_store\n");

	tx_buf[0] = 0x88;
	cc1120_rd_reg(priv, CC112X_SYNC3, rx_buf, 1);
	dev_info(dev,"read 0x%02x value 0x%02x before\n",CC112X_SYNC3,rx_buf[0]);

	cc1120_wr_reg(priv, CC112X_SYNC3, tx_buf, 1);

	cc1120_rd_reg(priv, CC112X_SYNC3, rx_buf, 1);
	dev_info(dev,"read 0x%02x value 0x%02x after\n",CC112X_SYNC3,rx_buf[0]);

	tx_buf[0] = 0x15;
	cc1120_rd_reg(priv, CC112X_DCFILTOFFSET_I0, rx_buf, 1);
	dev_info(dev,"read 0x%04x value 0x%02x before\n",CC112X_DCFILTOFFSET_I0,rx_buf[0]);

	cc1120_wr_reg(priv, CC112X_DCFILTOFFSET_I0, tx_buf, 1);

	cc1120_rd_reg(priv, CC112X_DCFILTOFFSET_I0, rx_buf, 1);
	dev_info(dev,"read 0x%04x value 0x%02x after\n",CC112X_DCFILTOFFSET_I0,rx_buf[0]);

	return count;
}
static DEVICE_ATTR(test, 0644, cc1120_debug_show, cc1120_debug_store);

static ssize_t cc1120_tx_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct cc1120_private *priv = dev_get_drvdata(dev);
	uint8_t tx_buf[7] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07};
	int ret = 0;
	struct sk_buff sf_buffer;
	dev_info(dev, "cc1120_tx_show v1.1\n");

	sf_buffer.data = tx_buf;
	sf_buffer.len = ARRAY_SIZE(tx_buf);

	ret = cc1120_tx_debug(priv, &sf_buffer);

	dev_info(dev, "cc1120_tx_show ret = %d\n",ret);

	return ret;
}
static DEVICE_ATTR(tx, 0644, cc1120_tx_show, NULL);

static struct attribute *cc1120_attributes[] = {
	&dev_attr_test.attr,
	&dev_attr_tx.attr,
	NULL
};

static struct attribute_group cc1120_debug_attr_group = {
	.name   = "cc1120_debug",
	.attrs  = cc1120_attributes,
};

static int cc1120_probe(struct spi_device *spi)
{
	struct cc1120_private *priv;
	struct cc1120_platform_data pdata;
	int ret = 0;

	dev_info(&spi->dev,"cc1120_probe match start \n");

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spi_set_drvdata(spi, priv);

	ret = cc1120_get_platform_data(spi, &pdata);
	if (ret < 0) {
		dev_err(&spi->dev, "no platform data\n");
		return -EINVAL;
	}

	priv->spi = spi;
	priv->buf = devm_kzalloc(&spi->dev,
				 SPI_COMMAND_BUFFER, GFP_KERNEL);
	if (!priv->buf)
		return -ENOMEM;

	mutex_init(&priv->buffer_mutex);
	INIT_WORK(&priv->fifop_irqwork, cc1120_fifop_irqwork);
	spin_lock_init(&priv->lock);
	init_completion(&priv->tx_complete);

	/* Assumption that CC1900 is not connected */
	priv->amplified = true;

	/**
	 * There some bug in this usage, Pending investigation
	 */
#if 0
	pdata = devm_kzalloc(&spi->dev, sizeof(struct cc1120_platform_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* Request all the gpio's */
    pdata->interrrupt = devm_gpiod_get_index_optional(&spi->dev, "interrrupt", 0, GPIOD_OUT_LOW);
	if (IS_ERR(pdata->interrrupt))
	{
		dev_err(&spi->dev, "interrrupt gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata->reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 0, GPIOD_OUT_LOW);
	if (IS_ERR(pdata->reset))
	{
		dev_err(&spi->dev, "reset gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata->hgm = devm_gpiod_get_index_optional(&spi->dev, "hgm", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->hgm))
	{
		dev_err(&spi->dev, "hgm gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata->tcxo = devm_gpiod_get_index_optional(&spi->dev, "tcxo", 0, GPIOD_OUT_LOW);
	if (IS_ERR(pdata->tcxo))
	{
		dev_err(&spi->dev, "tcxo gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
#endif

	/* Request all the gpio's */
	if (!gpio_is_valid(pdata.pin_interrupt))
	{
		dev_err(&spi->dev, "pin_interrupt gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
	ret = devm_gpio_request_one(&spi->dev, pdata.pin_interrupt,
				    GPIOF_IN, "interrupt");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.pin_tcxo))
	{
		dev_err(&spi->dev, "pin_tcxo gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
	ret = devm_gpio_request_one(&spi->dev, pdata.pin_tcxo,
				    GPIOF_OUT_INIT_LOW, "tcxo");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.pin_hgm))
	{
		dev_err(&spi->dev, "pin_hgm gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
	ret = devm_gpio_request_one(&spi->dev, pdata.pin_hgm,
				    GPIOF_OUT_INIT_HIGH, "hgm");
	if (ret)
		goto err_hw_init;

	if (!gpio_is_valid(pdata.pin_rst))
	{
		dev_err(&spi->dev, "pin_rst gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
	ret = devm_gpio_request_one(&spi->dev, pdata.pin_rst,
				    GPIOF_OUT_INIT_HIGH, "rst");
	if (ret)
		goto err_hw_init;

	/* reset hardware modules */
	gpio_set_value( pdata.pin_rst , 0 );
	mdelay(100);
	gpio_set_value( pdata.pin_rst , 1 );
	mdelay(100);

	ret = spi_setup(spi);
	if(ret)
	{
		dev_err(&spi->dev, "spi_setup error %d\n",ret);
		goto err_hw_init;
	}

	/* Maybe we can choose group's index  by dts */
	priv->p_rf_settings = &g_cc1120_rf_settings_group[0];
	ret = cc1120_hw_init(priv);
	if (ret)
		goto err_hw_init;

	/* Calibrate module*/
	cc1120_rf_calibrate(priv);

	/* Set up interrupt */
	ret = devm_request_irq(&spi->dev,
			       gpio_to_irq(pdata.pin_interrupt),
			       cc1120_isr,
			       IRQF_TRIGGER_FALLING,
			       dev_name(&spi->dev),
			       priv);
	if (ret)
	{
		dev_err(&spi->dev, "could not get interrupt irq\n");
		goto err_hw_init;
	}

	/* add sysfs for debug*/
	ret = sysfs_create_group(&spi->dev.kobj, &cc1120_debug_attr_group);
	if (ret)
		goto err_hw_init;

	ret = cc1120_ieee802154_register(priv);
	if (ret)
		goto err_hw_init;

	dev_info(&spi->dev,"cc1120_probe match finish\n");
	return 0;

err_hw_init:
	mutex_destroy(&priv->buffer_mutex);
	flush_work(&priv->fifop_irqwork);
	sysfs_remove_group(&spi->dev.kobj, &cc1120_debug_attr_group);
	return ret;
}

static int cc1120_remove(struct spi_device *spi)
{
	struct cc1120_private *priv = spi_get_drvdata(spi);

	dev_info(&spi->dev,"cc1120_remove\n");

	mutex_destroy(&priv->buffer_mutex);
	flush_work(&priv->fifop_irqwork);
	sysfs_remove_group(&spi->dev.kobj, &cc1120_debug_attr_group);

	if(priv->hw)
	{
		ieee802154_unregister_hw(priv->hw);
		ieee802154_free_hw(priv->hw);
	}

	return 0;
}

static const struct spi_device_id cc1120_ids[] = {
	{"cc1120", 0},
	{},
};
MODULE_DEVICE_TABLE(spi, cc1120_ids);

static const struct of_device_id cc1120_of_ids[] = {
	{.compatible = "ti,cc1120", },
	{},
};
MODULE_DEVICE_TABLE(of, cc1120_of_ids);

/* SPI driver structure */
static struct spi_driver cc1120_driver = {
	.driver = {
		.name = "cc1120",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cc1120_of_ids),
	},
	.id_table = cc1120_ids,
	.probe = cc1120_probe,
	.remove = cc1120_remove,
};
module_spi_driver(cc1120_driver);

MODULE_AUTHOR("Baozhu Zuo <zuobaozhu@gmail.com>");
MODULE_DESCRIPTION("CC1120 Transceiver Driver");
MODULE_LICENSE("GPL v2");
