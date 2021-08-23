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

#define	SPI_COMMAND_BUFFER	3
/******************************************************************************
 * CONSTANTS
 */

/* configuration registers */
#define CC112X_IOCFG3                   0x0000
#define CC112X_IOCFG2                   0x0001
#define CC112X_IOCFG1                   0x0002
#define CC112X_IOCFG0                   0x0003
#define CC112X_SYNC3                    0x0004
#define CC112X_SYNC2                    0x0005
#define CC112X_SYNC1                    0x0006
#define CC112X_SYNC0                    0x0007
#define CC112X_SYNC_CFG1                0x0008
#define CC112X_SYNC_CFG0                0x0009
#define CC112X_DEVIATION_M              0x000A
#define CC112X_MODCFG_DEV_E             0x000B
#define CC112X_DCFILT_CFG               0x000C
#define CC112X_PREAMBLE_CFG1            0x000D
#define CC112X_PREAMBLE_CFG0            0x000E
#define CC112X_FREQ_IF_CFG              0x000F
#define CC112X_IQIC                     0x0010
#define CC112X_CHAN_BW                  0x0011
#define CC112X_MDMCFG1                  0x0012
#define CC112X_MDMCFG0                  0x0013
#define CC112X_SYMBOL_RATE2             0x0014
#define CC112X_SYMBOL_RATE1             0x0015
#define CC112X_SYMBOL_RATE0             0x0016
#define CC112X_AGC_REF                  0x0017
#define CC112X_AGC_CS_THR               0x0018
#define CC112X_AGC_GAIN_ADJUST          0x0019
#define CC112X_AGC_CFG3                 0x001A
#define CC112X_AGC_CFG2                 0x001B
#define CC112X_AGC_CFG1                 0x001C
#define CC112X_AGC_CFG0                 0x001D
#define CC112X_FIFO_CFG                 0x001E
#define CC112X_DEV_ADDR                 0x001F
#define CC112X_SETTLING_CFG             0x0020
#define CC112X_FS_CFG                   0x0021
#define CC112X_WOR_CFG1                 0x0022
#define CC112X_WOR_CFG0                 0x0023
#define CC112X_WOR_EVENT0_MSB           0x0024
#define CC112X_WOR_EVENT0_LSB           0x0025
#define CC112X_PKT_CFG2                 0x0026
#define CC112X_PKT_CFG1                 0x0027
#define CC112X_PKT_CFG0                 0x0028
#define CC112X_RFEND_CFG1               0x0029
#define CC112X_RFEND_CFG0               0x002A
#define CC112X_PA_CFG2                  0x002B
#define CC112X_PA_CFG1                  0x002C
#define CC112X_PA_CFG0                  0x002D
#define CC112X_PKT_LEN                  0x002E

/* Extended Configuration Registers */
#define CC112X_IF_MIX_CFG               0x2F00
#define CC112X_FREQOFF_CFG              0x2F01
#define CC112X_TOC_CFG                  0x2F02
#define CC112X_MARC_SPARE               0x2F03
#define CC112X_ECG_CFG                  0x2F04
#define CC112X_CFM_DATA_CFG             0x2F05
#define CC112X_EXT_CTRL                 0x2F06
#define CC112X_RCCAL_FINE               0x2F07
#define CC112X_RCCAL_COARSE             0x2F08
#define CC112X_RCCAL_OFFSET             0x2F09
#define CC112X_FREQOFF1                 0x2F0A
#define CC112X_FREQOFF0                 0x2F0B
#define CC112X_FREQ2                    0x2F0C
#define CC112X_FREQ1                    0x2F0D
#define CC112X_FREQ0                    0x2F0E
#define CC112X_IF_ADC2                  0x2F0F
#define CC112X_IF_ADC1                  0x2F10
#define CC112X_IF_ADC0                  0x2F11
#define CC112X_FS_DIG1                  0x2F12
#define CC112X_FS_DIG0                  0x2F13
#define CC112X_FS_CAL3                  0x2F14
#define CC112X_FS_CAL2                  0x2F15
#define CC112X_FS_CAL1                  0x2F16
#define CC112X_FS_CAL0                  0x2F17
#define CC112X_FS_CHP                   0x2F18
#define CC112X_FS_DIVTWO                0x2F19
#define CC112X_FS_DSM1                  0x2F1A
#define CC112X_FS_DSM0                  0x2F1B
#define CC112X_FS_DVC1                  0x2F1C
#define CC112X_FS_DVC0                  0x2F1D
#define CC112X_FS_LBI                   0x2F1E
#define CC112X_FS_PFD                   0x2F1F
#define CC112X_FS_PRE                   0x2F20
#define CC112X_FS_REG_DIV_CML           0x2F21
#define CC112X_FS_SPARE                 0x2F22
#define CC112X_FS_VCO4                  0x2F23
#define CC112X_FS_VCO3                  0x2F24
#define CC112X_FS_VCO2                  0x2F25
#define CC112X_FS_VCO1                  0x2F26
#define CC112X_FS_VCO0                  0x2F27
#define CC112X_GBIAS6                   0x2F28
#define CC112X_GBIAS5                   0x2F29
#define CC112X_GBIAS4                   0x2F2A
#define CC112X_GBIAS3                   0x2F2B
#define CC112X_GBIAS2                   0x2F2C
#define CC112X_GBIAS1                   0x2F2D
#define CC112X_GBIAS0                   0x2F2E
#define CC112X_IFAMP                    0x2F2F
#define CC112X_LNA                      0x2F30
#define CC112X_RXMIX                    0x2F31
#define CC112X_XOSC5                    0x2F32
#define CC112X_XOSC4                    0x2F33
#define CC112X_XOSC3                    0x2F34
#define CC112X_XOSC2                    0x2F35
#define CC112X_XOSC1                    0x2F36
#define CC112X_XOSC0                    0x2F37
#define CC112X_ANALOG_SPARE             0x2F38
#define CC112X_PA_CFG3                  0x2F39
#define CC112X_IRQ0M                    0x2F3F
#define CC112X_IRQ0F                    0x2F40 

/* Status Registers */
#define CC112X_WOR_TIME1                0x2F64
#define CC112X_WOR_TIME0                0x2F65
#define CC112X_WOR_CAPTURE1             0x2F66
#define CC112X_WOR_CAPTURE0             0x2F67
#define CC112X_BIST                     0x2F68
#define CC112X_DCFILTOFFSET_I1          0x2F69
#define CC112X_DCFILTOFFSET_I0          0x2F6A
#define CC112X_DCFILTOFFSET_Q1          0x2F6B
#define CC112X_DCFILTOFFSET_Q0          0x2F6C
#define CC112X_IQIE_I1                  0x2F6D
#define CC112X_IQIE_I0                  0x2F6E
#define CC112X_IQIE_Q1                  0x2F6F
#define CC112X_IQIE_Q0                  0x2F70
#define CC112X_RSSI1                    0x2F71
#define CC112X_RSSI0                    0x2F72
#define CC112X_MARCSTATE                0x2F73
#define CC112X_LQI_VAL                  0x2F74
#define CC112X_PQT_SYNC_ERR             0x2F75
#define CC112X_DEM_STATUS               0x2F76
#define CC112X_FREQOFF_EST1             0x2F77
#define CC112X_FREQOFF_EST0             0x2F78
#define CC112X_AGC_GAIN3                0x2F79
#define CC112X_AGC_GAIN2                0x2F7A
#define CC112X_AGC_GAIN1                0x2F7B
#define CC112X_AGC_GAIN0                0x2F7C
#define CC112X_CFM_RX_DATA_OUT          0x2F7D
#define CC112X_CFM_TX_DATA_IN           0x2F7E
#define CC112X_ASK_SOFT_RX_DATA         0x2F7F
#define CC112X_RNDGEN                   0x2F80
#define CC112X_MAGN2                    0x2F81
#define CC112X_MAGN1                    0x2F82
#define CC112X_MAGN0                    0x2F83
#define CC112X_ANG1                     0x2F84
#define CC112X_ANG0                     0x2F85
#define CC112X_CHFILT_I2                0x2F86
#define CC112X_CHFILT_I1                0x2F87
#define CC112X_CHFILT_I0                0x2F88
#define CC112X_CHFILT_Q2                0x2F89
#define CC112X_CHFILT_Q1                0x2F8A
#define CC112X_CHFILT_Q0                0x2F8B
#define CC112X_GPIO_STATUS              0x2F8C
#define CC112X_FSCAL_CTRL               0x2F8D
#define CC112X_PHASE_ADJUST             0x2F8E
#define CC112X_PARTNUMBER               0x2F8F
#define CC112X_PARTVERSION              0x2F90
#define CC112X_SERIAL_STATUS            0x2F91
#define CC112X_MODEM_STATUS1            0x2F92
#define CC112X_MODEM_STATUS0            0x2F93
#define CC112X_MARC_STATUS1             0x2F94
#define CC112X_MARC_STATUS0             0x2F95
#define CC112X_PA_IFAMP_TEST            0x2F96
#define CC112X_FSRF_TEST                0x2F97
#define CC112X_PRE_TEST                 0x2F98
#define CC112X_PRE_OVR                  0x2F99
#define CC112X_ADC_TEST                 0x2F9A
#define CC112X_DVC_TEST                 0x2F9B
#define CC112X_ATEST                    0x2F9C
#define CC112X_ATEST_LVDS               0x2F9D
#define CC112X_ATEST_MODE               0x2F9E
#define CC112X_XOSC_TEST1               0x2F9F
#define CC112X_XOSC_TEST0               0x2FA0  
                                        
#define CC112X_RXFIRST                  0x2FD2   
#define CC112X_TXFIRST                  0x2FD3   
#define CC112X_RXLAST                   0x2FD4 
#define CC112X_TXLAST                   0x2FD5 
#define CC112X_NUM_TXBYTES              0x2FD6  /* Number of bytes in TXFIFO */ 
#define CC112X_NUM_RXBYTES              0x2FD7  /* Number of bytes in RXFIFO */
#define CC112X_FIFO_NUM_TXBYTES         0x2FD8  
#define CC112X_FIFO_NUM_RXBYTES         0x2FD9  

                                                                                                                                                
/* DATA FIFO Access */
#define CC112X_SINGLE_TXFIFO            0x003F      /*  TXFIFO  - Single accecss to Transmit FIFO */
#define CC112X_BURST_TXFIFO             0x007F      /*  TXFIFO  - Burst accecss to Transmit FIFO  */
#define CC112X_SINGLE_RXFIFO            0x00BF      /*  RXFIFO  - Single accecss to Receive FIFO  */
#define CC112X_BURST_RXFIFO             0x00FF      /*  RXFIFO  - Busrrst ccecss to Receive FIFO  */

#define CC112X_LQI_CRC_OK_BM            0x80
#define CC112X_LQI_EST_BM               0x7F



/* Command strobe registers */
#define CC112X_SRES                     0x30      /*  SRES    - Reset chip. */
#define CC112X_SFSTXON                  0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define CC112X_SXOFF                    0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define CC112X_SCAL                     0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define CC112X_SRX                      0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define CC112X_STX                      0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define CC112X_SIDLE                    0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define CC112X_SWOR                     0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define CC112X_SPWD                     0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define CC112X_SFRX                     0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define CC112X_SFTX                     0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define CC112X_SWORRST                  0x3C      /*  SWORRST - Reset real time clock. */
#define CC112X_SNOP                     0x3D      /*  SNOP    - No operation. Returns status byte. */
#define CC112X_AFC                      0x37      /*  AFC     - Automatic Frequency Correction */

/* Chip states returned in status byte */
#define CC112X_STATE_IDLE               0x00
#define CC112X_STATE_RX                 0x10
#define CC112X_STATE_TX                 0x20
#define CC112X_STATE_FSTXON             0x30
#define CC112X_STATE_CALIBRATE          0x40
#define CC112X_STATE_SETTLING           0x50
#define CC112X_STATE_RXFIFO_ERROR       0x60
#define CC112X_STATE_TXFIFO_ERROR       0x70




struct cc1120_platform_data {
	struct gpio_desc *interrrupt;
	struct gpio_desc *tcxo;
	struct gpio_desc *hgm;
	struct gpio_desc *reset;
};


/* Driver private information */
struct cc1120_private {
	struct spi_device *spi;		/* SPI device structure */
	struct ieee802154_hw *hw;	/* IEEE-802.15.4 device */
	u8 *buf;			/* SPI TX/Rx data buffer */
	struct mutex buffer_mutex;	/* SPI buffer mutex */
	bool is_tx;			/* Flag for sync b/w Tx and Rx */
	bool amplified;			/* Flag for CC2591 */
	int fifo_pin;			/* FIFO GPIO pin number */
	struct work_struct fifop_irqwork;/* Workqueue for FIFOP */
	spinlock_t lock;		/* Lock for is_tx*/
	struct completion tx_complete;	/* Work completion for Tx */
	bool promiscuous;               /* Flag for promiscuous mode */
};

/* Generic Functions */
static int
cc1120_cmd_strobe(struct cc1120_private *priv, u8 cmd)
{
#if 0
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer.len++] = cmd;
	dev_vdbg(&priv->spi->dev,
		 "command strobe buf[0] = %02x\n",
		 priv->buf[0]);

	ret = spi_sync(priv->spi, &msg);
	dev_vdbg(&priv->spi->dev,
		 "buf[0] = %02x\n", priv->buf[0]);
	mutex_unlock(&priv->buffer_mutex);

	return ret;
#endif
	return 0;
}

static int
cc1120_get_status(struct cc1120_private *priv, u8 *status)
{
#if 0
	int ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer.len++] = CC1120_CMD_SNOP;
	dev_vdbg(&priv->spi->dev,
		 "get status command buf[0] = %02x\n", priv->buf[0]);

	ret = spi_sync(priv->spi, &msg);
	if (!ret)
		*status = priv->buf[0];
	dev_vdbg(&priv->spi->dev,
		 "buf[0] = %02x\n", priv->buf[0]);
	mutex_unlock(&priv->buffer_mutex);

	return ret;
#endif
	return 0;
}

static int
cc1120_write_register(struct cc1120_private *priv, u8 reg, u8 value)
{
#if 0
	int status;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	mutex_lock(&priv->buffer_mutex);

	if (reg <= CC1120_FREG_MASK) {
		priv->buf[xfer.len++] = CC1120_CMD_REGISTER_WRITE | reg;
		priv->buf[xfer.len++] = value;
	} else {
		priv->buf[xfer.len++] = CC1120_CMD_MEMORY_WRITE;
		priv->buf[xfer.len++] = reg;
		priv->buf[xfer.len++] = value;
	}
	status = spi_sync(priv->spi, &msg);
	if (msg.status)
		status = msg.status;

	mutex_unlock(&priv->buffer_mutex);

	return status;
#endif 
	return 0;
}

static int
cc1120_write_ram(struct cc1120_private *priv, u16 reg, u8 len, u8 *data)
{
#if 0
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len        = 0,
		.tx_buf        = priv->buf,
		.rx_buf        = priv->buf,
	};

	struct spi_transfer xfer_buf = {
		.len = len,
		.tx_buf = data,
	};

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer_head.len++] = (CC1120_CMD_MEMORY_WRITE |
						((reg >> 8) & 0xff));
	priv->buf[xfer_head.len++] = reg & 0xff;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	status = spi_sync(priv->spi, &msg);
	dev_dbg(&priv->spi->dev, "spi status = %d\n", status);
	if (msg.status)
		status = msg.status;

	mutex_unlock(&priv->buffer_mutex);
	return status;
#endif 
	return 0;
}

static int
cc1120_read_register(struct cc1120_private *priv, u8 reg, u8 *data)
{
#if 0
	int status;
	struct spi_message msg;
	struct spi_transfer xfer1 = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};

	struct spi_transfer xfer2 = {
		.len = 1,
		.rx_buf = data,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer1, &msg);
	spi_message_add_tail(&xfer2, &msg);

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer1.len++] = CC1120_CMD_MEMORY_READ;
	priv->buf[xfer1.len++] = reg;

	status = spi_sync(priv->spi, &msg);
	dev_dbg(&priv->spi->dev,
		"spi status = %d\n", status);
	if (msg.status)
		status = msg.status;

	mutex_unlock(&priv->buffer_mutex);

	return status;
#endif 
	return 0;
}

static int
cc1120_write_txfifo(struct cc1120_private *priv, u8 pkt_len, u8 *data, u8 len)
{
#if 0
	int status;

	/* length byte must include FCS even
	 * if it is calculated in the hardware
	 */
	int len_byte = pkt_len;

	struct spi_message msg;

	struct spi_transfer xfer_head = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};
	struct spi_transfer xfer_len = {
		.len = 1,
		.tx_buf = &len_byte,
	};
	struct spi_transfer xfer_buf = {
		.len = len,
		.tx_buf = data,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_len, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer_head.len++] = CC1120_CMD_TXBUF;
	dev_vdbg(&priv->spi->dev,
		 "TX_FIFO cmd buf[0] = %02x\n", priv->buf[0]);

	status = spi_sync(priv->spi, &msg);
	dev_vdbg(&priv->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_vdbg(&priv->spi->dev, "status = %d\n", status);
	dev_vdbg(&priv->spi->dev, "buf[0] = %02x\n", priv->buf[0]);
	mutex_unlock(&priv->buffer_mutex);

	return status;
#endif
	return 0;
}

static int
cc1120_read_rxfifo(struct cc1120_private *priv, u8 *data, u8 len)
{
#if 0
	int status;
	struct spi_message msg;

	struct spi_transfer xfer_head = {
		.len = 0,
		.tx_buf = priv->buf,
		.rx_buf = priv->buf,
	};
	struct spi_transfer xfer_buf = {
		.len = len,
		.rx_buf = data,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	mutex_lock(&priv->buffer_mutex);
	priv->buf[xfer_head.len++] = CC1120_CMD_RXBUF;

	dev_vdbg(&priv->spi->dev, "read rxfifo buf[0] = %02x\n", priv->buf[0]);
	dev_vdbg(&priv->spi->dev, "buf[1] = %02x\n", priv->buf[1]);

	status = spi_sync(priv->spi, &msg);
	dev_vdbg(&priv->spi->dev, "status = %d\n", status);
	if (msg.status)
		status = msg.status;
	dev_vdbg(&priv->spi->dev, "status = %d\n", status);
	dev_vdbg(&priv->spi->dev,
		 "return status buf[0] = %02x\n", priv->buf[0]);
	dev_vdbg(&priv->spi->dev, "length buf[1] = %02x\n", priv->buf[1]);

	mutex_unlock(&priv->buffer_mutex);

	return status;
#endif 
	return 0;
}

static int cc1120_start(struct ieee802154_hw *hw)
{
	//return cc1120_cmd_strobe(hw->priv, CC1120_CMD_SRXON);
	return 0;
}

static void cc1120_stop(struct ieee802154_hw *hw)
{
	//cc1120_cmd_strobe(hw->priv, CC1120_CMD_SRFOFF);
	return 0;
}

static int
cc1120_tx(struct ieee802154_hw *hw, struct sk_buff *skb)
{
#if 0
	struct cc1120_private *priv = hw->priv;
	unsigned long flags;
	int rc;
	u8 status = 0;
	u8 pkt_len;

	/* In promiscuous mode we disable AUTOCRC so we can get the raw CRC
	 * values on RX. This means we need to manually add the CRC on TX.
	 */
	if (priv->promiscuous) {
		u16 crc = crc_ccitt(0, skb->data, skb->len);

		put_unaligned_le16(crc, skb_put(skb, 2));
		pkt_len = skb->len;
	} else {
		pkt_len = skb->len + 2;
	}

	rc = cc1120_cmd_strobe(priv, CC1120_CMD_SFLUSHTX);
	if (rc)
		goto err_tx;

	rc = cc1120_write_txfifo(priv, pkt_len, skb->data, skb->len);
	if (rc)
		goto err_tx;

	rc = cc1120_get_status(priv, &status);
	if (rc)
		goto err_tx;

	if (status & CC1120_STATUS_TX_UNDERFLOW) {
		dev_err(&priv->spi->dev, "cc1120 tx underflow exception\n");
		goto err_tx;
	}

	spin_lock_irqsave(&priv->lock, flags);
	WARN_ON(priv->is_tx);
	priv->is_tx = 1;
	spin_unlock_irqrestore(&priv->lock, flags);

	rc = cc1120_cmd_strobe(priv, CC1120_CMD_STXONCCA);
	if (rc)
		goto err;

	rc = wait_for_completion_interruptible(&priv->tx_complete);
	if (rc < 0)
		goto err;

	cc1120_cmd_strobe(priv, CC1120_CMD_SFLUSHTX);
	cc1120_cmd_strobe(priv, CC1120_CMD_SRXON);

	return rc;
err:
	spin_lock_irqsave(&priv->lock, flags);
	priv->is_tx = 0;
	spin_unlock_irqrestore(&priv->lock, flags);
err_tx:
	return rc;
#endif 
	return 0;
}

static int cc1120_rx(struct cc1120_private *priv)
{
#if 0
	u8 len = 0, lqi = 0, bytes = 1;
	struct sk_buff *skb;

	/* Read single length byte from the radio. */
	cc1120_read_rxfifo(priv, &len, bytes);

	if (!ieee802154_is_valid_psdu_len(len)) {
		/* Corrupted frame received, clear frame buffer by
		 * reading entire buffer.
		 */
		dev_dbg(&priv->spi->dev, "corrupted frame received\n");
		len = IEEE802154_MTU;
	}

	skb = dev_alloc_skb(len);
	if (!skb)
		return -ENOMEM;

	if (cc1120_read_rxfifo(priv, skb_put(skb, len), len)) {
		dev_dbg(&priv->spi->dev, "frame reception failed\n");
		kfree_skb(skb);
		return -EINVAL;
	}

	/* In promiscuous mode, we configure the radio to include the
	 * CRC (AUTOCRC==0) and we pass on the packet unconditionally. If not
	 * in promiscuous mode, we check the CRC here, but leave the
	 * RSSI/LQI/CRC_OK bytes as they will get removed in the mac layer.
	 */
	if (!priv->promiscuous) {
		bool crc_ok;

		/* Check if the CRC is valid. With AUTOCRC set, the most
		 * significant bit of the last byte returned from the CC1120
		 * is CRC_OK flag. See section 20.3.4 of the datasheet.
		 */
		crc_ok = skb->data[len - 1] & BIT(7);

		/* If we failed CRC drop the packet in the driver layer. */
		if (!crc_ok) {
			dev_dbg(&priv->spi->dev, "CRC check failed\n");
			kfree_skb(skb);
			return -EINVAL;
		}

		/* To calculate LQI, the lower 7 bits of the last byte (the
		 * correlation value provided by the radio) must be scaled to
		 * the range 0-255. According to section 20.6, the correlation
		 * value ranges from 50-110. Ideally this would be calibrated
		 * per hardware design, but we use roughly the datasheet values
		 * to get close enough while avoiding floating point.
		 */
		lqi = skb->data[len - 1] & 0x7f;
		if (lqi < 50)
			lqi = 50;
		else if (lqi > 113)
			lqi = 113;
		lqi = (lqi - 50) * 4;
	}

	ieee802154_rx_irqsafe(priv->hw, skb, lqi);

	dev_vdbg(&priv->spi->dev, "RXFIFO: %x %x\n", len, lqi);
#endif 
	return 0;
}

static int
cc1120_ed(struct ieee802154_hw *hw, u8 *level)
{
#if 0
	struct cc1120_private *priv = hw->priv;
	u8 status = 0xff;
	u8 rssi;
	int ret;

	ret = cc1120_read_register(priv, CC1120_RSSISTAT, &status);
	if (ret)
		return ret;

	if (status != RSSI_VALID)
		return -EINVAL;

	ret = cc1120_read_register(priv, CC1120_RSSI, &rssi);
	if (ret)
		return ret;

	/* level = RSSI(rssi) - OFFSET [dBm] : offset is 76dBm */
	*level = rssi - RSSI_OFFSET;
#endif
	return 0;
}

static int
cc1120_set_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
#if 0
	struct cc1120_private *priv = hw->priv;
	int ret;

	dev_dbg(&priv->spi->dev, "trying to set channel\n");

	WARN_ON(page != 0);
	WARN_ON(channel < CC1120_MINCHANNEL);
	WARN_ON(channel > CC1120_MAXCHANNEL);

	ret = cc1120_write_register(priv, CC1120_FREQCTRL,
				    11 + 5 * (channel - 11));

	return ret;
#endif 
	return 0;
}

static int
cc1120_filter(struct ieee802154_hw *hw,
	      struct ieee802154_hw_addr_filt *filt, unsigned long changed)
{
#if 0
	struct cc1120_private *priv = hw->priv;
	int ret = 0;

	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		u16 panid = le16_to_cpu(filt->pan_id);

		dev_vdbg(&priv->spi->dev, "%s called for pan id\n", __func__);
		ret = cc1120_write_ram(priv, CC1120RAM_PANID,
				       sizeof(panid), (u8 *)&panid);
	}

	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		dev_vdbg(&priv->spi->dev,
			 "%s called for IEEE addr\n", __func__);
		ret = cc1120_write_ram(priv, CC1120RAM_IEEEADDR,
				       sizeof(filt->ieee_addr),
				       (u8 *)&filt->ieee_addr);
	}

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		u16 addr = le16_to_cpu(filt->short_addr);

		dev_vdbg(&priv->spi->dev, "%s called for saddr\n", __func__);
		ret = cc1120_write_ram(priv, CC1120RAM_SHORTADDR,
				       sizeof(addr), (u8 *)&addr);
	}

	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		u8 frmfilt0;

		dev_vdbg(&priv->spi->dev,
			 "%s called for panc change\n", __func__);

		cc1120_read_register(priv, CC1120_FRMFILT0, &frmfilt0);

		if (filt->pan_coord)
			frmfilt0 |= FRMFILT0_PAN_COORDINATOR;
		else
			frmfilt0 &= ~FRMFILT0_PAN_COORDINATOR;

		ret = cc1120_write_register(priv, CC1120_FRMFILT0, frmfilt0);
	}

	return ret;
#endif
	return 0;
}

static inline int cc1120_set_tx_power(struct cc1120_private *priv, s32 mbm)
{
#if 0
	u8 power;

	switch (mbm) {
	case 500:
		power = 0xF7;
		break;
	case 300:
		power = 0xF2;
		break;
	case 200:
		power = 0xAB;
		break;
	case 100:
		power = 0x13;
		break;
	case 0:
		power = 0x32;
		break;
	case -200:
		power = 0x81;
		break;
	case -400:
		power = 0x88;
		break;
	case -700:
		power = 0x2C;
		break;
	case -1800:
		power = 0x03;
		break;
	default:
		return -EINVAL;
	}

	return cc1120_write_register(priv, CC1120_TXPOWER, power);
#endif 
	return 0;
}

static inline int cc1120_cc2591_set_tx_power(struct cc1120_private *priv,
					     s32 mbm)
{
#if 0
	u8 power;

	switch (mbm) {
	case 1700:
		power = 0xF9;
		break;
	case 1600:
		power = 0xF0;
		break;
	case 1400:
		power = 0xA0;
		break;
	case 1100:
		power = 0x2C;
		break;
	case -100:
		power = 0x03;
		break;
	case -800:
		power = 0x01;
		break;
	default:
		return -EINVAL;
	}

	return cc1120_write_register(priv, CC1120_TXPOWER, power);
#endif
	return 0;
}

#define CC1120_MAX_TX_POWERS 0x8
static const s32 cc1120_powers[CC1120_MAX_TX_POWERS + 1] = {
	500, 300, 200, 100, 0, -200, -400, -700, -1800,
};

#define CC1120_CC2591_MAX_TX_POWERS 0x5
static const s32 cc1120_cc2591_powers[CC1120_CC2591_MAX_TX_POWERS + 1] = {
	1700, 1600, 1400, 1100, -100, -800,
};

static int
cc1120_set_txpower(struct ieee802154_hw *hw, s32 mbm)
{
#if 0
	struct cc1120_private *priv = hw->priv;

	if (!priv->amplified)
		return cc1120_set_tx_power(priv, mbm);

	return cc1120_cc2591_set_tx_power(priv, mbm);
#endif 
	return 0;
}

static int
cc1120_set_promiscuous_mode(struct ieee802154_hw *hw, bool on)
{
#if 0
	struct cc1120_private *priv = hw->priv;
	u8 frmfilt0;

	dev_dbg(&priv->spi->dev, "%s : mode %d\n", __func__, on);

	priv->promiscuous = on;

	cc1120_read_register(priv, CC1120_FRMFILT0, &frmfilt0);

	if (on) {
		/* Disable automatic ACK, automatic CRC, and frame filtering. */
		cc1120_write_register(priv, CC1120_FRMCTRL0, 0);
		frmfilt0 &= ~FRMFILT0_FRAME_FILTER_EN;
	} else {
		cc1120_write_register(priv, CC1120_FRMCTRL0, FRMCTRL0_AUTOACK |
							     FRMCTRL0_AUTOCRC);
		frmfilt0 |= FRMFILT0_FRAME_FILTER_EN;
	}
	return cc1120_write_register(priv, CC1120_FRMFILT0, frmfilt0);
#endif 
	return 0;
}

static const struct ieee802154_ops cc1120_ops = {
	.owner = THIS_MODULE,
	.start = cc1120_start,
	.stop = cc1120_stop,
	.xmit_sync = cc1120_tx,
	.ed = cc1120_ed,
	.set_channel = cc1120_set_channel,
	.set_hw_addr_filt = cc1120_filter,
	.set_txpower = cc1120_set_txpower,
	.set_promiscuous_mode = cc1120_set_promiscuous_mode,
};

static int cc1120_register(struct cc1120_private *priv)
{
	int ret = -ENOMEM;

	priv->hw = ieee802154_alloc_hw(sizeof(*priv), &cc1120_ops);
	if (!priv->hw)
		goto err_ret;

	priv->hw->priv = priv;
	priv->hw->parent = &priv->spi->dev;
	priv->hw->extra_tx_headroom = 0;
	ieee802154_random_extended_addr(&priv->hw->phy->perm_extended_addr);

	/* We do support only 2.4 Ghz */
	priv->hw->phy->supported.channels[0] = 0x7FFF800;
	priv->hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT |
			  IEEE802154_HW_PROMISCUOUS;

	priv->hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	if (!priv->amplified) {
		priv->hw->phy->supported.tx_powers = cc1120_powers;
		priv->hw->phy->supported.tx_powers_size = ARRAY_SIZE(cc1120_powers);
		priv->hw->phy->transmit_power = priv->hw->phy->supported.tx_powers[4];
	} else {
		priv->hw->phy->supported.tx_powers = cc1120_cc2591_powers;
		priv->hw->phy->supported.tx_powers_size = ARRAY_SIZE(cc1120_cc2591_powers);
		priv->hw->phy->transmit_power = priv->hw->phy->supported.tx_powers[0];
	}

	priv->hw->phy->current_channel = 11;

	dev_vdbg(&priv->spi->dev, "registered cc1120\n");
	ret = ieee802154_register_hw(priv->hw);
	if (ret)
		goto err_free_device;

	return 0;

err_free_device:
	ieee802154_free_hw(priv->hw);
err_ret:
	return ret;
}

static void cc1120_fifop_irqwork(struct work_struct *work)
{
#if 0
	struct cc1120_private *priv
		= container_of(work, struct cc1120_private, fifop_irqwork);

	dev_dbg(&priv->spi->dev, "fifop interrupt received\n");

	if (gpio_get_value(priv->fifo_pin))
		cc1120_rx(priv);
	else
		dev_dbg(&priv->spi->dev, "rxfifo overflow\n");

	cc1120_cmd_strobe(priv, CC1120_CMD_SFLUSHRX);
	cc1120_cmd_strobe(priv, CC1120_CMD_SFLUSHRX);
#endif
}

static irqreturn_t cc1120_isr(int irq, void *data)
{
	struct cc1120_private *priv = data;

	schedule_work(&priv->fifop_irqwork);

	return IRQ_HANDLED;
}



static int cc1120_get_platform_data(struct spi_device *spi,
				    struct cc1120_platform_data *pdata)
{
	struct device_node *np = spi->dev.of_node;
	struct cc1120_private *priv = spi_get_drvdata(spi);

	if (!np) {
		struct cc1120_platform_data *spi_pdata = spi->dev.platform_data;

		if (!spi_pdata)
			return -ENOENT;
		*pdata = *spi_pdata;
		priv->fifo_pin = pdata->interrrupt;
		return 0;
	}

	return 0;
}

static int cc1120_hw_init(struct cc1120_private *priv)
{
	u8 status = 0, state = 0xff;
	int ret;
	int timeout = 100;
	struct cc1120_platform_data pdata;

	ret = cc1120_get_platform_data(priv->spi, &pdata);
	if (ret)
		goto err_ret;
	return 0;
err_ret:
	return ret;
}

static int cc1120_probe(struct spi_device *spi)
{
	struct cc1120_private *priv;
	struct cc1120_platform_data pdata;
	int ret, irq_type;

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
	priv->amplified = false;

	/* Request all the gpio's */

    pdata.interrrupt = devm_gpiod_get_index_optional(&spi->dev, "interrrupt", 0,
                                               GPIOD_OUT_LOW);
	if (IS_ERR(pdata.interrrupt)){
		dev_err(&spi->dev, "interrrupt gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata.reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 0,
                                               GPIOD_OUT_LOW);
	if (IS_ERR(pdata.reset)){
		dev_err(&spi->dev, "reset gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata.hgm = devm_gpiod_get_index_optional(&spi->dev, "hgm", 0,
                                               GPIOD_OUT_LOW);
	if (IS_ERR(pdata.hgm)){
		dev_err(&spi->dev, "hgm gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}
    pdata.tcxo = devm_gpiod_get_index_optional(&spi->dev, "tcxo", 0,
                                               GPIOD_OUT_LOW);
	if (IS_ERR(pdata.tcxo)){
		dev_err(&spi->dev, "tcxo gpio is not valid\n");
		ret = -EINVAL;
		goto err_hw_init;
	}

	/* request IRQF_TRIGGER_LOW as fallback default */
	irq_type = irq_get_trigger_type(spi->irq);
	if (!irq_type)
			irq_type = IRQF_TRIGGER_LOW;

	ret = devm_request_irq(&spi->dev, spi->irq, cc1120_isr,
							irq_type, dev_name(&spi->dev), priv);
	if (ret) {
			dev_err(&spi->dev, "Unable to get IRQ");
			goto err_hw_init;
	}


	ret = cc1120_register(priv);
	if (ret)
		goto err_hw_init;

	return 0;

err_hw_init:
	mutex_destroy(&priv->buffer_mutex);
	flush_work(&priv->fifop_irqwork);
	return ret;
}

static int cc1120_remove(struct spi_device *spi)
{
	struct cc1120_private *priv = spi_get_drvdata(spi);

	mutex_destroy(&priv->buffer_mutex);
	flush_work(&priv->fifop_irqwork);

	ieee802154_unregister_hw(priv->hw);
	ieee802154_free_hw(priv->hw);

	return 0;
}

static const struct spi_device_id cc1120_ids[] = {
	{"cc1120", },
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
