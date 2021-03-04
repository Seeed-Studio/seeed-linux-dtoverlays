/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/*
 * Definitions for LS98xx als/ps sensor chip.
 */
#ifndef __LS98xx_H__
#define __LS98xx_H__

#include <linux/ioctl.h>


#define LS98XX_DBG
#ifdef LS98XX_DBG
#define DBG_FUNC(format, x...)		printk(KERN_INFO "[LS98xx]%s:" format"\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[LS98xx]" format"\n", ##x)
#else
#define DBG_FUNC(format, x...)
#define DBG_PRINT(format, x...)
#endif


/*ALSPS REGS*/
/*********************************************
* Register definitions *
*********************************************/
#define LS98xx_OPSEL_REG                0x00
#define LS98xx_STAT_REG                 0x01 
#define LS98xx_IFLAG_REG                0x02 
#define LS98xx_IENL_REG                 0x03
#define LS98xx_IENP_REG                 0x04
#define LS98xx_ICONFL_REG               0x05
#define LS98xx_ICONFP_REG               0x06

#define LS98xx_ILTL_L_REG               0x07
#define LS98xx_ILTH_L_REG               0x08
#define LS98xx_IHTL_L_REG               0x09
#define LS98xx_IHTH_L_REG               0x0A

#define LS98xx_ILTL_P_REG               0x0B
#define LS98xx_ILTH_P_REG               0x0C
#define LS98xx_IHTL_P_REG               0x0D
#define LS98xx_IHTH_P_REG               0x0E

#define LS98xx_LCONF1_REG               0x0F
#define LS98xx_LCONF2_REG               0x10 

#define	LS98xx_LDATA1L_REG              0x11
#define LS98xx_LDATA1H_REG              0x12
#define LS98xx_LDATA2L_REG              0x13
#define LS98xx_LDATA2H_REG              0x14
#define LS98xx_LDATA3L_REG              0x15
#define LS98xx_LDATA3H_REG              0x16

#define LS98xx_PCONF1_REG               0x17
#define LS98xx_PCONF2_REG               0x18
#define LS98xx_PCONF3_REG               0x19
#define LS98xx_PCONF4_REG               0x1A
#define LS98xx_PCONF5_REG               0x1B
#define LS98xx_PCONF6_REG               0x1C

#define LS98xx_PDATAL_REG               0x1E
#define LS98xx_PDATAH_REG               0x1F
#define LS98xx_PDATAAR_REG              0x1D

#define	LS98xx_ID_REG                   0x20


/*********************************************
* Register control                           *
*********************************************/
/* Define state reg */
//LS98xx_OPSEL_REG @0x00
#define	LS98xx_ALS_EN                   (0x01 << 0)
#define	LS98xx_PS_EN                    (0x01 << 1)
#define	LS98xx_SYNC                     (0x01 << 6)
#define	LS98xx_RESET                    (0x01 << 7)

//LS98xx_STAT_REG @0x01
#define	LS98xx_LDRY                     (0x01 << 0)
#define	LS98xx_PDRY                     (0x01 << 1)

/* Define interrupt reg */
//LS9800_IENL_REG @0x03
#define LS98xx_IENLL		             0x01
#define LS98xx_IENLH                    (0x01 << 1)
#define LS98xx_IENLCH		            (0x01 << 2)

//LS9800_IENP_REG @0x04
#define LS98xx_IENPL	                 0x01
#define LS98xx_IENPH                    (0x01 << 1)
#define LS98xx_IPERSP                    0x01

/* Define als ctrl reg */
//LS9800_GAIN_REG @0x0F
#define LS98xx_LAGAIN_1x                 0X00
#define LS98xx_LAGAIN_4x                 0X01
#define LS98xx_LAGAIN_16x                0X02
#define LS98xx_LAGAIN_128x               0X03
#define LS98xx_LAGAIN_256x               0X04

#define LS98xx_LDGAIN_1x                (0x00 << 3)
#define LS98xx_LDGAIN_2x                (0x01 << 3)
#define LS98xx_LDGAIN_4x                (0x02 << 3)
#define LS98xx_LDGAIN_8x                (0x03 << 3)
#define LS98xx_LDGAIN_16x               (0x04 << 3)

//LS9800_W&I_REG @0x10
#define LS98xx_LWAIT_50ms               (0X07 << 4)
#define LS98xx_LIT_100ms                 0X09

/* Define ps ctrl reg */
//LS9800_PD_REG @0x17
#define LS98xx_PPDSEL                   (0x03 << 3)

//LS9800_PGAIN_Ctrl @0x18
#define LS98xx_PAGAIN_1x                 0x00
#define LS98xx_PAGAIN_2x                 0x10
#define LS98xx_PAGAIN_4x                 0x18
#define LS98xx_PAGAIN_8x                 0x1C
#define LS98xx_PAGAIN_16x                0x1E
#define LS98xx_PAGAIN_32x                0x1F

//LS9800_POS_Ctrl @0x19
#define LS98xx_POS_0                     0x00
#define LS98xx_POS_1                     0x01 
#define LS98xx_POS_2                     0x02 
#define LS98xx_POS_3                     0x03

//LS9800_PS_W&Itime_Ctrl @0x1A
#define LS98xx_PWAIT_50ms               (0X03 << 4)
#define LS98xx_LIT_2ms                   0X06

//LS9800_LED_PULSE_Ctrl @0x1B
#define LS98xx_PPULSE_16                 0X0F
#define LS98xx_PPULSE_8                  0X07
#define LS98xx_PPULSE_4                  0X03
//LS9800_LED_Current_Ctrl @0x1C
#define LS98xx_PBLANK_16                (0X03 << 4)
#define LS98xx_DRV_150mA                 0X0F
#define LS98xx_DRV_125mA                 0X0D
#define LS98xx_DRV_100mA                 0X0B
#define LS98xx_DRV_50mA                  0X07
#define LS98xx_DRV_25mA                  0X05
/* Define Reset_Ctrl*/
#define LS98xx_RESET_REG_DEF             0X80

/* Define flag*/
#define LS98xx_FLAG_LINTL_MASK           (0X01 << 0)
#define LS98xx_FLAG_LINTH_MASK           (0X01 << 1)
#define LS98xx_FLAG_PINTL_MASK           (0X01 << 6)
#define LS98xx_FLAG_PINTH_MASK           (0X01 << 7)

#define LS98xx_FLAG_LDRY_MASK            (0X01 << 0)
#define LS98xx_FLAG_PDRY_MASK            (0X01 << 1)
#define LS98xx_FLAG_PEER0_MASK           (0X01 << 2)
#define LS98xx_FLAG_PEER1_MASK           (0X01 << 3)
#define LS98xx_FLAG_PEER2_MASK           (0X03 << 2)

#endif
