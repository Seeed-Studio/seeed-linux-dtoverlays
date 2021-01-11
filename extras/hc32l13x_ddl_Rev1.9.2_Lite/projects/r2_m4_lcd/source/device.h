#ifndef __DEVICE_H__
#define __DEVICE_H__


/******************************************************************************
 * Include files
 ******************************************************************************/
#include <stdio.h>

#include "i2c.h"
#include "gpio.h"
#include "uart.h"
#include "pca.h"


typedef char s8_t, s8;
typedef unsigned char u8_t, u8;
typedef short s16_t, s16;
typedef unsigned short u16_t, u16;
typedef int s32_t, s32;
typedef unsigned int u32_t, u32;


#ifdef __DEBUG
#define DBG_FUNC(format, x...)		printf("%s:" format"\r\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printf(format"\r\n", ##x)
#else
#define DBG_FUNC(format, x...)
#define DBG_PRINT(format, x...)
#endif


#if 0
#define VCC_LCD2V8		// PA00
#define VCC_LCD1V8		// PA01
#define LCD_RST			// PA05
#define LCD_PWM			// PA06
#define LCD_SCL0		// PA09
#define LCD_SDA0		// PA10

#define TP_INT			// PA04
#define TP_RST			// PA07
#define TP_SCL1			// PA11
#define TP_SDA1			// PA12
#endif

#define VCC_LCD2V8		GpioPin0
#define VCC_LCD1V8		GpioPin1
#define LCD_RST			GpioPin5
#define TP_INT			GpioPin4
#define TP_RST			GpioPin7

void APP_Pout(en_gpio_pin_t pin, boolean_t enable);


/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*****************************************************************************
 * Function implementation - global ('extern')
 ******************************************************************************/
/* i2c */
en_result_t I2C_MasterReadData(M0P_I2C_TypeDef* I2CX, uint8_t slave_addr, uint8_t *pu8Data,uint32_t u32Len);
en_result_t I2C_MasterWriteData(M0P_I2C_TypeDef* I2CX, uint8_t slave_addr, uint8_t *pu8Data,uint32_t u32Len);
#ifdef I2C_MASTER
void App_I2cMasterCfg(void);
void App_I2cMaster_Test(void);
#else
void App_I2cSlaveCfg(void);

en_result_t goodix_i2c_read(u16_t reg, u8_t *buf, s32_t len);
en_result_t goodix_i2c_write_u8(u16_t reg, u8_t value);
uint8_t App_GetTpState(void);
void App_I2cTPCfg(void);
#endif
void App_I2cSlaveHandle(void);

/* pwm */
void App_PcaInit(void);
void App_PcaCfg(uint8_t pulse);

/* pout */
void App_PoutCfg(void);
void APP_Pdir(en_gpio_pin_t pin, uint8_t dir);
void APP_Pout(en_gpio_pin_t pin, boolean_t enable);

/* uart */
void App_UartCfg(void);


#endif // __DEVICE_H__
