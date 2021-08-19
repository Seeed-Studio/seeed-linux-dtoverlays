#ifndef __DEVICE_H__
#define __DEVICE_H__


#include <stdio.h>
#include <string.h>

#include "main.h"


#define _DEBUG
#ifdef _DEBUG
extern uint8_t TxCplt;
extern UART_HandleTypeDef huart2;
#define DBG_PRINT(_s...)	{ \
								int len = 0; char str[256]; \
								len = sprintf(str, _s); \
								HAL_UART_Transmit(&huart2, (uint8_t*)str, len, 1000); \
							}
#else
#define DBG_PRINT(_s...)
#endif


#if 0// TIMER
typedef uint32_t TIMER_HANDLE;
typedef enum {
	TIMER_IDLE = 0,
	TIMER_RESET,
	TIMER_START,
	TIMER_STOP,
}TIMER_STATE;
void Timer_Init(void);
void Timer_Tick(void);
TIMER_HANDLE Timer_Register(uint32_t timeout, void(*func)(void*), void *param);
uint8_t Timer_State(TIMER_HANDLE htimer, TIMER_STATE state);
void Timer_Process(void);
#endif


// PWM
void PWM_config(uint8_t duty);


// GPIO output
void LCD_VDD_out(uint8_t level);
void LCD_RST_out(uint8_t level);
void TP_INT_out(uint8_t level);
void TP_INT_cfg(void);
void TP_RST_out(uint8_t level);

// MCU auto reset when the kernel driver of TP is not working
void MCU_AUTO_reset(uint8_t enable);

// MCU firmware update
void UPDATE_firmware(uint8_t enable);

// Touchpanel
/* Register defines */
#define GOODIX_REG_COMMAND				0x8040
#define GOODIX_CMD_SCREEN_OFF			0x05

#define GOODIX_READ_COOR_ADDR		    0x814E
#define GOODIX_GT1X_REG_CONFIG_DATA		0x8050
#define GOODIX_GT9X_REG_CONFIG_DATA		0x8047
#define GOODIX_REG_ID					0x8140

#define GOODIX_BUFFER_STATUS_READY		(((uint32_t)0x01)<<7)//BIT(7)
#define GOODIX_HAVE_KEY					(((uint32_t)0x01)<<4)//BIT(4)

#define GOODIX_MAX_CONTACTS		5
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_STATUS_SIZE		2

#define TP_CONTACT_SIZE			(GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE)
#define TP_BUF_SIZE				(GOODIX_STATUS_SIZE+TP_CONTACT_SIZE)
HAL_StatusTypeDef goodix_i2c_read(uint16_t reg, uint8_t *buf, uint32_t len);
HAL_StatusTypeDef goodix_i2c_write_u8(uint16_t reg, uint8_t value);
void TP_Ver(uint8_t *data);
void TP_Init(I2C_HandleTypeDef *hi2c);


// I2C slave
void I2C_Slave_ISR(I2C_HandleTypeDef *hi2c);
void I2C_Slave_Init(I2C_HandleTypeDef *hi2c);
void I2C_Slave_Process(void);


#endif // __DEVICE_H__

