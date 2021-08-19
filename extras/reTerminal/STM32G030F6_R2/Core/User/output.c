#include "device.h"


extern TIM_HandleTypeDef htim3;

void PWM_config(uint8_t duty)
{
	htim3.Instance->CCR2 = duty;
	if (duty) {
		 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	}
	else {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	}
}

void LCD_VDD_out(uint8_t level)
{
	HAL_GPIO_WritePin(LCD_VDD_GPIO_Port, LCD_VDD_Pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void LCD_RST_out(uint8_t level)
{
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void TP_INT_out(uint8_t level)
{
	HAL_GPIO_WritePin(TP_INT_GPIO_Port, TP_INT_Pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

#if 1//defined(TP_INT_ENABLE)
void TP_INT_cfg(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Configure GPIO pin : TP_INT_Pin */
	GPIO_InitStruct.Pin = TP_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TP_INT_GPIO_Port, &GPIO_InitStruct);

//	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
//	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
#endif

void TP_RST_out(uint8_t level)
{
	HAL_GPIO_WritePin(TP_RST_GPIO_Port, TP_RST_Pin, (level ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void MCU_AUTO_reset(uint8_t enable)
{
	if(enable)
		HAL_TIM_Base_Start_IT(&htim14);
	else
		HAL_TIM_Base_Stop_IT(&htim14);
}

void UPDATE_firmware(uint8_t enable)
{
	if(enable) {
		HAL_FLASH_Unlock();
		HAL_FLASH_OB_Unlock();

		CLEAR_BIT(FLASH->OPTR, FLASH_OPTR_nBOOT0);
		SET_BIT(FLASH->ACR, FLASH_ACR_PROGEMPTY);

		SET_BIT(FLASH->CR, FLASH_CR_STRT);
		FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
		CLEAR_BIT(FLASH->CR, FLASH_CR_OPTSTRT);

		HAL_FLASH_OB_Lock();
		HAL_FLASH_Lock();

		NVIC_SystemReset();//reboot
	}
}

