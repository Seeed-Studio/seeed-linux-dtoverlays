#include "device.h"


uint8_t TxCplt = 1;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	TxCplt = 1;
}

