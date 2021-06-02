#include "device.h"


#define GOODIX_I2C_ADDR					(0x5d<<1)
#define GOOGIX_I2C_TIMEOUT				10

static I2C_HandleTypeDef *hTP_I2C = NULL;

/*static */HAL_StatusTypeDef goodix_i2c_read(uint16_t reg, uint8_t *buf, uint32_t len)
{
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t addr[2] = { 0 };

    addr[0] = reg >> 8;
    addr[1] = reg & 0xFF;

    ret = HAL_I2C_Master_Transmit(hTP_I2C, GOODIX_I2C_ADDR, addr, 2, GOOGIX_I2C_TIMEOUT);
    if (ret != HAL_OK) {
        return ret;
    }
    ret = HAL_I2C_Master_Receive(hTP_I2C, GOODIX_I2C_ADDR, buf, len, GOOGIX_I2C_TIMEOUT);
	if (ret != HAL_OK) {
        return ret;
    }

    return ret;
}

/*static */HAL_StatusTypeDef goodix_i2c_write_u8(uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = { 0 };

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = value;

    return HAL_I2C_Master_Transmit(hTP_I2C, GOODIX_I2C_ADDR, buf, sizeof(buf), GOOGIX_I2C_TIMEOUT);
}


static uint8_t Ver[8] = {M_VER, S_VER};
void TP_Ver(uint8_t *data)
{
	memcpy(data, Ver, sizeof(Ver));
}

void TP_Init(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef ret = HAL_OK;

	// -----
	hTP_I2C = hi2c;

	// Reset
	TP_INT_out(0);
	TP_RST_out(0);
	HAL_Delay(20);
	TP_RST_out(1);
	HAL_Delay(100);

	TP_INT_cfg();

	// Check
	ret = goodix_i2c_read(GOODIX_REG_ID, &Ver[2], 6);
	DBG_PRINT("Version:%d.%d\n", Ver[0], Ver[1]);
	if (HAL_OK == ret) {
		DBG_PRINT("TP detected:%02x,%02x,%02x,%02x,%02x,%02x\n", 
			Ver[2], Ver[3], Ver[4], Ver[5], Ver[6], Ver[7]);
	}
	else {
		DBG_PRINT("No TP detected:%d!\n", ret);
	}
}

