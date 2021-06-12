#include "device.h"


#if 0
static int cnt = 0;
#define MDBG		DBG_PRINT
#else
#define MDBG(...)	
#endif

typedef enum {
	REG_ID = 0x0/*0x80*/,
	REG_PORTA,  /* BIT(2) for horizontal flip, BIT(3) for vertical flip */
	REG_PORTB,  // --
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,// --
	REG_PWM,    // --
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,

	REG_LCD_RST,
	REG_TP_RST,
	REG_TP_STATUS,
	REG_TP_POINT,
	REG_TP_VERSION,
	REG_ADC1,
	REG_ADC2,

	REG_MAX
}REG_ADDR;

static int CurReg = 0;
static uint8_t Regs[REG_MAX] = { 0xc3, 0x00 };

static uint8_t RxCnt = 0;
static uint8_t RxBuf[2] = { 0 };
static uint8_t TxCnt = 0;
static uint8_t TxBuf[TP_BUF_SIZE] = { 0 };
static uint8_t TxTotal = 0;
static uint8_t HandleFlag = 0;
extern uint32_t i2c_int_cnt;
void I2C_Slave_Process(void)
{
	if (!HandleFlag) return;
	HandleFlag = 0;

	MDBG("RxCnt=%d\n", RxCnt);
	if (RxCnt == 1) { // set addr
		if (RxBuf[0] >= 0x80) {
			CurReg = RxBuf[0] - 0x80;
		}
		MDBG("CurReg=%d\n", CurReg);
		if (CurReg >= REG_MAX) {
			CurReg = 0;
		}
		
		// -----
		if (REG_TP_STATUS == CurReg) {
			TxCnt = 0;
			TxTotal = TP_BUF_SIZE;
			if (HAL_OK != goodix_i2c_read(GOODIX_READ_COOR_ADDR, TxBuf, TxTotal)) {
				memset(TxBuf, 0, TxTotal);
			}
			else {
				goodix_i2c_write_u8(GOODIX_READ_COOR_ADDR, 0);
			}
		}
		else if (REG_TP_POINT == CurReg) {
			TxCnt = GOODIX_STATUS_SIZE + GOODIX_CONTACT_SIZE;
			TxTotal = TP_BUF_SIZE;
		}
		else if (REG_TP_VERSION == CurReg) {
			TxCnt = 0;
			TxTotal = 6;
			TP_Ver(TxBuf);
		}
		else if (CurReg >= REG_ADC1) {
			int idx = CurReg-REG_ADC1;
			if (idx >= ADC_CH_MAX) {
				idx = 0;
			}
			
			TxCnt = 0;
			TxTotal = 2;
			TxBuf[0] = (uint8_t)((AdcVal[idx]>>8)&0xFF);
			TxBuf[1] = (uint8_t)(AdcVal[idx]&0xFF);
		}
		else {
			TxCnt = 0;
			TxTotal = 1;
			TxBuf[0] = Regs[CurReg];
		}
	}
	else if (RxCnt == 2) { // write reg
		if ((REG_ID == CurReg)) { // read-only
			return;
		}
		if (CurReg >= REG_MAX) {
			MDBG("Invalid reg!\n");
			return;
		}
		
		MDBG("Wr:[%d]0x%x\n", CurReg, RxBuf[1]);
		Regs[CurReg] = RxBuf[1];
		
		switch (CurReg) { // operations
		case REG_POWERON:
			LCD_VDD_out(Regs[CurReg]);
			Regs[REG_PORTB] = Regs[CurReg];
			MDBG("POR:0x%x\n", Regs[CurReg]);
		break;
		
		case REG_LCD_RST:
			LCD_RST_out(Regs[CurReg]);
			MDBG("RST:0x%x\n", Regs[CurReg]);
		break;

		case REG_PWM:
			PWM_config(Regs[CurReg]);
			MDBG("PWM:0x%x\n", Regs[CurReg]);
		break;
		}
	}
	else {
		MDBG("Error!\n");
	}
}

void I2C_Slave_Init(I2C_HandleTypeDef *hi2c)
{
	__HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI|I2C_IT_TCI|I2C_IT_STOPI\
		|I2C_IT_NACKI|I2C_IT_ADDRI|I2C_IT_RXI/*|I2C_IT_TXI*/);
}

void I2C_Slave_ISR(I2C_HandleTypeDef *hi2c)
{
	uint32_t isr = hi2c->Instance->ISR;

	i2c_int_cnt++;

	MDBG("[%d]0x%x\n", cnt++, isr);
	if (isr & I2C_FLAG_ALERT) {
		MDBG("ALERT\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ALERT);
	}
	if (isr & I2C_FLAG_TIMEOUT) {
		MDBG("TIMEOUT\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TIMEOUT);
	}
	if (isr & I2C_FLAG_PECERR) {
		MDBG("PECERR\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_PECERR);
	}
	if (isr & I2C_FLAG_OVR) {
		MDBG("OVR\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
	}
	if (isr & I2C_FLAG_BERR) {
		MDBG("BERR\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
	}

	// -----
	if (isr & I2C_FLAG_ADDR) {
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
		if (isr & I2C_FLAG_DIR) { // read
			__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
			__HAL_I2C_ENABLE_IT(hi2c, I2C_IT_TXI);
		}
		else { // write
			__HAL_I2C_DISABLE_IT(hi2c, I2C_IT_TXI);
		}
		MDBG("ADDR: %s\n", (isr&I2C_FLAG_DIR)?"R":"W");
	}
	if (isr & I2C_FLAG_RXNE) {
		if (RxCnt >= sizeof(RxBuf)) {
			RxCnt = 0;
		}
		RxBuf[RxCnt] = hi2c->Instance->RXDR;
		MDBG("RXNE:%d,0x%x\n", RxCnt+1, RxBuf[RxCnt]);
		RxCnt++;

		//I2C_Slave_Process();
		HandleFlag = 1;
	}

	if ((I2C_FLAG_DIR|I2C_FLAG_TXE) == (isr&(I2C_FLAG_DIR|I2C_FLAG_TXE))) {
		MDBG("TXE:%d/%d 0x%x", TxCnt, TxTotal, TxBuf[TxCnt]);
		hi2c->Instance->TXDR = TxBuf[TxCnt];
		if (++TxCnt >= TxTotal) {
			TxCnt = 0;
		}
		MDBG("\n");
	}

	if (isr & I2C_FLAG_AF) {
		MDBG("AF\n");
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
	}
	if (isr & I2C_FLAG_STOPF) {
		MDBG("STOPF\n\n");
		RxCnt = 0;
		__HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
		__HAL_I2C_DISABLE_IT(hi2c, I2C_IT_TXI);
	}
}

