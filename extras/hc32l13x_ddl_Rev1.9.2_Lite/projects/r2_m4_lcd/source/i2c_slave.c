#include "device.h"
#include "goodix.h"


#define I2C_SLVADDR      0x45


enum REG_ADDR {
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

    REG_MAX
};
static uint8_t CurReg = REG_MAX;
static uint8_t Regs[REG_MAX] = { 0xc3, 0 };

static u32_t RxCnt = 0;
static u8_t RxBuf[2] = { 0 };
static u32_t TxCnt = 0;
static u8_t TxBuf[42] = { 0 };
static u8_t HandleFlag = 0;

///< IO端口配置
static void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///< 开启GPIO时钟门控 

    stcGpioCfg.enDir = GpioDirOut;                ///< 端口方向配置->输出
    stcGpioCfg.enOD = GpioOdEnable;               ///< 端口开漏输出配置->开漏输出使能
    stcGpioCfg.enPu = GpioPuEnable;               ///< 端口上拉配置->使能
    stcGpioCfg.enPd = GpioPdDisable;              ///< 端口下拉配置->禁止
        
    Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);   ///< 端口初始化
    Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg);
    
    Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf4);  ///< 配置PA09为SCL
    Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf4);  ///< 配置PA10为SDA
}


///< I2C 模块配置
void App_I2cSlaveCfg(void)
{
    stc_i2c_cfg_t stcI2cCfg;
    
    App_PortCfg();

    DDL_ZERO_STRUCT(stcI2cCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c0,TRUE);  ///< 开启I2C0时钟门控
    
    EnableNvic(I2C0_IRQn,IrqLevel3,TRUE);       ///< 使能NVIC对应中断位
        
    stcI2cCfg.enMode  = I2cSlaveMode;           ///< I2C从机模式
    stcI2cCfg.u8SlaveAddr = I2C_SLVADDR;        ///< 从地址，主模式无效
    stcI2cCfg.bGc = FALSE;                      ///< 广播地址应答使能关闭，主模式无效
    I2C_Init(M0P_I2C0, &stcI2cCfg);             ///< 模块初始化
    I2C_SetFunc(M0P_I2C0,I2cHlm_En);            ///< 使能I2C0高速通讯功能（高于100KHz必须使能）
}


void App_I2cSlaveHandle(void)
{
    if (!HandleFlag) return;
    HandleFlag = 0;

    if (RxCnt == 1) { // set addr
        if (RxBuf[0] >= 0x80) {
            CurReg = RxBuf[0] - 0x80;
        }

        if (CurReg >= REG_MAX)
            return;

        TxCnt = 0;
        if (REG_TP_STATUS == CurReg) {
            if (App_GetTpState()) {
                goodix_i2c_read(GOODIX_READ_COOR_ADDR, TxBuf, 2);
                //printf("S:0x%x,0x%x\r\n", TxBuf[0], TxBuf[1]);
            }
        }
        else if (REG_TP_POINT == CurReg) {
            if (App_GetTpState()) {
                goodix_i2c_read(GOODIX_READ_COOR_ADDR+2, TxBuf, 40);
                goodix_i2c_write_u8(GOODIX_READ_COOR_ADDR, 0);
                //printf("P:0x%x,0x%x,0x%x,0x%x\r\n", TxBuf[0], TxBuf[1], TxBuf[2], TxBuf[3]);
            }
        }
        else {
            TxBuf[0] = Regs[CurReg];
        }
    }
    else if (RxCnt == 2) { // write reg
        if ((REG_ID == CurReg) || (REG_PORTB == CurReg) 
            || (REG_TP_STATUS == CurReg) || (REG_TP_POINT == CurReg)) { // read-only
            return;
        }

        printf("R:0x%x,0x%x\n", RxBuf[0], RxBuf[1]);

        Regs[CurReg] = RxBuf[1];
        TxCnt = 0;
        TxBuf[0] = RxBuf[1];
        switch (CurReg) { // operations
        case REG_POWERON:
            APP_Pout(VCC_LCD2V8, Regs[CurReg]);
            APP_Pout(VCC_LCD1V8, Regs[CurReg]);
            Regs[REG_PORTB] = Regs[CurReg];
            printf("A:0x%x\n", Regs[CurReg]);
        break;
        
        case REG_PWM:
            App_PcaCfg(Regs[CurReg]);
	        Pca_StartPca(Regs[CurReg]?TRUE:FALSE);
            printf("B:0x%x\n", Regs[CurReg]);
        break;
        
        case REG_LCD_RST:
            APP_Pout(LCD_RST, Regs[CurReg]);
            printf("C:0x%x\n", Regs[CurReg]);
        break;
        
        case REG_TP_RST:
            APP_Pout(TP_RST, Regs[CurReg]);
            printf("D:0x%x\n", Regs[CurReg]);
        break;
        }
    }
}


///< I2c0中断函数
void I2c0_IRQHandler(void)
{
    uint8_t u8Value = 0;
    uint8_t u8State = I2C_GetState(M0P_I2C0);
    
    switch(u8State)
    {
        case 0x60: //已接收到(与自身匹配的)SLA+W；已接收ACK
        case 0x70: //已接收通用调用地址（0x00）；已接收ACK
        case 0xa0: //接收到停止条件或重复起始条件
            RxCnt = 0;//u8RxLen = 0;
            I2C_SetFunc(M0P_I2C0,I2cAck_En);
            break;

        case 0x68: //主控时在SLA+读写丢失仲裁；已接收自身的SLA+W；已返回ACK
        case 0x78: //主控时在SLA+读写中丢失仲裁；已接收通用调用地址；已返回ACK
        case 0x88: //前一次寻址使用自身从地址；已接收数据字节；已返回非ACK
            I2C_SetFunc(M0P_I2C0,I2cAck_En);
            break;
            
        case 0x80: //前一次寻址使用自身从地址；已接收数据字节；已返回ACK
        case 0x98: //前一次寻址使用通用调用地址；已接收数据；已返回非ACK
        case 0x90: //前一次寻址使用通用调用地址；已接收数据；已返回ACK
            I2C_SetFunc(M0P_I2C0,I2cAck_En);
            if (RxCnt >= sizeof(RxBuf)) {
                RxCnt = 0;
            }
            RxBuf[RxCnt++] = I2C_ReadByte(M0P_I2C0);//接收数据
            HandleFlag = 1;
            break;

        case 0xa8: //已接收自身的SLA+R；已返回ACK,将要发出数据并将收到ACK位
        case 0xb0: //当主控时在SLA+读写中丢失仲裁；已接收自身SLA+R；已返回ACK
            if (TxCnt >= sizeof(TxBuf)) {
                TxCnt = 0;
            }
            I2C_WriteByte(M0P_I2C0, TxBuf[TxCnt++]);
            break;

        case 0xc8: //装入的数据字节已被发送；已接收ACK
        case 0xb8: //已发送数据；已接收ACK
            if (TxCnt >= sizeof(TxBuf)) {
                TxCnt = 0;
            }
            I2C_WriteByte(M0P_I2C0, TxBuf[TxCnt++]);
            break;

        case 0xc0: //发送数据，接收非ACK
            break;
    }
    
    I2C_ClearIrq(M0P_I2C0);

    App_I2cSlaveHandle();
}
