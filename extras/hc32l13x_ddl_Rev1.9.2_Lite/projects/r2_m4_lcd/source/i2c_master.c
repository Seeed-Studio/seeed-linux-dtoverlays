#include "device.h"


#define I2C_DEVADDR           0x45


///< IO端口配置
static void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);   //开启GPIO时钟门控 
    
    stcGpioCfg.enDir = GpioDirOut;                           ///< 端口方向配置->输出    
    stcGpioCfg.enOD = GpioOdEnable;                          ///< 开漏输出
    stcGpioCfg.enPu = GpioPuEnable;                          ///< 端口上拉配置->使能
    stcGpioCfg.enPd = GpioPdDisable;                         ///< 端口下拉配置->禁止
    
    Gpio_Init(GpioPortB,GpioPin8,&stcGpioCfg);               ///< 端口初始化
    Gpio_Init(GpioPortB,GpioPin9,&stcGpioCfg);
    
    Gpio_SetAfMode(GpioPortB,GpioPin8,GpioAf1);              ///< 配置PB08为SCL
    Gpio_SetAfMode(GpioPortB,GpioPin9,GpioAf1);              ///< 配置PB09为SDA
}

///< I2C 模块配置
void App_I2cMasterCfg(void)
{
    stc_i2c_cfg_t stcI2cCfg;
    
    App_PortCfg();

    DDL_ZERO_STRUCT(stcI2cCfg);                            ///< 初始化结构体变量的值为0
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c0,TRUE); ///< 开启I2C0时钟门控
    
    stcI2cCfg.u32Pclk = Sysctrl_GetPClkFreq();             ///< 获取PCLK时钟
    stcI2cCfg.u32Baud = 1000000;                           ///< 1MHz
    stcI2cCfg.enMode = I2cMasterMode;                      ///< 主机模式
    stcI2cCfg.u8SlaveAddr = 0x55;                          ///< 从地址，主模式无效
    stcI2cCfg.bGc = FALSE;                                 ///< 广播地址应答使能关闭
    I2C_Init(M0P_I2C0,&stcI2cCfg);                         ///< 模块初始化
}

/**
 ******************************************************************************
 ** \brief  主机接收函数
 **
 ** \param u8Addr从机内存地址，pu8Data读数据存放缓存，u32Len读数据长度
 **
 ** \retval 读数据是否成功
 **
 ******************************************************************************/
en_result_t I2C_MasterReadData(M0P_I2C_TypeDef* I2CX, uint8_t slave_addr, uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    uint32_t timeout = 1000;
    
    I2C_SetFunc(I2CX,I2cStart_En);
    
    while(1) {
        while(0 == I2C_GetIrq(I2CX)) {
            timeout--;
            if (timeout == 0) {
                I2C_SetFunc(I2CX,I2cStop_En);
                return ErrorTimeout;
            }
            delay100us(1);
        }

        u8State = I2C_GetState(I2CX);
        switch(u8State)
        {
            case 0x08:                                    //已发送起始条件，将发送SLA+R
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,(slave_addr<<1)|0x01);//发送SLA+W
                break;
            case 0x18:                                    //已发送SLA+W,并接收到ACK
                I2C_WriteByte(I2CX,0);                    //发送内存地址
                break;
            case 0x28:                                    //已发送数据，接收到ACK
                I2C_SetFunc(I2CX,I2cStart_En);
                break;
            case 0x10:                                    //已发送重复起始条件
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,(slave_addr<<1)|0x01);//读命令发送
                break;
            case 0x40:                                    //已发送SLA+R，并接收到ACK
                if(u32Len>1)
                {
                    I2C_SetFunc(I2CX,I2cAck_En);
                }
                break;
            case 0x50:                                    //已接收数据字节，并已返回ACK信号
                pu8Data[u8i++] = I2C_ReadByte(I2CX);
                if(u8i==u32Len-1)
                {
                    I2C_ClearFunc(I2CX,I2cAck_En);        //读数据时，倒数第二个字节ACK关闭
                }
                break;
            case 0x58:                                    //已接收到最后一个数据，NACK已返回
                pu8Data[u8i++] = I2C_ReadByte(I2CX);
                I2C_SetFunc(I2CX,I2cStop_En);             //发送停止条件
                break;    
            case 0x38:                                    //在发送地址或数据时，仲裁丢失
                I2C_SetFunc(I2CX,I2cStart_En);            //当总线空闲时发起起始条件
                break;
            case 0x48:                                    //发送SLA+R后，收到一个NACK
                I2C_SetFunc(I2CX,I2cStop_En);
                I2C_SetFunc(I2CX,I2cStart_En);
                break;
            default:                                      //其他错误状态，重新发送起始条件
                I2C_SetFunc(I2CX,I2cStart_En);            //其他错误状态，重新发送起始条件
                break;
        }
        I2C_ClearIrq(I2CX);                               //清除中断状态标志位
        if(u8i==u32Len)                                   //数据全部读取完成，跳出while循环
        {
            break;
        }
    }
    enRet = Ok;
    return enRet;
}
/**
 ******************************************************************************
 ** \brief  主机发送函数
 **
 ** \param u8Addr从机内存地址，pu8Data写数据，u32Len写数据长度
 **
 ** \retval 写数据是否成功
 **
 ******************************************************************************/
en_result_t I2C_MasterWriteData(M0P_I2C_TypeDef* I2CX, uint8_t slave_addr, uint8_t *pu8Data,uint32_t u32Len)
{
    en_result_t enRet = Error;
    uint8_t u8i=0,u8State;
    uint32_t timeout = 1000;

    I2C_SetFunc(I2CX,I2cStart_En);
    while(1)
    {
        while(0 == I2C_GetIrq(I2CX))
        {
            timeout--;
            if (timeout == 0) {
                I2C_SetFunc(I2CX,I2cStop_En);
                return ErrorTimeout;
            }
            delay100us(1);
        }

        u8State = I2C_GetState(I2CX);
        switch(u8State)
        {
            case 0x08:                                 ///已发送起始条件
                I2C_ClearFunc(I2CX,I2cStart_En);
                I2C_WriteByte(I2CX,(slave_addr<<1));  ///从设备地址发送
                break;
            case 0x18:                                 ///已发送SLA+W，并接收到ACK
            case 0x28:                                 ///上一次发送数据后接收到ACK
                I2C_WriteByte(I2CX,pu8Data[u8i++]);
                break;
            case 0x20:                                 ///上一次发送SLA+W后，收到NACK
            case 0x38:                                 ///上一次在SLA+读或写时丢失仲裁
                I2C_SetFunc(I2CX,I2cStart_En);         ///当I2C总线空闲时发送起始条件
                break;
            case 0x30:                                 ///已发送I2Cx_DATA中的数据，收到NACK，将传输一个STOP条件
                I2C_SetFunc(I2CX,I2cStop_En);          ///发送停止条件
                break;
            default:
                break;
        }
        if(u8i>u32Len)
        {
            I2C_SetFunc(I2CX,I2cStop_En);              ///此顺序不能调换，出停止条件
            I2C_ClearIrq(I2CX);
            break;
        }
        I2C_ClearIrq(I2CX);                            ///清除中断状态标志位
    }
    enRet = Ok;
    return enRet;
}


#ifdef I2C_MASTER
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
static uint8_t u8Senddata[10] = {0x12,0x34,0x77,0x66,0x55,0x44,0x33,0x22,0x11,0x99};
static uint8_t u8Recdata[10]={0x00};

void Reg_Read(uint8_t reg)
{
    u8Senddata[0] = reg;
    I2C_MasterWriteData(M0P_I2C0, I2C_DEVADDR, u8Senddata, 1);
    I2C_MasterReadData(M0P_I2C0, I2C_DEVADDR, u8Recdata, 1);
    DBG_FUNC("R[0x%x]0x%x", u8Senddata[0], u8Recdata[0]);
}

void Reg_WriteRead(uint8_t reg, uint8_t val)
{
    u8Senddata[0] = reg;
    u8Senddata[1] = val;
    I2C_MasterWriteData(M0P_I2C0, I2C_DEVADDR, u8Senddata,2);
    delay1ms(100);

    u8Senddata[0] = reg;
    I2C_MasterWriteData(M0P_I2C0, I2C_DEVADDR, u8Senddata, 1);
    I2C_MasterReadData(M0P_I2C0, I2C_DEVADDR, u8Recdata, 1);
    DBG_FUNC("R[0x%x]0x%x", u8Senddata[0], u8Recdata[0]);
}

void App_I2cMaster_Test(void)
{
    Reg_Read(0x80);
    delay1ms(100);

    Reg_Read(0x82);
    delay1ms(100);

    Reg_WriteRead(0x85, 1);
    delay1ms(100);

    Reg_Read(0x82);
    delay1ms(100);

    Reg_WriteRead(0x86, 250);
    delay1ms(100);

    Reg_WriteRead(0x93, 1);
    delay1ms(100);

    Reg_WriteRead(0x94, 1);
    delay1ms(100);

    Reg_WriteRead(0x85, 0);
    delay1ms(100);

    Reg_Read(0x82);
    delay1ms(100);

    Reg_WriteRead(0x86, 0);
    delay1ms(100);

    Reg_WriteRead(0x93, 0);
    delay1ms(100);

    Reg_WriteRead(0x94, 0);
    delay1ms(100);
}
#endif

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
