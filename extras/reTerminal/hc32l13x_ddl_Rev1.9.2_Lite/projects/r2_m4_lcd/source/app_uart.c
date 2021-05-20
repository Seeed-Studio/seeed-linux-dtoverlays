#include "device.h"



//串口引脚配置
static void App_UartInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    DDL_ZERO_STRUCT(stcGpioCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  ///<使能GPIO外设时钟门控开关

    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin2, GpioAf1);             ///<配置PA02 为UART1 TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortA, GpioPin3, GpioAf1);             ///<配置PA03 为UART1 RX
}


//串口模块配置
void App_UartCfg(void)
{
    stc_uart_cfg_t  stcCfg;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;

    App_UartInit();

    DDL_ZERO_STRUCT(stcCfg);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//UART0外设模块时钟使能
    
    stcCfg.enRunMode = UartMskMode3;     //模式3
    stcCfg.enStopBit = UartMsk1bit;      //1位停止位
    stcCfg.enMmdorCk = UartMskEven;      //偶校验
    stcCfg.stcBaud.u32Baud = 115200;       //波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;         //通道采样分频配置
    stcCfg.stcBaud.u32Pclk = Sysctrl_GetPClkFreq();    //获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART1, &stcCfg);       //串口初始化

    Uart_ClrStatus(M0P_UART1,UartRC);    //清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);    //清发送请求
//    Uart_EnableIrq(M0P_UART1,UartRxIrq); //使能串口接收中断
//    Uart_EnableIrq(M0P_UART1,UartTxIrq); //使能串口发送中断
//    EnableNvic(UART1_IRQn, IrqLevel3, TRUE);              ///<系统中断使能
}

void Uart1_SendByte(char ch)
{
    Uart_SendDataIt(M0P_UART1, ch);
	Uart_ClrStatus(M0P_UART1, UartTC);
	while (!Uart_GetStatus(M0P_UART1, UartTC)) {
        ;
    }
}

#if 0
//UART1中断
void Uart1_IRQHandler(void)
{
    if(Uart_GetStatus(M0P_UART1, UartRC))
    {
/*        Uart_ClrStatus(M0P_UART1, UartRC);              //清除中断状态位
        u8RxData[u8RxCnt]=Uart_ReceiveData(M0P_UART1);  //发送数据
        u8RxCnt++;
        if(u8RxCnt>1)                                   //如果已接收两个字节
        {
            Uart_DisableIrq(M0P_UART1,UartRxIrq);       //禁止接收中断
        }*/
    }

    if(Uart_GetStatus(M0P_UART1, UartTC))
    {
/*        Uart_ClrStatus(M0P_UART1, UartTC);              //清除中断状态位
        Uart_SendDataIt(M0P_UART1, u8TxData[u8TxCnt++]);//发送数据
        if(u8TxCnt>1)                                   //如果已发送两个字节
        {
            u8TxCnt = 0;
            u8RxCnt = 0;
            Uart_DisableIrq(M0P_UART1,UartTxIrq);       //禁止发送中断
            Uart_EnableIrq(M0P_UART1,UartRxIrq);        //使能接收中断
        }*/
    }
}
#endif

