#include "device.h"


///< IO端口配置
void App_PoutCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);  ///< 开启GPIO时钟门控 

    stcGpioCfg.enDir = GpioDirOut;                ///< 端口方向配置->输出
    stcGpioCfg.enPu = GpioPuDisable;              ///< 端口上拉配置->使能
    stcGpioCfg.enPd = GpioPdDisable;              ///< 端口下拉配置->禁止

    Gpio_Init(GpioPortA, GpioPin0, &stcGpioCfg);
    Gpio_Init(GpioPortA, GpioPin1, &stcGpioCfg);
    Gpio_Init(GpioPortA, GpioPin5, &stcGpioCfg);
    Gpio_Init(GpioPortA, GpioPin7, &stcGpioCfg);

    APP_Pout(VCC_LCD2V8, TRUE);
    APP_Pout(VCC_LCD1V8, TRUE);
    APP_Pout(LCD_RST, TRUE);
//    APP_Pout(TP_RST, TRUE);
}

void APP_Pdir(en_gpio_pin_t pin, uint8_t dir)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);

    if (dir) { // output
        stcGpioCfg.enDir = GpioDirOut;                ///< 端口方向配置->输出
        stcGpioCfg.enPu = GpioPuDisable;              ///< 端口上拉配置->使能
        stcGpioCfg.enPd = GpioPdDisable;              ///< 端口下拉配置->禁止
    }
    else { // input
        stcGpioCfg.enDir = GpioDirIn;                ///< 端口方向配置->输出
        stcGpioCfg.enPu = GpioPuDisable;              ///< 端口上拉配置->使能
        stcGpioCfg.enPd = GpioPdDisable;              ///< 端口下拉配置->禁止
    }

    Gpio_Init(GpioPortA, pin, &stcGpioCfg);
}

void APP_Pout(en_gpio_pin_t pin, boolean_t enable)
{
    if (enable)
        Gpio_SetIO(GpioPortA, pin);
    else
        Gpio_ClrIO(GpioPortA, pin);
}
