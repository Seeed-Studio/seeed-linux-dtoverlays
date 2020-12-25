#include "device.h"


/**
 ******************************************************************************
 ** \brief  初始化外部GPIO引脚
 **
 ** \return 无
 ******************************************************************************/
static void App_GpioInit(void)
{
    stc_gpio_cfg_t    GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    //PA06
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(GpioPortA, GpioPin6, &GpioInitStruct);
    Gpio_ClrIO(GpioPortA, GpioPin6);
    Gpio_SetAfMode(GpioPortA, GpioPin6, GpioAf2);
}


/**
 ******************************************************************************
 ** \brief  配置PCA
 **
 ** \return 无
 ******************************************************************************/
void App_PcaInit(void)
{	
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);

    App_PcaCfg(0);
    Pca_StartPca(FALSE);

    App_GpioInit();
}


void App_PcaCfg(uint8_t pulse)
{
    stc_pcacfg_t  PcaInitStruct;

    DDL_ZERO_STRUCT(PcaInitStruct);
    PcaInitStruct.pca_clksrc = PcaPclkdiv32;
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomEnable;       //允许比较器功能
    PcaInitStruct.pca_capp   = PcaCappDisable;      //禁止上升沿捕获
    PcaInitStruct.pca_capn   = PcaCapnDisable;      //禁止下降沿捕获
    PcaInitStruct.pca_mat    = PcaMatDisable;       //禁止匹配功能
    PcaInitStruct.pca_tog    = PcaTogDisable;       //禁止翻转控制功能
    PcaInitStruct.pca_pwm    = PcaPwm8bitEnable;    //使能PWM控制输出
    PcaInitStruct.pca_epwm   = PcaEpwmDisable;      //禁止16bitPWM输出
    PcaInitStruct.pca_ccapl  = 255-pulse;
    PcaInitStruct.pca_ccaph  = 255-pulse;
    Pca_M0Init(&PcaInitStruct);
}
