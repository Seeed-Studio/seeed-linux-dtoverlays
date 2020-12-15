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

	App_PcaCfg(255);
	Pca_StartPca(FALSE);

	App_GpioInit();
}


void App_PcaCfg(uint8_t pulse)
{
	stc_pcacfg_t  PcaInitStruct;

	DDL_ZERO_STRUCT(PcaInitStruct);
    PcaInitStruct.pca_clksrc = PcaPclkdiv32;
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomEnable;       //�����Ƚ�������
    PcaInitStruct.pca_capp   = PcaCappDisable;      //��ֹ�����ز���
    PcaInitStruct.pca_capn   = PcaCapnDisable;      //��ֹ�½��ز���
    PcaInitStruct.pca_mat    = PcaMatDisable;       //��ֹƥ�书��
    PcaInitStruct.pca_tog    = PcaTogDisable;       //��ֹ��ת���ƹ���
    PcaInitStruct.pca_pwm    = PcaPwm8bitEnable;    //ʹ��PWM�������
    PcaInitStruct.pca_epwm   = PcaEpwmDisable;      //��ֹ16bitPWM���
    PcaInitStruct.pca_ccapl  = pulse;
    PcaInitStruct.pca_ccaph  = pulse;
    Pca_M0Init(&PcaInitStruct);
}
