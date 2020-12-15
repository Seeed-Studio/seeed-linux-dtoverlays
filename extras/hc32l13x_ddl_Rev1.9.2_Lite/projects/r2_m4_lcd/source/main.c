/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/


/******************************************************************************
 * Include files
 ******************************************************************************/
#include "device.h"

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void App_ClkCfg(void);

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{
	uint32_t count = 0;
	uint8_t pulse = 0;
	
    App_ClkCfg();
    App_UartCfg();

	DBG_FUNC("POUT");
	App_PoutCfg();
/*	APP_Pout(VCC_LCD2V8, FALSE);
	APP_Pout(VCC_LCD1V8, FALSE);
	APP_Pout(LCD_RST, FALSE);
	APP_Pout(TP_RST, FALSE);
*/
	DBG_FUNC("PWM");
	App_PcaInit();
//	App_PcaCfg(250);
//	Pca_StartPca(TRUE);

#ifdef I2C_MASTER
	DBG_FUNC("I2C Master");
	App_I2cMasterCfg();
	App_I2cMaster_Test();
#else
	DBG_FUNC("I2C Slave");
	App_I2cCfg();

	App_I2cTPCfg();
	App_I2cTPTest();
#endif

	DBG_FUNC("loop");
    while (1)
    {
		App_I2cSlaveHandle();
	}
}


///< 时钟配置，配置时钟24mHz
static void App_ClkCfg(void)
{    
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);       //配置为外部24MHz时钟
    Sysctrl_SysClkSwitch(SysctrlClkRCH);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL,FALSE);
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
