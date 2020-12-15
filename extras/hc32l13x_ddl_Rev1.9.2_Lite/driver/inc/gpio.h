/*******************************************************************************
* Copyright (C) 2018, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file gpio.h
 **
 ** GPIO driver
 ** @link GPIO Group Some description @endlink
 **
 **   - 2018-04-18
 **
 ******************************************************************************/

#ifndef __GPIO_H__
#define __GPIO_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"


/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 *******************************************************************************
 ** \defgroup GpioGroup General Purpose I/O (GPIO)
 **
 **
 ******************************************************************************/
//@{
#define GPIO_GPSZ       (0x40u)

/*******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief GPIO PORT类型定义
 ******************************************************************************/
typedef enum en_gpio_port
{
    GpioPortA = 0x00u,                 ///< GPIO PORT A
    GpioPortB = 0x40u,                 ///< GPIO PORT B
    GpioPortC = 0x80u,                 ///< GPIO PORT C
    GpioPortD = 0xc0u,                 ///< GPIO PORT D
}en_gpio_port_t;
 
/**
 *******************************************************************************
 ** \brief GPIO PIN类型定义
 ******************************************************************************/
typedef enum en_gpio_pin
{
    GpioPin0  = 0u,                 ///< GPIO PIN0 
    GpioPin1  = 1u,                 ///< GPIO PIN1 
    GpioPin2  = 2u,                 ///< GPIO PIN2 
    GpioPin3  = 3u,                 ///< GPIO PIN3 
    GpioPin4  = 4u,                 ///< GPIO PIN4 
    GpioPin5  = 5u,                 ///< GPIO PIN5 
    GpioPin6  = 6u,                 ///< GPIO PIN6 
    GpioPin7  = 7u,                 ///< GPIO PIN7 
    GpioPin8  = 8u,                 ///< GPIO PIN8 
    GpioPin9  = 9u,                 ///< GPIO PIN9 
    GpioPin10 = 10u,                ///< GPIO PIN10
    GpioPin11 = 11u,                ///< GPIO PIN11
    GpioPin12 = 12u,                ///< GPIO PIN12
    GpioPin13 = 13u,                ///< GPIO PIN13
    GpioPin14 = 14u,                ///< GPIO PIN14
    GpioPin15 = 15u,                ///< GPIO PIN15
}en_gpio_pin_t; 

/**
 *******************************************************************************
 ** \brief GPIO 端口复用功能(AF-Alternate function)类型定义
 ** \note  具体功能及含义请参考用户手册GPIO复用表
 ******************************************************************************/
typedef enum en_gpio_af
{
    GpioAf0  = 0u,      ///< GPIO功能
    GpioAf1  = 1u,      ///< GPIO AF1:复用功能1
    GpioAf2  = 2u,      ///< GPIO AF2:复用功能2
    GpioAf3  = 3u,      ///< GPIO AF3:复用功能3
    GpioAf4  = 4u,      ///< GPIO AF4:复用功能4
    GpioAf5  = 5u,      ///< GPIO AF5:复用功能5 
    GpioAf6  = 6u,      ///< GPIO AF6:复用功能6 
    GpioAf7  = 7u,      ///< GPIO AF7:复用功能7
}en_gpio_af_t;  
 
/**
 *******************************************************************************
 ** \brief GPIO输入输出配置数据类型定义
 ******************************************************************************/
typedef enum en_gpio_dir
{
    GpioDirOut = 0u,                ///< GPIO 输出
    GpioDirIn  = 1u,                ///< GPIO 输入            
}en_gpio_dir_t;

/**
 *******************************************************************************
 ** \brief GPIO端口上拉配置数据类型定义
 ******************************************************************************/
typedef enum en_gpio_pu
{
    GpioPuDisable = 0u,                ///< GPIO无上拉 
    GpioPuEnable  = 1u,                ///< GPIO上拉        
}en_gpio_pu_t;

/**
 *******************************************************************************
 ** \brief GPIO端口下拉配置数据类型定义
 ******************************************************************************/
typedef enum en_gpio_pd
{
    GpioPdDisable = 0u,                ///< GPIO无下拉
    GpioPdEnable  = 1u,                ///< GPIO下拉            
}en_gpio_pd_t;
/**
 *******************************************************************************
 ** \brief GPIO端口输出驱动能力配置数据类型定义
 ******************************************************************************/
typedef enum en_gpio_drv
{
    GpioDrvH = 0u,                  ///< GPIO高驱动能力
    GpioDrvL = 1u,                  ///< GPIO低驱动能力          
}en_gpio_drv_t;

/**
 *******************************************************************************
 ** \brief GPIO端口开漏输出控制数据类型定义
 ******************************************************************************/
typedef enum en_gpio_od
{
    GpioOdDisable = 0u,             ///< GPIO开漏输出关闭          
    GpioOdEnable  = 1u,             ///< GPIO开漏输出使能
}en_gpio_od_t;

/**
 *******************************************************************************
 ** \brief GPIO端口输入/输出值寄存器总线控制模式选择
 ******************************************************************************/
typedef enum en_gpio_ctrl_mode
{
    GpioFastIO = 0u,             ///< FAST IO 总线控制模式          
    GpioAHB  = 1u,               ///< AHB 总线控制模式
}en_gpio_ctrl_mode_t;

/**
 *******************************************************************************
 ** \brief GPIO中断触发方式类型定义
 ******************************************************************************/
typedef enum en_gpio_irqtype
{
    GpioIrqHigh     = 0u,           ///< GPIO高电平触发
    GpioIrqLow      = 1u,           ///< GPIO低电平触发
    GpioIrqRising   = 2u,           ///< GPIO上升沿触发
    GpioIrqFalling  = 3u,           ///< GPIO下降沿触发
}en_gpio_irqtype_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）端口中断模式类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_irqmode
{
    GpioSfIrqDpslpMode  = 1u,        ///< Deep Sleep模式
    GpioSfIrqActSlpMode = 0u,        ///< Active/Sleep模式
}en_gpio_sf_irqmode_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）HCLK输出门控类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_hclkout_g
{
    GpioSfHclkOutDisable = 0u,       ///< HCLK输出门控关闭
    GpioSfHclkOutEnable  = 1u,       ///< HCLK输出门控使能
}en_gpio_sf_hclkout_g_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）HCLK输出分频选择类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_hclkout_div
{
    GpioSfHclkOutDiv1 = 0u,         ///< HCLK
    GpioSfHclkOutDiv2 = 1u,         ///< HCLK/2
    GpioSfHclkOutDiv4 = 2u,         ///< HCLK/4
    GpioSfHclkOutDiv8 = 3u,         ///< HCLK/8
}en_gpio_sf_hclkout_div_t;
/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）PCLK输出门控类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_pclkout_g
{
    GpioSfPclkOutDisable = 0u,       ///< PCLK输出门控关闭
    GpioSfPclkOutEnable  = 1u,       ///< PCLK输出门控使能
}en_gpio_sf_pclkout_g_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）PCLK输出分频选择类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_pclkout_div
{
    GpioSfPclkOutDiv1 = 0u,         ///< PCLK
    GpioSfPclkOutDiv2 = 1u,         ///< PCLK/2
    GpioSfPclkOutDiv4 = 2u,         ///< PCLK/4
    GpioSfPclkOutDiv8 = 3u,         ///< PCLK/8
}en_gpio_sf_pclkout_div_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）IR输出极性选择类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_irpol
{
    GpioSfIrP = 0u,                  ///< IR正向输出
    GpioSfIrN = 1u,                  ///< IR反向输出
}en_gpio_sf_irpol_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）SSN通道类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_ssnspi
{
    GpioSpi0 = 0u,                   ///< SPI0 SSN 
    GpioSpi1 = 1u,                   ///< SPI1 SSN
}en_gpio_sf_ssnspi_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）SSN与外部时钟输入信号源选择类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_ssn_extclk
{
    GpioSfSsnExtClkH    = 0u,        ///< 高电平
    GpioSfSsnExtClkPA03 = 1u,        ///< PA03
    GpioSfSsnExtClkPA04 = 2u,        ///< PA04
    GpioSfSsnExtClkPA06 = 3u,        ///< PA06
    GpioSfSsnExtClkPA08 = 4u,        ///< PA08
    GpioSfSsnExtClkPA09 = 5u,        ///< PA09
    GpioSfSsnExtClkPA12 = 6u,        ///< PA12
    GpioSfSsnExtClkPA15 = 7u,        ///< PA15
    GpioSfSsnExtClkPB01 = 8u,        ///< PB01
    GpioSfSsnExtClkPB02 = 9u,        ///< PB02
    GpioSfSsnExtClkPB05 = 10u,       ///< PB05
    GpioSfSsnExtClkPB06 = 11u,       ///< PB06
    GpioSfSsnExtClkPB09 = 12u,       ///< PB09
    GpioSfSsnExtClkPB10 = 13u,       ///< PB10
    GpioSfSsnExtClkPB12 = 14u,       ///< PB12
    GpioSfSsnExtClkPB14 = 15u,       ///< PB14       
}en_gpio_sf_ssn_extclk_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）定时器互联功能选择类型定义
 ** \note  具体功能及含义请参考用户手册GPIO辅助寄存器描述
 ******************************************************************************/
typedef enum en_gpio_sf
{
    GpioSf0 = 0u,       ///< SF0:PX_SEL的配置功能
    GpioSf1 = 1u,       ///< SF1:辅助功能1
    GpioSf2 = 2u,       ///< SF2:辅助功能2
    GpioSf3 = 3u,       ///< SF3:辅助功能3
    GpioSf4 = 4u,       ///< SF4:辅助功能4
    GpioSf5 = 5u,       ///< SF5:辅助功能5 
    GpioSf6 = 6u,       ///< SF6:辅助功能6 
    GpioSf7 = 7u,       ///< SF7:辅助功能7
}en_gpio_sf_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）定时器门控类型选择数据类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_tim_g
{
    GpioSfTim0G  = 0u,                   ///<Tim0定时器GATE输入选择 
    GpioSfTim1G  = 3u,                   ///<Tim1定时器GATE输入选择 
    GpioSfTim2G  = 6u,                   ///<Tim2定时器GATE输入选择 
    GpioSfTim3G  = 9u,                   ///<Tim3定时器GATE输入选择 
    GpioSfLpTimG = 12u,                  ///<LPTim定时器GATE输入选择 
}en_gpio_sf_tim_g_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）定时器ETR类型选择数据类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_tim_e
{
    GpioSfTim0E  = 0u,                   ///<Tim0定时器ETR输入选择 
    GpioSfTim1E  = 3u,                   ///<Tim1定时器ETR输入选择 
    GpioSfTim2E  = 6u,                   ///<Tim2定时器ETR输入选择 
    GpioSfTim3E  = 9u,                   ///<Tim3定时器ETR输入选择 
    GpioSfLpTimE = 12u,                  ///<LPTim定时器ETR输入选择 
}en_gpio_sf_tim_e_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）定时器捕获输入类型选择数据类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_tim_c
{
    GpioSfTim0CA = 0u,                   ///<Tim0定时器CHA输入选择 
    GpioSfTim1CA = 3u,                   ///<Tim1定时器CHA输入选择 
    GpioSfTim2CA = 6u,                   ///<Tim2定时器CHA输入选择 
    GpioSfTim3CA = 9u,                   ///<Tim3定时器CH0A输入选择 
    GpioSfTim3CB = 12u,                  ///<Tim3定时器CH0B输入选择 
}en_gpio_sf_tim_c_t;

/**
 *******************************************************************************
 ** \brief GPIO 辅助功能（SF-Secondary Function）PCA捕获选择数据类型定义
 ******************************************************************************/
typedef enum en_gpio_sf_pca
{
    GpioSfPcaCH0 = 0u,                   ///<PCA_CH0捕获口输入选择 
    GpioSfPcaECI = 3u,                   ///<PCA ECI时钟输入选择 
}en_gpio_sf_pca_t;

/**
 *******************************************************************************
 ** \brief GPIO 端口配置结构体定义
 ******************************************************************************/
typedef struct
{
    en_gpio_dir_t       enDir;           ///< 端口方向配置
    en_gpio_drv_t       enDrv;           ///< 端口驱动能力配置
    en_gpio_pu_t        enPu;            ///< 端口上拉配置
    en_gpio_pd_t        enPd;            ///< 端口下拉配置
    en_gpio_od_t        enOD;            ///< 端口开漏输出配置
    en_gpio_ctrl_mode_t enCtrlMode;      ///< 端口输入/输出值寄存器总线控制模式配置
}stc_gpio_cfg_t;

/*******************************************************************************
 * Global definitions
 ******************************************************************************/
 
/******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/******************************************************************************
  Global function prototypes (definition in C source)
*******************************************************************************/
 ///< GPIO IO初始化/去初始化
en_result_t Gpio_Init(en_gpio_port_t enPort, en_gpio_pin_t enPin, stc_gpio_cfg_t *pstcGpioCfg);
 
///< GPIO 获取端口输入电平
boolean_t   Gpio_GetInputIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
uint16_t    Gpio_GetInputData(en_gpio_port_t enPort);

///< GPIO 设置端口输出
///< GPIO 端口输出电平配置及获取
en_result_t Gpio_WriteOutputIO(en_gpio_port_t enPort, en_gpio_pin_t enPin, boolean_t bVal);
boolean_t   Gpio_ReadOutputIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
///< GPIO 端口/引脚输出电平置位
en_result_t Gpio_SetPort(en_gpio_port_t enPort, uint16_t u16ValMsk);
en_result_t Gpio_SetIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
///< GPIO 端口/引脚输出电平清零
en_result_t Gpio_ClrPort(en_gpio_port_t enPort, uint16_t u16ValMsk);
en_result_t Gpio_ClrIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
///< GPIO 端口输出电平置位与清零设置
en_result_t Gpio_SetClrPort(en_gpio_port_t enPort, uint32_t u32ValMsk);

///< GPIO 设置端口为模拟功能
en_result_t Gpio_SetAnalogMode(en_gpio_port_t enPort, en_gpio_pin_t enPin);

///< GPIO 设置端口为端口复用功能
en_result_t Gpio_SetAfMode(en_gpio_port_t enPort, en_gpio_pin_t enPin, en_gpio_af_t enAf);

///< GPIO 端口中断控制功能使能/关闭
en_result_t Gpio_EnableIrq(en_gpio_port_t enPort, en_gpio_pin_t enPin, en_gpio_irqtype_t enType);
en_result_t Gpio_DisableIrq(en_gpio_port_t enPort, en_gpio_pin_t enPin, en_gpio_irqtype_t enType);
///< GPIO 中断状态获取
boolean_t   Gpio_GetIrqStatus(en_gpio_port_t enPort, en_gpio_pin_t enPin);
///< GPIO 中断标志清除
en_result_t Gpio_ClearIrq(en_gpio_port_t enPort, en_gpio_pin_t enPin);

///< GPIO 端口辅助功能配置
///< GPIO 中断模式配置
en_result_t Gpio_SfIrqModeCfg(en_gpio_sf_irqmode_t enIrqMode);
///< GPIO IR输出极性配置
en_result_t Gpio_SfIrPolCfg(en_gpio_sf_irpol_t enIrPolMode);
///< GPIO HCLK输出配置
en_result_t Gpio_SfHClkOutputCfg(en_gpio_sf_hclkout_g_t enGate, en_gpio_sf_hclkout_div_t enDiv);
///< GPIO PCLK输出配置
en_result_t Gpio_SfPClkOutputCfg(en_gpio_sf_pclkout_g_t enGate, en_gpio_sf_pclkout_div_t enDiv);
///< GPIO 外部时钟输入配置
en_result_t Gpio_SfExtClkCfg(en_gpio_sf_ssn_extclk_t enExtClk);
///< GPIO SPI SSN输入配置
en_result_t Gpio_SfSsnCfg(en_gpio_sf_ssnspi_t enSpi, en_gpio_sf_ssn_extclk_t enSsn);
///< GPIO Timer 门控输入配置
en_result_t Gpio_SfTimGCfg(en_gpio_sf_tim_g_t enTimG, en_gpio_sf_t enSf);
///< GPIO Timer ETR选择配置
en_result_t Gpio_SfTimECfg(en_gpio_sf_tim_e_t enTimE, en_gpio_sf_t enSf);
///< GPIO Timer 捕获输入配置
en_result_t Gpio_SfTimCCfg(en_gpio_sf_tim_c_t enTimC, en_gpio_sf_t enSf);
///< GPIO PCA捕获选择配置
en_result_t Gpio_SfPcaCfg(en_gpio_sf_pca_t enPca, en_gpio_sf_t enSf);


//@} // GpioGroup

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */
/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
