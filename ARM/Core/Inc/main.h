/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERR_LED2_Pin LL_GPIO_PIN_2
#define ERR_LED2_GPIO_Port GPIOE
#define ERR_LED1_Pin LL_GPIO_PIN_3
#define ERR_LED1_GPIO_Port GPIOE
#define ACQ_LED2_Pin LL_GPIO_PIN_4
#define ACQ_LED2_GPIO_Port GPIOE
#define ACQ_LED1_Pin LL_GPIO_PIN_5
#define ACQ_LED1_GPIO_Port GPIOE
#define PWR_LED1_Pin LL_GPIO_PIN_6
#define PWR_LED1_GPIO_Port GPIOE
#define PWR_LED2_Pin LL_GPIO_PIN_13
#define PWR_LED2_GPIO_Port GPIOC
#define KeyPad_IO1_Pin LL_GPIO_PIN_5
#define KeyPad_IO1_GPIO_Port GPIOA
#define KeyPad_IO0_Pin LL_GPIO_PIN_6
#define KeyPad_IO0_GPIO_Port GPIOA
#define W5500_SCLK_Pin LL_GPIO_PIN_2
#define W5500_SCLK_GPIO_Port GPIOB
#define W5500_nRST_Pin LL_GPIO_PIN_7
#define W5500_nRST_GPIO_Port GPIOC
#define W5500_nINT_Pin LL_GPIO_PIN_8
#define W5500_nINT_GPIO_Port GPIOC
#define W5500_MOSI_Pin LL_GPIO_PIN_9
#define W5500_MOSI_GPIO_Port GPIOC
#define MCU_MCO1_Pin LL_GPIO_PIN_8
#define MCU_MCO1_GPIO_Port GPIOA
#define W5500_MISO_Pin LL_GPIO_PIN_10
#define W5500_MISO_GPIO_Port GPIOC
#define W5500_nCS_Pin LL_GPIO_PIN_6
#define W5500_nCS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* LED */
#define ERR_LED1_OFF  	LL_GPIO_SetOutputPin  (GPIOE, ERR_LED1_Pin);
#define ERR_LED1_ON   	LL_GPIO_ResetOutputPin(GPIOE, ERR_LED1_Pin);
#define ERR_LED2_OFF		LL_GPIO_SetOutputPin  (GPIOE, ERR_LED2_Pin);
#define ERR_LED2_ON   	LL_GPIO_ResetOutputPin(GPIOE, ERR_LED2_Pin);
#define ACQ_LED1_OFF  	LL_GPIO_SetOutputPin  (GPIOE, ACQ_LED1_Pin);
#define ACQ_LED1_ON   	LL_GPIO_ResetOutputPin(GPIOE, ACQ_LED1_Pin);
#define ACQ_LED2_OFF  	LL_GPIO_SetOutputPin  (GPIOE, ACQ_LED2_Pin);
#define ACQ_LED2_ON   	LL_GPIO_ResetOutputPin(GPIOE, ACQ_LED2_Pin);
#define ACQ_LED1_TOGGLE	LL_GPIO_TogglePin(GPIOE, ACQ_LED1_Pin);
#define ACQ_LED2_TOGGLE	LL_GPIO_TogglePin(GPIOE, ACQ_LED2_Pin);
#define PWR_LED1_OFF  	LL_GPIO_SetOutputPin  (GPIOE, PWR_LED1_Pin);
#define PWR_LED1_ON   	LL_GPIO_ResetOutputPin(GPIOE, PWR_LED1_Pin);
#define PWR_LED2_OFF  	LL_GPIO_SetOutputPin  (GPIOC, PWR_LED2_Pin);
#define PWR_LED2_ON   	LL_GPIO_ResetOutputPin(GPIOC, PWR_LED2_Pin);

// System events
#define ATTR_CHANGE_EVT					( 1 << 0 )	//!< 属性值变化
#define TCP_RECV_EVT						( 1 << 1 ) 	//!< TCP端口接收到一帧
#define TCP_PROCESSCLP_EVT			( 1 << 2 ) 	//!< TCP帧协议处理完毕
#define TCP_SEND_EVT            ( 1 << 3 )	//!< TCP端口回复完成
#define UDP_RECV_EVT						( 1 << 4 )	//!< UDP端口接收到一帧
#define UDP_DTPROCESSCLP_EVT		( 1 << 5 )	//!< UDP数据帧协议处理完毕
#define UDP_EVTPROCESSCLP_EVT		( 1 << 6 )	//!< UDP事件帧协议处理完毕
#define EEG_IMP_MODE						( 1 << 7 )	//!< 阻抗检测模式
#define CHX_IMP_START						( 1 << 8 )	//!< 一个通道阻抗检测开始
//#define	CHX_IMP_REDY						( 1 << 9 )	//!< 一个通道的采样值待读取
#define CHX_IMP_DONE						( 1 << 10 )	//!< 一通道阻抗值已读取完毕
#define EEG_DATA_START_EVT			( 1 << 11 )	//!< 一包AD数据开始采集
#define EEG_DATA_ACQ_EVT				( 1 << 12 )	//!< 一包AD数据采集中
#define EEG_DATA_CPL_EVT				( 1 << 13 )	//!< 一包AD数据采集完成
#define EEG_STOP_EVT						( 1 << 14 )	//!< AD数据暂停采集
#define TRIGGER_EVT							(	1	<< 15 )	//!< 标签事件
#define POWERDOWN_EVT						(	1	<< 16 )	//!< 异常断电事件
#define SOCKETDOWN_EVT					(	1	<< 17 )	//!< w5500连接断开事件

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
