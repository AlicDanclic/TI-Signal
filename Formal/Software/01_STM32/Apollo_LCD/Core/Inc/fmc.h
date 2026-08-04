/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : FMC.h
  * Description        : This file provides code for the configuration
  *                      of the FMC peripheral.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FMC_H
#define __FMC_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*
 * FMC 将外部 MCU 并口 LCD 映射成异步 SRAM 空间。
 * LCD 命令/数据寄存器的具体地址见 lcd.h，初始化必须早于 LCD_Init。
 */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* FMC Bank1 的 SRAM 外设句柄，HAL 会用它配置 NE1、NOE、NWE 和数据线。 */
extern SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* 配置 16 位异步 SRAM 总线及 LCD 所需的读写时序。 */
void MX_FMC_Init(void);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram);
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__FMC_H */

/**
  * @}
  */

/**
  * @}
  */
