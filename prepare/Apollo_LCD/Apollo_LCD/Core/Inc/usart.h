/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USART1 RX carries fixed FPGA display frames; TX is only used at boot. */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USART1 句柄由 usart.c 定义，应用层通过 HAL_UART_Transmit 使用。 */
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

#define FPGA_UART_SAMPLE_COUNT       181U
#define FPGA_UART_FRAME_SIZE         247U
#define FPGA_UART_SIGNAL_PRESENT     0x01U

/* USER CODE END Private defines */

/* 初始化 PA9/PA10 复用和 115200-8-N-1 串口参数。 */
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct
{
  uint32_t frequency_hz;
  uint32_t sample_div;
  uint32_t period_cycles;
  uint32_t received_tick_ms;
  uint16_t samples[FPGA_UART_SAMPLE_COUNT];
  uint8_t flags;
  uint8_t sequence;
} FPGA_UART_Frame;

typedef struct
{
  uint32_t bytes_received;
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t format_errors;
  uint32_t ring_overflows;
  uint32_t uart_errors;
  uint32_t sequence_errors;
  uint32_t last_frame_tick_ms;
  uint8_t last_sequence;
} FPGA_UART_Stats;

HAL_StatusTypeDef FPGA_UART_Start(void);
void FPGA_UART_IRQHandler(void);
void FPGA_UART_Process(void);
uint8_t FPGA_UART_TakeLatest(FPGA_UART_Frame *frame);
void FPGA_UART_GetStats(FPGA_UART_Stats *stats);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

