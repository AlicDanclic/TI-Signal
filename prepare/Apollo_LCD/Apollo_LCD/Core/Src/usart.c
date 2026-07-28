/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USART1 receives FPGA display frames on PA10; PA9 keeps boot diagnostics. */

/* USER CODE BEGIN 0 */

#define FPGA_UART_RING_SIZE          2048U
#define FPGA_UART_RING_MASK          (FPGA_UART_RING_SIZE - 1U)
#define FPGA_UART_SYNC_0             0xA5U
#define FPGA_UART_SYNC_1             0x5AU
#define FPGA_UART_VERSION_V1         0x01U
#define FPGA_UART_VERSION_V2         0x02U
#define FPGA_UART_FRAME_SIZE_V1      197U
#define FPGA_UART_FRAME_SIZE_V2      FPGA_UART_FRAME_SIZE
#define FPGA_UART_CRC_DATA_SIZE_V1   195U
#define FPGA_UART_CRC_DATA_SIZE_V2   245U
#define FPGA_UART_PERIOD_OFFSET      14U
#define FPGA_UART_SAMPLE_OFFSET_V1   14U
#define FPGA_UART_SAMPLE_OFFSET_V2   18U

static volatile uint8_t fpgaRxRing[FPGA_UART_RING_SIZE];
static volatile uint16_t fpgaRxHead;
static volatile uint16_t fpgaRxTail;
static volatile uint8_t fpgaRxStarted;

static uint8_t fpgaParserFrame[FPGA_UART_FRAME_SIZE];
static uint16_t fpgaParserLength;
static uint16_t fpgaExpectedFrameSize;
static FPGA_UART_Frame fpgaLatestFrame;
static uint8_t fpgaLatestReady;
static volatile FPGA_UART_Stats fpgaStats;
static uint8_t fpgaHaveSequence;

static uint16_t FPGA_UART_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for (index = 0U; index < length; index++)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }
  return crc;
}

static uint32_t FPGA_UART_ReadU32Le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static uint8_t FPGA_UART_RingPop(uint8_t *value)
{
  uint16_t tail = fpgaRxTail;

  if (tail == fpgaRxHead)
  {
    return 0U;
  }
  *value = fpgaRxRing[tail];
  fpgaRxTail = (uint16_t)((tail + 1U) & FPGA_UART_RING_MASK);
  return 1U;
}

static void FPGA_UART_ResetParser(uint8_t trailingByte)
{
  fpgaExpectedFrameSize = 0U;
  if (trailingByte == FPGA_UART_SYNC_0)
  {
    fpgaParserFrame[0] = FPGA_UART_SYNC_0;
    fpgaParserLength = 1U;
  }
  else
  {
    fpgaParserLength = 0U;
  }
}

static void FPGA_UART_ResyncFrame(void)
{
  uint16_t start;
  uint16_t index;
  uint16_t remaining;

  for (start = 1U; start + 1U < fpgaParserLength; start++)
  {
    if (fpgaParserFrame[start] == FPGA_UART_SYNC_0 &&
        fpgaParserFrame[start + 1U] == FPGA_UART_SYNC_1)
    {
      remaining = (uint16_t)(fpgaParserLength - start);
      for (index = 0U; index < remaining; index++)
      {
        fpgaParserFrame[index] = fpgaParserFrame[start + index];
      }
      fpgaParserLength = remaining;
      fpgaExpectedFrameSize = 0U;
      if (remaining >= 3U)
      {
        if (fpgaParserFrame[2] == FPGA_UART_VERSION_V1)
        {
          fpgaExpectedFrameSize = FPGA_UART_FRAME_SIZE_V1;
        }
        else if (fpgaParserFrame[2] == FPGA_UART_VERSION_V2)
        {
          fpgaExpectedFrameSize = FPGA_UART_FRAME_SIZE_V2;
        }
        else
        {
          fpgaParserLength = 0U;
        }
      }
      return;
    }
  }
  FPGA_UART_ResetParser(fpgaParserFrame[fpgaParserLength - 1U]);
}

static void FPGA_UART_AcceptFrame(void)
{
  uint16_t index;
  uint16_t packed;
  uint16_t byteOffset;
  uint8_t bitShift;
  uint8_t sequence = fpgaParserFrame[3];

  fpgaLatestFrame.sequence = sequence;
  fpgaLatestFrame.frequency_hz = FPGA_UART_ReadU32Le(&fpgaParserFrame[4]);
  fpgaLatestFrame.sample_div = FPGA_UART_ReadU32Le(&fpgaParserFrame[8]);
  fpgaLatestFrame.flags = fpgaParserFrame[13];
  if (fpgaParserFrame[2] == FPGA_UART_VERSION_V1)
  {
    fpgaLatestFrame.period_cycles = 0U;
    for (index = 0U; index < FPGA_UART_SAMPLE_COUNT; index++)
    {
      packed = fpgaParserFrame[FPGA_UART_SAMPLE_OFFSET_V1 + index];
      fpgaLatestFrame.samples[index] = (uint16_t)((packed << 2U) | (packed >> 6U));
    }
  }
  else
  {
    fpgaLatestFrame.period_cycles =
        FPGA_UART_ReadU32Le(&fpgaParserFrame[FPGA_UART_PERIOD_OFFSET]);
    for (index = 0U; index < FPGA_UART_SAMPLE_COUNT; index++)
    {
      byteOffset = (uint16_t)(FPGA_UART_SAMPLE_OFFSET_V2 + ((index * 10U) >> 3U));
      bitShift = (uint8_t)((index * 10U) & 7U);
      packed = (uint16_t)fpgaParserFrame[byteOffset] |
               ((uint16_t)fpgaParserFrame[byteOffset + 1U] << 8U);
      fpgaLatestFrame.samples[index] = (uint16_t)((packed >> bitShift) & 0x03FFU);
    }
  }
  fpgaLatestFrame.received_tick_ms = HAL_GetTick();
  fpgaLatestReady = 1U;

  if (fpgaHaveSequence != 0U && sequence != (uint8_t)(fpgaStats.last_sequence + 1U))
  {
    fpgaStats.sequence_errors++;
  }
  fpgaHaveSequence = 1U;
  fpgaStats.last_sequence = sequence;
  fpgaStats.last_frame_tick_ms = fpgaLatestFrame.received_tick_ms;
  fpgaStats.valid_frames++;
}

static void FPGA_UART_ParseByte(uint8_t value)
{
  uint16_t receivedCrc;
  uint16_t calculatedCrc;
  uint8_t accepted = 0U;

  if (fpgaParserLength == 0U)
  {
    if (value == FPGA_UART_SYNC_0)
    {
      fpgaParserFrame[0] = value;
      fpgaParserLength = 1U;
    }
    return;
  }

  if (fpgaParserLength == 1U)
  {
    if (value == FPGA_UART_SYNC_1)
    {
      fpgaParserFrame[1] = value;
      fpgaParserLength = 2U;
    }
    else if (value != FPGA_UART_SYNC_0)
    {
      fpgaParserLength = 0U;
    }
    return;
  }

  if (fpgaParserLength == 2U)
  {
    if (value == FPGA_UART_VERSION_V1)
    {
      fpgaExpectedFrameSize = FPGA_UART_FRAME_SIZE_V1;
    }
    else if (value == FPGA_UART_VERSION_V2)
    {
      fpgaExpectedFrameSize = FPGA_UART_FRAME_SIZE_V2;
    }
    else
    {
      fpgaStats.format_errors++;
      FPGA_UART_ResetParser(value);
      return;
    }
    fpgaParserFrame[2] = value;
    fpgaParserLength = 3U;
    return;
  }

  fpgaParserFrame[fpgaParserLength] = value;
  fpgaParserLength++;
  if (fpgaParserLength == 13U && fpgaParserFrame[12] != FPGA_UART_SAMPLE_COUNT)
  {
    fpgaStats.format_errors++;
    FPGA_UART_ResetParser(value);
    return;
  }
  if (fpgaExpectedFrameSize == 0U ||
      fpgaParserLength < fpgaExpectedFrameSize)
  {
    return;
  }

  receivedCrc = (uint16_t)fpgaParserFrame[fpgaExpectedFrameSize - 2U] |
                ((uint16_t)fpgaParserFrame[fpgaExpectedFrameSize - 1U] << 8U);
  calculatedCrc = FPGA_UART_Crc16(
      fpgaParserFrame,
      fpgaExpectedFrameSize == FPGA_UART_FRAME_SIZE_V1 ?
      FPGA_UART_CRC_DATA_SIZE_V1 : FPGA_UART_CRC_DATA_SIZE_V2);
  if (receivedCrc != calculatedCrc)
  {
    fpgaStats.crc_errors++;
  }
  else if (fpgaParserFrame[12] != FPGA_UART_SAMPLE_COUNT)
  {
    fpgaStats.format_errors++;
  }
  else
  {
    FPGA_UART_AcceptFrame();
    accepted = 1U;
  }
  if (accepted != 0U)
  {
    FPGA_UART_ResetParser(fpgaParserFrame[fpgaParserLength - 1U]);
  }
  else
  {
    FPGA_UART_ResyncFrame();
  }
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{
  /* 115200 波特率、8 位数据、无校验、1 位停止位。 */

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* 打开 USART1 时钟，并将 PA9/PA10 配置为 AF7。 */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

    HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef FPGA_UART_Start(void)
{
  fpgaRxStarted = 0U;
  CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);
  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
  fpgaRxHead = 0U;
  fpgaRxTail = 0U;
  fpgaParserLength = 0U;
  fpgaExpectedFrameSize = 0U;
  fpgaLatestReady = 0U;
  fpgaHaveSequence = 0U;
  fpgaStats.bytes_received = 0U;
  fpgaStats.valid_frames = 0U;
  fpgaStats.crc_errors = 0U;
  fpgaStats.format_errors = 0U;
  fpgaStats.ring_overflows = 0U;
  fpgaStats.uart_errors = 0U;
  fpgaStats.sequence_errors = 0U;
  fpgaStats.last_frame_tick_ms = 0U;
  fpgaStats.last_sequence = 0U;

  (void)huart1.Instance->SR;
  (void)huart1.Instance->DR;
  fpgaRxStarted = 1U;
  SET_BIT(huart1.Instance->CR3, USART_CR3_EIE);
  SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);
  return HAL_OK;
}

void FPGA_UART_IRQHandler(void)
{
  uint32_t status = huart1.Instance->SR;
  uint8_t value;
  uint16_t head;
  uint16_t next;

  if ((status & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U)
  {
    fpgaStats.uart_errors++;
  }

  if ((status & USART_SR_RXNE) == 0U)
  {
    if ((status & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U)
    {
      (void)huart1.Instance->DR;
    }
    return;
  }

  value = (uint8_t)huart1.Instance->DR;
  if (fpgaRxStarted == 0U)
  {
    return;
  }

  fpgaStats.bytes_received++;
  head = fpgaRxHead;
  next = (uint16_t)((head + 1U) & FPGA_UART_RING_MASK);
  if (next == fpgaRxTail)
  {
    fpgaStats.ring_overflows++;
    fpgaRxTail = (uint16_t)((fpgaRxTail + 1U) & FPGA_UART_RING_MASK);
  }
  fpgaRxRing[head] = value;
  fpgaRxHead = next;
}

void FPGA_UART_Process(void)
{
  uint8_t value;

  while (FPGA_UART_RingPop(&value) != 0U)
  {
    FPGA_UART_ParseByte(value);
  }

}

uint8_t FPGA_UART_TakeLatest(FPGA_UART_Frame *frame)
{
  if (frame == NULL || fpgaLatestReady == 0U)
  {
    return 0U;
  }
  *frame = fpgaLatestFrame;
  fpgaLatestReady = 0U;
  return 1U;
}

void FPGA_UART_GetStats(FPGA_UART_Stats *stats)
{
  if (stats == NULL)
  {
    return;
  }
  stats->bytes_received = fpgaStats.bytes_received;
  stats->valid_frames = fpgaStats.valid_frames;
  stats->crc_errors = fpgaStats.crc_errors;
  stats->format_errors = fpgaStats.format_errors;
  stats->ring_overflows = fpgaStats.ring_overflows;
  stats->uart_errors = fpgaStats.uart_errors;
  stats->sequence_errors = fpgaStats.sequence_errors;
  stats->last_frame_tick_ms = fpgaStats.last_frame_tick_ms;
  stats->last_sequence = fpgaStats.last_sequence;
}

/* USER CODE END 1 */
