/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "lcd.h"
#include "waveform.h"
#include "touch.h"

/*
 * 应用启动顺序：HAL/时钟 -> GPIO/串口/FMC -> LCD -> 触摸 -> 示波器界面。
 * FMC 必须先初始化，否则访问 LCD 映射地址会触发总线错误。
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static FPGA_UART_Frame fpgaFrame;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LCD_ReportStatus(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  /* 初始化 HAL、SysTick 和 Flash 接口；这是所有 HAL 外设的前置步骤。 */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  /* 使用外部 25 MHz 晶振配置 180 MHz 系统时钟。 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 先关闭背光，再初始化总线和 LCD，避免上电白屏闪烁。 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_FMC_Init();
  /* USER CODE BEGIN 2 */
  /* FMC 已就绪后才能访问 0x6007FFFE / 0x6007FFFF 的 LCD 命令和数据地址。 */
  LCD_Init();
  LCD_ReportStatus();
  /* 上电显示静态波形界面，避免在主循环反复整屏刷新而造成闪烁。 */
  /* 触摸初始化失败时仍显示波形，右侧状态栏会提示 NOT FOUND。 */
  Waveform_Init(Touch_Init());
  if (FPGA_UART_Start() != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t lastFrameTick;
    static uint32_t lastTouchPollTick;
    static uint32_t lastStatsTick;
    static uint32_t lastTouchRetryTick;
    static uint8_t touchWasDown;
    static uint8_t touchReleaseCount;
    FPGA_UART_Stats uartStats;
    uint16_t touchX;
    uint16_t touchY;
    uint8_t touchIsDown;

    FPGA_UART_Process();
    if (FPGA_UART_TakeLatest(&fpgaFrame) != 0U)
    {
      Waveform_SubmitFrame(fpgaFrame.samples, fpgaFrame.frequency_hz,
                           fpgaFrame.sample_div, fpgaFrame.period_cycles,
                           fpgaFrame.flags,
                           fpgaFrame.sequence);
    }

    if (Touch_GetController() == TOUCH_CONTROLLER_NONE &&
        (HAL_GetTick() - lastTouchRetryTick) >= 500U)
    {
      lastTouchRetryTick = HAL_GetTick();
      touchWasDown = 0U;
      touchReleaseCount = 0U;
      Waveform_SetTouchAvailable(0U);
      if (Touch_Reconnect() != 0U)
      {
        Waveform_SetTouchAvailable(1U);
      }
    }

    /* 每 10 ms 轮询触摸，只在“释放 -> 按下”边沿执行一次操作。 */
    if ((HAL_GetTick() - lastTouchPollTick) >= 10U)
    {
      lastTouchPollTick = HAL_GetTick();
      touchIsDown = Touch_Read(&touchX, &touchY);
      if (touchIsDown != 0U && touchWasDown == 0U)
      {
        Waveform_ShowTouch(touchX, touchY);
        Waveform_HandleTouch(touchX, touchY);
      }
      if (touchIsDown != 0U)
      {
        touchWasDown = 1U;
        touchReleaseCount = 0U;
      }
      else if (touchWasDown != 0U)
      {
        if (++touchReleaseCount >= 3U)
        {
          touchWasDown = 0U;
          touchReleaseCount = 0U;
        }
      }
    }

    /* 每 40 ms 刷新一次，约 25 FPS；局部重绘避免整屏闪烁。 */
    if ((HAL_GetTick() - lastFrameTick) >= 40U)
    {
      Waveform_Update();
      lastFrameTick = HAL_GetTick();
    }
    if ((HAL_GetTick() - lastStatsTick) >= 250U)
    {
      lastStatsTick = HAL_GetTick();
      FPGA_UART_GetStats(&uartStats);
      Waveform_SetLinkStats(uartStats.bytes_received, uartStats.valid_frames,
                            uartStats.crc_errors + uartStats.format_errors +
                            uartStats.ring_overflows + uartStats.uart_errors +
                            uartStats.sequence_errors);
    }
    HAL_Delay(2U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* HSE=25 MHz，PLL 后 SYSCLK=180 MHz，APB1=45 MHz，APB2=90 MHz。 */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* 板载 HSE 为 25 MHz：25 / 25 * 360 / 2 = 180 MHz。 */
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void LCD_ReportStatus(void)
{
  /* 通过串口输出控制器 ID，便于区分 NT35510、SSD1963、ILI9806 等型号。 */
  static const char hex[] = "0123456789ABCDEF";
  /* 串口用于确认控制器 ID；显示异常时首先检查这里是否为支持的 ID。 */
  uint16_t id = LCD_GetID();
  uint8_t message[] = "LCD ID: 0x0000, status: UNKNOWN\r\n";

  message[10] = (uint8_t)hex[(id >> 12) & 0x0FU];
  message[11] = (uint8_t)hex[(id >> 8) & 0x0FU];
  message[12] = (uint8_t)hex[(id >> 4) & 0x0FU];
  message[13] = (uint8_t)hex[id & 0x0FU];
  if (LCD_IsSupported() != 0U)
  {
    message[24] = 'O';
    message[25] = 'K';
    message[26] = ' ';
    message[27] = ' ';
    message[28] = ' ';
    message[29] = ' ';
    message[30] = ' ';
  }
  HAL_UART_Transmit(&huart1, message, sizeof(message) - 1U, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* 初始化失败时关闭中断并停机，便于调试器定位故障位置。 */
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
