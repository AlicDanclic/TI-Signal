/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AD9910 全功能循环验证（软件 SPI）
  * @details        : 每 3 秒切换：单频(20MHz) → OSK(幅度减半) → DRG(20~30MHz扫频)
  *                   → RAM(10MHz幅度正弦调制) → 三角波(~2.4kHz) → 方波(~2.4kHz)
  *                   所有功能均基于 Profile0，不使用 FPGA 并行口。
  *
  *                   修复记录：
  *                   - case1(OSK)：ASF 寄存器位域改为 [15:2]（原 <<18 错误），
  *                     并拉高 OSK 引脚（OSK 低电平输出强制为 0）
  *                   - case3(RAM)：调整为 先 RAMSetProfile 设边界 -> SelectProfile
  *                     -> RAMWrite（写入范围由选中的 RAM Profile 起止地址决定）
  *                   - case4/5：三角波/方波 = RAM 幅度回放 + 载波 FTW=0 + 相位90°，
  *                     输出直接等于包络（单极性 0~满幅，DDS 无负半周）
  *                   - 移除调试残留：state 写死、case2 的 while(1)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9910_sw.h"
#include <math.h>     // 用于 RAM 正弦表生成
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// -------------------- 引脚配置（根据 FPGA 原理图映射） --------------------
const AD9910_SW_PinConfig pin_cfg = {
    .sck_port = GPIOC, .sck_pin = GPIO_PIN_8,   // C08
    .sdi_port = GPIOC, .sdi_pin = GPIO_PIN_12,  // C12
    .cs_port  = GPIOC, .cs_pin  = GPIO_PIN_9,   // C09
    .rst_port = GPIOD, .rst_pin = GPIO_PIN_3,   // D03
    .iup_port = GPIOD, .iup_pin = GPIO_PIN_2,   // D02

    .pf0_port = GPIOE, .pf0_pin = GPIO_PIN_5,   // E05
    .pf1_port = GPIOE, .pf1_pin = GPIO_PIN_4,   // E04
    .pf2_port = GPIOE, .pf2_pin = GPIO_PIN_3,   // E03
    .osk_port = GPIOE, .osk_pin = GPIO_PIN_6,   // E06
    .drc_port = GPIOC, .drc_pin = GPIO_PIN_10,  // C10
    .drh_port = GPIOC, .drh_pin = GPIO_PIN_11,  // C11
};

AD9910_SW_HandleTypeDef had9910;

// RAM 数据缓冲区（1024 点，全局静态）
static uint32_t ram_data[1024];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void generate_ram_sine_wave(uint16_t amplitude_max);
static void generate_ram_triangle_wave(uint16_t amplitude_max);
static void generate_ram_square_wave(uint16_t amplitude_max);
static void play_ram_amp_wave(uint16_t step_rate, float carrier_hz);
static void set_single_tone(float freq_hz, uint16_t amp, float phase_deg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  生成半正弦波幅度表（逆序存储，适配 RAM 写入顺序）
  * @param  amplitude_max : 14-bit 最大幅度值 (0~0x3FFF)
  */
static void generate_ram_sine_wave(uint16_t amplitude_max)
{
    for (int i = 0; i < 1024; i++) {
        double val = amplitude_max * sin((double)i * 3.1415926 / 1023.0);
        uint32_t reg_val = (uint32_t)(val + 0.5) << 18; // RAM 幅度字格式：ASF 位于 [31:18]
        // 逆序存放：索引 0 对应最高地址（1023），满足驱动要求
        ram_data[1023 - i] = reg_val;
    }
}

/**
  * @brief  生成三角波幅度表（逆序存储）：0 -> max -> 0
  */
static void generate_ram_triangle_wave(uint16_t amplitude_max)
{
    for (int i = 0; i < 1024; i++) {
        uint32_t v = (i < 512) ? ((uint32_t)amplitude_max * i / 512)
                               : ((uint32_t)amplitude_max * (1023 - i) / 512);
        ram_data[1023 - i] = v << 18;
    }
}

/**
  * @brief  生成方波幅度表（逆序存储）：前半周 max，后半周 0
  */
static void generate_ram_square_wave(uint16_t amplitude_max)
{
    for (int i = 0; i < 1024; i++) {
        uint32_t v = (i < 512) ? amplitude_max : 0;
        ram_data[1023 - i] = v << 18;
    }
}

/**
  * @brief  RAM 幅度回放公共流程（顺序不能乱）：
  *         失能 RAM -> 设 Profile 边界 -> 选中 profile -> 写数据 -> 使能回放 -> 载波
  * @param  step_rate  地址步进速率：波形频率 = 250MHz / (step_rate × 1024)
  * @param  carrier_hz 载波频率；基带波形(三角/方波)传 0 并配合相位 90°
  */
static void play_ram_amp_wave(uint16_t step_rate, float carrier_hz)
{
    AD9910_SW_RAMDisable(&had9910);
    AD9910_SW_RAMSetProfile(&had9910, 0, step_rate, 0, 1023, AD9910_RAM_MODE_CONT_RECIRC);
    AD9910_SW_SelectProfile(&had9910, 0);
    AD9910_SW_RAMWrite(&had9910, ram_data, 1024);
    AD9910_SW_RAMEnable(&had9910, AD9910_CFR1_RAM_DEST_AMP);
    AD9910_SW_SetFTW(&had9910, carrier_hz);
}

/**
  * @brief  便捷函数：设置 Profile0 单频参数
  */
static void set_single_tone(float freq_hz, uint16_t amp, float phase_deg)
{
    AD9910_SW_SingleToneSet(&had9910, 0, freq_hz, amp, phase_deg);
    AD9910_SW_SelectProfile(&had9910, 0);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();   // CubeMX 生成，包含所有 GPIO 初始化

  /* USER CODE BEGIN 2 */
  // 1. AD9910 初始化（复位、PLL 40 倍频至 1GHz）
  had9910.sysclk_hz = 1000000000UL;
  AD9910_SW_Init(&had9910, &pin_cfg);

  // 2. RAM 数据表在各状态中按需生成

  // 3. 初始化状态机
  uint8_t state = 2;
  const uint32_t interval_ms = 3000;

  /* USER CODE END 2 */

  while (1)
  {
    switch (state)
    {
        case 0:  // ---------- 单频 20MHz ----------
        {
            AD9910_SW_SingleToneInit(&had9910);
            set_single_tone(20e6f, 0x3FFF, 0.0f);
            break;
        }

        case 1:  // ---------- OSK 自动幅度控制（幅度减半） ----------
        {
            // 载波 20MHz（OSK 使能后幅度由 OSK 接管，Profile 幅度无效，频率照常）
            AD9910_SW_SingleToneInit(&had9910);
            set_single_tone(20e6f, 0x3FFF, 0.0f);

            // 自动 OSK：内部已写满幅 ASF(0x3FFF) + 斜坡速率 ARR(0x0100)
            AD9910_SW_OSKInit(&had9910);
            // 幅度减半：ASF[15:2] = 0x2000，斜坡从 0 升到一半约 8ms
            AD9910_SW_SetASF(&had9910, 0x2000, 0x0100);
            // 关键！OSK 引脚(E06)为低时输出强制为 0，必须拉高
            AD9910_SW_OSKPin(&had9910, 1);
            break;
        }

        case 2:  // ---------- DRG 频率三角波扫描 (20~30MHz) ----------
        {
            // 清除 DRAMP 累加器，确保从下限开始
            AD9910_SW_Write32(&had9910, AD9910_REG_CFR1,
                              AD9910_CFR1_BASE | AD9910_CFR1_CLR_DRAMP);
            AD9910_SW_IOUpdate(&had9910);
            AD9910_SW_Write32(&had9910, AD9910_REG_CFR1, AD9910_CFR1_BASE);
            AD9910_SW_IOUpdate(&had9910);

            // 初始化 DRG（扫频目标），再覆盖 CFR2 增加 NODWELL_HIGH/LOW 实现自动往返
            AD9910_SW_DRGInit(&had9910, AD9910_DRG_FREQ);
            AD9910_SW_Write32(&had9910, AD9910_REG_CFR2,
                              AD9910_CFR2_BASE |
                              AD9910_CFR2_DRG_ENABLE |
                              ((uint32_t)AD9910_DRG_FREQ << 20) |
                              AD9910_CFR2_DRG_NODWELL_HIGH |
                              AD9910_CFR2_DRG_NODWELL_LOW);
            AD9910_SW_IOUpdate(&had9910);

            // 配置斜坡参数：上限 30MHz，下限 20MHz，步进 100kHz，速率 5000
            AD9910_SW_DRGConfigFreq(&had9910,
                                    30e6f,   // upper
                                    20e6f,   // lower
                                    1e5f,    // dec step
                                    1e5f,    // inc step
                                    5000,    // neg rate
                                    5000);   // pos rate

            // 方向向上（DRCTL=0），释放保持（DRH=0）
            AD9910_SW_DRGDirection(&had9910, 1);
            AD9910_SW_DRGHold(&had9910, 0);
            break;
        }

        case 3:  // ---------- RAM 幅度回放（载波 10MHz，半正弦幅度调制） ----------
        {
            generate_ram_sine_wave(0x3FFF);
            play_ram_amp_wave(100, 10e6f);   // 调制频率 ≈ 250M/(100×1024) ≈ 2.4kHz
            break;
        }

        case 4:  // ---------- 三角波（基带 ≈ 2.4kHz，单极性 0~满幅） ----------
        {
            generate_ram_triangle_wave(0x3FFF);
            play_ram_amp_wave(100, 0.0f);    // 载波 FTW=0
            AD9910_SW_SetPhase(&had9910, 90.0f);  // 相位 90° -> sin=1，输出=包络
            break;
        }

        case 5:  // ---------- 方波（基带 ≈ 2.4kHz，单极性 0~满幅） ----------
        {
            generate_ram_square_wave(0x3FFF);
            play_ram_amp_wave(100, 0.0f);    // 载波 FTW=0
            AD9910_SW_SetPhase(&had9910, 90.0f);  // 相位 90° -> sin=1，输出=包络
            break;
        }

        default:
            break;
    }
    while (1)
    {
      /* code */
    }
    
    // 每 3 秒切换到下一个状态：0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 0 ...
    state = (state + 1) % 6;
    HAL_Delay(interval_ms);
  }
}

/**
  * @brief System Clock Configuration（CubeMX 生成的代码，保持原样）
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/**
  * @brief  Error handler.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 可添加串口打印 */
}
#endif
