/**
  * @file    si5351.h
  * @brief   Si5351A 时钟发生器 HAL 驱动（硬件 I2C · 句柄配置版）
  * @details 通过 STM32 硬件 I2C 外设与 Si5351A 通信（7 位地址 0x60，
  *          8 位寄存器地址）。I2C 外设请在 CubeMX 中配置
  *          （100kHz/400kHz 均可，7-bit 地址模式）。
  *
  * 两种使用方式：
  *   1. 固定配置：直接烧写 ClockBuilder Pro 导出的寄存器表
  *      （Si5351A-RevB-Registers.h），再调 SI5351_PLLResetAndRun()；
  *   2. 动态频率：SI5351_Init() + SI5351_SetupCLK0/CLK2() 任意改频。
  *
  * 注意：硬件 I2C 在从机无应答时会等到超时，Si5351 模块未连接时
  *       本驱动函数会返回错误而不会卡死（所有 I2C 操作都有超时），
  *       但调试时仍建议确认模块已上电、SDA/SCL 接好且有上拉。
  */

#ifndef __SI5351_H__
#define __SI5351_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C 超时（ms）：模块未接/无应答时按此超时返回错误，避免卡死 */
#ifndef SI5351_I2C_TIMEOUT_MS
#define SI5351_I2C_TIMEOUT_MS   100u
#endif

/* Si5351A 7 位地址 0x60，HAL 需要左移 1 位的 8 位地址 */
#define SI5351_I2C_ADDR         (0x60u << 1)

/* ======================== 设备句柄 ======================== */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /* I2C 句柄（CubeMX 配置好后传入） */
    uint8_t  addr;                  /* 8 位 I2C 地址，填 0 则默认 0x60<<1 */
    uint32_t xtal_hz;               /* 晶振频率 Hz，填 0 则默认 25MHz */
    uint8_t  xtal_load;             /* 晶振负载电容，取 SI5351_CRYSTAL_LOAD_x */
    int32_t  correction;            /* 频偏校准：100MHz 处实测偏差(Hz)，无校准填 0 */
} SI5351_HandleTypeDef;

/* ======================== 枚举 ======================== */
typedef enum {
    SI5351_PLL_A = 0,
    SI5351_PLL_B,
} SI5351_PLL_t;

typedef enum {
    SI5351_R_DIV_1   = 0,
    SI5351_R_DIV_2   = 1,
    SI5351_R_DIV_4   = 2,
    SI5351_R_DIV_8   = 3,
    SI5351_R_DIV_16  = 4,
    SI5351_R_DIV_32  = 5,
    SI5351_R_DIV_64  = 6,
    SI5351_R_DIV_128 = 7,
} SI5351_RDiv_t;

typedef enum {
    SI5351_DRIVE_2MA = 0x00,    /* ~2.2 dBm */
    SI5351_DRIVE_4MA = 0x01,    /* ~7.5 dBm */
    SI5351_DRIVE_6MA = 0x02,    /* ~9.5 dBm */
    SI5351_DRIVE_8MA = 0x03,    /* ~10.7 dBm */
} SI5351_Drive_t;

typedef enum {
    SI5351_CRYSTAL_LOAD_6PF  = (1u << 6),
    SI5351_CRYSTAL_LOAD_8PF  = (2u << 6),
    SI5351_CRYSTAL_LOAD_10PF = (3u << 6),
} SI5351_CrystalLoad_t;

/* 寄存器地址（见 Skyworks AN619） */
enum {
    SI5351_REG_DEVICE_STATUS            = 0,
    SI5351_REG_INTERRUPT_STATUS_STICKY  = 1,
    SI5351_REG_INTERRUPT_STATUS_MASK    = 2,
    SI5351_REG_OUTPUT_ENABLE_CONTROL    = 3,
    SI5351_REG_OEB_PIN_ENABLE_CONTROL   = 9,
    SI5351_REG_PLL_INPUT_SOURCE         = 15,
    SI5351_REG_CLK0_CONTROL             = 16,   /* CLK1~7 依次 17~23 */
    SI5351_REG_CLK3_0_DISABLE_STATE     = 24,
    SI5351_REG_CLK7_4_DISABLE_STATE     = 25,
    SI5351_REG_MSNA_BASE                = 26,   /* PLLA 多合成参数基址 */
    SI5351_REG_MSNB_BASE                = 34,   /* PLLB 多合成参数基址 */
    SI5351_REG_MS0_BASE                 = 42,   /* MS0 参数基址，MS1=50, MS2=58 */
    SI5351_REG_MS6_P1                   = 90,
    SI5351_REG_MS7_P1                   = 91,
    SI5351_REG_CLK6_7_OUTPUT_DIVIDER    = 92,
    SI5351_REG_CLK0_PHOFF               = 165,  /* CLK1~7 依次 166~172 */
    SI5351_REG_PLL_RESET                = 177,
    SI5351_REG_XTAL_LOAD_CAP            = 183,
};

/* ======================== 配置结构体 ======================== */
typedef struct {
    int32_t mult;       /* 整数部分 a */
    int32_t num;        /* 分数分子 b */
    int32_t denom;      /* 分数分母 c */
} SI5351_PLLConfig_t;

typedef struct {
    uint8_t       allowIntegerMode;
    int32_t       div;          /* 整数部分 x */
    int32_t       num;          /* 分数分子 y */
    int32_t       denom;        /* 分数分母 z */
    SI5351_RDiv_t rdiv;
} SI5351_OutputConfig_t;

/* 寄存器表项（用于批量烧写，兼容 ClockBuilder Pro 导出的表） */
typedef struct {
    uint16_t addr;
    uint8_t  value;
} SI5351_RegWrite_t;

/* ======================== 函数声明 ======================== */

/* 底层读写（带超时，模块未接返回 HAL_ERROR/HAL_TIMEOUT 而不卡死） */
HAL_StatusTypeDef SI5351_WriteReg(SI5351_HandleTypeDef *hdev, uint8_t reg, uint8_t val);
HAL_StatusTypeDef SI5351_ReadReg(SI5351_HandleTypeDef *hdev, uint8_t reg, uint8_t *val);

/**
  * @brief 基础初始化：关闭所有输出 -> 所有输出驱动掉电 -> 晶振负载电容
  * @note  hdev 先填 hi2c（addr/xtal_hz/xtal_load 填 0 用默认值），再调本函数
  */
HAL_StatusTypeDef SI5351_Init(SI5351_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);

/**
  * @brief 批量烧写寄存器表（通用，自定义表用）
  * @note  只写寄存器，不复位 PLL、不使能输出，之后请调 SI5351_PLLResetAndRun()
  */
HAL_StatusTypeDef SI5351_LoadRegisters(SI5351_HandleTypeDef *hdev,
                                       const SI5351_RegWrite_t *list, uint16_t count);

/**
  * @brief 烧写 ClockBuilder Pro 导出的配置（Si5351A-RevB-Registers.h），
  *        并自动完成 PLL 复位 + 输出使能，调用即生效
  * @note  配置变更时用 ClockBuilder 重新生成该头文件替换即可，驱动不用改
  */
HAL_StatusTypeDef SI5351_LoadClockBuilderConfig(SI5351_HandleTypeDef *hdev);

/**
  * @brief PLL 复位（177 = 0xA0）并使能所有输出（3 = 0x00）
  * @note  ClockBuilder 导出表不含这两步，烧表后必须调用本函数才有输出
  */
HAL_StatusTypeDef SI5351_PLLResetAndRun(SI5351_HandleTypeDef *hdev);

/* 使能/关闭输出：mask 位 1 = 使能，例 0x01 只开 CLK0，0x07 全开 */
HAL_StatusTypeDef SI5351_EnableOutputs(SI5351_HandleTypeDef *hdev, uint8_t mask);

/* ---- 动态频率接口（算法见 AN619，误差 < 6Hz @ 8kHz~200MHz） ---- */
void SI5351_Calc(SI5351_HandleTypeDef *hdev, int32_t freq_hz,
                 SI5351_PLLConfig_t *pll_conf, SI5351_OutputConfig_t *out_conf);
HAL_StatusTypeDef SI5351_SetupPLL(SI5351_HandleTypeDef *hdev, SI5351_PLL_t pll,
                                  const SI5351_PLLConfig_t *conf);
HAL_StatusTypeDef SI5351_SetupOutput(SI5351_HandleTypeDef *hdev, uint8_t clk,
                                     SI5351_PLL_t pll_src, SI5351_Drive_t drive,
                                     const SI5351_OutputConfig_t *conf, uint8_t phase_offset);

/* 便捷接口：CLK0 用 PLLA、CLK2 用 PLLB，互不影响可同时独立改频 */
HAL_StatusTypeDef SI5351_SetupCLK0(SI5351_HandleTypeDef *hdev, int32_t freq_hz, SI5351_Drive_t drive);
HAL_StatusTypeDef SI5351_SetupCLK2(SI5351_HandleTypeDef *hdev, int32_t freq_hz, SI5351_Drive_t drive);

#ifdef __cplusplus
}
#endif
#endif /* __SI5351_H__ */
