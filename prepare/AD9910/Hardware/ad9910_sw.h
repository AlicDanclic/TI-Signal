/**
  * @file    ad9910_sw.h
  * @brief   AD9910 DDS 驱动头文件 (软件模拟 SPI · 结构体配置版)
  * @details 用普通 GPIO 模拟 SPI 时序与 AD9910 通信（对应凌智参考程序的
  *          Write_8bit 位 bang 方式），不占用 SPI 外设，任意引脚均可。
  *          所有 GPIO 引脚通过结构体字段传入，不硬编码引脚宏。
  *
  * 使用方式：
  *   1. 定义 AD9910_SW_PinConfig，填好 SCK/SDI/CS/MRT/IUP 等 GPIO 引脚
  *   2. 定义 AD9910_SW_HandleTypeDef，填好 sysclk_hz
  *   3. 调用 AD9910_SW_Init(&had9910, &pin_cfg)
  */

#ifndef __AD9910_SW_H__
#define __AD9910_SW_H__

#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include "ad9910_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 位 bang 时序延时循环数：72MHz 下约 50 -> SCK 约 1MHz，
   AD9910 串口 SCLK 上限 70MHz，软件模拟远达不到，延时只是保险 */
#ifndef AD9910_SW_DELAY_LOOP
#define AD9910_SW_DELAY_LOOP   50u
#endif

/* ======================== 引脚配置结构体 ======================== */
typedef struct {
    GPIO_TypeDef *sck_port;  uint16_t sck_pin;    /* SCK  串行时钟(必须) */
    GPIO_TypeDef *sdi_port;  uint16_t sdi_pin;    /* SDI  串行数据(必须) */
    GPIO_TypeDef *cs_port;   uint16_t cs_pin;     /* CSN  片选,低有效(必须) */
    GPIO_TypeDef *rst_port;  uint16_t rst_pin;    /* MRT  主复位,高有效(必须) */
    GPIO_TypeDef *iup_port;  uint16_t iup_pin;    /* IUP  I/O_UPDATE(必须) */

    GPIO_TypeDef *pf0_port;  uint16_t pf0_pin;    /* PF0~PF2 Profile 选择(可 NULL) */
    GPIO_TypeDef *pf1_port;  uint16_t pf1_pin;
    GPIO_TypeDef *pf2_port;  uint16_t pf2_pin;

    GPIO_TypeDef *osk_port;  uint16_t osk_pin;    /* OSK  输出移位键控(可 NULL) */
    GPIO_TypeDef *drc_port;  uint16_t drc_pin;    /* DRC  DRCTL 斜坡方向(可 NULL) */
    GPIO_TypeDef *drh_port;  uint16_t drh_pin;    /* DRH  DRHOLD 斜坡保持(可 NULL) */
} AD9910_SW_PinConfig;

/* ======================== 设备句柄 ======================== */
typedef struct {
    AD9910_SW_PinConfig pin;         /* 引脚配置 */
    uint32_t            sysclk_hz;   /* DDS 系统时钟, PLL x40 时填 1000000000 */
} AD9910_SW_HandleTypeDef;

/* ======================== 函数声明 ======================== */

/* 初始化：GPIO 时钟使能 + 引脚配置 + 复位 + CFR3(PLL) + 辅助DAC */
void     AD9910_SW_Init(AD9910_SW_HandleTypeDef *hdev, const AD9910_SW_PinConfig *cfg);
void     AD9910_SW_Reset(AD9910_SW_HandleTypeDef *hdev);
void     AD9910_SW_IOUpdate(AD9910_SW_HandleTypeDef *hdev);

/* 底层写寄存器：1 字节地址 + len 字节数据（大端），CSN 内部分合 */
void     AD9910_SW_WriteReg(AD9910_SW_HandleTypeDef *hdev, uint8_t addr,
                            const uint8_t *data, uint8_t len);
void     AD9910_SW_Write32(AD9910_SW_HandleTypeDef *hdev, uint8_t addr, uint32_t val);
void     AD9910_SW_Write64(AD9910_SW_HandleTypeDef *hdev, uint8_t addr, uint64_t val);

/* 换算工具 */
uint32_t AD9910_SW_FreqToFTW(AD9910_SW_HandleTypeDef *hdev, float freq_hz);
uint16_t AD9910_SW_PhaseToPOW(float phase_deg);

/* 单频模式 */
void     AD9910_SW_SingleToneInit(AD9910_SW_HandleTypeDef *hdev);
void     AD9910_SW_SingleToneSet(AD9910_SW_HandleTypeDef *hdev, uint8_t profile,
                                 float freq_hz, uint16_t amp, float phase_deg);
void     AD9910_SW_SelectProfile(AD9910_SW_HandleTypeDef *hdev, uint8_t profile);
void     AD9910_SW_SetFTW(AD9910_SW_HandleTypeDef *hdev, float freq_hz);
/* POW 寄存器(0x08,16bit)：RAM 频率/幅度回放、DRG 扫频/扫幅时的相位偏移。
   RAM 幅度回放做基带波形(三角/方波)时设 FTW=0 + 相位90°，输出=包络 */
void     AD9910_SW_SetPhase(AD9910_SW_HandleTypeDef *hdev, float phase_deg);

/* ASF 寄存器(0x09)：仅在 OSK 使能时决定输出幅度（自动OSK=斜坡满度值，
   手动OSK=静态值）。amp = 0x0000~0x3FFF(14bit)，ramp_rate = 自动OSK斜坡速率ARR */
void     AD9910_SW_SetASF(AD9910_SW_HandleTypeDef *hdev, uint16_t amp, uint16_t ramp_rate);

/* OSK：内部默认写满幅 ASF(0x3FFF) + ARR(0x0100)，需调幅再调 SetASF；
   注意 OSK 引脚为低时输出强制为 0，须 OSKPin(1) 拉高 */
void     AD9910_SW_OSKInit(AD9910_SW_HandleTypeDef *hdev);
void     AD9910_SW_OSKPin(AD9910_SW_HandleTypeDef *hdev, uint8_t on);

/* 数字斜坡 DRG：扫频/扫相时输出恒为满幅（无幅度源，缩放器旁路，调幅只能用 OSK）；
   扫幅/扫相时频率来自单频 Profile 的 FTW，需先 SingleToneSet 设好 */
typedef enum { AD9910_DRG_FREQ = 0, AD9910_DRG_PHASE = 1, AD9910_DRG_AMP = 2 } AD9910_DRGDest;
void     AD9910_SW_DRGInit(AD9910_SW_HandleTypeDef *hdev, AD9910_DRGDest dest);
void     AD9910_SW_DRGConfigFreq(AD9910_SW_HandleTypeDef *hdev,
                                 float upper_hz, float lower_hz,
                                 float dec_step_hz, float inc_step_hz,
                                 uint16_t neg_rate, uint16_t pos_rate);
void     AD9910_SW_DRGDirection(AD9910_SW_HandleTypeDef *hdev, uint8_t up);
void     AD9910_SW_DRGHold(AD9910_SW_HandleTypeDef *hdev, uint8_t hold);

/* RAM 模式：RAMWrite 内部先失能 RAM 再写数据，但写入边界由 Profile 引脚选中的
   RAM Profile 决定——调用前必须先 RAMSetProfile + SelectProfile；
   频率/相位回放时 RAMEnable 内部配好静态满幅(手动OSK+ASF，可用 SetASF 调幅) */
void     AD9910_SW_RAMWrite(AD9910_SW_HandleTypeDef *hdev,
                            const uint32_t *words, uint16_t len);
void     AD9910_SW_RAMSetProfile(AD9910_SW_HandleTypeDef *hdev, uint8_t profile,
                                 uint16_t step_rate, uint16_t start_addr,
                                 uint16_t end_addr, uint8_t mode);
void     AD9910_SW_RAMEnable(AD9910_SW_HandleTypeDef *hdev, uint32_t cfr1_dest);
void     AD9910_SW_RAMDisable(AD9910_SW_HandleTypeDef *hdev);

/* 并行数据口（使能后交予 FPGA 驱动 D0~D15/F0/F1/TE） */
void     AD9910_SW_ParallelPortEnable(AD9910_SW_HandleTypeDef *hdev, uint8_t fm_gain);
void     AD9910_SW_ParallelPortDisable(AD9910_SW_HandleTypeDef *hdev);

#ifdef __cplusplus
}
#endif
#endif /* __AD9910_SW_H__ */
