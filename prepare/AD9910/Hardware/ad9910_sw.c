/**
  * @file    ad9910_sw.c
  * @brief   AD9910 DDS 驱动实现 (软件模拟 SPI · 结构体配置版)
  * @details 用普通 GPIO 模拟 SPI 模式0 时序：SCK 空闲低，SCK 上升沿前
  *          把数据放到 SDI 上，AD9910 在上升沿采样，MSB 先发。
  *          每写完一组寄存器后必须给 IUP 一个上升沿才会生效。
  */

#include "ad9910_sw.h"

/* ======================== 静态辅助函数 ======================== */

static inline void PIN_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t state)
{
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief 位 bang 延时，保证 SCK 高低电平有足够宽度 */
static void SW_Delay(void)
{
    for (volatile uint32_t i = 0; i < AD9910_SW_DELAY_LOOP; i++) { __NOP(); }
}

/** @brief 按端口使能 GPIO 时钟（F1/F4 通用，未定义的端口自动跳过） */
static void GPIO_ClockEnable(GPIO_TypeDef *port)
{
    if (port == NULL) return;
#ifdef GPIOA
    if (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); return; }
#endif
#ifdef GPIOB
    if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); return; }
#endif
#ifdef GPIOC
    if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); return; }
#endif
#ifdef GPIOD
    if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); return; }
#endif
#ifdef GPIOE
    if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); return; }
#endif
#ifdef GPIOF
    if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); return; }
#endif
#ifdef GPIOG
    if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); return; }
#endif
#ifdef GPIOH
    if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); return; }
#endif
}

/** @brief 软件模拟 SPI 写入 1 字节，MSB 先发（SPI 模式0 时序） */
static void AD9910_SW_Write8(AD9910_SW_HandleTypeDef *hdev, uint8_t data)
{
    AD9910_SW_PinConfig *p = &hdev->pin;
    uint8_t mask;

    PIN_Write(p->sck_port, p->sck_pin, 0);
    for (mask = 0x80u; mask != 0u; mask >>= 1)
    {
        PIN_Write(p->sdi_port, p->sdi_pin, (data & mask) ? 1 : 0);
        SW_Delay();
        PIN_Write(p->sck_port, p->sck_pin, 1);   /* AD9910 在上升沿采样 SDI */
        SW_Delay();
        PIN_Write(p->sck_port, p->sck_pin, 0);
    }
}

/* ======================== 基础操作 ======================== */

/** @brief MRT 主复位，高有效 */
void AD9910_SW_Reset(AD9910_SW_HandleTypeDef *hdev)
{
    PIN_Write(hdev->pin.rst_port, hdev->pin.rst_pin, 1);
    HAL_Delay(1);
    PIN_Write(hdev->pin.rst_port, hdev->pin.rst_pin, 0);
    HAL_Delay(1);
}

/** @brief I/O_UPDATE：上升沿把串行缓冲搬运到有效寄存器 */
void AD9910_SW_IOUpdate(AD9910_SW_HandleTypeDef *hdev)
{
    PIN_Write(hdev->pin.iup_port, hdev->pin.iup_pin, 0);
    SW_Delay();
    PIN_Write(hdev->pin.iup_port, hdev->pin.iup_pin, 1);
    SW_Delay();
    PIN_Write(hdev->pin.iup_port, hdev->pin.iup_pin, 0);
}

void AD9910_SW_WriteReg(AD9910_SW_HandleTypeDef *hdev, uint8_t addr,
                        const uint8_t *data, uint8_t len)
{
    AD9910_SW_PinConfig *p = &hdev->pin;
    uint8_t i;

    PIN_Write(p->cs_port, p->cs_pin, 0);
    AD9910_SW_Write8(hdev, addr);
    for (i = 0; i < len; i++)
        AD9910_SW_Write8(hdev, data[i]);
    PIN_Write(p->cs_port, p->cs_pin, 1);
}

void AD9910_SW_Write32(AD9910_SW_HandleTypeDef *hdev, uint8_t addr, uint32_t val)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val);
    AD9910_SW_WriteReg(hdev, addr, buf, 4);
}

void AD9910_SW_Write64(AD9910_SW_HandleTypeDef *hdev, uint8_t addr, uint64_t val)
{
    uint8_t buf[8];
    uint8_t i;
    for (i = 0; i < 8; i++)
        buf[i] = (uint8_t)(val >> (56 - 8 * i));
    AD9910_SW_WriteReg(hdev, addr, buf, 8);
}

/* ======================== 初始化 ======================== */

/**
  * @brief 初始化：GPIO 时钟使能 -> 引脚全部配为推挽输出 -> 复位
  *        -> CFR3(PLL 40 倍频) -> 辅助DAC -> I/O_UPDATE
  */
void AD9910_SW_Init(AD9910_SW_HandleTypeDef *hdev, const AD9910_SW_PinConfig *cfg)
{
    GPIO_InitTypeDef    gpio = {0};
    AD9910_SW_PinConfig *p   = &hdev->pin;

    *p = *cfg;

    /* GPIO 时钟 */
    GPIO_ClockEnable(p->sck_port); GPIO_ClockEnable(p->sdi_port);
    GPIO_ClockEnable(p->cs_port);  GPIO_ClockEnable(p->rst_port);
    GPIO_ClockEnable(p->iup_port);
    GPIO_ClockEnable(p->pf0_port); GPIO_ClockEnable(p->pf1_port);
    GPIO_ClockEnable(p->pf2_port); GPIO_ClockEnable(p->osk_port);
    GPIO_ClockEnable(p->drc_port); GPIO_ClockEnable(p->drh_port);

    /* 全部配置为推挽输出 */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    if (p->sck_port) { gpio.Pin = p->sck_pin; HAL_GPIO_Init(p->sck_port, &gpio); }
    if (p->sdi_port) { gpio.Pin = p->sdi_pin; HAL_GPIO_Init(p->sdi_port, &gpio); }
    if (p->cs_port)  { gpio.Pin = p->cs_pin;  HAL_GPIO_Init(p->cs_port,  &gpio); }
    if (p->rst_port) { gpio.Pin = p->rst_pin; HAL_GPIO_Init(p->rst_port, &gpio); }
    if (p->iup_port) { gpio.Pin = p->iup_pin; HAL_GPIO_Init(p->iup_port, &gpio); }
    if (p->pf0_port) { gpio.Pin = p->pf0_pin; HAL_GPIO_Init(p->pf0_port, &gpio); }
    if (p->pf1_port) { gpio.Pin = p->pf1_pin; HAL_GPIO_Init(p->pf1_port, &gpio); }
    if (p->pf2_port) { gpio.Pin = p->pf2_pin; HAL_GPIO_Init(p->pf2_port, &gpio); }
    if (p->osk_port) { gpio.Pin = p->osk_pin; HAL_GPIO_Init(p->osk_port, &gpio); }
    if (p->drc_port) { gpio.Pin = p->drc_pin; HAL_GPIO_Init(p->drc_port, &gpio); }
    if (p->drh_port) { gpio.Pin = p->drh_pin; HAL_GPIO_Init(p->drh_port, &gpio); }

    /* 初始电平 */
    PIN_Write(p->sck_port, p->sck_pin, 0);
    PIN_Write(p->sdi_port, p->sdi_pin, 0);
    PIN_Write(p->cs_port,  p->cs_pin,  1);
    PIN_Write(p->rst_port, p->rst_pin, 0);
    PIN_Write(p->iup_port, p->iup_pin, 0);
    PIN_Write(p->pf0_port, p->pf0_pin, 0);
    PIN_Write(p->pf1_port, p->pf1_pin, 0);
    PIN_Write(p->pf2_port, p->pf2_pin, 0);
    PIN_Write(p->osk_port, p->osk_pin, 0);
    PIN_Write(p->drc_port, p->drc_pin, 0);
    PIN_Write(p->drh_port, p->drh_pin, 0);

    /* 复位 -> CFR3(PLL x40) -> 辅助DAC -> I/O_UPDATE */
    AD9910_SW_Reset(hdev);
    AD9910_SW_Write32(hdev, AD9910_REG_CFR3,   AD9910_CFR3_PLL_X40);
    AD9910_SW_Write32(hdev, AD9910_REG_AUXDAC, AD9910_AUXDAC_DEFAULT);
    AD9910_SW_IOUpdate(hdev);
    HAL_Delay(1);   /* 等待 PLL 锁定 */
}

/* ======================== 换算工具 ======================== */

/** @brief FTW = round(f * 2^32 / fSYSCLK)，64 位运算避免 float 精度损失 */
uint32_t AD9910_SW_FreqToFTW(AD9910_SW_HandleTypeDef *hdev, float freq_hz)
{
    double ftw = (double)freq_hz * 4294967296.0 / (double)hdev->sysclk_hz;
    return (uint32_t)(ftw + 0.5);
}

/** @brief POW = round(phase_deg * 2^16 / 360) */
uint16_t AD9910_SW_PhaseToPOW(float phase_deg)
{
    double pow_ = (double)phase_deg * 65536.0 / 360.0;
    return (uint16_t)((uint32_t)(pow_ + 0.5) & 0xFFFFu);
}

/* ======================== 单频模式 ======================== */

void AD9910_SW_SingleToneInit(AD9910_SW_HandleTypeDef *hdev)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR1, AD9910_CFR1_BASE);
    /* CFR2[24]=1：单频模式下输出幅度由 Profile 的 ASF 字段缩放，
       否则缩放器旁路、恒满幅（SingleToneSet 的 amp 参数无效） */
    AD9910_SW_Write32(hdev, AD9910_REG_CFR2,
                      AD9910_CFR2_BASE | AD9910_CFR2_PROFILE_ASF_EN);
    AD9910_SW_IOUpdate(hdev);
}

/**
  * @brief 设置某个单频 Profile 的频率/幅度/相位
  * @param profile   0~7
  * @param freq_hz   1Hz ~ 约 450MHz
  * @param amp       0x0000~0x3FFF (14bit)
  * @param phase_deg 0~360 度
  * @note  Profile 字(64bit) = ASF[63:50] | POW[47:32] | FTW[31:0]
  */
void AD9910_SW_SingleToneSet(AD9910_SW_HandleTypeDef *hdev, uint8_t profile,
                             float freq_hz, uint16_t amp, float phase_deg)
{
    uint64_t word;

    if (profile > 7u) return;

    word = ((uint64_t)(amp & 0x3FFFu) << 48) |
           ((uint64_t)AD9910_SW_PhaseToPOW(phase_deg) << 32) |
           AD9910_SW_FreqToFTW(hdev, freq_hz);

    AD9910_SW_Write64(hdev, AD9910_REG_PROFILE0 + profile, word);
    AD9910_SW_IOUpdate(hdev);
}

/** @brief 通过 PF2~PF0 引脚选择当前生效的 Profile */
void AD9910_SW_SelectProfile(AD9910_SW_HandleTypeDef *hdev, uint8_t profile)
{
    AD9910_SW_PinConfig *p = &hdev->pin;

    if (p->pf0_port == NULL || p->pf1_port == NULL || p->pf2_port == NULL) return;

    PIN_Write(p->pf0_port, p->pf0_pin, (profile & 0x01u) ? 1 : 0);
    PIN_Write(p->pf1_port, p->pf1_pin, (profile & 0x02u) ? 1 : 0);
    PIN_Write(p->pf2_port, p->pf2_pin, (profile & 0x04u) ? 1 : 0);
    AD9910_SW_IOUpdate(hdev);
}

/** @brief 直接写 FTW 寄存器（RAM 幅度/相位回放时作为载波频率） */
void AD9910_SW_SetFTW(AD9910_SW_HandleTypeDef *hdev, float freq_hz)
{
    AD9910_SW_Write32(hdev, AD9910_REG_FTW, AD9910_SW_FreqToFTW(hdev, freq_hz));
    AD9910_SW_IOUpdate(hdev);
}

/** @brief 直接写 POW 寄存器(0x08, 16bit)（RAM/DRG 模式下的相位偏移） */
void AD9910_SW_SetPhase(AD9910_SW_HandleTypeDef *hdev, float phase_deg)
{
    uint16_t pow_ = AD9910_SW_PhaseToPOW(phase_deg);
    uint8_t  buf[2] = { (uint8_t)(pow_ >> 8), (uint8_t)(pow_) };
    AD9910_SW_WriteReg(hdev, AD9910_REG_POW, buf, 2);
    AD9910_SW_IOUpdate(hdev);
}

/* ======================== OSK ======================== */

/**
  * @brief 写 ASF 寄存器(0x09)：[31:16]=ARR  [15:2]=14bit幅度  [1:0]=0
  * @note  仅在 OSK 使能时影响输出幅度（自动OSK=斜坡满度值，手动OSK=静态值）；
  *        ARR 决定自动 OSK 斜坡速度：每步 Δt = 4×ARR/fSYSCLK，步进 1LSB，
  *        从 0 升到满幅 0x3FFF 共 16383 步（ARR=0x0100 时约 17ms）。
  */
void AD9910_SW_SetASF(AD9910_SW_HandleTypeDef *hdev, uint16_t amp, uint16_t ramp_rate)
{
    AD9910_SW_Write32(hdev, AD9910_REG_ASF, AD9910_PackASF(amp, ramp_rate));
    AD9910_SW_IOUpdate(hdev);
}

/**
  * @brief 自动 OSK：CFR1[9]=1 使能 OSK, CFR1[8]=1 选择自动 OSK
  * @note  OSK 使能后幅度完全由 OSK 功能接管（优先级最高，Profile ASF 无效），
  *        故此处默认写满幅 ASF(0x3FFF) + 斜坡速率 ARR(0x0100)，调幅用 SetASF。
  *        注意：OSK 引脚为低时输出强制为 0，必须用 OSKPin(1) 拉高才有输出。
  */
void AD9910_SW_OSKInit(AD9910_SW_HandleTypeDef *hdev)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR1,
                      AD9910_CFR1_BASE | AD9910_CFR1_OSK_ENABLE | AD9910_CFR1_AUTO_OSK);
    AD9910_SW_Write32(hdev, AD9910_REG_CFR2, AD9910_CFR2_BASE);
    AD9910_SW_Write32(hdev, AD9910_REG_ASF,  AD9910_PackASF(0x3FFFu, 0x0100u));
    AD9910_SW_IOUpdate(hdev);
}

void AD9910_SW_OSKPin(AD9910_SW_HandleTypeDef *hdev, uint8_t on)
{
    if (hdev->pin.osk_port == NULL) return;
    PIN_Write(hdev->pin.osk_port, hdev->pin.osk_pin, on);
}

/* ======================== 数字斜坡 DRG ======================== */

/**
  * @brief DRG 初始化：使能数字斜坡并选择斜坡目标
  * @note  幅度来源（手册 Table 5）：扫频/扫相时无幅度源，缩放器旁路，
  *        输出恒为满幅（要调幅只能用 OSK + ASF 寄存器）；
  *        扫幅/扫相时频率来自单频 Profile 的 FTW，需先 SingleToneSet 设好。
  */
void AD9910_SW_DRGInit(AD9910_SW_HandleTypeDef *hdev, AD9910_DRGDest dest)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR1, AD9910_CFR1_BASE);
    AD9910_SW_Write32(hdev, AD9910_REG_CFR2,
                      AD9910_CFR2_BASE | AD9910_CFR2_DRG_ENABLE | ((uint32_t)dest << 20));
    AD9910_SW_IOUpdate(hdev);
}

/**
  * @brief 频率数字斜坡（线性扫频）
  * @param upper_hz/lower_hz  上/下限频率（上限 > 下限）
  * @param dec_step_hz        下行步进
  * @param inc_step_hz        上行步进
  * @param neg_rate/pos_rate  下/上行速率计时字 M：每 M*4ns(1GHz 时) 走一个步进
  */
void AD9910_SW_DRGConfigFreq(AD9910_SW_HandleTypeDef *hdev,
                             float upper_hz, float lower_hz,
                             float dec_step_hz, float inc_step_hz,
                             uint16_t neg_rate, uint16_t pos_rate)
{
    /* 斜坡限值: [63:32]=上限 [31:0]=下限 */
    uint64_t limit = ((uint64_t)AD9910_SW_FreqToFTW(hdev, upper_hz) << 32) |
                     AD9910_SW_FreqToFTW(hdev, lower_hz);
    /* 斜坡步长: [63:32]=下行步进 [31:0]=上行步进 */
    uint64_t step  = ((uint64_t)AD9910_SW_FreqToFTW(hdev, dec_step_hz) << 32) |
                     AD9910_SW_FreqToFTW(hdev, inc_step_hz);
    /* 斜坡速率: [31:16]=下行速率 [15:0]=上行速率 */
    uint32_t rate  = ((uint32_t)neg_rate << 16) | pos_rate;

    AD9910_SW_Write64(hdev, AD9910_REG_DR_LIMIT, limit);
    AD9910_SW_Write64(hdev, AD9910_REG_DR_STEP,  step);
    AD9910_SW_Write32(hdev, AD9910_REG_DR_RATE,  rate);
    AD9910_SW_IOUpdate(hdev);
}

void AD9910_SW_DRGDirection(AD9910_SW_HandleTypeDef *hdev, uint8_t up)
{
    if (hdev->pin.drc_port == NULL) return;
    /* DRCTL = 0 向上斜坡, 1 向下斜坡 */
    PIN_Write(hdev->pin.drc_port, hdev->pin.drc_pin, up ? 0 : 1);
}

void AD9910_SW_DRGHold(AD9910_SW_HandleTypeDef *hdev, uint8_t hold)
{
    if (hdev->pin.drh_port == NULL) return;
    PIN_Write(hdev->pin.drh_port, hdev->pin.drh_pin, hold);
}

/* ======================== RAM 模式 ======================== */

/**
  * @brief 写 RAM（1024 x 32bit）
  * @note  - 写 RAM 前必须 CFR1[31]=0（RAM 失能），本函数内部已处理；
  *        - 【重要】写入的地址范围由“当前 Profile 引脚选中的 RAM Profile”的
  *          起止地址决定（手册 RAM Control 三步流程），因此调用本函数前必须
  *          先 RAMSetProfile 设好边界、再 SelectProfile 选中该 profile；
  *        - 串行口先写的字进入 RAM 高地址端，数组按“末地址 -> 首地址”传入；
  *        - 数据格式由回放目标决定（手册 Table 12）：
  *            频率: word = FTW (32bit)
  *            相位: word = POW << 16
  *            幅度: word = ASF << 18 (14bit)
  */
void AD9910_SW_RAMWrite(AD9910_SW_HandleTypeDef *hdev,
                        const uint32_t *words, uint16_t len)
{
    uint16_t i, j;
    uint8_t  b;

    if (len == 0u || len > 1024u) return;

    /* 写 RAM 前必须失能 RAM（CFR1[31]=0），否则数据写不进去 */
    AD9910_SW_Write32(hdev, AD9910_REG_CFR1, AD9910_CFR1_BASE);
    AD9910_SW_IOUpdate(hdev);

    PIN_Write(hdev->pin.cs_port, hdev->pin.cs_pin, 0);
    AD9910_SW_Write8(hdev, AD9910_REG_RAM);
    for (i = 0; i < len; i++)
    {
        for (j = 0; j < 4; j++)
        {
            b = (uint8_t)(words[i] >> (24 - 8 * j));
            AD9910_SW_Write8(hdev, b);
        }
    }
    PIN_Write(hdev->pin.cs_port, hdev->pin.cs_pin, 1);
}

void AD9910_SW_RAMSetProfile(AD9910_SW_HandleTypeDef *hdev, uint8_t profile,
                             uint16_t step_rate, uint16_t start_addr,
                             uint16_t end_addr, uint8_t mode)
{
    if (profile > 7u) return;
    AD9910_SW_Write64(hdev, AD9910_REG_PROFILE0 + profile,
                      AD9910_PackRAMProfile(step_rate, start_addr, end_addr, mode));
    AD9910_SW_IOUpdate(hdev);
}

/**
  * @brief 使能 RAM 回放
  * @param cfr1_dest  AD9910_CFR1_RAM_DEST_FREQ / _PHASE / _AMP / _POLAR
  * @note  - 使能后 Profile0~7 寄存器变为 RAM Profile，用 PF2~PF0 选择回放段；
  *        - 目标为幅度/极性时幅度直接来自 RAM 数据，不开 OSK；
  *        - 目标为频率/相位时，默认无幅度源（缩放器旁路，满幅输出）；
  *          为了能调幅，此处使能手动 OSK 且 CFR1[23]=0（忽略 OSK 引脚），
  *          幅度 = ASF 寄存器静态值，先写满幅 0x3FFF，调幅用 SetASF；
  *        - 载波频率（相位/幅度回放时）用 AD9910_SW_SetFTW 设置。
  */
void AD9910_SW_RAMEnable(AD9910_SW_HandleTypeDef *hdev, uint32_t cfr1_dest)
{
    uint32_t dest = cfr1_dest & 0x60000000UL;
    uint32_t cfr1 = AD9910_CFR1_BASE | AD9910_CFR1_RAM_ENABLE | dest;

    if (dest == AD9910_CFR1_RAM_DEST_FREQ || dest == AD9910_CFR1_RAM_DEST_PHASE)
    {
        /* 手动 OSK(CFR1[8]=0) + CFR1[23]=0 -> OSK 引脚无效，幅度=ASF 静态值 */
        cfr1 |= AD9910_CFR1_OSK_ENABLE;
        AD9910_SW_Write32(hdev, AD9910_REG_ASF, AD9910_PackASF(0x3FFFu, 0u));
    }

    AD9910_SW_Write32(hdev, AD9910_REG_CFR1, cfr1);
    AD9910_SW_IOUpdate(hdev);
}

void AD9910_SW_RAMDisable(AD9910_SW_HandleTypeDef *hdev)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR1, AD9910_CFR1_BASE);
    AD9910_SW_IOUpdate(hdev);
}

/* ======================== 并行数据口 ======================== */

/**
  * @brief 使能并行数据口（之后 D0~D15/F0/F1/TE 由 FPGA 驱动）
  * @param fm_gain  FM 增益 0~15：频率目标下 FTW = 端口字 << fm_gain
  */
void AD9910_SW_ParallelPortEnable(AD9910_SW_HandleTypeDef *hdev, uint8_t fm_gain)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR2,
                      AD9910_CFR2_BASE | AD9910_CFR2_PAR_PORT_ENABLE | (fm_gain & 0xFu));
    AD9910_SW_IOUpdate(hdev);
}

void AD9910_SW_ParallelPortDisable(AD9910_SW_HandleTypeDef *hdev)
{
    AD9910_SW_Write32(hdev, AD9910_REG_CFR2, AD9910_CFR2_BASE);
    AD9910_SW_IOUpdate(hdev);
}
