/**
  * @file    si5351.c
  * @brief   Si5351A 时钟发生器 HAL 驱动实现（硬件 I2C）
  * @details 频率计算算法见 Skyworks AN619（Multisynth a+b/c 分数分频），
  *          动态频率部分参考成熟的 stm32-si5351 实现思路：
  *          - f < 1MHz 时先乘 64 再用 R 分频器除 64，减小误差；
  *          - f < 81MHz 时 PLL 固定 900MHz，调 MS 分频；
  *          - f >= 81MHz 时 MS 取 4/6/8 整数分频，调 PLL。
  */

#include "si5351.h"

/* ClockBuilder Pro 导出的寄存器配置表（配置变更时由官方软件重新生成，
   替换此头文件重新编译即可，驱动代码不用动。
   注意：该文件需加入工程头文件搜索路径，且只在 si5351.c 中包含一次） */
#include "Si5351A-RevB-Registers.h"

/* ======================== 底层读写 ======================== */

HAL_StatusTypeDef SI5351_WriteReg(SI5351_HandleTypeDef *hdev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, SI5351_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef SI5351_ReadReg(SI5351_HandleTypeDef *hdev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, SI5351_I2C_TIMEOUT_MS);
}

/* ======================== 初始化 ======================== */

HAL_StatusTypeDef SI5351_Init(SI5351_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    HAL_StatusTypeDef st;

    hdev->hi2c = hi2c;
    if (hdev->addr == 0u)     hdev->addr = SI5351_I2C_ADDR;
    if (hdev->xtal_hz == 0u)  hdev->xtal_hz = 25000000UL;
    if (hdev->xtal_load == 0u) hdev->xtal_load = SI5351_CRYSTAL_LOAD_10PF;

    /* 关闭所有输出（CLKx_OEB = 1 关闭） */
    st = SI5351_WriteReg(hdev, SI5351_REG_OUTPUT_ENABLE_CONTROL, 0xFF);
    if (st != HAL_OK) return st;

    /* 所有输出驱动掉电（CLKx_PDN = 1） */
    for (i = 0; i < 8; i++)
    {
        st = SI5351_WriteReg(hdev, SI5351_REG_CLK0_CONTROL + i, 0x80);
        if (st != HAL_OK) return st;
    }

    /* 晶振负载电容（寄存器 183，保留位必须置 0x12 中的 bit4/bit1） */
    return SI5351_WriteReg(hdev, SI5351_REG_XTAL_LOAD_CAP,
                           hdev->xtal_load | 0x12u);
}

/* ======================== ClockBuilder 寄存器表烧写 ======================== */

HAL_StatusTypeDef SI5351_LoadRegisters(SI5351_HandleTypeDef *hdev,
                                       const SI5351_RegWrite_t *list, uint16_t count)
{
    uint16_t i;
    HAL_StatusTypeDef st;

    for (i = 0; i < count; i++)
    {
        st = SI5351_WriteReg(hdev, (uint8_t)list[i].addr, list[i].value);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

HAL_StatusTypeDef SI5351_LoadClockBuilderConfig(SI5351_HandleTypeDef *hdev)
{
    uint16_t i;
    HAL_StatusTypeDef st;

    for (i = 0; i < SI5351A_REVB_REG_CONFIG_NUM_REGS; i++)
    {
        st = SI5351_WriteReg(hdev, (uint8_t)si5351a_revb_registers[i].address,
                             si5351a_revb_registers[i].value);
        if (st != HAL_OK) return st;
    }
    /* 导出表不含 PLL 复位和输出使能，补上后才真正输出 */
    return SI5351_PLLResetAndRun(hdev);
}

HAL_StatusTypeDef SI5351_PLLResetAndRun(SI5351_HandleTypeDef *hdev)
{
    HAL_StatusTypeDef st;

    st = SI5351_WriteReg(hdev, SI5351_REG_PLL_RESET, (1u << 7) | (1u << 5));
    if (st != HAL_OK) return st;
    return SI5351_WriteReg(hdev, SI5351_REG_OUTPUT_ENABLE_CONTROL, 0x00);
}

HAL_StatusTypeDef SI5351_EnableOutputs(SI5351_HandleTypeDef *hdev, uint8_t mask)
{
    return SI5351_WriteReg(hdev, SI5351_REG_OUTPUT_ENABLE_CONTROL, (uint8_t)~mask);
}

/* ======================== 内部：写 Multisynth 参数组 ======================== */

/** @brief 写一组 8 字节的 Multisynth 参数（PLL 或输出级共用格式，见 AN619 3.x/4.x） */
static HAL_StatusTypeDef SI5351_WriteMultisynth(SI5351_HandleTypeDef *hdev, uint8_t baseaddr,
                                                int32_t P1, int32_t P2, int32_t P3,
                                                uint8_t divBy4, SI5351_RDiv_t rdiv)
{
    uint8_t buf[8];
    uint8_t i;
    HAL_StatusTypeDef st;

    buf[0] = (uint8_t)((P3 >> 8) & 0xFF);
    buf[1] = (uint8_t)(P3 & 0xFF);
    buf[2] = (uint8_t)(((P1 >> 16) & 0x3) | ((divBy4 & 0x3) << 2) | ((rdiv & 0x7) << 4));
    buf[3] = (uint8_t)((P1 >> 8) & 0xFF);
    buf[4] = (uint8_t)(P1 & 0xFF);
    buf[5] = (uint8_t)(((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF));
    buf[6] = (uint8_t)((P2 >> 8) & 0xFF);
    buf[7] = (uint8_t)(P2 & 0xFF);

    for (i = 0; i < 8; i++)
    {
        st = SI5351_WriteReg(hdev, baseaddr + i, buf[i]);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

/* ======================== 动态频率配置 ======================== */

HAL_StatusTypeDef SI5351_SetupPLL(SI5351_HandleTypeDef *hdev, SI5351_PLL_t pll,
                                  const SI5351_PLLConfig_t *conf)
{
    int32_t P1, P2, P3;
    uint8_t baseaddr;
    HAL_StatusTypeDef st;

    P1 = 128 * conf->mult + (128 * conf->num) / conf->denom - 512;
    P2 = (128 * conf->num) % conf->denom;
    P3 = conf->denom;

    baseaddr = (pll == SI5351_PLL_A) ? SI5351_REG_MSNA_BASE : SI5351_REG_MSNB_BASE;
    st = SI5351_WriteMultisynth(hdev, baseaddr, P1, P2, P3, 0, SI5351_R_DIV_1);
    if (st != HAL_OK) return st;

    /* 改 PLL 参数后必须复位两个 PLL 才会生效 */
    return SI5351_WriteReg(hdev, SI5351_REG_PLL_RESET, (1u << 7) | (1u << 5));
}

HAL_StatusTypeDef SI5351_SetupOutput(SI5351_HandleTypeDef *hdev, uint8_t clk,
                                     SI5351_PLL_t pll_src, SI5351_Drive_t drive,
                                     const SI5351_OutputConfig_t *conf, uint8_t phase_offset)
{
    int32_t P1, P2, P3;
    uint8_t divBy4 = 0;
    uint8_t baseaddr, clkCtrlReg, phoffReg, clkCtrl;
    HAL_StatusTypeDef st;

    if (clk > 2u) return HAL_ERROR;

    if ((!conf->allowIntegerMode) && ((conf->div < 8) || ((conf->div == 8) && (conf->num == 0))))
        return HAL_ERROR;   /* div ∈ {4,6,8} 仅整数模式可用 */

    if (conf->div == 4)
    {
        /* DIVBY4 特例，见 AN619 4.1.3 */
        P1 = 0; P2 = 0; P3 = 1;
        divBy4 = 0x3;
    }
    else
    {
        P1 = 128 * conf->div + (128 * conf->num) / conf->denom - 512;
        P2 = (128 * conf->num) % conf->denom;
        P3 = conf->denom;
    }

    baseaddr  = SI5351_REG_MS0_BASE + clk * 8;      /* MS0=42, MS1=50, MS2=58 */
    clkCtrlReg = SI5351_REG_CLK0_CONTROL + clk;
    phoffReg   = SI5351_REG_CLK0_PHOFF + clk;

    clkCtrl = 0x0C | drive;                          /* 不反相、上电、源=Multisynth */
    if (pll_src == SI5351_PLL_B) clkCtrl |= (1u << 5);
    if (conf->allowIntegerMode && ((conf->num == 0) || (conf->div == 4)))
        clkCtrl |= (1u << 6);                        /* 整数模式 */

    st = SI5351_WriteReg(hdev, clkCtrlReg, clkCtrl);
    if (st != HAL_OK) return st;
    st = SI5351_WriteMultisynth(hdev, baseaddr, P1, P2, P3, divBy4, conf->rdiv);
    if (st != HAL_OK) return st;
    return SI5351_WriteReg(hdev, phoffReg, phase_offset & 0x7F);
}

/**
  * @brief 计算 PLL 与 MS 参数：f = Fxtal × (a+b/c) / ((x+y/z) × R)
  * @note  支持 8kHz ~ 200MHz，误差 < 6Hz（correction 准确时）；
  *        > 160MHz 时 MS=4 整数分频（DIVBY4 特例），PLL 最高 800MHz
  */
void SI5351_Calc(SI5351_HandleTypeDef *hdev, int32_t freq_hz,
                 SI5351_PLLConfig_t *pll_conf, SI5351_OutputConfig_t *out_conf)
{
    int32_t a, b, c, x, y, z, t;

    if (freq_hz < 8000)        freq_hz = 8000;
    else if (freq_hz > 200000000) freq_hz = 200000000;

    out_conf->allowIntegerMode = 1;

    if (freq_hz < 1000000)
    {
        /* 低于 1MHz：先算 64 倍频，再用 R=64 除回来，减小误差 */
        freq_hz *= 64;
        out_conf->rdiv = SI5351_R_DIV_64;
    }
    else
    {
        out_conf->rdiv = SI5351_R_DIV_1;
    }

    /* 晶振频偏校准（在确定 rdiv 之后应用） */
    freq_hz -= (int32_t)(((double)freq_hz / 100000000.0) * (double)hdev->correction);

    if (freq_hz < 81000000)
    {
        /* PLL 固定 900MHz，调输出 MS */
        a = 36; b = 0; c = 1;
        x = 900000000 / freq_hz;
        t = (freq_hz >> 20) + 1;
        y = (900000000 % freq_hz) / t;
        z = freq_hz / t;
    }
    else
    {
        /* MS 固定整数 4/6/8，调 PLL */
        if      (freq_hz >= 150000000) x = 4;
        else if (freq_hz >= 100000000) x = 6;
        else                           x = 8;
        y = 0; z = 1;

        {
            int32_t numerator = x * freq_hz;
            a = numerator / (int32_t)hdev->xtal_hz;
            t = (int32_t)((hdev->xtal_hz >> 20) + 1);
            b = (numerator % (int32_t)hdev->xtal_hz) / t;
            c = (int32_t)hdev->xtal_hz / t;
        }
    }

    pll_conf->mult  = a;  pll_conf->num   = b;  pll_conf->denom = c;
    out_conf->div   = x;  out_conf->num   = y;  out_conf->denom = z;
}

HAL_StatusTypeDef SI5351_SetupCLK0(SI5351_HandleTypeDef *hdev, int32_t freq_hz, SI5351_Drive_t drive)
{
    SI5351_PLLConfig_t    pll_conf;
    SI5351_OutputConfig_t out_conf;
    HAL_StatusTypeDef st;

    SI5351_Calc(hdev, freq_hz, &pll_conf, &out_conf);
    st = SI5351_SetupPLL(hdev, SI5351_PLL_A, &pll_conf);
    if (st != HAL_OK) return st;
    return SI5351_SetupOutput(hdev, 0, SI5351_PLL_A, drive, &out_conf, 0);
}

HAL_StatusTypeDef SI5351_SetupCLK2(SI5351_HandleTypeDef *hdev, int32_t freq_hz, SI5351_Drive_t drive)
{
    SI5351_PLLConfig_t    pll_conf;
    SI5351_OutputConfig_t out_conf;
    HAL_StatusTypeDef st;

    SI5351_Calc(hdev, freq_hz, &pll_conf, &out_conf);
    st = SI5351_SetupPLL(hdev, SI5351_PLL_B, &pll_conf);
    if (st != HAL_OK) return st;
    return SI5351_SetupOutput(hdev, 2, SI5351_PLL_B, drive, &out_conf, 0);
}
