/**
  * @file    ad9910_regs.h
  * @brief   AD9910 寄存器地址与位定义（软件/硬件 SPI 两个版本共用）
  * @details 所有位定义均已对照 AD9910 数据手册(Rev.B) Table 17~21 核实，
  *          RAM Profile 位域用凌智参考程序实例逐位反推验证。
  */

#ifndef __AD9910_REGS_H__
#define __AD9910_REGS_H__

#include <stdint.h>

/* ======================== 寄存器地址（Table 17） ======================== */
#define AD9910_REG_CFR1        0x00u   /* 控制功能寄存器 1 (32bit) */
#define AD9910_REG_CFR2        0x01u   /* 控制功能寄存器 2 (32bit) */
#define AD9910_REG_CFR3        0x02u   /* 控制功能寄存器 3 (32bit, 时钟/PLL) */
#define AD9910_REG_AUXDAC      0x03u   /* 辅助 DAC 控制 (32bit, FSC[7:0]) */
#define AD9910_REG_IOUPRATE    0x04u   /* I/O 更新速率 (32bit) */
#define AD9910_REG_FTW         0x07u   /* 频率调谐字 (32bit) */
#define AD9910_REG_POW         0x08u   /* 相位偏移字 (16bit) */
#define AD9910_REG_ASF         0x09u   /* 幅度控制 (32bit) */
#define AD9910_REG_DR_LIMIT    0x0Bu   /* 数字斜坡限值 (64bit) */
#define AD9910_REG_DR_STEP     0x0Cu   /* 数字斜坡步长 (64bit) */
#define AD9910_REG_DR_RATE     0x0Du   /* 数字斜坡速率 (32bit) */
#define AD9910_REG_PROFILE0    0x0Eu   /* 单频 Profile0 / RAM Profile0 (64bit) */
/* Profile1~7 地址依次 0x0F~0x15 */
#define AD9910_REG_RAM         0x16u   /* RAM 数据区 (1024 x 32bit) */

/* ======================== CFR1 位定义（Table 18） ======================== */
#define AD9910_CFR1_RAM_ENABLE        (1UL << 31)
#define AD9910_CFR1_RAM_DEST_FREQ     (0UL << 29)  /* RAM 回放目标: 频率 */
#define AD9910_CFR1_RAM_DEST_PHASE    (1UL << 29)  /* 相位 */
#define AD9910_CFR1_RAM_DEST_AMP      (2UL << 29)  /* 幅度 */
#define AD9910_CFR1_RAM_DEST_POLAR    (3UL << 29)  /* 极性(相位+幅度) */
#define AD9910_CFR1_MANUAL_OSK_EXT    (1UL << 23)
#define AD9910_CFR1_INVSINC_ENABLE    (1UL << 22)  /* 反 Sinc 滤波器使能 */
#define AD9910_CFR1_COSINE_OUTPUT     (1UL << 16)  /* 0=正弦(默认) 1=余弦 */
#define AD9910_CFR1_LOAD_LRR_IOUPD    (1UL << 15)
#define AD9910_CFR1_AUTOCLR_DRAMP     (1UL << 14)  /* 自动清除数字斜坡累加器 */
#define AD9910_CFR1_AUTOCLR_PHACC     (1UL << 13)  /* 自动清除相位累加器 */
#define AD9910_CFR1_CLR_DRAMP         (1UL << 12)
#define AD9910_CFR1_CLR_PHACC         (1UL << 11)
#define AD9910_CFR1_LOAD_ARR_IOUPD    (1UL << 10)
#define AD9910_CFR1_OSK_ENABLE        (1UL << 9)
#define AD9910_CFR1_AUTO_OSK          (1UL << 8)   /* 0=手动OSK 1=自动OSK */

/* ======================== CFR2 位定义（Table 19） ======================== */
/* CFR2[24]: 幅度缩放器采用有效单频 Profile 的 ASF 字段。
   仅在 DRG/RAM/OSK 都未使能时有效（即单频模式）；DRG/RAM/OSK 使能时该位无效。
   0 = 幅度缩放器旁路并关闭（默认），输出恒为满幅；
   1 = 输出幅度由有效 Profile 的 ASF 字段缩放（此时 SingleToneSet 的 amp 才生效） */
#define AD9910_CFR2_PROFILE_ASF_EN    (1UL << 24)
#define AD9910_CFR2_IOUP_RATE_ENABLE  (1UL << 23)  /* 内部 I/O 更新使能 */
#define AD9910_CFR2_SYNC_CLK_ENABLE   (1UL << 22)  /* SYNC_CLK 输出使能(默认) */
#define AD9910_CFR2_DRG_DEST_FREQ     (0UL << 20)  /* 数字斜坡目标: 频率 */
#define AD9910_CFR2_DRG_DEST_PHASE    (1UL << 20)  /* 相位 */
#define AD9910_CFR2_DRG_DEST_AMP      (2UL << 20)  /* 幅度 */
#define AD9910_CFR2_DRG_ENABLE        (1UL << 19)  /* 数字斜坡使能 */
#define AD9910_CFR2_DRG_NODWELL_HIGH  (1UL << 18)
#define AD9910_CFR2_DRG_NODWELL_LOW   (1UL << 17)
#define AD9910_CFR2_READ_EFF_FTW      (1UL << 16)
#define AD9910_CFR2_PDCLK_ENABLE      (1UL << 11)  /* PDCLK 输出使能(默认) */
#define AD9910_CFR2_PDCLK_INVERT      (1UL << 10)  /* PDCLK 反相 */
#define AD9910_CFR2_TXEN_INVERT       (1UL << 9)   /* TxENABLE 反相 */
#define AD9910_CFR2_MATCHED_LATENCY   (1UL << 7)
#define AD9910_CFR2_HOLD_LAST_VALUE   (1UL << 6)   /* 数据组装器保持最后值 */
#define AD9910_CFR2_SYNC_VAL_DISABLE  (1UL << 5)   /* 同步时序验证禁用(默认) */
#define AD9910_CFR2_PAR_PORT_ENABLE   (1UL << 4)   /* 并行数据口使能 */
/* CFR2[3:0] = FM 增益（并行口频率字左移位，默认 0） */

/* ======================== 并行口 F[1:0] 编码（Table 4） ================== */
#define AD9910_PAR_DEST_AMP      0u    /* D[15:2] -> 14bit 幅度 */
#define AD9910_PAR_DEST_PHASE    1u    /* D[15:0] -> 16bit 相位 */
#define AD9910_PAR_DEST_FREQ     2u    /* 两个 16bit 字 -> 32bit FTW（先发低16位） */
#define AD9910_PAR_DEST_POLAR    3u    /* D[15:8]=幅度高8位, D[7:0]=相位高8位 */

/* ======================== RAM 回放模式（Table 13） ====================== */
#define AD9910_RAM_MODE_DIRECT        0u   /* 直接转换 */
#define AD9910_RAM_MODE_RAMPUP        1u   /* 上斜坡 */
#define AD9910_RAM_MODE_BIDIR_RAMP    2u   /* 双向斜坡 */
#define AD9910_RAM_MODE_CONT_BIDIR    3u   /* 连续双向斜坡 */
#define AD9910_RAM_MODE_CONT_RECIRC   4u   /* 连续循环 */

/* ======================== ASF 寄存器位域（Table 25, 地址0x09） ============ */
/* [31:16]=幅度斜坡速率ARR  [15:2]=14bit幅度比例因子ASF  [1:0]=幅度步进(保留0)
   仅在 OSK 使能时参与输出幅度：自动OSK -> 斜坡至 ASF 满度值；手动OSK -> 静态 ASF 值。
   未使能 OSK 时幅度缩放器旁路（输出满幅），本寄存器无作用 */
static inline uint32_t AD9910_PackASF(uint16_t amp, uint16_t ramp_rate)
{
    return ((uint32_t)ramp_rate << 16) | ((uint32_t)(amp & 0x3FFFu) << 2);
}

/* ======================== 默认寄存器值 ==================================== */
/* CFR3: VCO SEL=101, REFCLK 分频器旁路, PLL 使能, N[7:1]=40
   -> 板载 25MHz 晶振 x40 = 1GHz
   注意：若把 N 改成 25(0x32) 则 SYSCLK=625MHz，sysclk_hz 必须跟着改 */
#define AD9910_CFR3_PLL_X40      0x050F4150UL
#define AD9910_AUXDAC_DEFAULT    0x0000007FUL
#define AD9910_CFR1_BASE         (AD9910_CFR1_INVSINC_ENABLE)                 /* 0x00400000 */
#define AD9910_CFR2_BASE         (AD9910_CFR2_SYNC_CLK_ENABLE | \
                                  AD9910_CFR2_PDCLK_ENABLE   | \
                                  AD9910_CFR2_SYNC_VAL_DISABLE)               /* 0x00400820 */

/**
  * @brief  打包 RAM Profile 的 64 位控制字
  *         [63:56]保留(0)  [55:40]地址步进速率M  [39:30]结束地址
  *         [23:14]起始地址  [2:0]回放模式
  * @note   位域已用凌智参考程序的 5 段实例逐位验证
  */
static inline uint64_t AD9910_PackRAMProfile(uint16_t step_rate, uint16_t start_addr,
                                             uint16_t end_addr, uint8_t mode)
{
    return ((uint64_t)step_rate << 40) |
           ((uint64_t)(end_addr   & 0x3FFu) << 30) |
           ((uint64_t)(start_addr & 0x3FFu) << 14) |
           (uint64_t)(mode & 0x7u);
}

#endif /* __AD9910_REGS_H__ */
