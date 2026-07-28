#include "delay.h"

// DWT 计数器相关宏定义
#define DWT_CYCCNT   *(volatile uint32_t *)0xE0001004
#define DWT_CTRL     *(volatile uint32_t *)0xE0001000
#define DEM_CR       *(volatile uint32_t *)0xE000EDFC

/**
 * 初始化 DWT 计数器，使其能够用于延时功能
 * 启用 Cortex-M 的数据观察点和跟踪功能（DWT），并清空计数器
 */
void delay_init(void) {
    DEM_CR |= 1 << 24;       // 使能 TRC
    DWT_CTRL |= 1 << 0;      // 使能 CYCCNT
    DWT_CYCCNT = 0;          // 清空计数器
}

/**
 * 微秒级延时函数
 * 通过 DWT 计数器实现精确的微秒级延时
 * @param us 需要延时的微秒数
 */
void delay_us(uint32_t us) {
    uint32_t start = DWT_CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT_CYCCNT - start) < cycles);
}

/**
 * 毫秒级延时函数
 * 通过调用微秒级延时函数实现毫秒级延时
 * @param ms 需要延时的毫秒数
 */
void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}

/**
 * 秒级延时函数
 * 通过调用毫秒级延时函数实现秒级延时
 * @param s 需要延时的秒数
 */
void delay_s(uint32_t s) {
    while (s--) {
        delay_ms(1000);
    }
}