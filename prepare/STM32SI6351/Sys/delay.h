#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx_hal.h"

/**
 * 初始化延时功能
 * 需要在使用延时函数前调用此函数
 */
void delay_init(void);

/**
 * 微秒级延时函数
 * @param us 需要延时的微秒数
 */
void delay_us(uint32_t us);

/**
 * 毫秒级延时函数
 * @param ms 需要延时的毫秒数
 */
void delay_ms(uint32_t ms);

/**
 * 秒级延时函数
 * @param s 需要延时的秒数
 */
void delay_s(uint32_t s);

#endif