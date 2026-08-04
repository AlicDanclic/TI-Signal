#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>

/* 触摸模块只向上层提供横屏逻辑坐标，底层寄存器和 I2C 时序对应用隐藏。 */

typedef enum
{
    TOUCH_CONTROLLER_NONE = 0,
    TOUCH_CONTROLLER_GT9147,
    TOUCH_CONTROLLER_FT5X06,
    TOUCH_CONTROLLER_OTT2001A
} TouchController;

/* 初始化 4.3 寸屏的电容触摸控制器，成功返回 1。 */
uint8_t Touch_Init(void);

/* 运行时快速重连；普通尝试不复位控制器，周期性失败后才执行一次硬复位。 */
uint8_t Touch_Reconnect(void);

/* 查询第一个触点；按下时返回 1，并输出 0~799、0~479 横屏坐标。 */
uint8_t Touch_Read(uint16_t *x, uint16_t *y);

TouchController Touch_GetController(void);

#endif
