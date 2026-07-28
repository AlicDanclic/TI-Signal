#ifndef DAC5688_H
#define DAC5688_H

#include "main.h"

// DAC5688寄存器配置结构体
typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} DAC5688_t;

/**
 * @brief 初始化DAC5688
 * @param dac DAC5688实例
 * @param hspi SPI句柄
 * @param cs_port CS GPIO端口
 * @param cs_pin CS GPIO引脚
 * @param input_freq FPGA输入频率, 单位MHz (100MHz/200MHz)
 * @return 0: 初始化成功, -1: 不支持的输入频率, -2: 配置验证失败
 */
uint8_t DAC5688_Init(DAC5688_t *dac, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t input_freq);

/**
 * @brief 读取DAC5688状态寄存器
 * @param dac DAC5688实例
 * @return 状态寄存器值
 */
uint8_t DAC5688_ReadStatus(DAC5688_t *dac);

/**
 * @brief 设置FMIX功能
 * @param dac DAC5688实例
 * @param enable 使能标志
 * @param fmix_freq FMIX频率, 单位Hz
 */
void DAC5688_SetFMIX(DAC5688_t *dac, uint8_t enable, uint32_t fmix_freq);

#endif /* DAC5688_H */