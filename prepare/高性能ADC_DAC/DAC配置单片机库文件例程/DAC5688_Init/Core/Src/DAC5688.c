#include "DAC5688.h"

uint8_t reg_config[] = {
		0x01, 0x0A, // 4x Interpolation
		0X02, 0X68, // Enable FIR4, Disable Mixer
		0x03, 0x00,
		0X05, 0XE2, // 4BIT Serial
		0X1D, 0X39, // M=8, N=2
		0X1E, 0X38, // 330MHz/V
		0x04, 0x00,
		0x16, 0xAA,
		0x05, 0xA2
};

uint8_t DAC5688_Init(DAC5688_t *dac, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t input_freq_MHz)
{
	dac->hspi = hspi;
	dac->cs_port = cs_port;
	dac->cs_pin = cs_pin;
	if (input_freq_MHz == 100)
	{
		reg_config[1] = 0x0B; // 8x Interpolation
		reg_config[9] = 0x38; // M=8, N=1
	}
	else if (input_freq_MHz == 200)
	{
		reg_config[1] = 0x0A; // 4x Interpolation
		reg_config[9] = 0x39; // M=8, N=2
	}
	else
		return -1;

	// Write initial configuration to DAC5688
	for (int i = 0; i < sizeof(reg_config); i += 2)
	{
		uint16_t send_data = (reg_config[i] << 8) | reg_config[i + 1], read_data;
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(dac->hspi, (const uint8_t *)(&send_data), (uint8_t *)(&read_data), 1, 0xFF);
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
	}
	// Verify configuration by reading back the registers
	for (int i = 0; i < sizeof(reg_config); i += 2)
	{
		uint16_t send_data = (reg_config[i] << 8) | 0x8000, read_data;
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(dac->hspi, (const uint8_t *)(&send_data), (uint8_t *)(&read_data), 1, 0xFF);
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
		// 0x05刷新用,无需比较
		if (reg_config[i] != 0x05 && (read_data & 0xFF) != reg_config[i + 1])
		{
			return -2; // Configuration verification failed
		}
	}
	return 0; // Initialization successful
}
uint8_t DAC5688_ReadStatus(DAC5688_t *dac)
{
	uint16_t send_data = 0x8000, read_data;
	HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(dac->hspi, (const uint8_t *)(&send_data), (uint8_t *)(&read_data), 1, 0xFF);
	HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
	return read_data & 0xFF;
}
void DAC5688_SetFMIX(DAC5688_t *dac, uint8_t enable, uint32_t fmix_freq_Hz)
{
	uint8_t reg_config_for_mixer[] = {
		0X02, 0X68 | (!!enable), 
		0X05, 0XE2,
		0x08, 0x00,
		0x09, 0x00,
		0x0A, 0x00,
		0x0B, 0x00,
		0x05, 0xA2};

	// f=800MHz, 32位频率字, 计算
	uint32_t freq_word = (uint32_t)(((uint64_t)fmix_freq_Hz << 32) / 800000000);
	reg_config_for_mixer[5] = freq_word & 0xFF;
	reg_config_for_mixer[7] = (freq_word >> 8) & 0xFF;
	reg_config_for_mixer[9] = (freq_word >> 16) & 0xFF;
	reg_config_for_mixer[11] = (freq_word >> 24) & 0xFF;

	for (int i = 0; i < sizeof(reg_config_for_mixer); i += 2)
	{
		uint16_t send_data = (reg_config_for_mixer[i] << 8) | reg_config_for_mixer[i + 1], read_data;
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(dac->hspi, (const uint8_t *)(&send_data), (uint8_t *)(&read_data), 1, 0xFF);
		HAL_GPIO_WritePin(dac->cs_port, dac->cs_pin, GPIO_PIN_SET);
		if (reg_config_for_mixer[i] == 0x05)
			HAL_Delay(100);
	}
}