#include "ext_dac.h"

#define SHUTDOWN_CMD 0x2000

/*
 * Convert 8-bit dac levels to valid DAC commands
 */
static inline uint16_t val_to_dac(uint8_t val) {
	return (0b111 << 12) | val << 4;
}

void shutdown_dac(Ext_DAC_t* ext_dac) {
	uint16_t cmd = SHUTDOWN_CMD;
	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(ext_dac->hspi, (uint8_t*)&cmd, 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(ext_dac->hspi) != HAL_SPI_STATE_READY) {}

	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_SET);
}

void write_to_dac(Ext_DAC_t* ext_dac, uint16_t val) {

	//printf("%d\n\r", val);
	//uint16_t cmd = val_to_dac(val);
	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(ext_dac->hspi, (uint8_t*)&val, 1, HAL_MAX_DELAY);
	while (HAL_SPI_GetState(ext_dac->hspi) != HAL_SPI_STATE_READY) {}

	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_SET);

}








