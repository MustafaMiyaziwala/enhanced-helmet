#include "ext_dac.h"

#define SHUTDOWN_CMD 0x2000

/*
 * Convert 8-bit dac levels to valid DAC commands
 */
static inline uint16_t val_to_dac(uint8_t val) {
	return (0b111 << 12) | (uint16_t)(val) << 4;
}

static inline void transmit_cmd(Ext_DAC_t* ext_dac, uint16_t cmd) {
	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(ext_dac->hspi, (uint8_t*)&cmd, 1, 1);
	while (HAL_SPI_GetState(ext_dac->hspi) != HAL_SPI_STATE_READY) {}

	HAL_GPIO_WritePin(ext_dac->cs_port, ext_dac->cs_pin, GPIO_PIN_SET);
}

void shutdown_dac(Ext_DAC_t* ext_dac) {
	transmit_cmd(ext_dac, SHUTDOWN_CMD);
}

void write_to_dac(Ext_DAC_t* ext_dac, uint8_t val) {
	transmit_cmd(ext_dac, val_to_dac(val));
}
