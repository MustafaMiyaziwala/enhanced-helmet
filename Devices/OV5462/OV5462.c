#include "OV5462.h"

uint8_t OV5462_write_reg_byte(OV5462_t* ov5462, int addr, int data) {
	uint8_t buf[4];
	buf[0] = (uint8_t)(addr >> 8); // upper addr byte
	buf[1] = (uint8_t)(addr & 0xFF); // lower addr byte
	buf[2] = data;

	ret = HAL_I2C_Master_Transmit(ov5462->hi2c, OV5462_I2C_ADDR_W, buf, 3, HAL_MAX_DELAY);

	if ( ret != HAL_OK ) {
		return 1;
	}

	return 0;
}

