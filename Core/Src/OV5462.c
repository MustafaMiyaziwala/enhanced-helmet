#include "OV5462.h"

uint8_t OV5462_write_reg_byte(OV5462_t* ov5462, int addr, int data) {
	uint8_t buf[4];
	buf[0] = (uint8_t)(addr >> 8); // upper addr byte
	buf[1] = (uint8_t)(addr & 0xFF); // lower addr byte
	buf[2] = data;

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(ov5462->hi2c, OV5462_I2C_ADDR_W, buf, 3, HAL_MAX_DELAY);

	if ( ret != HAL_OK ) {
		return 1;
	}

	return 0;
}

uint8_t OV5462_write_regs(OV5462_t* ov5462, const reg_value_pair regs[]) {
	const reg_value_pair* curr = regs;
	uint8_t resp;

	while (!(curr->addr == 0xFFFF && curr->value == 0xFF)) {
		resp = OV5462_write_reg_byte(ov5462, curr->addr, curr->value);
		// do we need a delay here?
		++curr;
	}

	return resp;
}

void OV5462_init(OV5462_t* ov5462) {
	OV5462_write_regs(ov5462, SET_RESOLUTION_320X240); // set sensor to low resolution
}

