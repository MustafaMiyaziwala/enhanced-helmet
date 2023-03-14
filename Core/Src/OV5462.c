#include "OV5462.h"

uint8_t OV5462_write_i2c_reg(OV5462_t* ov5462, int addr, int data) {
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

uint8_t OV5462_write_i2c_regs(OV5462_t* ov5462, const reg_value_pair regs[]) {
	const reg_value_pair* curr = regs;
	uint8_t resp;

	while (!(curr->addr == 0xFFFF && curr->value == 0xFF)) {
		resp = OV5462_write_i2c_reg(ov5462, curr->addr, curr->value);
		// do we need a delay here?
		++curr;
	}

	return resp;
}

void OV5462_write_spi_reg(OV5462_t* ov5462, uint8_t addr, uint8_t data) {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET); // chip select LOW

//	HAL_Delay(100);

	uint8_t buf[1] = { addr | 0x80 };
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);
	buf[0] = data;
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select HIGH

//	HAL_Delay(100);
}

uint8_t OV5462_read_spi_reg(OV5462_t* ov5462, uint8_t addr) {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET); // chip select LOW

//	HAL_Delay(100);

	uint8_t buf[1] = { addr };
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);
	HAL_SPI_Receive(ov5462->hspi, buf, 1, 100);

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select HIGH

//	HAL_Delay(100);

	return buf[0];
}


void OV5462_init(OV5462_t* ov5462) {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select is default HIGH

	OV5462_write_i2c_reg(ov5462, 0x3008, 0x80);
	OV5462_write_i2c_regs(ov5462, SET_QVGA_MODE); // determine if we need to do this for our application

	// configure camera for JPEG capture
	OV5462_write_i2c_regs(ov5462, CONFIGURE_JPEG_CAPTURE); // use JPEG capture mode
	OV5462_write_i2c_regs(ov5462, SET_RESOLUTION_320X240); // set sensor to low resolution
	OV5462_write_i2c_reg(ov5462, 0x3818, 0xa8);
	OV5462_write_i2c_reg(ov5462, 0x3621, 0x10);
	OV5462_write_i2c_reg(ov5462, 0x3801, 0xb0);
	OV5462_write_i2c_reg(ov5462, 0x4407, 0x04);

	OV5462_write_spi_reg(ov5462, OV5462_ARDUCHIP_TIM, OV5462_VSYNC_LEVEL_MASK);
}

