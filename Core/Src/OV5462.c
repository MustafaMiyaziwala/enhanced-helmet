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
	HAL_StatusTypeDef ret;

	while (!(curr->addr == 0xFFFF && curr->value == 0xFF)) {
		ret = OV5462_write_i2c_reg(ov5462, curr->addr, curr->value);
		// do we need a delay here?
		HAL_Delay(1);
		++curr;

		if ( ret != HAL_OK ) {
			return 1;
		}
	}

	return 0;
}

uint8_t OV5462_read_i2c_reg(OV5462_t* ov5462, int addr) {
	uint8_t buf[2];
	buf[0] = (uint8_t)(addr >> 8);
	buf[1] = (uint8_t)(addr & 0xFF);

	HAL_I2C_Master_Transmit(ov5462->hi2c, OV5462_I2C_ADDR_R, buf, 2, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(ov5462->hi2c, OV5462_I2C_ADDR_R, buf, 1, HAL_MAX_DELAY);

	return buf[0];
}

void OV5462_write_spi_reg(OV5462_t* ov5462, uint8_t addr, uint8_t data) {
	DISABLE_NONZERO_IRQ();
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET); // chip select LOW

//	HAL_Delay(100);

	uint8_t buf[1] = { addr | 0x80 };
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);
	buf[0] = data;
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select HIGH
	ENABLE_ALL_IRQ();
//	HAL_Delay(100);
}

uint8_t OV5462_read_spi_reg(OV5462_t* ov5462, uint8_t addr) {
	DISABLE_NONZERO_IRQ();
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET); // chip select LOW

//	HAL_Delay(100);

	uint8_t buf[1] = { addr };
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);
	HAL_SPI_Receive(ov5462->hspi, buf, 1, 100);

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select HIGH

//	HAL_Delay(100);
	ENABLE_ALL_IRQ();
	return buf[0];
}


uint8_t OV5462_init(OV5462_t* ov5462) {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET); // chip select is default HIGH

	OV5462_write_i2c_reg(ov5462, 0x3008, 0x80);
	OV5462_write_i2c_regs(ov5462, SET_QVGA_MODE); // determine if we need to do this for our application

	uint8_t err = 0;
	// configure camera for JPEG capture
	err |= OV5462_write_i2c_regs(ov5462, CONFIGURE_JPEG_CAPTURE); // use JPEG capture mode
	err |=OV5462_write_i2c_regs(ov5462, SET_RESOLUTION_320X240); // set sensor to low resolution
	err |=OV5462_write_i2c_reg(ov5462, 0x3818, 0xa8);
	err |=OV5462_write_i2c_reg(ov5462, 0x3621, 0x10);
	err |=OV5462_write_i2c_reg(ov5462, 0x3801, 0xb0);
	err |=OV5462_write_i2c_reg(ov5462, 0x4407, 0x04); // 04?

	OV5462_write_spi_reg(ov5462, OV5462_ARDUCHIP_TIM, OV5462_VSYNC_LEVEL_MASK);
	OV5462_write_spi_reg(ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

	return err;
}

void OV5462_continuous_capture_init(OV5462_t* ov5462) {
	uint8_t camera_version = OV5462_read_spi_reg(ov5462, 0x40);
	uint8_t frames;

	// set continuous capture (depends on version)
	if (camera_version && 0x70) {
		frames = 0xFF;
	} else {
		frames = 0x07;
	}

	OV5462_write_spi_reg(ov5462, ARDUCHIP_FRAMES, frames);
}

uint32_t OV5462_read_fifo_length(OV5462_t* ov5462) {
	uint32_t lower, middle, upper;
	lower = OV5462_read_spi_reg(ov5462, FIFO_SIZE_LOWER);
	middle = OV5462_read_spi_reg(ov5462, FIFO_SIZE_MIDDLE);
	upper = OV5462_read_spi_reg(ov5462, FIFO_SIZE_UPPER);

	return ((upper << 16) | (middle << 8) | lower) & 0x07fffff;
}

void OV5462_request_FIFO_burst(OV5462_t* ov5462) {
	uint8_t buf[1] = { BURST_FIFO_READ };
	HAL_SPI_Transmit(ov5462->hspi, buf, 1, 100);
}

uint8_t SPI_OptimizedReadByte(uint8_t* data) {
	while (((SPI1->SR)&(1>>7))) {}; // wait for BSY bit to reset
	SPI1->DR = 0; // dummy byte
	while (!((SPI1->SR) & (1<<0))) {};
	return SPI1->DR;
}

void OV5462_CS_High() {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
}
void OV5462_CS_Low() {
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
}

void OV5462_trigger_capture(OV5462_t* ov) {
	OV5462_write_spi_reg(ov, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // clear flag
	OV5462_write_spi_reg(ov, ARDUCHIP_FIFO, FIFO_RESET_WRITE);
	OV5462_write_spi_reg(ov, ARDUCHIP_FIFO, FIFO_RESET_READ);
	OV5462_write_spi_reg(ov, ARDUCHIP_FIFO, FIFO_START_MASK); // start capture
}
