#ifndef OV5462_H
#define OV5462_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define OV5462_I2C_ADDR_W 0x78
#define OV5462_I2C_ADDR_R 0x79

#define OV5462_CS_GPIO GPIOF
#define OV5462_CS_PIN GPIO_PIN_9

#define OV5462_ARDUCHIP_TIM 0x03
#define OV5462_VSYNC_LEVEL_MASK 0x02

typedef struct OV5462_t {
	I2C_HandleTypeDef* hi2c;
	SPI_HandleTypeDef* hspi;
} OV5462_t;

typedef struct reg_value_pair {
	uint16_t addr;
	uint8_t value;
} reg_value_pair;

extern const reg_value_pair SET_RESOLUTION_320X240[];

extern const reg_value_pair SET_QVGA_MODE[];

extern const reg_value_pair CONFIGURE_JPEG_CAPTURE[];

void OV5462_init(OV5462_t*);

uint8_t OV5462_write_i2c_reg(OV5462_t*, int addr, int data);
uint8_t OV5462_write_i2c_regs(OV5462_t*, const reg_value_pair regs[]);
void OV5462_write_spi_reg(OV5462_t* ov5462, uint8_t addr, uint8_t data);
uint8_t OV5462_read_spi_reg(OV5462_t* ov5462, uint8_t addr);

#endif

