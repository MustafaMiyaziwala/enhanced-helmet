#ifndef OV5462_H
#define OV5462_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define OV5462_I2C_ADDR_W 0x78
#define OV5462_I2C_ADDR_R 0x79

#define OV5462_CS_GPIO GPIOC
#define OV5462_CS_PIN GPIO_PIN_4

#define OV5462_ARDUCHIP_TIM 0x03
#define OV5462_VSYNC_LEVEL_MASK 0x02

#define CHIPID_UPPER 0x300A
#define CHIPID_LOWER 0x300B

#define ARDUCHIP_FIFO 0x04
#define FIFO_CLEAR_MASK 0x01
#define FIFO_START_MASK 0x02

#define ARDUCHIP_FRAMES 0x01

#define ARDUCHIP_TRIGGER 0x41
#define CAPTURE_DONE_MASK 0x08

#define FIFO_SIZE_LOWER 0x42
#define FIFO_SIZE_MIDDLE 0x43
#define FIFO_SIZE_UPPER 0x44 // only two bits

#define MAX_FIFO_LENGTH 0x5FFFF
#define BURST_FIFO_READ 0x3C


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

uint8_t OV5462_init(OV5462_t*);

uint8_t OV5462_write_i2c_reg(OV5462_t*, int addr, int data);
uint8_t OV5462_write_i2c_regs(OV5462_t*, const reg_value_pair regs[]);
uint8_t OV5462_read_i2c_reg(OV5462_t*, int addr);
void OV5462_write_spi_reg(OV5462_t* ov5462, uint8_t addr, uint8_t data);
uint8_t OV5462_read_spi_reg(OV5462_t* ov5462, uint8_t addr);
void OV5462_clear_fifo(OV5462_t*);
uint32_t OV5462_read_fifo_length(OV5462_t*);
void SPI_OptimizedReadByte(uint8_t* data);

#endif
