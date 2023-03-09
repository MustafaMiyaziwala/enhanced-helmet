#include "stm32l4xx_hal.h"
#include <stdint.h>

#define OV5462_I2C_ADDR_W 0x78
#define OV5462_I2C_ADDR_R 0x79

typedef struct OV5462_t {
	I2C_HandleTypeDef* hi2c;
} OV5462_t;

void OV5462_init(OV5462_t*);

uint8_t OV5462_write_reg_byte(OV5462_t*, int addr, int data);
