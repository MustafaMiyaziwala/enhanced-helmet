#ifndef __EXTDAC_H
#define __EXTDAC_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct Ext_DAC_t {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;
} Ext_DAC_t;


static void shutdown_dac(Ext_DAC_t*);

#endif
