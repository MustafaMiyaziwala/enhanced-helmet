#ifndef __BUTTONS_H
#define __BUTTONS_H

#include "stm32f4xx_hal.h"

#define NUM_BTNS 	4
#define BTN_INT		9

#define HELP_BTN 	0
#define RECORD_BTN 	1
#define HAPTICS_BTN 2
#define LIGHT_BTN	3
#define NO_BTN 		4

typedef struct Buttons {
	I2C_HandleTypeDef* hi2c;

	uint8_t btns_state[NUM_BTNS];
	uint8_t status;
} Buttons;

void buttons_init(Buttons*);

uint8_t get_released_button(Buttons*);

#endif
