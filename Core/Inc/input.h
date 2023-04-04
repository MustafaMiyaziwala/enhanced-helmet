#ifndef __INPUT_H
#define __INPUT_H

#include "stm32f4xx_hal.h"
#include "xbee.h"

void poll_button();

#define BUTTON_GPIO_BANK GPIOC
#define BUTTON_GPIO_PIN GPIO_PIN_13

#endif
