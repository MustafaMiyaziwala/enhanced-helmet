#ifndef __HEADLAMP_H
#define __HEADLAMP_H

#include "stm32f4xx_hal.h"

void Headlamp_Init();
void toggle_headlamp();

extern TIM_HandleTypeDef htim10;

#define HEADLAMP_TIMER &htim10

#endif
