#ifndef __HEADLAMP_H
#define __HEADLAMP_H

#include "stm32f4xx_hal.h"

#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))

void Headlamp_Init();
void toggle_headlamp();

extern TIM_HandleTypeDef htim10;

#define HEADLAMP_TIMER &htim10

#endif
