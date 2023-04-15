#include "headlamp.h"
#include <stdio.h>
#include "main.h"

int headlamp_on = 0;

void Headlamp_Init() {
	HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port, HEADLAMP_OUT_Pin, GPIO_PIN_SET);
}

void toggle_headlamp() {
	if (HAL_TIM_Base_GetState(HEADLAMP_TIMER) == HAL_TIM_STATE_READY) {
		HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port, HEADLAMP_OUT_Pin,
				GPIO_PIN_RESET);
		if (headlamp_on) {
			__HAL_TIM_SET_AUTORELOAD(HEADLAMP_TIMER, 10000);
			printf("Light off\n");
			headlamp_on = 0;
		} else {
			__HAL_TIM_SET_AUTORELOAD(HEADLAMP_TIMER, 2500);
			printf("Light on\n");
			headlamp_on = 1;
		}
		HAL_TIM_Base_Start_IT(HEADLAMP_TIMER);
	}
}
