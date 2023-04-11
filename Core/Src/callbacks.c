#include "xbee.h"
#include "input.h"
#include "headlamp.h"
#include <stdio.h>

extern int input_connected;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == HEADLAMP_TIMER) {
		HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port, HEADLAMP_OUT_Pin, GPIO_PIN_SET);
		HAL_TIM_Base_Stop_IT(HEADLAMP_TIMER);
	} else if (htim == FILE_TIMER) {
		XBee_Transmit_File();
		HAL_TIM_Base_Stop_IT(FILE_TIMER);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	if (pin & (1 << 8) && input_connected) {
		Input_Resolve();
	}
}
