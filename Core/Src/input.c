#include "input.h"
#include "headlamp.h"

void poll_button() {
	if (HAL_GPIO_ReadPin(BUTTON_GPIO_BANK, BUTTON_GPIO_PIN) == GPIO_PIN_SET) {
//		XBee_Data data;
//		data.command = ToggleHeadlamp;
//		data.target = 0;
//		XBee_Transmit(&data);
		toggle_headlamp();
		while (HAL_GPIO_ReadPin(BUTTON_GPIO_BANK, BUTTON_GPIO_PIN)); //BAD METHOD
	}
}
