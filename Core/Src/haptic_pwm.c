#include "haptic_pwm.h"

#define FALSE  0 /* LOGICAL FALSE*/
#define	TRUE   1 /* LOGICAL TRUE */

#define CENTER_PWM TIM_CHANNEL_1 /* NOTE these may change */
#define LEFT_PWM   TIM_CHANNEL_2 /* fix these and it should work */
#define RIGHT_PWM  TIM_CHANNEL_3

extern TIM_HandleTypeDef htim3;

/* FLAG */
/* Decides whether to ignore input */
static uint8_t IGNORE = 0;

/* Y Intercept of linear */
static uint16_t offset = 0; /* Offset to change base duty cycle of PWM */

static inline uint32_t PWM_SET(uint32_t CHANNEL, uint8_t val) {
	if (!IGNORE && val) {
		uint16_t compareVal = (val << 8) | offset;
		__HAL_TIM_SetCompare(&htim3, CHANNEL, compareVal);
		return PWM_ON;
	}
	else {
		__HAL_TIM_SetCompare(&htim3, CHANNEL, 0);
		return PWM_OFF;
	}
}

/* PWM_SET definitions, call PWM_Set with correct channel */
uint32_t PWM_SET_LEFT(uint8_t val) {
	return PWM_SET(LEFT_PWM, val);
}
uint32_t PWM_SET_RIGHT(uint8_t val) {
	return PWM_SET(RIGHT_PWM, val);
}
uint32_t PWM_SET_CENTER(uint8_t val) {
	return PWM_SET(CENTER_PWM, val);
}

/* offset function set and get */
void PWM_SET_OFFSET(const uint16_t val) {
	offset = val;
}
uint8_t PWM_GET_OFFSET() {
	return offset;
}

/* PWM commands */
void PWM_SET_IGNORE() {
	IGNORE = TRUE;
	PWM_SET(LEFT_PWM, 0);
	PWM_SET(RIGHT_PWM, 0);
	PWM_SET(CENTER_PWM, 0);
}
void PWM_RESET_IGNORE() {
	IGNORE = FALSE;
}
void PWM_TOGGLE_IGNORE() {
	IGNORE = IGNORE ? TRUE : FALSE;
}
