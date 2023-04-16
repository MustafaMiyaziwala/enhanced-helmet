#ifndef __HAPTIC_PWM_H
#define __HAPTIC_PWM_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

/* GLOBAL DEFINES */
#define PWM_ON  0X1 /* duty cycle != 0% */
#define PWM_OFF 0X0 /* duty cycle == 0% */

/* Functions */

/* Sets PWM of specified timer
 * MODIFIED BY SET OFFSET (with an exception for zero)
 * RETURNS PWM_ON | PWM_OFF
 */
uint32_t PWM_SET_LEFT(uint8_t);
uint32_t PWM_SET_RIGHT(uint8_t);
uint32_t PWM_SET_CENTER(uint8_t);

/* Sets offset value
 * offset is base duty cycle regardless of input
 * setting
 */
void PWM_SET_OFFSET(const uint16_t);

/* returns uint8_t offset value */
uint8_t PWM_GET_OFFSET();

/* Ignore commands */
void PWM_SET_IGNORE();
void PWM_RESET_IGNORE();
void PWM_TOGGLE_IGNORE();

void PWM_ADD(uint32_t CHANNEL, int16_t val);

void PWM_PULSE_LEFT();
void PWM_PULSE_RIGHT();

void PWM_INIT();

extern TIM_HandleTypeDef htim5;

#define PULSE_TIMER &htim5
#define PULSE_VAL 6

#define CENTER_PWM TIM_CHANNEL_2
#define LEFT_PWM   TIM_CHANNEL_1
#define RIGHT_PWM  TIM_CHANNEL_3

#endif
