#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct IMU_t {
	I2C_HandleTypeDef* hi2c;

	int16_t x_accel, y_accel, z_accel;
} IMU_t;

uint8_t imu_init(IMU_t*);

uint8_t imu_update(IMU_t*);

uint8_t imu_clear_int1(IMU_t*);




#endif
