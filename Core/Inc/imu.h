#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct IMU {
	I2C_HandleTypeDef* hi2c;

	int16_t x_accel, y_accel, z_accel;
} IMU;

uint8_t imu_init(IMU*);

uint8_t imu_update(IMU*);

uint8_t imu_clear_int1(IMU*);



#endif
