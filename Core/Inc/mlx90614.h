#ifndef __MLX90614_H
#define __MLX90614_H

#include <stdint.h>
#include "stm32l4xx_hal.h"



typedef struct MLX90614_t {
	I2C_HandleTypeDef* hi2c;
} MLX90614_t;



uint8_t MLX90614_init(MLX90614_t*);


/**
 * @brief 		read the ambient temperature
 * @param[in]	*handle to an MLX90614 struct
 * @param[out]	*temp_out points to double for temperature in F
 * @return 		status code:
 * 				- 0 error
 * 				- 1 success
 */
uint8_t MLX90614_read_ambient(MLX90614_t*, float* temp_out);

uint8_t MLX90614_read_object(MLX90614_t*, float* temp_out);

uint8_t MLX90614_read_emm(MLX90614_t*, uint16_t* emm);


#endif /* __MLX90614_H */
