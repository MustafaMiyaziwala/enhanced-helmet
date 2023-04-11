#include "stm32f4xx_hal.h"

#define LEFT_SENSOR 0
#define CENTER_SENSOR 1
#define RIGHT_SENSOR 2

typedef struct distance_sensor_array_t {
	ADC_HandleTypeDef* hadc;
	uint32_t readings[3];
} distance_sensor_array_t;

void update_readings_async(distance_sensor_array_t* ds) {
	HAL_ADC_Start_DMA(ds->hadc, ds->readings, 3);
}

