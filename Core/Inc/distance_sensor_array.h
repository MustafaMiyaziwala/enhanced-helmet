#include "stm32f4xx_hal.h"


#define LEFT_SENSOR 2
#define CENTER_SENSOR 0
#define RIGHT_SENSOR 1

#define ROLLING_AVG_BUF_SIZE 5

typedef struct distance_sensor_array_t {
	ADC_HandleTypeDef* hadc;
	uint32_t readings[3];
} distance_sensor_array_t;

void update_readings_async(distance_sensor_array_t* ds) {
//	if (!(__HAL_ADC_GET_FLAG(ds->hadc, ADC_FLAG_EOC))) {
//		return;
//	}

	HAL_ADC_Start_DMA(ds->hadc, ds->readings, 3);
}


uint8_t get_motor_value(distance_sensor_array_t* ds, uint8_t direction) {
//	return  ds->readings[direction] > 300 ? 0 : ds->readings[direction] < 90
//			? 50 : (uint8_t)(50.0 / (ds->readings[direction] - 88));

	return (uint8_t)(ds->readings[direction] > 300 ? 0 : (300 - ds->readings[direction]) / 7.0);
}


