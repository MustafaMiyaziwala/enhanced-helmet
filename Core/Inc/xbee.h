#ifndef __XBEE_H
#define __XBEE_H

#include "stm32f4xx_hal.h"

typedef enum {
	PrintMessage, ToggleHeadlamp, Placeholder
} XBee_Command;

typedef volatile struct {
	XBee_Command command;
	uint32_t target;
	uint8_t data[100];
} XBee_Data;

void XBee_Transmit(XBee_Data *data);
void XBee_Receive(XBee_Data *data);
void XBee_Resolve();
void XBee_Init();

extern UART_HandleTypeDef huart1;

#define XBEE_UART &huart1

#endif
