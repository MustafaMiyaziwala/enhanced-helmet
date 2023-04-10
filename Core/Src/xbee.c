#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>

uint32_t UID;
XBee_Data XBee_Received;
uint32_t last_transmit;

void XBee_Transmit(XBee_Data *data) {
	uint32_t time = HAL_GetTick();
	if (time < last_transmit || time - last_transmit > MIN_TRANSMIT_PERIOD) {
		HAL_UART_Transmit_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
		last_transmit = time;
	}
}

void XBee_Receive(XBee_Data *data) {
	HAL_UART_Receive_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
}

void XBee_Resolve() {
	if (XBee_Received.target == 0 || XBee_Received.target == UID) {
		switch (XBee_Received.command) {
		case PrintMessage:
			printf("%s\n", (char *) XBee_Received.data);
			break;
		case ToggleHeadlamp:
			toggle_headlamp();
			break;
		case Placeholder:
			printf("Placeholder\n");
			break;
		default:
			printf("Unknown command received over network\n");
		}
	}
}

void XBee_Init() {
	UID = HAL_GetUIDw0() + HAL_GetUIDw1() + HAL_GetUIDw2();
	printf("UID: %u\n", (unsigned int) UID);
	XBee_Receive(&XBee_Received);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
		printf("XBee Error\n");
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart == XBEE_UART) {
//		printf("Transmitted data\n");
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
//		printf("Received - Type: %i, Target: %u, Data: [%u, %u, %u, ...]\n",
//				XBee_Received.command, (unsigned int) XBee_Received.target,
//				XBee_Received.data[0], XBee_Received.data[1],
//				XBee_Received.data[2]);
		XBee_Resolve();
	}
}
