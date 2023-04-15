#ifndef __XBEE_H
#define __XBEE_H

#include "stm32f4xx_hal.h"
#include "ff.h"

#define MIN_TRANSMIT_PERIOD 100
#define MAX_PATH_LENGTH 20

typedef enum {
	PrintMessage, BroadcastIdentity, RequestDevices, SendDevices, ReceiveFile, ImpactEvent
} XBee_Command;

typedef struct {
	XBee_Command command;
	uint32_t source;
	uint32_t target;
	uint8_t data[100];
} XBee_Data;

typedef struct {
	uint32_t uid;
	TCHAR file_path[MAX_PATH_LENGTH];
} Network_Device;


void XBee_Transmit(XBee_Data *data);
int XBee_Transmit_File_Start(const TCHAR *path);
void XBee_Transmit_File();
void XBee_Broadcast_Identity();
void XBee_Init();

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim11;

#define XBEE_UART &huart1
#define FILE_TIMER &htim11

#define MASTER_UID 1234567890

#endif
