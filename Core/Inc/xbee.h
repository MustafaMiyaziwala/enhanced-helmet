#ifndef __XBEE_H
#define __XBEE_H

#include "stm32f4xx_hal.h"
#include "ff.h"

#define MAX_DEVICES 4
#define MIN_TRANSMIT_PERIOD 100
#define MAX_PATH_LENGTH 50

#define MY_FILE_PATH "/AUDIO/JOSEPH_MAFFETONE.WAV"

typedef enum {
	PrintMessage, ReceiveFile, // universal
	Register, RequestDevices, ImpactEventAnnounce, HelpEventAnnounce, // only handled by base station (only sent by helmets)
	RequestAudio, ReceiveDevices, ImpactEventRelay, HelpEventRelay // only handled by helmets (only sent by base station)
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
void XBee_Handshake();

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim11;

#define XBEE_UART &huart1
#define FILE_TIMER &htim11

#define MASTER_UID 1234567890

#endif
