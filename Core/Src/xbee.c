#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>
#include <string.h>

extern FATFS fs;
extern FIL fil;
extern uint32_t total_space, free_space;

extern uint32_t devices[MAX_DEVICES];
extern int num_registered_devices;

extern XBee_Data xbee_packet;

uint32_t UID;
extern XBee_Data XBee_Received;
uint32_t last_transmit = 0;
extern uint8_t tr;
extern uint32_t from_UID;

volatile int transmitting_file = 0;
volatile int receiving_devices = 0;
int receiving_file = 0;
int is_receive_target = 0;

uint8_t *file_buf;
FSIZE_t fsize;
FSIZE_t rsize;
TCHAR rpath[MAX_PATH_LENGTH];
FRESULT fr;
extern uint bw;
extern uint br;

void XBee_Transmit(XBee_Data *data) {
	uint32_t time = HAL_GetTick();
	if (last_transmit == 0 || time < last_transmit || time - last_transmit > MIN_TRANSMIT_PERIOD) {
		HAL_UART_Transmit_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
		last_transmit = time;
	}
}

void XBee_Transmit_File_Start(const TCHAR* path, uint32_t target) {
	printf("Preparing to transmit file\r\n");
	if (transmitting_file == 0) {
		int ret = 0;
		ret = f_open(&fil, path, FA_READ);
		if(ret != FR_OK) {
			printf("Failed to open file (%i)\r\n", ret);
			return;
		}
		fsize = f_size(&fil);
		file_buf = (uint8_t *) malloc(fsize * sizeof(uint8_t));
		ret = f_read(&fil, file_buf, fsize, &br);
		if(ret != FR_OK) {
			printf("Failed to read file (%i)\r\n", ret);
			return;
		}
		ret = f_close(&fil);
		if(ret != FR_OK) {
			printf("Failed to close file (%i)\r\n", ret);
			return;
		}
		xbee_packet.command = ReceiveFile;
		xbee_packet.target = target;
		*((FSIZE_t *) xbee_packet.data) = fsize;
		strcpy((char *) &xbee_packet.data[sizeof(FSIZE_t)], path);
		XBee_Transmit(&xbee_packet);
		transmitting_file = 1;
	} else {
		printf("Already transmitting file\r\n");
	}
}

void XBee_Transmit_File() {
	printf("Transmitting file\r\n");
	HAL_UART_Transmit_DMA(XBEE_UART, file_buf, fsize);
	transmitting_file = 2;
}

void XBee_Receive(XBee_Data *data) {
	HAL_UART_Receive_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
}

void XBee_Receive_File() {
	if (!receiving_file) {
		printf("Receiving file");
		if (!is_receive_target) {
			printf(" but not target of file transfer");
		}
		printf("\r\n");
		file_buf = (uint8_t *) malloc(rsize);
		HAL_UART_Receive_DMA(XBEE_UART, file_buf, rsize);
		__HAL_TIM_SET_AUTORELOAD(FILE_TIMER, FILE_TIMEOUT * 2);
		tr = 0;
		HAL_TIM_Base_Start_IT(FILE_TIMER);
		receiving_file = 1;
	} else {
		printf("Already receiving file\r\n");
	}
}

void XBee_Resolve() {
	is_receive_target = XBee_Received.target == 0 || XBee_Received.target == UID;
	if (XBee_Received.command == ReceiveFile) {
		printf("Preparing to receive file\r\n");
		rsize = *((FSIZE_t *) XBee_Received.data);
		strcpy(rpath, (TCHAR *) &XBee_Received.data[sizeof(FSIZE_t)]);
		from_UID = XBee_Received.source;
		XBee_Receive_File();
	} else if (is_receive_target) {
		switch (XBee_Received.command) {
			case PrintMessage:
				printf("%s\r\n", (char *) XBee_Received.data);
				break;
			case ImpactEvent:
				printf("Impact detected on device %u\r\n", (unsigned int) XBee_Received.source);
				break;
			case HelpEvent:
				printf("Help requested by device %u\r\n", (unsigned int) XBee_Received.source);
				break;
			case Register:
				if (num_registered_devices >= MAX_DEVICES) {
					printf("Maximum registered network devices reached\r\n");
					break;
				}
				for (int i = 0; i < num_registered_devices; i++) {
					if (devices[i] == XBee_Received.source) {
						printf("Already registered device %u\r\n", (unsigned int) XBee_Received.source);
						goto done;
					}
				}
				devices[num_registered_devices] = XBee_Received.source;
				printf("Registered new device with UID %u\r\n", (unsigned int) devices[num_registered_devices]);
				num_registered_devices++;
			done:
				break;
			case RequestDevices:
				printf("Sending device files\r\n");
				const TCHAR path[MAX_PATH_LENGTH];
				for (int i = 0; i < num_registered_devices; i++) {
					if (devices[i] != XBee_Received.source) {
						sprintf((char *) path, "/audio/%u.wav", (unsigned int) devices[i]);
						XBee_Transmit_File_Start(path, XBee_Received.source);
						while (transmitting_file);
						HAL_Delay(MIN_TRANSMIT_PERIOD);
					}
				}
				printf("Sending device list\r\n");
				xbee_packet.command = SendDevices;
				xbee_packet.source = UID;
				xbee_packet.target = XBee_Received.source;
				int found = 0;
				for (int i = 0; i < num_registered_devices; i++) {
					if (devices[i] == XBee_Received.source) {
						found = 1;
					} else {
						*((uint32_t *) &xbee_packet.data[sizeof(int) + i * sizeof(uint32_t)] + found) = devices[i];
					}
				}
				*((int *) xbee_packet.data) = num_registered_devices - found;
				XBee_Transmit(&xbee_packet);
				break;
			default:
				printf("Incorrect command sent to base station\r\n");
		}
	}
	if (!receiving_file) {
		XBee_Receive(&XBee_Received);
	}
}

void XBee_Resolve_File() {
	int ret = 0;
	if (!is_receive_target) {
		printf("Received file but not target of file transfer\r\n");
	} else {
		ret = f_open(&fil, rpath, FA_OPEN_ALWAYS | FA_WRITE);
		if(ret != FR_OK) {
			printf("Failed to open file (%i)\r\n", ret);
		}
		ret = f_write(&fil, file_buf, rsize, &bw);
		if(ret != FR_OK) {
			printf("Failed to write file (%i)\r\n", ret);
		}
		ret = f_close(&fil);
		if(ret != FR_OK) {
			printf("Failed to close file (%i)\r\n", ret);
		}
		printf("Received file\r\n");
	}
	free(file_buf);
	receiving_file = 0;
	XBee_Receive(&XBee_Received);
}

void XBee_Handshake() {
	printf("Requesting devices\r\n");
	xbee_packet.command = RequestDevices;
	xbee_packet.source = UID;
	xbee_packet.target = MASTER_UID;
	XBee_Transmit(&xbee_packet);
	receiving_devices = 1;
	while (receiving_devices);
	printf("Broadcasting identity\r\n");
	xbee_packet.command = Register;
	xbee_packet.target = 0;
	*((uint32_t *) xbee_packet.data) = UID;
	XBee_Transmit(&xbee_packet);
//	const TCHAR path[MAX_PATH_LENGTH];
//	strcpy((char *) path, (char *) &xbee_packet.data[sizeof(uint32_t)]);
//	HAL_Delay(500);
//	printf("Transmitting file\r\n");
//	XBee_Transmit_File_Start(path, 0);
//	while (transmitting_file);
}

void XBee_Init() {
	UID = HAL_GetUIDw0() + HAL_GetUIDw1() + HAL_GetUIDw2();
	printf("UID: %u\r\n", (unsigned int) UID);
	XBee_Receive(&XBee_Received);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
		printf("XBee Error\r\n");
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
		if (transmitting_file == 1) {
			while (HAL_TIM_Base_GetState(FILE_TIMER) != HAL_TIM_STATE_READY);
			__HAL_TIM_SET_AUTORELOAD(FILE_TIMER, 1000);
			tr = 0;
			HAL_TIM_Base_Start_IT(FILE_TIMER);
		} else if (transmitting_file == 2) {
			printf("Transmitted file\r\n");
			free(file_buf);
			transmitting_file = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
//		printf("Received - Type: %i, Target: %u, Data: [%u, %u, %u, ...]\r\n",
//				XBee_Received.command, (unsigned int) XBee_Received.target,
//				XBee_Received.data[0], XBee_Received.data[1],
//				XBee_Received.data[2]);
		if (!receiving_file) {
			XBee_Resolve();
		} else {
			XBee_Resolve_File();
		}
	}
}
