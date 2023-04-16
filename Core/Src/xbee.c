#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>
#include <string.h>

extern FATFS fs;
extern FIL fil;
extern uint32_t total_space, free_space;
extern int num_registered_devices;

extern XBee_Data xbee_packet;
extern uint32_t devices[MAX_DEVICES];


extern uint32_t victim_uid;
extern uint8_t alert_flag;

extern uint32_t UID;
XBee_Data XBee_Received;
uint32_t last_transmit = 0;

volatile int transmitting_file;
volatile int receiving_file;
volatile int receiving_devices;
int is_receive_target = 0;

uint8_t *file_buf;
FSIZE_t fsize;
FSIZE_t rsize;
TCHAR rpath[MAX_PATH_LENGTH];

void XBee_Transmit(XBee_Data *data) {
	data->source = UID;
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
			printf("Failed to open file for transmission (%i)\r\n", ret);
			return;
		}
		fsize = f_size(&fil);
		file_buf = (uint8_t *) malloc(fsize * sizeof(uint8_t));
		UINT bytes_read;
		ret = f_read(&fil, file_buf, fsize, &bytes_read);
		if(ret != FR_OK) {
			printf("Failed to read file for transmission (%i)\r\n", ret);
			return;
		}
		ret = f_close(&fil);
		if(ret != FR_OK) {
			printf("Failed to close file for transmission (%i)\r\n", ret);
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
		printf("Receiving file\r\n");
		file_buf = (uint8_t *) malloc(rsize);
		HAL_UART_Receive_DMA(XBEE_UART, file_buf, rsize);
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
		XBee_Receive_File();
	} else if (is_receive_target) {
		switch (XBee_Received.command) {
			case PrintMessage:
				printf("%s\r\n", (char *) XBee_Received.data);
				break;
			case ImpactEvent:
				victim_uid = XBee_Received.source;
				alert_flag = IMPACT_ALERT;
				printf("Impact detected on device UID %u\r\n", (unsigned int) XBee_Received.source);
				break;
			case HelpEvent:
				victim_uid = XBee_Received.source;
				alert_flag = HELP_ALERT;
				printf("Help requested by device UID %u\r\n", (unsigned int) XBee_Received.source);
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
				devices[num_registered_devices] = *((uint32_t *) XBee_Received.source);
				printf("Registered new device with UID %u\r\n", (unsigned int) XBee_Received.source);
				num_registered_devices++;
			done:
				break;
			case SendDevices:
				num_registered_devices = *((int *) XBee_Received.data);
				for (int i = 0; i < num_registered_devices; i++) {
					devices[i] = *((uint32_t *) &XBee_Received.data[sizeof(int) + i * sizeof(uint32_t)]);
					printf("Registered device %u\r\n", (unsigned int) devices[i]);
				}
				receiving_devices = 0;
				printf("Received device list\r\n");
				break;
			default:
				printf("Incorrect command sent to helmet\r\n");
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
			printf("Failed to open file for reception (%i)\r\n", ret);
			goto fail;
		}
		UINT bytes_written;
		ret = f_write(&fil, file_buf, rsize, &bytes_written);
		if(ret != FR_OK) {
			printf("Failed to write file for reception (%i)\r\n", ret);
			goto fail;
		}
		ret = f_close(&fil);
		if(ret != FR_OK) {
			printf("Failed to close file for reception (%i)\r\n", ret);
			goto fail;
		}
		printf("Received file\r\n");
	}
fail:
	free(file_buf);
	receiving_file = 0;
	XBee_Receive(&XBee_Received);
}

void XBee_Handshake() {
	printf("Requesting devices\r\n");
	xbee_packet.command = RequestDevices;
	xbee_packet.target = MASTER_UID;
	XBee_Transmit(&xbee_packet);
	receiving_devices = 1;
	while (receiving_devices);
	HAL_Delay(MIN_TRANSMIT_PERIOD);
	printf("Broadcasting identity\r\n");
	xbee_packet.command = Register;
	xbee_packet.target = 0;
	XBee_Transmit(&xbee_packet);
	HAL_Delay(500);
	printf("Broadcasting file\r\n");
	const TCHAR path[MAX_PATH_LENGTH];
	sprintf((char *) path, "/audio/%u.wav", (unsigned int) UID);
	XBee_Transmit_File_Start(path, 0);
	while (transmitting_file);
}

void XBee_Init() {
	UID = HAL_GetUIDw0() + HAL_GetUIDw1() + HAL_GetUIDw2();
	printf("UID: %u\r\n", (unsigned int) UID);
	__HAL_TIM_SET_AUTORELOAD(FILE_TIMER, 5000);
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
			FIX_TIMER_TRIGGER(FILE_TIMER);
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
