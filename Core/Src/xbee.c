#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>
#include <string.h>

extern FATFS fs;
extern FIL fil;
extern uint32_t total_space, free_space;
extern Network_Device devices_removed[MAX_DEVICES];
extern int num_registered_devices;

extern XBee_Data xbee_packet;

uint32_t UID;
XBee_Data XBee_Received;
uint32_t last_transmit = 0;

volatile int transmitting_file;
int receiving_file;
uint8_t *file_buf;
FSIZE_t fsize;
FSIZE_t rsize;
TCHAR rpath[MAX_PATH_LENGTH];

void XBee_Transmit(XBee_Data *data) {
	uint32_t time = HAL_GetTick();
	if (last_transmit == 0 || time < last_transmit || time - last_transmit > MIN_TRANSMIT_PERIOD) {
		HAL_UART_Transmit_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
		last_transmit = time;
	}
}

int XBee_Transmit_File_Start(const TCHAR* path) {
	printf("Preparing to transmit file\n");
	if (transmitting_file == 0) {
		int ret = 0;
		ret = f_open(&fil, path, FA_OPEN_ALWAYS | FA_READ);
		if(ret != FR_OK) {
			printf("Failed to open file (%i) \r\n", ret);
			return -1;
		}
		fsize = f_size(&fil);
		file_buf = (uint8_t *) malloc(fsize * sizeof(uint8_t));
		UINT bytes_read;
		ret = f_read(&fil, file_buf, fsize, &bytes_read);
		if(ret != FR_OK) {
			printf("Failed to read file (%i) \r\n", ret);
			return -1;
		}
		ret = f_close(&fil);
		if(ret != FR_OK) {
			printf("Failed to close file (%i) \r\n", ret);
			return -1;
		}
		XBee_Data data;
		data.command = ReceiveFile;
		data.target = 0;
		*((FSIZE_t *) data.data) = fsize;
		strcpy((char *) &data.data[sizeof(FSIZE_t)], path);
		XBee_Transmit(&data);
		transmitting_file = 1;
		return ret;
	}
	printf("Already transmitting file\n");
	return -1;
}

void XBee_Transmit_File() {
	printf("Transmitting file\n");
	HAL_UART_Transmit_DMA(XBEE_UART, file_buf, fsize);
	transmitting_file = 2;
}

void XBee_Receive(XBee_Data *data) {
	HAL_UART_Receive_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
}

void XBee_Receive_File() {
	if (!receiving_file) {
		printf("Receiving file\n");
		file_buf = (uint8_t *) malloc(rsize);
		HAL_UART_Receive_DMA(XBEE_UART, file_buf, rsize);
		receiving_file = 1;
	} else {
		printf("Already receiving file\n");
	}
}

void XBee_Resolve() {
	if (XBee_Received.target == 0 || XBee_Received.target == UID) {
		switch (XBee_Received.command) {
		case PrintMessage:
			printf("%s\n", (char *) XBee_Received.data);
			break;
		case BroadcastIdentity:
			if (num_registered_devices == MAX_DEVICES) {
				printf("Maximum registered network devices reached\n");
				break;
			}
			uint32_t uid = *((uint32_t *) XBee_Received.data);
			for (int i = 0; i < num_registered_devices; i++) {
				if (devices_removed[i].uid == uid) {
					printf("Already registered device %u\n", (unsigned int) uid);
					goto done;
				}
			}
			devices_removed[num_registered_devices].uid = uid;
			strcpy(devices_removed[num_registered_devices].file_path, (TCHAR *) &XBee_Received.data[sizeof(uint32_t)]);
			num_registered_devices++;
			printf("Registered new device with UID %u\n", (unsigned int) uid);
done:
			break;
		case ReceiveFile:
			printf("Preparing to receive file\n");
			rsize = *((FSIZE_t *) XBee_Received.data);
			strcpy(rpath, (TCHAR *) &XBee_Received.data[sizeof(FSIZE_t)]);
			XBee_Receive_File();
			break;
		case ImpactEvent:
			printf("Impacted detected on device %u\n", XBee_Received.data[0]);
			break;
		default:
			printf("Unknown command received over network\n");
		}
		if (!receiving_file) {
			XBee_Receive(&XBee_Received);
		}
	}
}

int XBee_Resolve_File() {
	int ret = 0;
	ret = f_open(&fil, rpath, FA_OPEN_ALWAYS | FA_WRITE);
	if(ret != FR_OK) {
		printf("Failed to open file (%i) \r\n", ret);
		return -1;
	}
	UINT bytes_written;
	ret = f_write(&fil, file_buf, rsize, &bytes_written);
	if(ret != FR_OK) {
		printf("Failed to write file (%i) \r\n", ret);
		return -1;
	}
	ret = f_close(&fil);
	if(ret != FR_OK) {
		printf("Failed to close file (%i) \r\n", ret);
		return -1;
	}
	printf("Received file\n");
	free(file_buf);
	receiving_file = 0;
	XBee_Receive(&XBee_Received);
	return ret;
}

void XBee_Handshake() {
	printf("Broadcasting identity\n");
	xbee_packet.command = BroadcastIdentity;
	xbee_packet.target = 0;
	*((uint32_t *) xbee_packet.data) = UID;
	sprintf((char *) &xbee_packet.data[sizeof(uint32_t)], "%u.wav", (unsigned int) UID);
	XBee_Transmit(&xbee_packet);
	const TCHAR *path = (TCHAR *) &xbee_packet.data[sizeof(uint32_t)];
	HAL_Delay(500);
	printf("Transmitting file\n");
	XBee_Transmit_File_Start(path);
	while (transmitting_file);
	xbee_packet.command = BroadcastIdentity;
}

void XBee_Init() {
	UID = HAL_GetUIDw0() + HAL_GetUIDw1() + HAL_GetUIDw2();
	printf("UID: %u\n", (unsigned int) UID);
	__HAL_TIM_SET_AUTORELOAD(FILE_TIMER, 5000);
	XBee_Receive(&XBee_Received);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
		printf("XBee Error\n");
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
		if (transmitting_file == 1) {
			while (HAL_TIM_Base_GetState(FILE_TIMER) != HAL_TIM_STATE_READY);
			FIX_TIMER_TRIGGER(FILE_TIMER);
			HAL_TIM_Base_Start_IT(FILE_TIMER);
		} else if (transmitting_file == 2) {
			printf("Transmitted file\n");
			free(file_buf);
			transmitting_file = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == XBEE_UART) {
//		printf("Received - Type: %i, Target: %u, Data: [%u, %u, %u, ...]\n",
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
