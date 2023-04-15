#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>
#include <string.h>

extern FATFS fs;
extern FIL fil;
extern uint32_t total_space, free_space;

extern uint32_t devices[MAX_DEVICES];
extern int num_registered_devices;
extern char name_audio_paths[MAX_DEVICES][MAX_PATH_LENGTH];
extern size_t name_audio_path_lengths[MAX_DEVICES];

extern XBee_Data xbee_packet;

uint32_t UID;
XBee_Data XBee_Received;
uint32_t last_transmit = 0;

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

static void load_known_devices() {
	num_registered_devices = 0;
	fr = f_open(&fil, DEVICE_LIST_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if (fr) {  // file error open/not found
		return;
	}

	uint32_t uid;

	int i = 0;

	while (1) {
		if (i >= MAX_DEVICES) return;
		fr = f_read(&fil, (void*) &uid, sizeof uid, &br);
		if (fr || br < sizeof uid) break; // eof
		devices[i] = uid;
		f_read(&fil, (void*) &name_audio_path_lengths[i], sizeof(size_t), &br);
		f_read(&fil, (void*) &name_audio_paths[i], name_audio_path_lengths[i], &br);
		++i;
	}

	num_registered_devices = i;
	f_close(&fil);
}

static void store_known_devices() {
	fr = f_open(&fil, DEVICE_LIST_FILENAME, FA_OPEN_ALWAYS | FA_WRITE);
	for (int i = 0; i < num_registered_devices; ++i) {
		f_write(&fil, (void*) &devices[i], sizeof(uint32_t), &bw);
		f_write(&fil, (void*) &name_audio_path_lengths[i], sizeof(size_t), &bw);
		f_write(&fil, (void*) &name_audio_paths[i], name_audio_path_lengths[i], &bw);
	}
	f_close(&fil);
}

// return:
// 0 = new device created
// 1 = existing device updated
// 2 = existing device, no change
// 3 = error
static int add_update_device(uint32_t uid, char* audio_path) {
	return 0;
	for (int i = 0; i < num_registered_devices; ++i) {
		if (devices[i] == uid) {
			if (!strcmp(audio_path, name_audio_paths[i])) {
				return 2; // no change
			} else {
				strcpy(name_audio_paths[i], audio_path);
				name_audio_path_lengths[i] = strlen(audio_path)+1;
				store_known_devices();
				return 1;
			}
		}
	}

	// if we reached here, this is a new device!
	if (num_registered_devices == MAX_DEVICES) return 3;
	devices[num_registered_devices] = uid;
	strcpy(name_audio_paths[num_registered_devices], audio_path);
	name_audio_path_lengths[num_registered_devices] = strlen(audio_path)+1;
	++num_registered_devices;
	store_known_devices();
	return 0;
}

void XBee_Transmit(XBee_Data *data) {
	uint32_t time = HAL_GetTick();
	if (last_transmit == 0 || time < last_transmit || time - last_transmit > MIN_TRANSMIT_PERIOD) {
		HAL_UART_Transmit_DMA(XBEE_UART, (uint8_t*) data, sizeof(XBee_Data));
		last_transmit = time;
	}
}

int XBee_Transmit_File_Start(const TCHAR* path, uint32_t target) {
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
		data.target = target;
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
		printf("Receiving file");
		if (!is_receive_target) {
			printf(" but not target of file transfer");
		}
		printf("\n");
		file_buf = (uint8_t *) malloc(rsize);
		HAL_UART_Receive_DMA(XBEE_UART, file_buf, rsize);
		receiving_file = 1;
	} else {
		printf("Already receiving file\n");
	}
}

void XBee_Resolve() {
	if (XBee_Received.command == ReceiveFile) {
		printf("Preparing to receive file\n");
		rsize = *((FSIZE_t *) XBee_Received.data);
		strcpy(rpath, (TCHAR *) &XBee_Received.data[sizeof(FSIZE_t)]);
		is_receive_target = XBee_Received.target == 0 || XBee_Received.target == UID;
		XBee_Receive_File();
	} else if (XBee_Received.target == 0 || XBee_Received.target == UID) {
		switch (XBee_Received.command) {
		case PrintMessage:
			printf("%s\n", (char *) XBee_Received.data);
			break;
		case Register:
			if (num_registered_devices >= MAX_DEVICES) {
				printf("Maximum registered network devices reached\n");
				break;
			}
			uint32_t uid = *((uint32_t *) XBee_Received.data);
			strcpy(rpath, (char *) &XBee_Received.data[sizeof(uint32_t)]);
			int res = add_update_device(uid, rpath);

			int request_audio = 0;

			if (res == 0) {
				printf("Registered new device with UID %u! Requesting...\r\n", (unsigned int) uid);
				request_audio = 1;
			} else if (res == 1) {
				printf("New file detected for device %u! Requesting... \r\n", (uint) uid);
				request_audio = 1;
			} else if (res == 2) {
				printf("Device %u already registered. Audio file up to date. \r\n", (uint) uid);
			}

			if (request_audio) {
				XBee_Data data;
				data.command = RequestAudio;
				data.target = uid;
				XBee_Transmit(&data);
			}

			break;
		case RequestDevices:
			printf("Sending device files\r\n");
			const TCHAR path[MAX_PATH_LENGTH];
			for (int i = 0; i < num_registered_devices; i++) {
				sprintf((char *) path, "/audio/%u.wav", (unsigned int) devices[i]);
				XBee_Transmit_File_Start(path, XBee_Received.source);
				while (transmitting_file);
			}
			printf("Sending device list\r\n");
			xbee_packet.command = ReceiveDevices;
			xbee_packet.source = UID;
			xbee_packet.target = XBee_Received.source;
			*((int *) xbee_packet.data) = num_registered_devices;
			for (int i = 0; i < num_registered_devices; i++) {
				*((uint32_t *) &xbee_packet.data[sizeof(int) + i * sizeof(uint32_t)]) = devices[i];
			}
			XBee_Transmit(&xbee_packet);
			break;
		case ImpactEventAnnounce:
			xbee_packet.command = ImpactEventRelay;
			xbee_packet.source = UID;
			xbee_packet.target = 0;
			memcpy(xbee_packet.data, &XBee_Received.source, sizeof(uint32_t));
			XBee_Transmit(&xbee_packet);
			break;

		case HelpEventAnnounce:
			xbee_packet.command = HelpEventRelay;
			xbee_packet.source = UID;
			xbee_packet.target = 0;
			memcpy(xbee_packet.data, &XBee_Received.source, sizeof(uint32_t));
			XBee_Transmit(&xbee_packet);
			break;
		default:
			printf("Incorrect command send to base station\r\n");
		}
		if (!receiving_file) {
			XBee_Receive(&XBee_Received);
		}
	}
}

int XBee_Resolve_File() {
	int ret = 0;
	if (!is_receive_target) {
		printf("Received file but not target of file transfer\n");
	} else {
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
	}
	free(file_buf);
	receiving_file = 0;
	XBee_Receive(&XBee_Received);
	return ret;
}

void XBee_Handshake() {
	printf("Requesting devices\n");
	xbee_packet.command = RequestDevices;
	xbee_packet.source = UID;
	xbee_packet.target = MASTER_UID;
	XBee_Transmit(&xbee_packet);
	receiving_devices = 1;
	while (receiving_devices);
	printf("Broadcasting identity\n");
	xbee_packet.command = Register;
	xbee_packet.target = 0;
	*((uint32_t *) xbee_packet.data) = UID;
	XBee_Transmit(&xbee_packet);
//	const TCHAR path[MAX_PATH_LENGTH];
//	strcpy((char *) path, (char *) &xbee_packet.data[sizeof(uint32_t)]);
//	HAL_Delay(500);
//	printf("Transmitting file\n");
//	XBee_Transmit_File_Start(path, 0);
//	while (transmitting_file);
}

void XBee_Init() {
//	add_update_device(123456, "/COOL_NAME.WAV");
	load_known_devices();

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
