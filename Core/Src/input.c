#include "input.h"
#include "xbee.h"
#include "headlamp.h"
#include <stdio.h>
#include <string.h>
#include "ff.h"

extern FATFS fs;
extern uint32_t UID;

uint8_t status[8];
int input_connected = 1;

extern XBee_Data XBee_Send;

void Input_Touched(int button) {
	if (button <= NUM_ELECTRODES) {
		printf("Button %i pressed\n", button);
	} else {
		printf("Button not configured\n");
	}
	if (button == 5) {
		toggle_headlamp();
	}
//	char buffer[50];
//	switch (button) {
//	case 0:
//		printf("Sending message\n");
//		XBee_Send.command = PrintMessage;
//		XBee_Send.target = 0;
//		sprintf(buffer, "Hello from device %u", (unsigned int) UID);
//		strcpy((char *) XBee_Send.data, buffer);
//		XBee_Transmit(&XBee_Send);
//		break;
//	case 1:
//		printf("Toggling headlamp\n");
//		toggle_headlamp();
//		break;
//	default:
//		printf("Button not configured\n");
//	}
}

void Input_Released(int button) {
	if (button <= NUM_ELECTRODES) {
		printf("Button %i released\n", button);
	} else {
		printf("Button not configured\n");
	}
}

void Input_Write(uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(INPUT_I2C, MPR121_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
	HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		input_connected = 0;
	}
}

void Input_Write_Byte(uint16_t MemAddress, uint8_t pData) {
	uint8_t buf[] = { pData };
	Input_Write(MemAddress, buf, 1);
}

void Input_Read(uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(INPUT_I2C, MPR121_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size,
	HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		input_connected = 0;
	}
}

uint8_t Input_Read_Byte(uint16_t MemAddress) {
	uint8_t buf[1];
	Input_Read(MemAddress, buf, 1);
	return buf[0];
}

void Input_Resolve() {
	uint8_t read = Input_Read_Byte(MPR121_TOUCHSTATUS_L);
	for (int i = 0; i < 8; i++) {
		uint8_t button = read & (1 << i);
		if (button && !status[i]) {
			Input_Touched(i);
		} else if (!button && status[i]) {
			Input_Released(i);
		}
		status[i] = button;
	}
}

void Input_Set_Thresholds(uint8_t touch, uint8_t release) {
	for (uint8_t i = 0; i < 12; i++) {
		Input_Write_Byte(MPR121_TOUCHTH_0 + 2 * i, touch);
		Input_Write_Byte(MPR121_RELEASETH_0 + 2 * i, release);
	}
}

void Input_Init() {
	Input_Write_Byte(MPR121_SOFTRESET, 0x63);
	HAL_Delay(1);
	Input_Write_Byte(MPR121_ECR, 0x00);
	Input_Set_Thresholds(MPR121_TOUCH_THRESHOLD, MPR121_RELEASE_THRESHOLD);
	Input_Write_Byte(MPR121_MHDR, 0x01);
	Input_Write_Byte(MPR121_NHDR, 0x01);
	Input_Write_Byte(MPR121_NCLR, 0x0E);
	Input_Write_Byte(MPR121_FDLR, 0x00);

	Input_Write_Byte(MPR121_MHDF, 0x01);
	Input_Write_Byte(MPR121_NHDF, 0x05);
	Input_Write_Byte(MPR121_NCLF, 0x01);
	Input_Write_Byte(MPR121_FDLF, 0x00);

	Input_Write_Byte(MPR121_NHDT, 0x00);
	Input_Write_Byte(MPR121_NCLT, 0x00);
	Input_Write_Byte(MPR121_FDLT, 0x00);

	Input_Write_Byte(MPR121_DEBOUNCE, 0);
	Input_Write_Byte(MPR121_CONFIG1, 0x10); // default, 16uA charge current
	Input_Write_Byte(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

	Input_Write_Byte(MPR121_AUTOCONFIG0, 0x0B);

	// correct values for Vdd = 3.3V
	Input_Write_Byte(MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
	Input_Write_Byte(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
	Input_Write_Byte(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65

	// enable X electrodes and start MPR121
	uint8_t ECR_SETTING = 0b10000000 + NUM_ELECTRODES; // 5 bits for baseline tracking & proximity disabled + N electrodes running
	Input_Write_Byte(MPR121_ECR, ECR_SETTING); // start with above ECR setting
	if (input_connected) {
		printf("Capacitive touch board initialized\n");
	} else {
		printf("Capacitive touch board not connected\n");
	}
}
