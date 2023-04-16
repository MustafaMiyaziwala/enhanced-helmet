#include "button_array.h"

#define MPR121_ADDR 0x5A << 1
#define MPR121_TOUCH_THRESHOLD 48
#define MPR121_RELEASE_THRESHOLD 12

#define MPR121_TOUCHSTATUS_L 0x00
#define MPR121_TOUCHSTATUS_H 0x01
#define MPR121_FILTDATA_0L 0x04
#define MPR121_FILTDATA_0H 0x05
#define MPR121_BASELINE_0 0x1E
#define MPR121_MHDR 0x2B
#define MPR121_NHDR 0x2C
#define MPR121_NCLR 0x2D
#define MPR121_FDLR 0x2E
#define MPR121_MHDF 0x2F
#define MPR121_NHDF 0x30
#define MPR121_NCLF 0x31
#define MPR121_FDLF 0x32
#define MPR121_NHDT 0x33
#define MPR121_NCLT 0x34
#define MPR121_FDLT 0x35

#define MPR121_TOUCHTH_0 0x41
#define MPR121_RELEASETH_0 0x42
#define MPR121_DEBOUNCE 0x5B
#define MPR121_CONFIG1 0x5C
#define MPR121_CONFIG2 0x5D
#define MPR121_CHARGECURR_0 0x5F
#define MPR121_CHARGETIME_1 0x6C
#define MPR121_ECR 0x5E
#define MPR121_AUTOCONFIG0 0x7B
#define MPR121_AUTOCONFIG1 0x7C
#define MPR121_UPLIMIT 0x7D
#define MPR121_LOWLIMIT 0x7E
#define MPR121_TARGETLIMIT 0x7F

#define MPR121_GPIODIR 0x76
#define MPR121_GPIOEN 0x77
#define MPR121_GPIOSET 0x78
#define MPR121_GPIOCLR 0x79
#define MPR121_GPIOTOGGLE 0x7A

#define MPR121_SOFTRESET 0x80



static inline void write(Buttons* btns, uint16_t MemAddress, uint8_t pData) {
	btns->status &= HAL_I2C_Mem_Write(btns->hi2c, MPR121_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, &pData, 1, 100) == HAL_OK;
}

static inline void set_thresholds(Buttons* btns, uint8_t touch, uint8_t release) {
	for (uint8_t i = 0; i < 12; i++) {
		write(btns, MPR121_TOUCHTH_0 + 2 * i, touch);
		write(btns, MPR121_RELEASETH_0 + 2 * i, release);
	}
}


void buttons_init(Buttons* btns) {

	btns->status = 1;

	for (uint8_t i = 0; i < NUM_BTNS; ++i) {
		btns->btns_state[i] = 0;
	}
	write(btns, MPR121_SOFTRESET, 0x63);
	HAL_Delay(1);

	write(btns, MPR121_ECR, 0x00);
	set_thresholds(btns, MPR121_TOUCH_THRESHOLD, MPR121_RELEASE_THRESHOLD);
	write(btns, MPR121_MHDR, 0x01);
	write(btns, MPR121_NHDR, 0x01);
	write(btns, MPR121_NCLR, 0x0E);
	write(btns, MPR121_FDLR, 0x00);
	write(btns, MPR121_MHDF, 0x01);
	write(btns, MPR121_NHDF, 0x05);
	write(btns, MPR121_NCLF, 0x01);
	write(btns, MPR121_FDLF, 0x00);
	write(btns, MPR121_NHDT, 0x00);
	write(btns, MPR121_NCLT, 0x00);
	write(btns, MPR121_FDLT, 0x00);

	write(btns, MPR121_DEBOUNCE, 0b01110111);
	write(btns, MPR121_CONFIG1, 0x10); // default, 16uA charge current
	write(btns, MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

	write(btns, MPR121_AUTOCONFIG0, 0x0B);

	// correct values for Vdd = 3.3V
	write(btns, MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
	write(btns, MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
	write(btns, MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65

	// enable X electrodes and start MPR121
	uint8_t ECR_SETTING = 0b10000000 + NUM_BTNS; // 5 bits for baseline tracking & proximity disabled + N electrodes running
	write(btns, MPR121_ECR, ECR_SETTING); // start with above ECR setting

	if (!btns->status) {
		printf("Capacitive touch init failed!\r\n");
	}
}

uint8_t get_released_button(Buttons* btns) {
	uint8_t read;
	HAL_I2C_Mem_Read(btns->hi2c, MPR121_ADDR, MPR121_TOUCHSTATUS_L, I2C_MEMADD_SIZE_8BIT, &read, 1,
		100);

	for (uint8_t i = 0; i < NUM_BTNS; ++i) {
		if (read & (1 << i)) {
			return i;
		}
	}

	return NO_BTN;
}





