#include "imu.h"

#define SAD_W			0x32
#define SAD_R			0x33
#define CTRL_REG_1		0x20
#define CTRL_REG_3		0x22
#define CTRL_REG_4		0x23
#define CTRL_REG_5		0x24
#define INT1_CFG		0x30
#define INT1_THS		0x32
#define INT1_DUR		0x33
#define INT1_SRC		0x31
#define READ_START_REG	0x28 | 0x80






uint8_t imu_init(IMU_t* imu) {

	uint8_t buf[2];

	//	Set control register 1:
	//	- highest data rate
	//	- normal power mode
	//	- all axis enabled
	buf[0] = CTRL_REG_1;
	buf[1] = 0b10010111;

	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}


	//  Set control register 5:
	//  - full scale selection +/- 4G
	buf[0] = CTRL_REG_5;
	buf[1] = 0b00001111;

	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}


	//	Set control register 3:
	//	- AOI1 interrupt on INT1
	buf[0] = CTRL_REG_3;
	buf[1] = 0b01000000;

	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}


	//  Set control register 4:
	//  - full scale selection +/- 16G
	buf[0] = CTRL_REG_4;
	buf[1] = 0b00110000;

	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}


	//  Interrupt 1 Config:
	//  - trigger on OR of events (HI Z)

	buf[0] = INT1_CFG;
	buf[1] = 0b00100000;
	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	//  Interrupt 1 threshold:
	//  - trigger on +/- 16

	buf[0] = INT1_THS;
	buf[1] = 0b01111111;
	if (HAL_I2C_Master_Transmit(imu->hi2c,
		SAD_W, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	imu_clear_int1(imu);

	return 1;

}

uint8_t imu_update(IMU_t* imu) {
	uint8_t buf[6] = {READ_START_REG};

	if (HAL_I2C_Master_Transmit(imu->hi2c, SAD_W, buf, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	if (HAL_I2C_Master_Receive(imu->hi2c, SAD_R, buf, 6, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	imu->x_accel = (buf[1] << 8) | buf[0];
	imu->y_accel = (buf[3] << 8) | buf[2];
	imu->z_accel = (buf[5] << 8) | buf[4];

	return 1;
}


uint8_t imu_clear_int1(IMU_t* imu) {
	uint8_t reg = INT1_SRC;

	if (HAL_I2C_Master_Transmit(imu->hi2c, SAD_W, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	if (HAL_I2C_Master_Receive(imu->hi2c, SAD_R, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	return 1;
}
