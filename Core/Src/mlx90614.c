#include "mlx90614.h"

#define SAD 		(0x5A << 1)

#define RAM_OP 		0x00
#define TAMB		(RAM_OP | 0x06)
#define TOBJ1		(RAM_OP | 0x07)
#define TOBJ2		(RAM_OP | 0x08)


#define EEPROM_OP 		0x20
#define TOMAX_REG		(EEPROM_OP | 0x00)
#define TOMIN_REG		(EEPROM_OP | 0x01)
#define PWM_CTRL_REG	(EEPROM_OP | 0x02)
#define TA_RANGE_REG	(EEPROM_OP | 0x03)
#define EMISSIVITY_REG	(EEPROM_OP | 0x04)
#define CONFIG_REG		(EEPROM_OP | 0x05)

#define HUMAN_EMISSIVITY 1.0
#define TEMP_MIN_F 80
#define TEMP_MAX_F 110



/**
 * @brief 		calculate 8-bit crc code with polynomial X8+X2+X1+1
 * @param[in]	*pointer to byte buffer
 * @param[in]	byte buffer length
 * @return 		crc-8 code
 * @note 		copied from Adafruit Arduino library
 * 				https://github.com/adafruit/Adafruit-MLX90614-Library/blob/master/Adafruit_MLX90614.cpp
 */
static uint8_t crc8(uint8_t* buf, uint8_t buf_len) {
	uint8_t crc = 0;

	while (buf_len--) {
		uint8_t inbyte = *buf++;

		for (uint8_t i = 8; i; --i) {
			uint8_t carry = (crc ^ inbyte) & 0x80;
			crc <<= 1;
			if (carry) {
				crc ^= 0x7;
			}

			inbyte <<= 1;
		}

	}

	return crc;

}

/**
 * @brief 		write a word to MLX90614 with appropriate CCR
 * @param[in]	*handle to an MLX90614 struct
 * @param[in]	register/sub-address
 * @param[in]	word to be written
 * @return		status code:
 * 				- 0 error
 * 				- 1 success
 * @note For an EEPROM opcode, the memory must be cleared first
 */
uint8_t write_word(MLX90614_t* mlx90614, uint8_t address, const uint16_t word) {
	uint8_t buf[5];
	buf[0] = SAD;
	buf[1] = address;

	if (address & EEPROM_OP) {
		buf[2] = 0;
		buf[3] = 0;
		buf[4] = crc8(buf, 4);

		if (HAL_I2C_Mem_Write(mlx90614->hi2c, SAD, address,
				I2C_MEMADD_SIZE_8BIT, buf + 2, 3, HAL_MAX_DELAY) != HAL_OK) {
			return 0;
		}
	}

	HAL_Delay(1000);

	buf[2] = word & 0xFF;
	buf[3] = word >> 8;
	buf[4] = crc8(buf, 4);

	uint8_t status = HAL_I2C_Mem_Write(mlx90614->hi2c, SAD, address,
			I2C_MEMADD_SIZE_8BIT, buf + 2, 3, HAL_MAX_DELAY) == HAL_OK;

	HAL_Delay(1000);

	return status;
}



/**
 * @brief 		read a word from MLX90614 and check CCR
 * @param[in]	*handle to an MLX90614 struct
 * @param[in]	register/sub-address
 * @param[in]	*pointer to a word to be populated
 * @return		status code:
 * 				- 0 error
 * 				- 1 success
 * @note Check manual page 15 for read word transaction
 */
uint8_t read_word(MLX90614_t* mlx90614, uint8_t address, uint16_t* word) {
	uint8_t buf[6];

	if (HAL_I2C_Mem_Read(mlx90614->hi2c, SAD, address,
			I2C_MEMADD_SIZE_8BIT, buf + 3, 3, HAL_MAX_DELAY) != HAL_OK) {
		return 0;
	}

	buf[0] = SAD;
	buf[1] = address;
	buf[2] = SAD + 1;

	if (crc8(buf, 5) != buf[5]) {
		return 0;
	}

	*word = (uint16_t) ((buf[4] << 8) | buf[3]);

	return 1;
}

/**
 * @brief 		convert raw MLX90614 output data to Fahrenheit
 * @param[in]	raw
 * @return 		Fahrenheit value
 */
inline static float raw_to_F(uint16_t raw) {
	return 1.8f * (((float)(raw) * 0.02f) - 273.15) + 32 + 7;
}



uint8_t MLX90614_init(MLX90614_t* mlx90614) {

	uint8_t status = 1;
	status &= write_word(mlx90614, EMISSIVITY_REG, (uint16_t)(HUMAN_EMISSIVITY * 0xffff));

	return status;
}



uint8_t MLX90614_read_ambient(MLX90614_t* mlx90614, float* temp_out) {
	uint16_t raw;

	if (!read_word(mlx90614, TAMB, &raw)) {
		return 0;
	}

	*temp_out = raw_to_F(raw);
	return 1;
}

uint8_t MLX90614_read_object(MLX90614_t* mlx90614, float* temp_out) {
	uint16_t raw;

	if (!read_word(mlx90614, TOBJ1, &raw)) {
		return 0;
	}

	*temp_out = raw_to_F(raw);
	return 1;
}

uint8_t MLX90614_read_emm(MLX90614_t* mlx90614, uint16_t* emm) {
	return read_word(mlx90614, EMISSIVITY_REG, emm);
}








