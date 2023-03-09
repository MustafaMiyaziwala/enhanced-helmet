#include "stm32l4xx_hal.h"
#include <stdint.h>

#define OV5462_I2C_ADDR_W 0x78
#define OV5462_I2C_ADDR_R 0x79

typedef struct OV5462_t {
	I2C_HandleTypeDef* hi2c;
} OV5462_t;

typedef struct reg_value_pair {
	uint16_t addr;
	uint8_t value;
} reg_value_pair;

const reg_value_pair SET_RESOLUTION_320X240[] = {
	 {0x3800 ,0x1 },
	 {0x3801 ,0xa8},
	 {0x3802 ,0x0 },
	 {0x3803 ,0xA },
	 {0x3804 ,0xA },
	 {0x3805 ,0x20},
	 {0x3806 ,0x7 },
	 {0x3807 ,0x98},
	 {0x3808 ,0x1 },
	 {0x3809 ,0x40},
	 {0x380a ,0x0 },
	 {0x380b ,0xF0},
	 {0x380c ,0xc },
	 {0x380d ,0x80},
	 {0x380e ,0x7 },
	 {0x380f ,0xd0},
	 {0x5001 ,0x7f},
	 {0x5680 ,0x0 },
	 {0x5681 ,0x0 },
	 {0x5682 ,0xA },
	 {0x5683 ,0x20},
	 {0x5684 ,0x0 },
	 {0x5685 ,0x0 },
	 {0x5686 ,0x7 },
	 {0x5687 ,0x98},
	 {0x3011 ,0x0F},
	 {0xffff, 0xff}
};

void OV5462_init(OV5462_t*);

uint8_t OV5462_write_reg(OV5462_t*, int addr, int data);
uint8_t OV5462_write_regs(OV5462_t*, const reg_value_pair regs[]);
