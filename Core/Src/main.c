/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "OV5462.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHUNK_SIZE 1024
#define AVIOFFSET 240
#define   rate     0x0a
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS fs;
FATFS * pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t total_space, free_space;
OV5462_t ov5462;

int is_header = 0;
uint bw;
unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
const char zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
const int avi_header[AVIOFFSET] = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, rate, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54,
  0x10, 0x00, 0x00, 0x00, 0x6F, 0x64, 0x6D, 0x6C, 0x64, 0x6D, 0x6C, 0x68, 0x04, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int testSD() {
/* Mount SD Card */
	int ret = 0;
	if(f_mount(&fs, "/", 0) != FR_OK) {
		printf("Failed to mount SD Card\r\n");
		return -1;
	}

	/* Open file to write */
	ret = f_open(&fil, "/TEST.TXT", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if(ret != FR_OK) {
		printf("Failed to open file (%i) \r\n", ret);
		return -1;
	}

	if(f_getfree("", &fre_clust, &pfs) != FR_OK) {
		printf("Free space check failed\r\n");
		return -1;
	}

	total_space = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

	/* free space is less than 1kb */
	if(free_space < 1) {
		printf("Drive is full\r\n");
		return -1;
	}

//	printf("SD CARD MOUNTED! TESTING R/W...\r\n");

	f_puts("TEST", &fil);

	/* Close file */
	ret = f_close(&fil);
	if(ret != FR_OK) {
		printf("Failed to close file (%i) \r\n", ret);
		return -1;
	}

	/* Open file to read */
	ret = f_open(&fil, "/TEST.TXT", FA_READ);
	if(ret != FR_OK) {
		printf("Failed to open in read mode (%i) \r\n", ret);
		return -1;
	}

	char buffer[5];
	f_gets(buffer, sizeof(buffer), &fil);

	if (strcmp(buffer, "TEST")) {
		printf("File contents MISMATCH. FAIL R/W test\r\n");
		return -1;
	}

//	printf("PASSED: read file contents\r\n");

	/* Close file */
	if(f_close(&fil) != FR_OK) {
		printf("Failed to close\r\n");
		return -1;
	}

	if(f_unlink("test.txt") != FR_OK) {
		printf("Failed to delete test file \r\n");
	}

	return 0;
}

void write_quartet(unsigned long i) {
	FRESULT fr;

	uint8_t buf[1];
	buf[0] = i % 0x100;
	fr = f_write(&fil, buf, sizeof(uint8_t), &bw);
	if (fr) printf("quart %i\r\n", fr);
	i = i >> 8;

	buf[0] = i % 0x100;
	fr = f_write(&fil, buf, sizeof(uint8_t), &bw);
	if (fr) printf("quart%i\r\n", fr);
	i = i >> 8;

	buf[0] = i % 0x100;
	fr = f_write(&fil, buf, sizeof(uint8_t), &bw);
	if (fr) printf("quart%i\r\n", fr);
	i = i >> 8;

	buf[0] = i % 0x100;
	fr = f_write(&fil, buf, sizeof(uint8_t), &bw);
	if (fr) printf("quart%i\r\n", fr);
}

int read_fifo_and_write_avi_file() {
	uint8_t temp=0, temp_last=0;
	uint32_t length = 0;
	int i = 0;
	FSIZE_t position = 0;
	uint16_t frame_cnt = 0;
	uint8_t remnant = 0;
	char str[8];
	uint8_t buf[256];
	FRESULT fr;

	length = OV5462_read_fifo_length(&ov5462);
	printf("Buffer length: %u\r\n", length);

	if (length >= MAX_FIFO_LENGTH) {
		printf("Buffer too large\r\n");
		return -1;
	}

	if (length == 0) {
		printf("Buffer empty\r\n");
		return -1;
	}

	movi_size = 0;


	fr = f_open(&fil, "/COOL.AVI", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (fr) {
		printf("couldn't open file (%i)", fr);
		return -1;
	}

	for (i = 0; i < AVIOFFSET; ++i) {
		buf[i] = avi_header[i];
	}

	fr = f_write(&fil, buf, sizeof(uint8_t)*(AVIOFFSET), &bw);
	if (fr) printf("%i\r\n", fr);
	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
	OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command

	i = 0;

	while (length--) {
		temp_last = temp;
		SPI_OptimizedReadByte(&temp);

		if ((temp == 0xD9) && (temp_last == 0xFF)) { // end of image
			buf[i++] = temp;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
			fr = f_write(&fil, buf, sizeof(uint8_t)*i, &bw);
			jpeg_size += i;
			remnant =  (4 - (jpeg_size & 0x00000003)) & 0x00000003;
			jpeg_size = jpeg_size + remnant;
			movi_size += jpeg_size;
			if (remnant > 0) {
				f_write(&fil, zero_buf, sizeof(uint8_t)*remnant, &bw);
			}

			position = f_tell(&fil);
			fr = f_lseek(&fil, position - 4 - jpeg_size);
			write_quartet(jpeg_size);

			position = f_tell(&fil);
			fr = f_lseek(&fil, position + 6);
			f_puts("AVI1", &fil);
			position = f_tell(&fil);

			fr = f_lseek(&fil, position + jpeg_size - 10);

			is_header = 0;
			frame_cnt++;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
			OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
		}

		if (is_header) {
			if (i < 256) {
				buf[i++] = temp;
			} else {
				HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
				fr = f_write(&fil, buf, sizeof(uint8_t)*256, &bw);
				if (fr) return fr;
				i = 0;
				buf[i++] = temp;
				HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
				OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
				jpeg_size += 256;
			}
		} else if ((temp == 0xD8) && (temp_last == 0xFF)) {
			is_header = 1;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
			f_puts("00dc", &fil);
			fr = f_write(&fil, zero_buf, sizeof(uint8_t)*4, &bw);
			if (fr) return fr;
			i = 0;
			jpeg_size = 0;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
			OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
			buf[i++] = temp_last;
			buf[i++] = temp;

		}
	}

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
	fr = f_lseek(&fil, 4);
	if (fr) return fr;
	write_quartet(movi_size + 12*frame_cnt+4);
	unsigned long us_per_frame = 1000000 / rate;
	fr = f_lseek(&fil, 0x20);
	if (fr) return fr;
	write_quartet(us_per_frame);
	unsigned long max_bytes_per_sec = movi_size * rate/ frame_cnt;
	fr = f_lseek(&fil, 0x24);
	if (fr) return fr;
	write_quartet(max_bytes_per_sec);
	unsigned long tot_frames = frame_cnt;
	fr = f_lseek(&fil, 0x30);
	if (fr) return fr;
	write_quartet(tot_frames);

	unsigned long frames = frame_cnt;
	fr = f_lseek(&fil, 0xe0);
	if (fr) return fr;
	write_quartet(frames);
	fr = f_lseek(&fil, 0xe8);
	if (fr) return fr;
	write_quartet(movi_size);
	is_header = 0;

	return 0;
}

int read_fifo_and_write_jpeg_files() {
	uint8_t temp=0, temp_last=0;
	uint32_t length = 0;
	int i = 0;
	int frame_num = 0;
	uint8_t buf[CHUNK_SIZE];
	FRESULT fr;

	length = OV5462_read_fifo_length(&ov5462);
	printf("Buffer length: %u\r\n", length);

	if (length >= MAX_FIFO_LENGTH) {
		printf("Buffer too large\r\n");
		return -1;
	}

	if (length == 0) {
		printf("Buffer empty\r\n");
		return -1;
	}

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
	OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command

	f_open(&fil, "VIDEO.DAT", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	i = 0;

	while (length--) {
		temp_last = temp;
		SPI_OptimizedReadByte(&temp);

		if ((temp == 0xD9) && (temp_last == 0xFF)) { // end of image
			buf[i++] = temp;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
			f_write(&fil, buf, sizeof(uint8_t)*i, &bw);
//			f_close(&fil);

			is_header = 0;

			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
			OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
		}

		if (is_header) {
			if (i < CHUNK_SIZE) {
				buf[i++] = temp;
			} else {
				HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
				f_write(&fil, buf, sizeof(uint8_t)*CHUNK_SIZE, &bw);
				i = 0;
				buf[i++] = temp;
				HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
				OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
			}
		} else if ((temp == 0xD8) && (temp_last == 0xFF)) { // start of new image
			is_header = 1;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
//			++frame_num;
//			int filename_len = snprintf(NULL, 0, "/%d.jpg", frame_num);
//			char* filename = malloc(filename_len+1);
//			snprintf(filename, filename_len+1, "/%d.jpg", frame_num);
//			printf("%s\r\n", filename);
//
//			f_open(&fil, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//			free(filename);
			i = 0;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
			OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
			buf[i++] = temp_last;
			buf[i++] = temp;
		}
	}

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
	is_header = 0;

	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
  	uint8_t buf[1] = { 0x00 }; // dummy write
  	HAL_SPI_Transmit(&hspi1, buf, 1, 100);

  	ov5462.hi2c = &hi2c1;
	ov5462.hspi = &hspi1;

	FRESULT fr;

	printf("program start!\r\n");

  	HAL_Delay(1000);

  	while (1) {
		if(testSD()) {
			printf("SD test FAIL! Retrying...\r\n");
			f_mount(NULL, "/", 1);
			HAL_Delay(10000);
		} else {
			printf("SD test PASS!\r\n");
			break;
		}
  	}

  	while (1) {
  		OV5462_write_spi_reg(&ov5462, 0x00, 0x25);
  		uint8_t tmp = OV5462_read_spi_reg(&ov5462, 0x00);

  		if (tmp == 0x25) {
  		printf("SPI Test PASS!\r\n");
  		break; // continue to program
  		} else {
  		printf("SPI Test FAIL!\r\n");
  		HAL_Delay(1000);
  		}
  	}

  	while (1) {
  		uint8_t upper = OV5462_read_i2c_reg(&ov5462, CHIPID_UPPER);
  		uint8_t lower = OV5462_read_i2c_reg(&ov5462, CHIPID_LOWER);

  		if (upper == 0x56 && lower == 0x42) {
  			printf("I2C Test PASS!\r\n");
  			break; // continue to program
  		} else {
  			printf("I2C Test FAIL!\r\n");
  			HAL_Delay(1000);
  		}
  	}

  	// camera init (sets to JPEG mode)
  	if (OV5462_init(&ov5462)) {
  		printf("Init fail!\r\n");
  	}

  	// clear fifo
  	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

  	uint8_t camera_version = OV5462_read_spi_reg(&ov5462, 0x40);
  	printf("Camera version: %u\r\n", camera_version);
  	uint8_t frames;

  	// set continuous capture (depends on version)
  	if (camera_version && 0x70) {
  		frames = 0xFF;
  	} else {
  		frames = 0x07;
  	}

  	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FRAMES, frames);

  	int is_capture_flag = 1;

	if (is_capture_flag) {
	  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // flush
	  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // clear flag

	  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_START_MASK); // start capture

	  while (!(OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK)) {};
	  int length = (int) OV5462_read_fifo_length(&ov5462);

	  if (length < 0x3FFFFF) {
		  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // flush
		  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // clear flag

		  OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_START_MASK); // start capture
		  while (!(OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK)) {};
		  printf("Capture done\r\n");
	  } else {
		  printf("Capture done\r\n");
	  }

	  int why = read_fifo_and_write_jpeg_files();
	  if (why) {
		  printf("movie save failed (%i) \r\n", why);
	  } else {
		  printf("movie save success \r\n");
	  }

	  fr = f_close(&fil);
	  if (fr) printf("%i\r\n", fr);
	 }

  	if(f_mount(NULL, "/", 1) != FR_OK)
  		printf("Failed to unmount\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SD_SPI2_CS_Pin|CAM_SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_SPI2_CS_GPIO_Port, DAC_SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_SPI2_CS_Pin CAM_SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SD_SPI2_CS_Pin|CAM_SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_SPI2_CS_Pin */
  GPIO_InitStruct.Pin = DAC_SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_SPI2_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
