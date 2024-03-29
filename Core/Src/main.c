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
#include "distance_sensor_array.h"
#include "haptic_pwm.h"
#include "button_array.h"
#include "headlamp.h"
#include "xbee.h"
#include "audio.h"
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CAMERA_SPI hspi1
#define SD_SPI hspi2
#define DAC_SPI hspi3

#define CAMERA_CAPTURE_TIMER htim2
#define DISTANCE_SENSOR_TIMER htim9
#define CHUNK_SIZE 4096

#define HELP_EVENT 2
#define IMPACT_EVENT 1


#define camera
#define sd

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
// filesystem
FATFS fs;
FATFS * pfs;
FIL fil; // belongs to xbee
FIL fil_cam;
FIL audio_fil;
FRESULT fres;
DWORD fre_clust;
uint32_t total_space, free_space;
uint bw;

// devices
OV5462_t ov5462;
distance_sensor_array_t distance_sensor_array;
Audio audio;
Ext_DAC_t ext_dac;
Buttons btns;
IMU imu;

// flags
uint8_t event_flag = 0; // an impact has occurred, or help is requested
uint8_t init_complete = 0;
uint8_t countdown_flag = 0;
uint8_t alert_flag = 0;
uint8_t welcome_flag = 0;
uint8_t buttons_enable_flag = 0;
uint8_t record_btn_count = 0;
uint32_t record_btn_time = 0;

int capture_flag = 0;
int check_capturing = 0;
int is_header = 0;
int save_requested = 0;
int save_requested_flag = 0;

int impact_occurred = 0;

uint32_t start_audio_tick = 0;

char victim_filepath[MAX_PATH_LENGTH];


// state
enum CameraState { CAMERA_IDLE, CAMERA_RECAPTURE_WAIT, CAMERA_CHECK, CAMERA_SAVE };
enum CameraState camera_state = CAMERA_IDLE;

// xbee
uint32_t devices[MAX_DEVICES];
int num_registered_devices;
XBee_Data xbee_packet;
XBee_Data XBee_Received;
uint32_t UID;
uint32_t victim_uid = 0;
volatile uint8_t tr;
volatile int receiving_file;
extern uint8_t *file_buf;

// camera
uint8_t curr_camera_byte=0;
uint32_t camera_fifo_length = 0;
int camera_buf_idx = 0;
uint8_t camera_buf[CHUNK_SIZE];
int video_id = 0;
FRESULT fr;

uint8_t ULTRASONIC_IGNORE = 1;
uint8_t pulsing = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void trigger_capture() {
	printf("Capture!\r\n");
	capture_flag = 0;
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // clear flag
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_RESET_WRITE);
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_RESET_READ);
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_START_MASK); // start capture

	check_capturing = 1;
	TIM2->CNT = 0;
}

int read_fifo_and_write_data_file() {
//	while (!(OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK)) {}; // wait for buffer to fill before saving
//	while (!(OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK)) {}; // wait for final frame
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_RESET_READ);
	OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK); // clear flag

	uint8_t temp=0;
	uint32_t length = 0;
	int i = 0;
	uint8_t buf[CHUNK_SIZE];

	static int video_id = 0;

	length = OV5462_read_fifo_length(&ov5462);
	printf("Buffer length: %lu\r\n", length);

	if (length >= MAX_FIFO_LENGTH) {
		printf("Buffer too large\r\n");
		length = MAX_FIFO_LENGTH-1;
	}

	if (length == 0) {
		printf("Buffer empty\r\n");
		return -1;
	}

//	length = MAX_FIFO_LENGTH-1; // !! ASSUME BUFFER IS FULL !!

	int filename_len = snprintf(NULL, 0, "%d.DAT", video_id);
	char* filename = malloc(filename_len+1);
	snprintf(filename, filename_len+1, "%d.DAT", video_id);

	FRESULT fr = f_open(&fil_cam, filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	printf("%s\r\n", filename);
	free(filename);
	if (fr) printf("file open failed\r\n");
	++video_id;
	i = 0;

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);
	OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command

	while (length--) {
		temp = SPI_OptimizedReadByte();
		if (i < CHUNK_SIZE) {
			buf[i++] = temp;
		} else {
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);

			f_write(&fil_cam, buf, sizeof(uint8_t)*CHUNK_SIZE, &bw);

			i = 0;
			buf[i++] = temp;
			HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_RESET);

			OV5462_request_FIFO_burst(&ov5462); // send FIFO burst command
		}
	}

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
	is_header = 0;
	f_close(&fil_cam);
	printf("Save complete\r\n");
	xbee_packet.command = PrintMessage;
	xbee_packet.target = MASTER_UID;
	strcpy((char *) xbee_packet.data, "Helmet video save complete");
	XBee_Transmit(&xbee_packet);
	save_requested = 0;

	trigger_capture();

//	OV5462_continuous_capture_init(&ov5462); // restore continuous capture functionality

	return 0;
}

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

/* INTERRUPT CALLBACKS */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim == audio.htim) {
		audio_callback(&audio);
	}
	else if (htim == &DISTANCE_SENSOR_TIMER) {
		update_readings_async(&distance_sensor_array);
	} else if (htim == &CAMERA_CAPTURE_TIMER) {
		capture_flag = 1;
		if (impact_occurred) {
			save_requested = 1;
			impact_occurred = 0;
		}

	} else if (htim == HEADLAMP_TIMER) {
		HAL_TIM_Base_Stop_IT(HEADLAMP_TIMER);
		HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port, HEADLAMP_OUT_Pin, GPIO_PIN_SET);
	} else if (htim == FILE_TIMER) {
		HAL_TIM_Base_Stop_IT(FILE_TIMER);
		if (tr && receiving_file) {
			printf("Receiving file timed out\r\n");
			HAL_UART_DMAStop(XBEE_UART);
			HAL_Delay(500);
			free(file_buf);
			receiving_file = 0;
			XBee_Receive(&XBee_Received);
		} else if (!tr) {
			XBee_Transmit_File();
		}
	} else if (htim == PULSE_TIMER) {
		HAL_TIM_Base_Stop_IT(PULSE_TIMER);
		if (pulsing == 1) {
			PWM_ADD(LEFT_PWM, -PULSE_VAL);
		} else if (pulsing == 2) {
			PWM_ADD(RIGHT_PWM, -PULSE_VAL);
		} else {
			PWM_ADD(LEFT_PWM, -PULSE_VAL);
			PWM_ADD(RIGHT_PWM, -PULSE_VAL);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// TODO: remove, this is just for testing
	if (GPIO_Pin == 0x2000) {
		printf("Save requested\r\n");
		save_requested = 1;
	} else if (GPIO_Pin & (1 << 7)) { // Buttons interrupt

		if (!buttons_enable_flag) {
			return;
		}

		uint8_t btn = get_released_button(&btns);

		if (btn != NO_BTN && countdown_flag) {
			clear_queue(&audio);
			if (countdown_flag == HELP_EVENT) {
				play_wav(&audio, "/audio/cancel_request.wav");
			} else {
				play_wav(&audio, "/audio/cancel_alert.wav");
			}

			countdown_flag = 0;
//			if (btn == HELP_BTN || btn == RECORD_BTN) {
//				PWM_PULSE_RIGHT();
//			} else {
//				PWM_PULSE_LEFT();
//			}
		} else if (!is_playing(&audio)) {

		switch (btn) {
			case HELP_BTN:
				printf("Help\r\n");
				play_wav(&audio, "/audio/siren.wav"); // possibly change chime
				play_wav(&audio, "/audio/help_siren.wav");
				countdown_flag = HELP_EVENT;
//				PWM_PULSE_RIGHT();
				break;

			case RECORD_BTN:
				printf("Record\r\n");

				if (HAL_GetTick() - record_btn_time < 2000) {
					++record_btn_count;
				} else {
					record_btn_count = 1;
				}

				record_btn_time = HAL_GetTick();

				if (record_btn_count == 2) {
					printf("Recording triggered\r\n");
					record_btn_count = 0;
					if (!save_requested && camera_state != CAMERA_SAVE) {
						// play_wav(&audio, "/audio/important_chime.wav");
						// play_wav(&audio, "/audio/save_video.wav");
						// start_audio_tick = HAL_GetTick();
						 save_requested = 1;
						// capture_flag = 0;
						// TIM2->CNT = 0;
					} else {
						play_wav(&audio, "/audio/error.wav");
						printf("already recording\r\n");
					}
					// TODO: Set camera state machine
					PWM_PULSE_RIGHT();
				}

				break;

			case HAPTICS_BTN:
				if(ULTRASONIC_IGNORE) {
					printf("Enabling Haptics\r\n");
					//play_wav(&audio, "/audio/siren.wav");
					play_wav(&audio, "/audio/enable_haptics.wav");

					PWM_RESET_IGNORE();
				} else {
					printf("Disabling Haptics\r\n");
//					play_wav(&audio, "/audio/video_chime.wav");
					play_wav(&audio, "/audio/disable_haptics.wav");
					PWM_SET_IGNORE();
				}
//				PWM_PULSE_LEFT();
				break;

			case LIGHT_BTN:
				printf("Lights\r\n");
				toggle_headlamp();
				PWM_PULSE_LEFT();
				break;

			default:
				printf("None\r\n");
				break;

			}
		}


	} else if (GPIO_Pin & (1 << 9)) { // IMU collision interrupt
		printf("Impact detected\r\n");
		imu_clear_int1(&imu);
//		play_wav(&audio, "/audio/siren.wav");
		play_wav(&audio, "/audio/impact_siren.wav");
		countdown_flag = IMPACT_EVENT;
		impact_occurred = 1;
	}
}


//
//void play_welcome() {
//
//	if (welcome_flag == 2) {
//		if (is_playing(&audio) || audio.queue[audio.read_pos] != NULL ) {
//			return;
//		}
//		printf("Switching\r\n");
//		welcome_flag = 3;
//	}
//
//	if (welcome_flag == 1) {
//		play_wav(&audio, "/intro/intro_p1.wav");
//		++welcome_flag;
//		//welcome_flag = 0;
//		return;
//	}
//
//	if (welcome_flag == 3) {
//		welcome_flag = 0;
//	}
//
//
//	play_wav(&audio, "/intro/intro_p2.wav");
//	play_wav(&audio, "/intro/intro_p3.wav");
//	play_wav(&audio, "/intro/intro_p4.wav");
//	play_wav(&audio, "/intro/intro_p5.wav");
//	play_wav(&audio, "/intro/intro_p6.wav");
//
//
//}

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
//  SCB->VTOR = FLASH_BASE;
//  __set_BASEPRI(0);

	Headlamp_Init();


	FIX_TIMER_TRIGGER(&htim2);
	FIX_TIMER_TRIGGER(&htim3);
	FIX_TIMER_TRIGGER(&htim4);
	FIX_TIMER_TRIGGER(&htim5);
	FIX_TIMER_TRIGGER(&htim10);
	FIX_TIMER_TRIGGER(&htim9);
	FIX_TIMER_TRIGGER(&htim11);

	HAL_TIM_Base_Start_IT(&htim2);
	HAPTICS_INIT();

	HAL_GPIO_WritePin(OV5462_CS_GPIO, OV5462_CS_PIN, GPIO_PIN_SET);
	uint8_t buf[1] = { 0x00 }; // dummy write
	HAL_SPI_Transmit(&hspi1, buf, 1, 100);

	ov5462.hi2c = &hi2c1;
	ov5462.hspi = &CAMERA_SPI;

	printf("program start!\r\n");





//	/* INITIALIZATION + TESTS BEGIN */
#ifdef sd
	if(testSD()) {
		printf("SD test FAIL!\r\n");
//		return 1;
	} else {
		printf("SD test PASS!\r\n");
	}
#endif

#ifdef camera
	while (1) {
		OV5462_write_spi_reg(&ov5462, 0x00, 0x25);
		uint8_t tmp = OV5462_read_spi_reg(&ov5462, 0x00);

		if (tmp == 0x25) {
			printf("Camera SPI Test PASS!\r\n");
			break; // continue to program
		} else {
			printf("Camera SPI Test FAIL!\r\n");
			HAL_Delay(1000);
		}
	}

	while (1) {
		uint8_t upper = OV5462_read_i2c_reg(&ov5462, CHIPID_UPPER);
		uint8_t lower = OV5462_read_i2c_reg(&ov5462, CHIPID_LOWER);

		if (upper == 0x56 && lower == 0x42) {
			printf("Camera I2C Test PASS!\r\n");
			break; // continue to program
		} else {
			printf("Camera I2C Test FAIL!\r\n");
			HAL_Delay(1000);
		}
	}

	// camera init (sets to JPEG mode)
	if (OV5462_init(&ov5462)) {
		printf("Camera init fail!\r\n");
	}

	OV5462_continuous_capture_init(&ov5462);
#endif
	
	XBee_Init();
	XBee_Handshake();


	// audio struct initialize
	ext_dac.cs_port = GPIOD;
	ext_dac.cs_pin = GPIO_PIN_2;
	ext_dac.hspi = &hspi3;

	audio.fs = &fs;
	audio.fil = &audio_fil;
	audio.ext_dac = &ext_dac;
	audio.htim = &htim4;
	audio.amp_enable_port = GPIOC;
	audio.amp_enable_pin = GPIO_PIN_5;
	audio_init(&audio);

	btns.hi2c = &hi2c1;
	buttons_init(&btns);

	imu.hi2c = &hi2c1;
	imu_init(&imu);

	distance_sensor_array.hadc = &hadc1;


	
	/* INITIALIZATION + TESTS END */

	//HAL_NVIC_DisableIRQ(TIM4_IRQn);
#ifdef camera
	OV5462_continuous_capture_init(&ov5462);

	 trigger_capture();
	 check_capturing = 1;

	TIM2->CNT = 0;
#endif
	//HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_TIM_Base_Start_IT(&htim9);
	init_complete = 1;

//	__set_BASEPRI(1 << 4);
//	while (1) {}




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* UPDATES (ALWAYS RUN) */

		if (!buttons_enable_flag) {
			imu_update(&imu);

			printf("%d\r\n", imu.z_accel);

			if (imu.z_accel < -3500) {
				buttons_enable_flag = 1;
				play_wav(&audio, "/audio/init.wav");
			}
		}

//		printf("%lu\t%lu\t%lu\r\n", distance_sensor_array.readings[LEFT_SENSOR],
//					distance_sensor_array.readings[CENTER_SENSOR],
//					distance_sensor_array.readings[RIGHT_SENSOR]);
		PWM_SET_CENTER(get_motor_value(&distance_sensor_array, CENTER_SENSOR));
		PWM_SET_LEFT(get_motor_value(&distance_sensor_array, LEFT_SENSOR));
		PWM_SET_RIGHT(get_motor_value(&distance_sensor_array, RIGHT_SENSOR));


		check_and_fill_audio_buf(&audio);

		/* MAIN STATE MACHINE */
		if (countdown_flag && (audio.read_pos == audio.write_pos) && !is_playing(&audio)) {
			event_flag = countdown_flag;
			countdown_flag = 0;

			xbee_packet.target = 0; // alert everyone


			if (event_flag == HELP_EVENT) {
//				play_wav(&audio, "/audio/request_sent.wav");
				xbee_packet.command = HelpEvent;
			}
			else {
//				play_wav(&audio, "/audio/alert_sent.wav");
				xbee_packet.command = ImpactEvent;
			}

			XBee_Transmit(&xbee_packet);

			event_flag = 0;
//			save_requested = 1;
//			capture_flag = 0;
//			TIM2->CNT = 0;
//			start_audio_tick = HAL_GetTick();
		}

		if (alert_flag) {
			clear_queue(&audio);
			play_wav(&audio, "/audio/important_chime.wav");
			sprintf(victim_filepath, "/audio/%u.wav", (uint) victim_uid);
			play_wav(&audio, victim_filepath);

			if (alert_flag == HELP_ALERT) {
				play_wav(&audio, "/audio/help_response.wav");
			} else if (alert_flag == IMPACT_ALERT) {
				play_wav(&audio, "/audio/impact_response.wav");
			}

			alert_flag = 0;
		}

//		if (save_requested_flag) {
//			save_requested = 1;
//			save_requested_flag = 0;
//		}

//		if (!audio.is_playing) {
//				printf("Audio not playing\r\n");
//		}

#ifdef camera
		/* CAMERA STATE MACHINE */

//		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		if (capture_flag && (OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK)) {
			 if (save_requested) {
				 read_fifo_and_write_data_file();
			 } else if (!impact_occurred) {
				 trigger_capture();
			 }
		 }

		 if (!impact_occurred && check_capturing) {
			 if (TIM2->CNT < 10000) {
				 if (OV5462_read_spi_reg(&ov5462, ARDUCHIP_TRIGGER) & CAPTURE_DONE_MASK) {
					 uint32_t length = OV5462_read_fifo_length(&ov5462);

					 if (length < 0x3FFFFF && length > 0) {
						 printf("Premature capture completion! %lu bytes \r\n", length);

						OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
						OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_START_MASK);
						check_capturing = 0;
					 } else {
						 OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
						 OV5462_write_spi_reg(&ov5462, ARDUCHIP_FIFO, FIFO_START_MASK);
					 }
				 }
			 } else {
				 check_capturing = 0; // it's been over a second, the capture probably started successfully
			 }
		 }

//		HAL_NVIC_EnableIRQ(TIM4_IRQn);

		//HAL_NVIC_EnableIRQ(TIM4_IRQn);
#endif
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7619;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8399;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 20000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 42000 - 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HEADLAMP_OUT_Pin|SD_CS_Pin|CAM_CS_Pin|AMP_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HEADLAMP_OUT_Pin */
  GPIO_InitStruct.Pin = HEADLAMP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEADLAMP_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_CS_Pin AMP_ENABLE_Pin */
  GPIO_InitStruct.Pin = CAM_CS_Pin|AMP_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_CS_Pin */
  GPIO_InitStruct.Pin = DAC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_INT_Pin */
  GPIO_InitStruct.Pin = BTN_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
