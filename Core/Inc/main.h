/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_capture_flag(int f);
void set_save_requested(int f);
int read_fifo_and_write_data_file();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define HEADLAMP_OUT_Pin GPIO_PIN_0
#define HEADLAMP_OUT_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOC
#define SD_MISO_Pin GPIO_PIN_2
#define SD_MISO_GPIO_Port GPIOC
#define SD_MOSI_Pin GPIO_PIN_3
#define SD_MOSI_GPIO_Port GPIOC
#define ULTRA_RIGHT_ADC_Pin GPIO_PIN_0
#define ULTRA_RIGHT_ADC_GPIO_Port GPIOA
#define ULTRA_LEFT_ADC_Pin GPIO_PIN_1
#define ULTRA_LEFT_ADC_GPIO_Port GPIOA
#define DEBUG_USART_TX_Pin GPIO_PIN_2
#define DEBUG_USART_TX_GPIO_Port GPIOA
#define DEBUG_USART_RX_Pin GPIO_PIN_3
#define DEBUG_USART_RX_GPIO_Port GPIOA
#define ULTRA_CENTER_ADC_Pin GPIO_PIN_4
#define ULTRA_CENTER_ADC_GPIO_Port GPIOA
#define CAM_SCK_Pin GPIO_PIN_5
#define CAM_SCK_GPIO_Port GPIOA
#define CAM_MISO_Pin GPIO_PIN_6
#define CAM_MISO_GPIO_Port GPIOA
#define CAM_MOSI_Pin GPIO_PIN_7
#define CAM_MOSI_GPIO_Port GPIOA
#define CAM_CS_Pin GPIO_PIN_4
#define CAM_CS_GPIO_Port GPIOC
#define AMP_ENABLE_Pin GPIO_PIN_5
#define AMP_ENABLE_GPIO_Port GPIOC
#define SD_SPI2_SCK_Pin GPIO_PIN_10
#define SD_SPI2_SCK_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_9
#define IMU_INT1_GPIO_Port GPIOC
#define XBEE_TX_Pin GPIO_PIN_9
#define XBEE_TX_GPIO_Port GPIOA
#define XBEE_RX_Pin GPIO_PIN_10
#define XBEE_RX_GPIO_Port GPIOA
#define XBEE_CTS_Pin GPIO_PIN_11
#define XBEE_CTS_GPIO_Port GPIOA
#define XBEE_RTS_Pin GPIO_PIN_12
#define XBEE_RTS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DAC_SCK_Pin GPIO_PIN_10
#define DAC_SCK_GPIO_Port GPIOC
#define DAC_MOSI_Pin GPIO_PIN_12
#define DAC_MOSI_GPIO_Port GPIOC
#define DAC_SPI_CS_Pin GPIO_PIN_2
#define DAC_SPI_CS_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define IMU_I2C1_SCL_Pin GPIO_PIN_8
#define IMU_I2C1_SCL_GPIO_Port GPIOB
#define IMU_I2C1_SDA_Pin GPIO_PIN_9
#define IMU_I2C1_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
