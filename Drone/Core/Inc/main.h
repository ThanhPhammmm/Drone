/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PMW3901_CS_Pin GPIO_PIN_13
#define PMW3901_CS_GPIO_Port GPIOC
#define NR2401_CSN_Pin GPIO_PIN_1
#define NR2401_CSN_GPIO_Port GPIOA
#define NR2401_IRQ_Pin GPIO_PIN_2
#define NR2401_IRQ_GPIO_Port GPIOA
#define NR2401_IRQ_EXTI_IRQn EXTI2_IRQn
#define CS_gyro_Pin GPIO_PIN_3
#define CS_gyro_GPIO_Port GPIOA
#define CS_accel_Pin GPIO_PIN_4
#define CS_accel_GPIO_Port GPIOA
#define BMI088_SCK_Pin GPIO_PIN_5
#define BMI088_SCK_GPIO_Port GPIOA
#define BMI088_MISO_Pin GPIO_PIN_6
#define BMI088_MISO_GPIO_Port GPIOA
#define BMI088_MOSI_Pin GPIO_PIN_7
#define BMI088_MOSI_GPIO_Port GPIOA
#define INT_accel_Pin GPIO_PIN_0
#define INT_accel_GPIO_Port GPIOB
#define INT_accel_EXTI_IRQn EXTI0_IRQn
#define INT_gyro_Pin GPIO_PIN_1
#define INT_gyro_GPIO_Port GPIOB
#define INT_gyro_EXTI_IRQn EXTI1_IRQn
#define NRF2401_CE_Pin GPIO_PIN_2
#define NRF2401_CE_GPIO_Port GPIOB
#define MAG_BARO_SCK_Pin GPIO_PIN_10
#define MAG_BARO_SCK_GPIO_Port GPIOB
#define PMW3901_SCK_Pin GPIO_PIN_12
#define PMW3901_SCK_GPIO_Port GPIOB
#define NRF2401_SCK_Pin GPIO_PIN_13
#define NRF2401_SCK_GPIO_Port GPIOB
#define NRF2401_MISO_Pin GPIO_PIN_14
#define NRF2401_MISO_GPIO_Port GPIOB
#define NRF2401_MOSI_Pin GPIO_PIN_15
#define NRF2401_MOSI_GPIO_Port GPIOB
#define VL53L1X_GPIO1_Pin GPIO_PIN_12
#define VL53L1X_GPIO1_GPIO_Port GPIOA
#define VL53L1X_GPIO1_EXTI_IRQn EXTI15_10_IRQn
#define GPS_TX_Pin GPIO_PIN_15
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOB
#define PMW3901_MISO_Pin GPIO_PIN_4
#define PMW3901_MISO_GPIO_Port GPIOB
#define PMW3901_MOSI_Pin GPIO_PIN_5
#define PMW3901_MOSI_GPIO_Port GPIOB
#define VL53L1X_SCL_Pin GPIO_PIN_6
#define VL53L1X_SCL_GPIO_Port GPIOB
#define VL53L1X_SDA_Pin GPIO_PIN_7
#define VL53L1X_SDA_GPIO_Port GPIOB
#define BMP388_INT_Pin GPIO_PIN_8
#define BMP388_INT_GPIO_Port GPIOB
#define BMP388_INT_EXTI_IRQn EXTI9_5_IRQn
#define MAG_BARO_SDA_Pin GPIO_PIN_9
#define MAG_BARO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
