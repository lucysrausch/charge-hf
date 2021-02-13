/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	float Vin;
	float Vout;
	float Iin;
  float Icycle;

  float Vset;
  float Iset;
} sepic_control_t;

typedef struct{
	float knob1;
	float knob2;
} knobs_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_8
#define BOOT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* This is the VDDA supply in V */
#define VDDA            ((float)3.3f)

/* Deadtime values for GaN stage */
#define HF_PERIOD       ((uint32_t)680)
#define DT_RISING       ((uint16_t)0)
#define DT_FALLING      ((uint16_t)0)

#define SEPIC_PERIOD    ((uint32_t)16383)

#define SEPIC_VMAX      ((float)    60.0)
#define SEPIC_IMAX      ((float)     3.0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
