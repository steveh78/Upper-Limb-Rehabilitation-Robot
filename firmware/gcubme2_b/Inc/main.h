/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define LD1_Pin GPIO_PIN_9
#define LD1_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// ROTATE | TRANS | PUSHOFF | WHEELCYCLE | U/D | A/R | STEP.1 | STEP.2 |
//    0   |   0   |    0    |      0     |  0  |  0  |   0        0    |
// ROTATE(8bit)				 : 1 = rotate sysmod ON, 0 = rotate sysmod OFF
// TRANS(7bit) 				 : 1 = translate sysmod on, 0 = translate sysmod off
// PUSH_OFF(6bit)     	 	 : 1 = push off sysmod on, 0 = push off sysmod off
// WHEEL_CYCLE(5bit)  		 : 1 = wheel cycle mod sel , 0 = sel off
// HEIGHT LEVEL (4bit)		 : up
// ASSIST RESIST(3bit) 		 : 1 = ASSIST mod , 0 = RESIST mod
// SPEED STEP(2~1bit)		 : 00 = speed0, 01 = speed1, 10 = speed2, 11 = speed3
#define ROTATE 0x80;
#define TRANS 0x40;
#define PUSHOFF 0x20;
#define WHEELCYCLE 0x10;
#define LEVEL 0x08;
#define MOE 0x04;//mode of exercise
#define SP4 0x03;
#define SP3 0x02;
#define SP2 0x01;
#define SP1 0x00;//STOP DEFUALT
#define TEST	0x00
#define ROTATE_ASSIST_1 0b10000101
#define ROTATE_ASSIST_2 0b10000110
#define ROTATE_ASSIST_3 0b10000111
#define ROTATE_RESIST_1 0b10000001
#define ROTATE_RESIST_2 0b10000010
#define ROTATE_RESIST_3 0b10000011
#define TRANS_ASSIST_1	0b01000101
#define TRANS_ASSIST_2	0b01000110
#define TRANS_ASSIST_3	0b01000111
#define TRANS_RESIST_1	0b01000001
#define TRANS_RESIST_2	0b01000010
#define TRANS_RESIST_3	0b01000011

#define MASK_EMERGENCY 0b10000000
#define MASK_SRV	0b01000000
#define MASK_BRK	0b00100000
#define MASK_CL		0b00010000
#define MASK_MBRK	0b00001000
#define MASK_DAC 	0b00000100
#define MASK_ENC 	0b00000010
#define MASK_LNA 	0b00000001

#define MASK_ROTATE 0b10000000
#define MASK_TRANSLATE 0b01000000
#define MASK_PUSHOFF 0b00100000
#define MASK_WHEEL 0b00010000
#define MASK_HEIGHT 0b00001000
#define MASK_ASSIST 0b00000100
#define MASK_STEP 0b00000011

#define ASSIST_TORQUE_DEFAULT 0
#define ASSIST_TORQUE_STEP1 180
#define ASSIST_TORQUE_STEP2 120
#define ASSIST_TORQUE_STEP3 60

#define RESIST_TORQUE_DEFAULT 0
#define RESIST_TORQUE_STEP1 21
#define RESIST_TORQUE_STEP2 42
#define RESIST_TORQUE_STEP3 63

#define ASSIST_FORCE_DEFAULT 1000
#define ASSIST_FORCE_STEP1 10
#define ASSIST_FORCE_STEP2 10
#define ASSIST_FORCE_STEP3 10

#define RESIST_FORCE_DEFAULT 1000
#define RESIST_FORCE_STEP1 100
#define RESIST_FORCE_STEP2 200
#define RESIST_FORCE_STEP3 300

#define TRANS_POINT_0 2000
#define TRANS_POINT_A 2000
#define TRANS_POINT_B 400

#define READY 0
#define BUSY 1

#define STX	0x02
#define ETX 0x03
#define DTL 0x09
#define REQUEST_WRITE	0x10
#define RESPONSE_WRITE	0x20
#define REQUEST_READ	0x30
#define RESPONSE_READ	0x40
#define INT8	0x00
#define INT16	0x04
#define INT32	0x08
#define FLOAT	0x0c
#define POSITION_CMD 111
#define CURRENT_CMD	 113
#define POSITION	 125
#define CH1	1
#define CH2	2
#define POS_CH1 0x01
#define POS_CH2 0x02
#define CUR_CH1 0x11
#define CUR_CH2 0x12

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
