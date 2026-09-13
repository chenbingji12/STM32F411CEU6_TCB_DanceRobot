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

#include "iwdg.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "LX-16A.h"
#include "Single_action.h"
#include "Task.h"
#include "IMU.h"
#include "FIFO.h"
#include "I2S_beat.h"
#include "Leg_action.h"
#include "Location_deal.h"
#include "Body_action.h"
#include "Optical_flow.h"
#include "WS2812.h"
#include "Servo_pwm.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
  DEBUG=0,
  NORMAL,
  SINGLE_ACTION,
  CIRCULAR_DANCE
} Mode;
extern volatile Mode g_mode;    //动作模式枚举变量

typedef struct {
    uint8_t key_event;          //按键事件标志
    uint8_t beat_active;        //节拍检测任务激活标志
    uint8_t ws2812_change;
    uint8_t zero_yaw;           //软件归零标志
    uint8_t uart1_rx_ready;
    uint8_t uart6_rx_ready;
    uint8_t init_to_half_stand;     //从初始状态到半起立动作标志
    uint8_t half_stand_to_init;     //从半起立到初始状态动作标志
    uint8_t half_stand_to_full_stand; //从半起立到完全起立动作标志
    uint8_t full_stand_to_half_stand; //从完全起立到半起立动作标志
    uint8_t walk_forward;
    uint8_t walk_backward;
    uint8_t move_to_left;
    uint8_t move_to_right;
    uint8_t turn_left;
    uint8_t turn_right;
    uint8_t moving;          //移动动作执行标志
    uint8_t flow_active;        //光流任务激活标志
    uint8_t flow_data_update;   //光流数据更新标志
} Flag;
extern volatile Flag flag;    //动作执行状态标志变量

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn
#define L_R_Pin GPIO_PIN_14
#define L_R_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_10
#define BEEP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define PI 3.1415926f //圆周率
#define UART1_RX_SIZE (64)//UART1 接收缓冲区大小，64 字节
#define UART6_RX_SIZE (100)//UART6 接收缓冲区大小，100 字节
#define VOLTAGE_DIVIDER_RATIO 0.384615f //分压系数：10K/(10K+16K)=0.384615，电压采样值=ADC采样值*3.3/4095/0.384615

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
