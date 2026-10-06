/**
  ******************************************************************************
  * @file    IMU.h
  * @brief   Yahboom九轴IMU模块串口数据解析头文件
  *          协议: 帧头0x7E 0x23, 变长帧, 小端序, 累加和校验
  *          使用USART2 (PA2/PA3), DMA循环模式 + 空闲中断接收
  ******************************************************************************
  */

#ifndef __IMU_H
#define __IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"                  // Device header

//#include <stdint.h>

/* 协议常量 ------------------------------------------------------------------*/
#define IMU_HDR1            0x7E    /* 帧头第1字节 */
#define IMU_HDR2            0x23    /* 帧头第2字节 */
#define IMU_FUNC_RAW        0x04    /* 原始数据功能字 */
#define IMU_FUNC_EULER      0x26    /* 欧拉角功能字 */
#define IMU_RAW_FRAME_LEN   0x17    /* 原始数据帧总长度(23字节) */
#define IMU_EULER_FRAME_LEN 0x11    /* 欧拉角帧总长度(17字节) */

/* DMA接收缓冲区大小(字节) */
#define IMU_DMA_BUF_SIZE    256

/* IMU数据结构 ----------------------------------------------------------------*/
typedef struct {
    float roll;       /* 横滚角, 单位: 度 */
    float pitch;      /* 俯仰角, 单位: 度 */
    float yaw;        /* 偏航角, 单位: 度 */
    float acc_x;      /* X轴加速度, 单位: m/s^2 */
    float acc_y;      /* Y轴加速度, 单位: m/s^2 */
    float acc_z;      /* Z轴加速度, 单位: m/s^2 */
    uint8_t acc_updated;       /* 新加速度数据标志 */
    uint8_t attitude_updated;  /* 新姿态数据标志 */
    uint32_t acc_update_tick;  /* 加速度数据到达时间, 单位: 毫秒 */
    uint8_t updated;  /* 新数据标志: 1=有新数据, 主循环读取后清零 */
} IMU_Data_t;

/* 函数声明 ------------------------------------------------------------------*/

/**
  * @brief  初始化IMU模块, 启动USART2的DMA+空闲中断接收
  * @param  huart: USART2句柄指针 (&huart2)
  * @retval 无
  */
void IMU_Init(UART_HandleTypeDef *huart);

/**
  * @brief  USART2空闲中断回调, 由HAL_UARTEx_RxEventCallback中调用
  * @param  Size: 本次接收到的字节数
  * @retval 无
  */
void IMU_RxEventCallback(uint16_t Size);

/**
  * @brief  获取最新IMU数据指针
  * @retval IMU_Data_t指针
  */
IMU_Data_t* IMU_GetData(void);

#endif /* __IMU_H */
