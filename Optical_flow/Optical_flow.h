/**
  * @file    Optical_flow.h
  * @brief   光流模块头文件
  */
#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H

#include "main.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;    //I2C1句柄

/**
 * @brief   I2C获取的光流数据结构体
 */
typedef struct {
    uint8_t id;    //光流模块 ID
    uint16_t distance[2];    //TOF 测距数据，单位毫米，distance[0] 为低八位，distance[1] 为高八位
    uint8_t TOF_quality;    //TOF 测距强度，0-100，数值越大表示测距质量越好
    uint8_t no_fuction;   //保留
    int16_t flow_x_integral[2];    //X像素点累计时间内的累加位移.(radians*10000)[ 除以10000乘以高度(mm)后为实际位移(mm)]
    int16_t flow_y_integral[2];    //Y像素点累计时间内的累加位移.(radians*10000)[ 除以10000乘以高度(mm)后为实际位移(mm)]
    int16_t integration_timespan[2];    //上一次发送光流数据到本次发送光流数据的累计时间，微秒
    uint8_t valid;  //状态值:0为光流数据不可用，245为光流数据可用
    uint8_t version;    //光流固件版本号
} OpticalFlow_Original_Data_t;

/**
 * @brief   处理好的光流数据
 */
typedef struct {
    float distance_x;    //X方向位移，单位厘米
    float distance_y;    //Y方向位移，单位厘米
} OpticalFlow_Data_t;

void I2C1_BusRecover(void);

void OpticalFlow_Init(void);

OpticalFlow_Data_t* OpticalFlow_ProcessData(volatile uint8_t* flow_data_update);

void OpticalFlow_Data_Reset(char* param);

#endif
