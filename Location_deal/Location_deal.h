/**
 * @file    Location_deal.h
 * @brief   位置处理函数声明
 */

 #ifndef __LOCATION_DEAL_H
 #define __LOCATION_DEAL_H

 #include "IMU.h"
 #include <stdio.h>

/**
 * @brief   IMU计算速度与位移
 */
typedef struct {
    float velocity_x;  // x方向速度 (m/s)
    float velocity_y;  // y方向速度 (m/s)
    float displacement_x;  // x方向位移 (m)
    float displacement_y;  // y方向位移 (m)
} IMU_SpeedDisplacement_t;

void Location_deal_GetIMUData(IMU_Data_t *imu);
void Location_deal_ZeroYaw(IMU_Data_t *imu);
void Location_deal_CalcYawError(IMU_Data_t *imu);
void Location_deal_ClearYawError(void);
void IMU_CalcSpeedAndDisplacement(void);

 #endif
