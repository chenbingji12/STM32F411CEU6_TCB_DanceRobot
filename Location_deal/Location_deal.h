/**
 * @file    Location_deal.h
 * @brief   位置处理函数声明
 */

 #ifndef __LOCATION_DEAL_H
 #define __LOCATION_DEAL_H

 #include "IMU.h"

void Location_deal_GetIMUData(IMU_Data_t *imu);
void Location_deal_ZeroYaw(IMU_Data_t *imu);
void Location_deal_CalcYawError(IMU_Data_t *imu);
void Location_deal_ClearYawError(void);

 #endif
