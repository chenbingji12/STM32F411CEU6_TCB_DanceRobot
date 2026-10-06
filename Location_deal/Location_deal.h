/**
 * @file    Location_deal.h
 * @brief   位置处理函数声明
 */

 #ifndef __LOCATION_DEAL_H
 #define __LOCATION_DEAL_H

 #include "IMU.h"
 #include <stdio.h>

/**
 * @brief   IMU安装方向配置
 *          默认：IMU X轴朝机器人前方，Y轴朝机器人左方，Z轴朝机器人上方
 */
#define IMU_AXIS_X                 0U
#define IMU_AXIS_Y                 1U
#define IMU_AXIS_Z                 2U

#define IMU_SIGN_X                 1.0f
#define IMU_SIGN_Y                 1.0f
#define IMU_SIGN_Z                 1.0f

/**
 * @brief   IMU速度与位移计算参数
 */
#define IMU_GRAVITY                9.80665f
#define IMU_GRAVITY_SIGN           1.0f
#define IMU_ACC_FILTER_ALPHA       0.25f
#define IMU_ACC_DEAD_BAND          0.08f
#define IMU_STATIC_ACC_THRESHOLD   0.20f
#define IMU_STATIC_VELOCITY_LIMIT  0.05f
#define IMU_STATIC_SAMPLE_COUNT    5U
#define IMU_MIN_DT                 0.005f
#define IMU_MAX_DT                 0.12f
#define IMU_MAX_VELOCITY           2.0f
#define IMU_CALIBRATION_SAMPLE_COUNT 50U

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
void Turn_Left_Or_Right_Task(void);
void Location_deal_ZeroYaw(IMU_Data_t *imu);
void Location_deal_CalcYawError(IMU_Data_t *imu);
void Location_deal_ClearYawError(void);
void IMU_CalcSpeedAndDisplacement(void);
void IMU_SpeedDisplacement_StartCalibration(void);
void IMU_SpeedDisplacement_Reset(void);
IMU_SpeedDisplacement_t* IMU_GetSpeedDisplacement(void);
uint8_t IMU_SpeedDisplacement_IsCalibrating(void);

 #endif
