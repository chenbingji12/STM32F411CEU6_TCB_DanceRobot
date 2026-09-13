/**
 * @file    Location_deal.c
 * @brief   位置处理函数定义
 */

#include "Location_deal.h"

volatile float yaw = 0.0f;
volatile float pitch = 0.0f;
volatile float roll = 0.0f;

volatile float self_yaw = 0.0f;//软件自校准yaw角度
volatile float yaw_error = 0.0f;//yaw 偏差

/**
 * @brief   获取当前IMU数据,放在主循环中调用
 * @retval  无
 */
void Location_deal_GetIMUData(IMU_Data_t* imu)
{
    imu=IMU_GetData();
    if (imu->updated)
    {
        roll = imu->roll;
        pitch = imu->pitch;
        yaw = imu->yaw;
        imu->updated = 0;
    }
		//printf("roll=%f,pitch=%f,yaw=%f\r\n",roll,pitch,yaw);
}

/**
 * @brief   软件归零（设置当前yaw为0度）
 * @retval  无
 */
void Location_deal_ZeroYaw(IMU_Data_t* imu)
{
    float yaw_sum = 0.0f;
    for(uint8_t i=0;i<10;i++)
    {
        while(!imu->updated);
        if (imu->updated)
        {
            yaw_sum += imu->yaw;
            imu->updated = 0;
        }
    }
    self_yaw = yaw_sum/10.0f;
}

/**
 * @brief   计算当前yaw角度的偏差
 * @retval  无
 */
void Location_deal_CalcYawError(IMU_Data_t* imu)
{
    yaw_error = imu->yaw - self_yaw;
}

/**
 * @brief   清空软件归零与yaw角度偏差
 * @retval  无
 */
void Location_deal_ClearYawError(void)
{
    self_yaw = 0.0f;
    yaw_error = 0.0f;
}
