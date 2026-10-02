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

extern IMU_Data_t *imu;
/**
 * @brief   根据IMU的加速度计算速度与位移,放在task.c中，5ms调用一次
 * @retval  无
 */
void IMU_CalcSpeedAndDisplacement(void)
{
    static IMU_SpeedDisplacement_t imu_speed_displacement = {0.0f, 0.0f, 0.0f, 0.0f};
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    float dt = (current_time - last_time) /1000.0f; // 时间间隔 (秒)

    // 使用IMU的加速度计算速度和位移
    imu_speed_displacement.velocity_x=imu->acc_x*dt+imu_speed_displacement.velocity_x;
    imu_speed_displacement.velocity_y=imu->acc_y*dt+imu_speed_displacement.velocity_y;
    imu_speed_displacement.displacement_x=imu_speed_displacement.velocity_x*dt
    +imu_speed_displacement.displacement_x;
    imu_speed_displacement.displacement_y=imu_speed_displacement.velocity_y*dt
    +imu_speed_displacement.displacement_y;

    static uint8_t count = 0;
    count++;
    if(count>=10)
    {
    printf("vx=%.2f,vy=%.2f,dx=%.2f,dy=%.2f\n",
           imu_speed_displacement.velocity_x, imu_speed_displacement.velocity_y,
           imu_speed_displacement.displacement_x, imu_speed_displacement.displacement_y);
           count=0;
    }
    last_time = current_time;
}
