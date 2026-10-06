/**
 * @file    Location_deal.c
 * @brief   位置处理函数定义
 */

#include "Location_deal.h"
#include "main.h"

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
 * @brief  角度归一化
 * @param  angle  输入角度
 * @retval  归一化后的角度
 */
static float Wrap_Yaw_Delta(float angle)
{
    while (angle >= 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

extern volatile Flag flag;
float turnleftangle=0.0f;
void Turn_Left_Or_Right_Task(void)
{
    static float kp=0.12f;
    static float kd=0.002f;
    static float dt=0.01f;
    static float last_error=0.0f;
    static float start_yaw=0.0f;
    static float target_yaw=0.0f;
    static float current_yaw=0.0f;
    static IMU_Data_t* imu_only_yaw;
    imu_only_yaw=IMU_GetData();
    //如果是第一次执行任务，初始化开始角度
    if(start_yaw==0.0f)
    {
        start_yaw=imu_only_yaw->yaw;
        target_yaw=Wrap_Yaw_Delta(start_yaw+turnleftangle);
    }

    current_yaw=imu_only_yaw->yaw;

    //计算误差
    float error=Wrap_Yaw_Delta(target_yaw-current_yaw);

    //此次误差与上次误差符号相反，说明已经过了目标角度，停止转动
    if((last_error > 0.0f && error < 0.0f) ||
     (last_error < 0.0f && error > 0.0f))
    {
        flag.Turn_Left_Or_Right=0U;
        flag.walk_forward=0U;
        flag.moving=0;
        start_yaw=0.0f;

        last_error=0.0f;
        printf("Turn_Left_Or_Right_Task finish\n");//向香橙派反馈
    }
    else if(fabs(error)>0.7f)
    {
        float pid_correct_w=0.0f;
        if(last_error!=0.0f)
        {
            pid_correct_w=kp*error+kd*(error-last_error)/dt;
        }
        else
        {
            pid_correct_w=kp*error;
        }

        pid_correct_w=(pid_correct_w>0.8f)?0.8f:pid_correct_w;
        pid_correct_w=(pid_correct_w<-0.8f)?-0.8f:pid_correct_w;
        pid_correct_w=(fabs(pid_correct_w)<0.1f)?0.1f*pid_correct_w/fabs(pid_correct_w):pid_correct_w;

        Get_Step_Length(0.0f,-pid_correct_w,1.5f);
        flag.moving=1;
        flag.walk_forward=1;

        last_error=error;


        printf("start_yaw=%f,target_yaw=%f,current_yaw=%f,error=%f\n",start_yaw,target_yaw,current_yaw,error);
    }
    else
    {
        flag.Turn_Left_Or_Right=0U;
        flag.walk_forward=0U;
        flag.moving=0;
        start_yaw=0.0f;

        last_error=0.0f;
        printf("Turn_Left_Or_Right_Task finish\n");
    }
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

static IMU_SpeedDisplacement_t imu_speed_displacement =
    {0.0f, 0.0f, 0.0f, 0.0f};

static float acc_bias_x = 0.0f;
static float acc_bias_y = 0.0f;
static float acc_bias_z = 0.0f;

static float acc_filter_x = 0.0f;
static float acc_filter_y = 0.0f;
static float acc_filter_z = 0.0f;
static float acc_previous_x = 0.0f;
static float acc_previous_y = 0.0f;

static float calibration_sum_x = 0.0f;
static float calibration_sum_y = 0.0f;
static float calibration_sum_z = 0.0f;

static uint32_t last_acc_tick = 0;
static uint32_t last_acc_update_tick = 0;
static uint32_t last_print_tick = 0;
static uint16_t calibration_count = 0;
static uint8_t calibration_active = 0;
static uint8_t acc_filter_valid = 0;
static uint8_t acc_previous_valid = 0;
static uint8_t acc_time_valid = 0;
static uint8_t acc_update_valid = 0;
static uint8_t static_count = 0;

/**
 * @brief   获取指定轴的加速度
 * @param   imu IMU数据指针
 * @param   axis 轴编号，0为X轴，1为Y轴，2为Z轴
 * @retval  加速度，单位m/s^2
 */
static float IMU_GetAxisAcceleration(IMU_Data_t* imu, uint8_t axis)
{
    switch(axis)
    {
        case 0U:
            return imu->acc_x;
        case 1U:
            return imu->acc_y;
        case 2U:
            return imu->acc_z;
        default:
            return 0.0f;
    }
}

/**
 * @brief   将机体坐标系加速度转换为世界坐标系并消除重力
 * @param   imu IMU数据指针
 * @param   world_x 世界X轴加速度
 * @param   world_y 世界Y轴加速度
 * @param   world_z 世界Z轴加速度
 * @retval  无
 */
static void IMU_CompensateGravity(IMU_Data_t* imu,
                                   float* world_x,
                                   float* world_y,
                                   float* world_z)
{
    float body_x;
    float body_y;
    float body_z;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float sin_yaw;
    float cos_yaw;

    body_x = IMU_GetAxisAcceleration(imu, IMU_AXIS_X) * IMU_SIGN_X;
    body_y = IMU_GetAxisAcceleration(imu, IMU_AXIS_Y) * IMU_SIGN_Y;
    body_z = IMU_GetAxisAcceleration(imu, IMU_AXIS_Z) * IMU_SIGN_Z;

    roll_rad = imu->roll * 0.01745329252f;
    pitch_rad = imu->pitch * 0.01745329252f;
    yaw_rad = (imu->yaw - self_yaw) * 0.01745329252f;

    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);
    sin_yaw = sinf(yaw_rad);
    cos_yaw = cosf(yaw_rad);

    /* 使用Rz(yaw)*Ry(pitch)*Rx(roll)进行坐标变换 */
    *world_x = cos_yaw * cos_pitch * body_x
             + (cos_yaw * sin_pitch * sin_roll
             - sin_yaw * cos_roll) * body_y
             + (cos_yaw * sin_pitch * cos_roll
             + sin_yaw * sin_roll) * body_z;

    *world_y = sin_yaw * cos_pitch * body_x
             + (sin_yaw * sin_pitch * sin_roll
             + cos_yaw * cos_roll) * body_y
             + (sin_yaw * sin_pitch * cos_roll
             - cos_yaw * sin_roll) * body_z;

    *world_z = -sin_pitch * body_x
             + cos_pitch * sin_roll * body_y
             + cos_pitch * cos_roll * body_z;

    *world_z -= IMU_GRAVITY * IMU_GRAVITY_SIGN;
}

/**
 * @brief   对加速度进行低通滤波
 * @param   input 当前加速度
 * @param   last 上一次滤波后的加速度
 * @retval  滤波后的加速度
 */
static float IMU_LowPassFilter(float input, float last)
{
    return IMU_ACC_FILTER_ALPHA * input
         + (1.0f - IMU_ACC_FILTER_ALPHA) * last;
}

/**
 * @brief   对加速度进行死区处理
 * @param   input 当前加速度
 * @retval  死区处理后的加速度
 */
static float IMU_ApplyDeadBand(float input)
{
    if(fabsf(input) < IMU_ACC_DEAD_BAND)
    {
        return 0.0f;
    }

    return input;
}

/**
 * @brief   限制速度范围
 * @param   velocity 当前速度
 * @retval  限制后的速度
 */
static float IMU_LimitVelocity(float velocity)
{
    if(velocity > IMU_MAX_VELOCITY)
    {
        velocity = IMU_MAX_VELOCITY;
    }
    else if(velocity < -IMU_MAX_VELOCITY)
    {
        velocity = -IMU_MAX_VELOCITY;
    }

    return velocity;
}

/**
 * @brief   清空速度计算状态
 * @param   clear_displacement 是否清空累计位移
 * @retval  无
 */
static void IMU_ClearCalculationState(uint8_t clear_displacement)
{
    imu_speed_displacement.velocity_x = 0.0f;
    imu_speed_displacement.velocity_y = 0.0f;

    if(clear_displacement == 1U)
    {
        imu_speed_displacement.displacement_x = 0.0f;
        imu_speed_displacement.displacement_y = 0.0f;
    }

    acc_filter_x = 0.0f;
    acc_filter_y = 0.0f;
    acc_filter_z = 0.0f;
    acc_previous_x = 0.0f;
    acc_previous_y = 0.0f;
    last_acc_tick = 0;
    last_acc_update_tick = 0;
    last_print_tick = 0;
    static_count = 0;
    acc_filter_valid = 0;
    acc_previous_valid = 0;
    acc_time_valid = 0;
    acc_update_valid = 0;
}

/**
 * @brief   开始IMU静止校准
 * @retval  无
 */
void IMU_SpeedDisplacement_StartCalibration(void)
{
    calibration_sum_x = 0.0f;
    calibration_sum_y = 0.0f;
    calibration_sum_z = 0.0f;
    calibration_count = 0;
    calibration_active = 1;

    IMU_ClearCalculationState(0U);
}

/**
 * @brief   清空IMU速度与位移
 * @retval  无
 */
void IMU_SpeedDisplacement_Reset(void)
{
    calibration_active = 0;
    calibration_count = 0;
    calibration_sum_x = 0.0f;
    calibration_sum_y = 0.0f;
    calibration_sum_z = 0.0f;

    IMU_ClearCalculationState(1U);
}

/**
 * @brief   获取IMU速度与位移
 * @retval  IMU速度与位移数据指针
 */
IMU_SpeedDisplacement_t* IMU_GetSpeedDisplacement(void)
{
    return &imu_speed_displacement;
}

/**
 * @brief   获取IMU校准状态
 * @retval  1为正在校准，0为未校准
 */
uint8_t IMU_SpeedDisplacement_IsCalibrating(void)
{
    return calibration_active;
}

/**
 * @brief   根据IMU的加速度计算速度与位移,放在task.c中，5ms调用一次
 * @retval  无
 */
void IMU_CalcSpeedAndDisplacement(void)
{
    IMU_Data_t* imu_data = IMU_GetData();
    uint32_t current_tick;
    float world_x;
    float world_y;
    float world_z;
    float dt;
    float old_velocity_x;
    float old_velocity_y;
    float horizontal_acc;
    float horizontal_velocity;

    if(imu_data == NULL || imu_data->acc_updated == 0U)
    {
        return;
    }

    current_tick = imu_data->acc_update_tick;
    if(acc_update_valid == 1U &&
       current_tick == last_acc_update_tick)
    {
        return;
    }

    last_acc_update_tick = current_tick;
    acc_update_valid = 1;
    imu_data->acc_updated = 0;

    IMU_CompensateGravity(imu_data, &world_x, &world_y, &world_z);

    if(calibration_active == 1U)
    {
        calibration_sum_x += world_x;
        calibration_sum_y += world_y;
        calibration_sum_z += world_z;
        calibration_count++;

        if(calibration_count >= IMU_CALIBRATION_SAMPLE_COUNT)
        {
            acc_bias_x = calibration_sum_x / (float)calibration_count;
            acc_bias_y = calibration_sum_y / (float)calibration_count;
            acc_bias_z = calibration_sum_z / (float)calibration_count;
            calibration_active = 0;

            IMU_ClearCalculationState(0U);
            last_acc_update_tick = current_tick;
            acc_update_valid = 1;

            printf("[IMU] calibration success, bias=(%.3f,%.3f,%.3f)\r\n",
                   acc_bias_x, acc_bias_y, acc_bias_z);
        }
        return;
    }

    world_x -= acc_bias_x;
    world_y -= acc_bias_y;
    world_z -= acc_bias_z;

    if(acc_time_valid == 0U)
    {
        last_acc_tick = current_tick;
        acc_time_valid = 1;
        acc_filter_x = world_x;
        acc_filter_y = world_y;
        acc_filter_z = world_z;
        acc_filter_valid = 1;
        return;
    }

    dt = (float)(current_tick - last_acc_tick) / 1000.0f;
    last_acc_tick = current_tick;

    if(dt < IMU_MIN_DT)
    {
        return;
    }

    if(dt > IMU_MAX_DT)
    {
        acc_previous_valid = 0;
        acc_filter_x = world_x;
        acc_filter_y = world_y;
        acc_filter_z = world_z;
        acc_filter_valid = 1;
        return;
    }

    if(acc_filter_valid == 0U)
    {
        acc_filter_x = world_x;
        acc_filter_y = world_y;
        acc_filter_z = world_z;
        acc_filter_valid = 1;
    }
    else
    {
        acc_filter_x = IMU_LowPassFilter(world_x, acc_filter_x);
        acc_filter_y = IMU_LowPassFilter(world_y, acc_filter_y);
        acc_filter_z = IMU_LowPassFilter(world_z, acc_filter_z);
    }

    acc_filter_x = IMU_ApplyDeadBand(acc_filter_x);
    acc_filter_y = IMU_ApplyDeadBand(acc_filter_y);
    acc_filter_z = IMU_ApplyDeadBand(acc_filter_z);

    if(acc_previous_valid == 0U)
    {
        acc_previous_x = acc_filter_x;
        acc_previous_y = acc_filter_y;
        acc_previous_valid = 1;
        return;
    }

    old_velocity_x = imu_speed_displacement.velocity_x;
    old_velocity_y = imu_speed_displacement.velocity_y;

    /* 使用梯形积分计算速度 */
    imu_speed_displacement.velocity_x =
        old_velocity_x
        + 0.5f * (acc_previous_x + acc_filter_x) * dt;
    imu_speed_displacement.velocity_y =
        old_velocity_y
        + 0.5f * (acc_previous_y + acc_filter_y) * dt;

    imu_speed_displacement.velocity_x =
        IMU_LimitVelocity(imu_speed_displacement.velocity_x);
    imu_speed_displacement.velocity_y =
        IMU_LimitVelocity(imu_speed_displacement.velocity_y);

    /* 使用梯形积分计算位移 */
    imu_speed_displacement.displacement_x +=
        0.5f * (old_velocity_x
        + imu_speed_displacement.velocity_x) * dt;
    imu_speed_displacement.displacement_y +=
        0.5f * (old_velocity_y
        + imu_speed_displacement.velocity_y) * dt;

    acc_previous_x = acc_filter_x;
    acc_previous_y = acc_filter_y;

    horizontal_acc = sqrtf(acc_filter_x * acc_filter_x
                         + acc_filter_y * acc_filter_y);
    horizontal_velocity = sqrtf(
        imu_speed_displacement.velocity_x
        * imu_speed_displacement.velocity_x
        + imu_speed_displacement.velocity_y
        * imu_speed_displacement.velocity_y);

    if(horizontal_acc < IMU_STATIC_ACC_THRESHOLD &&
       horizontal_velocity < IMU_STATIC_VELOCITY_LIMIT)
    {
        if(static_count < IMU_STATIC_SAMPLE_COUNT)
        {
            static_count++;
        }

        if(static_count >= IMU_STATIC_SAMPLE_COUNT)
        {
            imu_speed_displacement.velocity_x = 0.0f;
            imu_speed_displacement.velocity_y = 0.0f;
        }
    }
    else
    {
        static_count = 0;
    }

    if(current_tick - last_print_tick >= 200U)
    {
        /*printf("ax=%.3f,ay=%.3f,az=%.3f,"
               "vx=%.3f,vy=%.3f,dx=%.3f,dy=%.3f,static=%d\n",
               acc_filter_x, acc_filter_y, acc_filter_z,
               imu_speed_displacement.velocity_x,
               imu_speed_displacement.velocity_y,
               imu_speed_displacement.displacement_x,
               imu_speed_displacement.displacement_y,
               (static_count >= IMU_STATIC_SAMPLE_COUNT) ? 1 : 0);
        last_print_tick = current_tick;*/
    }
}
