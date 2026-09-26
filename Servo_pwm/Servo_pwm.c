/**
 * @file   pwm舵机
 * */

#include "Servo_pwm.h"

static MG90S_Smooth_Control s_smooth_control = {0};//平滑控制结构体

static void Servo_pwm_SetPulse(uint16_t pulse_us);

void Set_Servo_pwm_TargetAngle(float angle, uint16_t run_time);

/**
 * @brief  初始化舵机PWM
 * @param  init_angle  初始化角度值
 * @param  init_run_time  初始化运行时间值
 * @retval None
 * */
void Servo_pwm_Init(float init_angle)
{
    // 初始化舵机角度
    Servo_pwm_SetAngle(init_angle);
    s_smooth_control.start_angle
    =s_smooth_control.current_angle
    =s_smooth_control.target_angle = init_angle;
    // 启动定时器
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
        // 处理错误，跳转到错误处理函数,死循环，看门狗复位
        Error_Handler();
    }
}

/**
 * @brief   设置脉宽和限位
 * @param   pulse  脉宽值
 * @retval  None
 * */
static void Servo_pwm_SetPulse(uint16_t pulse_us)
{
    pulse_us = (pulse_us>MG90_MAX_PULSE)?MG90_MAX_PULSE:pulse_us;
    pulse_us = (pulse_us<MG90_MIN_PULSE)?MG90_MIN_PULSE:pulse_us;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse_us);
}

/**
 * @brief   设置舵机角度
 * @param   angle  角度值
 * @retval  None
 * */
void Servo_pwm_SetAngle(float angle)
{
    // 角度转换为脉宽
    uint16_t pulse_us = (uint16_t)(500.0f+2000.0f/180.0f*angle);
    Servo_pwm_SetPulse(pulse_us);
}

/**
 * @brief   设置舵机角度与运行时间
 * @param   angle  角度值
 * @param   run_time  运行时间值
 * @retval  None
 * */
void Set_Servo_pwm_TargetAngle(float angle, uint16_t run_time)
{
    s_smooth_control.target_angle = angle;
    s_smooth_control.target_run_time = run_time;
    s_smooth_control.is_running = 1;
    s_smooth_control.start_time = HAL_GetTick();
    s_smooth_control.start_angle = s_smooth_control.current_angle;
}
/**
 * @brief   更新舵机角度,用5次S型曲线法平滑控制舵机角度,在20ms的定时器中断中调用
 * @retval  None
 * */
void Update_Servo_pwm_Angle(void)
{
    if(s_smooth_control.is_running)
    {
        uint32_t now_time_ms=HAL_GetTick();
        float t_T=(float)(now_time_ms-s_smooth_control.start_time)/(float)s_smooth_control.target_run_time;
        s_smooth_control.current_angle = s_smooth_control.start_angle
        + (s_smooth_control.target_angle-s_smooth_control.start_angle)
        *(t_T*t_T*6.0f-15.0f*t_T+10.0f)
        *t_T*t_T*t_T;
        Servo_pwm_SetAngle(s_smooth_control.current_angle);
        if( t_T >=1.0f)
        {
            s_smooth_control.is_running = 0;
            s_smooth_control.start_angle = s_smooth_control.target_angle;
        }
    }
}

