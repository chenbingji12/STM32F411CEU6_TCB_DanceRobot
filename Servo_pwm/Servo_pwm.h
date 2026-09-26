/**
 * @file   pwm舵机
 */

#ifndef __SERVO_PWM_H__
#define __SERVO_PWM_H__

#include "main.h"

#define MG90_MAX_PULSE 2500
#define MG90_MIN_PULSE 500

extern TIM_HandleTypeDef htim10;

/**
 * @brief   MG90S舵机平滑控制（5次S型曲线法）
 */
typedef struct{
    uint32_t start_time;
    uint16_t target_run_time;
    float target_angle;
    float start_angle;
    float current_angle;
    uint8_t is_running;
}MG90S_Smooth_Control;

void Servo_pwm_Init(float init_angle);

void Servo_pwm_SetAngle(float angle);

void Set_Servo_pwm_TargetAngle(float angle, uint16_t run_time);

void Update_Servo_pwm_Angle(void);

#endif
