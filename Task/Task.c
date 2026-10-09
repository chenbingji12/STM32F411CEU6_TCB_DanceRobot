/**
  * @file    Task.c
  * @brief   任务控制函数实现（TCB+调度器）
  */

#include "Task.h"

/**
  * @brief  移动动作周期与激活状态
  */
uint32_t move_action_interval_ms = 10;   //移动任务周期，10ms
uint8_t move_action_active = 0;   //移动任务默认不激活

/**
  * @brief  按键事件周期与激活状态，枚举定义
  */
uint32_t key_event_interval_ms = 13;   //按键任务周期，13ms
extern volatile Flag flag;    //动作执行状态标志变量，初始为未执行状态
KeyState key_state = KEY_UP;    //按键状态变量，初始为未按下状态

/**
  * @brief  WS2812B灯带刷新任务周期与激活状态
  */
uint32_t ws2812_change_interval_ms = 39;   //WS2812B灯带刷新任务周期，39ms

/**
  * @brief  实时步态任务周期与激活状态
  */
uint32_t realtime_gait_interval_ms = 25;   //实时步态任务周期，25ms

/**
  * @brief  角度校正任务周期与激活状态
  */
uint32_t angle_correct_interval_ms = 10;   //角度校正任务周期，10ms

/**
  * @brief  IMU速度与位移计算任务周期与激活状态
  */
uint32_t IMU_CalcSpeedAndDisplacement_interval_ms = 5;   //IMU速度与位移计算任务周期，5ms

/**
  * @brief  原地旋转
 */
uint32_t Turn_Left_Or_Right_interval_ms = 10;   //原地旋转任务周期，10ms

/**
 * @brief  倒立摆PID调整任务周期与激活状态
 */
uint32_t Inverted_Pendulum_PID_Adjust_interval_ms = 10;   //倒立摆PID调整任务周期，10ms

/**
  * @brief  表驱动的时间触发合作式调度器
  */
TaskDef task_table[] = {
    {Key_Event,&key_event_interval_ms, 0,(uint8_t*) &flag.key_event},
    {WS2812_Change,&ws2812_change_interval_ms,0,(uint8_t*)&flag.ws2812_change},
    {Angle_Correct_Process,&angle_correct_interval_ms,0,(uint8_t*)&flag.moving},
    {Realtime_Gait,&realtime_gait_interval_ms,0,(uint8_t*)&flag.moving},
    {IMU_CalcSpeedAndDisplacement,&IMU_CalcSpeedAndDisplacement_interval_ms,0,(uint8_t*)&flag.imu_speed_displacement_active},
    {Turn_Left_Or_Right_Task,&Turn_Left_Or_Right_interval_ms,0,(uint8_t*)&flag.Turn_Left_Or_Right},
    {Inverted_Pendulum_PID_Adjust,&Inverted_Pendulum_PID_Adjust_interval_ms,0,(uint8_t*)&flag.inverted_pendulum_pid_adjust},
};

const uint8_t task_count=sizeof(task_table) / sizeof(task_table[0]);// 任务表中任务的数量

/**
  * @brief  处理按键事件,当flag.key_event为1时,15ms执行一次
  * @retval 无
  */
void Key_Event(void)
{
    switch (key_state)      // 根据按钮事件状态进行处理
    {
        case KEY_UP:
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
                key_state = KEY_DOWN;
            } else {
                flag.key_event = 0;   // 误触发，清除标志
            }
            break;
        case KEY_DOWN:
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                key_state = KEY_STAY;
            } else {
                key_state = KEY_UP;
                flag.key_event = 0;   // 短暂按下后松开，结束
            }
            break;
        case KEY_STAY:
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                key_state = KEY_UP;
                flag.key_event = 0;   // 松开，完整周期结束
            }
            break;
    }
}

/**
 * @brief   倒立摆PID调整，在task.c每10ms执行一次
 */
void Inverted_Pendulum_PID_Adjust(void)
{
    const float kp=4.0f;
    const float ki=0.0f;
    const float kd=0.0f;
    const float dt=0.01f; // 10ms
    static float integral_pitch_sum=0.0f;

    static float last_error=0.0f;

    static float target_pitch=0.0f;
    
    static IMU_Data_t* imu_roll_pitch;
    imu_roll_pitch=IMU_GetData();
    float current_pitch=imu_roll_pitch->pitch;

    float error_pitch=target_pitch-current_pitch;

    integral_pitch_sum+=error_pitch*dt;

    integral_pitch_sum=(integral_pitch_sum>10.0f)?10.0f:integral_pitch_sum;
    integral_pitch_sum=(integral_pitch_sum<-10.0f)?-10.0f:integral_pitch_sum;

    float correct_pitch=0.0f;

    if(last_error==0.0f)
    {
      Servo_Write(17,600,500);
      correct_pitch=kp*error_pitch;
    }
    else
    {
      correct_pitch=kp*error_pitch+ki*integral_pitch_sum+kd*(error_pitch-last_error)/dt;
    }

    correct_pitch=(correct_pitch>60.0f)?60.0f:correct_pitch;
    correct_pitch=(correct_pitch<-60.0f)?-60.0f:correct_pitch;

    last_error=error_pitch;

    if(current_pitch>-5.0f&&current_pitch<5.0f)
    {
      correct_pitch=0.0f;// 当角度在5度内时，不调整角度
    }

    Servo_Write(18,(uint16_t)(500+correct_pitch/180.0f*500.0f),10);
    Servo_Write(20,(uint16_t)(500-correct_pitch/180.0f*500.0f),10);

    static uint32_t last_print_time=0;
    if(HAL_GetTick()-last_print_time>=50)
    {
      printf("pitch=%f\n",current_pitch);
    printf("correct_pitch=%f\n",correct_pitch);
    last_print_time=HAL_GetTick();
    }
}

/**
  * @brief  实时步态任务
  */
void Realtime_Gait(void)
{
    Realtime_Gait_Process();
}

/**
  * @brief  处理任务表
  * @retval 无
  */
void Task_Process(void)
{
    for (uint8_t i = 0; i < task_count; i++)      // 遍历任务列表
    {
        if (*task_table[i].is_active==1)      // 如果任务已激活
        {
            if (HAL_GetTick() - task_table[i].last_run_time >= *task_table[i].interval_ms)      // 如果时间到
            {
                task_table[i].last_run_time = HAL_GetTick();      // 更新上次运行时间
                task_table[i].task_func();      // 执行任务
            }
        }
    }
}
