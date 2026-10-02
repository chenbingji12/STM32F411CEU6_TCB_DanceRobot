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
  * @brief  表驱动的时间触发合作式调度器
  */
TaskDef task_table[] = {
    {Key_Event,&key_event_interval_ms, 0,(uint8_t*) &flag.key_event},
    {WS2812_Change,&ws2812_change_interval_ms,0,(uint8_t*)&flag.ws2812_change},
    {Angle_Correct_Process,&angle_correct_interval_ms,0,(uint8_t*)&flag.moving},
    {Realtime_Gait,&realtime_gait_interval_ms,0,(uint8_t*)&flag.moving},
    {IMU_CalcSpeedAndDisplacement,&IMU_CalcSpeedAndDisplacement_interval_ms,0,(uint8_t*)&flag.imu_speed_displacement_active},
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
