/**
  * @file    Task.c
  * @brief   任务控制函数实现（TCB+调度器）
  */

#include "main.h"
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
  * @brief  节拍检测任务周期与激活状态
  */
uint32_t beat_task_interval_ms = 6;   //节拍任务周期，6ms

/**
  * @brief  当检测到节拍时翻转电平任务
  */
uint32_t beat_action_task_interval_ms = 11;   //节拍触发任务周期，11ms

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
  * @brief  表驱动的时间触发合作式调度器
  */
TaskDef task_table[] = {
    {Key_Event,&key_event_interval_ms, 0,(uint8_t*) &flag.key_event},
    {I2S_Beat_Task,&beat_task_interval_ms, 0, (uint8_t*)&flag.beat_active},
    {I2S_Beat_Action,&beat_action_task_interval_ms, 0, (uint8_t*)&flag.beat_active},
    {WS2812_Change,&ws2812_change_interval_ms,0,(uint8_t*)&flag.ws2812_change},
    {Angle_Correct_Process,&angle_correct_interval_ms,0,(uint8_t*)&flag.moving},
    {Realtime_Gait,&realtime_gait_interval_ms,0,(uint8_t*)&flag.moving},
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
  * @brief  处理节拍动作任务
  * @retval 无
  */
void I2S_Beat_Action(void)
{
  static uint8_t last_beat = 0;   //记录上一次节拍状态
  if(I2S_Beat_IsBeat() && !last_beat)   //检测到节拍触发瞬间
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  last_beat = I2S_Beat_IsBeat();   //更新上一次节拍状态
}

/**
 * @brief   ws2812颜色渐变
 */
void WS2812_Change(void)
{
    static uint8_t red = 0;
    static uint8_t green = 0;
    static uint8_t blue = 0;

    static uint8_t red_direction = 1;   // 红色分量变化方向，1表示增加，0表示减少
    static uint8_t green_direction = 1; // 绿色分量变化方向，1表示增加，0表示减少
    static uint8_t blue_direction = 1;  // 蓝色分量变化方向，1表示增加，0表示减少

    if(red>=250)
    {
      red_direction=0;   // 红色分量达到最大值，改变方向为减少
    }
    else if(red<=5)
    {
      red_direction=1;   // 红色分量达到最小值，改变方向为增加
    }
    if(red_direction==1)
    {
      red++;
    }
    else if(red_direction==0)
    {
      red--;
    }
    if(green>=250)
    {
      green_direction=0;   // 绿色分量达到最大值，改变方向为减少
    }
    else if(green<=5)
    {
      green_direction=1;   // 绿色分量达到最小值，改变方向为增加
    }
    if(green_direction==1)
    {
      green=green+2;
    }
    else if(green_direction==0)
    {
      green=green-2;
    }
    if(blue>=250)
    {
      blue_direction=0;   // 蓝色分量达到最大值，改变方向为减少
    }
    else if(blue<=5)
    {
      blue_direction=1;   // 蓝色分量达到最小值，改变方向为增加
    }
    if(blue_direction==1)
    {
      blue=blue+3;
    }
    else if(blue_direction==0)
    {
      blue=blue-3;
    }
    WS2812_Fill(red, green, blue);
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
