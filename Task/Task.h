/**
  * @file    Task.h
  * @brief   任务控制函数声明
  */

#ifndef __TASK_H
#define __TASK_H

#include "main.h"

/**
  *@brief 计算动作个数
  */
#define ACTION_NUM (sizeof(move_actions) / sizeof(move_actions[0]))

/**
  *@brief 表驱动的时间触发合作式调度器
  */
typedef struct {
    void (*task_func)(void); // 函数指针：指向具体要执行的任务函数
    uint32_t *interval_ms;    // 任务的期望执行周期 (单位：毫秒)
    uint32_t last_run_time;  // 记录该任务上一次执行时的系统时间戳
    uint8_t  *is_active;      // 任务使能开关 (1:运行, 0:挂起休眠)
} TaskDef;

extern TaskDef task_table[];   // 任务表，存放所有任务的控制块
extern const uint8_t task_count;

/**
  *@brief 按键状态枚举
  */
typedef enum {
    KEY_UP = 0,   //按键未按下状态
    KEY_DOWN,   //按键按下状态
    KEY_STAY    //按键保持状态
} KeyState;
extern KeyState key_state;    //按键状态变量（定义在 Task.c）

/**
  *@brief 任务处理函数
  */
void Key_Event(void);    //按键事件处理函数
void I2S_Beat_Action(void);    //节拍触发任务函数
void I2S_Beat_Task(void);    //节拍检测任务函数
void WS2812_Change(void);    //WS2812B灯带刷新任务函数
void Realtime_Gait(void);    //实时步态任务函数

void Task_Process(void);    //任务处理函数

#endif
