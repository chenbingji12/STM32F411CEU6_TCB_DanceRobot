/**
 * @file    Body_action.h
 * @brief   体动作函数声明
 */
#ifndef __BODY_ACTION_H
#define __BODY_ACTION_H

#include "main.h"

extern const uint16_t arm_action_data_1[][4][3]; // 机械臂第一个动作数据
extern const uint8_t arm_action_count_1; // 机械臂第一个动作的帧数

extern const uint16_t arm_action_data_2[][4][3]; // 机械臂第二个动作数据
extern const uint8_t arm_action_count_2; // 机械臂第二个动作的帧数

/**
 * @brief   机械臂动作调用结构体
 */
typedef struct{
    void (*action_func)(char *param);   //动作函数指针
    uint32_t last_run_time;  //记录该动作上一次执行时的系统时间戳
    uint16_t *interval_ms;    //动作执行间隔时间 (单位：毫秒)
    uint8_t  *is_active;      //动作使能开关 (1:运行, 0:挂起休眠)
    uint8_t  *is_waiting;     //动作等待节拍启动标志
    uint8_t  *frame_index;    //当前动作帧索引
}ArmAction;

extern ArmAction arm_action_table[];   // 机械臂动作表，存放所有动作的控制块

extern const uint8_t arm_action_count; // 机械臂动作表中动作的数量

void Arm_Action_1(char *param);    // 机械臂第一个动作的执行函数
void Arm_Action_2(char *param);    // 机械臂第二个动作的执行函数
void Arm_Action_1_Beat(char *param); // 机械臂第一个动作的节拍执行函数
void Arm_Action_2_Beat(char *param); // 机械臂第二个动作的节拍执行函数
void Arm_Action_1_Request(char *param); // 请求机械臂第一个动作
void Arm_Action_2_Request(char *param); // 请求机械臂第二个动作
void Arm_Action_StopAll(void);          // 停止所有机械臂动作

void Arm_Action_Process(void);    // 机械臂动作调度函数

#endif
