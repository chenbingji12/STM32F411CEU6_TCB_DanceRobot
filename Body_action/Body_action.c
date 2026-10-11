/**
  * @file    Body_action.c
  * @brief   体动作函数定义
  */

#include "Body_action.h"
#include <stdint.h>

uint8_t arm_action_1_active = 0; // 机械臂动作一是否激活 (1:激活, 0:未激活)
uint8_t arm_action_1_frame_index = 0; // 当前机械臂动作的帧索引
uint16_t Arm_Action_1_Step_Time = 0; // 机械臂第一个动作每帧的执行时间 (毫秒)
uint16_t Arm_Action_1_Bmp_Step_Time = 0; // 机械臂第一个节拍动作每帧的执行时间
uint8_t arm_action_1_bmp_active = 0; // 机械臂动作一节拍模式是否激活
uint8_t arm_action_1_bmp_frame_index = 0; // 机械臂动作一节拍模式帧索引
uint8_t arm_action_1_waiting = 0; // 机械臂动作一是否等待节拍

uint8_t arm_action_2_active = 0; // 机械臂动作二是否激活 (1:激活, 0:未激活)
uint8_t arm_action_2_frame_index = 0; // 当前机械臂动作的帧索引
uint16_t Arm_Action_2_Step_Time = 0; // 机械臂第二个动作每帧的执行时间 (毫秒)
uint16_t Arm_Action_2_Bmp_Step_Time = 0; // 机械臂第二个节拍动作每帧的执行时间
uint8_t arm_action_2_bmp_active = 0; // 机械臂动作二节拍模式是否激活
uint8_t arm_action_2_bmp_frame_index = 0; // 机械臂动作二节拍模式帧索引
uint8_t arm_action_2_waiting = 0; // 机械臂动作二是否等待节拍

/**
 * @brief   将数据发送给舵机
 * @param   array: 舵机数据数组
 * @param   index: 当前帧索引
 */
static void Send_Data_to_Servo(const uint16_t array[][4][3], uint8_t index)
{
    for (uint8_t i = 0; i < 4; i++) {
        Servo_Write((uint8_t)array[index][i][0],array[index][i][1],array[index][i][2]);
    }
}

/**
 * @brief   按统一节拍时间发送机械臂当前帧
 */
static void Send_Data_to_Servo_Beat(const uint16_t array[][4][3], uint8_t index,
                                    uint16_t run_time)
{
    for (uint8_t i = 0; i < 4; i++) {
        Servo_Write((uint8_t)array[index][i][0],array[index][i][1],run_time);
    }
}

/**
 * @brief   计算完成当前帧所需的时间
 */
static uint16_t Max_Frame_Time(const uint16_t array[][4][3], uint8_t frame_index)
{
    uint16_t max_time = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (array[frame_index][i][2] > max_time)
        {
            max_time = array[frame_index][i][2];
        }
    }
    return max_time;
}

/**
 * @brief   执行机械臂动作表
 */
ArmAction arm_action_table[] = {
    {Arm_Action_1, 0, &Arm_Action_1_Step_Time, &arm_action_1_active,
    NULL,NULL},
    {Arm_Action_2, 0, &Arm_Action_2_Step_Time, &arm_action_2_active,
    NULL,NULL},
    {Arm_Action_1_Beat, 0, &Arm_Action_1_Bmp_Step_Time, &arm_action_1_bmp_active,
     &arm_action_1_waiting, &arm_action_1_bmp_frame_index},
    {Arm_Action_2_Beat, 0, &Arm_Action_2_Bmp_Step_Time, &arm_action_2_bmp_active,
     &arm_action_2_waiting, &arm_action_2_bmp_frame_index}
};

const uint8_t arm_action_count = sizeof(arm_action_table) / sizeof(arm_action_table[0]); // 机械臂动作表中动作的数量

/**
 * @brief   机械臂第一个动作的执行函数,单次
 */
void Arm_Action_1(char *param)
{
    arm_action_1_active = 1; // 设置机械臂动作一标志为运行状态
    Send_Data_to_Servo(arm_action_data_1, arm_action_1_frame_index); // 发送当前帧的数据给舵机
    Arm_Action_1_Step_Time = Max_Frame_Time(arm_action_data_1, arm_action_1_frame_index); // 计算当前帧所需的时间
    arm_action_1_frame_index = (arm_action_1_frame_index + 1) % arm_action_count_1; // 更新帧索引
    if (arm_action_1_frame_index == 0) {
        arm_action_1_active = 0; // 动作完成后，设置标志为未激活
    }
}

/**
 * @brief   机械臂第二个动作的执行函数，单次
 */
void Arm_Action_2(char *param)
{
    arm_action_2_active = 1; // 设置机械臂动作二标志为运行状态
    Send_Data_to_Servo(arm_action_data_2, arm_action_2_frame_index); // 发送当前帧的数据给舵机
    Arm_Action_2_Step_Time = Max_Frame_Time(arm_action_data_2, arm_action_2_frame_index); // 计算当前帧所需的时间
    arm_action_2_frame_index = (arm_action_2_frame_index + 1) % arm_action_count_2; // 更新帧索引
    if (arm_action_2_frame_index == 0) {
        arm_action_2_active = 0; // 动作完成后，设置标志为未激活
    }
}

/**
 * @brief   机械臂第一个动作的节拍执行函数
 */
void Arm_Action_1_Beat(char *param)
{
    uint16_t beat_interval=Beat_GetCurrentInterval();
    if(beat_interval==0U)
    {
        return;
    }
    arm_action_1_bmp_active = 1;
    Send_Data_to_Servo_Beat(arm_action_data_1, arm_action_1_bmp_frame_index, beat_interval);
    Arm_Action_1_Bmp_Step_Time = beat_interval;
    arm_action_1_bmp_frame_index = (arm_action_1_bmp_frame_index + 1U) % arm_action_count_1;
    if(arm_action_1_bmp_frame_index==0U)
    {
        arm_action_1_bmp_active = 0;
    }
}

/**
 * @brief   机械臂第二个动作的节拍执行函数
 */
void Arm_Action_2_Beat(char *param)
{
    uint16_t beat_interval=Beat_GetCurrentInterval();
    if(beat_interval==0U)
    {
        return;
    }
    arm_action_2_bmp_active = 1;
    Send_Data_to_Servo_Beat(arm_action_data_2, arm_action_2_bmp_frame_index, beat_interval);
    Arm_Action_2_Bmp_Step_Time = beat_interval;
    arm_action_2_bmp_frame_index = (arm_action_2_bmp_frame_index + 1U) % arm_action_count_2;
    if(arm_action_2_bmp_frame_index==0U)
    {
        arm_action_2_bmp_active = 0;
    }
}

/**
 * @brief  请求动作组在最近的节拍到达时启动
 */
static void Arm_Action_Request(uint8_t action_index)
{
    uint32_t start_time;

    if((action_index>=arm_action_count) ||
       (Beat_GetNextStartTime(&start_time)==0U))
    {
        return;
    }

    *(arm_action_table[action_index].is_active)=0U;
    *(arm_action_table[action_index].is_waiting)=1U;
    *(arm_action_table[action_index].frame_index)=0U;
    arm_action_table[action_index].last_run_time=start_time;
}

void Arm_Action_1_Request(char *param)
{
    Arm_Action_Request(2U);
}

void Arm_Action_2_Request(char *param)
{
    Arm_Action_Request(3U);
}

void Arm_Action_StopAll(void)
{
    for(uint8_t i=0U;i<arm_action_count;i++)
    {
        *(arm_action_table[i].is_active)=0U;
        if(arm_action_table[i].is_waiting!=NULL)
        {
            *(arm_action_table[i].is_waiting)=0U;
        }
    }
}

/**
 * @brief   任务函数，用于执行机械臂动作表中的动作
 */
void Arm_Action_Process(void)
{
    for (uint8_t i = 0; i < arm_action_count; i++)
    {
        /* 没有等待指针的表项是原来的非节拍动作 */
        if(arm_action_table[i].is_waiting==NULL)
        {
            if (*(arm_action_table[i].is_active))
            {
                uint32_t current_time=HAL_GetTick();
                if(current_time-arm_action_table[i].last_run_time >=
                   *(arm_action_table[i].interval_ms))
                {
                    arm_action_table[i].action_func(NULL);
                    arm_action_table[i].last_run_time=current_time;
                }
            }
            continue;
        }

        if(Beat_IsRunning()==0U)
        {
            *(arm_action_table[i].is_active)=0U;
            *(arm_action_table[i].is_waiting)=0U;
            continue;
        }

        if(*(arm_action_table[i].is_waiting)!=0U)
        {
            uint32_t current_time=HAL_GetTick();
            if((int32_t)(current_time-arm_action_table[i].last_run_time)>=0)
            {
                *(arm_action_table[i].is_waiting)=0U;
                arm_action_table[i].action_func(NULL);
                arm_action_table[i].last_run_time=current_time;
            }
            continue;
        }

        if (*(arm_action_table[i].is_active)) // 检查动作是否使能
        {
            uint32_t current_time = HAL_GetTick(); // 获取当前系统时间戳
            if (current_time - arm_action_table[i].last_run_time >= *(arm_action_table[i].interval_ms)) // 检查是否到达执行间隔
            {
                arm_action_table[i].action_func(NULL); // 执行动作函数
                arm_action_table[i].last_run_time = current_time; // 更新上一次执行时间戳
            }
        }
    }
}
