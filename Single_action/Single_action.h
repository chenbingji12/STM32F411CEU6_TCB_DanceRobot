/**
  * @file    Single_action.h
  * @brief   单一动作控制函数声明
  */

#ifndef __SINGLE_ACTION_H
#define __SINGLE_ACTION_H

#include "main.h"

/**
  * @brief  定义计算出的任务个数
  */
#define ACTION_COUNT (sizeof(action)/sizeof(action[0]))

/**
  * @brief  定义动作结构体
  */
typedef struct {
    char name[100];      //动作名称
    uint8_t is_prefix;  //是否完全匹配，0为完全匹配，1为前缀匹配
    uint8_t is_circular;  //是否为循环动作标志，1表示循环动作，0表示单次动作
    void (*handler)(char *param);    //指针函数，单个指令
} Action;

/**************导入数据*****************/
extern const uint16_t Reset_Whole_Data[][3];
extern const uint8_t Reset_Whole_Count;

/*******************上位机获取与直接设置舵机指令****************/
void ReadAllPos(char *param);
void Set_Servo_Pos(char *param);
void Read_Servo_Pos(char *param);
void StopAll(char *param);

void Set_pwm_Servo_TargetAngle(char *param);

/*********************上位机控制IMU软件归零指令****************/
void Zero_Yaw(char *param);
void Off_Zero_Yaw(char *param);
void IMU_Calibrate(char *param);
void IMU_Reset(char *param);

/*******************上位机直接设置整体动作****************/
void Reset_Whole(char *param);
void Arm_Action1(char *param);

/********************手柄控制指令****************/
void Gamepad_Control(char *param);

/*******************查找命令***************/
void Single_Action(char *name);

void Receive_Bmp(char *param);

#endif
