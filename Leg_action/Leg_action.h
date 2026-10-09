/**
 * @file    Leg_action.h
 * @brief   腿部动作控制函数声明
 */

#ifndef __LEG_ACTION_H
#define __LEG_ACTION_H

#include "main.h"

/***************************************帧动作*******************************************/

extern const uint16_t Init_To_Half_Stand_Data[][16][3]; // 从初始状态到半起立动作数据
extern uint8_t const Init_To_Half_Stand_Count; // 从初始状态到半起立动作的帧数

extern const uint16_t Half_Stand_To_Full_Stand_Data[][16][3]; // 从半起立到完全起立动作数据
extern uint8_t const Half_Stand_To_Full_Stand_Count; // 从半起立到完全起立动作的帧数

extern const uint16_t Inverted_Pendulum_Prepare_Data[][16][3]; // 倒立摆动作准备数据
extern uint8_t const Inverted_Pendulum_Prepare_Count; // 倒立摆

extern const uint16_t Left_Depend_Data[][16][3]; // 左支撑动作数据
extern uint8_t const Left_Depend_Count; // 左支撑动作的帧数

extern const uint16_t Right_Depend_Data[][16][3]; // 右支撑动作数据
extern uint8_t const Right_Depend_Count; // 右支撑动作的帧数

/**
 *@brief   腿部动作调度结构体
 */
typedef struct{
    void (*action_func)(void);   //动作函数指针
    uint32_t last_run_time;  //记录该动作上一次执行时的系统时间戳
    uint16_t *interval_ms;    //动作执行间隔时间 (单位：毫秒)
    uint8_t  *is_active;      //动作使能开关 (1:运行, 0:挂起休眠)
}LegAction;

extern LegAction leg_action_table[];   // 腿部动作表，存放所有动作的控制块

extern const uint8_t leg_action_count; // 腿部动作表中动作的数量

void Init_To_Half_Stand(void);    // 从初始状态到半起立动作的执行函数
void Half_Stand_To_Init(void);    // 从半起立到初始状态动作的执行函数
void Half_Stand_To_Full_Stand(void);    // 从半起立到完全起立动作的执行函数
void Full_Stand_To_Half_Stand(void);    // 从完全起立到半起立动作的执行函数
void Inverted_Pendulum_Prepare(void);    // 倒立摆动作准备的执行函数
void Left_Depend(void);    // 左支撑动作的执行函数
void Right_Depend(void);    // 右支撑动作的执行函数

void Leg_Action_Process(void);    // 腿部动作调度函数

/**********************************************实时步态算法***********************************************/

/**
 * @brief   几种步态枚举
 */
typedef enum{
    TROT=0, // 对角步态
    TURTLE,   // 海龟步态
    BOUND,  // 跳跃步态
    WALK    // 行走步态
}GaitMode;

extern GaitMode gaitmode; // 步态模式枚举变量

/**
 * @brief   步态对应的相位
 */
typedef struct{
    float phase_offset[4]; // 四条腿的相位偏移，单位：弧度
}GaitPhaseOffset;

/**
 * @brief   步态算法参数结构体
 */
typedef struct{
    float phase;//相位，单位：弧度
    float period;//周期，单位：毫秒
    float duty;//支撑相的占空比
    uint16_t original_angle[3];//初始角度，0-1000的舵机值
    float step_length;//步长，单位：厘米
    float step_height;//步高，单位：厘米
    float z_ground;//地面高度，单位：厘米
    int8_t derection;//摆动方向
}GaitParams;

/**
 * @brief   单条腿三个关节角度
 */
typedef struct{
    float q[3];//关节角度，单位：弧度
}LegAngles;

/**
 * @brief   四条腿的舵机id与偏移
 */
typedef struct{
    uint8_t id[3];//舵机id,分别对应髋关节、膝关节、踝关节
    uint16_t offset[3];//舵机偏移,0-1000
}LegServo;

/**
 * @brief   设定的角速度
 */
typedef struct{
    float target_w;//设定的角速度，单位：弧度/秒
    float target_v;//设定的速度，单位：米/秒
    float correct_w;//校正的角速度，单位：弧度/秒
    float correct_v;//校正的速度，单位：米/秒
}SpeedParams;

void Get_Step_Length(float v,float w,float T);
void Realtime_Gait_Process(void);
void Angle_Correct_Process(void);

#endif
