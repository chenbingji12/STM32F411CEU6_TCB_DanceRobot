/**
 * @file    Leg_action.c
 * @brief   腿部动作控制函数实现
 */

#include "Leg_action.h"

uint8_t Init_To_Half_Stand_Frame_Index = 0; // 当前从初始状态到半起立动作的帧索引
uint16_t Init_To_Half_Stand_Step_Time = 1000; // 从初始状态到半起立动作每帧的执行时间 (毫秒)

uint8_t Half_Stand_To_Init_Frame_Index = 5; // 当前从半起立到初始状态动作的帧索引
uint16_t Half_Stand_To_Init_Step_Time = 1000; // 从半起立到初始状态动作每帧的执行时间 (毫秒)

uint8_t Half_Stand_To_Full_Stand_Frame_Index = 0; // 当前从半起立到完全起立动作的帧索引
uint16_t Half_Stand_To_Full_Stand_Step_Time = 1000; // 从半起立到完全起立动作每帧的执行时间 (毫秒)

uint8_t Full_Stand_To_Half_Stand_Frame_Index = 4; // 当前从完全起立到半起立动作的帧索引
uint16_t Full_Stand_To_Half_Stand_Step_Time = 1000; // 从完全起立到半起立动作每帧的执行时间 (毫秒)

uint8_t Inverted_Pendulum_Prepare_Index = 0; // 当前倒立摆动作准备的帧索引
uint16_t Inverted_Pendulum_Prepare_Step_Time = 1000; // 倒立摆动作准备每帧的执行时间 (毫秒)

uint8_t Left_Depend_Frame_Index = 0; // 当前左支撑动作的帧索引
uint16_t Left_Depend_Step_Time = 1000; // 左支撑动作每帧的执行时间 (毫秒)

uint8_t Right_Depend_Frame_Index = 0; // 当前右支撑动作的帧索引
uint16_t Right_Depend_Step_Time = 1000; // 右支撑动作每帧的执行时间 (毫秒)

/**
 * @brief   计算完成当前帧所需的时间
 * @param   array: 三维数组，包含动作数据
 * @param   frame_index: 当前帧索引
 * @return  所需时间 (毫秒)
 */
static uint16_t Max_Frame_Time(const uint16_t array[][16][3], uint8_t frame_index)
{
    uint16_t max_time = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (array[frame_index][i][2] > max_time)
        {
            max_time = array[frame_index][i][2];
        }
    }
    return max_time;
}

/**
 * @brief   将数据发送给舵机
 * @param   array: 舵机数据数组
 * @param   index: 当前帧索引
 * @param   direction: 数据发送方向,默认为1，表示正向移动；为-1表示反向移动
 */
static void Send_Data_to_Servo(const uint16_t array[][16][3], uint8_t index)
{
    for (uint8_t i = 0; i < 16; i++) {
      Servo_Write((uint8_t)array[index][i][0], array[index][i][1],
                  array[index][i][2]);
    }
}

/**
 * @brief   腿部动作表
 */
LegAction leg_action_table[] = {
    {Init_To_Half_Stand, 0, &Init_To_Half_Stand_Step_Time, (uint8_t*)&flag.init_to_half_stand},
    {Half_Stand_To_Init, 0, &Half_Stand_To_Init_Step_Time, (uint8_t*)&flag.half_stand_to_init},
    {Half_Stand_To_Full_Stand, 0, &Half_Stand_To_Full_Stand_Step_Time, (uint8_t*)&flag.half_stand_to_full_stand},
    {Full_Stand_To_Half_Stand, 0, &Full_Stand_To_Half_Stand_Step_Time, (uint8_t*)&flag.full_stand_to_half_stand},
    {Inverted_Pendulum_Prepare, 0, &Inverted_Pendulum_Prepare_Step_Time, (uint8_t*)&flag.inverted_pendulum_prepare},
    {Left_Depend, 0, &Left_Depend_Step_Time, (uint8_t*)&flag.left_depend},
    {Right_Depend, 0, &Right_Depend_Step_Time, (uint8_t*)&flag.right_depend},
};

const uint8_t leg_action_count = sizeof(leg_action_table) / sizeof(leg_action_table[0]); // 腿部动作表中动作的数量

/**
 * @brief   从初始状态到半起立动作的执行函数
 */
void Init_To_Half_Stand(void)
{
    Send_Data_to_Servo(Init_To_Half_Stand_Data, Init_To_Half_Stand_Frame_Index); // 发送当前帧的数据给舵机
    Init_To_Half_Stand_Step_Time = Max_Frame_Time(Init_To_Half_Stand_Data, Init_To_Half_Stand_Frame_Index); // 计算当前帧所需的时间
    Init_To_Half_Stand_Frame_Index = (Init_To_Half_Stand_Frame_Index + 1) % Init_To_Half_Stand_Count; // 更新帧索引
    if (Init_To_Half_Stand_Frame_Index == 0)
    {
        flag.init_to_half_stand = 0;
    }
}

/**
 * @brief   从半起立到初始状态动作的执行函数
 */
void Half_Stand_To_Init(void)
{
    Send_Data_to_Servo(Init_To_Half_Stand_Data, Half_Stand_To_Init_Frame_Index); // 发送当前帧的数据给舵机
    Half_Stand_To_Init_Step_Time = Max_Frame_Time(Init_To_Half_Stand_Data, Half_Stand_To_Init_Frame_Index); // 计算当前帧所需的时间

    if (Half_Stand_To_Init_Frame_Index == 0)   // 已播到第0帧, 动作结束
    {
        flag.half_stand_to_init = 0;
        Half_Stand_To_Init_Frame_Index = Init_To_Half_Stand_Count - 1;   // 复位到最后一帧
    }
    else
    {
        Half_Stand_To_Init_Frame_Index--;   // 正常递减
    }
}

/**
 * @brief   从半起立到完全起立动作的执行函数
 */
void Half_Stand_To_Full_Stand(void)
{
    Send_Data_to_Servo(Half_Stand_To_Full_Stand_Data, Half_Stand_To_Full_Stand_Frame_Index); // 发送当前帧的数据给舵机
    Half_Stand_To_Full_Stand_Step_Time = Max_Frame_Time(Half_Stand_To_Full_Stand_Data, Half_Stand_To_Full_Stand_Frame_Index); // 计算当前帧所需的时间
    Half_Stand_To_Full_Stand_Frame_Index = (Half_Stand_To_Full_Stand_Frame_Index + 1) % Half_Stand_To_Full_Stand_Count; // 更新帧索引
    if (Half_Stand_To_Full_Stand_Frame_Index == 0)
    {
        flag.half_stand_to_full_stand = 0;
    }
}

/**
 * @brief   从完全起立到半起立动作的执行函数
 */
void Full_Stand_To_Half_Stand(void)
{
    Send_Data_to_Servo(Half_Stand_To_Full_Stand_Data, Full_Stand_To_Half_Stand_Frame_Index); // 发送当前帧的数据给舵机
    Full_Stand_To_Half_Stand_Step_Time = Max_Frame_Time(Half_Stand_To_Full_Stand_Data, Full_Stand_To_Half_Stand_Frame_Index); // 计算当前帧所需的时间

    if (Full_Stand_To_Half_Stand_Frame_Index == 0)   // 已播到第0帧, 动作结束
    {
        flag.full_stand_to_half_stand = 0;
        Full_Stand_To_Half_Stand_Frame_Index = Half_Stand_To_Full_Stand_Count - 1;   // 复位到最后一帧
    }
    else
    {
        Full_Stand_To_Half_Stand_Frame_Index--;   // 正常递减
    }
}

/**
 * @brief   倒立摆动作准备的执行函数
 */
void Inverted_Pendulum_Prepare(void)
{
    Send_Data_to_Servo(Inverted_Pendulum_Prepare_Data, Inverted_Pendulum_Prepare_Index); // 发送当前帧的数据给舵机
    Inverted_Pendulum_Prepare_Step_Time = Max_Frame_Time(Inverted_Pendulum_Prepare_Data, Inverted_Pendulum_Prepare_Index); // 计算当前帧所需的时间
    Inverted_Pendulum_Prepare_Index = (Inverted_Pendulum_Prepare_Index + 1) % Inverted_Pendulum_Prepare_Count; // 更新帧索引
    if (Inverted_Pendulum_Prepare_Index == 0)
    {
        flag.inverted_pendulum_prepare = 0;
        flag.inverted_pendulum_pid_adjust = 1;
    }
}

/**
 * @brief   左支撑动作的执行函数
 */
void Left_Depend(void)
{
    Send_Data_to_Servo(Left_Depend_Data, Left_Depend_Frame_Index); // 发送当前帧的数据给舵机
    Left_Depend_Step_Time = Max_Frame_Time(Left_Depend_Data, Left_Depend_Frame_Index); // 计算当前帧所需的时间
    Left_Depend_Frame_Index = (Left_Depend_Frame_Index + 1) % Left_Depend_Count; // 更新帧索引
    if (Left_Depend_Frame_Index == 0)
    {
        flag.left_depend = 0;
    }
}

/**
 * @brief   右支撑动作的执行函数
 */
void Right_Depend(void)
{
    Send_Data_to_Servo(Right_Depend_Data, Right_Depend_Frame_Index); // 发送当前帧的数据给舵机
    Right_Depend_Step_Time = Max_Frame_Time(Right_Depend_Data, Right_Depend_Frame_Index); // 计算当前帧所需的时间
    Right_Depend_Frame_Index = (Right_Depend_Frame_Index + 1) % Right_Depend_Count; // 更新帧索引
    if (Right_Depend_Frame_Index == 0)
    {
        flag.right_depend = 0;
    }
}

/**
 * @brief  任务调度
 */
void Leg_Action_Process(void)
{
    for (uint8_t i = 0; i < leg_action_count; i++)      // 遍历任务列表
    {
        if (*leg_action_table[i].is_active==1)      // 如果任务已激活
        {
            if (HAL_GetTick() - leg_action_table[i].last_run_time >= *leg_action_table[i].interval_ms)      // 如果时间到
            {
                leg_action_table[i].last_run_time = HAL_GetTick();      // 更新上次运行时间
                leg_action_table[i].action_func();      // 执行任务
            }
        }
    }
}
