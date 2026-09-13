/**
 * @file    Beat_data.c
 * @brief   将从上位机接收的节拍存储，用于机器人随节奏运动
 */

#include "I2S_beat.h"

uint32_t dance_start_time=0;// 跳舞开始时间

BeatData beat_data={0};//唯一实例

/**
 * @brief   获取节拍数据指针
 * @retval  节拍数据指针
 */
BeatData* Get_BeatData(void)
{
    return &beat_data;
}

/**
 * @brief   检查当前进行到哪个节拍，在主循环中调用
 */
void Update_BeatIndex(BeatData* beat_data)
{
    if(dance_start_time!=0)
    {
    uint32_t current_time=HAL_GetTick()-dance_start_time;
    if(current_time>=beat_data->beat_data[beat_data->index])
    {
        beat_data->index++;
        if(beat_data->index>=beat_data->beat_count)
        {
            beat_data->index=0;
        }
    }
}
}
