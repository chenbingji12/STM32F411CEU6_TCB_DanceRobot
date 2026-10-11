/**
 * @file    Beat_data.c
 * @brief   将从上位机接收的节拍存储，用于机器人随节奏运动
 */

#include "I2S_beat.h"

uint32_t dance_start_time=0;// 跳舞开始时间
static uint8_t beat_running=0U;// 节拍播放状态

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
 * @brief  开始按时间戳播放节拍
 */
void Beat_Start(void)
{
    if(beat_data.beat_count<2U)
    {
        dance_start_time=0U;
        beat_data.index=0U;
        beat_running=0U;
        return;
    }

    dance_start_time=HAL_GetTick();
    beat_data.index=0U;
    beat_running=1U;
}

/**
 * @brief  获取节拍播放状态
 */
uint8_t Beat_IsRunning(void)
{
    return beat_running;
}

/**
 * @brief  获取当前节拍区间的机械臂周期
 */
uint16_t Beat_GetCurrentInterval(void)
{
    uint32_t interval;
    uint16_t index=beat_data.index;

    if((beat_running==0U) || (beat_data.beat_count<2U) ||
       (index+1U>=beat_data.beat_count))
    {
        return 0U;
    }

    if(beat_data.beat_data[index+1U]<=beat_data.beat_data[index])
    {
        return 1U;
    }

    interval=beat_data.beat_data[index+1U]-beat_data.beat_data[index];
    if((interval<BEAT_SHORT_INTERVAL_MS) &&
       (index+2U<beat_data.beat_count) &&
       (beat_data.beat_data[index+2U]>beat_data.beat_data[index]))
    {
        interval=beat_data.beat_data[index+2U]-beat_data.beat_data[index];
    }

    if(interval==0U)
    {
        interval=1U;
    }
    if(interval>0xFFFFU)
    {
        interval=0xFFFFU;
    }
    return (uint16_t)interval;
}

/**
 * @brief  获取当前时刻之后最近的节拍时间
 * @param  start_time: 输出绝对系统时间戳
 * @retval 1=成功, 0=没有可等待的节拍
 */
uint8_t Beat_GetNextStartTime(uint32_t *start_time)
{
    uint32_t elapsed_time;
    uint16_t index;

    if((start_time==NULL) || (beat_running==0U) ||
       (beat_data.beat_count==0U))
    {
        return 0U;
    }

    elapsed_time=HAL_GetTick()-dance_start_time;
    index=beat_data.index;
    while((index<beat_data.beat_count) &&
          (elapsed_time>=beat_data.beat_data[index]))
    {
        index++;
    }

    if(index>=beat_data.beat_count)
    {
        return 0U;
    }

    *start_time=dance_start_time+beat_data.beat_data[index];
    return 1U;
}

/**
 * @brief   检查当前进行到哪个节拍，在主循环中调用
 */
void Update_BeatIndex(BeatData* beat_data)
{
    uint32_t current_time;

    if((beat_data==NULL) || (beat_running==0U) ||
       (beat_data->beat_count<2U) ||
       (beat_data->index>=beat_data->beat_count))
    {
        return;
    }

    current_time=HAL_GetTick()-dance_start_time;
    while((beat_data->index+1U<beat_data->beat_count) &&
          (current_time>=beat_data->beat_data[beat_data->index+1U]))
    {
        beat_data->index++;
    }

    if((beat_data->index+1U>=beat_data->beat_count) &&
       (current_time>=beat_data->beat_data[beat_data->index]))
    {
        beat_running=0U;
        dance_start_time=0U;
    }
}
