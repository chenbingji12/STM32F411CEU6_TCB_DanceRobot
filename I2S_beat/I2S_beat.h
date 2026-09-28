/**
 * @file    I2S_beat.h
 * @brief   INMP441 麦克风音频采集 + 节拍识别模块
 * @note    适用于 STM32F411CEU6 + INMP441 + HAL 库 + 16kHz 音频
 *          仅需在主循环调用 I2S_Beat_Init() / I2S_Beat_Task() / Get / IsBeat
 */

#ifndef I2S_BEAT_H
#define I2S_BEAT_H

#include "main.h"
//#include <stdint.h>

/* 单个半缓冲的样本数 (16-bit PCM)
 * 16kHz 下: 256 样本 = 16ms, 足够响应节拍 */
#define FRAME_SAMPLES       256U

/* 传给 HAL_I2S_Receive_DMA 的半字数
 * HAL 内部对 24-bit/32-bit 格式会自动 <<1 翻倍:
 *   实际 DMA 传输 = HALF_N * 2 = 1024 半字 = 512 个 32-bit 帧
 *   缓冲区 dma_rx[512] = 1024 半字, 正好匹配
 * 双半缓冲: 每半区 256 个 32-bit 样本 */
#define HALF_N              512U

/**
 * @brief  初始化模块, 启动 I2S2 DMA 循环接收
 * @note   必须在 MX_I2S2_Init() 与 MX_DMA_Init() 之后调用
 */
void I2S_Beat_Init(void);

/**
 * @brief  处理音频半缓冲, 执行节拍检测
 * @note   非阻塞, 没有新数据时立即返回; 应在主循环每次迭代中调用
 */
void I2S_Beat_Task(void);

/**
 * @brief  获取当前估算的 BPM
 * @return BPM 值, 0 表示尚未检测到有效节拍
 */
uint16_t I2S_Beat_GetBPM(void);

/**
 * @brief  查询是否处于节拍触发瞬间 (100ms 内)
 * @return 1 = 节拍刚发生, 0 = 否
 * @note   用于驱动 LED 闪烁等即时反馈
 */
uint8_t I2S_Beat_IsBeat(void);

/**
 * @brief  重置节拍检测器状态 (用于切换曲目或长时间静音后)
 * @note   不会重置 DMA 缓冲, 仅清空检测器状态变量
 */
void I2S_Beat_Reset(void);

/*****************************************从上位机接收的节拍数据*****************************************/
typedef struct {
    uint32_t beat_data[500]; // 存储节拍的数组，最多存储500个节拍
    uint16_t beat_count;     // 节拍数量
    uint16_t index;          // 当前节拍索引
} BeatData;

extern BeatData beat_data; // 节拍数据结构体变量，用于存储从上位机接收的节拍数据

BeatData* Get_BeatData(void);
void Update_BeatIndex(BeatData* beat_data);
void I2S_Beat_Action(void);

#endif
