/**
 * @file    ws2812.h
 * @brief   WS2812B灯带PWM+DMA驱动头文件
 * @note    PB0/TIM3_CH3、PB1/TIM3_CH4、PB10/TIM2_CH3分别驱动三路WS2812
 *          每路独立使用颜色数组、DMA缓冲区和刷新标志
 */

#ifndef __WS2812_H
#define __WS2812_H

#include <stdint.h>
#include "main.h"

/*============================================================================
 * 灯带参数 (ARR=124, 800kHz PWM)
 *============================================================================*/

/* 三路LED数量 */
#define WS2812_BODY_LED_COUNT       60U
#define WS2812_SCREEN_LED_COUNT     256U

/* 16*16灯屏参数 */
#define WS2812_SCREEN_WIDTH         16U
#define WS2812_SCREEN_HEIGHT        16U

/* 每帧末尾增加3个LED的全零数据, 产生约90us复位电平 */
#define WS2812_BODY_DMA_BUF_SIZE \
    ((WS2812_BODY_LED_COUNT + 3U) * 24U)
#define WS2812_SCREEN_DMA_BUF_SIZE \
    ((WS2812_SCREEN_LED_COUNT + 3U) * 24U)

/* PWM CCR值: 0码和1码
 * 0码约0.4us -> CCR=40 (40/125 = 32%)
 * 1码约0.8us -> CCR=80 (80/125 = 64%) */
#define WS2812_CCR_0               40U
#define WS2812_CCR_1               80U

/*============================================================================
 * 数据类型
 *============================================================================*/

/* WS2812设备编号 */
typedef enum
{
    WS2812_BODY = 0,       /* PB0, TIM3_CH3, 60颗 */
    WS2812_SCREEN_1,       /* PB1, TIM3_CH4, 256颗 */
    WS2812_SCREEN_2,       /* PB10, TIM2_CH3, 256颗 */
    WS2812_DEVICE_COUNT
} WS2812_Device;

/* RGB颜色结构体 */
typedef struct
{
    uint8_t r;             /* 红色分量0-255 */
    uint8_t g;             /* 绿色分量0-255 */
    uint8_t b;             /* 蓝色分量0-255 */
} WS2812_Color_t;

/*============================================================================
 * 公开API
 *============================================================================*/

/**
 * @brief  初始化三路WS2812并启动PWM+DMA
 * @note   必须在MX_DMA_Init、MX_TIM2_Init和MX_TIM3_Init之后调用
 */
void WS2812_Init(void);

/**
 * @brief  设置指定设备的指定LED颜色
 * @param  device: WS2812设备编号
 * @param  index:  数据链路中的LED索引
 * @param  r:      红色分量0-255
 * @param  g:      绿色分量0-255
 * @param  b:      蓝色分量0-255
 */
void WS2812_SetLED(WS2812_Device device,
                   uint16_t index,
                   uint8_t r,
                   uint8_t g,
                   uint8_t b);

/**
 * @brief  按16*16逻辑坐标设置灯屏像素
 * @param  device: WS2812_SCREEN_1或WS2812_SCREEN_2
 * @param  row:    行号, 0为最上面一行
 * @param  col:    列号, 0为最左边一列
 * @param  r:      红色分量0-255
 * @param  g:      绿色分量0-255
 * @param  b:      蓝色分量0-255
 * @note   灯屏从右上角输入, 第一行从右向左, 使用蛇形索引
 */
void WS2812_ScreenSetPixel(WS2812_Device device,
                            uint8_t row,
                            uint8_t col,
                            uint8_t r,
                            uint8_t g,
                            uint8_t b);

/**
 * @brief  填充指定设备的全部LED
 * @param  device: WS2812设备编号
 * @param  r:      红色分量0-255
 * @param  g:      绿色分量0-255
 * @param  b:      蓝色分量0-255
 */
void WS2812_Fill(WS2812_Device device,
                 uint8_t r,
                 uint8_t g,
                 uint8_t b);

/**
 * @brief  三路设备填充为同一颜色
 */
void WS2812_FillAll(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  清空指定设备
 */
void WS2812_Clear(WS2812_Device device);

/**
 * @brief  清空三路设备
 */
void WS2812_ClearAll(void);

/**
 * @brief  获取指定设备的LED数量
 */
uint16_t WS2812_GetLEDCount(WS2812_Device device);

/*============================================================================
 * 命令处理函数 (供Single_action命令表使用)
 * 签名匹配void (*handler)(char *param)
 *============================================================================*/

void WS2812_TestCmd(char *param);
void WS2812_FillCmd(char *param);
void WS2812_OffCmd(char *param);
void WS2812_Change(void);

#endif /* __WS2812_H */
