/**
 * @file    ws2812.c
 * @brief   WS2812B灯带PWM+DMA驱动实现
 * @note    PB0/TIM3_CH3、PB1/TIM3_CH4、PB10/TIM2_CH3分别驱动三路WS2812
 *          DMA使用CIRCULAR模式, 每路独立循环发送整帧数据
 */

#include "WS2812.h"

#include <string.h>

/*============================================================================
 * 外部句柄
 *============================================================================*/

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim2_ch3_up;

/*============================================================================
 * 内部状态
 *============================================================================*/

static WS2812_Color_t body_colors[WS2812_BODY_LED_COUNT];
static WS2812_Color_t screen1_colors[WS2812_SCREEN_LED_COUNT];
static WS2812_Color_t screen2_colors[WS2812_SCREEN_LED_COUNT];

static uint16_t body_dma_buf[WS2812_BODY_DMA_BUF_SIZE]
    __attribute__((aligned(4)));
static uint16_t screen1_dma_buf[WS2812_SCREEN_DMA_BUF_SIZE]
    __attribute__((aligned(4)));
static uint32_t screen2_dma_buf[WS2812_SCREEN_DMA_BUF_SIZE]
    __attribute__((aligned(4)));

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t active_channel;
    uint16_t led_count;
    uint16_t dma_buf_size;
    WS2812_Color_t *colors;
    void *dma_buf;
    uint8_t dma_word_size;
    volatile uint8_t refresh_pending;
} WS2812_Context;

static WS2812_Context ws2812_context[WS2812_DEVICE_COUNT] =
{
    {
        &htim3,
        TIM_CHANNEL_3,
        HAL_TIM_ACTIVE_CHANNEL_3,
        WS2812_BODY_LED_COUNT,
        WS2812_BODY_DMA_BUF_SIZE,
        body_colors,
        body_dma_buf,
        0U,
        0U
    },
    {
        &htim3,
        TIM_CHANNEL_4,
        HAL_TIM_ACTIVE_CHANNEL_4,
        WS2812_SCREEN_LED_COUNT,
        WS2812_SCREEN_DMA_BUF_SIZE,
        screen1_colors,
        screen1_dma_buf,
        0U,
        0U
    },
    {
        &htim2,
        TIM_CHANNEL_3,
        HAL_TIM_ACTIVE_CHANNEL_3,
        WS2812_SCREEN_LED_COUNT,
        WS2812_SCREEN_DMA_BUF_SIZE,
        screen2_colors,
        screen2_dma_buf,
        1U,
        0U
    }
};

/*============================================================================
 * 私有函数
 *============================================================================*/

/**
 * @brief  获取设备上下文
 */
static WS2812_Context *WS2812_GetContext(WS2812_Device device)
{
    if(device >= WS2812_DEVICE_COUNT)
    {
        return NULL;
    }

    return &ws2812_context[device];
}

/**
 * @brief  将16*16逻辑坐标转换为蛇形数据索引
 * @param  row: 行号, 0为最上面一行
 * @param  col: 列号, 0为最左边一列
 * @return uint16_t 数据链路中的LED索引
 * @note   第0行从右向左, 第1行从左向右
 */
static uint16_t WS2812_ScreenIndex(uint8_t row, uint8_t col)
{
    if((row >= WS2812_SCREEN_HEIGHT) ||
       (col >= WS2812_SCREEN_WIDTH))
    {
        return 0U;
    }

    if((row % 2U) == 0U)
    {
        return (uint16_t)(row * WS2812_SCREEN_WIDTH
                          + (WS2812_SCREEN_WIDTH - 1U - col));
    }

    return (uint16_t)(row * WS2812_SCREEN_WIDTH + col);
}

/**
 * @brief  将单个LED编码为24个CCR值 (GRB序)
 */
static void WS2812_EncodeLED(WS2812_Context *context,
                             uint16_t index)
{
    WS2812_Color_t color = context->colors[index];
    uint8_t grb[3] = {color.g, color.r, color.b};
    uint16_t pos = (uint16_t)(index * 24U);
    uint16_t ccr_value;
    uint8_t byte;
    uint8_t mask;

    for(byte = 0U; byte < 3U; byte++)
    {
        for(mask = 0x80U; mask != 0U; mask >>= 1U)
        {
            ccr_value = (grb[byte] & mask) ? WS2812_CCR_1 : WS2812_CCR_0;

            if(context->dma_word_size != 0U)
            {
                ((uint32_t *)context->dma_buf)[pos] = ccr_value;
            }
            else
            {
                ((uint16_t *)context->dma_buf)[pos] = ccr_value;
            }

            pos++;
        }
    }
}

/**
 * @brief  重建指定设备的DMA缓冲区
 * @note   在对应DMA传输完成回调中调用
 */
static void WS2812_RebuildBuffer(WS2812_Context *context)
{
    uint16_t pos = 0U;
    uint16_t i;

    for(i = 0U; i < context->led_count; i++)
    {
        WS2812_EncodeLED(context, i);
        pos += 24U;
    }

    while(pos < context->dma_buf_size)
    {
        if(context->dma_word_size != 0U)
        {
            ((uint32_t *)context->dma_buf)[pos] = 0U;
        }
        else
        {
            ((uint16_t *)context->dma_buf)[pos] = 0U;
        }

        pos++;
    }
}

/*============================================================================
 * 公开API
 *============================================================================*/

void WS2812_Init(void)
{
    uint8_t device;
    HAL_StatusTypeDef screen2_start_status = HAL_ERROR;
    WS2812_Context *context;

    for(device = 0U; device < WS2812_DEVICE_COUNT; device++)
    {
        context = &ws2812_context[device];

        (void)memset(context->colors, 0,
                     sizeof(WS2812_Color_t) * context->led_count);
        (void)memset(context->dma_buf, 0,
                     (context->dma_word_size != 0U) ?
                     sizeof(uint32_t) * context->dma_buf_size :
                     sizeof(uint16_t) * context->dma_buf_size);

        WS2812_RebuildBuffer(context);
        context->refresh_pending = 0U;

        HAL_StatusTypeDef start_status = HAL_TIM_PWM_Start_DMA(
            context->htim,
            context->channel,
            (uint32_t *)context->dma_buf,
            context->dma_buf_size);

        if(device == WS2812_SCREEN_2)
        {
            screen2_start_status = start_status;
        }

        if(start_status != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 打印PB10屏启动结果和关键寄存器状态 */
    printf("[WS2812] PB10 screen2 start=%d\r\n",
           (int)screen2_start_status);
    printf("[WS2812] PB10 GPIO MODER=%08lX AFR1=%08lX\r\n",
           (unsigned long)GPIOB->MODER,
           (unsigned long)GPIOB->AFR[1]);
    printf("[WS2812] TIM2 CR1=%08lX DIER=%08lX CCER=%08lX ARR=%08lX CCR3=%08lX\r\n",
           (unsigned long)TIM2->CR1,
           (unsigned long)TIM2->DIER,
           (unsigned long)TIM2->CCER,
           (unsigned long)TIM2->ARR,
           (unsigned long)TIM2->CCR3);
    printf("[WS2812] DMA1S1 CR=%08lX NDTR=%08lX PAR=%08lX M0AR=%08lX\r\n",
           (unsigned long)hdma_tim2_ch3_up.Instance->CR,
           (unsigned long)hdma_tim2_ch3_up.Instance->NDTR,
           (unsigned long)hdma_tim2_ch3_up.Instance->PAR,
           (unsigned long)hdma_tim2_ch3_up.Instance->M0AR);
}

void WS2812_SetLED(WS2812_Device device,
                   uint16_t index,
                   uint8_t r,
                   uint8_t g,
                   uint8_t b)
{
    WS2812_Context *context = WS2812_GetContext(device);

    if((context != NULL) && (index < context->led_count))
    {
        context->colors[index].r = r;
        context->colors[index].g = g;
        context->colors[index].b = b;
        context->refresh_pending = 1U;
    }
}

void WS2812_ScreenSetPixel(WS2812_Device device,
                            uint8_t row,
                            uint8_t col,
                            uint8_t r,
                            uint8_t g,
                            uint8_t b)
{
    if((device != WS2812_SCREEN_1) &&
       (device != WS2812_SCREEN_2))
    {
        return;
    }

    if((row >= WS2812_SCREEN_HEIGHT) ||
       (col >= WS2812_SCREEN_WIDTH))
    {
        return;
    }

    WS2812_SetLED(device,
                  WS2812_ScreenIndex(row, col),
                  r,
                  g,
                  b);
}

void WS2812_Fill(WS2812_Device device,
                 uint8_t r,
                 uint8_t g,
                 uint8_t b)
{
    WS2812_Context *context = WS2812_GetContext(device);
    WS2812_Color_t color;
    uint16_t i;

    if(context == NULL)
    {
        return;
    }

    color.r = r;
    color.g = g;
    color.b = b;

    for(i = 0U; i < context->led_count; i++)
    {
        context->colors[i] = color;
    }

    context->refresh_pending = 1U;
}

void WS2812_FillAll(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t device;

    for(device = 0U; device < WS2812_DEVICE_COUNT; device++)
    {
        WS2812_Fill((WS2812_Device)device, r, g, b);
    }
}

void WS2812_Clear(WS2812_Device device)
{
    WS2812_Fill(device, 0U, 0U, 0U);
}

void WS2812_ClearAll(void)
{
    uint8_t device;

    for(device = 0U; device < WS2812_DEVICE_COUNT; device++)
    {
        WS2812_Clear((WS2812_Device)device);
    }
}

uint16_t WS2812_GetLEDCount(WS2812_Device device)
{
    WS2812_Context *context = WS2812_GetContext(device);

    if(context == NULL)
    {
        return 0U;
    }

    return context->led_count;
}

/*============================================================================
 * HAL DMA全传输完成回调
 *============================================================================*/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t device;
    WS2812_Context *context;

    for(device = 0U; device < WS2812_DEVICE_COUNT; device++)
    {
        context = &ws2812_context[device];

        if((htim == context->htim) &&
           (htim->Channel == context->active_channel))
        {
            if(context->refresh_pending != 0U)
            {
                WS2812_RebuildBuffer(context);
                context->refresh_pending = 0U;
            }

            break;
        }
    }
}

/*============================================================================
 * 命令处理函数
 *============================================================================*/

void WS2812_TestCmd(char *param)
{
    (void)param;
    flag.ws2812_change = 1U;   //WS2812B颜色渐变任务激活
}

void WS2812_FillCmd(char *param)
{
    int r = 0;
    int g = 0;
    int b = 0;

    if(sscanf(param, "%d %d %d", &r, &g, &b) == 3)
    {
        WS2812_FillAll((uint8_t)r, (uint8_t)g, (uint8_t)b);
    }
}

void WS2812_OffCmd(char *param)
{
    (void)param;
    flag.ws2812_change = 0U;   //WS2812B颜色渐变任务挂起
    WS2812_ClearAll();
}

/**
 * @brief   ws2812颜色渐变,在task.c中调用，每39ms执行一次
 */
void WS2812_Change(void)
{
    static uint8_t red = 0;
    static uint8_t green = 0;
    static uint8_t blue = 0;

    static uint8_t red_direction = 1;   // 红色分量变化方向，1表示增加，0表示减少
    static uint8_t green_direction = 1; // 绿色分量变化方向，1表示增加，0表示减少
    static uint8_t blue_direction = 1;  // 蓝色分量变化方向，1表示增加，0表示减少

    if(red>=250)
    {
      red_direction=0;   // 红色分量达到最大值，改变方向为减少
    }
    else if(red<=5)
    {
      red_direction=1;   // 红色分量达到最小值，改变方向为增加
    }
    if(red_direction==1)
    {
      red++;
    }
    else if(red_direction==0)
    {
      red--;
    }
    if(green>=250)
    {
      green_direction=0;   // 绿色分量达到最大值，改变方向为减少
    }
    else if(green<=5)
    {
      green_direction=1;   // 绿色分量达到最小值，改变方向为增加
    }
    if(green_direction==1)
    {
      green=green+2;
    }
    else if(green_direction==0)
    {
      green=green-2;
    }
    if(blue>=250)
    {
      blue_direction=0;   // 蓝色分量达到最大值，改变方向为减少
    }
    else if(blue<=5)
    {
      blue_direction=1;   // 蓝色分量达到最小值，改变方向为增加
    }
    if(blue_direction==1)
    {
      blue=blue+3;
    }
    else if(blue_direction==0)
    {
      blue=blue-3;
    }
    WS2812_Fill(WS2812_BODY, red, green, blue);
    WS2812_Fill(WS2812_SCREEN_1, green/4.0f, blue/4.0f, red/4.0f);
    WS2812_Fill(WS2812_SCREEN_2, blue/4.0f, red/4.0f, green/4.0f);
}
