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
    /*static uint8_t red = 0;
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
    WS2812_Fill(WS2812_BODY, red/4.0f, green/4.0f, blue/4.0f);
    WS2812_Fill(WS2812_SCREEN_1, green/4.0f, blue/4.0f, red/4.0f);
    WS2812_Fill(WS2812_SCREEN_2, blue/4.0f, red/4.0f, green/4.0f);*/
}

//void WS2812_Change(void)
//{
//    static uint8_t animation_frame = 0U;
//    static uint8_t strip_index = 0U;

//    uint8_t phase1;
//    uint8_t phase2;
//    uint8_t wave1;
//    uint8_t wave2;
//    uint8_t radius1;
//    uint8_t radius2;
//    uint8_t row;
//    uint8_t col;
//    uint8_t row_distance;
//    uint8_t col_distance;
//    uint8_t distance;
//    uint8_t trail;
//    uint8_t i;
//    uint16_t led_index;

//    /* 两块灯屏错开半个动画周期 */
//    phase1 = animation_frame;
//    phase2 = (uint8_t)((animation_frame + 28U) % 56U);

//    /* 让波纹从中心扩散到边缘后再收回 */
//    wave1 = (phase1 <= 28U) ? phase1 : (uint8_t)(56U - phase1);
//    wave2 = (phase2 <= 28U) ? phase2 : (uint8_t)(56U - phase2);

//    radius1 = (uint8_t)(2U + wave1);
//    radius2 = (uint8_t)(2U + wave2);

//    WS2812_Fill(WS2812_SCREEN_1, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_SCREEN_2, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_BODY, 0U, 0U, 0U);

//    for(row = 0U; row < WS2812_SCREEN_HEIGHT; row++)
//    {
//        /* 距离屏幕中心的纵向距离，中心位于四颗灯珠之间 */
//        row_distance = (row <= 7U) ?
//                       (uint8_t)(15U - 2U * row) :
//                       (uint8_t)(2U * row - 15U);

//        for(col = 0U; col < WS2812_SCREEN_WIDTH; col++)
//        {
//            col_distance = (col <= 7U) ?
//                           (uint8_t)(15U - 2U * col) :
//                           (uint8_t)(2U * col - 15U);

//            /* 使用曼哈顿距离，形成菱形扩散波纹 */
//            distance = (uint8_t)(row_distance + col_distance);

//            if(distance <= radius1)
//            {
//                trail = (uint8_t)(radius1 - distance);

//                if(trail <= 2U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_1, row, col, 0U, 180U, 255U);
//                }
//                else if(trail <= 7U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_1, row, col, 0U, 35U, 90U);
//                }
//            }

//            if(distance <= radius2)
//            {
//                trail = (uint8_t)(radius2 - distance);

//                if(trail <= 2U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_2, row, col, 255U, 80U, 0U);
//                }
//                else if(trail <= 7U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_2, row, col, 70U, 10U, 0U);
//                }
//            }
//        }
//    }

//    /* 灯带显示移动光点和渐变尾巴 */
//    for(i = 0U; i < 4U; i++)
//    {
//        led_index = (uint16_t)((strip_index + WS2812_BODY_LED_COUNT - i)
//                               % WS2812_BODY_LED_COUNT);

//        if(i == 0U)
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 255U, 255U, 255U);
//        }
//        else if(i == 1U)
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 0U, 160U, 255U);
//        }
//        else if(i == 2U)
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 0U, 70U, 140U);
//        }
//        else
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 0U, 25U, 50U);
//        }
//    }

//    animation_frame++;
//    if(animation_frame >= 56U)
//    {
//        animation_frame = 0U;
//    }

//    strip_index++;
//    if(strip_index >= WS2812_BODY_LED_COUNT)
//    {
//        strip_index = 0U;
//    }
//}

//void WS2812_Change(void)
//{
//    enum
//    {
//        TAIL_LEN = 6U,
//        CYCLE_LEN = 22U
//    };

//    static uint8_t frame = 0U;
//    static uint8_t strip_index = 0U;

//    static const uint8_t start1[WS2812_SCREEN_WIDTH] =
//        {0U, 9U, 3U, 15U, 6U, 12U, 1U, 18U,
//         7U, 14U, 4U, 20U, 10U, 2U, 17U, 5U};

//    static const uint8_t speed1[WS2812_SCREEN_WIDTH] =
//        {1U, 2U, 3U, 1U, 2U, 1U, 3U, 2U,
//         1U, 3U, 2U, 1U, 3U, 2U, 1U, 2U};

//    static const uint8_t start2[WS2812_SCREEN_WIDTH] =
//        {13U, 2U, 18U, 6U, 21U, 8U, 15U, 3U,
//         19U, 5U, 11U, 1U, 16U, 7U, 20U, 4U};

//    static const uint8_t speed2[WS2812_SCREEN_WIDTH] =
//        {2U, 1U, 3U, 2U, 1U, 3U, 2U, 1U,
//         3U, 2U, 1U, 3U, 1U, 2U, 3U, 1U};

//    uint8_t col;
//    uint8_t tail;
//    uint8_t cycle_pos;
//    uint8_t i;
//    int16_t head1;
//    int16_t head2;
//    int16_t row1;
//    int16_t row2;
//    uint16_t led_index;

//    WS2812_Fill(WS2812_SCREEN_1, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_SCREEN_2, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_BODY, 0U, 0U, 0U);

//    for(col = 0U; col < WS2812_SCREEN_WIDTH; col++)
//    {
//        /* 第一块屏的光点从上往下移动 */
//        cycle_pos = (uint8_t)((frame * speed1[col] + start1[col])
//                              % CYCLE_LEN);
//        head1 = (int16_t)cycle_pos - TAIL_LEN;

//        /* 第二块屏的光点从下往上移动 */
//        cycle_pos = (uint8_t)((frame * speed2[col] + start2[col])
//                              % CYCLE_LEN);
//        head2 = (int16_t)(WS2812_SCREEN_HEIGHT - 1U) - cycle_pos;

//        for(tail = 0U; tail < TAIL_LEN; tail++)
//        {
//            row1 = head1 - tail;
//            row2 = head2 + tail;

//            if((row1 >= 0) && (row1 < WS2812_SCREEN_HEIGHT))
//            {
//                if(tail == 0U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_1, (uint8_t)row1, col,
//                        180U, 255U, 200U);
//                }
//                else if(tail < 3U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_1, (uint8_t)row1, col,
//                        0U, 150U, 40U);
//                }
//                else
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_1, (uint8_t)row1, col,
//                        0U, 45U, 12U);
//                }
//            }

//            if((row2 >= 0) && (row2 < WS2812_SCREEN_HEIGHT))
//            {
//                if(tail == 0U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_2, (uint8_t)row2, col,
//                        180U, 220U, 255U);
//                }
//                else if(tail < 3U)
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_2, (uint8_t)row2, col,
//                        30U, 90U, 180U);
//                }
//                else
//                {
//                    WS2812_ScreenSetPixel(
//                        WS2812_SCREEN_2, (uint8_t)row2, col,
//                        8U, 22U, 55U);
//                }
//            }
//        }
//    }

//    /* PB0灯带显示移动光点和尾迹 */
//    for(i = 0U; i < 5U; i++)
//    {
//        led_index = (uint16_t)((strip_index + WS2812_BODY_LED_COUNT - i)
//                               % WS2812_BODY_LED_COUNT);

//        if(i == 0U)
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 255U, 255U, 255U);
//        }
//        else if(i < 3U)
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 0U, 180U, 70U);
//        }
//        else
//        {
//            WS2812_SetLED(WS2812_BODY, led_index, 0U, 45U, 18U);
//        }
//    }

//    frame++;
//    if(frame >= CYCLE_LEN)
//    {
//        frame = 0U;
//    }

//    strip_index++;
//    if(strip_index >= WS2812_BODY_LED_COUNT)
//    {
//        strip_index = 0U;
//    }
//}

//static void WS2812_SetRainbowPixel(WS2812_Device device,
//                                   uint8_t row,
//                                   uint8_t col,
//                                   uint8_t hue,
//                                   uint8_t brightness)
//{
//    uint8_t r;
//    uint8_t g;
//    uint8_t b;

//    /* 将色相转换为彩虹RGB颜色 */
//    if(hue < 85U)
//    {
//        r = (uint8_t)(255U - 3U * hue);
//        g = (uint8_t)(3U * hue);
//        b = 0U;
//    }
//    else if(hue < 170U)
//    {
//        hue = (uint8_t)(hue - 85U);
//        r = 0U;
//        g = (uint8_t)(255U - 3U * hue);
//        b = (uint8_t)(3U * hue);
//    }
//    else
//    {
//        hue = (uint8_t)(hue - 170U);
//        r = (uint8_t)(3U * hue);
//        g = 0U;
//        b = (uint8_t)(255U - 3U * hue);
//    }

//    /* 调整亮度 */
//    r = (uint8_t)((uint16_t)r * brightness / 255U);
//    g = (uint8_t)((uint16_t)g * brightness / 255U);
//    b = (uint8_t)((uint16_t)b * brightness / 255U);

//    WS2812_ScreenSetPixel(device, row, col, r, g, b);
//}

//void WS2812_Change(void)
//{
//    static uint8_t frame = 0U;

//    uint8_t row;
//    uint8_t col;
//    uint8_t wave1;
//    uint8_t wave2;
//    uint8_t pulse1;
//    uint8_t pulse2;
//    uint8_t brightness;
//    uint8_t hue1;
//    uint8_t hue2;
//    uint8_t hue;
//    uint8_t i;

//    WS2812_Fill(WS2812_SCREEN_1, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_SCREEN_2, 0U, 0U, 0U);
//    WS2812_Fill(WS2812_BODY, 0U, 0U, 0U);

//    for(row = 0U; row < WS2812_SCREEN_HEIGHT; row++)
//    {
//        for(col = 0U; col < WS2812_SCREEN_WIDTH; col++)
//        {
//            /* 两组斜向波相互叠加，产生流动的干涉纹理 */
//            wave1 = (uint8_t)(row * 17U + col * 29U + frame * 7U);
//            wave2 = (uint8_t)(row * 13U
//                              + (WS2812_SCREEN_WIDTH - 1U - col) * 23U
//                              + frame * 11U);

//            /* 三角波只用整数运算，生成明暗起伏 */
//            pulse1 = (wave1 < 128U) ?
//                     (uint8_t)(wave1 * 2U) :
//                     (uint8_t)((255U - wave1) * 2U);

//            pulse2 = (wave2 < 128U) ?
//                     (uint8_t)(wave2 * 2U) :
//                     (uint8_t)((255U - wave2) * 2U);

//            brightness = (uint8_t)(45U
//                + ((uint16_t)(pulse1 + pulse2) * 210U / 510U));

//            /* 两块灯屏使用错开的色带和波纹相位 */
//            hue1 = (uint8_t)(row * 9U + col * 13U
//                             + frame * 5U + pulse2 / 2U);
//            hue2 = (uint8_t)(hue1 + 85U + frame * 2U);

//            WS2812_SetRainbowPixel(
//                WS2812_SCREEN_1, row, col, hue1, brightness);

//            WS2812_SetRainbowPixel(
//                WS2812_SCREEN_2, row, col, hue2,
//                (uint8_t)(255U - brightness / 3U));
//        }
//    }

//    /* PB0灯带显示滚动彩虹 */
//    for(i = 0U; i < WS2812_BODY_LED_COUNT; i++)
//    {
//        hue = (uint8_t)(i * 5U + frame * 4U);

//        /* 复用直线LED接口设置灯带颜色 */
//        WS2812_SetLED(WS2812_BODY, i,
//                      (uint8_t)(hue),
//                      (uint8_t)(255U - hue),
//                      (uint8_t)(hue / 2U));
//    }

//    frame++;
//}

/* 简单伪随机数，避免每帧调用 rand() */
//static uint32_t Fire_Random(void)
//{
//    static uint32_t seed = 0x12345678U;

//    seed ^= seed << 13;
//    seed ^= seed >> 17;
//    seed ^= seed << 5;
//    return seed;
//}

///* 热量 0~255 转成黑、红、橙、黄、浅白 */
//static void Fire_Color(uint8_t heat, uint8_t *r,
//                       uint8_t *g, uint8_t *b)
//{
//    if(heat < 85U)
//    {
//        *r = (uint8_t)(heat * 2U);
//        *g = 0U;
//        *b = 0U;
//    }
//    else if(heat < 170U)
//    {
//        *r = 170U;
//        *g = (uint8_t)((heat - 85U) * 2U);
//        *b = 0U;
//    }
//    else
//    {
//        *r = 170U;
//        *g = 170U;
//        *b = (uint8_t)(heat - 170U);
//    }
//}

//static void Fire_UpdateScreen(WS2812_Device screen, uint8_t heat[256])
//{
//    uint8_t row;
//    uint8_t col;
//    uint8_t below2;
//    uint8_t left;
//    uint8_t right;
//    uint8_t cooling;
//    uint8_t r;
//    uint8_t g;
//    uint8_t b;
//    uint16_t sum;
//    uint16_t index;

//    /* 从上往下更新：读取的下一行仍是上一帧的数据 */
//    for(row = 0U; row < 15U; row++)
//    {
//        below2 = (row < 14U) ? (uint8_t)(row + 2U) : 15U;

//        for(col = 0U; col < 16U; col++)
//        {
//            left = (col > 0U) ? (uint8_t)(col - 1U) : col;
//            right = (col < 15U) ? (uint8_t)(col + 1U) : col;

//            sum = (uint16_t)heat[(row + 1U) * 16U + col] * 2U
//                + heat[below2 * 16U + col]
//                + heat[(row + 1U) * 16U + left]
//                + heat[(row + 1U) * 16U + right];

//            cooling = (uint8_t)(Fire_Random() % 12U);
//            sum /= 5U;

//            heat[row * 16U + col] =
//                (sum > cooling) ? (uint8_t)(sum - cooling) : 0U;
//        }
//    }

//    /* 最底行持续产生随机火苗，偶尔留出暗区 */
//    for(col = 0U; col < 16U; col++)
//    {
//        heat[15U * 16U + col] =
//            ((Fire_Random() & 7U) == 0U) ?
//            45U : (uint8_t)(180U + Fire_Random() % 76U);
//    }

//    /* 将热量画到灯屏；row=0 为顶部 */
//    for(row = 0U; row < 16U; row++)
//    {
//        for(col = 0U; col < 16U; col++)
//        {
//            index = (uint16_t)row * 16U + col;
//            Fire_Color(heat[index], &r, &g, &b);
//            WS2812_ScreenSetPixel(screen, row, col, r, g, b);
//        }
//    }
//}

//void WS2812_Change(void)
//{
//    static uint8_t fire1[256] = {0};
//    static uint8_t fire2[256] = {0};
//    uint8_t i;
//    uint8_t ember;

//    Fire_UpdateScreen(WS2812_SCREEN_1, fire1);
//    Fire_UpdateScreen(WS2812_SCREEN_2, fire2);

//    /* 灯带显示随火苗闪烁的余烬 */
//    for(i = 0U; i < WS2812_BODY_LED_COUNT; i++)
//    {
//        ember = fire1[15U * 16U + (i % 16U)];
//        WS2812_SetLED(WS2812_BODY, i,
//                      (uint8_t)(ember / 2U),
//                      (uint8_t)(ember / 10U),
//                      0U);
//    }
//}

//static void WS2812_DrawWu(WS2812_Device screen)
//{
//    static const uint16_t wu[16] =
//    {
//        0x1800,  /* ...##........... */
//        0x1000,  /* ...#............ */
//        0x3FFC,  /* ..############.. */
//        0x5250,  /* .#.#..#..#.#.... */
//        0x1250,  /* ...#..#..#.#.... */
//        0x3FFC,  /* ..############.. */
//        0x1250,  /* ...#..#..#.#.... */
//        0xFFFE,  /* ###############. */
//        0x1000,  /* ...#............ */
//        0x1010,  /* ...#.......#.... */
//        0x2EFC,  /* ..#.###.######.. */
//        0x5690,  /* .#.#.##.#..#.... */
//        0x1CFE,  /* ...###..#######. */
//        0x0810,  /* ....#......#.... */
//        0x7010,  /* .###.......#.... */
//        0x4010   /* .#.........#.... */
//    };

//    uint8_t row;
//    uint8_t col;

//    WS2812_Fill(screen, 0U, 0U, 0U);

//    for(row = 0U; row < 16U; row++)
//    {
//        for(col = 0U; col < 16U; col++)
//        {
//            if((wu[row] & (0x8000U >> col)) != 0U)
//            {
//                WS2812_ScreenSetPixel(screen, row, col,
//                                      20U, 0U, 50U);
//            }
//        }
//    }
//}

//void WS2812_Change(void)
//{
//    static uint8_t drawn = 0U;

//    if(drawn == 0U)
//    {
//        WS2812_DrawWu(WS2812_SCREEN_1);
//        WS2812_DrawWu(WS2812_SCREEN_2);
//        drawn = 1U;
//    }
//}

/* 绘制一行花瓣，超出灯屏的部分自动忽略 */
//static void Lotus_DrawRow(WS2812_Device screen, uint8_t row,
//                          int16_t left, int16_t right,
//                          uint8_t r, uint8_t g, uint8_t b)
//{
//    int16_t col;

//    for(col = left; col <= right; col++)
//    {
//        if((col >= 0) && (col < 16))
//        {
//            WS2812_ScreenSetPixel(screen, row, (uint8_t)col, r, g, b);
//        }
//    }
//}

///* open为花瓣开度：0是花苞，6是盛开 */
//static void Lotus_Draw(WS2812_Device screen, uint8_t open)
//{
//    static const uint8_t center_left[8]  = {7, 7, 6, 6, 6, 6, 7, 7};
//    static const uint8_t center_right[8] = {8, 8, 9, 9, 9, 9, 8, 8};
//    uint8_t row;
//    int16_t offset;
//    int16_t width;

//    WS2812_Fill(screen, 0U, 0U, 0U);

//    /* 外层花瓣：上端向两侧展开，下端始终连接花托 */
//    for(row = 4U; row <= 11U; row++)
//    {
//        offset = (int16_t)(open * (11U - row) / 7U);
//        width = ((row == 4U) || (row == 11U)) ? 0 : 1;

//        Lotus_DrawRow(screen, row, 7 - offset - width,
//                      7 - offset + width, 115U, 12U, 48U);
//        Lotus_DrawRow(screen, row, 8 + offset - width,
//                      8 + offset + width, 115U, 12U, 48U);
//    }

//    /* 内层花瓣比外层展开得少，形成前后层次 */
//    for(row = 5U; row <= 10U; row++)
//    {
//        offset = (int16_t)(open * (11U - row) / 12U);
//        width = (row == 5U) ? 0 : 1;

//        Lotus_DrawRow(screen, row, 7 - offset - width,
//                      7 - offset + width, 205U, 48U, 92U);
//        Lotus_DrawRow(screen, row, 8 + offset - width,
//                      8 + offset + width, 205U, 48U, 92U);
//    }

//    /* 中央花瓣 */
//    for(row = 3U; row <= 10U; row++)
//    {
//        Lotus_DrawRow(screen, row,
//                      center_left[row - 3U], center_right[row - 3U],
//                      235U, 105U, 130U);
//    }

//    /* 盛开时露出花心 */
//    if(open >= 3U)
//    {
//        Lotus_DrawRow(screen, 9U, 7, 8, 240U, 170U, 35U);
//    }

//    /* 花托和花茎 */
//    Lotus_DrawRow(screen, 11U, 4, 7, 15U, 100U, 35U);
//    Lotus_DrawRow(screen, 11U, 8, 11, 15U, 100U, 35U);
//    Lotus_DrawRow(screen, 12U, 5, 10, 8U, 75U, 24U);
//    Lotus_DrawRow(screen, 13U, 7, 8, 8U, 70U, 20U);
//    Lotus_DrawRow(screen, 14U, 7, 8, 8U, 70U, 20U);
//    Lotus_DrawRow(screen, 15U, 7, 8, 8U, 70U, 20U);
//}

///**
// * @brief 莲花开合动画，由任务表每39ms调用一次
// */
//void WS2812_Change(void)
//{
//    static uint8_t frame = 0U;
//    uint8_t open;

//    if(frame <= 48U)
//    {
//        open = (uint8_t)(frame * 6U / 48U); /* 约1.9秒展开 */
//    }
//    else if(frame <= 64U)
//    {
//        open = 6U;                          /* 约0.6秒保持盛开 */
//    }
//    else
//    {
//        open = (uint8_t)((113U - frame) * 6U / 48U);
//    }                                       /* 约1.9秒收拢 */

//    Lotus_Draw(WS2812_SCREEN_1, open);
//    Lotus_Draw(WS2812_SCREEN_2, open);

//    frame++;
//    if(frame >= 114U)
//    {
//        frame = 0U;
//    }
//}
