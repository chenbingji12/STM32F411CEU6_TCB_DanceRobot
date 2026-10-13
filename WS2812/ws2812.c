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

extern uint32_t dance_start_time;

typedef enum
{
    WS2812_ANIM_EYES = 0,
    WS2812_ANIM_BEAT_RING,
    WS2812_ANIM_RADAR,
    WS2812_ANIM_STARS,
    WS2812_ANIM_CHASE,
    WS2812_ANIM_FIREWORKS,
    WS2812_ANIM_TEXT,
    WS2812_ANIM_WAVE_CHARACTER
} WS2812_Animation;

static uint8_t ws2812_animation = WS2812_ANIM_EYES;
static uint16_t ws2812_animation_frame = 0U;
static uint8_t ws2812_wave_character_frame = 0U;

static uint8_t ws2812_ring_age = 255U;
static uint8_t ws2812_radar_direction = 0U;
static uint8_t ws2812_star_flash = 0U;
static uint8_t ws2812_firework_age = 255U;

/* 节拍游标独立于机械臂的 beat_data.index */
static uint16_t ws2812_beat_cursor = 0U;
static uint32_t ws2812_beat_start_seen = 0U;
static uint8_t ws2812_beat_was_running = 0U;

static int WS2812_AbsInt(int value)
{
    return (value < 0) ? -value : value;
}

static void WS2812_PutPixel(WS2812_Device device,
                            int row,
                            int col,
                            uint8_t r,
                            uint8_t g,
                            uint8_t b)
{
    if((row < 0) || (row >= (int)WS2812_SCREEN_HEIGHT) ||
       (col < 0) || (col >= (int)WS2812_SCREEN_WIDTH))
    {
        return;
    }

    WS2812_ScreenSetPixel(device,
                          (uint8_t)row,
                          (uint8_t)col,
                          r, g, b);
}

/* 按上位机存入的毫秒时间戳产生一次性节拍事件 */
static uint8_t WS2812_CheckBeatEvent(void)
{
    BeatData *data;
    uint32_t elapsed;
    uint8_t beat_event = 0U;

    if(Beat_IsRunning() == 0U)
    {
        ws2812_beat_was_running = 0U;
        ws2812_beat_cursor = 0U;
        return 0U;
    }

    data = Get_BeatData();
    if((data == NULL) || (data->beat_count == 0U) ||
       (data->beat_count > 600U))
    {
        return 0U;
    }

    /* beat_start 重新启动时，从第一个时间戳重新计数 */
    if((ws2812_beat_was_running == 0U) ||
       (ws2812_beat_start_seen != dance_start_time))
    {
        ws2812_beat_was_running = 1U;
        ws2812_beat_start_seen = dance_start_time;
        ws2812_beat_cursor = 0U;
    }

    elapsed = HAL_GetTick() - dance_start_time;

    /* 主循环延迟跨过多个时间戳时一次性追上，不让节拍游标落后 */
    while((ws2812_beat_cursor < data->beat_count) &&
          (elapsed >= data->beat_data[ws2812_beat_cursor]))
    {
        beat_event = 1U;
        ws2812_beat_cursor++;
    }

    return beat_event;
}

static void WS2812_DrawOneEye(WS2812_Device device,
                              uint16_t frame,
                              uint8_t beat_event)
{
    uint8_t phase = (uint8_t)(frame % 76U);
    uint8_t eye_height;
    uint8_t look_phase = (uint8_t)((frame / 18U) % 4U);
    int look;
    int row;
    int col;
    int dx;
    int dy;
    int iris_x;

    if(look_phase == 0U)
    {
        look = -2;
    }
    else if(look_phase == 2U)
    {
        look = 2;
    }
    else
    {
        look = 0;
    }

    /* 定期眨眼；节拍到来时两块屏同步闭眼一帧 */
    if((beat_event != 0U) || (phase < 2U))
    {
        for(col = 3; col <= 12; col++)
        {
            WS2812_PutPixel(device, 7, col, 120U, 220U, 255U);
        }
        return;
    }

    eye_height = (phase == 2U) ? 2U : 5U;

    /* 画眼白 */
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            dx = col - 7;
            dy = row - 7;

            if((dx * dx * eye_height * eye_height +
                dy * dy * 49) <= (49 * eye_height * eye_height))
            {
                WS2812_PutPixel(device, row, col, 190U, 235U, 255U);
            }
        }
    }

    /* 画蓝色虹膜、瞳孔和高光 */
    iris_x = 7 + look;

    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            dx = col - iris_x;
            dy = row - 7;

            if((dx * dx + dy * dy) <= 9)
            {
                WS2812_PutPixel(device, row, col, 20U, 100U, 255U);
            }

            if((dx * dx + dy * dy) <= 2)
            {
                WS2812_PutPixel(device, row, col, 0U, 0U, 20U);
            }
        }
    }

    WS2812_PutPixel(device, 6, iris_x - 1, 255U, 255U, 255U);
}

static void WS2812_DrawEyes(uint16_t frame, uint8_t beat_event)
{
    WS2812_Clear(WS2812_SCREEN_1);
    WS2812_Clear(WS2812_SCREEN_2);

    WS2812_DrawOneEye(WS2812_SCREEN_1, frame, beat_event);
    WS2812_DrawOneEye(WS2812_SCREEN_2, frame, beat_event);
}

/* 曼哈顿距离形成菱形光圈，节拍来时从中心重新开始 */
static void WS2812_DrawRing(uint8_t beat_event)
{
    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    uint8_t s;
    uint8_t bright;
    int row;
    int col;
    int distance;
    int radius;
    int difference;

    if(beat_event != 0U)
    {
        ws2812_ring_age = 0U;
    }

    for(s = 0U; s < 2U; s++)
    {
        WS2812_Clear(screens[s]);

        if(ws2812_ring_age > 14U)
        {
            continue;
        }

        bright = (uint8_t)(255U - ws2812_ring_age * 12U);
        radius = 2 + ws2812_ring_age * 2;

        for(row = 0; row < 16; row++)
        {
            for(col = 0; col < 16; col++)
            {
                distance = WS2812_AbsInt(2 * row - 15) +
                           WS2812_AbsInt(2 * col - 15);
                difference = WS2812_AbsInt(distance - radius);

                if(difference <= 1)
                {
                    if(s == 0U)
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        0U, bright / 2U, bright);
                    }
                    else
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        bright, bright / 3U, 0U);
                    }
                }
                else if((distance < radius) &&
                        ((radius - distance) <= 6))
                {
                    if(s == 0U)
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        0U, bright / 8U, bright / 4U);
                    }
                    else
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        bright / 4U, bright / 10U, 0U);
                    }
                }
            }
        }
    }

    if(ws2812_ring_age < 14U)
    {
        ws2812_ring_age++;
    }
    else
    {
        ws2812_ring_age = 255U;
    }
}

/* 用整数 Bresenham 画雷达射线，不需要 sin/cos */
static void WS2812_DrawRadarRay(WS2812_Device device,
                                uint8_t direction,
                                uint8_t r,
                                uint8_t g,
                                uint8_t b)
{
    static const int8_t vector_x[16] =
    {
         0,  3,  5,  7,  7,  7,  5,  3,
         0, -3, -5, -7, -7, -7, -5, -3
    };
    static const int8_t vector_y[16] =
    {
        -7, -7, -5, -3,  0,  3,  5,  7,
         7,  7,  5,  3,  0, -3, -5, -7
    };

    int x0 = 7;
    int y0 = 7;
    int x1 = 7 + vector_x[direction];
    int y1 = 7 + vector_y[direction];
    int dx = WS2812_AbsInt(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -WS2812_AbsInt(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;
    int error2;

    while(1)
    {
        WS2812_PutPixel(device, y0, x0, r, g, b);

        if((x0 == x1) && (y0 == y1))
        {
            break;
        }

        error2 = 2 * error;

        if(error2 >= dy)
        {
            error += dy;
            x0 += sx;
        }

        if(error2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}

static void WS2812_DrawRadar(uint8_t beat_event)
{
    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    uint8_t s;
    uint8_t trail;
    uint8_t direction;
    uint8_t level;

    if(beat_event != 0U)
    {
        ws2812_radar_direction = 0U;
    }

    for(s = 0U; s < 2U; s++)
    {
        WS2812_Clear(screens[s]);

        for(trail = 4U; trail > 0U; trail--)
        {
            direction = (uint8_t)((ws2812_radar_direction +
                                   16U - trail) % 16U);
            level = (uint8_t)(120U / (trail + 1U));

            if(s == 0U)
            {
                WS2812_DrawRadarRay(screens[s], direction,
                                    0U, level, level + 30U);
            }
            else
            {
                WS2812_DrawRadarRay(screens[s], direction,
                                    level, 0U, level + 30U);
            }
        }

        if(s == 0U)
        {
            WS2812_DrawRadarRay(screens[s], ws2812_radar_direction,
                                0U, 210U, 255U);
        }
        else
        {
            WS2812_DrawRadarRay(screens[s], ws2812_radar_direction,
                                255U, 80U, 220U);
        }

        WS2812_PutPixel(screens[s], 7, 7, 255U, 255U, 255U);
    }

    ws2812_radar_direction =
        (uint8_t)((ws2812_radar_direction + 1U) % 16U);
}

static void WS2812_DrawStars(uint16_t frame, uint8_t beat_event)
{
    static const uint8_t stars[11][2] =
    {
        {1U, 2U}, {2U, 11U}, {3U, 6U}, {5U, 14U},
        {7U, 3U}, {8U, 10U}, {10U, 1U}, {11U, 7U},
        {13U, 13U}, {14U, 4U}, {4U, 9U}
    };

    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    uint8_t s;
    uint8_t i;
    uint8_t phase;
    uint8_t triangle;
    uint8_t bright;
    uint8_t col;

    if(beat_event != 0U)
    {
        ws2812_star_flash = 4U;
    }

    for(s = 0U; s < 2U; s++)
    {
        WS2812_Fill(screens[s], 0U, 0U, 6U);

        for(i = 0U; i < 11U; i++)
        {
            phase = (uint8_t)((frame + i * 7U) % 24U);
            triangle = (phase <= 12U) ? phase : (uint8_t)(24U - phase);
            bright = (uint8_t)(20U + triangle * 14U);

            if(ws2812_star_flash != 0U)
            {
                bright = 255U;
            }

            col = (s == 0U) ?
                  stars[i][1] : (uint8_t)(15U - stars[i][1]);

            WS2812_PutPixel(screens[s], stars[i][0], col,
                            bright / 3U, bright, bright);
        }
    }

    if(ws2812_star_flash > 0U)
    {
        ws2812_star_flash--;
    }
}

/* 两块屏共用同一个全局 x 坐标，光点能从左屏接到右屏 */
static void WS2812_DrawChase(uint16_t frame)
{
    static const uint8_t tail_level[6] =
    {
        255U, 150U, 85U, 45U, 22U, 10U
    };

    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    int head = (int)(frame % 48U) - 4;
    int global_x;
    int local_x;
    int row;
    int tail;
    uint8_t s;
    uint8_t level;

    WS2812_Clear(WS2812_SCREEN_1);
    WS2812_Clear(WS2812_SCREEN_2);

    for(tail = 0; tail < 6; tail++)
    {
        global_x = head - tail;
        level = tail_level[tail];

        for(s = 0U; s < 2U; s++)
        {
            local_x = global_x - (s == 0U ? 0 : 16);

            if((local_x < 0) || (local_x >= 16))
            {
                continue;
            }

            for(row = 5; row <= 9; row++)
            {
                WS2812_PutPixel(screens[s], row, local_x,
                                0U,
                                (uint8_t)(level /
                                    (WS2812_AbsInt(row - 7) + 1)),
                                (uint8_t)(level /
                                    (WS2812_AbsInt(row - 7) + 1)));
            }
        }
    }
}

/* 节拍时炸开八条射线；没有节拍时保持熄灭 */
static void WS2812_DrawFireworks(uint8_t beat_event)
{
    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    uint8_t s;
    uint8_t bright;
    int row;
    int col;
    int x;
    int y;
    int distance2;
    int outer2;
    int inner2;
    int is_ray;

    if(beat_event != 0U)
    {
        ws2812_firework_age = 0U;
    }

    for(s = 0U; s < 2U; s++)
    {
        WS2812_Clear(screens[s]);

        if(ws2812_firework_age > 9U)
        {
            continue;
        }

        bright = (uint8_t)(255U - ws2812_firework_age * 18U);
        outer2 = (ws2812_firework_age + 1) *
                 (ws2812_firework_age + 1);
        inner2 = (ws2812_firework_age > 2U) ?
                 (ws2812_firework_age - 2) *
                 (ws2812_firework_age - 2) : 0;

        for(row = 0; row < 16; row++)
        {
            for(col = 0; col < 16; col++)
            {
                x = col - 7;
                y = row - 7;
                distance2 = x * x + y * y;

                is_ray = (x == 0) || (y == 0) ||
                         (WS2812_AbsInt(x) == WS2812_AbsInt(y));

                if(is_ray && (distance2 >= inner2) &&
                   (distance2 <= outer2))
                {
                    if(distance2 <= 2)
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        255U, 255U, 255U);
                    }
                    else if(s == 0U)
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        bright, bright / 2U, 0U);
                    }
                    else
                    {
                        WS2812_PutPixel(screens[s], row, col,
                                        bright, bright / 4U, bright / 2U);
                    }
                }
            }
        }
    }

    if(ws2812_firework_age < 9U)
    {
        ws2812_firework_age++;
    }
    else
    {
        ws2812_firework_age = 255U;
    }
}

/* 3x5 字模放大两倍，在两块屏上滚动显示 HI */
static void WS2812_DrawScrollingText(uint16_t frame,
                                     uint8_t beat_event)
{
    static const uint8_t glyph[2][5] =
    {
        {5U, 7U, 5U, 5U, 5U},  /* H */
        {7U, 2U, 2U, 2U, 7U}   /* I */
    };

    static const WS2812_Device screens[2] =
    {
        WS2812_SCREEN_1, WS2812_SCREEN_2
    };

    int text_left = 32 - (int)(frame % 48U);
    int global_x;
    int text_x;
    int pixel_col;
    int glyph_index;
    int glyph_col;
    uint8_t s;
    uint8_t row;
    uint8_t col;
    uint8_t font_row;
    uint8_t mask;
    uint8_t red = (beat_event != 0U) ? 0U : 255U;
    uint8_t green = (beat_event != 0U) ? 220U : 100U;
    uint8_t blue = 255U;

    WS2812_Clear(WS2812_SCREEN_1);
    WS2812_Clear(WS2812_SCREEN_2);

    for(s = 0U; s < 2U; s++)
    {
        for(row = 3U; row < 13U; row++)
        {
            font_row = (uint8_t)((row - 3U) / 2U);

            for(col = 0U; col < 16U; col++)
            {
                global_x = (s == 0U ? 0 : 16) + col;
                text_x = global_x - text_left;

                if((text_x < 0) || (text_x >= 14))
                {
                    continue;
                }

                pixel_col = text_x / 2;

                if(pixel_col <= 2)
                {
                    glyph_index = 0;
                    glyph_col = pixel_col;
                }
                else if(pixel_col >= 4)
                {
                    glyph_index = 1;
                    glyph_col = pixel_col - 4;
                }
                else
                {
                    continue;
                }

                mask = (uint8_t)(1U << (2 - glyph_col));

                if((glyph[glyph_index][font_row] & mask) != 0U)
                {
                    WS2812_PutPixel(screens[s], row, col,
                                    red, green, blue);
                }
            }
        }
    }
}

/* 绘制单块灯屏上的挥爪角色 */
static void WS2812_DrawWaveCharacterOne(WS2812_Device device,
                                        uint8_t frame)
{
    static const int8_t hand_x[8] = {8, 10, 12, 13, 12, 10, 8, 9};
    static const int8_t hand_y[8] = {7, 5, 6, 7, 8, 9, 8, 6};

    uint8_t pose;
    int row;
    int col;
    int dx;
    int dy;
    int arm_x;
    int arm_y;
    int step;

    pose = (uint8_t)((frame / 3U) % 8U);

    WS2812_Clear(device);

    /* 身体：深色轮廓和白色内部 */
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            dx = col - 7;
            dy = row - 12;

            if((dx * dx * 16 + dy * dy * 25) <= 400)
            {
                if((dx * dx * 9 + dy * dy * 16) <= 144)
                {
                    WS2812_PutPixel(device, row, col,
                                    245U, 238U, 225U);
                }
                else
                {
                    WS2812_PutPixel(device, row, col,
                                    45U, 27U, 19U);
                }
            }

            /* 头部：深棕轮廓和浅棕色 */
            dx = col - 7;
            dy = row - 5;

            if((dx * dx * 36 + dy * dy * 49) <= 1764)
            {
                if((dx * dx * 25 + dy * dy * 36) <= 900)
                {
                    WS2812_PutPixel(device, row, col,
                                    174U, 111U, 66U);
                }
                else
                {
                    WS2812_PutPixel(device, row, col,
                                    45U, 27U, 19U);
                }
            }
        }
    }

    /* 耳朵 */
    WS2812_PutPixel(device, 2, 3, 45U, 27U, 19U);
    WS2812_PutPixel(device, 3, 2, 45U, 27U, 19U);
    WS2812_PutPixel(device, 3, 3, 174U, 111U, 66U);
    WS2812_PutPixel(device, 3, 4, 45U, 27U, 19U);

    WS2812_PutPixel(device, 2, 11, 45U, 27U, 19U);
    WS2812_PutPixel(device, 3, 10, 45U, 27U, 19U);
    WS2812_PutPixel(device, 3, 11, 174U, 111U, 66U);
    WS2812_PutPixel(device, 3, 12, 45U, 27U, 19U);

    /* 蓝色眼睛 */
    WS2812_PutPixel(device, 5, 4, 30U, 120U, 255U);
    WS2812_PutPixel(device, 5, 5, 30U, 120U, 255U);
    WS2812_PutPixel(device, 6, 4, 30U, 120U, 255U);
    WS2812_PutPixel(device, 6, 5, 45U, 27U, 19U);
    WS2812_PutPixel(device, 5, 4, 245U, 245U, 255U);

    WS2812_PutPixel(device, 5, 9, 30U, 120U, 255U);
    WS2812_PutPixel(device, 5, 10, 30U, 120U, 255U);
    WS2812_PutPixel(device, 6, 9, 30U, 120U, 255U);
    WS2812_PutPixel(device, 6, 10, 45U, 27U, 19U);
    WS2812_PutPixel(device, 5, 9, 245U, 245U, 255U);

    /* 白色口鼻区域和嘴 */
    for(col = 6; col <= 8; col++)
    {
        WS2812_PutPixel(device, 7, col, 245U, 238U, 225U);
        WS2812_PutPixel(device, 9, col, 245U, 238U, 225U);
    }

    for(col = 5; col <= 9; col++)
    {
        WS2812_PutPixel(device, 8, col, 245U, 238U, 225U);
    }

    WS2812_PutPixel(device, 7, 7, 45U, 27U, 19U);
    WS2812_PutPixel(device, 8, 7, 45U, 27U, 19U);

    /* 手臂从身体伸出，爪子沿弧线挥动 */
    for(step = 0; step <= 4; step++)
    {
        arm_x = 8 + ((hand_x[pose] - 8) * step) / 4;
        arm_y = 10 + ((hand_y[pose] - 10) * step) / 4;

        WS2812_PutPixel(device, arm_y, arm_x - 1,
                        45U, 27U, 19U);
        WS2812_PutPixel(device, arm_y, arm_x,
                        225U, 165U, 120U);
    }

    /* 爪子轮廓和掌心 */
    for(dy = -1; dy <= 1; dy++)
    {
        for(dx = -1; dx <= 1; dx++)
        {
            WS2812_PutPixel(device,
                            hand_y[pose] + dy,
                            hand_x[pose] + dx,
                            45U, 27U, 19U);
        }
    }

    for(dy = 0; dy <= 1; dy++)
    {
        for(dx = 0; dx <= 1; dx++)
        {
            WS2812_PutPixel(device,
                            hand_y[pose] + dy,
                            hand_x[pose] + dx,
                            225U, 165U, 120U);
        }
    }
}

/* 两块灯屏同步显示；节拍到来时从挥爪起始帧重新播放 */
static void WS2812_DrawWaveCharacter(uint8_t beat_event)
{
    if(beat_event != 0U)
    {
        ws2812_wave_character_frame = 0U;
    }

    WS2812_DrawWaveCharacterOne(WS2812_SCREEN_1,
                                ws2812_wave_character_frame);
    WS2812_DrawWaveCharacterOne(WS2812_SCREEN_2,
                                ws2812_wave_character_frame);

    ws2812_wave_character_frame++;
    if(ws2812_wave_character_frame >= 24U)
    {
        ws2812_wave_character_frame = 0U;
    }
}

void WS2812_Change(void)
{
    uint8_t beat_event = WS2812_CheckBeatEvent();

    switch(ws2812_animation)
    {
        case WS2812_ANIM_EYES:
            WS2812_DrawEyes(ws2812_animation_frame, beat_event);
            break;

        case WS2812_ANIM_BEAT_RING:
            WS2812_DrawRing(beat_event);
            break;

        case WS2812_ANIM_RADAR:
            WS2812_DrawRadar(beat_event);
            break;

        case WS2812_ANIM_STARS:
            WS2812_DrawStars(ws2812_animation_frame, beat_event);
            break;

        case WS2812_ANIM_CHASE:
            WS2812_DrawChase(ws2812_animation_frame);
            break;

        case WS2812_ANIM_FIREWORKS:
            WS2812_DrawFireworks(beat_event);
            break;

        case WS2812_ANIM_TEXT:
            WS2812_DrawScrollingText(ws2812_animation_frame,
                                     beat_event);
            break;

        case WS2812_ANIM_WAVE_CHARACTER:
            WS2812_DrawWaveCharacter(beat_event);
            break;

        default:
            WS2812_Clear(WS2812_SCREEN_1);
            WS2812_Clear(WS2812_SCREEN_2);
            break;
    }

    ws2812_animation_frame++;
    if(ws2812_animation_frame >= 60000U)
    {
        ws2812_animation_frame = 0U;
    }
}

void WS2812_AnimationCmd(char *param)
{
    int animation_id;

    if((param == NULL) ||
       (sscanf(param, "%d", &animation_id) != 1) ||
       (animation_id < 0) ||
       (animation_id > 7))
    {
        return;
    }

    ws2812_animation = (uint8_t)animation_id;
    ws2812_animation_frame = 0U;
    ws2812_wave_character_frame = 0U;
    ws2812_ring_age = 255U;
    ws2812_radar_direction = 0U;
    ws2812_star_flash = 0U;
    ws2812_firework_age = 255U;

    flag.ws2812_change = 1U;
}

/**
 * @brief   ws2812颜色渐变,在task.c中调用，每39ms执行一次
 */
/*void WS2812_Change(void)
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
    WS2812_Fill(WS2812_BODY, red/4.0f, green/4.0f, blue/4.0f);
    WS2812_Fill(WS2812_SCREEN_1, green/4.0f, blue/4.0f, red/4.0f);
    WS2812_Fill(WS2812_SCREEN_2, blue/4.0f, red/4.0f, green/4.0f);
}*/

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
