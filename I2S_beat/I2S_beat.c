/**
 * @file    I2S_beat.c
 * @brief   INMP441 音频采集 + 节拍识别算法实现
 * @note    能量包络法 + 自适应阈值 + 滑动窗口 BPM 平均
 */

#include "I2S_beat.h"
#include "i2s.h"     /* hi2s2 句柄 (CubeMX 生成) */
#include <string.h>

/*============================================================================
 * 节拍检测算法参数 (可调, 针对 16kHz 优化)
 *============================================================================*/

/* 包络历史窗口大小 (帧数)
 * 16kHz / 256 样本/帧 = 62.5 帧/秒
 * 43 帧 ≈ 0.69 秒, 覆盖 1-2 个 90 BPM 节拍周期 */
#define ENV_HISTORY_SIZE    43U

/* 包络跟随器系数: 快攻击捕捉节拍, 慢释放平滑 */
#define ENV_ATTACK          0.5f
#define ENV_RELEASE         0.05f

/* 自适应阈值系数: threshold = avg * MULT + OFFSET */
#define THRESH_MULT         1.5f
#define THRESH_OFFSET       500.0f

/* 去抖与合法 interval 范围 (毫秒) */
#define DEBOUNCE_MS         200U     /* 200ms = 最快 300 BPM */
#define INTERVAL_MIN_MS     300U     /* 200 BPM 上限 */
#define INTERVAL_MAX_MS     2000U    /* 30 BPM 下限 */

/* BPM 滑动窗口大小 (最近 8 个 inter-beat interval) */
#define BPM_WINDOW          8U

/* 节拍标志自动清除时间 (毫秒) */
#define BEAT_FLAG_TIMEOUT   100U

/*============================================================================
 * 内部缓冲区 (全部 static, 封装)
 *============================================================================*/

/* DMA 接收缓冲: 32-bit 字, 24-bit 数据左对齐在 bit[31:8]
 * 4 字节对齐以匹配 DMA 突发传输要求 */
static uint32_t dma_rx[2U * FRAME_SAMPLES]
    __attribute__((aligned(4)));

/* 转换后的 16-bit PCM 双缓冲 */
static int16_t  pcm[2U][FRAME_SAMPLES];

/* 半缓冲就绪标志 (bit0 = 前半就绪, bit1 = 后半就绪)
 * volatile 关键字确保主循环与 ISR 之间的可见性 */
static volatile uint8_t ready_flag = 0U;

/*============================================================================
 * 节拍检测内部状态
 *============================================================================*/

static float    envelope_hist[ENV_HISTORY_SIZE];
static uint32_t envelope_idx     = 0U;
static float    current_envelope  = 0.0f;
static float    current_threshold = 1000.0f;

static uint32_t last_beat_tick_ms = 0U;
static uint32_t interval_records[BPM_WINDOW];
static uint32_t interval_idx     = 0U;
static uint16_t current_bpm      = 0U;
static uint8_t  beat_flag        = 0U;

/*============================================================================
 * 私有函数声明
 *============================================================================*/

static inline int16_t Convert24to16(uint32_t raw32);
static void ProcessHalfBuffer(const uint32_t *raw, int16_t *out, uint32_t count);
static float ComputeAverage(const float *arr, uint32_t n);
static uint16_t ComputeBPM(void);
static void BeatAlgorithm(const int16_t *samples, uint32_t count);

/*============================================================================
 * 私有函数实现
 *============================================================================*/

/**
 * @brief  将 24-bit (位于 32-bit 字高 24 位) 转换为 16-bit PCM
 * @param  raw32  从 DMA 读出的 32-bit 原始数据
 * @return 16-bit 有符号 PCM 样本
 * @note   INMP441 数据格式: bit[31] = 符号位, bit[30:8] = 23-bit 数据
 *         (int32_t) 强转会自动符号扩展
 *         两次 >> 8 分别取 24-bit 和 16-bit
 */
static inline int16_t Convert24to16(uint32_t raw32)
{
    /* DMA 半字传输: 低16位 = 帧的高16位 = 音频[23:8], 即24-bit音频的高16位 */
    return (int16_t)(raw32 & 0xFFFF);
}

/**
 * @brief  处理一个半缓冲 (DMA 原始数据 -> PCM)
 * @param  raw   指向 DMA 原始数据缓冲
 * @param  out   指向输出 PCM 缓冲
 * @param  count 样本数
 */
static void ProcessHalfBuffer(const uint32_t *raw, int16_t *out, uint32_t count)
{
    for (uint32_t i = 0U; i < count; i++) {
        out[i] = Convert24to16(raw[i]);
    }
}

/**
 * @brief  计算浮点数组平均值
 */
static float ComputeAverage(const float *arr, uint32_t n)
{
    float sum = 0.0f;
    for (uint32_t i = 0U; i < n; i++) {
        sum += arr[i];
    }
    return sum / (float)n;
}

/**
 * @brief  从 inter-beat intervals 滑动窗口计算 BPM
 * @return BPM 值 (基于有效 interval 平均), 0 表示无数据
 */
static uint16_t ComputeBPM(void)
{
    uint32_t sum   = 0U;
    uint32_t count = 0U;

    for (uint32_t i = 0U; i < BPM_WINDOW; i++) {
        if (interval_records[i] != 0U) {
            sum += interval_records[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0U;
    }

    /* 平均间隔 (ms) -> BPM = 60000 / 平均间隔 */
    return (uint16_t)(60000U * count / sum);
}

/**
 * @brief  节拍检测算法 (能量包络法)
 * @param  samples PCM 样本指针
 * @param  count   样本数
 */
static void BeatAlgorithm(const int16_t *samples, uint32_t count)
{
    if ((samples == NULL) || (count == 0U)) {
        return;
    }

    /*------------------------------------------------------------------------
     * Step 1: 计算瞬时能量 (归一化平方和, 只取左声道)
     *------------------------------------------------------------------------*/
    float energy = 0.0f;
    uint32_t valid_count = 0U;
    for (uint32_t i = 0U; i < count; i++) {   /* 跳过空声道 (值为0的样本) */
        int16_t s_raw = samples[i];
        if (s_raw == 0) continue;             /* INMP441 单声道, 另一声道为 0 */
        float s = (float)s_raw;
        energy += s * s;
        valid_count++;
    }
    energy /= (float)valid_count;

    /*------------------------------------------------------------------------
     * Step 2: 非对称包络跟随 (快攻击 / 慢释放)
     *------------------------------------------------------------------------*/
    if (energy > current_envelope) {
        current_envelope = current_envelope * (1.0f - ENV_ATTACK)
                         + energy * ENV_ATTACK;
    } else {
        current_envelope = current_envelope * (1.0f - ENV_RELEASE)
                         + energy * ENV_RELEASE;
    }

    /*------------------------------------------------------------------------
     * Step 3: 更新历史窗口 (环形)
     *------------------------------------------------------------------------*/
    envelope_hist[envelope_idx] = current_envelope;
    envelope_idx = (envelope_idx + 1U) % ENV_HISTORY_SIZE;

    /*------------------------------------------------------------------------
     * Step 4: 计算自适应阈值
     *------------------------------------------------------------------------*/
    float avg = ComputeAverage(envelope_hist, ENV_HISTORY_SIZE);
    current_threshold = avg * THRESH_MULT + THRESH_OFFSET;

    /*------------------------------------------------------------------------
     * Step 5: 峰值检测 + 去抖
     *------------------------------------------------------------------------*/
    uint32_t now_ms = HAL_GetTick();

    if ((current_envelope > current_threshold) &&
        ((now_ms - last_beat_tick_ms) > DEBOUNCE_MS))
    {
        if (last_beat_tick_ms > 0U) {
            uint32_t interval_ms = now_ms - last_beat_tick_ms;

            if ((interval_ms >= INTERVAL_MIN_MS) &&
                (interval_ms <= INTERVAL_MAX_MS))
            {
                /* 环形记录合法的 inter-beat interval */
                interval_records[interval_idx] = interval_ms;
                interval_idx = (interval_idx + 1U) % BPM_WINDOW;

                /* 重新计算 BPM */
                current_bpm = ComputeBPM();
            }
        }
        last_beat_tick_ms = now_ms;
        beat_flag = 1U;
    }

    /*------------------------------------------------------------------------
     * Step 6: 100ms 后自动清除节拍标志
     *------------------------------------------------------------------------*/
    if ((beat_flag != 0U) &&
        ((HAL_GetTick() - last_beat_tick_ms) > BEAT_FLAG_TIMEOUT))
    {
        beat_flag = 0U;
    }
}

/*============================================================================
 * 公开 API 实现
 *============================================================================*/

/**
 * @brief  初始化模块, 启动 I2S2 DMA 循环接收
 */
void I2S_Beat_Init(void)
{
    /* 清零所有缓冲区 */
    (void)memset(dma_rx,           0, sizeof(dma_rx));
    (void)memset(pcm,              0, sizeof(pcm));
    (void)memset(envelope_hist,    0, sizeof(envelope_hist));
    (void)memset(interval_records, 0, sizeof(interval_records));

    /* 状态变量复位 */
    ready_flag         = 0U;
    envelope_idx       = 0U;
    current_envelope   = 0.0f;
    current_threshold  = 1000.0f;
    last_beat_tick_ms  = 0U;
    interval_idx       = 0U;
    current_bpm        = 0U;
    beat_flag          = 0U;

    /* 启动 I2S2 DMA 循环接收
     * 参数: hi2s2 (CubeMX 生成), dma_rx 缓冲区, HALF_N 半字数 */
    if (HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)dma_rx, HALF_N) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  主循环处理函数
 * @note   检查 ready_flag, 对就绪的半缓冲执行节拍检测, 然后释放
 *         非阻塞, 没有新数据时立即返回
 */
void I2S_Beat_Task(void)
{
    /* 前半缓冲就绪 */
    if ((ready_flag & 0x01U) != 0U) {
        BeatAlgorithm(pcm[0], FRAME_SAMPLES);
        __disable_irq();  /* 临界区保护 ready_flag */
        ready_flag &= (uint8_t)~0x01U;
        __enable_irq();
    }

    /* 后半缓冲就绪 */
    if ((ready_flag & 0x02U) != 0U) {
        BeatAlgorithm(pcm[1], FRAME_SAMPLES);
        __disable_irq();  /* 临界区保护 ready_flag */
        ready_flag &= (uint8_t)~0x02U;
        __enable_irq();
    }
}

/**
 * @brief  获取当前估算的 BPM
 */
uint16_t I2S_Beat_GetBPM(void)
{
    return current_bpm;
}

/**
 * @brief  查询是否处于节拍触发瞬间
 */
uint8_t I2S_Beat_IsBeat(void)
{
    return beat_flag;
}

/**
 * @brief  重置节拍检测器状态
 * @note   不重置 DMA 缓冲与 ready_flag, 避免与正在进行的 DMA 传输冲突
 */
void I2S_Beat_Reset(void)
{
    (void)memset(envelope_hist,    0, sizeof(envelope_hist));
    (void)memset(interval_records, 0, sizeof(interval_records));

    envelope_idx      = 0U;
    current_envelope  = 0.0f;
    current_threshold = 1000.0f;
    last_beat_tick_ms = 0U;
    interval_idx      = 0U;
    current_bpm       = 0U;
    beat_flag         = 0U;
}

/*============================================================================
 * HAL 中断回调 (覆盖 HAL 库的 __weak 默认实现)
 *============================================================================*/

/**
 * @brief  DMA 半传输完成回调 (ISR 上下文)
 * @note   前半 256 个样本已填入 dma_rx, 立即转换并标记就绪
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) {
        ProcessHalfBuffer(&dma_rx[0], pcm[0], FRAME_SAMPLES);
        ready_flag |= 0x01U;
    }
}

/**
 * @brief  DMA 全传输完成回调 (ISR 上下文)
 * @note   后半 256 个样本已填入 dma_rx, 立即转换并标记就绪
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI2) {
        ProcessHalfBuffer(&dma_rx[FRAME_SAMPLES], pcm[1], FRAME_SAMPLES);
        ready_flag |= 0x02U;
    }
}

/**
  * @brief  处理节拍动作任务,在task.c中当flag.beat_active为1时,11ms执行一次
  * @retval 无
  */
void I2S_Beat_Action(void)
{
  static uint8_t last_beat = 0;   //记录上一次节拍状态
  if(I2S_Beat_IsBeat() && !last_beat)   //检测到节拍触发瞬间
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  last_beat = I2S_Beat_IsBeat();   //更新上一次节拍状态
}
