/**
  ******************************************************************************
  * @file    IMU.c
  * @brief   Yahboom九轴IMU模块串口数据解析实现
  *          协议: 帧头0x7E 0x23, 变长帧, 小端序float, 累加和校验
  *          接收方式: USART2 DMA循环模式 + 空闲中断
  *
  * 欧拉角帧格式 (功能字0x26, 共17字节):
  *   [0]  0x7E  帧头1
  *   [1]  0x23  帧头2
  *   [2]  0x11  帧长度(17)
  *   [3]  0x26  功能字(欧拉角)
  *   [4-7]  Roll  float32 小端序 (弧度)
  *   [8-11] Pitch float32 小端序 (弧度)
  *   [12-15] Yaw  float32 小端序 (弧度)
  *   [16] checksum 校验和(0x7E累加到[15]取低字节)
  *
  * 缩放: 模块直接输出float弧度值, 需转换为角度 (* 180 / PI)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "IMU.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define IMU_RAD_TO_DEG  (57.29577951308232f)  /* 180/PI, 弧度转角度 */
#define IMU_ACCEL_SCALE (16.0f / 32767.0f)    /* 原始加速度转换为g */
#define IMU_G_TO_MPS2   (9.80665f)             /* g转换为m/s^2 */

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *s_huart = NULL;    /* USART句柄 */
static uint8_t s_dma_buf[IMU_DMA_BUF_SIZE];   /* DMA循环接收缓冲区 */
static IMU_Data_t s_imu_data = {0};           /* IMU数据 */

/* 帧解析状态 */
typedef enum {
    PARSE_SEARCH_HDR1 = 0,  /* 搜索帧头第1字节 0x7E */
    PARSE_SEARCH_HDR2,      /* 搜索帧头第2字节 0x23 */
    PARSE_READ_LEN,         /* 读取帧长度 */
    PARSE_READ_BODY,        /* 读取帧体(功能字+数据+校验) */
} ParseState_t;

/* Private function prototypes -----------------------------------------------*/
static void IMU_ParseBuffer(uint8_t *buf, uint16_t len);
static uint8_t IMU_CalcChecksum(uint8_t *buf, uint16_t len);
static void IMU_ParseRawFrame(uint8_t *frame);
static void IMU_ParseEulerFrame(uint8_t *frame);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化IMU模块, 启动USART2的DMA+空闲中断接收
  * @param  huart: USART2句柄指针 (&huart2)
  * @retval 无
  */
void IMU_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&s_imu_data, 0, sizeof(s_imu_data));
    memset(s_dma_buf, 0, sizeof(s_dma_buf));

    /* 启动DMA+空闲中断接收 (与UART1/6保持一致的模式) */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, IMU_DMA_BUF_SIZE);
}

/**
  * @brief  空闲中断回调 (在main.c的HAL_UARTEx_RxEventCallback中调用)
  * @param  Size: 本次接收到的字节数
  * @retval 无
  */
void IMU_RxEventCallback(uint16_t Size)
{
    if (Size > 0) {
        IMU_ParseBuffer(s_dma_buf, Size);
    }
    /* 重新启动DMA接收 (HAL_UARTEx_ReceiveToIdle_DMA在触发后需手动重启) */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf, IMU_DMA_BUF_SIZE);
}

/**
  * @brief  获取IMU数据指针
  * @retval IMU_Data_t指针
  */
IMU_Data_t* IMU_GetData(void)
{
    return &s_imu_data;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  计算校验和: 从buf[0]累加到buf[len-1], 取最低字节
  * @param  buf: 待校验的缓冲区指针
  * @param  len: 校验长度
  * @retval 校验和
  */
static uint8_t IMU_CalcChecksum(uint8_t *buf, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/**
  * @brief  解析原始数据帧 (功能字0x04)
  *         加速度数据为int16_t小端序, 单位为g, 需转换为m/s^2
  * @param  frame: 帧起始指针 (包含帧头和长度)
  * @retval 无
  */
static void IMU_ParseRawFrame(uint8_t *frame)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    /* Accel X: frame[4..5] 小端序int16_t */
    raw_x = (int16_t)((uint16_t)frame[4] |
                      ((uint16_t)frame[5] << 8));

    /* Accel Y: frame[6..7] 小端序int16_t */
    raw_y = (int16_t)((uint16_t)frame[6] |
                      ((uint16_t)frame[7] << 8));

    /* Accel Z: frame[8..9] 小端序int16_t */
    raw_z = (int16_t)((uint16_t)frame[8] |
                      ((uint16_t)frame[9] << 8));

    s_imu_data.acc_x = raw_x * IMU_ACCEL_SCALE * IMU_G_TO_MPS2;
    s_imu_data.acc_y = raw_y * IMU_ACCEL_SCALE * IMU_G_TO_MPS2;
    s_imu_data.acc_z = raw_z * IMU_ACCEL_SCALE * IMU_G_TO_MPS2;

    s_imu_data.acc_updated = 1;
    s_imu_data.acc_update_tick = HAL_GetTick();
    s_imu_data.updated = 1;
}

/**
  * @brief  解析欧拉角帧 (功能字0x26)
  *         数据为float32小端序, 单位弧度, 需转为角度
  * @param  frame: 帧起始指针 (包含帧头和长度)
  * @retval 无
  */
static void IMU_ParseEulerFrame(uint8_t *frame)
{
    union {
        uint8_t b[4];
        float f;
    } conv;

    /* Roll: frame[4..7] 小端序float */
    conv.b[0] = frame[4];
    conv.b[1] = frame[5];
    conv.b[2] = frame[6];
    conv.b[3] = frame[7];
    s_imu_data.roll = conv.f * IMU_RAD_TO_DEG;

    /* Pitch: frame[8..11] 小端序float */
    conv.b[0] = frame[8];
    conv.b[1] = frame[9];
    conv.b[2] = frame[10];
    conv.b[3] = frame[11];
    s_imu_data.pitch = conv.f * IMU_RAD_TO_DEG;

    /* Yaw: frame[12..15] 小端序float */
    conv.b[0] = frame[12];
    conv.b[1] = frame[13];
    conv.b[2] = frame[14];
    conv.b[3] = frame[15];
    s_imu_data.yaw = conv.f * IMU_RAD_TO_DEG;

    s_imu_data.attitude_updated = 1;
    s_imu_data.updated = 1;
}

/**
  * @brief  解析DMA缓冲区中的数据帧
  *         在缓冲区中查找完整帧, 支持一帧或多帧连续解析
  * @param  buf: DMA缓冲区指针
  * @param  len: 缓冲区长度
  * @retval 无
  */
static void IMU_ParseBuffer(uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;

    while (i < len) {
        /* 步骤1: 查找帧头 0x7E 0x23 */
        if (buf[i] != IMU_HDR1) {
            i++;
            continue;
        }
        if (i + 1 >= len) break;  /* 缓冲区剩余不足, 等下次 */
        if (buf[i + 1] != IMU_HDR2) {
            i++;
            continue;
        }

        /* 步骤2: 读取帧长度 */
        if (i + 2 >= len) break;
        uint8_t frame_len = buf[i + 2];

        /* 长度合理性检查 (最小7字节, 最大不超过缓冲区大小) */
        if (frame_len < 7 || (uint16_t)frame_len > IMU_DMA_BUF_SIZE) {
            i += 2;
            continue;
        }

        /* 步骤3: 检查帧是否完整 */
        if (i + frame_len > len) break;  /* 不完整, 等下次 */

        /* 步骤4: 校验和验证 (从帧头到校验和前一位) */
        uint8_t checksum = IMU_CalcChecksum(&buf[i], frame_len - 1);
        if (buf[i + frame_len - 1] != checksum) {
            i += 2;  /* 校验失败, 跳过帧头继续搜索 */
            continue;
        }

        /* 步骤5: 根据功能字分发解析 */
        uint8_t func = buf[i + 3];
        if (func == IMU_FUNC_RAW) {
            IMU_ParseRawFrame(&buf[i]);
        }
        else if (func == IMU_FUNC_EULER) {
            IMU_ParseEulerFrame(&buf[i]);
        }
        /* 其他功能字(0x16四元数/0x32气压)暂不解析 */

        i += frame_len;  /* 跳到下一帧 */
    }
}
