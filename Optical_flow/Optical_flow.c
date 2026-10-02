/**
 * @file    Optical_flow.c
 * @brief   光流模块实现文件
 */
#include "Optical_flow.h"

static OpticalFlow_Data_t processed_data;//处理后的光流数据

float last_x=0.0f;
float last_y=0.0f;//上一次光流数据的x，y坐标值

/**
 * @brief   I2C从设备卡死处理
 */
void I2C1_BusRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // SCL high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // SDA high

//    HAL_Delay(1);

    // 如果 SDA 被从机拉低，手动给 SCL 9 个时钟
    for (uint8_t i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
            break;
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < 200; d++);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < 200; d++);
    }

    // 手动产生 STOP：SDA low -> SCL high -> SDA high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for (volatile uint32_t d = 0; d < 200; d++);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 200; d++);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    for (volatile uint32_t d = 0; d < 200; d++);

    // 复位 I2C 外设时钟
    __HAL_RCC_I2C1_FORCE_RESET();
    for (volatile uint32_t d = 0; d < 200; d++);
    __HAL_RCC_I2C1_RELEASE_RESET();

    MX_I2C1_Init();

}

/**
 * @brief   初始化模块
 */
void OpticalFlow_Init(void)
{
    if(HAL_I2C_IsDeviceReady(&hi2c1, 0x31<<1,3,500) == HAL_OK)
    {
        (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[OpticalFlow] is ready\n");
    }
    else
    {
        (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[OpticalFlow] init failed\n");
        while(1)
        {
            I2C1_BusRecover();
            if(HAL_I2C_IsDeviceReady(&hi2c1, 0x31<<1,3,500) == HAL_OK)
            {
                break;
            }
        }//初始化失败，死循环，等看门狗复位重试
    }
}
/**
 * @brief   获取光流数据
 * @retval  无
 */
static OpticalFlow_Original_Data_t* OpticalFlow_GetData(volatile uint8_t* flow_data_update)
{
    static OpticalFlow_Original_Data_t original_data;
    uint8_t buffer[13];
    if(HAL_I2C_Mem_Read(&hi2c1, 0x31<<1, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 13, 50) == HAL_OK)
    {
        if(buffer[11]==0)
        {
            return NULL;
        }
        original_data.distance[0] = buffer[1];
        original_data.distance[1] = buffer[2];
        original_data.TOF_quality = buffer[3];
        original_data.flow_x_integral[0] = buffer[5];
        original_data.flow_x_integral[1] = buffer[6];
        original_data.flow_y_integral[0] = buffer[7];
        original_data.flow_y_integral[1] = buffer[8];
        original_data.integration_timespan[0] = buffer[9];
        original_data.integration_timespan[1] = buffer[10];
        original_data.valid = buffer[11];
        original_data.version = buffer[12];

//        (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[OpticalFlow] read success\n");

        *flow_data_update=1;//更新光流数据标志位

    }
    else
    {
//        (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[OpticalFlow] read data failed\n");
        return NULL;
    }
    return &original_data;
}

/**
 * @brief   处理原始光流数据，与IMU，TOF融合，供外部使用
 * @retval  处理好的光流数据指针
 */
OpticalFlow_Data_t* OpticalFlow_ProcessData(volatile uint8_t* flow_data_update)
{
    OpticalFlow_Original_Data_t* original_data = OpticalFlow_GetData(flow_data_update);

    if(*flow_data_update==0||original_data==NULL)
    {
        return NULL;
    }

    *flow_data_update=0;

    //距离地面距离单位：mm
    uint16_t distance_to_ground=(uint16_t)(original_data->distance[0]|(original_data->distance[1]<<8));

    //未融合的位移数据，单位：mm
    int16_t flow_x=(int16_t)(original_data->flow_x_integral[0]|(original_data->flow_x_integral[1]<<8));
    int16_t flow_y=(int16_t)(original_data->flow_y_integral[0]|(original_data->flow_y_integral[1]<<8));

    //计算后的累计位移，单位：mm
    float flow_x_integral=(float)(((float)flow_x)/10000.0f*((float)distance_to_ground));
    float flow_y_integral=(float)(((float)flow_y)/10000.0f*((float)distance_to_ground));

    processed_data.distance_x=(flow_x_integral+last_x)/100.0f;//当前x位移=当前x位移+上一次x位移
    processed_data.distance_y=(flow_y_integral+last_y)/100.0f;

    last_x=processed_data.distance_x;//更新上一次的x位移
    last_y=processed_data.distance_y;

    ////上一次发送光流数据到本次发送光流数据的累计时间，微秒
    int16_t integration_timespan=(int16_t)
    (original_data->integration_timespan[0]|(original_data->integration_timespan[1]<<8));

    return &processed_data;
}

/**
 * @brief   复位光流数据
 */
void OpticalFlow_Data_Reset(char* param)
{
    last_x=0.0f;
    last_y=0.0f;

    processed_data.distance_x=0.0f;
    processed_data.distance_y=0.0f;
}
