/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile Mode g_mode = DEBUG;    //动作模式枚举变量，初始为调试模式

volatile Flag flag = {0};    //动作执行状态标志变量，初始为未执行状态

volatile uint32_t tick = 0;    //时间截，单位毫秒

//volatile float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;    //欧拉角，单位度

volatile uint8_t uart1_rx_buf[UART1_RX_SIZE];   // USART1 接收缓冲区，64 字节
volatile uint8_t uart6_rx_buf[UART6_RX_SIZE];   // USART6 接收缓冲区，100 字节

uint8_t pos_read_id=1;    //位置读取 ID，初始为 1，范围 1-19

volatile float voltage_sum = 0.0f;    //电池电压采样累加值
volatile uint8_t voltage_count = 0;    //电池电压采样计
volatile float battery_voltage = 0.0f;    //电池电压，单位伏特

IMU_Data_t *imu;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
//  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim10);     //启动 TIM10
  HAL_TIM_Base_Start_IT(&htim11);     //启动 TIM11
  HAL_TIM_Base_Start_IT(&htim4);      //启动 TIM4
  HAL_TIM_Base_Start_IT(&htim5);      //启动 TIM5
  
  SEGGER_RTT_Init ();   //J-Link RTT 初始化

HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*) uart1_rx_buf, sizeof(uart1_rx_buf));   //开启 USART1 的 DMA 接收，接收数据存入 uart1_rx_buf
HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*)uart6_rx_buf, sizeof(uart6_rx_buf));   //开启 USART6 的 DMA 接收，接收数据存入 uart6_rx_buf

IMU_Init(&huart2);    //启动IMU模块DMA接收

WS2812_Init();    //启动WS2812B灯带驱动
flag.ws2812_change=1;

//OpticalFlow_Init();    //初始化光流传感器
//flag.flow_active=1;

Servo_pwm_Init(90.0f);    //初始化舵机PWM

HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);//LED 点亮

  printf("Hello World!\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(battery_voltage < 6.45f)   //电池电压低于6.7V，提示用户更换电池
    {
//      (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[Warning] Battery voltage is low: %.2fV, please replace the battery!\n", battery_voltage);
//      (g_mode==DEBUG) && printf("[Warning] Battery voltage is low: %.2fV, please replace the battery!\n", battery_voltage);
      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);   //蜂鸣器响
    }
    else if(battery_voltage >= 6.55f)   //电池电压恢复正常，蜂鸣器不响,迟滞区间0.1V，避免频繁响起
    {
      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);   //蜂鸣器不响
    }

    if(flag.uart6_rx_ready == 1)//来自上位机的指令
    {
      Single_Action((char*)uart6_rx_buf);   //调用动作函数
      (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[DMA] USART6 received, executing action: %s\n", uart6_rx_buf);
//      (g_mode==DEBUG) && printf("uart6_rx_buf: %s\n", uart6_rx_buf);
      memset((char*)uart6_rx_buf, 0, UART6_RX_SIZE);   //清空传入的动作名称字符串，避免重复执行同一动作
      flag.uart6_rx_ready = 0;    //动作执行完成后，将标志位设为未执行状态
      HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*)uart6_rx_buf, sizeof(uart6_rx_buf));  //重新开启DMA接收
    }

    if(flag.uart1_rx_ready == 1)//来自舵机的指令
    {
      (g_mode==DEBUG) && printf("%s",uart1_rx_buf);
//      (g_mode==DEBUG) && printf("uart1_rx_buf: %s\n", uart1_rx_buf);
      memset((char*)uart1_rx_buf, 0, UART1_RX_SIZE);   //清空传入的动作名称字符串，避免重复执行同一动作
      flag.uart1_rx_ready = 0;    //动作执行完成后，将标志位设为未执行状态
      // 半双工：确保在接收模式，重新开启 DMA 接收
      HAL_HalfDuplex_EnableReceiver(&huart1);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)uart1_rx_buf, sizeof(uart1_rx_buf));  //重新开启DMA接收
    }

//    OpticalFlow_Data_t* flow_data = OpticalFlow_ProcessData(&flag.flow_data_update);
//    float x=flow_data->distance_x;
//    float y=flow_data->distance_y;printf("x:%f,y:%f\n",x,y);

    Leg_Action_Process();    //腿部动作处理函数

    Arm_Action_Process();    //机械臂动作处理函数

    Task_Process();    //任务处理函数

    Location_deal_GetIMUData(imu);    //获取当前IMU数据

    BeatData* data_current = Get_BeatData();    //获取当前节拍数据指针
    Update_BeatIndex(data_current);    //更新当前节拍索引

		tick = HAL_GetTick();

//(g_mode==DEBUG) && SEGGER_RTT_printf(0,"Tick:%d\n",HAL_GetTick());
//    HAL_IWDG_Refresh(&hiwdg);   // 喂独立看门狗，防止复位,2048ms
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  外部中断回调函数
  * @param  GPIO_Pin: 指示哪个引脚触发了中断
  * @retval 无
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) // 检测到按键按下
  {
  flag.key_event = 1; // 设置按键事件标志  
  }
}

/**
  * @brief  UART接收事件回调函数
  * @param  huart: UART句柄
  * @param  Size: 接收到的数据大小
  * @retval 无
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->Instance == USART1)
    {
			if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;  // 忽略半满事件,等 IDLE
        flag.uart1_rx_ready=1;    //DMA发送标志位为1，表示数据已接收完毕
    }
    if (huart->Instance == USART6)
    {
			if (huart->RxEventType == HAL_UART_RXEVENT_HT) return;  // 忽略半满事件,等 IDLE
      flag.uart6_rx_ready=1;    //DMA发送标志位为1，表示数据已接收完毕
    }
    if (huart->Instance == USART2)
    {
        IMU_RxEventCallback(Size);// 调用IMU模块的空闲中断回调函数，解析接收到的数据
    }
  }

  /**
  * @brief  字符输出函数
  * @param  c: 要输出的字符
  * @param  f: 文件指针
  * @retval 输出的字符
  */
  int fputc(int c, FILE *f)
  {
    uint8_t ch = (uint8_t)c;

    if (Printf_TxFifoWrite(ch) == 1U)
    {
      Printf_TxStartNext();
    }

    return c;
  }

  /**
  * @brief  UART发送完成回调函数
  * @param  huart: UART句柄
  * @retval 无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 半双工：DMA 发送完成
        uint8_t tx_len;
        if (Fifo_Read(&fifo, fifo_packet, &tx_len)==1)
        {
            // FIFO 还有数据，继续发送（使用实际包长度）
            HAL_UART_Transmit_DMA(&huart1, fifo_packet, tx_len);
        }
        else
        {
            // FIFO 空了，切换到接收模式，等待舵机回传
            HAL_HalfDuplex_EnableReceiver(&huart1);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)uart1_rx_buf, sizeof(uart1_rx_buf));
        }
    }
    else if (huart->Instance == USART6)
    {
        Printf_TxComplete();
        Printf_TxStartNext();
    }
}

/**
  * @brief  UART错误回调函数
  * @param  huart: UART句柄
  * @retval 无
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        Printf_TxAbortCurrent();
        Printf_TxStartNext();
    }
}

/**
  * @brief  定时器中断回调函数
  * @param  htim: TIM句柄
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM10) // 20ms 中断一次
  {
    Update_Servo_pwm_Angle();//更新舵机角度
  }

  if (htim->Instance == TIM11) // 15ms 中断一次
  {
    if(flag.zero_yaw == 1)//如果软件归零标志为已归零
    {
        Location_deal_CalcYawError(imu);//计算当前yaw角度的偏差
    }
  }

  if (htim->Instance == TIM4) // 100ms 中断一次
  {
    HAL_ADC_Start(&hadc1); // 启动 ADC 转换
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
      float voltage = (adc_value / 4009.09f) * 3.3f / VOLTAGE_DIVIDER_RATIO; // 电压分压系数为10K/(10K+16K)=0.384615,电池电压6.4-8.4V
      voltage_sum = voltage_sum + voltage;
      voltage_count++;

      if (voltage_count == 10) { // 每 10 次采样计算一次平均值
        battery_voltage = voltage_sum / voltage_count;
        voltage_sum = 0.0f;
        voltage_count = 0;
      }
    }
    HAL_ADC_Stop(&hadc1); // 停止 ADC 转换
  }

  if (htim->Instance == TIM5) // 10ms 中断一次
  {
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     SEGGER_RTT_printf(0, "\n[ASSERT FAILED] File: %s, Line: %d\n", file, line);
     //打印断言失败的文件名和行号
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
