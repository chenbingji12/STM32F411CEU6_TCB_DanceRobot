/**
 * @file    FIFO.c
 * @brief   FIFO队列操作函数实现
 */

#include "FIFO.h"
#include "usart.h"

#define PRINTF_TX_FIFO_SIZE   512U     //定义printf字符FIFO环形缓冲区大小
#define PRINTF_DMA_CHUNK_SIZE 64U      //定义printf每次DMA发送的最大字节数

fifo_t fifo = {0};               // 全局 FIFO
uint8_t fifo_packet[10]={0};        //fifo缓冲区

static uint8_t printf_tx_fifo[PRINTF_TX_FIFO_SIZE]={0};       //printf字符FIFO缓冲区
static uint8_t printf_tx_dma_buf[PRINTF_DMA_CHUNK_SIZE]={0};  //printf DMA发送缓冲区
static volatile uint16_t printf_tx_head = 0U;                 //printf FIFO写指针
static volatile uint16_t printf_tx_tail = 0U;                 //printf FIFO读指针
static volatile uint16_t printf_tx_active_len = 0U;           //printf当前DMA发送长度
static volatile uint8_t printf_tx_dma_busy = 0U;              //printf DMA发送忙标志

/**
  * @brief  启动printf下一包DMA发送
  * @param  无
  * @retval 无
  */
void Printf_TxStartNext(void)
{
    uint16_t read_index;
    uint16_t tx_len = 0U;
    uint32_t primask;

    if(huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if((printf_tx_dma_busy != 0U) || (printf_tx_head == printf_tx_tail))
    {
        __set_PRIMASK(primask);
        return;
    }

    read_index = printf_tx_tail;
    while((read_index != printf_tx_head) && (tx_len < PRINTF_DMA_CHUNK_SIZE))
    {
        printf_tx_dma_buf[tx_len] = printf_tx_fifo[read_index];
        tx_len++;
        read_index = (uint16_t)((read_index + 1U) % PRINTF_TX_FIFO_SIZE);
    }

    printf_tx_active_len = tx_len;
    printf_tx_dma_busy = 1U;
    __set_PRIMASK(primask);

    if(HAL_UART_Transmit_DMA(&huart6, printf_tx_dma_buf, tx_len) != HAL_OK)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        printf_tx_active_len = 0U;
        printf_tx_dma_busy = 0U;
        __set_PRIMASK(primask);
    }
}

/**
  * @brief  向printf字符FIFO写入1个字符
  * @param  ch: 待写入的字符
  * @retval 1=写入成功, 0=写入失败
  */
uint8_t Printf_TxFifoWrite(uint8_t ch)
{
    uint16_t next_head;
    uint8_t result = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    next_head = (uint16_t)((printf_tx_head + 1U) % PRINTF_TX_FIFO_SIZE);
    if(next_head != printf_tx_tail)
    {
        printf_tx_fifo[printf_tx_head] = ch;
        printf_tx_head = next_head;
        result = 1U;
    }

    __set_PRIMASK(primask);

    return result;
}

/**
  * @brief  printf DMA发送完成处理函数
  * @param  无
  * @retval 无
  */
void Printf_TxComplete(void)
{
    uint16_t i;
    uint16_t tx_len;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    tx_len = printf_tx_active_len;
    for(i = 0U; i < tx_len; i++)
    {
        printf_tx_tail = (uint16_t)((printf_tx_tail + 1U) % PRINTF_TX_FIFO_SIZE);
    }

    printf_tx_active_len = 0U;
    printf_tx_dma_busy = 0U;
    __set_PRIMASK(primask);
}

/**
  * @brief  printf DMA发送错误处理函数
  * @param  无
  * @retval 无
  */
void Printf_TxAbortCurrent(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    printf_tx_active_len = 0U;
    printf_tx_dma_busy = 0U;
    __set_PRIMASK(primask);
}

/**
  * @brief  检查FIFO是否已满
  * @param  fifo: FIFO指针
  * @retval 1=已满, 0=未满
  */
static uint8_t Fifo_Is_Full(fifo_t *fifo)
{
    return (((fifo->head+1)%TX_FIFO_SIZE) == ((fifo->tail)%TX_FIFO_SIZE)) ? 1 : 0;
}

/**
  * @brief  检查FIFO是否为空
  * @param  fifo: FIFO指针
  * @retval 1=为空, 0=不为空
  */
static uint8_t Fifo_Is_Empty(fifo_t *fifo)
{
    return ((fifo->head % TX_FIFO_SIZE) == (fifo->tail % TX_FIFO_SIZE)) ? 1 : 0;
}

/**
  * @brief  向FIFO写入数据（变长）
  * @param  fifo: FIFO指针
  * @param  packet: 待写入的数据包
  * @param  len:   数据包实际长度（6或10字节）
  * @retval 1=写入成功, 0=写入失败
  */
uint8_t Fifo_Write(fifo_t *fifo, uint8_t *packet, uint8_t len)
{
    if((fifo == NULL) || (packet == NULL) || (len == 0U) || (len > sizeof(fifo->buf[0])))
    {
        return 0;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if(Fifo_Is_Full(fifo))
    {
        __set_PRIMASK(primask);
        return 0;
    }
    else
    {
        memcpy(fifo->buf[fifo->head], packet, len);
        fifo->len[fifo->head] = len;          // 记录本包实际长度
        fifo->head = (fifo->head + 1) % TX_FIFO_SIZE;
        __set_PRIMASK(primask);
        return 1;
    }
}

/**
  * @brief  从FIFO读取数据（变长）
  * @param  fifo:   FIFO指针
  * @param  packet: 用于存储读取数据的缓冲区
  * @param  len:    输出参数，返回本包实际长度
  * @retval 1=读取成功, 0=读取失败
  */
uint8_t Fifo_Read(fifo_t *fifo, uint8_t *packet, uint8_t *len)
{
    if((fifo == NULL) || (packet == NULL) || (len == NULL))
    {
        return 0;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if(Fifo_Is_Empty(fifo))
    {
        __set_PRIMASK(primask);
        return 0;
    }
    else 
    {
        uint8_t n = fifo->len[fifo->tail];
        memcpy(packet, fifo->buf[fifo->tail], n);
        *len = n;
        fifo->tail = (fifo->tail + 1) % TX_FIFO_SIZE;
        __set_PRIMASK(primask);
        return 1;
    }
}
