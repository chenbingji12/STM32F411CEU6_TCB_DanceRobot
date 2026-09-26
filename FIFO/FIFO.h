/**
 * @file    FIFO.h
 * @brief   FIFO队列操作函数声明
 */

#ifndef __FIFO_H
#define __FIFO_H

#include "main.h"

#define TX_FIFO_SIZE 30      //定义FIFO环形缓冲区大小

typedef struct{
    uint8_t buf[TX_FIFO_SIZE][10];      //数组环形缓冲区
    uint8_t len[TX_FIFO_SIZE];          //每包实际长度（6或10字节）
    volatile uint8_t head;      //写指针
    volatile uint8_t tail;      //读指针
}fifo_t;

extern fifo_t fifo;

extern uint8_t fifo_packet[10];

uint8_t Fifo_Write(fifo_t *fifo, uint8_t *packet, uint8_t len);
uint8_t Fifo_Read(fifo_t *fifo, uint8_t *packet, uint8_t *len);

uint8_t Printf_TxFifoWrite(uint8_t ch);
void Printf_TxStartNext(void);
void Printf_TxComplete(void);
void Printf_TxAbortCurrent(void);

#endif
