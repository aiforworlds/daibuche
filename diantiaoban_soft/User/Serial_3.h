#ifndef __SERIAL_3_H_
#define __SERIAL_3_H_

#include <stdio.h>
#include <stdarg.h>

#define SERIAL_3_TX_BUFFER_SIZE 200
#define SERIAL_3_RX_BUFFER_SIZE 200
#define SERIAL_3_TX_QUEUE_SIZE  10      // 发送队列大小 最多缓存10个数据


extern volatile char Serial_3_RxPacket[];
extern volatile uint8_t Serial_3_RxFlag;
extern volatile uint16_t Serial_3_RxLength;

void Serial_3_Init(void);
void Serial_3_SetBaudRate(uint32_t baudrate);
void Serial_3_SendByte(uint8_t Byte);
void Serial_3_SendArray(uint8_t *Array,uint16_t Length);
void Serial_3_SendString(char*String);
void Serial_3_SendNumber(uint32_t Number,uint8_t Length);
void Serial_3_Printf(char *format, ...);



#endif
