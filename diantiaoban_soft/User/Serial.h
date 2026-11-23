#ifndef __SERIAL_H_
#define __SERIAL_H_

#include <stdio.h>
#include <stdarg.h>

#define SERIAL_TX_BUFFER_SIZE 200
#define SERIAL_RX_BUFFER_SIZE 200
#define SERIAL_TX_QUEUE_SIZE  10      // 发送队列大小 最多缓存10个数据

extern volatile char Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char*String);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
void Serial_Printf(char *format, ...);
//uint8_t Serial_GetRxFlag(void);



#endif
