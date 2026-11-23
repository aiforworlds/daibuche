/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "Serial_3.h"

// DMA缓冲区
uint8_t Serial_3_TxBuffer[SERIAL_3_TX_BUFFER_SIZE];
uint8_t Serial_3_RxBuffer[SERIAL_3_RX_BUFFER_SIZE];

char Serial_3_RxPacket[SERIAL_3_RX_BUFFER_SIZE];		//这两个数组中只存储发送或接收的载荷数据，不包括包头包尾	数量要给多防止溢出，这里要求单条不超过100个字符
uint8_t Serial_3_RxFlag;				//用于中断函数中的标志位
uint16_t Serial_3_RxLength = 0;
volatile uint8_t Serial_3_TxBusy = 0;

// 发送队列
uint8_t Serial_3_TxQueue[SERIAL_3_TX_QUEUE_SIZE][SERIAL_3_TX_BUFFER_SIZE];
uint8_t Serial_3_TxQueue_Head = 0;
uint8_t Serial_3_TxQueue_Tail = 0;
uint16_t Serial_3_TxQueue_Length[SERIAL_3_TX_QUEUE_SIZE];
uint8_t Serial_3_TxQueue_IsEmpty(void)
{
    return Serial_3_TxQueue_Head == Serial_3_TxQueue_Tail;
}
uint8_t Serial_3_TxQueue_IsFull(void)
{
    return ((Serial_3_TxQueue_Tail + 1) % SERIAL_3_TX_QUEUE_SIZE) == Serial_3_TxQueue_Head;
}


void Serial_3_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_3_RxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SERIAL_3_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // 连续接收
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);

	// 清标志
    DMA_ClearFlag(DMA1_FLAG_TC3 | DMA1_FLAG_HT3 | DMA1_FLAG_TE3);
}

void Serial_3_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_3_TxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0; // 动态设置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    // 配置 DMA1_Channel2 的中断
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Serial_3_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//初始化GPIO引脚：
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;		//TX引脚
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);

	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;			//RX引脚
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);

	//初始化USART：
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 19200;																					//填写波特率数值
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);

	//初始化中断：
	// 1) 只开 IDLE 中断，不要再开 RXNE
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    // 2) NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

	// 初始化DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	Serial_3_DMA_RX_Init();
	Serial_3_DMA_TX_Init();

	USART_Cmd(USART3,ENABLE);
}

void Serial_3_SetBaudRate(uint32_t baudrate)
{
    // 1. 关闭接收中断和 DMA 接收
    USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);   // 关闭 IDLE 中断
    DMA_Cmd(DMA1_Channel3, DISABLE);                  // 停止 DMA 接收
    USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);   // 关闭串口 DMA 接收请求

    // 2. 关闭串口
    USART_Cmd(USART3, DISABLE);

    // 3. 重新配置 USART 参数
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    // 4. 启用串口
    USART_Cmd(USART3, ENABLE);

    // 5. 重新启动 DMA 接收（Circular 模式）
    DMA_SetCurrDataCounter(DMA1_Channel3, SERIAL_3_RX_BUFFER_SIZE); // 重设计数器
    DMA_Cmd(DMA1_Channel3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); // 打开 DMA 接收

    // 6. 重新打开 IDLE 中断
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        // 1. 清除 IDLE 标志位
        (void)USART3->SR;
        (void)USART3->DR;

        // 2. 停止 DMA 接收
        DMA_Cmd(DMA1_Channel3, DISABLE);

        // 3. 计算接收到的数据长度
        uint16_t data_len = SERIAL_3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);

        // 4. 清空目标缓冲区并拷贝数据
        if (data_len > 0)
        {
            memset(Serial_3_RxPacket, 0, SERIAL_3_RX_BUFFER_SIZE);  // 清空整个目标数组
            memcpy(Serial_3_RxPacket, Serial_3_RxBuffer, data_len); // 拷贝接收到的数据
            Serial_3_RxLength = data_len;  // 记录数据长度
            Serial_3_RxFlag = 1;           // 标记有新数据
        }

        // 5. 重置 DMA 接收计数器并重新使能
        DMA_SetCurrDataCounter(DMA1_Channel3, SERIAL_3_RX_BUFFER_SIZE);
        DMA_Cmd(DMA1_Channel3, ENABLE);
    }
}

// DMA中断处理函数
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);

        if (!Serial_3_TxQueue_IsEmpty())
        {
            // 从队列取出下一条并发送
            uint8_t* nextString = Serial_3_TxQueue[Serial_3_TxQueue_Head];
            uint16_t length = Serial_3_TxQueue_Length[Serial_3_TxQueue_Head];

            memcpy(Serial_3_TxBuffer, nextString, length);

            Serial_3_TxQueue_Head = (Serial_3_TxQueue_Head + 1) % SERIAL_3_TX_QUEUE_SIZE;

            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNDTR = length;
            DMA1_Channel2->CMAR  = (uint32_t)Serial_3_TxBuffer;
            DMA_ClearFlag(DMA1_FLAG_TC2);
            DMA_Cmd(DMA1_Channel2, ENABLE);
        }
        else
        {
            Serial_3_TxBusy = 0;  // 没有数据了，清空忙标志
        }
    }
}

void Serial_3_SendByte(uint8_t Byte)																//发送数据函数，可以从TX引脚发送一个字节的数据
{
    // 判断当前DMA是否占用
    if (Serial_3_TxBusy) return;
    Serial_3_TxBusy = 1;

	// 把数据写到 DMA 发送缓冲区
    Serial_3_TxBuffer[0] = Byte;

    // 关闭通道，更新发送长度、地址，再打开通道
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA1_Channel2->CNDTR = 1;                              // 只发 1 个字节
    DMA1_Channel2->CMAR  = (uint32_t)&Serial_3_TxBuffer[0];  // 内存地址
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

void Serial_3_SendArray(uint8_t *Array,uint16_t Length)			//发送一个数组
{
    if (Length == 0 || Length > SERIAL_3_TX_BUFFER_SIZE) return;  // 边界保护

    if (Serial_3_TxBusy)
    {
        if (Serial_3_TxQueue_IsFull())
        {
            // 队列已满：删除最早的一条
            Serial_3_TxQueue_Head = (Serial_3_TxQueue_Head + 1) % SERIAL_3_TX_QUEUE_SIZE;
        }

        // 写入队列
        memcpy(Serial_3_TxQueue[Serial_3_TxQueue_Tail], Array, Length);
        Serial_3_TxQueue_Length[Serial_3_TxQueue_Tail] = Length;
        Serial_3_TxQueue_Tail = (Serial_3_TxQueue_Tail + 1) % SERIAL_3_TX_QUEUE_SIZE;
        return;
    }
    Serial_3_TxBusy = 1;

	// 拷贝数据到 DMA 发送缓冲区
    for (uint16_t i = 0; i < Length; i++)
    {
        Serial_3_TxBuffer[i] = Array[i];
    }

    // 关闭 DMA，更新长度，再使能
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA1_Channel2->CNDTR = Length;
    DMA1_Channel2->CMAR = (uint32_t)Serial_3_TxBuffer;
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

void Serial_3_SendString(char*String)		//发送一个字符串
{
    if (Serial_3_TxBusy) return;
    Serial_3_TxBusy = 1;

	uint16_t length = strlen(String);
    if (length == 0 || length > SERIAL_3_TX_BUFFER_SIZE) return;  // 边界保护

    // 把整个字符串拷贝到 DMA 缓冲区
    memcpy(Serial_3_TxBuffer, String, length);

    // 关闭通道，更新发送长度、地址，再打开通道
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA1_Channel2->CNDTR = length;                             // 字符串长度
    DMA1_Channel2->CMAR  = (uint32_t)Serial_3_TxBuffer;          // 缓冲区首地址
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

uint32_t Serial_3_Pow(uint32_t X,uint32_t Y)		//定义一个次方函数，将一串数字的个十百千位拆开，用下面发送的函数发送
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void Serial_3_SendNumber(uint32_t Number,uint8_t Length)		//发送一个数字
{
	uint8_t i;
	for (i = 0;i < Length;i++)
	{
		Serial_3_SendByte(Number/Serial_3_Pow(10,Length - i - 1)%10 + '0');
	}
}


//printf函数移植
int fputc_3(int ch,FILE *f)
{
	Serial_3_SendByte(ch);
	return ch;
}
void Serial_3_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_3_SendString(String);
}












