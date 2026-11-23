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
#include "Serial.h"


char Serial_RxPacket[SERIAL_RX_BUFFER_SIZE];		//这两个数组中只存储发送或接收的载荷数据，不包括包头包尾	数量要给多防止溢出，这里要求单条不超过100个字符
uint8_t Serial_RxFlag;				//用于中断函数中的标志位
volatile uint8_t Serial_TxBusy = 0;

// DMA缓冲区
uint8_t Serial_TxBuffer[SERIAL_TX_BUFFER_SIZE];
uint8_t Serial_RxBuffer[SERIAL_RX_BUFFER_SIZE];

// 发送队列
char Serial_TxQueue[SERIAL_TX_QUEUE_SIZE][SERIAL_TX_BUFFER_SIZE];
uint8_t Serial_TxQueue_Head = 0;
uint8_t Serial_TxQueue_Tail = 0;
uint8_t Serial_TxQueue_IsEmpty(void)
{
    return Serial_TxQueue_Head == Serial_TxQueue_Tail;
}
uint8_t Serial_TxQueue_IsFull(void)
{
    return ((Serial_TxQueue_Tail + 1) % SERIAL_TX_QUEUE_SIZE) == Serial_TxQueue_Head;
}


void Serial_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_RxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SERIAL_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // 连续接收
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);

	// 清标志
    DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5 | DMA1_FLAG_TE5);
}

void Serial_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_TxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0; // 动态设置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    // 配置中断
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);		//开启串口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);		//开启GPIOA时钟

	//初始化GPIO引脚：
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;		//TX引脚是USART外设控制输出引脚，所以要用复用推挽输出
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);							//初始化成复用推挽输出，供USART1的TX使用

	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;			//RX引脚是外设输入引脚，选择输入模式，一般是浮空输入或者上拉输入，不要下拉输入因为串口空闲是高电平
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);

	//初始化USART：
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;																					//填写波特率数值
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			//硬件流控制	（None=不使用流控。CTS=只用CTS。RTS=只用RTS。RTS_CTS=RTS，CTS都用）
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;											//串口模式 （Tx=发送模式。Rx=接收模式） 如果既需要发送又需要接收：USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;																	//校验位（No=无校验。Odd=奇校验。Even=偶校验
	USART_InitStructure.USART_StopBits = USART_StopBits_1;															//停止位	（参数可以选择0_5、1、1_5、2）
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;													//字长 （8b=8位。9b=9位）不需要校验就选择8位
	USART_Init(USART1, &USART_InitStructure);

	//初始化中断：
	// 1) 只开 IDLE 中断，不要再开 RXNE
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    // 2) NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

	// 初始化DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	Serial_DMA_RX_Init();
	Serial_DMA_TX_Init();

	USART_Cmd(USART1,ENABLE);				//使能USART1
}

//uint8_t Serial_GetRxFlag(void)		//判断是不是接收到了数据包，如果RxFlag在中断函数中被置为了1，就表示接收了数据包 将中断函数内读取到的值发送出去
//{
//	if (Serial_RxFlag == 1)		//实现读后自动清零的功能
//	{
//		Serial_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}
			//在.h文件里声明了uint8_t Serial_RxFlag，在main里读取完成之后手动清零，这样就不会数据包错位，只会丢包


void USART1_IRQHandler(void)
{
    // 1) 检查 IDLE 中断
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
    {
        volatile uint32_t tmp;
        // 读取 SR、DR 清除 IDLE 标志（必须先读 SR 再读 DR）
        tmp = USART1->SR;
        tmp = USART1->DR;

        // 关闭 DMA
        DMA_Cmd(DMA1_Channel5, DISABLE);

        // 计算本次接收长度
        uint16_t received = SERIAL_RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;

        // 2) 用和你原来完全一致的状态机解析
        {
            static uint8_t RxState = 0;
            static uint8_t pRxPacket = 0;
            for (uint16_t i = 0; i < received; i++)
            {
                uint8_t RxData = Serial_RxBuffer[i];
                if (RxState == 0)
                {
                    if (RxData == '@' && Serial_RxFlag == 0)
                    {
                        RxState = 1;
                        pRxPacket = 0;
                    }
                }
                else if (RxState == 1)
                {
                    if (RxData == '\r')
                    {
                        RxState = 2;
                    }
                    else
                    {
                        Serial_RxPacket[pRxPacket++] = RxData;
                        // 注意不要越界，确保 pRxPacket < sizeof(Serial_RxPacket)
                    }
                }
                else if (RxState == 2)
                {
                    if (RxData == '\n')
                    {
                        RxState = 0;
                        Serial_RxPacket[pRxPacket] = '\0';
                        Serial_RxFlag = 1;
                    }
                }
            }
        }

        // 3) 重装 DMA 环形缓冲，继续接收
        DMA1_Channel5->CNDTR = SERIAL_RX_BUFFER_SIZE;
        DMA_Cmd(DMA1_Channel5, ENABLE);

        // 清 IDLE 中断标志（已经在读 DR 时清了，这里可选）
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        return;
    }
}

// DMA中断处理函数
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);

        if (!Serial_TxQueue_IsEmpty())
        {
            // 从队列取出下一条并发送
            char* nextString = Serial_TxQueue[Serial_TxQueue_Head];
            uint16_t length = strlen(nextString);

            memcpy(Serial_TxBuffer, nextString, length);

            Serial_TxQueue_Head = (Serial_TxQueue_Head + 1) % SERIAL_TX_QUEUE_SIZE;

            DMA_Cmd(DMA1_Channel4, DISABLE);
            DMA1_Channel4->CNDTR = length;
            DMA1_Channel4->CMAR  = (uint32_t)Serial_TxBuffer;
            DMA_ClearFlag(DMA1_FLAG_TC4);
            DMA_Cmd(DMA1_Channel4, ENABLE);
        }
        else
        {
            Serial_TxBusy = 0;  // 没有数据了，清空忙标志
        }
    }
}


void Serial_SendByte(uint8_t Byte)																//发送数据函数，可以从TX引脚发送一个字节的数据
{
    if (Serial_TxBusy) return;
    Serial_TxBusy = 1;

	// 把数据写到 DMA 发送缓冲区
    Serial_TxBuffer[0] = Byte;

    // 关闭通道，更新发送长度、地址，再打开通道
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = 1;                              // 只发 1 个字节
    DMA1_Channel4->CMAR  = (uint32_t)&Serial_TxBuffer[0];  // 内存地址
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void Serial_SendArray(uint8_t *Array,uint16_t Length)			//发送一个数组的函数	uint8_t* 是一个指针类型，指向待发送数据的首地址	Length是数据长度
{
    if (Serial_TxBusy) return;
    Serial_TxBusy = 1;

	// 拷贝数据到 DMA 发送缓冲区
    for (uint16_t i = 0; i < Length; i++)
    {
        Serial_TxBuffer[i] = Array[i];
    }

    // 关闭 DMA，更新长度，再使能
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = Length;
    DMA1_Channel4->CMAR = (uint32_t)Serial_TxBuffer;
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

void Serial_SendString(char*String)		//发送一个字符串 由于字符串自带结束标志位，所以就不需要再填写长度了 char*=uint8_t*
{
	uint16_t length = strlen(String);
    if (length == 0 || length > SERIAL_TX_BUFFER_SIZE) return;  // 边界保护

    if (Serial_TxBusy)
    {
        if (!Serial_TxQueue_IsFull())
        {
            strcpy(Serial_TxQueue[Serial_TxQueue_Tail], String);
            Serial_TxQueue_Tail = (Serial_TxQueue_Tail + 1) % SERIAL_TX_QUEUE_SIZE;
        }
        // 否则队列满了，不处理（你也可以选择覆盖最老数据）

        return;
    }
    Serial_TxBusy = 1;

    // 把整个字符串拷贝到 DMA 缓冲区
    memcpy(Serial_TxBuffer, String, length);

    // 关闭通道，更新发送长度、地址，再打开通道
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CNDTR = length;                             // 字符串长度
    DMA1_Channel4->CMAR  = (uint32_t)Serial_TxBuffer;          // 缓冲区首地址
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

uint32_t Serial_Pow(uint32_t X,uint32_t Y)		//定义一个次方函数，将一串数字的个十百千位拆开，用下面发送的函数发送
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;		//Result累乘Y次X，就是X的Y次方
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number,uint8_t Length)		//发送一个数字
{
	uint8_t i;
	for (i = 0;i < Length;i++)
	{
		Serial_SendByte(Number/Serial_Pow(10,Length - i - 1)%10 + '0');
				/*参数会由十进制的高位到低位依次发送。Length-i-1表示发送的位数，
				比如当要发送2位数的时候，Length=2，此时第一次循环2-0-1=1，即10的1次方，发送的是十位，第二次循环1-0-1=0即10的0次方，发送的是个位。
				'0'是用来加上偏移，因为ASCII表上，数字0对应的是0x30
				*/

	}
}



int fputc(int ch,FILE *f)		//printf函数移植。不需要知道为什么
{
	Serial_SendByte(ch);			//将fputc函数重定向到串口 fputc函数是printf函数的底层，printf就是循环调用fputc函数实现的
	return ch;
}		//这种方法只能定义一个，如果现在定义到串口1了，串口2再用就没有了，如果多个串口都要用printf的话，就可以用sprintf，见main里的用法
/*
printf函数的移植方法：
打开keil5的‘工程选项’就是编译按钮那一行的魔法棒按钮 -> target -> Use MicroLIB打勾
在串口文件里加上头文件#include <stdio.h> -> 重写fputc函数
*/

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}
/*
封装sprintf函数
添加头文件：#include <stdarg.h>
原理很难不需要理解，只要会用就行
*/












