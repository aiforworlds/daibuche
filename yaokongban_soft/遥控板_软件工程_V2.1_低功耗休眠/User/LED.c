/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "LED.h"
#include "stm32f10x.h"                  // Device header

/*
低电平点亮LED

PA1为电池按键
*/

int BatteryKey_Able = 0;					//周期激活电池（0失能/1使能）
uint16_t BatteryKey_Count = 0;


void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);


	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;			//***
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);															//***
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_13;			//***
	GPIO_Init(GPIOC,&GPIO_Initstructure);															//***

	GPIO_SetBits(GPIOB,GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);		//设为高电平，防止一上电就点亮
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}

void LED_Set(int a,enum LED_State chose)		//点亮或熄灭LED
{
	switch (a)
	{
		case 1:if(chose == ON){GPIO_ResetBits(GPIOB,GPIO_Pin_13);}else{GPIO_SetBits(GPIOB,GPIO_Pin_13);}break;		//***
		case 2:if(chose == ON){GPIO_ResetBits(GPIOB,GPIO_Pin_14);}else{GPIO_SetBits(GPIOB,GPIO_Pin_14);}break;		//***
		case 3:if(chose == ON){GPIO_ResetBits(GPIOB,GPIO_Pin_15);}else{GPIO_SetBits(GPIOB,GPIO_Pin_15);}break;		//***
		case 4:if(chose == ON){GPIO_ResetBits(GPIOC,GPIO_Pin_13);}else{GPIO_SetBits(GPIOC,GPIO_Pin_13);}break;		//***

	}
}

void LED_Turn(int a)				//改变LED状态
{
	switch (a)
	{					//用GPIO_ReadOutputDataBit读取输出的值，然后判断输出值是0/1，然后取反
		case 1:if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_13)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_13);}else {GPIO_SetBits(GPIOB,GPIO_Pin_13);}break;		//***
		case 2:if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_14);}else {GPIO_SetBits(GPIOB,GPIO_Pin_14);}break;		//***
		case 3:if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_15)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_15);}else {GPIO_SetBits(GPIOB,GPIO_Pin_15);}break;		//***
		case 4:if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==1){GPIO_ResetBits(GPIOC,GPIO_Pin_13);}else {GPIO_SetBits(GPIOC,GPIO_Pin_13);}break;		//***
	}
}

enum LED_status LED_control[4];

void LED_Control(enum LED_choose LED, enum LED_status status)
{
	LED_control[LED] = status;
}

// LED闪烁定时器函数
void LED_Timer(void)
{
	for (uint8_t i = 0; i <= 3; i++)
	{
		switch (LED_control[i])
		{
			case off:
			{
				switch (i)
				{
					case LED_GREEN:
					{
						GPIO_SetBits(GPIOB, GPIO_Pin_15);
						break;
					}
					case LED_RED:
					{
						GPIO_SetBits(GPIOB, GPIO_Pin_14);
						break;
					}
					case LED_BLUE:
					{
						GPIO_SetBits(GPIOB, GPIO_Pin_13);
						break;
					}
					case LED_ERROR:
					{
						GPIO_SetBits(GPIOC, GPIO_Pin_13);
						break;
					}
					default:
						break;
				}
				break;
			}
			case on:
			{
				switch (i)
				{
					case LED_GREEN:
					{
						GPIO_ResetBits(GPIOB, GPIO_Pin_15);
						break;
					}
					case LED_RED:
					{
						GPIO_ResetBits(GPIOB, GPIO_Pin_14);
						break;
					}
					case LED_BLUE:
					{
						GPIO_ResetBits(GPIOB, GPIO_Pin_13);
						break;
					}
					case LED_ERROR:
					{
						GPIO_ResetBits(GPIOC, GPIO_Pin_13);
						break;
					}
					default:
						break;
				}
				break;
			}
			case flicker:
			{
				switch (i)
				{
					case LED_GREEN:
					{
						if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_15)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_15);}else {GPIO_SetBits(GPIOB,GPIO_Pin_15);}
						break;
					}
					case LED_RED:
					{
						if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_14);}else {GPIO_SetBits(GPIOB,GPIO_Pin_14);}
						break;
					}
					case LED_BLUE:
					{
						if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_13)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_13);}else {GPIO_SetBits(GPIOB,GPIO_Pin_13);}
						break;
					}
					case LED_ERROR:
					{
						if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13)==1){GPIO_ResetBits(GPIOC,GPIO_Pin_13);}else {GPIO_SetBits(GPIOC,GPIO_Pin_13);}
						break;
					}
					default:
						break;
				}
				break;
			}
		}
	}
}

