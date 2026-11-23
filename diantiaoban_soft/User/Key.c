/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "stm32f10x.h"                  // Device header
#include "Key.h"
#include "RC_cmd.h"
#include "Serial.h"
#include "Delay.h"
#include "APPTime.h"

uint8_t KeyStatus = 0;
uint32_t currentTime = 0;

extern uint32_t time_controllor;

uint8_t flag = 0;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU ;		// 上拉输入
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
}

uint8_t Key_GetStatus(void)
{
	if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12) == 0)
	{
		if (flag == 1)
		{
			flag = 0;
		}
		else
		{
			flag = 1;
		}
	}
	else
	{

	}

	if (flag == 1)
	{
        flag = 0;

		// RC_send((uint8_t *)&flag, CMD_TIME_CONTROLLOR_RPO, sizeof(uint8_t));
		Serial_Printf("APP_Time = %u \r\n", APP_time);
		// uint16_t temp = 666;
		// uint8_t temparr[2] = {0};
		// *((uint16_t*)temparr) = temp;
		// RC_send(temparr, CMD_TEMPERATURE_RPO, sizeof(uint16_t));
	}

	return 0;
}

