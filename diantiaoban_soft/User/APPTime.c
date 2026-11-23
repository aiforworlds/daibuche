/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "stm32f10x.h"                  // Device header
#include "Hall.h"
#include "LED.h"
#include "RC_Cmd.h"
#include "Delay.h"
#include "APPTime.h"
#include "FlashWrite.h"

// 系统时间 单位ms
volatile uint32_t APP_time = 0;
// NTP时间
volatile uint32_t NTP_time = 0;
// 心跳包 单位ms
#define HEARTBEAT_COUNT 150
uint32_t heartBeat_time = 0;
volatile uint8_t heartBeat_SendFlag = 0;	//中断写/主循环读，必须 volatile
// Hall计算速度
#define HALLSPEED_COUNT 100
uint32_t hallSpeed_time = 0;
// LED定时器
#define LED_COUNT 200
uint32_t LED_time = 0;
// NTP对时模式
#define NTP_COUNT 50
uint32_t ntp_time = 0;
extern volatile uint8_t NTP_req;
// 看门狗超时
#define WATCHDOG_COUNT 700
uint32_t watchdog_time = 0;
extern volatile enum cmd_status watchDog_status;
// 累计运行时间
#define RUNTIME_COUNT (60 * 1000)	// 1分钟
uint32_t runtime_time = 0;


void APPTimer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_InternalClockConfig(TIM4);                        // 使用 TIM4

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;         // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;    // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);                  // 清除更新中断标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);             // 开启更新中断

    // NVIC 配置（TIM4_IRQn）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);  // 启动定时器
}

// 定时器中断函数
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4,TIM_IT_Update)	==SET)
	{
		// 系统工作时间 单位ms
		// 大约连续工作49天才溢出
		APP_time++;
		NTP_time++;
		if (APP_time > 0xffffffff - 2)
		{
			APP_time = 0;
		}

		// 心跳包发送频率
		if (APP_time - heartBeat_time >= HEARTBEAT_COUNT)
		{
			heartBeat_time = APP_time;
			heartBeat_SendFlag = 1;
		}

		// 霍尔计算速度定时器
		if (APP_time - hallSpeed_time >= HALLSPEED_COUNT)
		{
			hallSpeed_time = APP_time;
			HallSpeed_Timer();
		}

		// LED定时器
		if (APP_time - LED_time >= LED_COUNT)
		{
			LED_time = APP_time;
			LED_Timer();
		}

		// ntp时间同步模式
		if (NTP_req == 1)
		{
			if (APP_time - ntp_time >= NTP_COUNT)
			{
				NTP_req = 0;
				ntp_time = APP_time;
			}
		}

		// 看门狗超时
		if (APP_time - watchdog_time >= WATCHDOG_COUNT)
		{
			watchdog_time = APP_time;
			if (watchDog_status == warning)
			{
				watchDog_status = error;
			}
			else if (watchDog_status == error)
			{
				watchDog_status = error;
			}
			else if (watchDog_status == normal)
			{
				watchDog_status = warning;
			}

		}

		// 记录运行时间
		if (APP_time - runtime_time >= RUNTIME_COUNT)
		{
			runtime_time = APP_time;
			Store_runTime(1);
		}


		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}


// ******************* 定时器执行函数 *******************
