/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "stm32f10x.h"                  // Device header
#include "RC_Cmd.h"
#include "LED.h"
#include "OLED.h"
#include "AD.h"
#include "FlashWrite.h"

// 系统时间 单位ms
uint32_t APP_time = 0;
// NTP时间
volatile uint32_t NTP_time = 0;
// 遥控通信频率 单位ms
#define HEARTBEAT_COUNT 100
uint32_t heartBeat_time = 0;
uint8_t heartBeat_SendFlag = 0;
// LED定时器
#define LED_COUNT 200
uint32_t LED_time = 0;
// NTP推送遥控时间
#define NTPSEND_COUNT 200
uint32_t ntp_time = 0;
extern uint8_t NTP_send;
// 看门狗超时
#define WATCHDOG_COUNT 1000
uint32_t watchdog_time = 0;
extern enum cmd_status watchDog_status;
// OLED更新
#define OLED_UPDATE 500
uint32_t oled_time = 0;
uint8_t oledupdate_flag = 0;
// 累计运行时间
#define RUNTIME_COUNT (60 * 1000)	// 1分钟
uint32_t runtime_time = 0;

// *************** 低功耗休眠（30秒无操作自动进入STOP模式） ***************
// 由 main.c 主循环负责：检测到活动就调 Sleep_ReportActivity() 复位计时；
// 连续 30 秒无活动则由 main.c 调用 Sleep_EnterStop() 进入休眠。
volatile uint32_t last_activity_time = 0;   // 最近一次活动的 APP_time（ms）
uint8_t  sleep_active = 0;                  // 1=当前处于休眠状态（由 main.c 维护）

// 复位空闲计时（main.c 检测到活动时调用）
void Sleep_ReportActivity(void)
{
	last_activity_time = APP_time;
}


void APPTimer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);																													//选择时基单元的时钟 选择为内部时钟

	//初始化时基单元:
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10 - 1;																				//决定时间长度的值 周期，就是ARR自动重装器的值		公式：定时频率=72M/(PSC+1)/(ARR+1) 定时1秒就是定时频率为1Hz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;			//7200-1												//决定时间长度的值 PSC预分频器的值 		注意PSC和ARR都要在0-65535之间	这里的配置实现的是计时1s
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;																		//重复计数器的值 重复计数器是高级定时器才有的，所以我们这里设为0
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);																									  //如果没有这一句，那刚上电的时候就会触发一次中断，也就是会从1开始计数。这句话应当加在TimeBaseInit后面NVIC开启中断的前面
	//使能更新中断：
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);																							  //开启更新中断到NVIC通路 第二个值这里选择的是更新中断

	//配置NVIC:
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);																					//配置NVIC优先级分组（中断分组） 这里配置成2位抢占2位响应
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;																//指定通道的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;																			//指定通道的相应优先级
	NVIC_Init(&NVIC_InitStructure);

	//启动定时器：
	TIM_Cmd(TIM2,ENABLE);

}

// 定时器中断函数
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2,TIM_IT_Update)	==SET)																					//这里的TIM_IT_Update对应了上面初始化时基单元L11的配置
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

		// LED定时器
		if (APP_time - LED_time >= LED_COUNT)
		{
			LED_time = APP_time;
			LED_Timer();
		}

		//ntp 推送遥控器时间
		if (NTP_send == 1)
		{
			if (APP_time - ntp_time >= NTPSEND_COUNT)
			{
				ntp_time = APP_time;
				NTP_send = 0;
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

		// OLED更新
		if (APP_time - oled_time >= OLED_UPDATE)
		{
			oled_time = APP_time;
			oledupdate_flag = 1;
		}

		// 记录运行时间
		if (APP_time - runtime_time >= RUNTIME_COUNT)
		{
			runtime_time = APP_time;
			Store_runTime(1);
		}

		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}

