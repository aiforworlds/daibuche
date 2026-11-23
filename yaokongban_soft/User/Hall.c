/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

/*
霍尔传感器程序
当hall位置改变时通过中断更新位置
*/

#include "Device/Include/stm32f10x.h"   // Device header
#include "Hall.h"
#include "Serial.h"

#define HALL_A_PIN GPIO_Pin_12
#define HALL_B_PIN GPIO_Pin_13
#define HALL_C_PIN GPIO_Pin_14


HallSensorTypeDef HallSensor;		//声明结构体，存储霍尔位置
int Hall_Movement_Log = 0;			//临时存储上一个Hall位置
int Hall_Count = 0;		//固定时间内，中断发起的次数，用于计算速度。
float Hall_RotationSpeed = 0;		//转速

//用于计算速度的定时器		0.5S一次中断
void SpeedTimer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);																													//选择时基单元的时钟 选择为内部时钟

	//初始化时基单元:
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 5000 - 1;																				//决定时间长度的值 周期，就是ARR自动重装器的值		公式：定时频率=72M/(PSC+1)/(ARR+1) 定时1秒就是定时频率为1Hz
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



void Hall_GetPosition(void)
{
	HallSensor.HallA = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12);
	HallSensor.HallB = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13);
	HallSensor.HallC = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
}

//确定Hall传感器位置的函数
//返回值为1-6 返回值为0表示出错
void Hall_Position(void)
{
	Hall_GetPosition();
	if (HallSensor.HallA == 0 && HallSensor.HallB == 1 && HallSensor.HallC == 0){ HallSensor.Position = 1;}	//0-60
	if (HallSensor.HallA == 0 && HallSensor.HallB == 1 && HallSensor.HallC == 1){ HallSensor.Position = 2;}	//120
	if (HallSensor.HallA == 0 && HallSensor.HallB == 0 && HallSensor.HallC == 1){ HallSensor.Position = 3;}	//180
	if (HallSensor.HallA == 1 && HallSensor.HallB == 0 && HallSensor.HallC == 1){ HallSensor.Position = 4;}	//240
	if (HallSensor.HallA == 1 && HallSensor.HallB == 0 && HallSensor.HallC == 0){ HallSensor.Position = 5;}	//300
	if (HallSensor.HallA == 1 && HallSensor.HallB == 1 && HallSensor.HallC == 0){ HallSensor.Position = 6;}	//360
}

//Hall旋转方向函数：
int Hall_Movement(void)
{
	int Movement = 0;
	Movement = Hall_Movement_Log - HallSensor.Position;
	Hall_Movement_Log = HallSensor.Position;
	if (Movement == 5) { Movement = -1;}
	if (Movement == -5) { Movement = 1;}
	return Movement;

}

void Hall_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = HALL_A_PIN | HALL_B_PIN | HALL_C_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//配置AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);

	// 配置外部中断线
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 													// 上升下降沿触发中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

	// 配置外部中断中断优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	Hall_Position();
}

//int Count = 0;
//hall位置 中断函数
void EXTI15_10_IRQHandler(void)
{
//	Serial_Printf("%d\r\n",Count);	//还需要修改，串口放在中断里时间太长了。
//	Count++;

	Hall_Position();
	//Serial_Printf("interrupt_position\r\n");
	//Serial_Printf("%d\r\n",HallSensor.Position);	//还需要修改，串口放在中断里时间太长了。

	Hall_Count++;														//一圈发起60次中断
//	Serial_Printf("%d\r\n",Hall_Count);

	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
}

/*
   *** 速度可以输出，但是没有写正确的计算公式，等有轮子后再调试。还要更改中断发起时间
*/
//转速计算：
static float Speed_Time = 0.5;		//秒
float Get_Speed(void)
{
	float Speed = 0;
	Speed = (Hall_Count / 60) / Speed_Time;		//转一圈Hall_RotationCount增加60次。(圈/秒)
	//Serial_Printf("%f\r\n",Speed);
	return Speed;
}



void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2,TIM_IT_Update)	==SET)																					//这里的TIM_IT_Update对应了上面初始化时基单元L11的配置
	{
//		Serial_Printf("%d\r\n",Hall_Count);					//每次中断发送一次串口数据
//		RotationCount = Get_Speed();
		Hall_Count = 0;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}







