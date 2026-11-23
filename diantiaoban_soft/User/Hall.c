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

#include "stm32f10x.h"                  // Device header
#include "Hall.h"
#include "Serial.h"
#include "FlashWrite.h"
#include "BLDC_Motor.h"

#define HALL_A_PIN GPIO_Pin_13
#define HALL_B_PIN GPIO_Pin_14
#define HALL_C_PIN GPIO_Pin_12


volatile HallSensorTypeDef HallSensor;		//声明结构体，存储霍尔位置（中断写/主循环读，必须 volatile）
int Hall_Movement_Log = 0;			//临时存储上一个Hall位置
volatile uint16_t Hall_Count = 0;		//固定时间内，中断发起的次数，用于计算速度。
volatile int Hall_Count_temp = 0;		//固定时间内，中断发起的次数，用于计算速度。
volatile float Hall_speed = 0;			//时速 km/h
volatile float Hall_rollSpeed = 0;		//转速 rp/s
volatile int Hall_Direction = 0;			//旋转方向


void Hall_GetPosition(void)
{
	HallSensor.HallA = GPIO_ReadInputDataBit(GPIOB,HALL_A_PIN);
	HallSensor.HallB = GPIO_ReadInputDataBit(GPIOB,HALL_B_PIN);
	HallSensor.HallC = GPIO_ReadInputDataBit(GPIOB,HALL_C_PIN);
}

//确定Hall传感器位置的函数
//返回值为1-6 返回值为0表示出错
void Hall_Position(void)
{
	Hall_GetPosition();
	if (HallSensor.HallA == 0 && HallSensor.HallB == 1 && HallSensor.HallC == 0)			HallSensor.Position = 1;	//0-60
	else if (HallSensor.HallA == 0 && HallSensor.HallB == 1 && HallSensor.HallC == 1)		HallSensor.Position = 2;	//120
	else if (HallSensor.HallA == 0 && HallSensor.HallB == 0 && HallSensor.HallC == 1)		HallSensor.Position = 3;	//180
	else if (HallSensor.HallA == 1 && HallSensor.HallB == 0 && HallSensor.HallC == 1)		HallSensor.Position = 4;	//240
	else if (HallSensor.HallA == 1 && HallSensor.HallB == 0 && HallSensor.HallC == 0)		HallSensor.Position = 5;	//300
	else if (HallSensor.HallA == 1 && HallSensor.HallB == 1 && HallSensor.HallC == 0)		HallSensor.Position = 6;	//360
	else 																					HallSensor.Position = 0;	//错误状态
}

//Hall旋转方向函数：
void Hall_directionLoopCheck(void)
{
	int Movement = 0;
	Movement = Hall_Movement_Log - HallSensor.Position;
	Hall_Movement_Log = HallSensor.Position;
	if (Movement == 5) { Movement = -1;}
	if (Movement == -5) { Movement = 1;}
	Movement = -Movement;

	if (Movement == 1 || Movement == -1)
	{
		Hall_Direction = Movement * DIRECTION * Motor_run_direction;
	}
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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
	Motor_Commutation();	// 霍尔位置变化立即换相（消除主循环轮询延迟，改善噪声与顿挫）
	//Serial_Printf("interrupt_position\r\n");
	//Serial_Printf("%d\r\n",HallSensor.Position);	//还需要修改，串口放在中断里时间太长了。

	Hall_Count_temp++;														//一圈发起60次中断
	Hall_directionLoopCheck();		// 更新转动方向信息
//	Serial_Printf("%d\r\n",Hall_Count);

	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
}

// 转速低通滤波
#define RPM_FILTER_ALPHA		0.8f
float RPM_filter_prev = 0;
static float RPM_filter(float input)
{
	float output = RPM_FILTER_ALPHA * input + (1 - RPM_FILTER_ALPHA) * RPM_filter_prev;
	RPM_filter_prev = output;
    return output;
}

#define Wheel_diameter 83.0f		//轮子直径 单位mm（83mm 直驱滑板轮）
#define Speed_Time 0.1f		//秒

// 由轮径派生：速度与里程共用同一轮径，避免两处硬编码不一致
#define Wheel_circumference (3.141592f * Wheel_diameter / 1000.0f)	// 轮子周长 单位 m
#define Meters_per_count  (Wheel_circumference / 60.0f)			// 每个霍尔中断前进距离（一圈60中断）
#define Counts_per_meter  (60.0f / Wheel_circumference)			// 前进1米所需中断数
// 转速计算
void Get_rollSpeed(void)
{
	float Speed = 0;
	float count = Hall_Count;
	Speed = (count / 60.0f) / Speed_Time;		//转一圈Hall_RotationCount增加60次。(圈/秒)

	if (Hall_Direction != 0)
	{
		Speed = Speed * Hall_Direction;
	}

	Hall_rollSpeed = RPM_filter(Speed);

	// 更新里程累计（每米所需中断数、每中断前进距离均由轮径派生）
	static uint16_t mileage_count = 0;
	mileage_count += count;
	if (mileage_count >= (uint16_t)Counts_per_meter)
	{
		static float decimal = 0;	// 累计每次计算留下的小数 在下次结算时加上 提高里程累计精度
		float meter = Meters_per_count * mileage_count + decimal;
		uint8_t add_meter = (uint8_t) meter;
		decimal = meter - add_meter;

		Store_Mileage(add_meter);
		mileage_count = 0;
	}

}

//速度计算
float Get_Speed(void)
{
	Get_rollSpeed();
	Hall_speed = Hall_rollSpeed * Wheel_circumference * 3.6f;	// 圈/秒 × 周长(m) × 3.6 → km/h
	return Hall_speed;
}


void HallSpeed_Timer(void)
{
	Hall_Count = Hall_Count_temp;
	Get_Speed();
	Hall_Count_temp = 0;

}







