
/**
 * 电动滑板开源代码
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */



#include "stm32f10x.h"                  // Device header
#include "LED.h"
#include "Hall.h"
#include "Serial.h"
#include "BLDC_Motor.h"
#include "Delay.h"
#include "FlashWrite.h"
#include "AD.h"
#include "Serial_3.h"
#include "RC_Cmd.h"
#include "APPTime.h"
#include "Key.h"
#include <stdlib.h>
#include <string.h>
#include "fan.h"

//解包函数
int8_t RC_unpack(const uint8_t *byteArray);
//软件看门狗
volatile enum cmd_status watchDog_status = error;	//中断(TIM4)写/主循环读，必须 volatile
void softWatchDog_init(void);
void softWatchDog_loopCheck(void);
void softWatchDog_timeout(void);
void softWatchDog_feed(void);
enum cmd_status sys_status = error;

int16_t SetSpeedRC = 0;

// 数据上报模式
static uint8_t serial_report_mode = 0;
void serial_report_loopcheck(void)
{
	// 注意：频繁上报数据会导致电机控制阻塞！仅用于临时调试

	static uint32_t last_report_time = 0;
	if (serial_report_mode == 0) {
		return;
	}
	if (APP_time - last_report_time < 100) {
		return;
	}
	last_report_time = APP_time;

	float battery = adc_calculate_battery_voltage();
	float temperature = adc_getTemperature();
	// Serial_Printf("%f,%f,%d,%f,%d\n", battery, temperature, AD_Value[0], Hall_speed, SetSpeedRC);
	// Serial_Printf("%d,%f\n", AD_Value[0], Hall_speed);

}

// ******************* 串口接收 *******************
void Serial_Rx_loopCheck(void)		// 该串口用于链接电脑调试
{
	//当串口收到任何内容时，返回当前Hall位置（需要加上包头@和包尾\r\n）
	if (Serial_RxFlag == 1)
	{
		// 用于调试
		// uint16_t temp = (uint16_t)APP_time;
		// uint8_t temparr[2] = {0};
		// *((uint16_t*)temparr) = temp;
		// RC_send(temparr, CMD_TEMPERATURE_RPO, sizeof(uint16_t));

		if (Serial_RxPacket[0] == '0')
		{
			Serial_Printf("\r\n--\r\n");
			Serial_Printf("Hall_Count:%d\r\n",Hall_Count);
			Serial_Printf("ADC_AGND:%d\r\n",AD_Value[0]);
			Serial_Printf("ADC_bat:%d\r\n",AD_Value[1]);
			Serial_Printf("ADC_NTC:%d\r\n",AD_Value[3]);
			Serial_Printf("--");
			Serial_Printf("\r\n--\r\n");
		}
		else if (Serial_RxPacket[0] == '1')
		{
			Log_print();
			Log_add(LOG_NORMAL, "serial print.");
			// Log_add(LOG_NORMAL, "");
		}
		else if (Serial_RxPacket[0] == '2')
		{
			Log_reset();
			Log_add(LOG_WARNING, "Log Reset.");
		}
		else if (Serial_RxPacket[0] == '3')
		{
			//Serial_Printf("Motor_Direction:%d\r\n", Motor_Direction());
			Serial_Printf("Speed:%f; Movement:%d", Get_Speed(), Hall_Direction);
		}
		else if (Serial_RxPacket[0] == '4')
		{
			for (uint16_t i = 1;i < USE_AREA;i++)				//获取Store_Data[512]里的数据
			{
				uint16_t data_item = Store_Data[i];

				if (i == 5 || i == 7 || i == 9 || i == 16 || i == 17 || i == 19)	// 检查当前位置存储的是否是数值数据（千小时、小时、小数）
				{
					// 如果是数值，直接以十进制格式打印整个 uint16_t
					Serial_Printf("%d", data_item);
				}
				else
				{
					// 如果是打包的字符，需要将 uint16_t 拆分成两个 uint8_t 字符
					uint8_t high_char = (uint8_t)(data_item >> 8); // 获取高8位
					uint8_t low_char = (uint8_t)(data_item & 0xFF);  // 获取低8位

					// 依次打印这两个字符
					Serial_Printf("%c", high_char);
					Serial_Printf("%c", low_char);
				}

				Delay_ms(1);
			}
		}
		else if (Serial_RxPacket[0] == '5')		// 写入flash
		{
			// Store_Mileage(10);
			// Store_runTime(6);
			Store_Save();
			Serial_Printf("Save OK");
		}
		else if (Serial_RxPacket[0] == '6')		// 清空flash
		{
			Flash_ErasePage(0x0800FC00);
			Serial_Printf("Erase OK");
		}
		else if (Serial_RxPacket[0] == '7')		// 风扇最高速
		{
			fan_setSpeed(100);
			Serial_Printf("Fan Max Speed");
		}
		else if (Serial_RxPacket[0] == '8')		// 风扇最低速
		{
			fan_setSpeed(0);
			Serial_Printf("Fan Min Speed");
		}
		else if (Serial_RxPacket[0] == 'A')		// 进入数据上报模式
		{
			serial_report_mode = 1;
			Serial_Printf("Report ON\r\n");
		}
		else if (Serial_RxPacket[0] == 'B')		// 退出数据上报模式
		{
			serial_report_mode = 0;
			Serial_Printf("Report OFF\r\n");
		}

		Serial_RxFlag = 0;
	}
}
void Serial_3_Rx_loopCheck(void)
{
	if (Serial_3_RxFlag == 1)
	{
		Serial_3_RxFlag = 0;
		RC_unpack((uint8_t *)Serial_3_RxPacket);
	}
}


uint16_t LED = 0;		//  DEBUG

uint32_t time_controllor = 0xffffffff;
volatile enum cmd_status NTP_status = error;
volatile uint8_t NTP_req = 0;	//中断(TIM4)写/主循环读，必须 volatile

// 在空闲时，写入flash
void writeFlash_loopcheck(void)
{
	static uint32_t lastWrite_time = 0;
	if (APP_time - lastWrite_time > (5 * 60 * 1000))		//5分钟写入一次
	{
		lastWrite_time = APP_time;

		static uint32_t lastNoSpeed_time = 0;
		if (Hall_speed == 0 && SetSpeedRC == 0)
		{
			if (APP_time - lastNoSpeed_time > (20 * 1000))	// 确保空闲 连续20秒没有速度
			{
				Store_Save();				// 写入flash
			}
		}
		else
		{
			lastNoSpeed_time = APP_time;
		}
	}
}

// 开机自检
// 自检项目开关
#define SELFCHECK_TEMPERATURE	1
#define SELFCHECK_BATTERY		1
#define SELFCHECK_HALL			1
#define SELFCHECK_CURRENT		1
// return 0 = 自检成功  1 = 自检错误
uint8_t selfCheck(void)
{
	uint8_t check_count = 0;
	if (SELFCHECK_TEMPERATURE) check_count++;
	if (SELFCHECK_BATTERY) check_count++;
	if (SELFCHECK_HALL) check_count++;
	if (SELFCHECK_CURRENT) check_count++;

	while (1)
	{
		LED_Control(LED_ERROR, on);
		LED_Control(LED_RED, on);
		Serial_Rx_loopCheck();
		serial_report_loopcheck();

		uint8_t success_count = 0;

		// 温度检测
		if (SELFCHECK_TEMPERATURE)
		{
			uint8_t temperatureSuccess_count = 0;
			for (int8_t i = 10; i > 0; i--)
			{
				float temperature = adc_getTemperature();
				if (temperature > TEMPERATURE_LOW && temperature < TEMPERATURE_ERROR)
				{
					temperatureSuccess_count ++;
				}
				else
				{
					temperatureSuccess_count = 0;
				}

				if (temperatureSuccess_count >= 5)
				{
					success_count ++;
					break;
				}
			}
		}

		// 电池电量
		if (SELFCHECK_BATTERY)
		{
			uint8_t batterySuccess_count = 0;
			for (int8_t i = 10; i > 0; i--)
			{
				float battery = adc_calculate_battery_voltage();
				if (battery > 30.0f && battery < 45.0f)
				{
					batterySuccess_count ++;
				}
				else
				{
					batterySuccess_count = 0;
				}

				if (batterySuccess_count >= 5)
				{
					success_count ++;
					break;
				}
			}
		}

		// 霍尔信号
		if (SELFCHECK_HALL)
		{
			if (HallSensor.Position == 1 || HallSensor.Position == 2 || HallSensor.Position == 3 ||
				HallSensor.Position == 4 || HallSensor.Position == 5 || HallSensor.Position == 6)
			{
				success_count ++;
			}
		}

		// 电流采样
		if (SELFCHECK_CURRENT)
		{
			uint16_t current = AD_Value[0];
			if (current != 0 && current < 5)
			{
				success_count ++;
			}
		}

		// 结算自检结果
		if (success_count == check_count)
		{
			LED_Control(LED_RED, off);
			Log_add(LOG_NORMAL, "selfCheck Success.");

			return 0;
		}

		static uint8_t logwriteflag_selfcheck = 0;
		if (logwriteflag_selfcheck == 0)
		{
			logwriteflag_selfcheck = 1;
			Log_add(LOG_NORMAL, "selfCheck Error.");
		}

	}
}



int main(void)
{
	APPTimer_Init();
	LED_Init();
	Serial_Init();
	Serial_3_Init();
	Motor_Control_Init();
	fan_init();

	Store_Init();

	AD_Init();
	adc_current_calibrate_zero();		// 电流采样零点校准（此时电机未运行）
	RC_setSysInit();

	Key_Init();
	softWatchDog_init();

	//PWM-LED PA0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;					//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 30;					//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = ENABLE;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_Cmd(TIM2,ENABLE);
	TIM_SetCompare1(TIM2,95);  //设置占空比为95

	Log_add(LOG_NORMAL, "System start.");
	selfCheck();


	while(1)
	{
		softWatchDog_loopCheck();
		adc_update_cache_10ms();


		// NTP时间同步
		if (NTP_status == error)
		{
			//softWatchDog_feed();
			Log_add(LOG_NTP_ERROR, "NTP_status error");
			Motor_Control(0);
			NTP_req = 1;

			while (NTP_status == error)
			{
				softWatchDog_loopCheck();
				LED_Control(LED_ERROR, flicker);
				NTP_req = 1;

				if (Serial_3_RxFlag == 1)
				{
					Serial_3_RxFlag = 0;
					RC_unpack((uint8_t *)Serial_3_RxPacket);
				}

				if (NTP_req != 1)
				{
					// NTP对时 请求发送
					NTPtime_t sendPack;
					sendPack.t1 = NTP_time;
					sendPack.t2 = 0;
					sendPack.t3 = 0;
					sendPack.t4 = 0;
					RC_send((uint8_t *)&sendPack, CMD_TIME_SYNCHRONOUS, sizeof(NTPtime_t));

					NTP_req = 1;
				}

				writeFlash_loopcheck();
				Serial_Rx_loopCheck();
				serial_report_loopcheck();
				fan_runloop();
			}
			LED_Control(LED_ERROR, off);
			uint8_t timestamp[4] = {0};
			*((uint32_t*)timestamp) = NTP_time;
			RC_send(timestamp, CMD_TIME_MOTOR_RPO, sizeof(uint32_t));

			Log_add(LOG_NTP_SUCCESS, "NTP_status success");
		}

		// 温度监测
		enum cmd_status sys_status_temperature = error;
		float temperature = adc_get_temperature_cached();
		if (temperature <= TEMPERATURE_HIGH)										sys_status_temperature = normal;
		else if (temperature > TEMPERATURE_HIGH && temperature < TEMPERATURE_ERROR)	sys_status_temperature = warning;
		else if (temperature >= TEMPERATURE_ERROR)									sys_status_temperature = error;
		// 电量检测
		enum cmd_status sys_status_battery = error;
		float battery = adc_get_battery_voltage_cached();
		if (battery < 30.0f && battery > 29.0f)			sys_status_battery = warning;
		else if (battery <= 29.0f || battery > 45.0f)	sys_status_battery = error;
		else											sys_status_battery = normal;

		// 系统状态结算  先结算warning 后结算error 实现error优先级比warning高
		sys_status = normal;
		if (sys_status_battery == warning)			sys_status = warning;
		else if (sys_status_temperature == warning)	sys_status = warning;
		if (sys_status_temperature == error)	sys_status = error;
		else if (watchDog_status == error)		sys_status = error;
		else if (sys_status_battery == error)	sys_status = warning;		// 低电量不断电，防止动力突然中断

		// 电机动力控制
		if (sys_status != error)
		{
			Motor_Control(SetSpeedRC);
		}
		else
		{
			Motor_Control(0);
		}

		Key_GetStatus();

		// 心跳包发送
		if (heartBeat_SendFlag == 1)
		{
			heartBeat_SendFlag = 0;

			RC_heartbeat_uplink_t sendpack;
			sendpack.speed = Hall_speed;
			sendpack.battery = battery;
			sendpack.temperature = temperature;
			sendpack.current = adc_get_current_cached();	// 母线电流上报
			sendpack.direction = Motor_run_direction;

			// 系统状态封包
			uint8_t inpack_battery = 0;
			if (sys_status_battery != normal)			inpack_battery = 1;
			uint8_t inpack_temperature_warning = 0;
			uint8_t inpack_temperature_error = 0;
			if (sys_status_temperature == warning)		inpack_temperature_warning = 1;
			else if (sys_status_temperature == error)	inpack_temperature_error = 1;
			uint8_t inpack_watchdog = 0;
			if (watchDog_status == error)				inpack_watchdog = 1;
			sendpack.status = RC_status_pack(inpack_battery, inpack_temperature_warning, inpack_temperature_error, inpack_watchdog, Motor_hallError);

			RC_send((const uint8_t *)&sendpack, CMD_HEARTBEAT_UPLINK, sizeof(RC_heartbeat_uplink_t));
		}

		// 串口接收
		Serial_3_Rx_loopCheck();
		Serial_Rx_loopCheck();
		serial_report_loopcheck();

		// 写入flash
		writeFlash_loopcheck();

		// 空闲时操作（电机未旋转一段时间判断为空闲）
		static uint32_t lastNoSpeed_time = 0;
		if (Hall_speed == 0 && SetSpeedRC == 0)
		{
			if (APP_time - lastNoSpeed_time > (10 * 1000))	// 连续10秒没有速度
			{
				static uint32_t lastSend_time = 0;
				if (APP_time - lastSend_time > (5 * 1000))	// 每5秒发送一次 里程与运行时间交错发送
				{
					lastSend_time = APP_time;

					static uint8_t sendContent_flag = 0;
					if (sendContent_flag == 0)
					{
						sendContent_flag = 1;

						// 上报累计里程
						uint16_t KKilometer = Store_Data[9];
						uint16_t Kilometer = Store_Data[7];
						uint16_t Meter = Store_Data[5];

						// 打包数据
						uint8_t sendpack[6] = {0};
						sendpack[0] = (uint8_t)(KKilometer >> 8);
						sendpack[1] = (uint8_t)(KKilometer & 0xFF);
						sendpack[2] = (uint8_t)(Kilometer >> 8);
						sendpack[3] = (uint8_t)(Kilometer & 0xFF);
						sendpack[4] = (uint8_t)(Meter >> 8);
						sendpack[5] = (uint8_t)(Meter & 0xFF);

						// 发送
						RC_send(sendpack, CMD_MILEAGE_RPO, sizeof(sendpack));
					}
					else
					{
						sendContent_flag = 0;

						// 上报运行时间
						uint16_t thousand_hours = Store_Data[16];
						uint16_t hours = Store_Data[17];
						uint16_t minutes = Store_Data[19];

						// 打包数据
						uint8_t sendpack[6] = {0};
						sendpack[0] = (uint8_t)(thousand_hours >> 8);
						sendpack[1] = (uint8_t)(thousand_hours & 0xFF);
						sendpack[2] = (uint8_t)(hours >> 8);
						sendpack[3] = (uint8_t)(hours & 0xFF);
						sendpack[4] = (uint8_t)(minutes >> 8);
						sendpack[5] = (uint8_t)(minutes & 0xFF);

						// 发送
						RC_send(sendpack, CMD_RUNTIME_RPO, sizeof(sendpack));
					}
				}

				// 空闲时温度降低 关闭风扇
				static uint32_t time_fancontrol = 0;
				if (APP_time - time_fancontrol > 10000)		// 10秒后关闭风扇
				{
					time_fancontrol = APP_time;
					if (temperature < 40)		fan_setSpeed(0);
					else						fan_setSpeed(100);
				}
			}
		}
		else
		{
			lastNoSpeed_time = APP_time;

			static uint32_t time_fancontrol = 0;
			if (APP_time - time_fancontrol > 5000)		// 5秒后打开风扇
			{
				time_fancontrol = APP_time;
				fan_setSpeed(100);
			}
		}

		// 更新风扇转速
		fan_runloop();

	}
}

// ******************* 软件看门狗 *******************
void softWatchDog_init(void)
{
	watchDog_status = normal;
	sys_status = normal;
}

uint8_t log_write = 0;
void softWatchDog_loopCheck(void)
{
	if (watchDog_status == error)
	{
		softWatchDog_timeout();
		LED_Control(LED_ERROR, on);

		if (log_write == 0)
		{
			log_write = 1;
			Log_add(LOG_HEARTBEAT, "Heartbeat loss.");
		}
	}
	else
	{
		if (NTP_status != error)
		{
			LED_Control(LED_ERROR, off);
		}
		log_write = 0;
	}
}

void softWatchDog_timeout(void)
{
	SetSpeedRC = 0;
	Motor_filter_prev = 0;
	Motor_Control(0);
	NTP_status = error;
	// sys_status = error;
}

void softWatchDog_feed(void)
{
	watchDog_status = normal;

	// LED呼吸灯  DEBUG
	static uint8_t breath_LED_flag = 0;
	if (breath_LED_flag == 0)	LED += 5;
	else						LED -= 5;
	if (LED >= 98)
	{
		LED = 98;
		breath_LED_flag = 1;
	}
	else if (LED <= 50)
	{
		LED = 50;
		breath_LED_flag = 0;
	}
	TIM_SetCompare1(TIM2,LED);

}

// ******************* 命令执行函数 *******************

void CMD_speed_control(uint8_t *pack)
{
	RC_SpeedControl_t *speedpack = (RC_SpeedControl_t*) pack;

	SetSpeedRC = (100 - speedpack->speed_target) / 2;
}

void CMD_speed_report(void)
{
	RC_SpeedControl_t speedpack;
	speedpack.speed_current = 1;
	RC_send((uint8_t *)&speedpack, CMD_SPEED_REPORT, sizeof(RC_SpeedControl_t));
}

void CMD_time_controllor_report(uint8_t *pack)
{
	uint32_t time = *((uint32_t *)pack);
	time_controllor = time;

	// 误差小于100 判断为成功对时
	if (((int64_t)NTP_time - (int64_t)time) < 100 && ((int64_t)NTP_time - (int64_t)time) > -50)
	{
		NTP_status = normal;
	}
	else
	{
		NTP_status = error;
	}
	//Serial_Printf("time_ctrl = %u local = %u \r\n", time_controllor, NTP_time);

	uint8_t timestamp[4] = {0};
	*((uint32_t*)timestamp) = NTP_time;
	RC_send(timestamp, CMD_TIME_MOTOR_RPO, sizeof(uint32_t));
}

void CMD_time_motor_report(uint8_t *pack)
{
	uint16_t time = *((uint16_t *)pack);
	Serial_Printf("CMD_TIME_CONTROLLOR_RPO:%d\r\n", time);
}

void CMD_temperature_report(void)
{
	uint16_t temperature = AD_Value[3];

	uint8_t sendArray[2] = {0};
	*((uint16_t*)sendArray) = temperature;

	RC_send(sendArray, CMD_TEMPERATURE_RPO, sizeof(uint16_t));
}

void CMD_batteryVoltage_report(void)
{
	uint16_t voltage = AD_Value[1];

	uint8_t sendArray[2] = {0};
	*((uint16_t*)sendArray) = voltage;

	RC_send(sendArray, CMD_TEMPERATURE_RPO, sizeof(uint16_t));
}

void CMD_timeSynchronous(uint8_t *pack)
{
	if (NTP_status != normal)
	{
		NTPtime_t *NTPpack = (NTPtime_t*) pack;

		const int64_t t1 = (int64_t)NTPpack->t1;
		const int64_t t2 = (int64_t)NTPpack->t2;
		const int64_t t3 = (int64_t)NTPpack->t3;
		const int64_t t4 = (int64_t)NTP_time;

		const int32_t delay = (t4 - t1) - (t3 - t2);
		const int32_t offset = ((t2 - t1) + (t3 - t4)) / 2;

		NTPpack->t1 = NTP_time;
		RC_send((uint8_t *)NTPpack, CMD_TIME_SYNCHRONOUS, sizeof(NTPtime_t));

		// 延迟在100ms以下才接受数据
		if (delay < 150 && delay > 0)
		{
			static int64_t tempoffset[3];
			static uint8_t tempcount = 0;
			if ((NTP_time + offset) > 0)
			{
				if (tempcount == 0 || offset != tempoffset[tempcount - 1])
				{
					tempoffset[tempcount ++] = offset;
				}
			}
			if (tempcount >= 3)
			{
				// 找最大值和最小值
				int64_t max_val = tempoffset[0];
				int64_t min_val = tempoffset[0];
				for (int i = 1; i < 3; ++i)
				{
					if (tempoffset[i] > max_val) max_val = tempoffset[i];
					if (tempoffset[i] < min_val) min_val = tempoffset[i];
				}

				// 判断差值是否小于50 满足则更新 NTP_time
				if ((max_val - min_val) < 50)
				{
					// 判断时间偏移量是否在合法范围
					if (((tempoffset[2] + NTP_time) >= 0 || (uint32_t)(-tempoffset[2]) <= NTP_time) && NTP_status == error)
					{
						// 误差小于100 判断为成功对时
						if ((((int64_t)NTP_time + tempoffset[2]) - (int64_t)NTPpack->t3) < 100 && (((int64_t)NTP_time + tempoffset[2]) - (int64_t)NTPpack->t3) > -50)
						{
							__disable_irq();
							NTP_time += tempoffset[2];
							__enable_irq();
							NTP_status = normal;
						}
					}
				}
				tempcount = 0;
				memset(tempoffset, 0, sizeof(tempoffset));
			}
		}
	}

	softWatchDog_feed();
}

void CMD_heartbeat(uint8_t *pack)
{
	RC_heartbeat_downlink_t *packet = (RC_heartbeat_downlink_t*) pack;

	SetSpeedRC = -((int16_t)packet->speed - 100);
	Motor_run_direction = packet->direction;
	if (packet->status != normal)
	{
		// LED_Control(LED_GREEN, on);
	}
	else
	{
		// LED_Control(LED_GREEN, off);
	}

	softWatchDog_feed();
}

// 换相超前角调节：遥控器调参模式下实时下发
void CMD_comm_adv_set(uint8_t *pack)
{
	int8_t fwd = (int8_t)pack[0];
	int8_t rev = (int8_t)pack[1];
	if (fwd >= 0 && fwd <= 4)	Comm_Adv_Fwd = fwd;
	if (rev >= 0 && rev <= 4)	Comm_Adv_Rev = rev;
}

// ******************* 遥控解包 *******************

int8_t RC_unpack(const uint8_t *byteArray)
{
	uint16_t packet_size = Serial_3_RxLength;
	if (packet_size < 8 || byteArray[0] != 0x5B)
	{
		return -1;
	}
	if (byteArray[packet_size - 2] != 0x5D)
	{
		return -1;
	}

	uint8_t crc8 = byteArray[packet_size - 1];
	if (crc8 != Calculate_CRC8(byteArray, packet_size - 1))
	{
		return -1;
	}

	const RC_pack_t *pack = (const RC_pack_t *)byteArray;

	// 判断当前NTP是否成功对时
	uint32_t time = pack->timestamp;
	// 误差小于100 判断为成功对时
	if (((int64_t)NTP_time - (int64_t)time) < 100 && ((int64_t)NTP_time - (int64_t)time) > -100)
	{
		NTP_status = normal;
	}
	else
	{
		NTP_status = error;
	}

	switch (pack->cmd_id)
	{
		case CMD_SPEED_CONTROL:
		{
			CMD_speed_control((uint8_t *)pack->data);
			return 0;
		}
		case CMD_SPEED_REPORT:
		{
			CMD_speed_report();
			return 0;
		}
		case CMD_SPEED_REPORT_REQ:
		{
			return 0;
		}
		case CMD_TIME_CONTROLLOR_RPO:
		{
			CMD_time_controllor_report((uint8_t *)pack->data);
			return 0;
		}
		case CMD_TIME_MOTOR_RPO:
		{
			CMD_time_motor_report((uint8_t *)pack->data);
			return 0;
		}
		case CMD_TEMPERATURE_RPO:
		{
			CMD_temperature_report();
			return 0;
		}
		case CMD_TIME_SYNCHRONOUS:
		{
			CMD_timeSynchronous((uint8_t *)pack->data);
			return 0;
		}
		case CMD_HEARTBEAT_DOWNLINK:
		{
			CMD_heartbeat((uint8_t *)pack->data);
			return 0;
		}
		case CMD_COMM_ADV_SET:
		{
			CMD_comm_adv_set((uint8_t *)pack->data);
			return 0;
		}
		default:
			return -2;
	}

}



