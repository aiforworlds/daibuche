/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */




#include "stm32f10x.h"                  // Device header
#include "LED.h"
#include "Serial.h"
#include "Delay.h"
#include "FlashWrite.h"
#include "AD.h"
#include "Serial_3.h"
#include "RC_Cmd.h"
#include "APPTime.h"
#include "OLED.h"
#include "key.h"

// *************** 低功耗休眠（30秒无操作自动进入STOP模式，按键唤醒） ***************
extern volatile uint32_t last_activity_time;
extern uint8_t  sleep_active;
extern uint16_t AD_Value[4];
void Sleep_ReportActivity(void);
void Serial_3_DMA_RX_Init(void);   // 唤醒后重启无线模块 DMA 接收

// 进入STOP低功耗模式：关OLED、关LED、关无线模块，然后 WFI 等待按键唤醒
static void Sleep_EnterStop(void)
{
	// 1. 关闭 OLED 显示（0xAE=关显示，可显著降低OLED电流）
	OLED_WriteCommand(0xAE);

	// 2. 关闭所有可关的 LED（设高电平=熄灭，本板低电平点亮）
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	// 3. 关闭无线模块 JC24B（PD=PB4 拉高 => 模块掉电/休眠；SET=PB5 拉高）
	GPIO_SetBits(GPIOB, GPIO_Pin_4);   // PD 高 = 模块休眠
	GPIO_SetBits(GPIOB, GPIO_Pin_5);   // SET 高 = 通信模式（保持）

	// 4. 关闭 USART3（无线模块串口），避免休眠期间漏电/误触发
	USART_Cmd(USART3, DISABLE);

	// 5. 进入 STOP 模式（稳压器低功耗档），WFI 等待任意 EXTI 唤醒
	//    按键 PA4/PA5 已配置为 EXTI 下降沿，可正常唤醒
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);   // 开启电源控制时钟
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	// 唤醒后从这里继续执行（默认时钟为 HSI 8MHz，需恢复）
	SystemInit();   // 恢复系统时钟到 72MHz（重新配置 HSE+PLL）
}

// 从STOP唤醒后的恢复：重新点亮OLED、LED、无线模块
static void Sleep_Wakeup(void)
{
	// 1. 重新开启 USART3 并恢复无线模块工作
	USART_Cmd(USART3, ENABLE);
	Serial_3_DMA_RX_Init();          // 重启 DMA 接收（STOP 后重新触发）

	// 2. 重新初始化无线模块 JC24B（RC_Pin_Init 重配 PD/SET/AUX 引脚 + SET 拉到通信模式）
	//    比手动设 PD 更完整：模块断电休眠再上电后，重新走一遍引脚初始化最稳
	RC_setSysInit();

	// 3. 重新开启 OLED 显示
	OLED_WriteCommand(0xAF);

	// 4. 清除休眠状态，复位空闲计时
	sleep_active = 0;
	last_activity_time = APP_time;
}

// 低功耗休眠检测：活动（摇杆偏离中位 / 串口收数据）则复位计时，30 秒无活动则进入 STOP。
// 注意：必须同时放进主循环和 NTP 对时的内层 while 里（无电调对时时程序会卡在内层 while，
// 若只放在主循环末尾，将永远走不到休眠）。
static void Sleep_Check(void)
{
	int16_t joy = (int16_t)AD_Value[0];
	uint8_t joy_active = (joy < (2048 - 100)) || (joy > (2048 + 100));   // 偏离中位死区
	uint8_t rx_active  = (Serial_3_RxFlag == 1) || (Serial_RxFlag == 1);

	if (joy_active || rx_active)
	{
		Sleep_ReportActivity();   // 复位空闲计时
	}
	else if (!sleep_active)
	{
		// 连续 30 秒无活动 => 进入休眠
		if ((APP_time - last_activity_time) >= (30 * 1000))
		{
			sleep_active = 1;
			Sleep_EnterStop();   // 进入 STOP，直到按键唤醒
			Sleep_Wakeup();      // 被按键唤醒后恢复
		}
	}
}

//解包函数
int8_t RC_unpack(const uint8_t *byteArray);
//软件看门狗
enum cmd_status watchDog_status = error;
void softWatchDog_init(void);
void softWatchDog_loopCheck(void);
void softWatchDog_timeout(void);
void softWatchDog_feed(void);
enum cmd_status sys_status = error;

// 目标速度 0-200 0-99反向 100-200正向
float target_speed = 0;
// 死区 AD值
#define DEAD_ZONE 100
// 油门归零偏置 AD值
#define SPEED_AD_OFFSET -57
// 动力系数
#define SPEED_LEVEL_L	0.3f
#define SPEED_LEVEL_M	0.5f
#define SPEED_LEVEL_H	0.9f
float motor_speed_percentage = SPEED_LEVEL_M;
// 方向
int8_t run_direction_set = 1;		// 1正向 -1反向 0停止 设置方向
int8_t run_direction_report = 1;	// 1正向 -1反向 0停止 上报方向
// 电机转速
float motor_speed = 0;
// DEBUG
float debug_motor_speed = 0;

// 滑板各状态标志位
uint8_t boardStatus_stop_mode = 0;
uint8_t boardStatus_battery_warning = 0;
uint8_t boardStatus_temperature_warning = 0;
uint8_t boardStatus_temperature_error = 0;
uint8_t boardStatus_watchdog_error = 0;
uint8_t boardStatus_Motor_hall_error = 0;
float car_speed = 0;

// 超前角调参模式（PA5 双击进入/退出）
static uint8_t adv_tune_mode = 0;		// 0=正常 1=调参模式
static int8_t adv_fwd_local = 4;		// 前进超前角本地值 0~4（默认 4，实测声音最小）
static int8_t adv_rev_local = 1;		// 后退超前角本地值 0~4

#define WARN_AUTO_CLEAR_WINDOW_MS  (60 * 1000)	// 开机后此时间内出现的No Signal/Downlink Loss，解除后自动清空
// 警告类型（索引越大显示优先级越高） 注意在warn_messages中的警告排序需要与该处索引一致
#define WARN_TYPE_CTRL_BAT_LOW    0
#define WARN_TYPE_CAR_BAT_LOW     1
#define WARN_TYPE_TEMP_WARN       2
#define WARN_TYPE_TEMP_HIGH       3
#define WARN_TYPE_NO_SIGNAL       4
#define WARN_TYPE_DOWNLINK_LOSS   5
#define WARN_TYPE_MOTOR_HALL      6
#define WARN_TYPE_COUNT           7

// 警告状态记录
static uint8_t warn_latched[WARN_TYPE_COUNT] = {0};		// 用于记录每种警告类型当前是否已被触发，1=报错已触发，0=未触发。
static uint8_t warn_auto_clear[WARN_TYPE_COUNT] = {0};	// 用于记录每种警告类型是否在开机窗口内触发，1=报错解除后自动清空（仅No Signal/Downlink Loss在开机窗口内触发时）
static uint8_t warn_display_index = 0;					// 用于记录当前显示的警告类型索引
// 警告信息
static const char *warn_messages[WARN_TYPE_COUNT] = {		// 警告信息 注意排序需要与WARN_TYPE_COUNT一致
	"Ctrl BAT Low",		// 控制器电量低
	"Car BAT Low",		// 滑板电量低
	"Temp Warn",		// 温度警告
	"Temp High",		// 温度过高
	"No Signal",		// 无信号
	"Downlink Loss",	// 下行信号丢失
	"Motor Hall",		// 电机霍尔故障
};

// 警告处理：更新OLED latch状态、切换/清空显示；LED按实时警告驱动
// key_action: 0=刷新（OLED周期调用） 1=切换显示（PA5单击） 2=清空全部（PA5三击，PA4未按住时）
// shown_type: 输出当前OLED应显示的警告类型，不需要时传NULL
// 返回值: 当前可显示的警告数量（0时显示Ready）
static uint8_t warn_process(uint8_t key_action, uint8_t *shown_type)
{
	float battery = adc_getBatteryVoltage();

	// -------- 1. 更新警告触发状态（仅影响OLED latch显示） --------
	if (key_action == 0)
	{
		// No Signal / Downlink Loss 在首次触发时记录是否处于开机自动清空窗口
		// 无信号警告
		if (watchDog_status == error)
		{
			if (!warn_latched[WARN_TYPE_NO_SIGNAL])
			{
				warn_latched[WARN_TYPE_NO_SIGNAL] = 1;
				warn_auto_clear[WARN_TYPE_NO_SIGNAL] = (APP_time < WARN_AUTO_CLEAR_WINDOW_MS) ? 1 : 0;
			}
		}
		else if (warn_latched[WARN_TYPE_NO_SIGNAL] && warn_auto_clear[WARN_TYPE_NO_SIGNAL])
		{
			// 开机窗口内触发：信号恢复后自动清空
			warn_latched[WARN_TYPE_NO_SIGNAL] = 0;
			warn_auto_clear[WARN_TYPE_NO_SIGNAL] = 0;
		}

		// 控制器电量低警告
		if (battery < 3.4f)
		{
			warn_latched[WARN_TYPE_CTRL_BAT_LOW] = 1;
		}
		// 滑板电量低警告
		if (boardStatus_battery_warning == 1)
		{
			warn_latched[WARN_TYPE_CAR_BAT_LOW] = 1;
		}
		// 温度警告
		if (boardStatus_temperature_warning == 1)
		{
			warn_latched[WARN_TYPE_TEMP_WARN] = 1;
		}
		// 温度过高警告
		if (boardStatus_temperature_error == 1)
		{
			warn_latched[WARN_TYPE_TEMP_HIGH] = 1;
		}
		// 下行信号丢失警告
		if (boardStatus_watchdog_error == 1)
		{
			if (!warn_latched[WARN_TYPE_DOWNLINK_LOSS])
			{
				warn_latched[WARN_TYPE_DOWNLINK_LOSS] = 1;
				warn_auto_clear[WARN_TYPE_DOWNLINK_LOSS] = (APP_time < WARN_AUTO_CLEAR_WINDOW_MS) ? 1 : 0;
			}
		}
		else if (warn_latched[WARN_TYPE_DOWNLINK_LOSS] && warn_auto_clear[WARN_TYPE_DOWNLINK_LOSS])
		{
			// 开机窗口内触发：下行恢复后自动清空
			warn_latched[WARN_TYPE_DOWNLINK_LOSS] = 0;
			warn_auto_clear[WARN_TYPE_DOWNLINK_LOSS] = 0;
		}
		// 电机霍尔故障警告
		if (boardStatus_Motor_hall_error == 1)
		{
			warn_latched[WARN_TYPE_MOTOR_HALL] = 1;
		}
	}
	else if (key_action == 2)
	{
		// PA5三击：手动清空全部警告，重置切换索引
		for (uint8_t i = 0; i < WARN_TYPE_COUNT; i++)
		{
			warn_latched[i] = 0;
			warn_auto_clear[i] = 0;
		}
		warn_display_index = 0;
	}

	// -------- 2. 构建OLED可显示警告列表（按优先级从高到低，基于 latch 手动清空逻辑） --------
	// active[warn_display_index] 为OLED当前显示项
	uint8_t active[WARN_TYPE_COUNT];
	uint8_t count = 0;
	for (int8_t i = WARN_TYPE_COUNT - 1; i >= 0; i--)
	{
		uint8_t displayable = 0;
		if (warn_latched[i])
		{
			if (i == WARN_TYPE_NO_SIGNAL)
			{
				// 无信号中直接显示；开机窗口外触发的在信号恢复后仍保留，直到手动清空
				displayable = (watchDog_status == error) || (warn_auto_clear[i] == 0);
			}
			else if (watchDog_status == error)
			{
				// 无信号模式下仅显示 No Signal 与 Ctrl BAT Low
				displayable = (i == WARN_TYPE_CTRL_BAT_LOW);
			}
			else
			{
				displayable = 1;
			}
		}
		if (displayable)
		{
			active[count++] = i;
		}
	}

	// -------- 3. 处理显示切换索引 --------
	if (key_action == 1 && count > 0)
	{
		// PA5单击：在可显示列表中循环切换
		warn_display_index = (warn_display_index + 1) % count;
	}
	if (count == 0)
	{
		warn_display_index = 0;
	}
	else if (warn_display_index >= count)
	{
		// 警告数量变化时，防止索引越界
		warn_display_index = count - 1;
	}

	uint8_t shown = count ? active[warn_display_index] : WARN_TYPE_COUNT;

	// -------- 4. 按实时警告状态更新LED（与OLED latch显示无关） --------
	if (watchDog_status == error)
	{
		LED_Control(LED_RED, off);
		if (battery < 3.4f)
		{
			LED_Control(LED_BLUE, on);
		}
		else
		{
			LED_Control(LED_BLUE, off);
		}
	}
	else
	{
		uint8_t led_highest = WARN_TYPE_COUNT;
		if (boardStatus_Motor_hall_error == 1)
		{
			led_highest = WARN_TYPE_MOTOR_HALL;
		}
		else if (boardStatus_watchdog_error == 1)
		{
			led_highest = WARN_TYPE_DOWNLINK_LOSS;
		}
		else if (boardStatus_temperature_error == 1)
		{
			led_highest = WARN_TYPE_TEMP_HIGH;
		}
		else if (boardStatus_temperature_warning == 1)
		{
			led_highest = WARN_TYPE_TEMP_WARN;
		}
		else if (boardStatus_battery_warning == 1)
		{
			led_highest = WARN_TYPE_CAR_BAT_LOW;
		}

		if (battery < 3.4f)
		{
			LED_Control(LED_BLUE, on);
		}
		else
		{
			LED_Control(LED_BLUE, off);
		}

		if (led_highest == WARN_TYPE_COUNT)
		{
			LED_Control(LED_RED, off);
		}
		else if (led_highest >= WARN_TYPE_TEMP_HIGH)
		{
			LED_Control(LED_RED, flicker);
		}
		else if (led_highest >= WARN_TYPE_CAR_BAT_LOW)
		{
			LED_Control(LED_RED, on);
		}
		else
		{
			LED_Control(LED_RED, off);
		}
	}

	if (shown_type)
	{
		*shown_type = shown;
	}
	return count;
}

enum cmd_status NTP_status = error;
uint8_t NTP_send = 0;

// 在空闲时，写入flash
void writeFlash_loopcheck(void)
{
	static uint32_t lastWrite_time = 0;
	if (APP_time - lastWrite_time > (5 * 60 * 1000))		//5分钟写入一次
	{
		lastWrite_time = APP_time;

		static uint32_t lastNoSpeed_time = 0;
		if ((target_speed == 0 || target_speed == 100) && motor_speed == 0)
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

// 更新OLED
void oled_refresh_runloop(void)
{
	if(oledupdate_flag == 1)	// 控制更新频率
	{
		oledupdate_flag = 0;

		static int16_t last_battery_centi = -1;
		static char last_level_char = 0;

		// 显示电池电量
		float battery = adc_getBatteryVoltage();
		int16_t battery_centi = (int16_t)(battery * 100.0f + 0.5f);
		if (battery_centi != last_battery_centi)
		{
			last_battery_centi = battery_centi;
			OLED_ShowFloatNum(66, 0, battery, 1, 2, OLED_6X8);
			OLED_UpdateArea(66, 0, 62, 8);
		}

		// 显示动力档位
		char level_char;
		if (boardStatus_stop_mode == 1)
		{
			level_char = '@';	// 停止模式 亮块
		}
		else if (motor_speed_percentage == SPEED_LEVEL_L)
		{
			level_char = 'L';
		}
		else if (motor_speed_percentage == SPEED_LEVEL_M)
		{
			level_char = 'M';
		}
		else
		{
			level_char = 'H';
		}
		if (level_char != last_level_char)
		{
			char level_str[2] = {level_char, '\0'};
			last_level_char = level_char;
			OLED_ShowString(90, 27, level_str, OLED_6X8);
			OLED_UpdateArea(90, 27, 38, 8);
		}

		// 显示方向
		if (run_direction_report == 1)
		{
			OLED_ShowString(108, 27, "->", OLED_6X8);
		}
		else if (run_direction_report == -1)
		{
			OLED_ShowString(108, 27, "<-", OLED_6X8);
		}
		else
		{
			OLED_ShowString(108, 27, "--", OLED_6X8);
		}
		OLED_UpdateArea(108, 27, 20, 8);

		// 显示系统状态
		uint8_t shown_warn;
		uint8_t warn_count = warn_process(0, &shown_warn);

		OLED_ShowString(0, 38, "                ", OLED_8X16);
		OLED_ShowString(121, 45, "  ", OLED_6X8);
		if (warn_count == 0)
		{
			OLED_ShowString(121, 45, " ", OLED_6X8);
		}
		else
		{
			OLED_ShowNum(121, 45, warn_count, 1, OLED_6X8);
		}

		if (adv_tune_mode)
		{
			// 调参模式：显示换相超前角 F(前进)/R(后退)
			OLED_ShowString(0, 38, "ADV F", OLED_8X16);
			OLED_ShowNum(40, 38, adv_fwd_local, 1, OLED_8X16);
			OLED_ShowString(48, 38, " R", OLED_8X16);
			OLED_ShowNum(64, 38, adv_rev_local, 1, OLED_8X16);
		}
		else if (boardStatus_stop_mode == 1 && watchDog_status != error)
		{
			OLED_ShowString(10, 38, "@@@ STOP @@@", OLED_8X16);
		}
		else if (warn_count == 0)
		{
			OLED_ShowString(10, 38, "Ready", OLED_8X16);
		}
		else
		{
			OLED_ShowString(10, 38, (char *)warn_messages[shown_warn], OLED_8X16);
		}

		OLED_UpdateArea(0, 38, 128, 16);

		// 显示滑板信息
		OLED_UpdateArea(96, 0, 32, 8);		// 电池电量
		OLED_UpdateArea(66, 9, 40, 8);		// 电调温度
	}
}

// 下发超前角设置命令给电调板
static void send_adv_set(void)
{
	int8_t data[2] = {adv_fwd_local, adv_rev_local};
	RC_send((const uint8_t *)data, CMD_COMM_ADV_SET, 2);
}

// 按键执行函数
void key_runloop(void)
{
	// 按键功能：
	// PA4：单击切换动力档位 三击切换运行方向
	// PA5：三击清空所有报错
	// PA4按住 + PA5三击：切换运行方向  如果三击过程中松开PA4，判断为清空所有报错
	// PA5：双击进入/退出超前角调参模式；调参模式内 PA4=前进角+1、PA5=后退角+1

	uint8_t oled_need_refresh = 0;

	static uint32_t last_pa4_click_time = 0;
	static uint8_t pa4_click_count = 0;
	static float motor_speed_before_pa4_combo = SPEED_LEVEL_M;
	if (key_PA4_flag == 1)
	{
		key_PA4_flag = 0;

		motor_speed_before_pa4_combo = motor_speed_percentage;

		if (adv_tune_mode)
		{
			// 调参模式：前进超前角 +1（0~4 循环）并下发
			adv_fwd_local = (adv_fwd_local + 1) % 5;
			send_adv_set();
			oled_need_refresh = 1;
		}
		else
		{
			// 按下瞬间切换动力档位
			if (motor_speed_percentage == SPEED_LEVEL_L)
			{
				motor_speed_percentage = SPEED_LEVEL_M;
			}
			else if (motor_speed_percentage == SPEED_LEVEL_M)
			{
				motor_speed_percentage = SPEED_LEVEL_H;
			}
			else if (motor_speed_percentage == SPEED_LEVEL_H)
			{
				motor_speed_percentage = SPEED_LEVEL_L;
			}
			oled_need_refresh = 1;
		}

		// 距上次按下 >300ms 视为新连击序列开始，正确计第 1 击（修复原 off-by-one）
		if (APP_time - last_pa4_click_time > 300)
		{
			pa4_click_count = 1;
		}
		else
		{
			pa4_click_count++;
		}
		last_pa4_click_time = APP_time;
	}

	if (APP_time - last_pa4_click_time > 300 && pa4_click_count != 0)
	{
		if (pa4_click_count == 3 && !adv_tune_mode)
		{
			// 三击进入/退出停止模式（三击已循环一圈档位，无需额外切换）
			if (boardStatus_stop_mode == 0)
			{
				if (car_speed <= 1.0f)
				{
					boardStatus_stop_mode = 1;
					oled_need_refresh = 1;
				}
			}
			else
			{
				boardStatus_stop_mode = 0;
				oled_need_refresh = 1;
			}
		}
		pa4_click_count = 0;
	}

	static uint32_t last_pa5_click_time = 0;
	static uint8_t pa5_click_count = 0;
	static uint8_t pa4_held_for_pa5_combo = 0;
	if (key_PA5_flag == 1)
	{
		key_PA5_flag = 0;

		// 距上次按下 >300ms 视为新连击序列开始，正确计第 1 击（修复原 off-by-one）
		if (APP_time - last_pa5_click_time > 300)
		{
			pa5_click_count = 1;
			pa4_held_for_pa5_combo = 1;
		}
		else
		{
			pa5_click_count++;
		}
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) != Bit_RESET)
		{
			pa4_held_for_pa5_combo = 0;
		}

		if (!adv_tune_mode)
		{
			// 非调参模式：按下瞬间切换报错显示
			warn_process(1, NULL);
			oled_need_refresh = 1;
		}
		// 调参模式下 PA5 的单击/双击动作延迟到下方结算，避免双击退出时误加后退角

		last_pa5_click_time = APP_time;
	}

	// 距上次按下超过 300ms：连击序列结束，结算本次是单击/双击/三击
	if (APP_time - last_pa5_click_time > 300 && pa5_click_count != 0)
	{
		if (pa5_click_count == 2)
		{
			// PA5 双击：进入/退出调参模式
			adv_tune_mode = !adv_tune_mode;
			oled_need_refresh = 1;
		}
		else if (pa5_click_count == 1 && adv_tune_mode)
		{
			// 调参模式：后退超前角 +1（0~4 循环）并下发
			adv_rev_local = (adv_rev_local + 1) % 5;
			send_adv_set();
			oled_need_refresh = 1;
		}
		else if (pa5_click_count >= 3 && !adv_tune_mode)
		{
			if (pa4_held_for_pa5_combo)
			{
				// PA4按住 + PA5三击：切换运行方向，并恢复长按PA4时误触的档位
				run_direction_set = (run_direction_set == 1) ? -1 : 1;
				motor_speed_percentage = motor_speed_before_pa4_combo;
				oled_need_refresh = 1;
			}
			else
			{
				// 三击清空所有报错
				warn_process(2, NULL);
				oled_need_refresh = 1;
			}
		}
		pa5_click_count = 0;
	}

	// 按下按键 刷新OLED
	if (oled_need_refresh)
	{
		oledupdate_flag = 1;
	}
}

// ******************* 串口接收 *******************
void Serial_Rx_loopCheck(void)
{
	//当串口收到任何内容时
	if (Serial_RxFlag == 1)
	{
		LED_Turn(1);

		if (Serial_RxPacket[0] == '0')		// 打印速度
		{
			Serial_Printf("\r\nscaledValue = %d\r\n", target_speed);
		}
		else if (Serial_RxPacket[0] == '1')	// 打印log
		{
			Log_print();
			Log_add(LOG_NORMAL, "serial print.");
		}
		else if (Serial_RxPacket[0] == '2')	// 清空log
		{
			Log_reset();
			Log_add(LOG_WARNING, "Log Reset.");
		}
		else if (Serial_RxPacket[0] == '3')	// 显示flash数据
		{
			for (uint16_t i = 1;i < USE_AREA;i++)				//获取Store_Data[512]里的数据
			{
				uint16_t data_item = Store_Data[i];

				if (i == 5 || i == 6 || i == 8)	// 检查当前位置存储的是否是数值数据（千小时、小时、小数）
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
		else if (Serial_RxPacket[0] == '4')	// 写入flash
		{
			Store_runTime(1);
			Store_Save();
		}
		else if (Serial_RxPacket[0] == '5')	// 擦除flash
		{
			Flash_ErasePage(0x0800FC00);
		}

		Serial_Printf("\r\n--\r\n");
		Serial_RxFlag = 0;
	}
}

int main(void)
{
	APPTimer_Init();
	LED_Init();
	key_init();
	Serial_Init();
	Serial_3_Init();
	AD_Init();

	Store_Init();

	RC_setSysInit();

	// 初始化低功耗空闲计时（避免上电即误判超时）
	last_activity_time = APP_time;
	sleep_active = 0;

	softWatchDog_init();

	// OLED初始化
	OLED_Init();
	// 第一行
	OLED_ShowString(0, 0, "Battery", OLED_6X8);
	OLED_ShowFloatNum(66, 0, 0.00, 1, 2, OLED_6X8);		// 遥控电量
	OLED_ShowFloatNum(96, 0, 00.00, 2, 2, OLED_6X8);	// 滑板电量
	// 第二行
	OLED_ShowString(0, 9, "Temperture", OLED_6X8);
	OLED_ShowFloatNum(66, 9, 000.0, 3, 1, OLED_6X8);	// 电调温度
	// 第三行
	OLED_ShowFloatNum(10, 20, 00.00, 2, 2, OLED_8X16);	// 速度
	OLED_ShowString(55, 27, "km/h", OLED_6X8);
	// 第四行
	OLED_ShowString(10, 38, "Start up", OLED_8X16);		// 系统状态
	OLED_ShowString(111, 38, "@@", OLED_8X16);			// 亮块
	// OLED_ShowString(0, 38, "                ", OLED_8X16);		// 清空行
	// 第五行
	OLED_ShowFloatNum(0, 57, 0000.00, 4, 2, OLED_6X8);	// 累计里程
	OLED_ShowString(42, 57, "km", OLED_6X8);
	OLED_ShowFloatNum(60, 57, 0000.0, 4, 1, OLED_6X8);	// 累计小时
	OLED_ShowString(96, 57, "h", OLED_6X8);
	OLED_Update();

	Log_add(LOG_NORMAL, "System start.");


	while(1)
	{
		softWatchDog_loopCheck();

		if (NTP_status == error)
		{
			Log_add(LOG_NTP_ERROR, "NTP_status error");

			while (NTP_status == error)
			{
				// 按键执行
				key_runloop();

				softWatchDog_loopCheck();
				LED_Control(LED_ERROR, flicker);
				NTP_send = 1;

				// 遥控模块接收控制
				if (Serial_3_RxFlag == 1)
				{
					Serial_3_RxFlag = 0;
					RC_unpack((uint8_t *)Serial_3_RxPacket);
				}

				if (NTP_send != 1)
				{
					uint8_t timestamp[4] = {0};
					*((uint32_t*)timestamp) = NTP_time;
					RC_send(timestamp, CMD_TIME_CONTROLLOR_RPO, sizeof(uint32_t));
				}

			// 更新oled
			oled_refresh_runloop();

			writeFlash_loopcheck();
			Serial_Rx_loopCheck();

			// 低功耗休眠检测（无电调对时程序会卡在此内层 while，必须在此也检测）
			Sleep_Check();
		}
		LED_Control(LED_ERROR, off);

			Log_add(LOG_NTP_SUCCESS, "NTP_status success");
		}

		// 遥控通信 将电位器AD值缩放到0-200作为占空比发送给电调
		// speed=100时为0位置
		int16_t speed_AD = AD_Value[0] + SPEED_AD_OFFSET;
		// 限幅保护
		if (speed_AD < 0)		speed_AD = 0;
		if (speed_AD > 4095)	speed_AD = 4095;
		// 死区设置
		if (speed_AD >= 2048 - DEAD_ZONE && speed_AD <= 2048 + DEAD_ZONE)
		{
			speed_AD = 2048;
		}
		target_speed = ((float)speed_AD * 200.0f) / 4096.0f;
		// 限速
		if (target_speed < 100)
		{
			float offset = 100 - target_speed;
			target_speed = 100 - (offset * motor_speed_percentage);		// 限速比例

			// 停止模式 禁止前进动力输出 仍保留刹车动力
			if (boardStatus_stop_mode == 1)
			{
				target_speed = 100;
			}
		}

		// 心跳包发送
		if (heartBeat_SendFlag == 1)
		{
			heartBeat_SendFlag = 0;

			RC_heartbeat_downlink_t sendpack;
			sendpack.speed = target_speed;
			sendpack.status = sys_status;
			sendpack.direction = run_direction_set;
			RC_send((const uint8_t *)&sendpack, CMD_HEARTBEAT_DOWNLINK, sizeof(RC_heartbeat_downlink_t));

			// DEBUG
			debug_motor_speed = sendpack.speed;
		}


		// 更新oled
		oled_refresh_runloop();

		// 遥控模块接收控制
		if (Serial_3_RxFlag == 1)
		{
			Serial_3_RxFlag = 0;

			RC_unpack((uint8_t *)Serial_3_RxPacket);
		}
		Serial_Rx_loopCheck();

		// 写flash
		writeFlash_loopcheck();

		// 按键执行
		key_runloop();

		// 低功耗休眠检测
		Sleep_Check();
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
	// NTP_status = error;
	sys_status = error;
	car_speed = 0;					// 电机速度归零
	run_direction_report = 0;		// 上报方向归零
}

void softWatchDog_feed(void)
{
	watchDog_status = normal;
}

// ******************* 命令执行函数 *******************

void CMD_speed_control(uint8_t *pack)
{
	RC_SpeedControl_t *speedpack = (RC_SpeedControl_t*) pack;

	uint16_t data = speedpack->speed_target;
	Serial_Printf("\r\nspeed_target = %d\r\n", data);
}

void CMD_speed_report(uint8_t *pack)
{
	RC_SpeedControl_t *speedpack = (RC_SpeedControl_t*) pack;

	uint16_t data = speedpack->speed_target;
	Serial_Printf("\r\nspeed_target = %d\r\n", data);
}

void CMD_timeControllor_report(void)
{
	uint8_t timestamp[4] = {0};
	*((uint32_t*)timestamp) = NTP_time;

	RC_send(timestamp, CMD_TIME_CONTROLLOR_RPO, sizeof(uint32_t));
}

void CMD_timeMotor_report(uint8_t *pack)
{
	uint32_t time = *((uint32_t *)pack);

	// 误差小于100 判断为成功对时
	if (((int64_t)NTP_time - (int64_t)time) < 100 && ((int64_t)NTP_time - (int64_t)time) > -50)
	{
		NTP_status = normal;
	}
	else
	{
		NTP_status = error;
	}
	Serial_Printf("CMD_TIME_CONTROLLOR_RPO:%d\r\n", time);
}

void CMD_temperature_report(uint8_t *pack)
{
	uint16_t temperature = *((uint16_t *)pack);
	Serial_Printf("CMD_TEMPERATURE_RPO:%d\r\n", temperature);
}

void CMD_timeSynchronous(uint8_t *pack)
{
	NTPtime_t *NTPpack = (NTPtime_t*) pack;

	NTP_status = error;

	NTPtime_t response;
    response.t1 = NTPpack->t1;		// 原样返回客户端t1
    response.t2 = NTP_time;			// 服务器接收时间
    response.t3 = NTP_time;			// 服务器发送时间
	response.t4 = 0;				// 从机接收时间 由从机填入

	RC_send((uint8_t *)&response, CMD_TIME_SYNCHRONOUS, sizeof(NTPtime_t));

	softWatchDog_feed();
}

void CMD_mileageReport(uint8_t *pack)
{
	uint16_t KKilometer = ((uint16_t)pack[0] << 8) | pack[1];
	uint16_t Kilometer = ((uint16_t)pack[2] << 8) | pack[3];
	uint16_t Meter = ((uint16_t)pack[4] << 8) | pack[5];

	// 合并为km单位float变量
	uint64_t total_meters = (uint64_t)KKilometer * 1000000UL + (uint64_t)Kilometer * 1000UL + Meter;
	float total_kilometers = (float)total_meters / 1000.0f;
	total_kilometers = (total_kilometers > 65535.999f) ? 65535.999f : total_kilometers;		// 限制最大值防止溢出

	if (total_kilometers < 0 || total_kilometers > 9999.99)		// 防止显示错误
	{
		total_kilometers =  9999.99;
	}
	OLED_ShowFloatNum(0, 57, total_kilometers, 4, 2, OLED_6X8);
	OLED_UpdateArea(0, 57, 42, 8);
}

void CMD_runTimeReport(uint8_t *pack)
{
	uint16_t thousand_hours = ((uint16_t)pack[0] << 8) | pack[1];
	uint16_t hours = ((uint16_t)pack[2] << 8) | pack[3];
	uint16_t minutes = ((uint16_t)pack[4] << 8) | pack[5];

	uint64_t total_integer_hours = (uint64_t)thousand_hours * 1000UL + hours;
	float total_hours = (float)total_integer_hours + ((float)minutes / 60.0f);
	total_hours = (total_hours > 65535.999f) ? 65535.999f : total_hours;

	if (total_hours < 0 || total_hours > 9999.9)
	{
		total_hours = 9999.9f;
	}
	OLED_ShowFloatNum(60, 57, total_hours, 4, 1, OLED_6X8);
	OLED_UpdateArea(60, 57, 36, 8);
}

void CMD_heartbeat(uint8_t *pack)
{
	RC_heartbeat_uplink_t *packet = (RC_heartbeat_uplink_t*) pack;

	static int16_t last_speed_centi = -9999;
	static int16_t last_car_battery_centi = -1;
	static int16_t last_temperature_deci = -9999;
	static uint8_t last_temp_wide = 0xFF;

	// 系统状态解包
	boardStatus_battery_warning = (packet->status >> 0) & 0x01;
	boardStatus_temperature_warning = (packet->status >> 1) & 0x01;
	boardStatus_temperature_error = (packet->status >> 2) & 0x01;
	boardStatus_watchdog_error = (packet->status >> 3) & 0x01;
	boardStatus_Motor_hall_error = (packet->status >> 4) & 0x01;

	float motor_speed = packet->speed;
	car_speed = motor_speed;

	int16_t speed_centi = (int16_t)(motor_speed * 100.0f + ((motor_speed >= 0) ? 0.5f : -0.5f));
	if (speed_centi != last_speed_centi)
	{
		last_speed_centi = speed_centi;
		if (motor_speed >= 0)
		{
			OLED_ShowString(2, 20, " ", OLED_8X16);
			OLED_ShowFloatNum(10, 20, motor_speed, 2, 2, OLED_8X16);
		}
		else
		{
			OLED_ShowFloatNum(2, 20, motor_speed, 2, 2, OLED_8X16);
		}
		OLED_UpdateArea(2, 20, 60, 16);
	}

	if (motor_speed >= 0)
	{
		int16_t car_battery_centi = (int16_t)(packet->battery * 100.0f + 0.5f);
		if (car_battery_centi != last_car_battery_centi)
		{
			last_car_battery_centi = car_battery_centi;
			OLED_ShowFloatNum(96, 0, packet->battery, 2, 2, OLED_6X8);
		}

		uint8_t temp_wide = (packet->temperature >= 99) ? 1 : 0;
		int16_t temperature_deci = (int16_t)(packet->temperature * 10.0f + 0.5f);
		if (temperature_deci != last_temperature_deci || temp_wide != last_temp_wide)
		{
			last_temperature_deci = temperature_deci;
			last_temp_wide = temp_wide;
			if (temp_wide)
			{
				OLED_ShowFloatNum(66, 9, packet->temperature, 3, 1, OLED_6X8);
			}
			else
			{
				OLED_ShowString(66, 9, "     ", OLED_6X8);
				OLED_ShowFloatNum(66, 9, packet->temperature, 2, 1, OLED_6X8);
			}
		}
	}

	// 上报当前方向
	run_direction_report = packet->direction;

	softWatchDog_feed();
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

	switch (pack->cmd_id)
	{
		case CMD_SPEED_CONTROL:
		{
			CMD_speed_control((uint8_t *)pack->data);
			return 0;
		}
		case CMD_SPEED_REPORT:
		{
			return 0;
		}
		case CMD_SPEED_REPORT_REQ:
		{
			CMD_speed_report((uint8_t *)pack->data);
			return 0;
		}
		case CMD_TIME_CONTROLLOR_RPO:
		{
			CMD_timeControllor_report();
			return 0;
		}
		case CMD_TIME_MOTOR_RPO:
		{
			CMD_timeMotor_report((uint8_t *)pack->data);
			return 0;
		}
		case CMD_TEMPERATURE_RPO:
		{
			CMD_temperature_report((uint8_t *)pack->data);
			return 0;
		}
		case CMD_TIME_SYNCHRONOUS:
		{
			CMD_timeSynchronous((uint8_t *)pack->data);
			return 0;
		}
		case CMD_MILEAGE_RPO:
		{
			CMD_mileageReport((uint8_t *)pack->data);
			return 0;
		}
		case CMD_RUNTIME_RPO:
		{
			CMD_runTimeReport((uint8_t *)pack->data);
			return 0;
		}
		case CMD_HEARTBEAT_UPLINK:
		{
			CMD_heartbeat((uint8_t *)pack->data);
			return 0;
		}
		default:
			return -2;
	}
}




