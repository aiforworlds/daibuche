/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "stm32f10x.h"                  // Device header
#include "BLDC_Motor.h"
#include "Hall.h"
#include "AD.h"
#include "RC_Cmd.h"
#include "APPTime.h"

/*
BLDC驱动程序

UH = PA7 VH = PB0 WH = PB1
UL = PA4 VL = PA5 WL = PA6
*/

#define UH_Pin		GPIO_Pin_7
#define UH_GPIO		GPIOA
#define VH_Pin		GPIO_Pin_0
#define VH_GPIO		GPIOB
#define WH_Pin		GPIO_Pin_1
#define WH_GPIO		GPIOB
#define UL_Pin		GPIO_Pin_6
#define UL_GPIO		GPIOA
#define VL_Pin		GPIO_Pin_7
#define VL_GPIO		GPIOB
#define WL_Pin		GPIO_Pin_8
#define WL_GPIO		GPIOB

#define	MOTOR_ARR	(300 - 1)
#define	MOTOR_PSC	(8 - 1)

// ===== 换相超前角（运行时可调）=====
// 单位：1 格 = 60° 电角度。换相公式：前进 = 霍尔位置 - 换相角，后退 = 霍尔位置 + 换相角。
//
// 关键概念：本电机霍尔信号与反电动势零点之间存在固定物理偏移，实测"换相与反电动势对齐"（零超前）为：
//   前进 Position-2、后退 Position+1。这组值即 V2.0 原始换相映射，弱场滑行/刹车必须用它，
//   绝不能改成 0（Position-0 相对对齐点滞后 120°，弱场时会反向转矩 → 松手后电机反转）。
#define COMM_ALIGN_FWD	2	// 前进标准对齐角（反电动势零点，弱场滑行/刹车用）
#define COMM_ALIGN_REV	1	// 后退标准对齐角（反电动势零点，弱场滑行/刹车用）

// 可调换相角（驱动用）：在对齐基础上再超前可降噪。
// Comm_Adv_Fwd=4 即 Position-4，相对对齐(Position-2)额外超前 120°，实测声音最小、最接近成品 FOC。
// 由遥控器 CMD_COMM_ADV_SET 命令实时修改（范围 0~4），断电不保存。
volatile int8_t Comm_Adv_Fwd = 4;	// 前进换相角（绝对值，Position - Comm_Adv_Fwd）
volatile int8_t Comm_Adv_Rev = 1;	// 后退换相角（绝对值，Position + Comm_Adv_Rev）
volatile int8_t Adv_Fwd_Now = 4;	// 实际应用的前进换相角（驱动↔弱场切换时渐变，避免换相突变产生"咯噔"）

#define ADV_STEP_MS		20	// 换相角渐变每步(60°电角度)间隔 ms，F4→F2(对齐)共 2 步约 40ms

#define HALL_ERROR_RESET_MS	60000

uint8_t Motor_hallError = 0;
static uint32_t Motor_hallError_trigger_time = 0;

// 计算Ke  弱场控制  计算值为：0.147333
#define K_rpm_to_rads (2.0f * 3.1415926535f) / 60.0f;
float Motor_calculate_Ke(uint8_t input_Duty)
{
	float Duty_min_hold = (float) input_Duty / 100.0f;
	float V_bus = adc_calculate_battery_voltage();
	float RPM_min_hold = Hall_rollSpeed * 60;
    float omega_mech = RPM_min_hold * K_rpm_to_rads;		// 计算机械角速度（rad/s）

	float Ke = (V_bus * Duty_min_hold) / omega_mech;
	return Ke;		// 单位  V·s/rad
}

// 计算最小滑行占空比  弱场控制
#define KE		0.130f
#define Delta_V	0.1f
uint16_t calculate_min_slip_duty(void)
{
	float RPM = Hall_rollSpeed * 60;
	float V_bus = adc_calculate_battery_voltage();

	float BEMF = KE * RPM * K_rpm_to_rads;
	float Duty_min_slip = (BEMF + Delta_V) / V_bus;

	// 限制范围
    if (Duty_min_slip < 0.0f) Duty_min_slip = 0.0f;
    if (Duty_min_slip > 1.0f) Duty_min_slip = 1.0f;

	float ccr_value = Duty_min_slip * 100.0f;
	// 四舍五入并限制在0-99范围内
    uint8_t ccr = (uint8_t)(ccr_value + 0.5f);  // 四舍五入
    if (ccr > 99) ccr = 99;

	return ccr * ((MOTOR_ARR + 1) / 100);
}

volatile uint16_t Motor_Speed = 0;	//占空比（范围：0-ARR的值）
volatile int8_t Motor_comm_dir = 0;	// 当前换相方向：-1=前进 1=后退 0=关闭（霍尔中断即时换相用）
volatile uint8_t Motor_weak_field = 0;	// 1=弱场控制(滑行/刹车)：换相回到标准对齐角，保证反电动势对齐
volatile int8_t Motor_run_direction = 1;		// 方向 1正向 -1反向 0停止（主循环写/霍尔中断读，必须 volatile）

void Motor_Init(void)
{
	// 配置PWM引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_Pin = UH_Pin;
	GPIO_Init(UH_GPIO,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin = VH_Pin;
	GPIO_Init(VH_GPIO,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin = WH_Pin;
	GPIO_Init(WH_GPIO,&GPIO_Initstructure);

	// 配置定时器	36kHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = MOTOR_ARR;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = MOTOR_PSC;	//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

	// 配置输出通道
	// UH
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = ENABLE;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_Cmd(TIM3,ENABLE);
	TIM_SetCompare2(TIM3,0);
	// VH
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = ENABLE;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_Cmd(TIM3,ENABLE);
	TIM_SetCompare3(TIM3,0);
	// WH
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = ENABLE;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	TIM_Cmd(TIM3,ENABLE);
	TIM_SetCompare4(TIM3,0);

	//SD引脚初始化
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin = UL_Pin;
	GPIO_Init(UL_GPIO,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin = VL_Pin;
	GPIO_Init(VL_GPIO,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin = WL_Pin;
	GPIO_Init(WL_GPIO,&GPIO_Initstructure);



}

void Motor_Set(uint8_t Position)
{
	switch (Position)
	{
		case 1:		//	UH-WL
		{
			TIM_SetCompare2(TIM3,Motor_Speed);					//UH
			TIM_SetCompare3(TIM3,0);										//VH
			TIM_SetCompare4(TIM3,0);										//WH
			GPIO_SetBits(UL_GPIO, UL_Pin);
			GPIO_ResetBits(VL_GPIO, VL_Pin);
			GPIO_SetBits(WL_GPIO, WL_Pin);
			break;
		}
		case 2:		//	VH-WL
		{
			TIM_SetCompare2(TIM3,0);										//UH
			TIM_SetCompare3(TIM3,Motor_Speed); 					//VH
			TIM_SetCompare4(TIM3,0);										//WH
			GPIO_ResetBits(UL_GPIO, UL_Pin);
			GPIO_SetBits(VL_GPIO, VL_Pin);
			GPIO_SetBits(WL_GPIO, WL_Pin);
			break;
		}
		case 3:		//	VH-UL
		{
			TIM_SetCompare2(TIM3,0);										//UH
			TIM_SetCompare3(TIM3,Motor_Speed); 					//VH
			TIM_SetCompare4(TIM3,0);										//WH
			GPIO_SetBits(UL_GPIO, UL_Pin);
			GPIO_SetBits(VL_GPIO, VL_Pin);
			GPIO_ResetBits(WL_GPIO, WL_Pin);
			break;
		}
		case 4:		//	WH-UL
		{
			TIM_SetCompare2(TIM3,0);										//UH
			TIM_SetCompare3(TIM3,0); 										//VH
			TIM_SetCompare4(TIM3,Motor_Speed);					//WH
			GPIO_SetBits(UL_GPIO, UL_Pin);
			GPIO_ResetBits(VL_GPIO, VL_Pin);
			GPIO_SetBits(WL_GPIO, WL_Pin);
			break;
		}
		case 5:		//	WH-VL
		{
			TIM_SetCompare2(TIM3,0);										//UH
			TIM_SetCompare3(TIM3,0); 										//VH
			TIM_SetCompare4(TIM3,Motor_Speed);					//WH
			GPIO_ResetBits(UL_GPIO, UL_Pin);
			GPIO_SetBits(VL_GPIO, VL_Pin);
			GPIO_SetBits(WL_GPIO, WL_Pin);
			break;
		}
		case 6:		//	UH-VL
		{
			TIM_SetCompare2(TIM3,Motor_Speed);					//UH
			TIM_SetCompare3(TIM3,0); 										//VH
			TIM_SetCompare4(TIM3,0);										//WH
			GPIO_SetBits(UL_GPIO, UL_Pin);
			GPIO_SetBits(VL_GPIO, VL_Pin);
			GPIO_ResetBits(WL_GPIO, WL_Pin);
			break;
		}
		default:	//未知状态，关闭所有MOS
		{
			TIM_SetCompare2(TIM3,0);	//UH
			TIM_SetCompare3(TIM3,0); 	//VH
			TIM_SetCompare4(TIM3,0);	//WH
			GPIO_ResetBits(UL_GPIO, UL_Pin);							//如果全部Set，那么电机转动会有阻力，可能是相当于三根线短路刹车
			GPIO_ResetBits(VL_GPIO, VL_Pin);
			GPIO_ResetBits(WL_GPIO, WL_Pin);
		}
	}
}

//占空比转换
//void Motor_SetSpeed(uint8_t Speed)
//{
//	Motor_ConvertedSpeed = (Speed * 1500) / 100;
//}

void Motor_Control_Init (void)
{
	Motor_Init();
	Hall_Init();

}

// 刹车控制 自动防下溢反转
// brake_direction_sign: 1=正向换向, -1=反向换向
// 返回值：速度值  0 = 切断动力输出
#define DEBOUNCE_THRESHOLD 5	// 采样多次进行滤波
int16_t BrakeControl(int16_t SetSpeed, int8_t brake_direction_sign)
{
	uint8_t power_on_flag = 0;
	static int8_t counter = 0;
	SetSpeed = SetSpeed / 2;		// 刹车力度系数调节（整数运算减少浮点开销）

	// 防抖
	// Hall_Direction 与当前刹车换向方向相反时切断动力输出
	if (Hall_Direction != 0 && (Hall_Direction * brake_direction_sign < 0))
	{
		// 切断动力
		counter++;
		if (counter >= DEBOUNCE_THRESHOLD)
		{
			counter = DEBOUNCE_THRESHOLD;  // 防止溢出
			power_on_flag = 0;
		}
		else
		{
			power_on_flag = 1;
		}
	}
	else
	{
		// 输出动力
		counter--;
		if (counter <= 0)
		{
			counter = 0;  // 防止下溢
			power_on_flag = 1;
		}
		else
		{
			power_on_flag = 0;
		}
	}

	// 结算动力输出
	int outputSpeed = 0 - SetSpeed;
	outputSpeed = calculate_min_slip_duty() - (outputSpeed);

	if (outputSpeed < 1)		outputSpeed = 1;
	else if (outputSpeed > MOTOR_ARR)	outputSpeed = MOTOR_ARR;

	if (power_on_flag == 1)		return outputSpeed;
	else						return 0;
}

// 方向切换滞回计数 避免零速附近在两种刹车算法间频繁抖动  增大->抖动减小响应减慢
#define REVERSE_BRAKE_COUNT		200

// 反向滑行时使用正向输出进行刹车（不使用弱场最小滑行占空比）
static int16_t BrakeControl_Reverse(int16_t SetSpeed)
{
	// SetSpeed < 0（刹车命令），将整段刹车力度线性映射到 0~10
	int32_t brake_mag = 0 - SetSpeed;
	if (brake_mag < 0) brake_mag = 0;
	else if (brake_mag > MOTOR_ARR) brake_mag = MOTOR_ARR;

	// 四舍五入映射到 0~10，避免“仅截断到10”导致前段分辨率丢失
	int16_t outputSpeed = (int16_t)((brake_mag * 10 + (MOTOR_ARR / 2)) / MOTOR_ARR);

	// 限制输出范围 1~MOTOR_ARR
	if (outputSpeed < 1)		outputSpeed = 1;
	else if (outputSpeed > MOTOR_ARR)	outputSpeed = MOTOR_ARR;

	return outputSpeed;
}

// 根据当前换相方向与霍尔位置，立即执行六步换相
// 幂等：同一导通相重复设置无副作用，可安全地在霍尔中断与主循环中重复调用
void Motor_Commutation(void)
{
	if (Motor_comm_dir == 0 || HallSensor.Position == 0)
	{
		Motor_Set(0);	// 关闭所有输出
		return;
	}

	// 换相角：前进用渐变后的实际值 Adv_Fwd_Now（驱动↔弱场平滑过渡，消除切换瞬间换相突变）；
	// 后退弱场用标准对齐角 COMM_ALIGN_REV、驱动用 Comm_Adv_Rev（后退场景少，不渐变）
	int offset;
	if (Motor_comm_dir == -1)
	{
		offset = -Adv_Fwd_Now;
	}
	else
	{
		offset = Motor_weak_field ? COMM_ALIGN_REV : Comm_Adv_Rev;
	}
	// 再模 6 归一化到 1~6
	int MotorPosition = ((HallSensor.Position + offset - 1 + 6) % 6) + 1;

	Motor_Set(MotorPosition);
}

//控制旋转方向
void Motor_Direction(int Direction)
{
	if (Motor_hallError && (APP_time - Motor_hallError_trigger_time > HALL_ERROR_RESET_MS)) {
		Motor_hallError = 0;
	}

	if (HallSensor.Position == 0)
	{
		Motor_Set(0);	// 如果霍尔位置异常，切断动力输出
		Motor_comm_dir = 0;
		if (Motor_hallError == 0) {
			Motor_hallError = 1;
			Motor_hallError_trigger_time = APP_time;
			Log_add(LOG_ERROR, "Motor Direction Error.");
		}
		return;
	}
	else
	{
		// 记录当前换相方向，并立即兜底换相一次（静止起步/状态切换时保证即时响应）
		Motor_comm_dir = (Direction > 0) ? 1 : (Direction < 0 ? -1 : 0);
		Motor_Commutation();
	}
}


// 一阶低通滤波器
// 用于速度控制
#define FILTER_ALPHA		0.03f	// 正常一阶低通滤波系数（越大越跟手，越小越平滑）
#define FILTER_ALPHA_FAST	0.30f	// 收油门/滑行时的快速响应系数
#define ZERO_RAMP_MS		100		// 松油门后占空比平滑归零时长(ms)，保持驱动换相，避免反向/咯噔
float Motor_filter_prev = 0;
uint8_t isCoasting = 0;		// 当前正在滑行标志位
static float setSpeed_filter(float input)
{
	input = input * ((MOTOR_ARR + 1) / 100);

	// 检测符号变化（正变负或负变正）
    static float last_input = 0.0f;
    int sign_change = 0;
    // 判断是否发生符号变化
    if ((last_input >= 0.0f && input < 0.0f) ||
        (last_input <= 0.0f && input > 0.0f)) {
        sign_change = 1;
    }
    last_input = input;
    // 如果检测到符号变化，重置滤波器状态
    if (sign_change) {
        Motor_filter_prev = 0.0f;
    }

	// 先按正常系数计算滤波输出
	// 原代码此处 output 初始化为 0、且未赋值即拿去与 input 比较（恒为 0），导致“收油门快速响应”分支对正油门永远不生效，已修复
	float output = FILTER_ALPHA * input + (1 - FILTER_ALPHA) * Motor_filter_prev;

	// 如果收油门 立刻终止加速：杆量在下降(input < 上次输出)且输出仍高于杆量时，加大系数使输出尽快降到杆量
	if (!sign_change && input < Motor_filter_prev && output > input)
	{
		output = FILTER_ALPHA_FAST * input + (1 - FILTER_ALPHA_FAST) * Motor_filter_prev;
	}

	// 如果当前正在滑行 则加大滤波系数使输出尽快达到杆量 防止滑行时增加杆量不跟手问题
	if (isCoasting == 1)
	{
		output = FILTER_ALPHA_FAST * input + (1 - FILTER_ALPHA_FAST) * Motor_filter_prev;
	}

	// 注意：杆量为 0（松油门）的情况已在 Motor_Control 开头单独处理（弱场滑行），
	// 本滤波器只处理非零油门/刹车的平滑。

	Motor_filter_prev = output;
    return output;
}

// 换相角渐变：驱动(目标=Comm_Adv_Fwd) ↔ 弱场滑行/刹车(目标=COMM_ALIGN_FWD 标准对齐) 之间平滑过渡
// 避免 F4↔对齐角 瞬间换相时刻突变产生的"咯噔"冲击
static void Motor_AdvGradient(void)
{
	static uint32_t adv_last_step_time = 0;
	int8_t adv_target = Motor_weak_field ? COMM_ALIGN_FWD : Comm_Adv_Fwd;
	if (Adv_Fwd_Now != adv_target)
	{
		if (APP_time - adv_last_step_time >= ADV_STEP_MS)
		{
			adv_last_step_time = APP_time;
			Adv_Fwd_Now += (adv_target > Adv_Fwd_Now) ? 1 : -1;
		}
	}
	else
	{
		adv_last_step_time = APP_time;
	}
}

//控制旋转方向和速度
/* ================= 母线电流限幅 =================
 * 通过 R39 采样电阻实时检测母线电流，负载过大（载人、上坡、急加速）时
 * 自动压低占空比，把电流限制在保险丝额定值以下，防止烧保险。
 * 仅作用于油门支路，刹车/松手滑行不受影响。
 */
#define CURRENT_LIMIT_MAX		8.0f	// 限流阈值(A)，低于10A保险丝留足余量
#define CURRENT_LIMIT_RECOVER	6.5f	// 恢复阈值(A)，滞回防止占空比来回抖动
#define CURRENT_LIMIT_BACKOFF	4		// 超流时每个控制周期回退的占空比
#define CURRENT_LIMIT_RISE		1		// 电流回落后每个控制周期允许上升的占空比
#define CURRENT_LIMIT_DUTY_MIN	30		// 限流期间的最低占空比，防止完全失速后卡死

uint8_t Motor_currentLimiting = 0;		// 1=正在限流（可用于上报/调试）

// 对油门输出占空比做电流限幅，返回限幅后的占空比
static int16_t Motor_current_limit(int16_t target_duty)
{
	static int16_t limited_duty = 0;
	float current = adc_get_current_cached();

	if (current > CURRENT_LIMIT_MAX)
	{
		// 超流：快速回退占空比
		Motor_currentLimiting = 1;
		limited_duty -= CURRENT_LIMIT_BACKOFF;
		if (limited_duty < CURRENT_LIMIT_DUTY_MIN) limited_duty = CURRENT_LIMIT_DUTY_MIN;
	}
	else
	{
		if (target_duty < limited_duty)
		{
			// 油门减小，直接跟随目标（保证收油门响应不受限流状态影响）
			limited_duty = target_duty;
		}
		else if (current < CURRENT_LIMIT_RECOVER)
		{
			// 电流回落后缓慢恢复，避免再次冲击电流
			limited_duty += CURRENT_LIMIT_RISE;
			if (limited_duty > target_duty) limited_duty = target_duty;
		}
		if (limited_duty <= target_duty && current < CURRENT_LIMIT_RECOVER)
		{
			Motor_currentLimiting = 0;
		}
	}

	if (limited_duty < 0) limited_duty = 0;
	return limited_duty;
}

void Motor_Control(int inputSpeed)
{
	static uint8_t reverse_brake_active = 0;		// 反向刹车激活标志位
	static uint8_t reverse_dir_detect_cnt = 0;		// 反向方向检测计数
	static uint8_t forward_dir_detect_cnt = 0;		// 正向方向检测计数
	static uint32_t coast_start_time = 0;			// 松手滑行起始时刻(ms)
	static uint16_t coast_start_duty = 0;			// 松手滑行起始占空比（松手前最后驱动值）

	// ===== 松手（油门归零）：占空比平滑降到 0，全程保持前进换相方向与驱动超前角 F4 =====
	// 之前用"弱场滑行"（换相角归零/对齐 + 弱场占空比）试图让电机自由滑行，
	// 但方波无电流采样的硬件上"零转矩对齐角"难以精确标定，Position-0/Position-2 都导致松手反向转。
	// 正确做法：不切弱场、不改换相角，驱动占空比平滑降到 0 后全关，靠惯性 + 齿槽转矩自然减速。
	if (inputSpeed == 0)
	{
		Motor_weak_field = 0;	// 保持驱动超前角 F4，不切弱场

		if (!isCoasting)
		{
			isCoasting = 1;
			coast_start_time = APP_time;
			coast_start_duty = Motor_Speed;	// 松手前最后的驱动占空比
		}

		uint32_t elapsed = APP_time - coast_start_time;
		// 占空比平滑降到 0；或转速已接近 0 则提前全关（避免占空比残值把电机锁住）
		if (elapsed >= ZERO_RAMP_MS || (Hall_rollSpeed > -0.5f && Hall_rollSpeed < 0.5f))
		{
			Motor_Speed = 0;
			Motor_Direction(0);
			isCoasting = 0;
		}
		else
		{
			float k = 1.0f - (float)elapsed / (float)ZERO_RAMP_MS;
			Motor_Speed = (uint16_t)((float)coast_start_duty * k);
		}

		Motor_filter_prev = 0;	// 松手时复位驱动滤波，下次推油门从 0 平滑上升

		Motor_AdvGradient();
		return;
	}

	// ===== 油门/刹车（inputSpeed != 0）=====
	// 速度输入滤波器
	int Speed = (int) setSpeed_filter(inputSpeed);

	if (Speed > 0)			// 油门
	{
		reverse_brake_active = 0;
		reverse_dir_detect_cnt = 0;
		forward_dir_detect_cnt = 0;
		Motor_Direction(1 * DIRECTION * Motor_run_direction);
		// 电流限幅：负载过大时压低占空比，防止母线电流烧保险
		Motor_Speed = (uint16_t) Motor_current_limit((int16_t) Speed);
		isCoasting = 0;
		Motor_weak_field = 0;	// 正常驱动：用可调超前角 F4
	}
	else if (Speed < 0)		// 刹车
	{
		Motor_weak_field = 1;	// 刹车（含反向/普通）统一用标准超前角，保证制动力稳定可预测

		// 连续检测方向，避免零速附近方向抖动引发模式抖动
		if (Hall_Direction < 0)
		{
			if (reverse_dir_detect_cnt < REVERSE_BRAKE_COUNT) reverse_dir_detect_cnt++;
			forward_dir_detect_cnt = 0;
		}
		else if (Hall_Direction > 0)
		{
			if (forward_dir_detect_cnt < REVERSE_BRAKE_COUNT) forward_dir_detect_cnt++;
			reverse_dir_detect_cnt = 0;
		}
		else
		{
			reverse_dir_detect_cnt = 0;
			forward_dir_detect_cnt = 0;
		}

		// 判断反向刹车激活条件
		if (!reverse_brake_active && reverse_dir_detect_cnt >= REVERSE_BRAKE_COUNT)
		{
			reverse_brake_active = 1;
		}

		// 判断当前刹车方式
		if (reverse_brake_active)
		{
			// 反向刹车 用正向六步换向输出刹车，并且不走弱场占空比算法
			if (Hall_Direction < 0)
			{
				Motor_Direction(1 * DIRECTION * Motor_run_direction);
				Motor_Speed = BrakeControl_Reverse(Speed);
			}
			else
			{
				// 反向刹车激活后，非反向区间只切断输出，禁止回落到普通刹车导致抖动
				Motor_Direction(0);
				Motor_Speed = 0;
				// 仅在连续检测到正向后退出反向刹车模式
				if (forward_dir_detect_cnt >= REVERSE_BRAKE_COUNT)
				{
					reverse_brake_active = 0;
				}
			}
		}
		else
		{
			// 不再反向旋转时 使用弱场控制刹车
			Motor_Direction(1 * DIRECTION * Motor_run_direction);
			Motor_Speed = BrakeControl(Speed, 1);
		}
		isCoasting = 0;
	}
	else					// Speed == 0（极小油门被滤波截断，罕见兜底）
	{
		Motor_weak_field = 0;
		reverse_brake_active = 0;
		reverse_dir_detect_cnt = 0;
		forward_dir_detect_cnt = 0;
		Motor_Direction(0);
		Motor_Speed = 0;
		isCoasting = 0;
	}

	Motor_AdvGradient();
}


