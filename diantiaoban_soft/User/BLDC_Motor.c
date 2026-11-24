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
// 2026-11-24：改用“反电动势绝对值 + 缓存母线电压”，并去掉每轮重复推进 battery_filter 的副作用。
// 返回值 = 刚好抵消反电动势（零转矩）的占空比码值，刹车分支以它为基准往下压。
#define KE		0.130f
#define Delta_V	0.1f
uint16_t calculate_min_slip_duty(void)
{
	float rps = Hall_rollSpeed;			// 圈/秒，正负都要（制动时可能被拖反）
	if (rps < 0.0f) rps = -rps;

	float RPM = rps * 60.0f;
	float V_bus = adc_get_battery_voltage_cached();		// 用 10ms 缓存值，避免多次调用把滤波器时间常数压掉
	if (V_bus < 5.0f) V_bus = 5.0f;						// 母线异常时给个下限，防止除零/满占空比

	float BEMF = KE * RPM * K_rpm_to_rads;				// 单位 V
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
volatile uint8_t Motor_brake3 = 0;		// 1=正在三相短路制动（V2.6）：此时霍尔中断不再按六步换相写 SD#

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
	// 2026-11-24：六路寄存器/引脚写必须是原子的。霍尔中断抢占优先级 0，随时能插进来，
	// 被撕开后会出现“一相高边导通 + 两相低边并联”的直通风险（V2.3 审查 P0-1）。
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

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

	__set_PRIMASK(primask);		// 恢复进入前的中断状态
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

/* ================= 刹车：短路制动（V2.5，2026-11-24 重写）=================
 * 台架实测结论（用户反馈，两条关键证据）：
 *   · V2.2_限流 旧版：下拉到底 → 占空比≈1（几乎就是短路）→ 轮子**立即停住、不反转**；
 *   · V2.4：把占空比按 KE 公式压到 20km/h 的 29% → 轮子**反转**、骑行刹不住。
 * 说明本电机在“弱场角”（Motor_weak_field=1 → COMM_ALIGN_FWD=2）下，只要给电压就是**反向转矩**，
 * 只有占空比≈0（两相下桥全通 = 绕组短路）才是纯制动。KE 公式算出的“零转矩点”在该角度上不成立。
 *
 * V2.5 改成“短路制动”（角独立、不会反转）：
 *   1) 刹车用**驱动同一个换相角**（F4，Motor_weak_field=0）。本工程的松手滑行本来就是
 *      “F4 + 占空比压到 0”，实测从不反转 —— 证明 F4 下降低占空比是制动方向。
 *   2) 刹车量把占空比从“进入刹车那一刻的巡航占空比”按转速比例压向 0：
 *        duty = ref_duty × (|rps| / ref_rps) × (1 − 刹车%/100)
 *      满刹车 → 0（短路，最强）；半刹车 → 一半。
 *      用实测巡航占空比当零转矩基准，比 KE 公式可靠（不受 KE 误差、母线压降、换相角误差影响）。
 *   3) 转子已经在反着转时不再给电压（duty=0 = 短路）；短路对反转同样是制动，不会自维持。
 *   4) 母线 > 41.5V 时也压到 0：短路不进电池，不会把母线顶高。
 *
 * 安全：满刹车 = 两相下桥全通（绕组短路），相电流≈反电动势/绕组电阻（20km/h 约 60~70A），
 *       这正是旧版一直在用的工况。若觉得太暴力/发热，把 BRAKE_DUTY_MIN 抬到 30~60（10%~20%）。
 */
#define BRAKE_VBUS_MAX		41.5f		// 母线过压门限：超过就只短接（不向电池回馈）
#define BRAKE_DUTY_MIN		0			// 满刹车允许压到的最低占空比码值（0 = 全短路）
#define BRAKE_DUTY_MAX		MOTOR_ARR

static uint8_t  brake_engaged = 0;		// 1 = 正在刹车（基准只采一次）
static uint16_t brake_ref_duty = 0;		// 基准：进入刹车时的巡航占空比
static float    brake_ref_rps  = 1.0f;	// 基准：进入刹车时的转速
static int16_t  plug_duty_state = 0;	// 反接制动软爬升状态（BrakeControl_Reset 清零，下次从 0 起步）

// 刹车分支每轮调用；离开刹车时由 BrakeControl_Reset() 清状态
// 入参 SetSpeed < 0（量纲是占空比码值），返回输出占空比码值
int16_t BrakeControl(int16_t SetSpeed)
{
	float rps_now = Hall_rollSpeed;
	if (rps_now < 0.0f) rps_now = -rps_now;

	if (!brake_engaged)
	{
		brake_engaged = 1;
		brake_ref_duty = Motor_Speed;						// 巡航占空比（零转矩基准）
		uint16_t ke_duty = calculate_min_slip_duty();		// 从滑行直接进刹车：用 KE 估算兜底
		if (ke_duty > brake_ref_duty) brake_ref_duty = ke_duty;
		if (brake_ref_duty < 30) brake_ref_duty = 30;
		brake_ref_rps = rps_now;
		if (brake_ref_rps < 1.0f) brake_ref_rps = 1.0f;
	}

	// 刹车量 0~100
	int32_t brake_pct = (-(int32_t)SetSpeed * 100) / MOTOR_ARR;
	if (brake_pct < 1)	 brake_pct = 1;
	if (brake_pct > 100) brake_pct = 100;

	// 占空比随转速线性下降（零转矩点随转速成正比），再按刹车量压到 0
	float ratio = rps_now / brake_ref_rps;
	if (ratio > 1.0f) ratio = 1.0f;
	if (ratio < 0.0f) ratio = 0.0f;

	int32_t duty = (int32_t)((float)brake_ref_duty * ratio);
	duty = duty * (100 - brake_pct) / 100;

	// 已经在倒转 → 只短接、不再加电压（短路对反转同样是制动，不会自维持）
	if (Hall_GetDirCmd() < 0)	duty = 0;

	// 母线过高 → 短路，不向电池回馈
	if (adc_get_battery_voltage_cached() > BRAKE_VBUS_MAX)	duty = 0;

	if (duty < BRAKE_DUTY_MIN)	duty = BRAKE_DUTY_MIN;
	if (duty > BRAKE_DUTY_MAX)	duty = BRAKE_DUTY_MAX;
	return (int16_t)duty;
}

// 离开刹车（油门/滑行/停止）时清基准
void BrakeControl_Reset(void)
{
	brake_engaged = 0;
	Motor_brake3 = 0;		// 退出三相短路制动
	plug_duty_state = 0;	// 反接制动软爬升状态清零，下次进入从 0 起步
}

/* ================= 反接制动（主动反向驱动，V2.6，2026-11-24 新增）=================
 * 三相短路制动（能耗制动）的制动力 ∝ 转速，低速段塌陷、永远刹不到 0。
 * 反接制动用「反向换相 + 受控反向占空比」主动把转子往反方向拽，制动力由刹车行程
 * 和母线电流限幅决定，与转速无关，能一路刹到 0 甚至轻微倒车。
 *
 * 安全约束（缺一不可）：
 *   1) 反向占空比每次从 0 软爬升（独立静态状态，不复用油门限流器的残留值），并用母线电流限幅；
 *   2) 母线过压(>BRAKE_VBUS_MAX) → 切回三相短路，停止向电池回馈；
 *   3) 近零速(|rps|<BRAKE_PLUG_STOP_RPS) → 切回三相短路，防止反向自维持成倒车。
 *
 * 方向：前进换相 = Motor_Direction(1*DIRECTION*run_dir)，反接取反 = Motor_Direction(-1*DIRECTION*run_dir)。
 * 由调用方（Motor_Control 刹车分支）负责先 Motor_Direction(反向) 再喂入本函数算出的占空比。
 */
#define BRAKE_PLUG_MIN_PCT	70			// 刹车行程 ≥ 此值(%) 才进入反接制动（<此值走两相短路，柔和）
#define BRAKE_PLUG_MAX_DUTY	120			// 反接反向占空比上限（码值，约 40% 满占空比），防止反向冲击过大
#define BRAKE_PLUG_STOP_RPS	0.8f		// 低于此转速(rps)视为接近停车，退出反接改短路（防倒车自维持）
#define BRAKE_PLUG_RISE		6			// 反接占空比每个控制周期软爬升量（码值），限制反向冲击
#define BRAKE_PLUG_I_MAX	7.0f		// 反接制动允许的最大母线电流(A)，低于保险丝留余量
#define BRAKE_PLUG_I_BACK	3			// 超流时每个控制周期回退的占空比

// 入参 SetSpeed < 0，返回反接制动的反向占空比码值（0~BRAKE_PLUG_MAX_DUTY，0=退化为短路）
static int16_t BrakeControl_Plug(int16_t SetSpeed)
{
	// 刹车行程 70%~100% 线性映射到目标反向占空比 0~BRAKE_PLUG_MAX_DUTY
	int32_t brake_pct = (-(int32_t)SetSpeed * 100) / MOTOR_ARR;
	if (brake_pct < BRAKE_PLUG_MIN_PCT) brake_pct = BRAKE_PLUG_MIN_PCT;
	if (brake_pct > 100) brake_pct = 100;

	int32_t target = (int32_t)((int64_t)BRAKE_PLUG_MAX_DUTY * (brake_pct - BRAKE_PLUG_MIN_PCT) / (100 - BRAKE_PLUG_MIN_PCT));
	if (target < 0) target = 0;
	if (target > BRAKE_PLUG_MAX_DUTY) target = BRAKE_PLUG_MAX_DUTY;

	// 母线过压 → 只短接，不向电池回馈（反向驱动会持续泵升母线电压）
	if (adc_get_battery_voltage_cached() > BRAKE_VBUS_MAX) { plug_duty_state = 0; return 0; }

	// 近零速 → 只短接，防止反向自维持成倒车
	float rps = Hall_rollSpeed;
	if (rps < 0.0f) rps = -rps;
	if (rps < BRAKE_PLUG_STOP_RPS) { plug_duty_state = 0; return 0; }

	// 软爬升 + 母线电流限幅（反接电流同样流入母线采样电阻，用同一套限流阈值保护保险丝）
	float current = adc_get_current_cached();
	if (current > BRAKE_PLUG_I_MAX)
	{
		plug_duty_state -= BRAKE_PLUG_I_BACK;		// 超流快速回退
		if (plug_duty_state < 0) plug_duty_state = 0;
	}
	else if (plug_duty_state < target)
	{
		plug_duty_state += BRAKE_PLUG_RISE;		// 软爬升到目标
		if (plug_duty_state > target) plug_duty_state = target;
	}
	else if (plug_duty_state > target)
	{
		plug_duty_state = target;					// 收刹车行程立即跟随
	}

	return plug_duty_state;
}

// 三相下桥全通（零矢量）= 三相短路制动（V2.6）
// 比“两相短路”制动强约 2 倍：三个绕组各自流过 e/R 的电流，耗散功率 = 3·e²/R；
// 两相短路只有 2 个绕组、电流 e_ll/(2R)，平均功率 ≈ 1.5·e²/R。
// 与换相角、霍尔位置、母线电压、电池电流都无关 —— 只取决于转速与绕组电阻。
void Motor_BrakeShort3(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	TIM_SetCompare2(TIM3, 0);		// UH
	TIM_SetCompare3(TIM3, 0);		// VH
	TIM_SetCompare4(TIM3, 0);		// WH
	GPIO_SetBits(UL_GPIO, UL_Pin);	// 三路下桥全开
	GPIO_SetBits(VL_GPIO, VL_Pin);
	GPIO_SetBits(WL_GPIO, WL_Pin);
	__set_PRIMASK(primask);
}

// 根据当前换相方向与霍尔位置，立即执行六步换相
// 幂等：同一导通相重复设置无副作用，可安全地在霍尔中断与主循环中重复调用
void Motor_Commutation(void)
{
	// V2.6：三相短路制动期间，霍尔中断/main 都只维持“三相下桥全通”，不按六步换相写 SD#
	if (Motor_brake3)
	{
		Motor_BrakeShort3();
		return;
	}

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
	static uint32_t coast_start_time = 0;			// 松手滑行起始时刻(ms)
	static uint16_t coast_start_duty = 0;			// 松手滑行起始占空比（松手前最后驱动值）

	// ===== 松手（油门归零）：占空比平滑降到 0，全程保持前进换相方向与驱动超前角 F4 =====
	// 之前用"弱场滑行"（换相角归零/对齐 + 弱场占空比）试图让电机自由滑行，
	// 但方波无电流采样的硬件上"零转矩对齐角"难以精确标定，Position-0/Position-2 都导致松手反向转。
	// 正确做法：不切弱场、不改换相角，驱动占空比平滑降到 0 后全关，靠惯性 + 齿槽转矩自然减速。
	if (inputSpeed == 0)
	{
		Motor_weak_field = 0;	// 保持驱动超前角 F4，不切弱场
		BrakeControl_Reset();	// 松手 = 离开刹车，清掉刹车基准

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
		BrakeControl_Reset();			// 离开刹车，清掉刹车基准
		Motor_Direction(1 * DIRECTION * Motor_run_direction);
		// 电流限幅：负载过大时压低占空比，防止母线电流烧保险
		Motor_Speed = (uint16_t) Motor_current_limit((int16_t) Speed);
		// 方向符号自学习：轮子已按指令方向转起来（>2rps 且占空比 >6.7%）时记下“前进”的符号
		if (Motor_Speed > 20) Hall_LearnForward(1);
		isCoasting = 0;
		Motor_weak_field = 0;	// 正常驱动：用可调超前角 F4
	}
	else if (Speed < 0)		// 刹车：两相短路（轻）+ 反接制动（重刹，能真正刹停）
	{
		Motor_weak_field = 0;	// 刹车用驱动同一个换相角 F4，不切弱场角（切了会给反向转矩）

		// 刹车行程 → 0~100
		int32_t brake_pct = (-(int32_t)Speed * 100) / MOTOR_ARR;
		if (brake_pct > 100) brake_pct = 100;

		if (brake_pct >= BRAKE_PLUG_MIN_PCT)	// 重刹：反接制动（反向换相 + 受控反向占空比）
		{
			Motor_brake3 = 0;
			int16_t plug_duty = BrakeControl_Plug(Speed);
			if (plug_duty <= 0)
			{
				// 母线过压 或 近零速 → 退化为三相短路（不向电池回馈、防倒车）
				Motor_brake3 = 1;
				Motor_BrakeShort3();
			}
			else
			{
				// 反接：反向换相（前进取反），占空比已在 BrakeControl_Plug 内完成软爬升+母线电流限幅
				Motor_Direction(-1 * DIRECTION * Motor_run_direction);
				Motor_Speed = (uint16_t) plug_duty;
			}
		}
		else								// 轻刹：原来的两相短路 + 占空比比例（柔和，不突兀）
		{
			Motor_brake3 = 0;
			Motor_Direction(1 * DIRECTION * Motor_run_direction);
			Motor_Speed = (uint16_t) BrakeControl(Speed);
			Motor_AdvGradient();
			Motor_Commutation();			// 立即生效：低速时下一个霍尔沿可能几十 ms 才来
		}
		isCoasting = 0;
	}
	else					// Speed == 0（极小油门被滤波截断，罕见兜底）
	{
		Motor_weak_field = 0;
		BrakeControl_Reset();
		Motor_Direction(0);
		Motor_Speed = 0;
		isCoasting = 0;
	}

	Motor_AdvGradient();
}


