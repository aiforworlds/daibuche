#ifndef _HALL_H_
#define _HALL_H_

// 控制电机转动方向  取值 1 / -1
#define DIRECTION -1

/* ===== 旋转方向符号归一化（2026-11-24，全工程唯一的“正方向”定义）=====
 * Hall_Direction 的原式计算 = 霍尔序列方向 × DIRECTION × Motor_run_direction，
 * 结果是 +1 还是 -1 取决于电机三相线/霍尔线的实际接法，光看代码推不出来。
 * 这个符号同时决定三件事，所以任何一处搞错都会“串味”：
 *   (1) 上报速度的正负（Hall_speed = 转速 × Hall_Direction）
 *   (2) 刹车时的“反向保护”（实际转向与换相方向相反 → 必须开路，否则反着给电会自维持成反转）
 *   (3) 刹车分支的“倒溜检测”（Hall_Direction < 0 才判为反向）
 *
 * 本工程的做法：**运行期自学习**，见 Hall_LearnForward()。
 * 只采信“从静止起步”那一次（上一拍 <1rps、这一拍 >2rps），把当时的 Hall_Direction 符号记成“前进”，
 * 之后统一用 Hall_GetDirCmd()：+1 = 与当前指令方向一致，-1 = 相反，0 = 未知。
 * 坡上倒溜 / 堵转抖动不会把符号学反。
 * HALL_FWD_FALLBACK_SIGN 只是“还没学到的第一秒”用的兜底值，不影响后续。
 */
#define HALL_FWD_FALLBACK_SIGN	(-1)

extern volatile int8_t Hall_fwd_sign;		// 学习结果：+1 / -1（“前进”对应的 Hall_Direction 符号）
extern volatile uint8_t Hall_fwd_learned;	// 1 = 已学到

int8_t Hall_GetDirCmd(void);				// 归一化方向：+1=按指令方向转，-1=反着转，0=未知
void Hall_LearnForward(uint8_t driving);	// 在“正在给油门”时调用，用于学习符号

typedef struct {					//定义结构体类型
    int HallA;
    int HallB;
    int HallC;
		int Position;
} HallSensorTypeDef;
extern volatile HallSensorTypeDef HallSensor;
extern volatile uint16_t Hall_Count;
extern volatile float Hall_speed;
extern volatile float Hall_rollSpeed;
extern volatile int Hall_Direction;


void Hall_Init(void);
void Hall_GetPosition(void);
void Hall_Position(void);
float Get_Speed(void);
void HallSpeed_Timer(void);




#endif
