#ifndef __BLDC_MOTOR_H_
#define __BLDC_MOTOR_H_

extern float Motor_filter_prev;
extern uint8_t Motor_hallError;
extern uint8_t Motor_currentLimiting;		// 1=正在母线限流（调试用）
extern volatile int8_t Motor_run_direction;
extern volatile int8_t Comm_Adv_Fwd;
extern volatile int8_t Comm_Adv_Rev;

// float Motor_calculate_Ke(uint8_t Duty_min_hold);
uint16_t calculate_min_slip_duty(void);

void Motor_Init(void);
void Motor_Set(uint8_t Position);
//void Motor_SetSpeed(uint8_t Speed);
void Motor_Control_Init (void);
void Motor_Direction(int Direction);
void Motor_Commutation(void);
void Motor_Control(int inputSpeed);

#endif

