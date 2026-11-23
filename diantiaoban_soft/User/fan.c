/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "fan.h"
#include "APPTime.h"
#include "Serial.h"

#define FAN_ARR     (300 - 1)
#define FAN_PSC     (8 - 1)

#define FAN_RUNLOOP_PERIOD_MS       100
#define FAN_SPEED_FILTER_ALPHA      0.05f    // 一阶低通滤波系数（每100ms更新一次）

static uint8_t fan_target_speed = 0;
static float fan_actual_speed = 0.0f;
static uint32_t fan_last_run_time = 0;

static void fan_applyPwm(float speed)
{
    uint16_t ccr_value;

    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < 0.0f)
    {
        speed = 0.0f;
    }

    ccr_value = (uint16_t)((speed * FAN_ARR) / 100.0f);
    TIM_SetCompare1(TIM1, ccr_value);
}

void fan_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_Initstructure;
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_Initstructure);
    GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_Initstructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_InternalClockConfig(TIM1);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = FAN_ARR;
    TIM_TimeBaseInitStructure.TIM_Prescaler = FAN_PSC;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    fan_target_speed = 50;
    fan_actual_speed = 0.0f;
    fan_last_run_time = APP_time;
    fan_applyPwm(0);
}

// 设置风扇目标转速 speed = 0~100
void fan_setSpeed(uint8_t speed)
{
    if (speed > 100)
    {
        speed = 100;
    }
    fan_target_speed = speed;
}

// 每100ms向目标转速缓变，并更新PWM占空比
void fan_runloop(void)
{
    if (APP_time - fan_last_run_time < FAN_RUNLOOP_PERIOD_MS)
    {
        return;
    }
    fan_last_run_time = APP_time;

    fan_actual_speed = FAN_SPEED_FILTER_ALPHA * (float)fan_target_speed
                     + (1.0f - FAN_SPEED_FILTER_ALPHA) * fan_actual_speed;
    fan_applyPwm(fan_actual_speed);
}

void debug_fan_report(void)
{
    static uint32_t last_report_time = 0;
    if (APP_time - last_report_time < 50)
    {
        return;
    }
    last_report_time = APP_time;
    Serial_Printf("%d,%d\n", fan_target_speed, (uint16_t)fan_actual_speed);
}
