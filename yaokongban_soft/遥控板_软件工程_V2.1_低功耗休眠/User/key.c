/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "key.h"

// 低功耗休眠：按键触发 EXTI 时复位空闲计时（main.c 用到）
void Sleep_ReportActivity(void);

uint8_t key_PA4_flag = 0;
uint8_t key_PA5_flag = 0;


void key_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IPU;				//配置为模拟输入 ADC的专属模式，此时GPIO失效，防止干扰
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);

    // 3. 配置EXTI（外部中断）
    // 3.1 将PA4和PA5映射到EXTI Line 4和Line 5
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);

    // 3.2 初始化EXTI
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line4 | EXTI_Line5; // 选择中断线4和5
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     // 中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发（按键按下）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 4. 配置NVIC（嵌套向量中断控制器）
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置中断优先级分组

    // 4.1 配置EXTI4中断
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;                     // PA4对应的中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;             // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                    // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 4.2 配置EXTI9_5中断
    // 注意：EXTI5到EXTI9是共享一个中断通道的
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;                   // PA5对应的中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;             // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                    // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI4_IRQHandler(void)
{
    // 检查是否是Line4上的中断
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        // 清除中断标志位，防止重复进入
        EXTI_ClearITPendingBit(EXTI_Line4);

        // 设置按键标志位
        key_PA4_flag = 1;

        // 低功耗：按键活动，复位空闲计时
        Sleep_ReportActivity();
    }
}

void EXTI9_5_IRQHandler(void)
{
    // 检查是否是Line5上的中断（因为PA5映射到Line5）
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        // 清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line5);

        // 设置按键标志位
        key_PA5_flag = 1;

        // 低功耗：按键活动，复位空闲计时
        Sleep_ReportActivity();
    }
}
