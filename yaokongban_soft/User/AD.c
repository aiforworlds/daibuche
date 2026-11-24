/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

#include "stm32f10x.h"                  // Device header

/*

*/



// 2026-11-24：DMA 目标数组必须是 volatile（DMA 在后台写、CPU 读）。
// 非 volatile 时，"读-判断-再读"的等待循环会被 GCC 优化成死循环（见 main.c joy_neutral_learn 注释）。
volatile uint16_t AD_Value[4];

void AD_Init(void)		//AD初始化函数
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);		//开启ADC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//开启GPIOA时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);		//打开DMA时钟,是AHB总线的设备，所以要用AHB开启时钟函数

	//ADCCLK配置：
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);		//这里选择6分频。可以设置2、4、6、8分频，分别是（Div2、4、6、8）  PCLK2是APB2时钟的意思
																			//分频之后 ADCCLK=72MHz/6=12MHz
	//配置GPIO：
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AIN;				//配置为模拟输入 ADC的专属模式，此时GPIO失效，防止干扰
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,1,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
//	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,3,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
//	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,4,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道

	//用结构体初始化ADC：
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;										//配置ADC工作模式（独立工作模式/双ADC模式）	配置成独立工作模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;								//数据对齐 （左对齐/右对齐）	配置成右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;		//外部触发转换选择，外部触发源选择	这里选择不使用外部触发，因为这里用程序触发
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;										//连续转换模式（单次转换/连续转换） ENABLE是连续，DISABLE是单次
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;													//扫描转换模式（扫描模式（多通道）/非扫描模式（单通道））ENABLE就是扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 2;																//通道数目 指定扫描模式下总共用到几个通道		//这个参数只在扫描模式有用，非扫描模式下只有第一个序列有效
	ADC_Init(ADC1,&ADC_InitStructure);																		//配置成单次转换、非扫描模式

//DMA配置：
	//配置DMA参数：
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;		//外设站点的起始地址 可以在函数名的括号里选择地址
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		//外设站点的数据宽度 （Byte=uint8_t,HalfWord=uint16_t,Word=uint32_t）
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设站点的是否自增
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;		//存储器站点的起始地址 可以在函数名的括号里选择地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		//存储器站点的数据宽度
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//存储器站点的是否自增
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;		//传输方向	(DST=外设站点作为目的地，传输方向就是从存储器到外设站点。SRC=外设站点作为源头，从外设站点到存储器)
	DMA_InitStructure.DMA_BufferSize = 2;		//缓冲区大小其实就是传输计数器 指定传输几次 在函数名处选择
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//传输模式，是否自动重装 （Circular=循环模式，传输计数器自动重装。Normal=正常模式，不自动重装，自减到0就停下）
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		//是否是存储器到存储器，就是选择硬件触发还是软件触发(ENABLE=软件触发。DISABLE=硬件触发)
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;		//优先级 (VeryHigh,High,Medium,Low)
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);		//选择DMA1的CH1

	DMA_Cmd(DMA1_Channel1,ENABLE);		//使能DMA,使其开始转运

	ADC_DMACmd(ADC1,ENABLE);		//开启DMA触发信号

	//开启ADC电源：
	ADC_Cmd(ADC1,ENABLE);

	//校准ADC：
	ADC_ResetCalibration(ADC1);		//复位校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);		//返回复位校准状态	如果未完成校准就一直在while循环里等待
	ADC_StartCalibration(ADC1);		//启动校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);//获取校准状态 等待校准完成

	ADC_SoftwareStartConvCmd(ADC1,ENABLE);										//软件触发转换函数 用于触发ADC进行转换

}


int AD_Temp(void)
{
	uint16_t Temp[1000];
	uint32_t ADD = 0;
	for (int i = 0; i < 1000;i++)
	{
		Temp[i] = AD_Value[1];
	}
	for (int i = 0; i < 1000;i++)
	{
		ADD = ADD + Temp[i];
	}
	return ADD / 1000;
}

// 获取电池电压
#define BAT_AVERAGE_FILTER_SIZE 100
uint32_t bat_Average_sum = 0;
uint16_t bat_Average_buffer[BAT_AVERAGE_FILTER_SIZE] = {0};
uint8_t bat_Average_index = 0;
uint8_t bat_Average_count = 0;
float adc_getBatteryVoltage(void)
{
	uint16_t adc_value = AD_Value[1];

	// 平均计算
	bat_Average_sum -= bat_Average_buffer[bat_Average_index];		// 减去旧值
    bat_Average_buffer[bat_Average_index] = adc_value;	// 存入新值
    bat_Average_sum += adc_value;
    bat_Average_index++;					// 更新索引
    if (bat_Average_index >= BAT_AVERAGE_FILTER_SIZE)
        bat_Average_index = 0;
    if (bat_Average_count < BAT_AVERAGE_FILTER_SIZE)	// 更新有效数据个数（前 100 次）
        bat_Average_count++;
    adc_value =  (uint16_t)(bat_Average_sum / bat_Average_count);

	float voltage = (float)adc_value * 3.3f / 4096;
	voltage = voltage / 36.0f * (10.0f + 36.0f);		// 分压电阻 10k 36k
	return voltage;
}
