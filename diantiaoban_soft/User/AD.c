/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "stm32f10x.h"                  // Device header
#include "APPTime.h"

/*
PA1 = ADC_AGND
PA2 = ADC_Bat
PA4 = ADC_2
PA5 = NTC
*/



uint16_t AD_Value[4];

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
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,2,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,3,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,4,ADC_SampleTime_55Cycles5);		//选择规则组的输入通道

	//用结构体初始化ADC：
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;										//配置ADC工作模式（独立工作模式/双ADC模式）	配置成独立工作模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;								//数据对齐 （左对齐/右对齐）	配置成右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;		//外部触发转换选择，外部触发源选择	这里选择不使用外部触发，因为这里用程序触发
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;										//连续转换模式（单次转换/连续转换） ENABLE是连续，DISABLE是单次
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;													//扫描转换模式（扫描模式（多通道）/非扫描模式（单通道））ENABLE就是扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 4;																//通道数目 指定扫描模式下总共用到几个通道		//这个参数只在扫描模式有用，非扫描模式下只有第一个序列有效
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
	DMA_InitStructure.DMA_BufferSize = 4;		//缓冲区大小其实就是传输计数器 指定传输几次 在函数名处选择
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

// 一阶低通滤波器
#define Vbattery_FILTER_ALPHA		0.5f	// 一阶低通滤波系数
float battery_filter_prev = 0;
static float battery_filter(float input)
{
	float output = Vbattery_FILTER_ALPHA * input + (1 - Vbattery_FILTER_ALPHA) * battery_filter_prev;
	battery_filter_prev = output;
    return output;
}

/**
 * @brief 计算当前电池电压
 * @param adc_value ADC采样值 (0-4096)
 * @return 电池电压值（单位：V）
 */
float adc_calculate_battery_voltage(void)
{
	uint16_t adc_value = battery_filter(AD_Value[1]);

	float voltage = (float)adc_value * 3.3f / 4096;
	voltage = voltage / 15;
	voltage = voltage / 5.1f * 1005.1f;
	voltage += 0.45;		// 实际测量offset
	return voltage;
}

/* NTC电阻温度计算 */
// 温度-电阻对照表（0-120°C范围，每1°C一个数据点）
typedef struct {
    float temp;     // 温度 (°C)
    float r_min;    // 最小电阻 (kΩ)
    float r_avg;    // 平均电阻 (kΩ)
    float r_max;    // 最大电阻 (kΩ)
} temp_resistance_t;
// 从规格书提取的温度数据（0-120°C）
static const temp_resistance_t r_table[] = {
    {0, 31.908, 32.613, 33.331},
    {1, 30.341, 30.996, 31.662},
    {2, 28.860, 29.468, 30.086},
    {3, 27.460, 28.024, 28.598},
    {4, 26.136, 26.660, 27.192},
    {5, 24.883, 25.370, 25.863},
    {6, 23.698, 24.149, 24.607},
    {7, 22.576, 22.994, 23.419},
    {8, 21.513, 21.901, 22.295},
    {9, 20.507, 20.867, 21.231},

    {10, 19.554, 19.888, 20.225},
    {11, 18.649, 18.959, 19.271},
    {12, 17.792, 18.079, 18.368},
    {13, 16.980, 17.245, 17.513},
    {14, 16.209, 16.454, 16.702},
    {15, 15.477, 15.704, 15.933},
    {16, 14.783, 14.993, 15.204},
    {17, 14.123, 14.317, 14.512},
    {18, 13.497, 13.676, 13.856},
    {19, 12.902, 13.067, 13.233},

    {20, 12.337, 12.489, 12.642},
    {21, 11.799, 11.940, 12.080},
    {22, 11.288, 11.417, 11.547},
    {23, 10.802, 10.921, 11.040},
    {24, 10.340, 10.449, 10.558},
    {25, 9.900, 10.000, 10.100},
    {26, 9.472, 9.572, 9.672},
    {27, 9.066, 9.165, 9.265},
    {28, 8.679, 8.778, 8.878},
    {29, 8.311, 8.409, 8.508},

    {30, 7.960, 8.058, 8.156},
    {31, 7.627, 7.724, 7.821},
    {32, 7.309, 7.405, 7.501},
    {33, 7.006, 7.101, 7.196},
    {34, 6.717, 6.811, 6.905},
    {35, 6.441, 6.534, 6.628},
    {36, 6.179, 6.271, 6.363},
    {37, 5.928, 6.019, 6.110},
    {38, 5.690, 5.779, 5.868},
    {39, 5.461, 5.549, 5.638},

    {40, 5.244, 5.330, 5.417},
    {41, 5.036, 5.121, 5.207},
    {42, 4.837, 4.921, 5.006},
    {43, 4.648, 4.730, 4.813},
    {44, 4.467, 4.547, 4.629},
    {45, 4.294, 4.373, 4.453},
    {46, 4.128, 4.206, 4.285},
    {47, 3.970, 4.046, 4.124},
    {48, 3.818, 3.893, 3.970},
    {49, 3.674, 3.747, 3.822},

    {50, 3.534, 3.607, 3.680},
    {51, 3.402, 3.473, 3.545},
    {52, 3.275, 3.345, 3.416},
    {53, 3.154, 3.222, 3.291},
    {54, 3.037, 3.104, 3.172},
    {55, 2.926, 2.991, 3.058},
    {56, 2.819, 2.883, 2.948},
    {57, 2.717, 2.780, 2.843},
    {58, 2.619, 2.680, 2.743},
    {59, 2.524, 2.585, 2.646},

    {60, 2.434, 2.493, 2.553},
    {61, 2.348, 2.405, 2.464},
    {62, 2.265, 2.321, 2.379},
    {63, 2.185, 2.240, 2.297},
    {64, 2.109, 2.163, 2.218},
    {65, 2.035, 2.088, 2.142},
    {66, 1.965, 2.017, 2.070},
    {67, 1.897, 1.948, 2.000},
    {68, 1.832, 1.882, 1.932},
    {69, 1.769, 1.818, 1.868},

    {70, 1.709, 1.757, 1.806},
    {71, 1.652, 1.698, 1.746},
    {72, 1.596, 1.642, 1.688},
    {73, 1.543, 1.588, 1.633},
    {74, 1.492, 1.535, 1.580},
    {75, 1.442, 1.485, 1.529},
    {76, 1.395, 1.437, 1.479},
    {77, 1.349, 1.390, 1.432},
    {78, 1.305, 1.345, 1.386},
    {79, 1.263, 1.302, 1.342},

    {80, 1.222, 1.261, 1.300},
    {81, 1.183, 1.221, 1.259},
    {82, 1.145, 1.182, 1.220},
    {83, 1.109, 1.145, 1.182},
    {84, 1.074, 1.109, 1.145},
    {85, 1.040, 1.075, 1.110},
    {86, 1.008, 1.041, 1.076},
    {87, 0.976, 1.009, 1.043},
    {88, 0.946, 0.978, 1.011},
    {89, 0.917, 0.948, 0.981},

    {90, 0.889, 0.920, 0.951},
    {91, 0.862, 0.892, 0.923},
    {92, 0.836, 0.865, 0.895},
    {93, 0.810, 0.839, 0.869},
    {94, 0.786, 0.814, 0.843},
    {95, 0.762, 0.790, 0.818},
    {96, 0.740, 0.767, 0.794},
    {97, 0.718, 0.744, 0.771},
    {98, 0.697, 0.722, 0.749},
    {99, 0.676, 0.701, 0.728},

    {100, 0.656, 0.681, 0.706},
    {101, 0.637, 0.662, 0.687},
    {102, 0.619, 0.643, 0.667},
    {103, 0.601, 0.624, 0.648},
    {104, 0.584, 0.607, 0.630},
    {105, 0.567, 0.589, 0.612},
    {106, 0.551, 0.573, 0.595},
    {107, 0.536, 0.557, 0.579},
    {108, 0.520, 0.541, 0.563},
    {109, 0.506, 0.526, 0.547},

    {110, 0.492, 0.512, 0.532},
    {111, 0.478, 0.498, 0.518},
    {112, 0.465, 0.484, 0.504},
    {113, 0.452, 0.471, 0.490},
    {114, 0.440, 0.458, 0.477},
    {115, 0.428, 0.446, 0.464},
    {116, 0.416, 0.434, 0.452},
    {117, 0.405, 0.422, 0.440},
    {118, 0.394, 0.411, 0.429},
    {119, 0.384, 0.400, 0.417},

    {120, 0.373, 0.390, 0.406}
};
#define TABLE_SIZE (sizeof(r_table) / sizeof(r_table[0]))
// NTC从电阻值到摄氏度计算
float resistance_to_temperature(float resistance_kohm)
{
	// 检查参数是否合法
	if (resistance_kohm < 0)
	{
		return -99;
	}

    int i;

    // 边界检查
    if (resistance_kohm >= r_table[0].r_avg) {
        return 0.0f; // 低于最小温度
    }
    if (resistance_kohm <= r_table[TABLE_SIZE-1].r_avg) {
        return 120.0f; // 高于最大温度
    }

    // 查找电阻所在的区间
    for (i = 0; i < TABLE_SIZE - 1; i++) {
        if (resistance_kohm <= r_table[i].r_avg && resistance_kohm >= r_table[i+1].r_avg) {
            // 线性插值计算温度
            float r1 = r_table[i].r_avg;
            float r2 = r_table[i+1].r_avg;
            float t1 = r_table[i].temp;
            float t2 = r_table[i+1].temp;

            return t1 + (t2 - t1) * (resistance_kohm - r1) / (r2 - r1);
        }
    }

    return -99.0f; // 错误值
}

#define TEMPERATURE_AVERAGE_SIZE	100

// 获取当前NTC温度值 单位:摄氏度
float adc_getTemperature(void)
{
	uint16_t new_sample = AD_Value[3];
	// 滑动窗口平均值计算
	static uint16_t buffer[TEMPERATURE_AVERAGE_SIZE] = {0};
	static uint32_t sum = 0;
	static uint16_t index = 0;
	static uint8_t is_buffer_filled = 0;
	if (is_buffer_filled == 1) {
		sum -= buffer[index];
	}
	sum += new_sample;
	buffer[index] = new_sample;
	index = (index + 1) % TEMPERATURE_AVERAGE_SIZE;
	if (!is_buffer_filled && index == 0) {
		is_buffer_filled = 1;
	}
	uint16_t current_count = is_buffer_filled ? TEMPERATURE_AVERAGE_SIZE : index;
	uint16_t temperature_adValue = (uint16_t)(sum / current_count);

	float voltage = (float)temperature_adValue * 3.3f / 4096;
										// 电阻值推算过程
										// V = 3.3 * (R / (R+220))
										// V / 3.3 = R / (R + 220)
										// 3.3 / V = (R + 220)/R = 1 + 220/R
										// (3.3 / V) - 1 = 220/R
										// R = 220 / ((3.3 / V) - 1)
	float ntc_value = 220000.0f / ((3.3f / voltage) - 1.0f);
	ntc_value = ntc_value / 1000;		// 将单位从ohm转为kohm
	// 根据阻值计算温度
	float temperature = resistance_to_temperature(ntc_value);

	return temperature;
}

/* ================= 母线电流采样 =================
 * 硬件链路：R39(5mΩ采样电阻) -> LM358运放放大60倍 -> "ADC_AGND"网络 -> PA1(CH1通道, AD_Value[0])
 * （20261123勘误：原理图上运放输出网络名叫ADC_AGND，实际接PA1；PA4网络悬空未接）
 * 量程：0-10A 对应运放输出 0-3V（LM358 单电源 +5V，>10A 饱和）
 * 换算：I = (Vout - 零点偏置) / (增益 * 采样电阻) = (Vout - offset) / 0.3
 */
#define CURRENT_SHUNT_OHM		0.005f		// R39 采样电阻 5mΩ
#define CURRENT_AMP_GAIN		60.0f		// LM358 放大倍数
#define CURRENT_FILTER_ALPHA	0.2f		// 一阶低通滤波系数

static float current_filter_prev = 0;
static float current_zero_offset_v = 0;

// 零点校准：主循环启动早期（电机未运行）调用，用10ms窗口平均零点偏置电压
void adc_current_calibrate_zero(void)
{
	uint32_t start = APP_time;
	uint32_t sum = 0;
	uint32_t n = 0;

	// 20261123修复：上电初期ADC/DMA尚未出数，原版校准采到全0导致偏置恒为0。
	// 以电池通道(AD_Value[1])出现非0数据作为"ADC已就绪"标志，最多等500ms
	while (AD_Value[1] == 0 && APP_time - start < 500);

	start = APP_time;
	while (APP_time - start < 10)
	{
		sum += AD_Value[0];
		n++;
	}
	if (n == 0) n = 1;
	current_zero_offset_v = (float)(sum / n) * 3.3f / 4096.0f;
	current_filter_prev = 0;
}

// 计算当前母线电流（单位：A）
float adc_calculate_current(void)
{
	// 20261123修复：LM358电流放大输出实际经"ADC_AGND"网络接PA1(CH1)=AD_Value[0]。
	// 原来读的AD_Value[2]是PA4——原理图上该网络悬空，静态读到~1.84V，虚标出6.1A假电流
	float voltage = (float)AD_Value[0] * 3.3f / 4096.0f;
	voltage -= current_zero_offset_v;

	float current = voltage / (CURRENT_AMP_GAIN * CURRENT_SHUNT_OHM);
	if (current < 0) current = 0;		// 运放单电源供电无负压，负值视为噪声

	// 一阶低通滤波
	current = CURRENT_FILTER_ALPHA * current + (1 - CURRENT_FILTER_ALPHA) * current_filter_prev;
	current_filter_prev = current;
	return current;
}

static float cached_battery_voltage = 0.0f;
static float cached_temperature = 0.0f;
static float cached_current = 0.0f;
static uint8_t sensor_cache_ready = 0;

void adc_update_cache_10ms(void)
{
	static uint32_t last_update_time = 0;

	if (sensor_cache_ready && (APP_time - last_update_time < 10))
	{
		return;
	}

	last_update_time = APP_time;
	cached_battery_voltage = adc_calculate_battery_voltage();
	cached_temperature = adc_getTemperature();
	cached_current = adc_calculate_current();
	sensor_cache_ready = 1;
}

float adc_get_battery_voltage_cached(void)
{
	if (!sensor_cache_ready)
	{
		cached_battery_voltage = adc_calculate_battery_voltage();
		cached_temperature = adc_getTemperature();
		cached_current = adc_calculate_current();
		sensor_cache_ready = 1;
	}
	return cached_battery_voltage;
}

float adc_get_temperature_cached(void)
{
	if (!sensor_cache_ready)
	{
		cached_battery_voltage = adc_calculate_battery_voltage();
		cached_temperature = adc_getTemperature();
		cached_current = adc_calculate_current();
		sensor_cache_ready = 1;
	}
	return cached_temperature;
}

float adc_get_current_cached(void)
{
	if (!sensor_cache_ready)
	{
		cached_battery_voltage = adc_calculate_battery_voltage();
		cached_temperature = adc_getTemperature();
		cached_current = adc_calculate_current();
		sensor_cache_ready = 1;
	}
	return cached_current;
}
