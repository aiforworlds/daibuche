/**
 * 《电动滑板开源代码》
 * 软件作者：bilibili 守辰宇
 * 软件开源文档：
 * https://shouchenyu.feishu.cn/wiki/PxnSwaRRRixkaokr8oBcpYotnyb
 */

 #include "stm32f10x.h"                  // Device header
#include "FlashWrite.h"
#include "APPTime.h"

/*

最后一页的地址是：0x0800 FC00				一般程序会使用前面的Flash，我们可以从最后一页开始使用Flash。

*/

uint16_t Store_Data[USE_AREA];		//需要存储到Flash的数据  512位对应闪存的一页1024个字节


//读取指定地址的一个字（32位）
uint32_t Flash_ReadWord(uint32_t Address)
{
	return *((__IO uint32_t *)(Address));
}

//读取指定地址的半个字（16位）
uint16_t Flash_ReadHalfWord(uint32_t Address)
{
	return *((__IO uint16_t *)(Address));
}

//读取指定地址的一个字节（8位）
uint8_t Flash_ReadByte(uint32_t Address)
{
	return *((__IO uint8_t *)(Address));
}

//擦除所有页
void Flash_EraseAllPages(void)					//(一页是1kb) (c8t6一共有64k)
{
    uint32_t primask = __get_PRIMASK();		// 保存当前中断状态
    __disable_irq();		// 关闭总中断

	FLASH_Unlock();
	FLASH_EraseAllPages();
	FLASH_Lock();

    __set_PRIMASK(primask);		//恢复之前的中断状态
}

//擦除指定页
void Flash_ErasePage(uint32_t PageAddress)
{
	uint32_t primask = __get_PRIMASK();		// 保存当前中断状态
    __disable_irq();		// 关闭总中断

	// --- 补偿开始 ---
	uint32_t cnt_before = TIM4->CNT; // 读取操作前的硬件计数器值

	FLASH_Unlock();
	FLASH_ErasePage(PageAddress);
	FLASH_Lock();

	// --- 补偿结束 ---
    uint32_t cnt_after = TIM4->CNT; // 读取操作后的硬件计数器值
    // 计算丢失的ticks数 (处理溢出)
    uint32_t elapsed_ticks;
    if (cnt_after >= cnt_before) {
        elapsed_ticks = cnt_after - cnt_before;
    } else {
        elapsed_ticks = (TIM4->ARR + 1) - cnt_before + cnt_after;
    }
    APP_time += elapsed_ticks; // 补偿到软件计时器上

	__set_PRIMASK(primask);		//恢复之前的中断状态
}

//写入一个字
void Flash_WriteWord(uint32_t Address, uint32_t Data)
{
	uint32_t primask = __get_PRIMASK();		// 保存当前中断状态
    __disable_irq();		// 关闭总中断

	FLASH_Unlock();
	FLASH_ProgramWord(Address, Data);
	FLASH_Lock();

	__set_PRIMASK(primask);		//恢复之前的中断状态
}

//写入半个字
void Flash_WriteHalfWord(uint32_t Address, uint32_t Data)
{
	uint32_t primask = __get_PRIMASK();		// 保存当前中断状态
    __disable_irq();		// 关闭总中断

	FLASH_Unlock();
	FLASH_ProgramHalfWord(Address, Data);
	FLASH_Lock();

	__set_PRIMASK(primask);		//恢复之前的中断状态
}

//初始化  上电时将Flash数据存入Store_Data数组
void Store_Init(void)
{
	if (Flash_ReadHalfWord(0x0800FC00) != 0x1218)								//用一个标志位判断flash里是否已经存有数据
	{
		Flash_ErasePage(0x0800FC00);															//第一次写入前先擦除整页
		Flash_WriteHalfWord(0x0800FC00,0x1218);										//写入标志位
		for (uint16_t i = 1;i < USE_AREA;i++)													//用循环将所有内容写为0（因为擦除页后会将所有置为F）（注意从1开始，因为第一位是标志位1218）
		{
			Flash_WriteHalfWord(0x0800FC00 + i * 2, 0x0000);				//一个半字占2个位置，所以*2
		}
	}
	for (uint16_t i = 0;i < USE_AREA;i++)														//将Flash里所有内容存入数组（包括标志位）
	{
		Store_Data[i] = Flash_ReadHalfWord(0x0800FC00 + i * 2);
	}
}

//写入数据
void Store_Save(void)
{
	Store_Mileage(0);
	Store_runTime(0);

	Flash_ErasePage(0x0800FC00);
	Store_Data[0] = 0x1218;
	for (uint16_t i = 0;i < USE_AREA;i++)		//将数组里所有内容存入Flash（包括标志位）
	{
		Flash_WriteHalfWord(0x0800FC00 + i * 2,Store_Data[i]);
	}
}

//Flash写入里程数据		使用数组区间：1-11
void Store_Mileage(uint16_t Plus_Meter)
{															//uint16_t最大65535，uint32_t最大4,294,967,295
	uint16_t Meter = 0;
	uint16_t Kilometer = 0;
	uint16_t KKilometer = 0;

	// 存储 "Mi" -> 0x4D69
	Store_Data[1] = ('M' << 8) | 'i';
	// 存储 "le" -> 0x6C65
	Store_Data[2] = ('l' << 8) | 'e';
	// 存储 "ag" -> 0x6167
	Store_Data[3] = ('a' << 8) | 'g';
	// 存储 "e:" -> 0x653A
	Store_Data[4] = ('e' << 8) | ':';

	Meter = Store_Data[5];
	Store_Data[6] = ('m' << 8) | '\0';
	Kilometer = Store_Data[7];
	Store_Data[8] = ('k' << 8) | 'm';
	KKilometer = Store_Data[9];
	Store_Data[10] = ('k' << 8) | 'k';
	Store_Data[11] = ('m' << 8) | '\0';

	// --- 进位逻辑 ---
	uint32_t total_meters = (uint32_t)KKilometer * 1000000 + (uint32_t)Kilometer * 1000 + Meter;
	total_meters += Plus_Meter;
	// 重新计算并拆分
	KKilometer = total_meters / 1000000;
	uint32_t remaining = total_meters % 1000000;
	Kilometer = remaining / 1000;
	Meter = remaining % 1000;

	Store_Data[5] = Meter;
	Store_Data[7] = Kilometer;
	Store_Data[9] = KKilometer;
}
void Store_Mileage_0(void)		//清零总里程
{
	Store_Data[5] = 0;
	Store_Data[7] = 0;
	Store_Data[9] = 0;
}

//写入运行时间		使用数组区间：12-21
void Store_runTime(uint16_t Plus_Minutes)
{
	Store_Data[12] = ('R' << 8) | 'u';
	Store_Data[13] = ('n' << 8) | 'T';
	Store_Data[14] = ('i' << 8) | 'm';
	Store_Data[15] = ('e' << 8) | '\0';

    // --- 读取旧的时间 ---
    uint16_t current_thousand_hours = Store_Data[16];
    uint16_t current_hours = Store_Data[17];
	Store_Data[18] = ('h' << 8) | '\0';
    uint16_t current_minutes = Store_Data[19];
	Store_Data[20] = ('m' << 8) | 'i';
	Store_Data[21] = ('n' << 8) | '\0';

	if (Plus_Minutes != 0)
	{
		// --- 将所有时间统一转换为“分钟”进行计算 ---
		// 1. 计算小时部分对应的总分钟数
		uint32_t current_total_minutes = ((uint32_t)current_thousand_hours * 1000 + current_hours) * 60;
		// 2. 加上之前存储的分钟数
		current_total_minutes += current_minutes;

		// --- 累加新的分钟数 ---
		uint32_t new_total_minutes = current_total_minutes + Plus_Minutes;

		// --- 将总分钟数转换回“千小时, 小时, 分钟”的格式 ---
		uint16_t new_thousand_hours = new_total_minutes / (1000 * 60);
		uint16_t remaining_minutes_after_thousand = new_total_minutes % (1000 * 60);

		uint16_t new_hours = remaining_minutes_after_thousand / 60;
		uint16_t new_minutes = remaining_minutes_after_thousand % 60;

		// --- 将新时间存回数组 ---
		Store_Data[16] = new_thousand_hours;
		Store_Data[17] = new_hours;
		Store_Data[19] = new_minutes; // 存储的是0-59的分钟数
	}

}
void Store_runTime_0(void)
{
	Store_Data[16] = 0;
    Store_Data[17] = 0;
    Store_Data[19] = 0;
}
