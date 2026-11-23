#ifndef __FLASHWRITE_H_
#define __FLASHWRITE_H_

#define USE_AREA 11                      // 当前已使用的范围 最大512对应一页flash
extern uint16_t Store_Data[USE_AREA];		//需要存储到Flash的数据

uint32_t Flash_ReadWord(uint32_t Address);
uint16_t Flash_ReadHalfWord(uint32_t Address);
uint8_t Flash_ReadByte(uint32_t Address);
void Flash_EraseAllPages(void);
void Flash_ErasePage(uint32_t PageAddress);
void Flash_WriteWord(uint32_t Address, uint32_t Data);
void Flash_WriteHalfWord(uint32_t Address, uint32_t Data);
void Store_Init(void);
void Store_Save(void);

void Store_runTime(uint16_t Plus_Minutes);
void Store_runTime_0(void);

#endif
