#ifndef __AD_H_
#define __AD_H_

extern volatile uint16_t AD_Value[4];		//这样就变成了外部可调用数组（DMA 写，必须 volatile）

void AD_Init(void);
int AD_Temp(void);
float adc_getBatteryVoltage(void);


#endif
