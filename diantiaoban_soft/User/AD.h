#ifndef __AD_H_
#define __AD_H_

extern uint16_t AD_Value[4];		//这样就变成了外部可调用数组

// 温度报警阈值设置 单位：℃
#define TEMPERATURE_LOW		    5
#define TEMPERATURE_HIGH		70
#define TEMPERATURE_ERROR		80

void AD_Init(void);
float adc_calculate_battery_voltage(void);
float adc_getTemperature(void);
void adc_update_cache_10ms(void);
float adc_get_battery_voltage_cached(void);
float adc_get_temperature_cached(void);

// 母线电流采样（R39 5mΩ + LM358 ×60 -> PA4）
void adc_current_calibrate_zero(void);
float adc_calculate_current(void);
float adc_get_current_cached(void);

#endif
