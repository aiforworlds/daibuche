#ifndef FAN_H
#define FAN_H

#include "stm32f10x.h"                  // Device header

void fan_init(void);
void fan_setSpeed(uint8_t speed);
void fan_runloop(void);
// void debug_fan_report(void);

#endif
