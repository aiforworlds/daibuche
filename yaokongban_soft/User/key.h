#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f10x.h"                  // Device header

extern uint8_t key_PA4_flag;
extern uint8_t key_PA5_flag;

void key_init(void);

#endif
