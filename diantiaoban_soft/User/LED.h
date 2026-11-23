#ifndef _LED_H_
#define _LED_H_


enum LED_State {
	ON = 1,
	OFF = 0
};

enum LED_choose {
	LED_GREEN = 1,
	LED_BLUE = 2,
	LED_RED = 3,
	LED_ERROR = 0
};

enum LED_status {
	off,
	on,
	flicker
};

void LED_Init(void);
void LED_Set(int a,enum LED_State chose);
void LED_Turn(int a);
void LED_Control(enum LED_choose LED, enum LED_status status);
void LED_Timer(void);



#endif
