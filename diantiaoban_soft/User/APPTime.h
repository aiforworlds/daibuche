#ifndef _APPTIME_H_
#define _APPTIME_H_

typedef struct __attribute__((packed)){
	uint32_t t1;    //请求发送时间
    uint32_t t2;    //请求接收时间
    uint32_t t3;    //响应发送时间
    uint32_t t4;    //响应接收时间
} NTPtime_t;

extern volatile uint32_t APP_time;
extern volatile uint32_t NTP_time;
extern volatile uint8_t heartBeat_SendFlag;

void APPTimer_Init(void);


#endif
