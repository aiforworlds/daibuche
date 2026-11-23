#ifndef __RC_CMD_H_
#define __RC_CMD_H_

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "Serial_3.h"
#include "Serial.h"

typedef struct __attribute__((packed)){     //取消字节填充 解决字节对齐问题
	uint8_t start_flag;				//包头
	uint32_t timestamp;				//发包时间
	uint8_t cmd_id;					//协议号

    uint8_t data[];

	// uint8_t end_flag;			//包尾
	// uint8_t crc8;				//crc校验
} RC_pack_t;

typedef struct {
	// 速度
	int16_t speed_target;
	int16_t speed_current;

	// 里程
	uint32_t milometer;     // 最大值：4,294,967,295
	uint8_t milometer_demcimal;

	// 时间
	uint32_t time_send;
	uint32_t time_receive;

} RC_SpeedControl_t;

enum cmd_status {
	normal,
	error,
	warning
};

typedef struct {

	float speed;		// 速度
	float battery;		// 电量
	float temperature;	// 温度
	float current;		// 母线电流(A) 20261123新增：R39+LM358 采样
	uint8_t status;		// 状态
	int8_t direction;	// 方向 1正向 -1反向 0停止

} RC_heartbeat_uplink_t;

typedef struct {

	float speed;		// 速度
	uint8_t status;		// 状态
	int8_t direction;	// 方向 1正向 -1反向 0停止

} RC_heartbeat_downlink_t;

#define CMD_SPEED_CONTROL			0x01
#define CMD_SPEED_REPORT			0x02
#define CMD_SPEED_REPORT_REQ		0x03

#define CMD_TIME_CONTROLLOR_RPO		0x11
#define CMD_TIME_MOTOR_RPO			0x12
#define CMD_TIME_SYNCHRONOUS		0x13

#define CMD_TEMPERATURE_RPO			0x20
#define CMD_MILEAGE_RPO 			0x21
#define CMD_RUNTIME_RPO 			0x22

// 心跳包协议
#define CMD_HEARTBEAT_DOWNLINK		0x30		// 遥控->滑板
#define CMD_HEARTBEAT_UPLINK		0x31		// 滑板->遥控

// 换相超前角调节（遥控->电调） data = {int8_t fwd, int8_t rev}
#define CMD_COMM_ADV_SET			0x40



extern uint8_t Cmd_Arr[18];
extern char Cmd_String[40];

void RC_Pin_Init(void);
void RC_cmd(uint8_t choice);
void RC_setSysInit(void);
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length);
void RC_send(const uint8_t *data, uint8_t cmd_id, size_t size);
uint8_t RC_status_pack(uint8_t battery_warning, uint8_t temperature_warning, uint8_t temperature_error, uint8_t watchdog_error, uint8_t hall_error);


// ******************* log报错码 *******************

// 报错码注册
// 通用报错码
#define	LOG_NORMAL		0x00
#define	LOG_WARNING		0x01
#define LOG_ERROR		0x02
// 时间与通信报错
#define	LOG_NTP_ERROR	0x11
#define	LOG_NTP_SUCCESS	0x12
#define	LOG_HEARTBEAT	0x13

// log信息长度限制
#define MAX_LOG_MESSAGE 100
// log条数
#define MAX_LOG_COUNT	100

typedef struct {
	uint32_t app_time;				// 报错时间
	uint8_t num;					// 报错码
	char message[MAX_LOG_MESSAGE];	// 报错内容
} log_t;

void Log_add(uint8_t error_number, const char *message);
void Log_print(void);
void Log_reset(void);


#endif
