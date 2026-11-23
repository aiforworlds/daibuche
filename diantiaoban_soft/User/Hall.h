#ifndef _HALL_H_
#define _HALL_H_

// 控制电机转动方向  取值 1 / -1
#define DIRECTION -1

typedef struct {					//定义结构体类型
    int HallA;
    int HallB;
    int HallC;
		int Position;
} HallSensorTypeDef;
extern volatile HallSensorTypeDef HallSensor;
extern volatile uint16_t Hall_Count;
extern volatile float Hall_speed;
extern volatile float Hall_rollSpeed;
extern volatile int Hall_Direction;


void Hall_Init(void);
void Hall_GetPosition(void);
void Hall_Position(void);
float Get_Speed(void);
void HallSpeed_Timer(void);




#endif
