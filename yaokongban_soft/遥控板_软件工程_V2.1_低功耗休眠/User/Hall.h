#ifndef _HALL_H_
#define _HALL_H_

typedef struct {					//定义结构体类型
    int HallA;
    int HallB;
    int HallC;
		int Position;
} HallSensorTypeDef;
extern HallSensorTypeDef HallSensor;
extern int Hall_Count;
extern float Hall_RotationSpeed;

void SpeedTimer_Init(void);
void Hall_Init(void);
void Hall_GetPosition(void);
void Hall_Position(void);
int Hall_Movement(void);
float Get_Speed(void);




#endif
