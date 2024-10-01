#ifndef __CONTROL_H
#define __CONTROL_H	 

#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "defines.h"
#include "globals.h"



float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);
void setMotorSpeedM1(int16_t tspeed);
void setMotorSpeedM2(int16_t tspeed);

extern uint32_t cnt2, cnt3;
extern uint32_t cnt2timer;

#endif
