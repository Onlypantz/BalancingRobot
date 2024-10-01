#ifndef __SERVO_H
#define __SERVO_H

#include "globals.h"
#include "defines.h"
#include "delay.h"

extern int battery;
extern int8_t  dir_M1, dir_M2; 

void Config_Servo(void);
void Write_Deg(u8 degree);
void BROBOT_moveServo(int pwm);
int BROBOT_readBattery(bool first_time);
void TIM_Init(void);
void TIM1_Init(void);
uint32_t get_timer_clock_frequency (void);




#endif
