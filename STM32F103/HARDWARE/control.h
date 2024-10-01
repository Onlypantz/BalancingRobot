#ifndef __CONTROL_H
#define __CONTROL_H	 

#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "main.h"
#include "servo.h"
#include "delay.h"


static float Kp = KP;
static float Kd = KD;
static float Kp_thr = KP_THROTTLE;
static float Ki_thr = KI_THROTTLE;
static float Kp_user = KP;
static float Kd_user = KD;
static float Kp_thr_user = KP_THROTTLE;
static float Ki_thr_user = KI_THROTTLE;
static float Kp_position = KP_POSITION;
static float Kd_position = KD_POSITION;
static bool newControlParameters = false;
static int16_t position_error_sum_M1;
static int16_t position_error_sum_M2;
static float PID_errorSum;
static float PID_errorOld = 0;
static float PID_errorOld2 = 0;
static float setPointOld = 0;
static float target_angle;
static int16_t throttle;
static float steering;
static float max_throttle = MAX_THROTTLE;
static float max_steering = MAX_STEERING;
static float max_target_angle = MAX_TARGET_ANGLE;
static float control_output;

// position control
static volatile int32_t steps1;
static volatile int32_t steps2;
static int32_t target_steps1;
static int32_t target_steps2;
static int16_t motor1_control;
static int16_t motor2_control;

static int16_t speed_M1, speed_M2;        // Actual speed of motors
static int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
static int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
static int16_t actual_robot_speed_Old;
static float estimated_speed_filtered;    // Estimated robot speed


float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);
void setMotorSpeedM1(int16_t tspeed);
void setMotorSpeedM2(int16_t tspeed);


uint32_t cnt2, cnt4;
uint32_t cnt2timer;

#endif
