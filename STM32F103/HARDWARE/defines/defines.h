
#ifndef __DEFINES_H
#define __DEFINES_H

#include "sys.h"
#include "stdio.h"
#include "stdbool.h"
#include "delay.h"

//--------------Arduino.h--------------
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define _min(a,b) ({ decltype(a) _a = (a); decltype(b) _b = (b); _a < _b? _a : _b; })
#define _max(a,b) ({ decltype(a) _a = (a); decltype(b) _b = (b); _a > _b? _a : _b; })
//********************************************

#define TELEMETRY "192.168.4.1" // Default telemetry server (first client) port 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32

// Default control terms for EVO 2
#define KP 0.32
#define KD 0.050
#define KP_THROTTLE 0.10 
#define KI_THROTTLE 0.12 
#define KP_POSITION 0.05 
#define KD_POSITION 0.65 
//#define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1 
#define KD_RAISEUP 0.16 
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#define SERVO_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2300

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE   1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 65535
#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output

/*-----PORT A-----*/
#define SETA(pin) ( *((volatile unsigned long  *)(0x42210180 + (pin<<2)))  = 1) 
#define CLRA(pin) ( *((volatile unsigned long  *)(0x42210180 + (pin<<2)))  = 0)	

/*-----PORT B-----*/
#define SETB(pin) ( *((volatile unsigned long  *)(0x42218180 + (pin<<2)))  = 1) 
#define CLRB(pin) ( *((volatile unsigned long  *)(0x42218180 + (pin<<2)))  = 0)	

/*-----PORT C-----*/
#define SETC(pin) ( *((volatile unsigned long  *)(0x42220180 + (pin<<2)))  = 1) 
#define CLRC(pin) ( *((volatile unsigned long  *)(0x42220180 + (pin<<2)))  = 0)	
	
#define RAD2GRAD 57.2957795 //  Convert radians to gradians: 180/pi
#define GRAD2RAD 0.01745329251994329576923690768489 // Convert gradians to radians: pi/180

#endif /* DEFINES_H_ */
