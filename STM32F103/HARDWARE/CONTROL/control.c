#include "control.h"

// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

//-----------------------------------------------

uint32_t cnt2, cnt3;
uint32_t cnt2timer;

// PD controller implementation(Proportional, derivative). DT in seconds

float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd){
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki){
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM){
  float output;
  float P;

  P = constrain(Kpp * (float)(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * (float)(speedM);
  return (output);
}

// Interrupt TIM2
void TIM2_IRQHandler(void){
//	if(TIM_GetFlagStatus(TIM2, TIM_FLAG_CC2) != RESET){
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET){
			TIM2->CNT=0;	// set CNT = 0 to preload		
			if (dir_M1 != 0){	
				SETA(5);
				if (dir_M1 > 0) 
					steps1--;
				else 
					steps1++;
				CLRA(5);
			}			
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		}
//		TIM_ClearFlag(TIM2,TIM_FLAG_CC2);				
//	}
}


// Interrupt TIM3
void TIM3_IRQHandler(void){
//	if(TIM_GetFlagStatus(TIM3, TIM_FLAG_CC1) != RESET){
		if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET){
			TIM3->CNT = 0;	// set CNT = 0 to preload	
			if (dir_M2 != 0){
				SETA(1);
				if (dir_M2 > 0)	
					steps2--;
				else	
					steps2++;
				CLRA(1);					
			}
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);		
		}
//		TIM_ClearFlag(TIM3,TIM_FLAG_CC1);		
//	}
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed){
  long timer_period;
  int16_t speed;
  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 25; // 1/8 Microstepping
#endif

  if (speed == 0){
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0){
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    // DIR Motor 1 (Forward)
		SETA(6); // PA5 = 1
  }
  else{
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
		// Dir Motor 1
		CLRA(6);  // PA5= 0
  }
  if (timer_period > ZERO_SPEED){   // Check for minimun speed (maximun period without overflow)
		timer_period = ZERO_SPEED;
	}

  TIM2->CCR2 = timer_period;
  // Check  if we need to reset the timer...
  if (TIM2->CNT > TIM2->CCR2){
    TIM2->CNT = 0;
	}
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed){
  long timer_period;
  int16_t speed;
  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 25; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    CLRA(4);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    SETA(4);  // DIR Motor 2
  }
  if (timer_period > 65535){   // Check for minimun speed (maximun period without overflow)
		timer_period = ZERO_SPEED;
	}

  TIM3->CCR1 = timer_period;
  // Check  if we need to reset the timer...
  if (TIM3->CNT > TIM3->CCR1){
    TIM3->CNT = 0;
	}
}

