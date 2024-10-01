/**
****
Enable: PC13
Step 1: PA1 - TIM2
Step 2: PA5 - TIM3
Dir 1:	PA4 
Dir 2:	PA6

Servo: PB8 - TIM4

I2C PB10- SCL, PB11 - SDA

USART: TX-PA9, RX-PA10
USART-ESP: TX A2, RX A3
****
**/

#include "defines.h"
#include "globals.h"
#include "usart.h"	
#include "MPU6050.h"
#include "servo.h"
#include "osc.h"
#include "network.h"
#include "control.h"
#include "stdbool.h"


long timer_old_tick;
long timer_value_tick;
float dt_tick;


//-----------------

//#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x))) // constrain function - limite value in the range: >=a & b<=

//-----------------test --------------------------------------
//volatile uint32_t ticks= 0; //4294967295 ~ 70 minutes

#define PRINT 
#define wifi

/**********************/

void Init_Wifi(void);
void TIM1_Init(void);



int answer;

int main(void)
{ 
	// float MPU_sensor_angle;
	delay_init();
	//systickInit(1000000); // 1us->interrupt
#ifdef PRINT 	
	USARTx_Init(USART1, Pins_PA9PA10, 115200);
#endif
//	USART2_Init(115200);
#ifdef wifi 	
	USARTx_Init(USART2, Pins_PA2PA3, 115200);
  //flaguart =0;
#endif
  
#ifdef PRINT		
	printf("USART OK\r\n");
	printf("clock: %d\r\n",SystemCoreClock);
#endif
	OSC_init();
	
#ifdef PRINT
	printf("OSC_OK\r\n");
#endif
  
	delay(1000);
	MPU6050_Setup();
	delay_ms(500);

/*
	...wifi...
*/
#ifdef wifi	
  Init_Wifi();	
#endif

#ifdef PRINT
	printf("ROBOT ");
  printf("Don't move for 10 sec...");
	printf("DON'T MOVE!\r");
#endif
	
	MPU6050_calibrate();

// 
#ifdef wifi  
  //do { answer = Send_AT("AT+CIPSEND\r\n", ">", 20);} while(answer==0); // Start transmission (transparent mode)
  //flaguart =1;

#endif	

#ifdef PRINT
  printf("Wifi Ok!\r\n");
#endif

	Config_Servo();
	BROBOT_moveServo(SERVO_AUX_NEUTRO);
	
	TIM_Init(); // TIMER MOTOR

	// Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
		BROBOT_moveServo(SERVO_AUX_NEUTRO + 40);
    delay_ms(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    BROBOT_moveServo(SERVO_AUX_NEUTRO - 40);
    delay_ms(200);
  }
	BROBOT_moveServo(SERVO_AUX_NEUTRO);
	
	
#ifdef PRINT
	printf("Start...");
#endif
	timer_old_tick= micros();
	
  while(1)
	{    
		OSC_MsgReadN();  // Read UDP OSC messages
    
		timer_value_tick = micros();
		
		// New IMU data?
		if (MPU6050_newData())
		{
			MPU6050_read_3axis();
			loop_counter++;
			slow_loop_counter++;
			dt_tick = (timer_value_tick - timer_old_tick)*0.000001;
			timer_old_tick = timer_value_tick;

			angle_adjusted_Old = angle_adjusted;
			// Get new orientation angle from IMU (MPU6050)
			float MPU_sensor_angle = MPU6050_getAngle(dt_tick);
			angle_adjusted = MPU_sensor_angle + angle_offset;
			if ((MPU_sensor_angle>-15)&&(MPU_sensor_angle<15))
				angle_adjusted_filtered = angle_adjusted_filtered*0.99 + MPU_sensor_angle*0.01;
 //       angle_adjusted_filtered = angle_adjusted_filtered*0.9934 + MPU_sensor_angle*0.0066;
				
	#if DEBUG==1
			Serial.print(dt_tick);
			Serial.print(" ");
			Serial.print(angle_offset);
			Serial.print(" ");
			Serial.print(angle_adjusted);
			Serial.print(",");
			Serial.println(angle_adjusted_filtered);
	#endif
			//Serial.print("\t");

			// We calculate the estimated robot speed:
			// Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
			actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward  

			int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
			int16_t estimated_speed = -actual_robot_speed + angular_velocity;
			estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

	#if DEBUG==2
			Serial.print(angle_adjusted);
			Serial.print(" ");
			Serial.println(estimated_speed_filtered);
	#endif

			if (positionControlMode)
			{
				// POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
				motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
				motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

				// Convert from motor position control to throttle / steering commands
				throttle = (motor1_control + motor2_control) / 2;
				throttle = constrain(throttle, -190, 190);
				steering = motor2_control - motor1_control;
				steering = constrain(steering, -50, 50);
			}

			// ROBOT SPEED CONTROL: This is a PI controller.
			//    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
			target_angle = speedPIControl(dt_tick, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
			target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output


	#if DEBUG ==3
			printf(angle_adjusted);
			printf(" ");
			printf(estimated_speed_filtered);
			printf(" ");
			printf(target_angle);
	#endif

			// Stability control (100Hz loop): This is a PD controller.
			//    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
			//    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
			control_output += stabilityPDControl(dt_tick, angle_adjusted, target_angle, Kp, Kd);
			control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

			// The steering part from the user is injected directly to the output
			motor1 = control_output + steering;
			motor2 = control_output - steering;

			// Limit max speed (control output)
			motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
			motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

			int angle_ready;
			if (OSCpush[0])     // If we press the SERVO button we start to move
				angle_ready = 82; //82
			else
				angle_ready = 74;  // Default angle
			if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
			{
				// NORMAL MODE
				CLRC(13);
				// NOW we send the commands to the motors
				setMotorSpeedM1(motor1);
				setMotorSpeedM2(motor2);						
			}
			else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
			{
				SETC(13);
				setMotorSpeedM1(0);
				setMotorSpeedM2(0);
				PID_errorSum = 0;  // Reset PID I term
				Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
				Kd = KD_RAISEUP;
				Kp_thr = KP_THROTTLE_RAISEUP;
				Ki_thr = KI_THROTTLE_RAISEUP;
				
				// RESET steps
				steps1 = 0;
				steps2 = 0;
				positionControlMode = false;
				OSCmove_mode = false;
				throttle = 0;
				steering = 0;
			}

			// Push1 Move servo arm
			if (OSCpush[0])  // Move arm
			{       
				if (angle_adjusted > -39)
					BROBOT_moveServo(SERVO_MIN_PULSEWIDTH);
				else
					BROBOT_moveServo(SERVO_MAX_PULSEWIDTH);
			}else{
				BROBOT_moveServo(SERVO_AUX_NEUTRO); 
			}

			// Servo2
      // BROBOT_moveServo2(SERVO2_NEUTRO + (OSCfader[2] - 0.5) * SERVO2_RANGE);
			// Normal condition?
			if ((angle_adjusted < 56) && (angle_adjusted > -56))
			{
				Kp = Kp_user;            // Default user control gains
				Kd = Kd_user;
				Kp_thr = Kp_thr_user;
				Ki_thr = Ki_thr_user;
			}
			else    // We are in the raise up procedure => we use special control parameters
			{
				Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
				Kd = KD_RAISEUP;
				Kp_thr = KP_THROTTLE_RAISEUP;
				Ki_thr = KI_THROTTLE_RAISEUP;
			}
		} // End of new IMU data

		  // Medium loop 7.5Hz
		if (loop_counter >= 15)
		{
			loop_counter = 0;
			// Telemetry here?
			#ifdef wifi 
			#if TELEMETRY_ANGLE==1
 			char auxS[25];
			int ang_out = constrain((int)(angle_adjusted * 10),-900,900);
				sprintf(auxS, "$tA,%+04d\r\n", ang_out); 
				USART_Puts(USART2,auxS);
      #endif
			#endif
			#if TELEMETRY_DEBUG==1
					char auxS[50];
					sprintf(auxS, "$tD,%d,%d,%ld\r\n", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
					USART_Puts(USART2,auxS);
			#endif
		} // End of medium loop
		else if (slow_loop_counter >= 100) // 1Hz
		{
			slow_loop_counter = 0;
					// Read  status
			#if TELEMETRY_BATTERY==1
					BatteryValue = (BatteryValue + BROBOT_readBattery(false)) / 2;
					sendBattery_counter++;
					if (sendBattery_counter >= 3) { //Every 3 seconds we send a message
						sendBattery_counter = 0;
            #ifdef PRINT
						//printf("B %i",BatteryValue);
            #endif
						#ifdef wifi 
						char auxS[25];
						//sprintf(auxS, "$tB,%04d\r\n", BatteryValue);
						//USART_Puts(USART2,auxS);
            #endif
					}
			#endif
		}  // End of slow loop
	}
}

void Init_Wifi(void){
 // USART_Puts(USART2, "AT+RST\r\n");
  delay_ms(1000);
	//printf("WIFI_SETUP\r\n");
	delay_ms(200);
//	do { answer = Send_AT("AT\r\n", "OK", 100);} while(answer==0);
//	do { answer = Send_AT("ATE0\r\n", "OK", 100);} while(answer==0);
//	//do { answer = Send_AT("AT+RST\r\n", "OK", 20); // ESP Wifi module RESET
//  //do { answer = Send_AT("AT+GMR\r\n", "OK", 100);} while(answer==0);
//	delay_ms(50);
//	do { answer = Send_AT("AT+CkWQAP\r\n", "OK", 100);} while(answer==0);
//  delay_ms(50);
//  Send_AT("AT+CWMODE=2\r\n", "OK", 100);// Soft AP mode
//	//char *cmd =  ; //  JJROBOTS_XX
//	do { answer = Send_AT("AT+CWSAP=\"ROBAI_EVO2\",\"87654321\",5,3\r\n", "OK", 2000);} while(answer==0); 
//	delay_ms(50);
//	// Start UDP SERVER on port 2222, telemetry port 2223
//	#ifdef PRINT
//    printf("\r\nStart UDP server\r\n");
//	#endif
//  do { answer = Send_AT("AT+CIPMUX=0\r\n", "OK", 100);} while(answer==0);  // Single connection mode
//  do { answer = Send_AT("AT+CIPMODE=1\r\n", "OK", 100);} while(answer==0); // Transparent mode
//  char Telemetry[80];	
//  strcpy(Telemetry,"AT+CIPSTART=\"UDP\",\"");
//  strcat(Telemetry,TELEMETRY);
//  strcat(Telemetry,"\",2223,2222,0\r\n");
//  do { answer = Send_AT(Telemetry, "OK", 2000);} while(answer==0); 
//  delay_ms(50);
	//printf("%s",Telemetry);
}





