#include "network.h"

//char Data_OSC[64];

// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
void readControlParameters(void){
  // Parameters Mode (page2 controls)
  // Parameter initialization (first time we enter page2)
  if (!modifing_control_parameters)
  {
    for (uint8_t i = 0; i < 4; i++)
      OSCfader[i] = 0.5;
    OSCtoggle[0] = 0;

    modifing_control_parameters = true;
   
    //USART_Puts(USART2, "$P2\r\n"); // Serial1.println("$P2"); 
  }
  // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder1,2,3,4)
  // Now we need to adjust all the parameters all the times because we dont know what parameter has been moved
  Kp_user = KP * 2 * OSCfader[0]; // P- Stability
  Kd_user = KD * 2 * OSCfader[1]; // D- Stability
  Kp_thr_user = KP_THROTTLE * 2 * OSCfader[2];  // P - Speed
  Ki_thr_user = KI_THROTTLE * 2 * OSCfader[3];  // I - Speed
  // Send a special telemetry message with the new parameters
  //char auxS[50];
  //sprintf(auxS, "$tP,%d,%d,%d,%d\r\n", (int)(Kp_user * 1000), (int)(Kd_user * 1000), (int)(Kp_thr_user * 1000), (int)(Ki_thr_user * 1000));
  //USART_Puts(USART2, auxS); // Serial1.println(auxS);

#if DEBUG>0
  Serial.print("Par: ");
  Serial.print(Kp_user);
  Serial.print(" ");
  Serial.print(Kd_user);
  Serial.print(" ");
  Serial.print(Kp_thr_user);
  Serial.print(" ");
  Serial.println(Ki_thr_user);
#endif

  // Calibration mode??
  if (OSCpush[2]==1)
  {
//    Serial.print("Calibration MODE ");
    angle_offset = angle_adjusted_filtered;
//    Serial.println(angle_offset);
  }

  // Kill robot => Sleep
  while (OSCtoggle[0] == 1)
  {
    //Reset external parameters
    PID_errorSum = 0;
    timer_old_tick = millis();
    setMotorSpeedM1(0);
    setMotorSpeedM2(0);
    SETC(13);  // Disable motors
    OSC_MsgReadN();
  }
}


int Send_AT(char *ATcmd, char *sample, u32 timeout_ms) {
  int x=0, answer=0;uint8_t c;
  char response[100];
  unsigned long previous;
  memset(response, '\0', 100);      // Initialize the string
  delay_ms(100);
  while( USART_Getc(USART2) > 0);
  USART_Puts(USART2,ATcmd);printf("%s",ATcmd);
  x = 0;
  previous = millis();
  do {
		//delay_ms(1);
		c=USART_Getc(USART2);USART_Putc(USART1,c);
    if(c!= 0){    
      response[x] = c;
      x++;
      if (strstr(response, sample) != NULL)    
      {
        answer = 1; //buffer_map=response;
      }
    }
  }
  while((answer == 0) && ((millis() - previous) < timeout_ms));    
  return answer;
}

//u8 check=0;
//int Send_AT(char *ATcmd, char *sample, u32 timeout_ms){
//	int loop=0;
//	check=0;
//	timeout_ms = timeout_ms*10; // = ms 
//	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)!=0);
//	Erase_Buffer(Data_OSC);
//	USART_Puts(USART2,ATcmd);
//  
//	while(loop==0){  
//		if(Data_OSC>0){
//			if(strstr(Data_OSC, sample)){
//				printf("Success\r\n");
//				loop=1;
//				check = 1;
//        Erase_Buffer(RX_BUF);
//        Erase_Buffer(Data_OSC);
//        RXi=0;
//				delay_ms(50);
//			}
//	
//      delay_ms(10);
//      timeout_ms--;
//      if(timeout_ms<5){
//      	USART_Puts(USART1,"Failed");
//				//printf("Failed %s",ATcmd);
//        Erase_Buffer(Data_OSC);
//        Erase_Buffer(RX_BUF);
//        loop = 0; 
//        if(strstr(ATcmd,"AT+CWQAP\r\n")){
//            loop = 1; 
//        }
//        delay_ms(200);
//        timeout_ms = timeout_ms*10;
//        USART_Puts(USART2,ATcmd);
//      }
//    }
//	}
//	return check;
//}

//void Erase_Buffer(char *str){
//	memset(str,'\0',strlen(str));
//	RXi=0;
//}


