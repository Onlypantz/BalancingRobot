#include "osc.h"

//Serial  -> USART1
//Serial1 -> USART2


//uint8_t RXi;
//char RXc;
//char RX_BUF[RX_BUF_SIZE];
//char buffdata[64];
//uint8_t UDP_RXi;


char UDPBuffer[64];

unsigned char OSCreadStatus;
unsigned char OSCreadCounter;
unsigned char OSCreadNumParams;
char OSCcommandType;
char OSCtouchMessage;

float OSC_extractParamFloat(uint8_t pos) {
  union {
    unsigned char Buff[4];
    float d;
  } u;

  u.Buff[0] = (unsigned char)UDPBuffer[pos];
  u.Buff[1] = (unsigned char)UDPBuffer[pos + 1];
  u.Buff[2] = (unsigned char)UDPBuffer[pos + 2];
  u.Buff[3] = (unsigned char)UDPBuffer[pos + 3];
  return (u.d);
}

float OSC_extractParamFloatN(uint8_t pos) {
  union {
    unsigned char Buff[4];
    float d;
  } u;

  u.Buff[0] = (unsigned char)UDPBuffer[pos];
  u.Buff[1] = (unsigned char)UDPBuffer[pos - 1];
  u.Buff[2] = (unsigned char)UDPBuffer[pos - 2];
  u.Buff[3] = (unsigned char)UDPBuffer[pos - 3];
  return (u.d);
}

int16_t OSC_extractParamInt(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;

  u.Buff[1] = (unsigned char)UDPBuffer[pos];
  u.Buff[0] = (unsigned char)UDPBuffer[pos + 1];
  return (u.d);
}

void OSC_init(void){
  OSCreadStatus = 0;
  OSCreadCounter = 0;
  OSCreadNumParams = 0;
  OSCcommandType = 0;
  OSCfader[0] = 0.5;
  OSCfader[1] = 0.5;
  OSCfader[2] = 0.5;
  OSCfader[3] = 0.5;
}

void OSC_MsgSend(char *c, unsigned char msgSize, float p){
  uint8_t i;
  union {
    unsigned char Buff[4];
    float d;
  } u;

  // We copy the param in the last 4 bytes
  u.d = p;
  c[msgSize - 4] = u.Buff[3];
  c[msgSize - 3] = u.Buff[2];
  c[msgSize - 2] = u.Buff[1];
  c[msgSize - 1] = u.Buff[0];
  for (i = 0; i < msgSize; i++)
  {
	  // USART to esp8266
     USART_Putc(USART2, (uint8_t)c[i]);  // dang bytes
  }
}


//void USART2_IRQHandler(void) 
//{
//  if ((USART2->SR & USART_FLAG_IDLE) != (u16)RESET)
//  {
//    USART_ReceiveData(USART2);   
//    if(flaguart == 1 && UDP_RXi==0)
//    {
//      strcpy(UDPBuffer, RX_BUF);
//      printf("%s\r\n", UDPBuffer);       
////      printf("%f\r\n", OSC_extractParamFloatN(14));
//      UDP_RXi= 1;
//    }else if(flaguart == 0){
//      strcpy(Data_OSC, RX_BUF);
//    }
//    
//    memset(RX_BUF, '\0', strlen(RX_BUF)); 
//    RXi = 0;
//  }  
//  
//  if(USART_GetITStatus(USART2, USART_IT_RXNE))
//	{
//    RXc = USART_ReceiveData(USART2);
//    if(RXc !=NULL){
//      RX_BUF[RXi] = RXc;
//      RXi++; 
//    }
//  }
//}


void OSC_MsgReadN(void){
  uint8_t i;
  float value;
	
	//uint8_t c;
  // New bytes available to process
	 // New bytes available to process?
	//c= USART_Getc(USART2);
	//char buffer[100];
	//memset(UDPBuffer, '\0', 100);
	uart:
	
  if (USART_Gets(USART2,UDPBuffer,sizeof(UDPBuffer))) {
    // We rotate the Buffer (we could implement a ring buffer in future)
//    for (i = 8; i > 0; i--) {
//      UDPBuffer[i] = buffer[8-i];//printf("UDP: %c\r\n", buffer[7-i]);
//    }
    //UDPBuffer[0] = c;
		

		
		printf("UDP: %s\r\n", UDPBuffer);
    // We look for an OSC message start like /x/
    if ((UDPBuffer[0] == '/') && (UDPBuffer[2] == '/') && ((UDPBuffer[1] == '1') || (UDPBuffer[1] == '2')))
		{
      if (OSCreadStatus == 0) {
        OSCpage = UDPBuffer[1] - '0';  // Convert page to int
        OSCreadStatus = 1;
        OSCtouchMessage = 0;  
        if (OSCreadStatus == 1) {//printf("UDP: %s\r\n", UDPBuffer);
          // Fadder    /1/fader1 ,f  xxxx
          if ((UDPBuffer[11] == '\0')&& (UDPBuffer[5] == 'd') && (UDPBuffer[6] == 'e') && (UDPBuffer[7] == 'r')) {
            OSCreadStatus = 2;  // Message type detected 
            OSCreadNumParams = 1; // 1 parameters
            OSCcommandType = UDPBuffer[8] - '0';printf("T: %d\r\n", OSCcommandType);
          }
          // Push message
          else if ((UDPBuffer[4] == 'u') && (UDPBuffer[5] == 's') && (UDPBuffer[6] == 'h')) {
              OSCreadStatus = 2;  // Message type detected
              OSCreadNumParams = 1; // 1 parameters
              OSCcommandType = 20 + (UDPBuffer[7] - '1');printf("T: %d\r\n", OSCcommandType);
          }
          // Toggle message
          else if ((UDPBuffer[6] == 'g') && (UDPBuffer[7] == 'l') && (UDPBuffer[8] == 'e')) {
              OSCreadStatus = 2;  // Message type detected
              OSCreadNumParams = 1; // 1 parameters
              OSCcommandType = 30 + (UDPBuffer[9] - '1');printf("T: %d\r\n", OSCcommandType);
          }else {memset(UDPBuffer, '\0', 64);OSCreadStatus =0; goto uart;} 
        } 
        
        if (OSCreadStatus == 2) {
         // if ((UDPBuffer[1] == '/') && (UDPBuffer[0] == 'z')) { // Touch up message? (/z) [only on page1]
            if ((OSCpage == 1) && (OSCcommandType <= 2)) { // Touchup message only on Fadder1 and Fadder2
              OSCtouchMessage = 1;
            }
            else {
              OSCtouchMessage = 0;
              OSCreadStatus = 0; //Finish
            }
         // }   
            OSCreadStatus = 0;
            OSCnewMessage = 1;
            if(OSCpage != 2){
              switch (OSCcommandType) {
                case 1:
                  //value = OSC_extractParamFloat(11);
								if(UDPBuffer[12] == ','){
                  value = OSC_extractParamFloatN(19);
								}
                  if(value<=1.0) {OSCfader[0] = value;printf("V: %f\r\n", value);}
									
                  if ((OSCtouchMessage) && (value == 0)) {
                    OSCfader[0] = 0.5;
                    //OSC_MsgSend("/1/fader1\0\0\0,f\0\0\0\0\0\0\r\n", 20, 0.5);
                  }
									
                  break;
                case 2:
                //  value = OSC_extractParamFloat(11);
								if(UDPBuffer[12] == ','){
                  value = OSC_extractParamFloatN(19);
								}
                  if(value<=1.0) {OSCfader[1] = value;printf("V: %f\r\n", value);}
                  if ((OSCtouchMessage) && (value == 0)) {
                    OSCfader[1] = 0.5;
                    //OSC_MsgSend("/1/fader2\0\0\0,f\0\0\0\0\0\0\r\n", 20, 0.5);  
                  }
									
                  break;
                case 3:
									if(UDPBuffer[12] == ','){
										if(value<=1.0) {OSCfader[2] = OSC_extractParamFloatN(19); printf("V: %f\r\n", value);}
									}
									
                  break;
                case 4:
									if(UDPBuffer[13] == ','){
										if(value<=1.0) {OSCfader[3] = OSC_extractParamFloatN(19);  printf("V: %f\r\n", value);}
									}
									
                  break;
                default:
                  // Push y toggle
									if(UDPBuffer[13] == ','){
										if(value<=1.0) {value = OSC_extractParamFloatN(19);  printf("V: %f\r\n", value);}
									}
                
                  if ((OSCcommandType >= 20) && (OSCcommandType < 25))
                  {
                    if (value == 0)
                      OSCpush[OSCcommandType - 20] = 0;
                    else
                      OSCpush[OSCcommandType - 20] = 1;
                  }
                  if ((OSCcommandType >= 30) && (OSCcommandType < 35))
                  {
                    if (value == 0)
                      OSCtoggle[OSCcommandType - 30] = 0;
                    else
                      OSCtoggle[OSCcommandType - 30] = 1;
                  }
                  break;
             }
          }
        }   
      }  
    }else {memset(UDPBuffer, '\0', 64);OSCreadStatus =0;goto uart;}
    //----------------
    if (OSCnewMessage == 1){
      OSCnewMessage = 0;
      if (OSCpage == 1)   // Get commands from user (PAGE1 are user commands: throttle, steering...)
      {
        if (modifing_control_parameters)  // We came from the settings screen
        {
          OSCfader[0] = 0.5; // default neutral values
          OSCfader[1] = 0.5;
          OSCtoggle[0] = 0;  // Normal mode
          mode = 0;
          modifing_control_parameters = false;
        }

				if (OSCmove_mode)
				{
					//printf("M %");
					//Serial.print(OSCmove_speed);
					//Serial.print(" ");
					//Serial.print(OSCmove_steps1);
					//Serial.print(",");
					//Serial.println(OSCmove_steps2);
					positionControlMode = true;
					OSCmove_mode = false;
					target_steps1 = steps1 + OSCmove_steps1;
					target_steps2 = steps2 + OSCmove_steps2;
				}
        else
        {
					positionControlMode = false;
					throttle = (OSCfader[0] - 0.5) * max_throttle;
					// We add some exponential on steering to smooth the center band
					steering = OSCfader[1] - 0.5;
					if (steering > 0)
						steering = (steering * steering + 0.5 * steering) * max_steering;
					else
						steering = (-steering * steering + 0.5 * steering) * max_steering;
        }

				if ((mode == 0) && (OSCtoggle[0]))
				{
					// Change to PRO mode
					max_throttle = MAX_THROTTLE_PRO;
					max_steering = MAX_STEERING_PRO;
					max_target_angle = MAX_TARGET_ANGLE_PRO;
					mode = 1;
				}
				if ((mode == 1) && (OSCtoggle[0] == 0))
				{
					// Change to NORMAL mode
					max_throttle = MAX_THROTTLE;
					max_steering = MAX_STEERING;
					max_target_angle = MAX_TARGET_ANGLE;
					mode = 0;
				}
      }
      OSCpage = 0;
     }else if(OSCpage == 2){
       readControlParameters();
       OSCpage = 0;
     }
         //-----------
    //----------------
			if(OSCnewMessage == 1){
				OSCnewMessage = 0;
				if (OSCpage == 1)   // Get commands from user (PAGE1 are user commands: throttle, steering...)
				{
					if (modifing_control_parameters)  // We came from the settings screen
					{
						OSCfader[0] = 0.5; // default neutral values
						OSCfader[1] = 0.5;
						OSCtoggle[0] = 0;  // Normal mode
						mode = 0;
						modifing_control_parameters = false;
					}

					if (OSCmove_mode)
					{
						//printf("M %");
						//Serial.print(OSCmove_speed);
						//Serial.print(" ");
						//Serial.print(OSCmove_steps1);
						//Serial.print(",");
						//Serial.println(OSCmove_steps2);
						positionControlMode = true;
						OSCmove_mode = false;
						target_steps1 = steps1 + OSCmove_steps1;
						target_steps2 = steps2 + OSCmove_steps2;
					}
					else
					{
						positionControlMode = false;
						throttle = (OSCfader[0] - 0.5) * max_throttle;
						// We add some exponential on steering to smooth the center band
						steering = OSCfader[1] - 0.5;
						if (steering > 0)
							steering = (steering * steering + 0.5 * steering) * max_steering;
						else
							steering = (-steering * steering + 0.5 * steering) * max_steering;
					}

					if ((mode == 0) && (OSCtoggle[0]))
					{
						// Change to PRO mode
						max_throttle = MAX_THROTTLE_PRO;
						max_steering = MAX_STEERING_PRO;
						max_target_angle = MAX_TARGET_ANGLE_PRO;
						mode = 1;
					}
					if ((mode == 1) && (OSCtoggle[0] == 0))
					{
						// Change to NORMAL mode
						max_throttle = MAX_THROTTLE;
						max_steering = MAX_STEERING;
						max_target_angle = MAX_TARGET_ANGLE;
						mode = 0;
					}
				}
        OSCpage = 0;
       }else if(OSCpage == 2){
         readControlParameters();
         OSCpage = 0;
       }
         //-----------
  }
	
}



