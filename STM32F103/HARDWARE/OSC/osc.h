#ifndef __OSC_H
#define __OSC_H	 

#include "sys.h"
#include "string.h"
#include "usart.h"
#include "defines.h"
#include "globals.h"
#include "network.h"

#define USART	USART2
// OSC functions  (OSC = Open Sound Control protocol)
// OSC Messages read:  OSC: /page/command parameters
//             FADER (1,2,3,4)   Ex: /1/fader1   f,  XXXX  => lenght:20, Param:  float (0.0-1.0)
//             XY (1,2)          Ex: /1/xy1  f,f,    XXXXXXXX => length: 24 Params: float,float (0.0-1.0)
//             PUSH (1,2,3,4)    Ex: /1/push1    f,  XXXX => length:20 Param: float
//             TOGGLE (1,2,3,4)  Ex: /1/toggle1  f,  XXXX => length:20 Param: float
//             MOVE              Ex: /1/m XXXX XXXX XXXX => length:16 Params: speed, steps1, steps2 (all float)
//
// OSC Message send:
//            string to send + param (float)[last 4 bytes]


// for DEBUG uncomment this lines...
//#define OSCDEBUG  1
//#define OSCDEBUG2 1
//#define OSCDEBUG3 1

//---------------------------------------------
////extern uint8_t RXi;
////extern char RXc;
////extern char RX_BUF[RX_BUF_SIZE];
////extern char Data_OSC[64];
////extern uint8_t flag_rec;
////extern int flaguart;

//extern uint8_t UDP_RXi;
//extern char UDPBuffer[20];



// ------- OSC functions -----------------------------------------

// Aux functions
float OSC_extractParamFloat(uint8_t pos);
float OSC_extractParamFloatN(uint8_t pos);
int16_t OSC_extractParamInt(uint8_t pos);
void OSC_init(void);
void OSC_MsgSend(char *c, unsigned char msgSize, float p);
extern void Erase_Buffer(char *str);
//void readControlParameters();
void OSC_MsgReadN(void);

	
#endif

