#ifndef __NETWORK_H
#define __NETWORK_H	

#include "sys.h"
#include "stdio.h"
#include "stdbool.h"
#include "defines.h"
#include "globals.h"
#include "control.h"
#include "osc.h"


extern long timer_old_tick;
//extern char Data_OSC[64];

void readControlParameters(void);
int Send_AT(char *ATcmd, char *sample, u32 timeout_ms);
void Erase_Buffer(char *str);
#endif

