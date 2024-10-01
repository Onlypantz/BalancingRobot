#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//

volatile uint32_t counter_ms = 0;
volatile uint32_t counter_us = 0;

__IO uint32_t TimingMillis;
static void __empty() {
	// Empty
}
void yield(void) __attribute__ ((weak, alias("__empty")));

void SysTick_Handler(void) {
	//neu ngat 1ms su dung 2 ham millis() v?micros()
  counter_ms++;//1ms
	TimingMillis++;
	//neu ngat 1us
	//counter_us++; counter_ms %= counter_us;//1us
}

u32 millis(void) {
  return (counter_ms/1000);
}

u32 micros(void)//thong qua ms
{

//int Micros = millis()*1000 + (SystemCoreClock/1000-SysTick->VAL)/72;
return TimingMillis;

}

void delay_init(void){
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	SysTick_Config(SystemCoreClock / 1000);	// ngat 1ms
	SysTick_Config (SystemCoreClock / 1000000); //1us per interrupt
}
void delay_ms (u32 nms)
{
//u32 Time = millis();
//while ((nms - (millis()-Time))> 0);
	delay( nms );
}


void delay_us(u32 nus)
{
	uint32_t end = micros() + nus;
    while (micros() < end);//-615
}
//////////////////////////////////////////////////


void delay( uint32_t ms )
{
    uint32_t end = millis() + ms;
    while (millis() < end)
    	yield();
}			 


void delayMicroseconds(uint32_t us)
{
	uint32_t end = micros() + us;
    while (micros() < end);
	
}






































