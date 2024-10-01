#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
 void GPIOx_Init(GPIO_TypeDef* gpio, u16 pin, GPIOMode_TypeDef mode,GPIOSpeed_TypeDef speed)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
	uint32_t gpio_clock;
	
	/* Get proper settings */
	if (gpio == GPIOA) {
		gpio_clock = RCC_APB2Periph_GPIOA;
	} else if (gpio == GPIOB) {
		gpio_clock = RCC_APB2Periph_GPIOB;
	} else if (gpio == GPIOC) {
		gpio_clock = RCC_APB2Periph_GPIOC;
	} else if (gpio == GPIOD) {
		gpio_clock = RCC_APB2Periph_GPIOD;
	} else if (gpio == GPIOE) {
		gpio_clock = RCC_APB2Periph_GPIOE;
	} else if (gpio == GPIOF) {
		gpio_clock = RCC_APB2Periph_GPIOF;
	} else if (gpio == GPIOG) {
		gpio_clock = RCC_APB2Periph_GPIOG;
	} 
 	
 RCC_APB2PeriphClockCmd(gpio_clock, ENABLE);	 //PB,PE
	
 GPIO_InitStructure.GPIO_Pin = pin;				 //LED0-->PB.5
 GPIO_InitStructure.GPIO_Mode = mode; 		 
 GPIO_InitStructure.GPIO_Speed = speed;		 
 GPIO_Init(gpio, &GPIO_InitStructure);					 
 GPIO_ResetBits(gpio,pin);						 //PB.5 =0;

}
//********************************************************************************  

void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}

void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}

void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}

__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
