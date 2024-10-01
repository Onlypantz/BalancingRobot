#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//
//STM32VN.COM								  
////////////////////////////////////////////////////////////////////////////////// 	   

//PB5;PE5		    
//LED =1: SANG;LED =0: TAT
void LED_Init(void)
{
 
// GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE);	 //PB,PE
//	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED0-->PB.5
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
// GPIO_Init(GPIOC, &GPIO_InitStructure);					 
// GPIO_ResetBits(GPIOC,GPIO_Pin_13);						 //PB.5 =0;

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5
// GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 
// GPIO_ResetBits(GPIOE,GPIO_Pin_5); 						 //PE.5=0;
	GPIOx_Init(GPIOC, GPIO_Pin_13|GPIO_Pin_6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	GPIOx_Init(GPIOB, GPIO_Pin_5|GPIO_Pin_6,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
}

