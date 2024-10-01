#include "servo.h"

/*
 * PWM pins are connected to fixed possible pins

 * 	TIMER	|CHANNEL 1				|CHANNEL 2				|CHANNEL 3				|CHANNEL 4
 * 				|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3		|PP1	PP2		PP3
 * 	TIM 4	|PB6	PB4		PC6		|PB7	PB5		PC7		|PB8	PC8		-			|PB9	PC9		-
*/

/**
 * @brief Cau hinh TIM3
 * @param cau hinh Timer 3 cho chan PA6 (Channel1, PinPack1). 
 */
 
 /*
		use clock 72MHz
 */
 
int battery;

void Config_Servo(void){
	/* Khai bao cau hinh chan */
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Khai bao cau hinh chuc nang */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* Khai bao cau hinh timer */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
	
	/* cau hinh chan */
	/*kenh 1 - pinpack 1*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);	//RCC_APB2Periph_GPIOA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; /* che do*/ 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; /* Chon chan */ // 7
	GPIO_Init(GPIOB, &GPIO_InitStruct);		/* khoi tao */
	
	/* Cap xung cho Timer 3*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

/** 
	*@Mucdich Tinh tan so
	*@param  F = (Clock_Sys/(Prescaler-1)*1MHz)/Period
	* Clock_Sys: xung he thong (stm32f103c8t6 = 72MHz)
	*/
	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;  /*Bo chia  72MHz/72  = 1MHz*/
	TIM_TimeBaseStructure.TIM_CounterMode= TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period= 20000-1;   /*1MHz/20000 = 50Hz*/ 
	TIM_TimeBaseStructure.TIM_ClockDivision=0;	
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; /*lap lai chu ky*/
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);  /*khoi tao*/
	
	
	/* xung cho kenh va pinpack cho phep PWM */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	/*  */
	TIM_OC3FastConfig(TIM4, DISABLE);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_Cmd(TIM4, ENABLE); 
}

void Write_Deg(u8 degree){
	u16 pwm;
	//0.5ms(500) - 2ms(2500) (T=20ms)
	//MG996R: 500 - 2500 : 0-180
	// -> pwm = (degree * (2320-740)/180)+740 || 2400 - 740
	pwm = ((degree *2000)/180)+500;
	TIM4->CCR3= pwm;
}

void BROBOT_moveServo(int pwm){
  pwm = constrain(pwm, SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH);
  TIM4->CCR3 = pwm;
}


// output : Battery voltage*10 (aprox) and noise filtered
int BROBOT_readBattery(bool first_time){
  if (first_time)
	//battery = analogRead(5)/BATT_VOLT_FACTOR;
	battery = 120;
  else
    //battery = (battery*9 + (analogRead(5)/BATT_VOLT_FACTOR))/10;
	battery = 120;
  return battery;
}

void TIM_Init(void){
	/*
		TIM2 Channel2 PinPack1 : GPIO_A, Pin_1
		TIM3 Channel1 PinPack1 : GPIO_A, Pin_6
	*/
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef       TIM_OC;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_5;		//step1-5//dir6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		//|GPIO_Pin_14
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	/*
		RCC_APB1Periph:
										TIM2,3,4,5,6,7,12,13,14
										WWDG
										USART2,3
										USART4,5
										SPI2,3
										I2C1,2
										CAN1,2
										PWR
										DAC	
	*/
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);  // APB1 36Mhz
	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36-1; // 72/36(PSC) = 2MHz	 //PSC
  TIM_TimeBaseInitStructure.TIM_Period = ZERO_SPEED-1; 			 //ARR
	TIM_TimeBaseInitStructure.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision= TIM_CKD_DIV1; 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);

	TIM_ARRPreloadConfig(TIM2, DISABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);

	//-----------------------------------------------------------
	TIM_OC.TIM_OCMode      = TIM_OCMode_Timing;                // Output compare toggling mode
  TIM_OC.TIM_OutputState = TIM_OutputState_Disable;          // the Output Compare state
  TIM_OC.TIM_Pulse       = ZERO_SPEED;                       // Output Compare 1 reg value  
	TIM_OC.TIM_OCPolarity  = TIM_OCPolarity_High; // 
	

	TIM_OC1FastConfig(TIM3, TIM_OCFast_Disable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);          // Enabling Ch.1 Output Compare preload Timer 4
	TIM_OC1Init(TIM3, &TIM_OC);         
	
	// Dir 1:
	dir_M1 = 0;TIM3->CNT=0;
	
	TIM_OC.TIM_OCMode      = TIM_OCMode_Timing;                // Output compare toggling mode
  TIM_OC.TIM_OutputState = TIM_OutputState_Disable;          // the Output Compare state
  TIM_OC.TIM_Pulse       = ZERO_SPEED;                       // Output Compare 1 reg value  
	TIM_OC.TIM_OCPolarity  = TIM_OCPolarity_High;

	TIM_OC1FastConfig(TIM2, TIM_OCFast_Disable);                             
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);      	// Disabling Ch.1 Output Compare preload Timer 2
	TIM_OC2Init(TIM2, &TIM_OC); 															// Initializing Output Compare 1 structure Timer 2
	
	// Dir 2:
	dir_M2 = 0;	TIM2->CNT=0;
	
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);                     // Enabling TIM2 Ch.2 interrupts
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);                     // Enabling TIM4 Ch.1 interrupt
	
	TIM_ClearFlag(TIM3,TIM_FLAG_CC1);
	TIM_ClearFlag(TIM2,TIM_FLAG_CC2);


	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;	
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	delay(200);
CLRC(13); //PB4: LOW - Ena
TIM_Cmd(TIM3,ENABLE);	
TIM_Cmd(TIM2,ENABLE);	
}
