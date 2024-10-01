#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
								  
////////////////////////////////////////////////////////////////////////////////// 	
   	   		   
////IO
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9IN
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9OUT

////IO 
#define SDA_IN()  GPIOx_Init(GPIOC, GPIO_Pin_14,GPIO_Mode_IPU,GPIO_Speed_50MHz);    // PC14
#define SDA_OUT() GPIOx_Init(GPIOC, GPIO_Pin_14,GPIO_Mode_Out_PP,GPIO_Speed_50MHz); // PC15


#define IIC_SCL    PCout(15) //SCL
#define IIC_SDA    PCout(14) //SDA	 
#define READ_SDA   PCin(14)  //SDA IN

//IIC

void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);		
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
	  
///////////////////////////////
void IIC_Cmd_Write(u8 add ,u8 reg,u8 data);
u8 IIC_Write(u8 addr,u8 reg,u8 data);
u8 Read_IIC(u8 addr, u8 reg);
u8 IIC_ReadMulti(u8 addr,u8 reg,u8 len,u8 *buf);
u8 IIC_WriteMulti(u8 addr,u8 reg,u8 len,u8 *buf);

#endif
















