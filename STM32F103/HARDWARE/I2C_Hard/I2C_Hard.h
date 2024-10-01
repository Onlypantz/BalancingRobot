#ifndef __I2C_HARD_H
#define __I2C_HARD_H	 
#include "sys.h"
#include "string.h"
#include "usart.h"


#ifndef TM_I2C_TIMEOUT
#define TM_I2C_TIMEOUT					20000
#endif

void RCC_Config(I2C_TypeDef* I2Cx);
void I2C_Hard_Config(I2C_TypeDef* I2Cx);
void I2C_Hard_Init(I2C_TypeDef* I2Cx,u8 addr, u32 clkspeed);
int8_t I2C_Hard_Start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t direction, uint8_t ack);
uint8_t I2C_Hard_stop(I2C_TypeDef* I2Cx);
void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);

int8_t I2C_Hard_Write(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t Data);
int8_t I2C_Hard_WriteMulti(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint16_t lenght, uint8_t *buf);
void i2c_write_multi_no_reg(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint8_t len);

int8_t I2C_Hard_Read(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t *data);
int8_t I2C_Hard_ReadMulti(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t lenght, uint8_t *buf);
void i2c_read_multi_no_reg(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t len, uint8_t* data);

		 				    
#endif



