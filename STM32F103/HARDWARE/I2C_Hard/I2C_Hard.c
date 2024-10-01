#include "I2C_hard.h"

/*
 * Add stm32f10x_i2c.h lib (FWLIB) to compile success!
 * I2C1: SDA = PB7.  SCL: PB6
 * I2C2: SDA = PB11. SCL: PB10
 * Start | ADDR | R/W | DATA | ACK | STOP . 
 */
#define Disable_Ack 0
#define Enable_Ack 	1
static uint32_t TM_I2C_Timeout;
 
void RCC_Config(I2C_TypeDef* I2Cx){
	
	/* GPIOB Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* I2C1 or I2C2 Periph clock enable */
	if(I2Cx == I2C1){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}else if(I2Cx == I2C2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}
	
}

void I2C_Hard_Config(I2C_TypeDef* I2Cx){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if(I2Cx == I2C1){
		
		/* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}else if(I2Cx == I2C2){
		
		/* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	
}

void I2C_Hard_Init(I2C_TypeDef* I2Cx, u8 addr, u32 clkspeed){
	
	I2C_InitTypeDef  I2C_InitStructure;
	
	RCC_Config(I2Cx);
	I2C_Hard_Config(I2Cx);
	
	I2C_Cmd(I2Cx, ENABLE);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = addr;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable; //  I2C_Ack_Disable
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = clkspeed;
	I2C_Init(I2Cx, &I2C_InitStructure);
	
	I2C_Cmd(I2Cx, ENABLE);
	
}

int8_t I2C_Hard_Start(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t direction, uint8_t ack){
	
	/* Send I2Cx START condition */
	I2Cx->CR1 |= I2C_CR1_START;
	TM_I2C_Timeout = TM_I2C_TIMEOUT;
	
	while (!(I2Cx->SR1 & I2C_SR1_SB)) {
		if (--TM_I2C_Timeout == 0x00) {
			return -1;
		}
	}
	/* Enable ack if we select it */
	if (ack) {
		I2Cx->CR1 |= I2C_CR1_ACK;
	}
	/* Send write/read bit */
	if (direction == I2C_Direction_Transmitter) {
		/* Send address with zero last bit */
		I2Cx->DR = addr & ~I2C_OAR1_ADD0;
		
		/* Wait till finished */
		TM_I2C_Timeout = TM_I2C_TIMEOUT;
		while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
			if (--TM_I2C_Timeout == 0x00) {
				return -1;
			}
		}
	}
	if (direction == I2C_Direction_Receiver) {
		/* Send address with 1 last bit */
		I2Cx->DR = addr | I2C_OAR1_ADD0;
		
		/* Wait till finished */
		TM_I2C_Timeout = TM_I2C_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			if (--TM_I2C_Timeout == 0x00) {
				return -1;
			}
		}
	}	
	/* Read status register to clear ADDR flag */
	I2Cx->SR2;
	/* Return 0, everything ok */
	return 0;
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */

void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	/* Wait till I2C is not busy anymore */
	TM_I2C_Timeout = TM_I2C_TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_TXE) && TM_I2C_Timeout) {
		TM_I2C_Timeout--;
	}
	/* Send I2C data */
	I2Cx->DR = data;
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	uint8_t data;
	/* Enable ACK */
	I2Cx->CR1 |= I2C_CR1_ACK;
	/* Wait till not received */
	TM_I2C_Timeout = TM_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--TM_I2C_Timeout == 0x00) {
			return 1;
		}
	}
	/* Read data */
	data = I2Cx->DR;
	/* Return data */
	return data;
	
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	uint8_t data;
	
	/* Disable ACK */
	I2Cx->CR1 &= ~I2C_CR1_ACK;
	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;
	/* Wait till received */
	TM_I2C_Timeout = TM_I2C_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if (--TM_I2C_Timeout == 0x00) {
			return 1;
		}
	}
	/* Read data */
	data = I2Cx->DR;
	/* Return data */
	return data;
	
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
uint8_t I2C_Hard_stop(I2C_TypeDef* I2Cx){
	/* Wait till transmitter not empty */
	TM_I2C_Timeout = TM_I2C_TIMEOUT;
	while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF)))) {
		if (--TM_I2C_Timeout == 0x00) {
			return 1;
		}
	}
	
	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;
	
	return 0;
}

/* -----------------------------------------------------
 * -------------------- WRITE I2C ----------------------
 * -----------------------------------------------------
																												*/

int8_t I2C_Hard_Write(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t Data){
	int8_t error;
	I2C_Hard_Start(I2Cx, addr, I2C_Direction_Transmitter, Disable_Ack);
	I2C_WriteData(I2Cx, reg);
	I2C_WriteData(I2Cx, Data);
	I2C_Hard_stop(I2Cx);

	return error;
}

int8_t I2C_Hard_WriteMulti(I2C_TypeDef* I2Cx, uint8_t addr,uint8_t reg,uint16_t lenght,uint8_t *buf){
	int8_t error;
	
	error = I2C_Hard_Start(I2Cx, addr, I2C_Direction_Transmitter, Disable_Ack);
	if(error != 0) return -1;
	I2C_WriteData(I2Cx, reg);

	while (lenght--) {
		I2C_WriteData(I2Cx, *buf++);
	}
	I2C_Hard_stop(I2Cx);
	
	return error;
}

/* -----------------------------------------------------
 * -------------------- READ I2C -----------------------
 * -----------------------------------------------------																												*/

int8_t I2C_Hard_ReadMulti(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t lenght,uint8_t *buf){
	int8_t error=0;
	error = I2C_Hard_Start(I2Cx, addr, I2C_Direction_Transmitter, Enable_Ack);
	I2C_WriteData(I2Cx, reg);
	I2C_Hard_Start(I2Cx, addr, I2C_Direction_Receiver, Enable_Ack);
	while (lenght--) {
		if (!lenght) {
			/* Last byte */
			*buf++ = I2C_read_nack(I2Cx);
		} else {
			*buf++ = I2C_read_ack(I2Cx);
		}
	}
	return error;
}

void i2c_read_multi_no_reg(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t len, uint8_t* data){
	int i;
	
	I2C_Hard_Start(I2Cx, addr, I2C_Direction_Transmitter, Disable_Ack);
	for (i = 0; i < len; i++)
	{
		if (i == (len - 1))
		{
			data[i] = I2C_read_nack(I2Cx);
		}
		else
		{
			data[i] = I2C_read_ack(I2Cx);
		}
	}
	I2C_Hard_stop(I2Cx);
}

int8_t I2C_Hard_Read(I2C_TypeDef* I2Cx, uint8_t addr, uint8_t reg, uint8_t *data){
	int8_t error = 1;
	error = I2C_Hard_ReadMulti(I2Cx, addr, reg, 1, data);
	return error;
}

