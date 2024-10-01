#include "MPU6050.h"
uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

// Global MPU6050 IMU variables
ACCCEL_T_GYRO_UNION accel_t_gyro;
float x_gyro_value;   // in deg/seg units
float x_gyro_offset = 0.0;
float accel_angle;  // in degree units
float angle;


// This function implements a complementary filter to fusion gyro and accel info
float MPU6050_getAngle(float dt){
	int16_t correction;
	
  accel_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  x_gyro_value = (accel_t_gyro.value.x_gyro - x_gyro_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s

  // Complementary filter
  // We integrate the gyro rate value to obtain the angle in the short term and we take the accelerometer angle with a low pass filter in the long term...
  angle = 0.99 * (angle + x_gyro_value * dt) + 0.01 * accel_angle;  // Time constant = 0.99*0.01(100hz)/(1-0.99) = 0.99, around 1 sec.

  // Gyro bias correction
  // We supose that the long term mean of the gyro_value should tend to zero (gyro_offset). This means that the robot is not continuosly rotating.
  correction = constrain(accel_t_gyro.value.x_gyro, x_gyro_offset - 10, x_gyro_offset + 10); // limit corrections...
  x_gyro_offset = x_gyro_offset * 0.9995 + correction * 0.0005; // Time constant of this correction is around 20 sec.

  //Serial.print(angle);
  //Serial.print(" ");
  //Serial.println(x_gyro_offset);

  return angle;
}

void MPU6050_calibrate(void){
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;
  
  delay_ms(500);
  while (!gyro_cal_ok){
		printf("Gyro calibration... DONT MOVE!");
		// we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++){
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.x_gyro;
      value += accel_t_gyro.value.x_gyro;
      delay_ms(25);
    }
    // mean value
    value = value / 100;
    // calculate the standard deviation
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (values[i] - value) * (values[i] - value);
		
    dev = sqrt((1 / 100.0) * dev);
		printf("offset: %ld\r", value); // ld: long, lu: unsigned long
		printf("stddev: %f\r\n", dev);	// f: float
    if (dev < 50.0)
      gyro_cal_ok = true;
		else{
			printf("Repeat, DONT MOVE!");
		}
  }
  x_gyro_offset = value;
  // Take the first reading of angle from accels
  angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
	printf("Calibrate OK!");
}

void MPU6050_read_3axis(void){
  int error;;
  // read 14 bytes (gyros, temp and accels)
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t*)&accel_t_gyro, sizeof(accel_t_gyro));
  if (error != 0) {	
    printf("MPU6050 Error:");
		printf("%u",error); 
	}
  // swap bytes
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

    // Print the raw acceleration values
	printf("ACCEL: %d, %d, %d \r\n", accel_t_gyro.value.x_accel, accel_t_gyro.value.y_accel, accel_t_gyro.value.z_accel);
    // Print the raw gyro values.
	printf("GYRO: %d, %d, %d \r\n", accel_t_gyro.value.x_gyro, accel_t_gyro.value.y_gyro, accel_t_gyro.value.y_gyro);

}

void MPU6050_read_1axis(void){
  int error;

  // read X accel
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_accel_h, 6);
  if (error != 0) {
		/*
    print("MPU6050 Error:");
    print(error);
		*/
  }
  // read X gyro
  error = MPU6050_read(MPU6050_GYRO_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_gyro_h, 2);
  if (error != 0) {
		/*
    print("MPU6050 Error:");
    print(error);
		*/
  }
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);

  // Print values

   printf("Acc_Gyro: %i, %i",accel_t_gyro.value.y_accel, accel_t_gyro.value.x_gyro);
}

bool MPU6050_newData(){
  uint8_t status;
  int error;
  error = MPU6050_read(MPU6050_INT_STATUS, &status, 1);
  if (error != 0) {
		/*
    print("MPU6050 Error:");
    print(error);
		*/
	}
  if(status&0x01)	// Data ready?
		return true; 
  else 
		return false;
}


void Init_IIC(I2C_TypeDef* I2Cx, u32 clk){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* I2C1 or I2C2 Periph clock enable */
	if(I2Cx == I2C1){
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		/* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}else if(I2Cx == I2C2){
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		/* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	
	I2C_InitStructure.I2C_ClockSpeed = clk;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//I2C_DutyCycle_16_9;	//I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable; //  I2C_Ack_Disable
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStructure);
	
	I2C_Cmd(I2Cx, ENABLE);
}

void MPU6050_Setup(void){
//  uint8_t c;
//	int error;
	
	Init_IIC(I2C, 100000);
//	IIC_Init();
	delay_ms(100);
//	error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
#if PRINT ==1	
	printf("WHO_AM_I: 0x%x\r", c);
	printf("error: %i\r", error);
#endif
	
	// RESET chip
	MPU6050_write_reg(MPU6050_PWR_MGMT_1, bit(MPU6050_DEVICE_RESET));
	delay_ms(125);
	
	// Clear the 'sleep' bit to start the sensor and select clock source
	MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);
	//MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);
	// Config Gyro scale (500deg/seg)
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  // Config Accel scale (2g)
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_2G);
  // Config Digital Low Pass Filter 10Hz
  MPU6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_10HZ);
  // Set Sample Rate to 100Hz
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 9);  // 100Hz : Sample Rate = 1000 / (1 + SMPLRT_DIV) Hz
  // Data ready interrupt enable
  MPU6050_write_reg(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
  // Clear the 'sleep' bit to start the sensor (and select clock source).
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);

  // Clear the 'sleep' bit to start the sensor.
  //MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);
  //MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
#if PRINT ==1	
	printf("MPU6050 Setup Ok!!!\r");
#endif
}

int MPU6050_read(uint8_t reg, uint8_t *pBuffer, uint8_t size){
	int16_t n;
	n = I2C_Hard_ReadMulti(I2C, MPU6050_I2C_ADDRESS, reg, size, pBuffer);
	if(n!=0) return -10;

	return 0; // no error
}

int MPU6050_write(int reg, uint8_t *pData, int size){
	int n;
	n = I2C_Hard_WriteMulti(I2C, MPU6050_I2C_ADDRESS, reg, size, pData);
	if(n!=0){
		if(n==-1)return (-20); // start error
		else return -21; // send error
	}
	
	return 0; // no error
}

// MPU6050_write_reg (only 1 byte)
int MPU6050_write_reg(int reg, uint8_t data){
  int error;
//	error = MPU6050_write(reg, &data, 1);
	I2C_Hard_Write(I2C, MPU6050_I2C_ADDRESS, reg, data);

  return (error);
}


