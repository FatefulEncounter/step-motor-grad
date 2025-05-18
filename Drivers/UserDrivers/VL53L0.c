#include "VL53L0.h"

#define RIGHT_VL53L0_ADDR  0X54
#define LEFT_VL53L0_ADDR   0X56

#define I2C_SDA_PIN  GPIO_PIN_8
#define I2C_SDA_PORT GPIOB

#define I2C_SCL_PIN  GPIO_PIN_9
#define I2C_SCL_PORT GPIOB

void VL53L0X_i2c_init(void)
{

}


/**************************
 * 
 * 自己写的I2C协议
 * 
 * 
 * **************************/

#include "tim.h"
#include "stm32f4xx_hal.h"

#define t_TIM  htim6    //需要一个1Mhz的定时器作为延时函数的时基单元
 
#define READ_SDA(DEV_SDA)		   HAL_GPIO_ReadPin(DEV_SDA.GPIOx, DEV_SDA.GPIO_Pin)
#define SDA(DEV_SDA,PinState)      HAL_GPIO_WritePin(DEV_SDA.GPIOx,DEV_SDA.GPIO_Pin,PinState)
#define SCL(DEV_SCL,PinState)      HAL_GPIO_WritePin(DEV_SCL.GPIOx,DEV_SCL.GPIO_Pin,PinState)
 
typedef enum{
	Nack= 0,
	Ack = 1
}DEV_Response;
typedef enum{
	Sda_In =0,
	Sda_Out=1
}SDA_Mode;
 
typedef struct DEV_SCL{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}DEV_SCL;
 
typedef struct DEV_SDA DEV_SDA; //声明
typedef struct DEV_SDA{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}DEV_SDA ;
 
typedef struct DEV_I2C{
	DEV_SDA Dev_I2c_Sda;
	DEV_SCL Dev_I2c_Scl;
}DEV_I2C;
void I2C_UsDelay(int xus);
void I2C_MsDelay(int xms);
void I2C_Init(DEV_I2C Dev_I2c);
void I2C_Start(DEV_I2C Dev_I2c);
void I2C_Stop(DEV_I2C Dev_I2c);
uint8_t I2C_WaitAck(DEV_I2C Dev_I2c);
uint8_t I2C_SendAck(DEV_I2C Dev_I2c);
uint8_t I2C_SendNotAck(DEV_I2C Dev_I2c);
uint8_t I2C_SendByte(DEV_I2C Dev_I2c,uint8_t Data);
uint8_t I2C_ReciveByte(DEV_I2C Dev_I2c,DEV_Response rep);
void SDA_IoSwitch(DEV_SDA Dev_Sda,SDA_Mode sda_mode);


void(*ModeChange_Func)(DEV_SDA Dev_Sda,SDA_Mode sda_mode);
 
void I2C_UsDelay(int xus){   //max -> 30ms =  30 000us
   TIM1->CNT = 0;
	while(xus>= (TIM1->CNT));
}
 
void I2C_MsDelay(int xms){
	do{
		I2C_UsDelay(1000);
	}while(xms--);
}
 
void I2C_Init(DEV_I2C Dev_I2c){
	HAL_TIM_Base_Start(&t_TIM);
	SDA(Dev_I2c.Dev_I2c_Sda,1);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	ModeChange_Func = SDA_IoSwitch;
}
void I2C_Start(DEV_I2C Dev_I2c){
	
	SDA(Dev_I2c.Dev_I2c_Sda,1);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	SDA(Dev_I2c.Dev_I2c_Sda,0);
	I2C_UsDelay(2);
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	I2C_UsDelay(2);
}
void I2C_Stop(DEV_I2C Dev_I2c){
	SDA(Dev_I2c.Dev_I2c_Sda,0);
	I2C_UsDelay(2);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	SDA(Dev_I2c.Dev_I2c_Sda,1);
	I2C_UsDelay(2);
}
uint8_t I2C_WaitAck(DEV_I2C Dev_I2c){
	uint8_t ack;
	uint8_t cErrTime = 5;
	(*ModeChange_Func)(Dev_I2c.Dev_I2c_Sda,Sda_In);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	while( READ_SDA(Dev_I2c.Dev_I2c_Sda)){   //判断是否有ACK
		cErrTime--;
		I2C_UsDelay(2);
		if(0==cErrTime){
			(*ModeChange_Func)(Dev_I2c.Dev_I2c_Sda,Sda_Out);
			I2C_Stop(Dev_I2c);
			return Nack;
		}
	}
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	(*ModeChange_Func)(Dev_I2c.Dev_I2c_Sda,Sda_Out);
	I2C_UsDelay(2);
	return  Ack;
}
uint8_t I2C_SendAck(DEV_I2C Dev_I2c){
	SDA(Dev_I2c.Dev_I2c_Sda,0);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	I2C_UsDelay(2);
}
uint8_t I2C_SendNotAck(DEV_I2C Dev_I2c){
	SDA(Dev_I2c.Dev_I2c_Sda,1);
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	I2C_UsDelay(2);
}
 
 
uint8_t I2C_SendByte(DEV_I2C Dev_I2c,uint8_t Data){
	
	uint8_t i;
	for(i = 0 ;i < 8 ;i++){
		SDA(Dev_I2c.Dev_I2c_Sda,Data &(0x80 >> i));
		SCL(Dev_I2c.Dev_I2c_Scl,1);
		I2C_UsDelay(2);
		SCL(Dev_I2c.Dev_I2c_Scl,0);
		I2C_UsDelay(2);
	}
	SCL(Dev_I2c.Dev_I2c_Scl,1);
	I2C_UsDelay(2);
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	I2C_UsDelay(2);
	return 1;
}
uint8_t I2C_ReciveByte(DEV_I2C Dev_I2c,DEV_Response rep){
	unsigned char i = 8;
	unsigned char cR_Byte = 0;
	(*ModeChange_Func)(Dev_I2c.Dev_I2c_Sda,Sda_In);
	while(i--){
		cR_Byte += cR_Byte;
		SCL(Dev_I2c.Dev_I2c_Scl,0);
		I2C_UsDelay(2);
		SCL(Dev_I2c.Dev_I2c_Scl,1);
		I2C_UsDelay(2);
		cR_Byte |= READ_SDA(Dev_I2c.Dev_I2c_Sda);
	}
	SCL(Dev_I2c.Dev_I2c_Scl,0);
	I2C_UsDelay(2);
	(*ModeChange_Func)(Dev_I2c.Dev_I2c_Sda,Sda_Out);
	return cR_Byte;
}
 
void SDA_IoSwitch(DEV_SDA Dev_Sda,SDA_Mode sda_mode){
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	switch(sda_mode){
		case Sda_In:
			GPIO_InitStruct.Pin = Dev_Sda.GPIO_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			HAL_GPIO_Init(Dev_Sda.GPIOx, &GPIO_InitStruct);
			break;
		case Sda_Out:
			GPIO_InitStruct.Pin = Dev_Sda.GPIO_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(Dev_Sda.GPIOx, &GPIO_InitStruct);
			break;
	}
}
