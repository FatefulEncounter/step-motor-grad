#include "Tmc2209.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>
#include <stdio.h>
#include "Debug.h"
#include "Key.h"

#define UNINT16MAX      65535
#define UNINT16MIN      0
#define TIMCLOK		    84
#define ONE_M           1000000

int pulse_cnt;
float usecircal;
char tmc2209buffer[100];

/*
    en pin ---> 0 is enable
    dir right --> 顺时针

    126 1100
    123 1000
*/
void TMC2209_Test(void)
{

        
}
void TMC2209_Init(void)
{
    /*EN & DIR INIT*/
    HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN, StepMotor_EN);
    HAL_GPIO_WritePin(GPIOB, MOTORDIR_PIN, Motor_right);
    
    HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, StepMotor_EN);
    HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, Motor_right);
    
    /*PWM START*/
    HAL_TIM_PWM_Stop_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1, MOTOR_RIGHT_TIM.Instance->ARR/2); 

    HAL_TIM_PWM_Stop_IT(&MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&MOTOR_LEFT_TIM, TIM_CHANNEL_1, MOTOR_LEFT_TIM.Instance->ARR/2); 

    /*UART*/
    // HAL_UART_MspInit(&huart1);

}

void TMC2209_Control(TIM_HandleTypeDef *htim,uint16_t GPIO_Pin,GPIO_PinState PinState)
{
    if(htim == &MOTOR_LEFT_TIM)
    {
        if(GPIO_Pin == MOTOR_DIR_LEFT_PIN)
        {
            if(PinState == Motor_right)
            {
                HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, Motor_right);
            }else {
                HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, Motor_left);
            }
        }
        if(GPIO_Pin == MOTOR_EN_LEFT_PIN)
        {
            if(PinState == StepMotor_EN)
            {
                //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, StepMotor_EN);
            }else {
                //HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, StepMotor_DISEN);
            }
        }
    }
    else if(htim == &MOTOR_RIGHT_TIM)
    {
        if(GPIO_Pin == MOTORDIR_PIN)
        {
            if(PinState == Motor_right)
            {
                HAL_GPIO_WritePin(GPIOB, MOTORDIR_PIN, Motor_right);
            }else {
                HAL_GPIO_WritePin(GPIOB, MOTORDIR_PIN, Motor_left);
            }
        }
        if(GPIO_Pin == MOTOREN_PIN)
        {
            if(PinState == StepMotor_EN)
            {
                //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN, StepMotor_EN);
            }else {
                //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN, StepMotor_DISEN);
            }
        }
    }
}

void TMC2209_Begain(void)
{
    TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
    TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);
    HAL_TIM_PWM_Start_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);

    TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
    TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
    HAL_TIM_PWM_Start_IT(&MOTOR_LEFT_TIM, TIM_CHANNEL_1);
}

void TMC2209_SpeedControl(TIM_HandleTypeDef *htim,float circle)
{
    uint32_t psc=0;
    psc = (uint32_t) ((TIMCLOK*ONE_M )/ (ONE_CIRCLE_PULSE * circle *(htim->Instance->ARR-1)));
    if(UNINT16MAX > psc && UNINT16MIN < psc)
    {
        htim->Instance->PSC = psc;
    }
    else 
    {
        if(htim == &MOTOR_RIGHT_TIM)
        {
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        }
        else {
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
        }
    }
}

int pulse_cnt_temp;
int current_circle;
int need_circle;
int need_point_cnt;
/*设置转几圈*/
float TMC2209_SetCircle(TIM_HandleTypeDef *htim,float circle) 
{     
    int circle_temp = circle;

    need_circle = (circle*ONE_CIRCLE_PULSE)/ONE_CIRCLE_PULSE;  //需要转的圈数
    need_point_cnt = (int)(circle * ONE_CIRCLE_PULSE) % ONE_CIRCLE_PULSE; //需要转的脉冲数

    if((need_circle <= 0 && pulse_cnt_temp <= 0))
    {
        pulse_cnt_temp = 0;
        current_circle = 0;
        if(htim == &MOTOR_RIGHT_TIM)
        {
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        }
        else {
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
        }
    }
    else if(need_circle != current_circle || need_point_cnt != pulse_cnt_temp) //尚未达到目标要求
    {
        
    }
    else if(need_circle == current_circle && need_point_cnt == pulse_cnt_temp) { //达到要求
        if(htim == &MOTOR_RIGHT_TIM)
        {
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        }
        else {
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
        }
    }
    return circle_temp;
}

uint8_t usb_tx_virtual_com[256];
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // float temp;
	if(htim == &MOTOR_RIGHT_TIM)
	{
        Motor_pulse_cnt++;
        if(Motor_pulse_cnt>=1500)
        {
            Motor_pulse_cnt = 0;
            Motor_circle_cnt ++;
        }
        sprintf(usb_tx_virtual_com,"circle:%d,pulse:%d\r\n",Motor_circle_cnt,Motor_pulse_cnt);
        CDC_Transmit_FS(usb_tx_virtual_com,sizeof(usb_tx_virtual_com));
    //     pulse_cnt+=1;

    //     if((need_circle != current_circle || need_point_cnt != pulse_cnt_temp) //尚未达到目标要求
    //         && (need_circle > 0 && pulse_cnt_temp>0)) 
    //     {
    //         pulse_cnt_temp+=1;
    //         if(pulse_cnt_temp == ONE_CIRCLE_PULSE)
    //         {
    //             current_circle++;
    //             pulse_cnt_temp = 0;
    //         }
    //     }
	}

    // if(htim == &MOTOR_LEFT_TIM)
    // {
    //     pulse_cnt+=1;

    //     if((need_circle != current_circle || need_point_cnt != pulse_cnt_temp) //尚未达到目标要求
    //         && (need_circle > 0 && pulse_cnt_temp>0)) 
    //     {
    //         pulse_cnt_temp+=1;
    //         if(pulse_cnt_temp == ONE_CIRCLE_PULSE)
    //         {
    //             current_circle++;
    //             pulse_cnt_temp = 0;
    //         }
    //     }
    // }
}



/***********usart driver********

    一帧数据只需 8 byte

*/
#define SYNC_RESERVED 0X05
#define SLAVE_ADDRESS 0X00


uint8_t tmc2209_rx_data[256];

void TMC2209_UartInit(void)
{
    HAL_UART_MspInit(&huart1);
    HAL_UART_Receive(&huart1, tmc2209_rx_data, sizeof(tmc2209_rx_data), 100);
    //TMC2209_SendByte(1);
}

void TMC2209_SendByte(uint8_t data)
{
    HAL_UART_Transmit(&huart1, &data, sizeof(data), 100);
}

void TMC2209_SendData(uint8_t *data,uint8_t len)
{
    for(uint8_t i = 0; i < len; i++)
    {
        TMC2209_SendByte(data[i]);
    }
}

//crc 校验位
void swuart_calc_crc(uint8_t *data , uint8_t len)
{
    int i,j;
    uint8_t *crc = data + (len-1); //located in data last byte of message
    uint8_t currentByte;

    *crc = 0;
    for(i=0; i < (len-1); i++)
    {
        currentByte = data[i];
        for(j=0; j<8; j++)
        {
            if((*crc >> 7) ^ (currentByte&0x01)) //update crc based result of xor opertation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        }
    }
}
