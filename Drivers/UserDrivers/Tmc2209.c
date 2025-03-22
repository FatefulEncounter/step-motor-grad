#include "Tmc2209.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>
#include <stdio.h>
#include "Debug.h"


#define UNINT16MAX      65535
#define UNINT16MIN      0
#define TIMCLOK		    84
#define ONE_M           1000000

#define StepMotor_EN    0
#define StepMotor_DISEN 1
#define Motor_right     0
#define Motor_left      1
#define StepMotor_DIR   0
#define StepMotor_Step  8
/*
THE MOTOR_STEP    !!!!!right!!!!
UART5 GPIO Configuration
    PC12   ------> UART5_TX
    PD2    ------> UART5_RX
TMC2209 EN AND DIR
    GPIOX  ------> GPIOB
    PB5    ------> TMC2209_EN
    PB7    ------> TMC2209_DIR
TIM4 GPIO Configuration
    PB6    ------> TIM4_CH1
*/
#define MOTORUART_TXPIN GPIO_PIN_12
#define MOTORUART_RXPIN GPIO_PIN_2
#define MOTOREN_PIN  GPIO_PIN_5
#define MOTORDIR_PIN GPIO_PIN_7
#define MOTORSTEP_PIN GPIO_PIN_6
#define MOTOR_RIGHT_TIM htim4

/*
THE MOTOR_STEP    !!!!!left!!!!

TMC2209 EN AND DIR
    GPIOX  ------> GPIOB
                   GPIOC
    PA7    ------> TMC2209_EN
    PC4    ------> TMC2209_DIR
TIM3 GPIO Configuration
    PA6    ------> TIM3_CH1
UART5 GPIO Configuration
    PC12   ------> UART5_TX
    PD2    ------> UART5_RX
*/
#define MOTOR_EN_LEFT_PIN     GPIO_PIN_7
#define MOTOR_DIR_LEFT_PIN    GPIO_PIN_4
#define MOTOR_STEP_LEFT_PIN   GPIO_PIN_6
#define MOTOR_UART_LEFT_TXPIN GPIO_PIN_12
#define MOTOR_UART_LEFT_RXPIN GPIO_PIN_2
#define MOTOR_LEFT_TIM htim3

#if (StepMotor_Step == 8)
#define ONE_CIRCLE_PULSE 1600
#elif(StepMotor_Step == 16)
#define ONE_CIRCLE_PULSE 3200
#elif(StepMotor_Step == 32)
#define ONE_CIRCLE_PULSE 6400
#elif(StepMotor_Step == 64)
#define ONE_CIRCLE_PULSE 12800
#endif

int pulse_cnt;
float usecircal;
char tmc2209buffer[100];
void TMC2209_Init(void)
{
    /*EN & DIR INIT*/
    HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN|MOTORDIR_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, GPIO_PIN_SET);
    
    /*PWM START*/
    HAL_TIM_PWM_Stop_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1, MOTOR_RIGHT_TIM.Instance->ARR/2); 

    HAL_TIM_PWM_Stop_IT(&MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&MOTOR_LEFT_TIM, TIM_CHANNEL_1, MOTOR_LEFT_TIM.Instance->ARR/2); 

    /*UART*/
    HAL_UART_MspInit(&huart1);

}

void TMC2209_Control(TIM_HandleTypeDef *htim,uint16_t GPIO_Pin,GPIO_PinState PinState)
{
    if(htim == &MOTOR_LEFT_TIM)
    {
        if(GPIO_Pin == MOTOR_DIR_LEFT_PIN)
        {
            if(PinState == Motor_right)
            {
                HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, GPIO_PIN_SET);
            }else {
                HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_LEFT_PIN, GPIO_PIN_RESET);
            }
        }
        if(GPIO_Pin == MOTOR_EN_LEFT_PIN)
        {
            if(PinState == StepMotor_EN)
            {
                //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, GPIO_PIN_SET);
            }else {
                //HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOA, MOTOR_EN_LEFT_PIN, GPIO_PIN_RESET);
            }
        }
    }
    else if(htim == &MOTOR_RIGHT_TIM)
    {
        if(GPIO_Pin == MOTORDIR_PIN)
        {
            if(PinState == Motor_right)
            {
                HAL_GPIO_WritePin(GPIOB, MOTORDIR_PIN, GPIO_PIN_SET);
            }else {
                HAL_GPIO_WritePin(GPIOB, MOTORDIR_PIN, GPIO_PIN_RESET);
            }
        }
        if(GPIO_Pin == MOTOREN_PIN)
        {
            if(PinState == StepMotor_EN)
            {
                //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN, GPIO_PIN_SET);
            }else {
                //HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(GPIOB, MOTOREN_PIN, GPIO_PIN_RESET);
            }
        }
    }
}

void TMC2209_Begain(void)
{
    TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
    TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);

    TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
    TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
}

void TMC2209_SpeedControl(TIM_HandleTypeDef *htim,float circle)
{
    uint32_t psc=0;
    psc = TIMCLOK*ONE_M / (ONE_CIRCLE_PULSE * circle *(htim->Instance->ARR-1));
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

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    float temp;
	if(htim == &MOTOR_RIGHT_TIM)
	{
        pulse_cnt+=1;

        if((need_circle != current_circle || need_point_cnt != pulse_cnt_temp) //尚未达到目标要求
            && (need_circle > 0 && pulse_cnt_temp>0)) 
        {
            pulse_cnt_temp+=1;
            if(pulse_cnt_temp == ONE_CIRCLE_PULSE)
            {
                current_circle++;
                pulse_cnt_temp = 0;
            }
        }
	}

    if(htim == &MOTOR_LEFT_TIM)
    {
        pulse_cnt+=1;

        if((need_circle != current_circle || need_point_cnt != pulse_cnt_temp) //尚未达到目标要求
            && (need_circle > 0 && pulse_cnt_temp>0)) 
        {
            pulse_cnt_temp+=1;
            if(pulse_cnt_temp == ONE_CIRCLE_PULSE)
            {
                current_circle++;
                pulse_cnt_temp = 0;
            }
        }
    }
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
