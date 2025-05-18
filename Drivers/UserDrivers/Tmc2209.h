#include "main.h"
#include "stdbool.h"

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


extern int pulse_cnt;
extern float usecircal;
extern char tmc2209buffer[100];
/*set circle*/
extern int pulse_cnt_temp;
extern int current_circle;
extern int need_circle;
extern int need_point_cnt;
/************/

void TMC2209_Init(void);
void TMC2209_Begain(void);
float TMC2209_SetCircle(TIM_HandleTypeDef *htim,float circle);
void TMC2209_Control(TIM_HandleTypeDef *htim,uint16_t GPIO_Pin,GPIO_PinState PinState);
void TMC2209_SpeedControl(TIM_HandleTypeDef *htim,float circle);

/********usart driver********/

void TMC2209_UartInit(void);
void TMC2209_SendByte(uint8_t data);
void TMC2209_SendData(uint8_t *data,uint8_t len);


/*************************** */
void tmc2209_simpleprocess(const float x, const float y);
void get_tmc2209_simple_flag(uint8_t *x,uint8_t *y);

/************************* */
void xy_stepmotor(void);
