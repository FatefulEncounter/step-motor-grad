#include "main.h"

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