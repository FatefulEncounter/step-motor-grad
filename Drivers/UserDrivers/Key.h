#ifndef __KEY_H
#define __KEY_H

#include "main.h"

extern int Motor_pulse_cnt;
extern int Motor_circle_cnt;


void Key_Init(void);
void Key_task(void);
GPIO_PinState Key_GetState(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);


#endif