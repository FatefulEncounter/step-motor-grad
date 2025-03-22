#ifndef __KEY_H
#define __KEY_H

#include "main.h"
void Key_Init(void);
void Key_task(void);
GPIO_PinState Key_GetState(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);


#endif