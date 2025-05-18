#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include "stdbool.h"
extern int Motor_pulse_cnt;
extern int Motor_circle_cnt;

extern bool motor_right ;
extern bool motor_left  ;

extern bool motor_dir   ;
extern bool gripper_down;

void Key_Init(void);
void Key_task(void);

GPIO_PinState Key_GetState(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

void Key_ControlMotor(TIM_HandleTypeDef* motor_port, uint16_t motor_pin, bool enable, bool direction);
void Key_stepmotor_other_test(void);
void gripper_test(void);

void key_deley(uint16_t time);

#endif