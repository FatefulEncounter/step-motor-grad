#ifndef __GRIPPER_H
#define __GRIPPER_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>

#define SERVO_UP            0
#define SERVO_DOWN          1

#define SERVO_GRIP          0
#define SERVO_RELEASE       1

//初始化舵机
void servo_init(void);
//控制舵机的停止
void servo_stop(void);
//控制舵机的升降
void servo_updown(bool state);
//控制舵机进行抓取
void servo_gripper(bool state);

#endif