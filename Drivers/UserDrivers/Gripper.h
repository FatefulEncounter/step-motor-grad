#ifndef __GRIPPER_H
#define __GRIPPER_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define SERVO_UP            0
#define SERVO_DOWN          1

#define SERVO_GRIP          0
#define SERVO_RELEASE       1

#define FAST_MODE   0
#define SLOW_MODE   1

typedef enum {
    speed_one,
    speed_two,
    speed_three,
    speed_four
} gripper_speed;

//初始化舵机
void servo_init(void);
//控制舵机的停止
void servo_stop(void);
//控制舵机的升降
void servo_updown(bool state,bool mode,gripper_speed level);
//控制舵机进行抓取
void servo_gripper(bool state,bool mode, gripper_speed level);

#endif