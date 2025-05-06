#ifndef __UARTFACE_H
#define __UARTFACE_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include "Debug.h"
#include "stdio.h"
#include "stdlib.h"

#define FRAME_HEAD 0XA5
#define FRAME_TAIL 0X5B

// 这是步进电机需要转的圈数
#define N_INTER  123 
#define N_OUTER  126

typedef struct  {
    uint8_t buffer[256];
    uint8_t length;
    uint8_t flag;
}copy_buffer;

typedef struct  {
    uint8_t type; // 1:舵机 2:步进电机
    uint8_t id; // 舵机id或步进电机id
    uint16_t option; // 修改项
    union {
        struct {
            float frequency; // PWM频率
            float angle; // 角度
            uint8_t direction; // 方向
        } servo;
        struct {
            float speed; // 步进电机速度
            float circle; // 圈数
            uint8_t direction; // 方向
        } stepper;
        struct {
            float x; // x坐标
            float y; // y坐标
            uint8_t none; // 占位符
        } xy;
    }kinddata;
}control_data;

extern copy_buffer buffer_rx;
extern control_data user_data;

void PrintControlData(const control_data *data);
void RxBufferParse(control_data *data , copy_buffer buffer);
#endif