#ifndef __UARTFACE_H
#define __UARTFACE_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include "Debug.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#define OPENMV_UART huart6
#define BLUETOOTH_UART huart3
#define FRAME_HEAD 0XA5
#define FRAME_TAIL 0X5B

// 这是步进电机需要转的圈数
#define N_INTER  123 
#define N_OUTER  126
// XY坐标系的范围
#define X_MAX    120
#define Y_MAX    160

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

typedef struct{
    float inter_circle;
    float outer_circle;
}setpmotor_circle;

extern copy_buffer buffer_rx;
extern control_data user_data;
extern setpmotor_circle setpmotor_circle_data;

void uart_opnemv_Init(void);
void TestCopyBuffer(void);

void PrintControlData(const control_data *data);
setpmotor_circle PosPrase(const control_data *data);
void PrintControlData_Type(const control_data *data);
void RxBufferParse(control_data *data, const copy_buffer *buffer);

/* 

        蓝牙专属
*/


typedef struct{

    uint8_t  type;                    // 1:舵机 2:步进电机
    uint8_t  id;                      // 舵机id或步进电机id
    uint16_t option;                  // 修改项
    struct
    {
        bool  servo_act;               // 抓取
        bool  servo_updown;            // 舵机抬起
    }servo;
    struct 
    {
        uint8_t  motor_direction;     // 方向
        float    motor_speed;         // 步进电机速度
        float    motor_circle;        // 圈数
    }motor;
}control_bluetooth_data;


extern copy_buffer xbuffer_bluetooth;
extern control_bluetooth_data user_bluetooth_data;

void Bluetooth_Init(void);
void Bluetooth_SendData(uint8_t *data, uint16_t length);
void Bluetooth_Parsedata(control_bluetooth_data *data, const copy_buffer *buffer);


void Bluetooth_SimpleProcess(const uint8_t data);
void Rxopenmv_SimpleProcess(const uint8_t data);
#endif