#include "Gripper.h"
#include <stdio.h>

#define SERVO_LIFT_CHANNEL    TIM_CHANNEL_1
#define SERVO_LIFT_TIM        htim5
#define SERVO_UPDOWN_CHANNEL  TIM_CHANNEL_2
#define SERVO_UPDOWN_TIM      htim5
#define SERVO_RIGHT_CHANNEL   TIM_CHANNEL_3
#define SERVO_RIGHT_TIM       htim5
/*
    20ms
    arr = 2000
    20/2000 = 0.01ms
    0.1ms -> cnt: 10

SERVO:
    sg90：180°舵机
        0->0.5MS
        45->1MS
        90->1.5MS
        135->2MS
        180->2.5MS
    MG996R：360°舵机
        0.5ms-1ms  顺时针最快
        1.4ms      顺时针最慢
        1.5MS      停止状态
        1.6ms      逆时针最慢
        2.5ms-2ms  逆时针最快
*/

#define MIN_RATE 0.0025   // 0
#define MID_RATE 0.0750   // 90
#define MAX_RATE 0.1250   // 180

#define SERVO_LIFT          0
#define SERVO_RIGHT         1

#define SG90_0              50
#define SG90_20             72
#define SG90_30             83
#define SG90_45             100
#define sg90_65             122
#define sg90_70             129
#define SG90_90             150
#define SG90_100            161

#define SG90_135            200
#define SG90_180            250

#define MG996R_stop         150
#define MG996R_LIFT_LOW     140
#define MG996R_RIGHT_LOW    160

/*
big servo:
    90°左右伸直（过了）
    角度越小则向上移动
    20-65
        20 抬起来

small servo 1(ccr1):
    90-180°范围
    90°  抬
    180° 捡


small servo 2(ccr3):
    0-90°范围
    90°  抬
    0° 捡
*/

//初始化舵机
void servo_init(void)
{
    HAL_TIM_PWM_Start(&SERVO_LIFT_TIM, SERVO_LIFT_CHANNEL);
    HAL_TIM_PWM_Start(&SERVO_UPDOWN_TIM, SERVO_UPDOWN_CHANNEL);
    HAL_TIM_PWM_Start(&SERVO_RIGHT_TIM, SERVO_RIGHT_CHANNEL);

    SERVO_LIFT_TIM.Instance->CCR1   = SG90_135;
    SERVO_UPDOWN_TIM.Instance->CCR2 = SG90_20;
    SERVO_RIGHT_TIM.Instance->CCR3  = SG90_45;
}

//控制舵机的停止
void servo_stop(void)
{
    SERVO_LIFT_TIM.Instance->CCR1   = SG90_135;
    SERVO_UPDOWN_TIM.Instance->CCR2 = SG90_20;
    SERVO_RIGHT_TIM.Instance->CCR3  = SG90_45;

    HAL_TIM_PWM_Stop(&SERVO_LIFT_TIM, SERVO_LIFT_CHANNEL);
    HAL_TIM_PWM_Stop(&SERVO_UPDOWN_TIM, SERVO_UPDOWN_CHANNEL);
    HAL_TIM_PWM_Stop(&SERVO_RIGHT_TIM, SERVO_RIGHT_CHANNEL);
}
//控制舵机的升降
void servo_updown(bool state)
{   
    if(state == SERVO_UP)
        SERVO_UPDOWN_TIM.Instance->CCR2 = SG90_20;
    else
        SERVO_UPDOWN_TIM.Instance->CCR2 = sg90_65;
}
//控制舵机进行抓取
void servo_gripper(bool state)
{
    if(state == SERVO_GRIP)
    {
        SERVO_LIFT_TIM.Instance->CCR1   = SG90_180;
        SERVO_RIGHT_TIM.Instance->CCR3 = SG90_0;
    }
    else
    {
        SERVO_LIFT_TIM.Instance->CCR1   = SG90_135;
        SERVO_RIGHT_TIM.Instance->CCR3  = SG90_45;
    }
}