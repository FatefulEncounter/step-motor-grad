#include "Key.h"
#include "gpio.h"
#include "Led.h"
#include "tim.h"
#include "Tmc2209.h"
#include "Gripper.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define KEY1_PORT GPIOC
#define KEY1_PIN GPIO_PIN_5

#define KEY2_PORT GPIOB
#define KEY2_PIN GPIO_PIN_1


#define KEY_TIMEBASE htim1
enum KeyState
{
    KEY_NON,
    KEY_LONG,    // > 1000ms
    KEY_SHORT,   // < 1000ms
    KEY_RELEASE,
};
enum Keyevnt
{
    NON_EVENT,
    SHORT_EVENT,
    LONG_EVENT,
    RELEASE_EVENT,
};
static enum KeyState key1_state = KEY_NON;
static enum KeyState key2_state = KEY_NON;

static enum Keyevnt key1_event = NON_EVENT;
static enum Keyevnt key2_event = NON_EVENT;

static uint16_t time_base = 0; // 20ms
static uint16_t time_tick = 0; 
/*
    KEY1: PC5
    KEY2: PB1
*/
void Key_Init(void)
{
    /*
        MX_GPIO_Init(void) is inited
    */
    //初始化按键时基
    HAL_TIM_Base_Start_IT(&KEY_TIMEBASE);
}

GPIO_PinState Key_GetState(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin); 
}
/*
    key1 
        长按： 控制电机1的启动和停止
        短按： 控制电机2的启动和停止
    key2
        长按： 控制电机1的方向
        短按： 控制电机2的方向
*/

static bool motor_right = false;
static bool motor_left  = false;

static bool motor_dir    = false;
static bool gripper_down = false;

// #define StepMotor_EN    0
// #define StepMotor_DISEN 1

// #define Motor_right     0
// #define Motor_left      1

void key_deley(uint16_t time) 
{
    // 假设每次循环大约需要 12 个时钟周期（根据编译器优化可能会有所不同）
    // 82MHz 时，1ms 需要 82,000 个时钟周期，因此循环次数为 82,000 / 12 ≈ 6833
    volatile uint32_t count_per_ms = 6833; 
    volatile uint32_t count;

    while (time--) {
        count = count_per_ms;
        while (count--) {
            // 空循环，用于延迟
        }
    }
}

void Key_stepmotor_other_test(void) {

    if (key1_event == SHORT_EVENT) {
        motor_right = !motor_right;
        if(motor_right == true)
        {
            LED_On(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);
        }
        else
        {
            LED_Off(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        }

    } else if (key1_event == LONG_EVENT) {
        motor_left = !motor_left;
        if(motor_left == true)
        {
            LED_On(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,StepMotor_EN);
        }
        else
        {
            LED_Off(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,StepMotor_DISEN);
        }
    }

    if (key2_event == SHORT_EVENT) {
        motor_dir = !motor_dir;

        if(motor_dir == true)
        {
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
        }
        else
        {
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_left);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_left);
        }

        if(motor_dir == true)
            LED_On(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
        else
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
    
    } else if (key2_event == LONG_EVENT) {
        gripper_down = !gripper_down;
        if(gripper_down == true)
        {
            servo_updown(SERVO_DOWN,FAST_MODE,NULL);
            // key_deley(3000);
            HAL_Delay(2000);
            servo_gripper(SERVO_GRIP,FAST_MODE,speed_one);
            // key_deley(3000);
            HAL_Delay(2000);
            servo_gripper(SERVO_RELEASE,FAST_MODE,speed_one);
            // key_deley(3000);
            HAL_Delay(2000);
            servo_updown(SERVO_UP,FAST_MODE,NULL);
        }
    }
}
/*上述是新的一些步进电机的控制方法*/


/*led test*/
void Key_led_test(void)
{
    if(key1_event == SHORT_EVENT)
    {
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
    }
    else if(key1_event == LONG_EVENT)
    {
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
    }
    else if(key2_event == SHORT_EVENT)
    {
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
    }
    else if(key2_event == LONG_EVENT)
    {
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
    }
}

/* right ---- no move*/
void Key_stepmotor1_test(void)
{
    if(key1_event == SHORT_EVENT)
    {
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
        HAL_TIM_PWM_Stop_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    }
    else if(key1_event == LONG_EVENT)
    {
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
        HAL_TIM_PWM_Start_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    }
    else if(key2_event == SHORT_EVENT)
    {
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_left);
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
        HAL_TIM_PWM_Stop_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    }
    else if(key2_event == LONG_EVENT)
    {
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_left);
        TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
        HAL_TIM_PWM_Start_IT(&MOTOR_RIGHT_TIM, TIM_CHANNEL_1);
    }
}  

/*left -----can move */
int Motor_pulse_cnt;
int Motor_circle_cnt;
void Key_stepmotor2_test(void)
{
    if(key1_event == SHORT_EVENT)
    {
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
        // HAL_TIM_PWM_Stop_IT(MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    }
    else if(key1_event == LONG_EVENT)
    {
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
        // HAL_TIM_PWM_Start_IT(MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    }
    else if(key2_event == SHORT_EVENT)
    {
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_left);
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
        LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
        // HAL_TIM_PWM_Stop_IT(MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    }
    else if(key2_event == LONG_EVENT)
    {
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_left);
        TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
        LED_Toggle(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
        // HAL_TIM_PWM_Start_IT(MOTOR_LEFT_TIM, TIM_CHANNEL_1);
    }

}

/**Gripper test */
void key_gripper_test(void)
{
    static gripper_speed level = speed_one;
    static bool mode = SLOW_MODE;
    static bool single[2] = false;
    static uint8_t circle;
    /*up and down*/
    if(key1_event == LONG_EVENT)
    {   
        if(mode==SLOW_MODE)
            single[0] = true;
        else
            servo_updown(SERVO_DOWN,FAST_MODE,NULL);
    }
    else if(key1_event == SHORT_EVENT)
    {
        if(mode==SLOW_MODE)
            single[0] = false;
        else
           servo_updown(SERVO_UP,FAST_MODE,NULL);
    }

    /*grab action*/
    if(key2_event == LONG_EVENT)
    {
        if(mode==SLOW_MODE)
            single[1] = true;
        else
        servo_gripper(SERVO_GRIP,FAST_MODE,NULL);
    }
    else if(key2_event == SHORT_EVENT)
    {
        if(mode==SLOW_MODE)
            single[1] = false;
        else
            servo_gripper(SERVO_RELEASE,FAST_MODE,NULL);
    }

    circle++;
    
    /*fast speed control*/
    if(mode == SLOW_MODE && circle >= 100)
    {
        circle = 0;
        if(single[0] && single[1])
        {
            
            servo_updown(SERVO_DOWN,FAST_MODE,speed_one);
            servo_gripper(SERVO_GRIP,FAST_MODE,speed_one);
        }
        else if(single[0] && !single[1])
        {
            LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
            servo_updown(SERVO_DOWN,FAST_MODE,speed_one);
            servo_gripper(SERVO_RELEASE,FAST_MODE,speed_one);
        }
        else if(!single[0] && single[1])
        {
            servo_updown(SERVO_UP,FAST_MODE,speed_one);
            servo_gripper(SERVO_GRIP,FAST_MODE,speed_one);
        }
        else if(!single[0] && !single[1])
        {
            servo_updown(SERVO_UP,FAST_MODE,speed_one);
            servo_gripper(SERVO_RELEASE,FAST_MODE,speed_one);
        }
    }
    
}


/*to run task--usr need */
void Key_task(void)
{
    // Key_led_test();
    // Key_stepmotor1_test();
    // Key_stepmotor2_test();
    // key_gripper_test();
    
    // Key_stepmotor_other_test();
    
    __disable_irq(); // 禁用全局中断
    key1_event = NON_EVENT;
    key2_event = NON_EVENT;
    __enable_irq(); // 重新启用全局中断
}


void key_level_state()
{
    static int time_cnt;
    
}
void key_press(void)
{
    static int time_cnt[2];
    if(Key_GetState(KEY1_PORT,KEY1_PIN) == GPIO_PIN_RESET)
    {
        switch (key1_state)
        {
            case KEY_NON:
                key1_state = KEY_SHORT;
                break;
            case KEY_SHORT:
                time_cnt[0]++;
                if(time_cnt[0] >= 50)
                {
                    time_cnt[0] =0;
                    key1_state = KEY_LONG;
                }else {
                    key1_state = KEY_SHORT;
                }
                break;
            case KEY_LONG:
                key1_state = KEY_LONG;
                break;
            default:
                break;
        }
    }
    else if(Key_GetState(KEY1_PORT,KEY1_PIN) == GPIO_PIN_SET)
    {
        switch (key1_state) {
            case KEY_SHORT:
                time_cnt[0] = 0;
                key1_state = KEY_NON;
                key1_event = SHORT_EVENT;
                break;
            case KEY_LONG:
                time_cnt[0] = 0;
                key1_state = KEY_NON;
                key1_event = LONG_EVENT;
                break;
            default:
                break;
        }
    }
    
    if(Key_GetState(KEY2_PORT,KEY2_PIN) == GPIO_PIN_RESET)
    {
        switch (key2_state)
        {
            case KEY_NON:
                key2_state = KEY_SHORT;
                break;
            case KEY_SHORT:
                time_cnt[1]++;
                if(time_cnt[1] >= 50)
                {
                    time_cnt[1] =0;
                    key2_state = KEY_LONG;
                }else {
                    key2_state = KEY_SHORT;
                }
                break;
            case KEY_LONG:
                key2_state = KEY_LONG;
                break;
            default:
                break;
        }
    }
    else if(Key_GetState(KEY2_PORT,KEY2_PIN) == GPIO_PIN_SET)
    {
        switch (key2_state) {
            case KEY_SHORT:
                time_cnt[1] =0;
                key2_state = KEY_NON;
                key2_event = SHORT_EVENT;
                break;
            case KEY_LONG:
                time_cnt[1] =0;
                key2_state = KEY_NON;
                key2_event = LONG_EVENT;
                break;
            default:
                break;
        }
    }

    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
    if(htim == &KEY_TIMEBASE)
    {
        time_base++;
        key_press();
    }

}

