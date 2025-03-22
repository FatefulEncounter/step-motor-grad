#include "Key.h"
#include "gpio.h"
#include "Led.h"
#include "tim.h"

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
void Key_task(void)
{
    Key_led_test();
    key1_event = NON_EVENT;
    key2_event = NON_EVENT;
}

void key_press(void)
{
    static int time_cnt;
    if(Key_GetState(KEY1_PORT,KEY1_PIN) == GPIO_PIN_RESET)
    {
        switch (key1_state)
        {
            case KEY_NON:
                key1_state = KEY_SHORT;
                break;
            case KEY_SHORT:
                time_cnt++;
                if(time_cnt >= 50)
                {
                    time_cnt =0;
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
                key1_state = KEY_NON;
                key1_event = SHORT_EVENT;
                break;
            case KEY_LONG:
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
                time_cnt++;
                if(time_cnt >= 50)
                {
                    time_cnt =0;
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
                key2_state = KEY_NON;
                key2_event = SHORT_EVENT;
                break;
            case KEY_LONG:
                key2_state = KEY_NON;
                key2_event = LONG_EVENT;
                break;
            default:
                break;
        }
    }
    Key_task();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &KEY_TIMEBASE)
    {
        key_press();
        // LED_Toggle(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
    }

}

