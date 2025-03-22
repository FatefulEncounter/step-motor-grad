#include "Led.h"
#include "gpio.h"

// Function prototypes
void LED_Init(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, LED_OFF);
}
void LED_On(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, LED_ON);
}
void LED_Off(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, LED_OFF);
}
void LED_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}


