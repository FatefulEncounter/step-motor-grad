#ifndef LED_H
#define LED_H
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal.h"
// Define the GPIO port and pin for the LED
#define LED_GPIO_READ_PORT GPIOA
#define LED_GPIO_READ_PIN  GPIO_PIN_8

#define LED_GPIO_BULE_PORT GPIOC
#define LED_GPIO_BULE_PIN  GPIO_PIN_9

#define LED_GPIO_GREEN_PORT GPIOC
#define LED_GPIO_GREEN_PIN  GPIO_PIN_8

#define  LED_ON  1
#define  LED_OFF 0

// Function prototypes
void LED_Init(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void LED_On(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void LED_Off(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void LED_Toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

#endif // LED_H