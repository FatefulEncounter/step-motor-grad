#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Tmc2209.h"
#include <stdint.h>

extern char debugstring[100];
extern char *thisok;
extern char debugbuffer[100];
extern char debugrx;
void DebugInit(void);
void debugprocess(void);