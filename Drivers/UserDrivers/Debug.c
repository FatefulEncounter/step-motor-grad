#include "Debug.h"
char debugstring[100];
char debugbuffer[100];
char debugrx;
char *thisok="this ok\r\n";
void DebugInit(void)
{
    HAL_UART_MspInit(&DEBUG_UART);
    sprintf(debugbuffer, "debug success");    
    HAL_UART_Transmit(&DEBUG_UART, debugbuffer, sizeof(debugbuffer), 100);
    HAL_UART_Receive_IT(&DEBUG_UART, &debugrx, 1);
}

char caputure_spcial = '\n';
char rx_cnt = 0;
char save_buffer[256];

void DebugReceive(void)
{
    if(debugrx == caputure_spcial)
    {
        save_buffer[rx_cnt] = '\0';
        rx_cnt = 0;
        HAL_UART_Transmit(&DEBUG_UART, save_buffer, sizeof(save_buffer), 100);
    }
    else
    {
        save_buffer[rx_cnt++] = debugrx;
        if(rx_cnt >= 255)
        {
            rx_cnt = 0;
        }
    }
}

void debugprocess(void)
{
  uint8_t num=0;
  num = debugrx-'0';
  sprintf(debugstring,"debugrx:%c num:%d,rx_cnt:%d\r\n",debugrx , num , rx_cnt);
  HAL_UART_Transmit(&DEBUG_UART, debugstring, sizeof(debugstring), 100);
  if(num>=0 && num<=10)
  {
    TMC2209_Begain();
    TMC2209_SpeedControl(&htim4,1);
  }
    else 
  {
    TMC2209_SpeedControl(&htim4,0);
  }
    usecircal = num;
}