#include "Debug.h"

char debugstring[100];
char debugbuffer[100];
char debugrx;
char *thisok="this ok\r\n";
void DebugInit(void)
{
    HAL_UART_MspInit(&huart3);
    sprintf(debugbuffer, "debug success");    
    HAL_UART_Transmit(&huart3, debugbuffer, sizeof(debugbuffer), 100);
    HAL_UART_Receive_IT(&huart3, &debugrx, 1);
}

char caputure_spcial = '\n';
char rx_cnt = 0;
char save_buffer[256];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t num=0;
	if(huart == &huart3)
	{
      num = debugrx-'0';
      sprintf(debugstring,"debugrx:%c num:%d,rx_cnt:%d\r\n",debugrx , num , rx_cnt);
      HAL_UART_Transmit(&huart3, debugstring, sizeof(debugstring), 100);
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
	HAL_UART_Receive_IT(&huart3, &debugrx, 1);
}