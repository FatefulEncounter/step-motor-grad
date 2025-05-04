#include "UartFace.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "Debug.h"
/*
    通过串口6接受openmv的数据，并且在此文件中写一份函数进行解析数据
    使用了串口六

帧格式：
    帧头 | 数据长度 | 数据内容 | 校验码 | 帧尾
    0xA5 | 0x03     | 0x01 0x02 0x03 | 0x06 | 0x5B


舵机：12字节
    数据内容：类别（1），舵机id（1），修改项（2），pwm频率(4个字节)，角度(3个字节，前两个整数，最后一个小数)， 方向（1字节）
步进电机：13字节
    数据内容：类别（1），步进电机id（1），修改项（2），速度(4个字节，前三字节存整数)，方向（1），圈数(4个字节，前三字节存整数)
*/


#define FRAME_HEAD 0XA5
#define FRAME_TAIL 0X5B

uint8_t Uart6_RxBuffer[256];
uint8_t Uart6_buf_index = 0;
uint8_t data_len = 0;
uint8_t checksum = 0;

enum FRAME_STATE {
    FRAME_READING_HEAD = 0,
    FRAME_READING_LENGTH,
    FRAME_READING_DATA,
    FRAME_READING_CHECKSUM,
    FRAME_READING_TAIL
}usart6;

typedef struct pos_xy {
    float x;
    float y;
};

pos_xy PosationParse(pos_xy *arg)
{

}
void UART_ReceiveCallback(uint8_t data) {
    static uint8_t state = 0;

    switch (state) {
        case FRAME_READING_HEAD: // 等待帧头
            if (data == FRAME_HEAD) {
                state = FRAME_READING_LENGTH;
            }
            break;
        case FRAME_READING_LENGTH: // 读取数据长度
            data_len = data;
            Uart6_buf_index = 0;
            state = FRAME_READING_DATA;
            break;
        case FRAME_READING_DATA: // 读取数据内容
            if (Uart6_buf_index < data_len) {
                Uart6_RxBuffer[Uart6_buf_index++] = data;
            } else {
                state = FRAME_READING_CHECKSUM;
            }
            break;
        case FRAME_READING_CHECKSUM: // 读取校验码
            checksum = data;
            state = FRAME_READING_TAIL;
            break;
        case FRAME_READING_TAIL: // 读取帧尾
            if (data == FRAME_TAIL) {
                // 校验数据
                uint8_t calc_checksum = 0;
                for (uint8_t i = 0; i < data_len; i++) {
                    calc_checksum ^= Uart6_RxBuffer[i];
                }
                if (calc_checksum == checksum) {
                    // 数据校验通过，处理数据
                    // TODO: 处理 Uart6_RxBuffer 中的数据
                }
            }
            state = FRAME_READING_HEAD;
            break;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        
        uint8_t data = *huart->pRxBuffPtr;
        UART_ReceiveCallback(data);
        HAL_UART_Receive_Start_IT(huart); // 继续接收下一个字节

    }else if(huart->Instance == USART3){
        
        debugprocess();
        HAL_UART_Receive_IT(&huart3, &debugrx, 1);
    
    }
}