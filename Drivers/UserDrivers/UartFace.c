#include "UartFace.h"

/*
    通过串口6接受openmv的数据，并且在此文件中写一份函数进行解析数据
    使用了串口六

帧格式：
    帧头 | 数据长度 |     数据内容   | 校验码 | 帧尾
    0xA5 | 0x03     | 0x01 0x02 0x03 | 0x06   | 0x5B

校验码的计算方法
  uint8_t  calc_checksum ^= Uart6_RxBuffer[i];

    其中Uart6_RxBuffer[i]是数据内容的每一个字节
    计算校验码时不包括帧头、数据长度、校验码和帧尾

舵机：12字节
    数据内容：类别（1），舵机id（1），修改项（2），pwm频率(4个字节)，角度(3个字节，前两个整数，最后一个小数)， 方向（1字节）
步进电机：13字节
    数据内容：类别（1），步进电机id（1），修改项（2），速度(4个字节，前三字节存整数)，方向（1），圈数(4个字节，前三字节存整数)
xy坐标系：
    数据内容：类别（1），x坐标（2个字节），y坐标（2个字节）  ---> 查看的物料位置
    0xA5 | 0x04     | 0x00 0x00 0xFF 0XFF | 0xXX   | 0x5B    --> x = 0  y = 255
*/

uint8_t Uart6_RxBuffer[256];
uint8_t Uart6_buf_index = 0;
uint8_t data_len = 0;
uint8_t checksum = 0;

/*
    1，先判断是否是舵机还是步进电机
    2，判断是哪个舵机还是步进电机
    3，判断修改项是什么
    
    4，舵机：修改pwm频率、角度、方向
    4, 步进电机：修改速度、方向、圈数

*/
enum FRAME_STATE {
    FRAME_READING_HEAD = 0,
    FRAME_READING_LENGTH,
    FRAME_READING_DATA,
    FRAME_READING_CHECKSUM,
    FRAME_READING_TAIL
};

enum classify {
    CLASSIFY_SERVO = 1,
    CLASSIFY_STEPPER = 2,
    CLASSIFY_xy = 3,
};

enum classify_id {
    CLASSIFY_ID_ONE = 1,
    CLASSIFY_ID_TWO = 2,
    CLASSIFY_ID_THREE = 3,
    CLASSIFY_ID_FOUR = 4,
};

enum option {
    OPTION_PWM_FREQUENCY = (1 << 0),    // 位0
    OPTION_ANGLE = (1 << 1),            // 位1
    OPTION_DIRECTION = (1 << 2),        // 位2
    OPTION_SPEED = (1 << 3),            // 位3
    OPTION_CIRCLE = (1 << 4),           // 位4
};

copy_buffer buffer_rx = {0};
control_data user_data = {0};

void RxBufferParse(control_data *data , copy_buffer buffer)
{
    if (buffer.flag == 1)
    {
        buffer.flag = 0;
        if (buffer.buffer[0] == CLASSIFY_xy)
        {
            data->type = CLASSIFY_xy;
            data->kinddata.xy.x = (buffer.buffer[1] << 8) | buffer.buffer[2]; // 读取x坐标
            data->kinddata.xy.y = (buffer.buffer[3] << 8) | buffer.buffer[4]; // 读取y坐标

        }
        else if (buffer.buffer[0] == CLASSIFY_SERVO)
        {
            data->type = CLASSIFY_SERVO;
            data->id = buffer.buffer[1]; // 读取舵机id
            data->option = (buffer.buffer[2] << 8) | buffer.buffer[3]; // 读取修改项

            // 判断需要修改的选项
            if (data->option & OPTION_PWM_FREQUENCY) {
                data->kinddata.servo.frequency = (buffer.buffer[4] << 24) |
                                                 (buffer.buffer[5] << 16) |
                                                 (buffer.buffer[6] << 8) |
                                                 buffer.buffer[7]; // 读取频率
            }

            if (data->option & OPTION_ANGLE) {
                data->kinddata.servo.angle = (buffer.buffer[8] << 16) |
                                             (buffer.buffer[9] << 8) |
                                             buffer.buffer[10]; // 读取角度
            }

            if (data->option & OPTION_DIRECTION) {
                data->kinddata.servo.direction = buffer.buffer[11]; // 读取方向
            }
        }
        else if (buffer.buffer[0] == CLASSIFY_STEPPER)
        {
            data->type = CLASSIFY_STEPPER;
            data->id = buffer.buffer[1]; // 读取步进电机id
            data->option = (buffer.buffer[2] << 8) | buffer.buffer[3]; // 读取修改项
            // 判断需要修改的选项
            if (data->option & OPTION_SPEED) {
                data->kinddata.stepper.speed = (buffer.buffer[4] << 24) |
                                               (buffer.buffer[5] << 16) |
                                               (buffer.buffer[6] << 8) |
                                               buffer.buffer[7]; // 读取速度
            }

            if (data->option & OPTION_DIRECTION) {
                data->kinddata.stepper.direction = buffer.buffer[8]; // 读取方向
            }

            if (data->option & OPTION_CIRCLE) {
                data->kinddata.stepper.circle = (buffer.buffer[9] << 24) |
                                                (buffer.buffer[10] << 16) |
                                                (buffer.buffer[11] << 8) |
                                                 buffer.buffer[12]; // 读取圈数
            }

        }
        else
        {
            // 其他类型数据
        }
    }
}

void UartFace_CallBack(uint8_t data) {
    static uint8_t state = 0;

    switch (state) {
        case FRAME_READING_HEAD: // 等待帧头
            if (data == FRAME_HEAD) {
                state = FRAME_READING_LENGTH;
            }
            break;
        case FRAME_READING_LENGTH: // 读取数据长度
            data_len = data;
            if (data_len > sizeof(Uart6_RxBuffer)) {
                // 数据长度超出缓冲区大小，重置状态
                state = FRAME_READING_HEAD;
                break;
            }
            Uart6_buf_index = 0;
            state = FRAME_READING_DATA;
            break;
        case FRAME_READING_DATA: // 读取数据内容
            if (Uart6_buf_index < data_len) {
                // 继续接收数据内容
                Uart6_RxBuffer[Uart6_buf_index++] = data;
            } else if (Uart6_buf_index == data_len) {
                // 接收校验码
                checksum = data;
                state = FRAME_READING_TAIL;
            } else {
                // 数据长度异常，重置状态
                state = FRAME_READING_HEAD;
            }
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
                    __disable_irq(); // 禁用中断，防止并发访问
                    buffer_rx.length = data_len;
                    memcpy(buffer_rx.buffer, Uart6_RxBuffer, data_len);
                    buffer_rx.flag = 1;
                    __enable_irq(); // 重新启用中断
                } else {
                    // 校验失败，丢弃数据并重置状态
                    memset(Uart6_RxBuffer, 0, sizeof(Uart6_RxBuffer));
                    Uart6_buf_index = 0;
                    data_len = 0;
                }
            }
            state = FRAME_READING_HEAD;
            break;

        default:
            // 未知状态，重置状态机
            state = FRAME_READING_HEAD;
            break;
    }
}

// uint8_t rx6data;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        uint8_t data = *huart->pRxBuffPtr;
        UartFace_CallBack(data);
        HAL_UART_Receive_IT(&huart6, NULL, 1);
    }else if(huart->Instance == USART3){
        debugprocess();
        HAL_UART_Receive_IT(&huart3, &debugrx, 1);
    }
}

//按照 type进行打印 --> 由于union的原因，打印的时候需要判断类型
void PrintControlData_Type(const control_data *data) {
    printf("Control Data:\n");
    printf("  Type: %u\n", data->type);
    printf("  ID: %u\n", data->id);
    printf("  Option: 0x%04X\n", data->option);

    switch (data->type) {
        case CLASSIFY_SERVO:
            printf("  Servo Data:\n");
            if (data->option & OPTION_PWM_FREQUENCY) {
                printf("    PWM Frequency: %.2f\n", data->kinddata.servo.frequency);
            }
            if (data->option & OPTION_ANGLE) {
                printf("    Angle: %.2f\n", data->kinddata.servo.angle);
            }
            if (data->option & OPTION_DIRECTION) {
                printf("    Direction: %u\n", data->kinddata.servo.direction);
            }
            break;

        case CLASSIFY_STEPPER:
            printf("  Stepper Data:\n");
            if (data->option & OPTION_SPEED) {
                printf("    Speed: %.2f\n", data->kinddata.stepper.speed);
            }
            if (data->option & OPTION_CIRCLE) {
                printf("    Circle: %.2f\n", data->kinddata.stepper.circle);
            }
            if (data->option & OPTION_DIRECTION) {
                printf("    Direction: %u\n", data->kinddata.stepper.direction);
            }
            break;

        case CLASSIFY_xy:
            printf("  XY Data:\n");
            printf("    X: %.2f\n", data->kinddata.xy.x);
            printf("    Y: %.2f\n", data->kinddata.xy.y);
            break;

        default:
            printf("  Unknown Type\n");
            break;
    }
}