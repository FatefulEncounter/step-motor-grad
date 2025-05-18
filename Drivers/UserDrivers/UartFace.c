#include "UartFace.h"
#include "Led.h"
#include "Gripper.h"
/*
    通过串口6接受openmv的数据，并且在此文件中写一份函数进行解析数据
    使用了串口六
帧格式：
    举例：
        帧头 | 数据长度  |     数据内容   | 校验码 | 帧尾
        0xA5 | 0x03     | 0x01 0x02 0x03 | 0x06   | 0x5B
校验码的计算方法
    uint8_t  calc_checksum ^= Uart6_RxBuffer[i];
    其中Uart6_RxBuffer[i]是数据内容的每一个字节
    计算校验码时不包括帧头、数据长度、校验码和帧尾
舵机：
    数据内容：类别(1),舵机id(1),修改项(1),pwm占空比(1个字节)，角度(3个字节，前两个整数，最后一个小数 最后一个字节的范围是 0-9)， 方向（1字节）
步进电机：
    数据内容：类别（1），步进电机id（1），修改项（1），速度(2个字节)， 方向（1），圈数(3个字节，前2字节存整数) 
xy坐标系：
    数据内容： 类别（1），x坐标（2个字节），y坐标（2个字节）  ---> 查看的物料位置
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
setpmotor_circle setpmotor_circle_data = {0};

static void Bluttooth_CallBack(uint8_t data);



setpmotor_circle PosPrase(const control_data *data) {
    // 提取 x 和 y 坐标
    float x = data->kinddata.xy.x;
    float y = data->kinddata.xy.y;
    setpmotor_circle setpmotor_circle_temp = {0};

    // 检查类型是否为 CLASSIFY_xy
    if (data->type != CLASSIFY_xy) {
        printf("Error: Invalid data type for PosPrase. Expected CLASSIFY_xy.\n");
        return setpmotor_circle_temp;
    }
    // 边界检查
    if (x < 0 || x > X_MAX || y < 0 || y > Y_MAX) {
        printf("Error: Coordinates out of range. X: %.2f, Y: %.2f\n", x, y);
        return setpmotor_circle_temp;
    }
    // 计算圈数
    setpmotor_circle_temp.inter_circle = (x / X_MAX) * N_INTER; // 内电机圈数
    setpmotor_circle_temp.outer_circle = (y / Y_MAX) * N_OUTER; // 外电机圈数

    return setpmotor_circle_temp;
}

void RxBufferParse(control_data *data, const copy_buffer *buffer)
{
    if (buffer->flag == 1)
    {
        printf("hello RxBufferParse \n");
        // 清除标志位
        ((copy_buffer *)buffer)->flag = 0;

        // 根据类别解析数据
        switch (buffer->buffer[0])
        {
            case CLASSIFY_SERVO: // 舵机数据解析
            {
                data->type = CLASSIFY_SERVO;
                data->id = buffer->buffer[1]; // 读取舵机id
                data->option = buffer->buffer[2]; // 读取修改项

                uint8_t offset = 3; // 数据内容起始偏移量

                // 判断需要修改的选项
                if (data->option & OPTION_PWM_FREQUENCY)
                {
                    data->kinddata.servo.frequency = buffer->buffer[offset] / 1000.0f; // 读取PWM占空比并转换为浮点数

                    if(data->kinddata.servo.frequency > 0.125f)
                    {
                        data->kinddata.servo.frequency = 0.125f;
                    }
                    else if(data->kinddata.servo.frequency < 0.025f) 
                    {
                        data->kinddata.servo.frequency = 0.025f;
                    }
                    offset += 1; // 占用1字节
                }

                if (data->option & OPTION_ANGLE)
                {
                    // 读取角度（前两个字节为整数部分，第 3 个字节为小数部分）
                    uint16_t angle_int = (buffer->buffer[offset] << 8) | buffer->buffer[offset + 1];
                    uint8_t angle_decimal = buffer->buffer[offset + 2];
                    data->kinddata.servo.angle = angle_int + (angle_decimal / 10.0f); // 合成浮点角度   

                    //printf("buffer->buffer[offset] << 8 : %x,   %x \n", buffer->buffer[offset], (buffer->buffer[offset + 1]));
                    //printf("angle_int = %d, angle_decimal = %d \n", angle_int, angle_decimal);
                    //printf("angle = %f \n", data->kinddata.servo.angle);
                    // 限制角度范围在 0 到 180 度之间  如果舵机ID是360°的就需要加限制条件 去处理360°的舵机
                    if(data->kinddata.servo.angle > 180.0f)
                    {
                        data->kinddata.servo.angle = 180.0f;
                    }
                    else if(data->kinddata.servo.angle < 0.0f) 
                    {
                        data->kinddata.servo.angle = 0.0f;
                    }
                    offset += 3; // 占用3字节
                }

                if (data->option & OPTION_DIRECTION)
                {
                    data->kinddata.servo.direction = buffer->buffer[offset]; // 读取方向
                    data->kinddata.servo.direction = (data->kinddata.servo.direction == 0) ? 0 : 1; // 确保方向值为 0 或 1
                    offset += 1; // 占用1字节
                }
                break;
            }

            case CLASSIFY_STEPPER: // 步进电机数据解析
            {
                data->type = CLASSIFY_STEPPER;
                data->id = buffer->buffer[1]; // 读取步进电机id
                data->option = buffer->buffer[2]; // 读取修改项

                uint8_t offset = 3; // 数据内容起始偏移量

                // 判断需要修改的选项
                if (data->option & OPTION_CIRCLE)
                {
                    // 读取圈数（前两个字节为整数部分，第 3 个字节为小数部分）
                    uint16_t circle_int = (buffer->buffer[offset] << 8) | buffer->buffer[offset + 1];
                    uint8_t circle_decimal = buffer->buffer[offset + 2];
                    data->kinddata.stepper.circle = circle_int + (circle_decimal / 10.0f); // 合成浮点圈数
                    if(data->kinddata.stepper.circle > 126.0f) // 限制圈数范围
                    {
                        data->kinddata.stepper.circle = 126.0f;
                    }
                    else if(data->kinddata.stepper.circle < 0.0f) 
                    {
                        data->kinddata.stepper.circle = 0.0f;
                    }
                    offset += 3; // 占用3字节
                }

                if (data->option & OPTION_SPEED)
                {
                    data->kinddata.stepper.speed = (buffer->buffer[offset] << 8) | (buffer->buffer[offset + 1]); // 读取速度（2字节）
                    if(data->kinddata.stepper.speed > 5250.0f) // 限制速度范围
                    {
                        printf("speed > 5250.0f \n");
                        data->kinddata.stepper.speed = 5250.0f;
                    }
                    else if(data->kinddata.stepper.speed < 52.0f) 
                    {
                        printf("speed < 52.0f \n");
                        data->kinddata.stepper.speed = 52.0f;
                    }
                    else
                    {
                        printf("speed \n");
                    }
                    offset += 2; // 占用2字节
                }
                if (data->option & OPTION_DIRECTION)
                {
                    data->kinddata.stepper.direction = buffer->buffer[offset]; // 读取方向
                    data->kinddata.stepper.direction = (data->kinddata.stepper.direction == 0) ? 0 : 1; // 确保方向值为 0 或 1
                    offset += 1; // 占用1字节
                }                
                break;
            }

            case CLASSIFY_xy: // XY 坐标数据解析
            {
                data->type = CLASSIFY_xy;
                data->kinddata.xy.x = (buffer->buffer[1] << 8) | buffer->buffer[2]; // 读取x坐标
                data->kinddata.xy.y = (buffer->buffer[3] << 8) | buffer->buffer[4]; // 读取y坐标
                if(data->kinddata.xy.x > X_MAX) // 限制x坐标范围
                {
                    data->kinddata.xy.x = X_MAX;
                }
                else if(data->kinddata.xy.x < 0.0f) 
                {
                    data->kinddata.xy.x = 0.0f;
                }
                
                if(data->kinddata.xy.y > Y_MAX) // 限制y坐标范围
                {
                    data->kinddata.xy.y = Y_MAX;
                }
                else if(data->kinddata.xy.y < 0.0f) 
                {
                    data->kinddata.xy.y = 0.0f;
                }

                break;
            }

            default:
                printf("Error: Unknown data type.\n");
                break;
            }
        // 打印解析后的数据
        if(data->type != 0)
            PrintControlData_Type(data);
    }
}

static void UartFace_CallBack(uint8_t data) {
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
        case FRAME_READING_DATA:  // 读取数据内容
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

uint8_t rx6data[100];
uint8_t rx3data[100];

void uart_opnemv_Init(void)
{
    // 初始化蓝牙串口
    HAL_UART_MspInit(&OPENMV_UART);
    HAL_UART_Receive_IT(&OPENMV_UART, &rx6data[0], 1);
    HAL_Delay(1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        // UartFace_CallBack(rx6data[0]);
        Rxopenmv_SimpleProcess(rx6data[0]);
        HAL_UART_Receive_IT(huart, &rx6data[0], 1);
    }
    if(huart->Instance == USART3){

    //    Bluttooth_CallBack(rx3data);
        Bluetooth_SimpleProcess(rx3data[0]);
        HAL_UART_Receive_IT(huart, rx3data, 1);
    
    }
    if(huart->Instance == USART1){
        // HAL_UART_Receive_IT(huart, &debugrx, 1);
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



// 模拟三个不同的数据集
void TestCopyBuffer(void) {
    // 数据集 1：XY 坐标系（有效数据部分）
    copy_buffer buffer1 = {
        .buffer = {0x03, 0x00, 0xFF, 0x00, 0xFF}, // 类别、X坐标、Y坐标
        .length = 5,
        .flag = 1
    };

    // 数据集 2：舵机数据（有效数据部分）
    copy_buffer buffer2 = {
        .buffer = {0x01,0x00,0x07,0x64,0x00,0x9C,0x02,0x01}, // 类别、ID、修改项、PWM频率、角度、方向
        .length = 9,
        .flag = 1
    };

    // 数据集 3：步进电机数据（有效数据部分）
    copy_buffer buffer3 = {
        .buffer = {0x02, 0x00, 0x18, 0x00, 0x78, 0x03, 0x14, 0x82}, // 类别、ID、修改项、速度、圈数
        .length = 8,
        .flag = 1
    };

    // 测试解析函数
    control_data data;

    printf("Testing buffer1 (XY Coordinates):\n");
    RxBufferParse(&data, &buffer1);
    PrintControlData_Type(&data);

    printf("\nTesting buffer2 (Servo Data):\n");
    RxBufferParse(&data, &buffer2);
    PrintControlData_Type(&data);

    printf("\nTesting buffer3 (Stepper Motor Data):\n");
    RxBufferParse(&data, &buffer3);
    PrintControlData_Type(&data);
}

/*
    通过蓝牙进行一些简单的控制，
    通过蓝牙写一些关于蓝牙的数据帧

    控制电机1和电机2的前进和后退，控制电机的速度，控制电机的圈数
    控制舵机的抓取

    
*/

#define FRAME_HEAD_BLUETOOTH 0xF0
#define FRAME_TAIL_BLUETOOTH 0x0F

uint8_t uBluetooth_Flag = 0;
uint8_t uBluetooth_len = 0;
uint8_t uBluetooth_checksum = 0;

uint8_t Uart3_RxBuffer[256];
uint8_t Uart3_buf_index = 0;

copy_buffer xbuffer_bluetooth = {0};
control_bluetooth_data user_bluetooth_data = {0};
/*   ---- 借用 上位机的数据格式 ---
帧格式：
    举例：
        帧头 | 数据长度 |     数据内容   | 校验码 | 帧尾
        0xF0 | 0x03     | 0x01 0x02 0x03 | 0x06   | 0x0F
舵机： 
    数据内容：类别(1),舵机id(1),修改项(1),抓取状态(1个字节)，上下状态(1字节)
    servo_act 设置 这个为 1为抓取 0为放开
    servo_updown 设置 这个为 1为上升 0为下降

   0xF0，0x5 ，CLASSIFY_SERVO_BLUETOOTH，CLASSIFY_ID_ONE,BLUETOOTH_OPTION_GRAB|BLUETOOTH_OPTION_UPDOWN,0X01,0X01，0x0F;  // 舵机的抓取，
电机：
数据内容：类别（1），步进电机id（1），修改项（1），速度(2个字节)， 方向（1），圈数(3个字节，前2字节存整数) 

    数据内容：类别(1),步进电机id(1),修改项(1),速度(2个字节)，方向（1），圈数(3个字节，前2字节存整数) 
   0xF0 CLASSIFY_STEPPER_BLUETOOTH，CLASSIFY_ID_ONE,BLUETOOTH_OPTION_SPEED|BLUETOOTH_OPTION_CIRCLE|BLUETOOTH_OPTION_DIRECTION,
    0X52,0x50,0X10,0X01 0x0F;  // 步进电机的速度、圈数、方向

    其中速度的范围是 52-5250
    圈数的范围是 0-126
    方向的范围是 0-1

enum classify_id {
    CLASSIFY_ID_ONE = 1,
    CLASSIFY_ID_TWO = 2,
    CLASSIFY_ID_THREE = 3,
    CLASSIFY_ID_FOUR = 4,
};
*/

enum option_bluetooth {
    BLUETOOTH_OPTION_GRAB       = (1 << 0),              // 位0
    BLUETOOTH_OPTION_UPDOWN     = (1 << 1),              // 位1
    BLUETOOTH_OPTION_DIRECTION  = (1 << 2),              // 位2
    BLUETOOTH_OPTION_SPEED      = (1 << 3),              // 位3
    BLUETOOTH_OPTION_CIRCLE     = (1 << 4),              // 位4
};

enum classify_bluetooth {
    CLASSIFY_SERVO_BLUETOOTH = 1,
    CLASSIFY_STEPPER_BLUETOOTH = 2,
};

enum FRAME_STATE_BLUETOOTH {
    FRAME_READING_HEAD_BLUETOOTH = 0,
    FRAME_READING_LENGTH_BLUETOOTH,
    FRAME_READING_DATA_BLUETOOTH,
    FRAME_READING_CHECKSUM_BLUETOOTH,
    FRAME_READING_TAIL_BLUETOOTH
};

void Bluetooth_Init(void)
{
    // 初始化蓝牙串口
    HAL_UART_MspInit(&BLUETOOTH_UART);
    // HAL_UART_Receive_IT(&BLUETOOTH_UART, rx3data, 1); // 启动接收中断
    HAL_UART_Receive_IT(&BLUETOOTH_UART, &rx3data[0], 1);
    HAL_Delay(1000);
}

void Bluetooth_SendData(uint8_t *data, uint16_t length)
{
    HAL_UART_Transmit(&BLUETOOTH_UART, data, length, 100);
}

static void PrintControlBluetoothData_Type(const control_bluetooth_data *data)
{
    // 检查指针是否为空
    if (data == NULL) {
        // printf("Error: data is NULL\n");
        return;
    }
    printf("Bluetooth Control Data:\n");
    printf("  Type: %u\n", data->type);
    printf("  ID: %u\n", data->id);
    printf("  Option: 0x%04X\n", data->option);
    switch (data->type) {
        case 1: // 舵机
            printf("  Servo Data:\n");
            printf("    Servo Action: %u\n", data->servo.servo_act);
            printf("    Servo Updown: %u\n", data->servo.servo_updown);
            break;

        case 2: // 步进电机
            printf("  Stepper Motor Data:\n");
            if (data->option & OPTION_SPEED) {
                printf("    Speed: %.2f\n", data->motor.motor_speed);
            }
            if (data->option & OPTION_CIRCLE) {
                printf("    Circle: %.2f\n", data->motor.motor_circle);
            }
            if (data->option & OPTION_DIRECTION) {
                printf("    Direction: %u\n", data->motor.motor_direction);
            }
            break;

        default:
            printf("  Unknown Type\n");
            break;
    }
}

void Bluetooth_Parsedata(control_bluetooth_data *data, const copy_buffer *buffer)
{
    // 检查标志位
    if (buffer->flag != 1) {
        // printf("Error: Buffer flag is not set.\n");
        return;
    }
    else
    printf("hello Bluetooth_Parsedata \n");
    
    // 清除标志位
    ((copy_buffer *)buffer)->flag = 0;
    // 初始化数据
    memset(data, 0, sizeof(control_bluetooth_data));

    // 检查数据长度是否足够
    if (buffer->length < 3) {
        printf("Error: Buffer length is too short.\n");
        return;
    }
    // 根据类别解析数据
    switch (buffer->buffer[0]) {
        case CLASSIFY_SERVO_BLUETOOTH: // 舵机数据解析
        {
            data->type = CLASSIFY_SERVO_BLUETOOTH;
            data->id = buffer->buffer[1]; // 读取舵机id
            data->option = buffer->buffer[2]; // 读取修改项

            uint8_t offset = 3; // 数据内容起始偏移量

            // 判断需要修改的选项
            if (data->option & BLUETOOTH_OPTION_GRAB) {
                if (offset >= buffer->length) break;
                data->servo.servo_act = buffer->buffer[offset]; // 读取抓取状态
                offset += 1; // 占用1字节
            }
            if (data->option & BLUETOOTH_OPTION_UPDOWN) {
                if (offset >= buffer->length) break;
                data->servo.servo_updown = buffer->buffer[offset]; // 读取抓取状态
                offset += 1; // 占用1字节
            }
            break;
        }

        case CLASSIFY_STEPPER_BLUETOOTH: // 步进电机数据解析
        {
            data->type = CLASSIFY_STEPPER_BLUETOOTH;
            data->id = buffer->buffer[1]; // 读取步进电机id
            data->option = buffer->buffer[2]; // 读取修改项

            uint8_t offset = 3; // 数据内容起始偏移量

            if (data->option & BLUETOOTH_OPTION_DIRECTION) {
                if (offset >= buffer->length) break;
                data->motor.motor_direction = buffer->buffer[offset]; // 读取方向
                data->motor.motor_direction = (data->motor.motor_direction == 0) ? 0 : 1; // 确保方向值为 0 或 1
                offset += 1; // 占用1字节
            }

            if (data->option & BLUETOOTH_OPTION_SPEED) {
                if (offset >= buffer->length) break;
                data->motor.motor_speed= (buffer->buffer[offset]<<8) | (buffer->buffer[offset + 1]); // 读取速度
                if(data->motor.motor_speed > 5250.0f) // 限制速度范围
                {
                    data->motor.motor_speed = 5250.0f;
                }
                else if(data->motor.motor_speed < 52.0f) 
                {
                    data->motor.motor_speed = 52.0f;
                }else
                {
                    printf("speed \n");
                }
                offset += 2; // 占用2字节
            }
            if (data->option & BLUETOOTH_OPTION_CIRCLE) {
                if (offset >= buffer->length) break;
                // 读取圈数（前两个字节为整数部分，第 3 个字节为小数部分）
                uint16_t circle_int = (buffer->buffer[offset] << 8) | buffer->buffer[offset + 1];
                uint8_t circle_decimal = buffer->buffer[offset + 2];
                data->motor.motor_circle= circle_int + (circle_decimal / 10.0f); // 合成浮点圈数
                if(data->motor.motor_circle > 126.0f) // 限制圈数范围
                {
                    data->motor.motor_circle = 126.0f;
                }
                else if(data->motor.motor_circle < 0.0f) 
                {
                    data->motor.motor_circle = 0.0f;
                }

                offset += 3; // 占用3字节
            }
            break;
        }
        default:
            printf("Error: Unknown data type.\n");
            return;
    }
    // 打印解析后的数据
    if (data->type != 0) {
        PrintControlBluetoothData_Type(data);
    }
}


static void Bluttooth_CallBack(uint8_t data) {
    static uint8_t state = FRAME_READING_HEAD_BLUETOOTH;
    static uint8_t checksum = 0;
    static uint8_t data_len = 0;
    switch (state) {
        case FRAME_READING_HEAD_BLUETOOTH: // 等待帧头
            if (data == FRAME_HEAD_BLUETOOTH) {
                state = FRAME_READING_LENGTH_BLUETOOTH;
            }
            break;

        case FRAME_READING_LENGTH_BLUETOOTH: // 读取数据长度
            data_len = data;
            if (data_len > sizeof(Uart3_RxBuffer)) {
                // 数据长度超出缓冲区大小，重置状态
                state = FRAME_READING_HEAD_BLUETOOTH;
                break;
            }
            Uart3_buf_index = 0;
            checksum = 0; // 重置校验码
            state = FRAME_READING_DATA_BLUETOOTH;
            break;

        case FRAME_READING_DATA_BLUETOOTH: // 读取数据内容
            if (Uart3_buf_index < data_len) {
                // 接收数据内容并计算校验码
                Uart3_RxBuffer[Uart3_buf_index] = data;
                checksum ^= data;
                Uart3_buf_index++;
            } else {
                // 数据接收完成，进入校验码状态
                state = FRAME_READING_TAIL_BLUETOOTH;
            }
            break;

        case FRAME_READING_TAIL_BLUETOOTH: // 读取帧尾
            if (data == FRAME_TAIL_BLUETOOTH) {
                // 校验数据
                if (checksum == data) {
                    // 数据校验通过，处理数据
                    __disable_irq(); // 禁用中断，防止并发访问
                    xbuffer_bluetooth.length = data_len;
                    memcpy(xbuffer_bluetooth.buffer, Uart3_RxBuffer, data_len);
                    xbuffer_bluetooth.flag = 1;
                    __enable_irq(); // 重新启用中断
                } else {
                    // 校验失败，丢弃数据
                    memset(Uart3_RxBuffer, 0, sizeof(Uart3_RxBuffer));
                }
            }
            // 重置状态机
            state = FRAME_READING_HEAD_BLUETOOTH;
            data_len = 0;
            Uart3_buf_index = 0;
            checksum = 0;
            break;
        default:
            // 未知状态，重置状态机
            state = FRAME_READING_HEAD_BLUETOOTH;
            data_len = 0;
            Uart3_buf_index = 0;
            checksum = 0;
            break;
    }
}

enum{
    BLUETOOTH_STOP = 0x44,
    BLUETOOTH_START = 0x46,
    BLUETOOTH_UP,
    BLUETOOTH_LEFT,
    BLUETOOTH_OK,
    BLUETOOTH_RIGHT,
    BLUETOOTH_DOWN,
};
/*
    为了快速实现毕设功能而写的一些程序

*/
void Bluetooth_SimpleProcess(const uint8_t data)
{
    switch (data)
    {
        case BLUETOOTH_STOP:
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            LED_Off(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN); 
            LED_Off(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
            
            break;
        case BLUETOOTH_START:
            break;
        case BLUETOOTH_UP: 
            LED_On(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_right);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_EN);
            break;
        case BLUETOOTH_LEFT: 
            LED_On(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN);
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
            break;
        case BLUETOOTH_OK: 
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTOREN_PIN,StepMotor_DISEN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_DISEN);
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            LED_Off(LED_GPIO_BULE_PORT, LED_GPIO_BULE_PIN); 
            LED_Off(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            key_deley(3000);
            servo_updown(SERVO_DOWN,FAST_MODE,NULL);
            LED_On(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            key_deley(3000);
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            servo_gripper(SERVO_GRIP,FAST_MODE,speed_one);
            key_deley(3000);
            LED_On(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            servo_gripper(SERVO_RELEASE,FAST_MODE,speed_one);
            key_deley(3000);
            LED_Off(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            servo_updown(SERVO_UP,FAST_MODE,NULL);
            break;
        case BLUETOOTH_RIGHT: 
            LED_On(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            LED_On(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_DIR_LEFT_PIN,Motor_right);
            TMC2209_Control(&MOTOR_LEFT_TIM,MOTOR_EN_LEFT_PIN,StepMotor_EN);
            break;
        case BLUETOOTH_DOWN:
            LED_On(LED_GPIO_READ_PORT, LED_GPIO_READ_PIN);
            LED_On(LED_GPIO_GREEN_PORT, LED_GPIO_GREEN_PIN);
            TMC2209_Control(&MOTOR_RIGHT_TIM,MOTORDIR_PIN,Motor_left);
            break;
        default:
            break;
    }
}
/*
\xa5\x04 \xff\x00 \x01\x00 \xfe \x5b
*/

int x_value = 0;
int y_value = 0;
uint8_t rx_flag = 0;
uint8_t rxopen_cnt=0;
volatile int rxopen_checksum =0;
volatile int _checksum =0;

void Rxopenmv_SimpleProcess(const uint8_t data)
{
    printf("Rxopenmv_SimpleProcess \n");
    printf("data = %x \n", data);
    switch (data)
    {
        case 0xA5:
            x_value = 0;
            y_value = 0;
            rx_flag = 0;
            rxopen_cnt=0;
        break;
        case 0x04:
            rx_flag = 0;
            break;
        case 0x5B:
            if(_checksum == rxopen_checksum)
            {
                rx_flag = 1;
                printf("x_value = %x, y_value = %x \n", x_value, y_value);
            }
            break;
        default:
            rxopen_cnt++;
            if(rxopen_cnt == 1)
                x_value = data;
            if(rxopen_cnt == 2)
                x_value = (x_value << 8) | data;
            if(rxopen_cnt == 3)
                y_value = data;
            if(rxopen_cnt == 4)
                y_value = (y_value << 8) | data;
            if(rxopen_cnt == 5)
            {
                _checksum = data;
                rxopen_checksum ^= x_value;
                rxopen_checksum ^= y_value;
            }
            rx_flag = 0;
        break;
    }
}

/*
    电机的转动
*/
float simple_xcircle = 0.0;
float simple_ycircle = 0.0;
uint8_t stepmotor_runflag = 0;

void stepmotor_SimpleProcess(void)
{
    if(rx_flag == 1)
    {
        rx_flag == 0;
        if (x_value < 0 || x_value > X_MAX || y_value < 0 || y_value > Y_MAX) {
            printf("Error: Coordinates out of range. X: %.2f, Y: %.2f\n", x_value, y_value);
            return;
        }
        simple_xcircle  =  (x_value / X_MAX) * N_INTER;
        simple_ycircle  =  (y_value / Y_MAX) * N_OUTER;

        tmc2209_simpleprocess(simple_xcircle,simple_ycircle);
        
        /*这里是启动定时器*/
        if(simple_xcircle !=0 && simple_ycircle !=0)
        {
            
        }else if(simple_xcircle !=0)
        {

        }
        else if(simple_ycircle != 0)
        {

        }

        printf("simple_xcircle: %d,simple_xcircle:%d\n",simple_xcircle,simple_ycircle);
        /*
            可以在这个地方去发送一个信号给openmv 
        */
        stepmotor_runflag = 1;
    }
}
