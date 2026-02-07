/**
 * @file OID_Encoder.c
 * @brief OID编码器驱动的源文件。
 * @author Xu ZheXuan
 * @date 2025-12-4
 */

#include "OID_Encoder.h"
#include "cmsis_os2.h"
#include "usart.h"

// 静态变量用于UART接收缓冲区
static uint8_t rx_buffer[32];
static int buffer_index = 0;
static int expected_length = 0;

// CRC校验函数 (来自手册)
uint16_t OID_Encoder_CRC(uint8_t *buf, uint8_t len)
{
    uint16_t wcrc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        wcrc ^= (uint16_t)buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (wcrc & 0x0001)
            {
                wcrc >>= 1;
                wcrc ^= 0xA001;
            }
            else
            {
                wcrc >>= 1;
            }
        }
    }
    return wcrc;
}

// 发送读取保持寄存器命令 (0x03)
void OID_Encoder_Send_Read_Command(UART_HandleTypeDef *huart, uint8_t address, uint16_t reg_addr, uint16_t reg_count)
{
    uint8_t tx_buf[8];
    tx_buf[0] = address;
    tx_buf[1] = OID_ENCODER_FUNCTION_READ_HOLDING_REGISTERS;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;
    uint16_t crc = OID_Encoder_CRC(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(huart, tx_buf, 8, HAL_MAX_DELAY);
}

// 初始化函数，发送读取多圈值的命令
void OID_Encoder_Init(UART_HandleTypeDef *huart, uint8_t address)
{
    // 读取虚拟多圈值 (0x0000-0x0001, 2个寄存器)
    OID_Encoder_Send_Read_Command(huart, address, OID_ENCODER_REG_MULTI_TURN_VALUE_LOW, 2);
}

// 初始化编码器地址为1
void OID_Encoder_Init_Address(UART_HandleTypeDef *huart)
{
    // 假设当前地址未知，使用默认地址1发送写命令设置地址为1
    OID_Encoder_Send_Write_Command(huart, OID_ENCODER_DEFAULT_ADDRESS, OID_ENCODER_REG_ADDRESS, 1);
}

// 初始化编码器数据结构体
void OID_Encoder_Init_Data(OID_Encoder_Data *data)
{
    data->address = OID_ENCODER_DEFAULT_ADDRESS;
    data->mode = OID_ENCODER_MODE_AUTO_SINGLE_TURN;
    data->angle_offset = 0.0f;
    data->PositionIncDeg = 0.0f;
    data->last_angle = 0.0f;
    data->flags.offset_initialized = false;
    data->flags.zero_init_flag = false;
    data->flags.mode_init_flag = true;
}

// 发送写单个寄存器命令 (0x06)
void OID_Encoder_Send_Write_Command(UART_HandleTypeDef *huart, uint8_t address, uint16_t reg_addr, uint16_t value)
{
    uint8_t tx_buf[8];
    tx_buf[0] = address;
    tx_buf[1] = OID_ENCODER_FUNCTION_WRITE_SINGLE_REGISTER;
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (value >> 8) & 0xFF;
    tx_buf[5] = value & 0xFF;
    uint16_t crc = OID_Encoder_CRC(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    HAL_UART_Transmit(huart, tx_buf, 8, HAL_MAX_DELAY);
}

// 设置编码器模式 (0x0006寄存器)
// mode: OID_ENCODER_MODE_QUERY, OID_ENCODER_MODE_AUTO_SINGLE_TURN, OID_ENCODER_MODE_AUTO_MULTI_TURN, OID_ENCODER_MODE_AUTO_ANGULAR_VELOCITY
void OID_Encoder_Set_Mode(UART_HandleTypeDef *huart, uint8_t address, uint8_t mode)
{
    if (mode > 5)
    {
        // 无效模式，忽略
        return;
    }
    OID_Encoder_Send_Write_Command(huart, address, OID_ENCODER_REG_MODE, mode);
}

// 更新函数，处理标志位并根据模式发送相应命令
void OID_Encoder_Update(UART_HandleTypeDef *huart, OID_Encoder_Data *data)
{
    // 处理重置零点标志位
    if (data->flags.zero_init_flag)
    {
        // 发送重置零点命令，写1到0x0008寄存器
        OID_Encoder_Send_Write_Command(huart, data->address, OID_ENCODER_REG_RESET_ZERO_POINT, 1);
    }

    // 处理mode初始化标志位
    if (data->flags.mode_init_flag)
    {
        OID_Encoder_Send_Write_Command(huart, data->address, OID_ENCODER_REG_MODE, data->mode);
    }

    data->PositionMeasure = data->multi_turn_angle;
}

// 假设buf包含完整的MODBUS响应帧，现在只处理多圈值
bool OID_Encoder_Parse_Read_Response(uint8_t *buf, uint16_t len, OID_Encoder_Data *data)
{
    uint8_t byte_count = buf[2];
    if (byte_count != 4)
        return false; // 多圈值4字节数据

    // 解析多圈值数据
    uint32_t value = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];
    data->raw.multi_turn_value = value;
    float current_angle = value * 360.0f / 1024.0f; // 假设分辨率1024
    data->multi_turn_angle = current_angle;

    // 处理累计角度
    if (!data->flags.offset_initialized)
    {
        // 第一次读取，设置偏置和初始值
        data->angle_offset = current_angle;
        data->PositionIncDeg = 0.0f;
        data->last_angle = current_angle;
        data->flags.offset_initialized = true;
    }
    else
    {
        // 计算角度变化，处理0-360跳变
        float delta = -(current_angle - data->last_angle);
        if (delta > 180.0f)
        {
            delta -= 360.0f;
        }
        else if (delta < -180.0f)
        {
            delta += 360.0f;
        }
        data->PositionIncDeg += delta;
        data->last_angle = current_angle;
    }

    return true;
}

// 解析写单个寄存器的响应
bool OID_Encoder_Parse_Write_Response(uint8_t *buf, uint16_t len, OID_Encoder_Data *data)
{
    if (len != 8)
        return false; // 写响应固定8字节
    // 解析寄存器地址
    uint16_t reg_addr = (buf[2] << 8) | buf[3];
    uint16_t value = (buf[4] << 8) | buf[5];

    // 根据reg_addr处理
    if (reg_addr == OID_ENCODER_REG_MODE)
    {
        data->flags.mode_init_flag = false;
    }
    else if (reg_addr == OID_ENCODER_REG_RESET_ZERO_POINT)
    {
        data->flags.zero_init_flag = false;
    }
    // 可以添加其他处理

    return true;
}

// 处理接收到的字节，存入队列并检查帧
void OID_Encoder_Process_Byte(uint8_t byte, OID_Encoder_Data *data)
{
    // 如果buffer_index >=32，清空buffer
    if (buffer_index >= 32)
    {
        buffer_index = 0;
        expected_length = 0;
        return;
    }

    // 存入buffer
    rx_buffer[buffer_index++] = byte;

    // 检查帧开始
    if (buffer_index == 1)
    {
        if (rx_buffer[0] != 0x01)
        {
            // 不是地址0x01，清空
            buffer_index = 0;
        }
        return;
    }

    // 检查功能码
    if (buffer_index == 2)
    {
        if (rx_buffer[1] == 0x03)
        {
            // 读保持寄存器，等待字节总数
            return;
        }
        else if (rx_buffer[1] == 0x06)
        {
            // 写单个寄存器，固定8字节
            expected_length = 8;
        }
        else
        {
            // 无效功能码，清空
            buffer_index = 0;
        }
        return;
    }

    // 对于0x03，检查字节总数
    if (buffer_index == 3 && rx_buffer[1] == 0x03)
    {
        uint8_t byte_count = rx_buffer[2];
        expected_length = 5 + byte_count; // ADR + FUNC + BYTE_COUNT + DATA + CRC
        if (expected_length > 32)
        {
            // 超出最大长度，清空
            buffer_index = 0;
            expected_length = 0;
        }
        return;
    }

    // 检查是否收到完整帧
    if (buffer_index == expected_length)
    {
        // 计算CRC（前expected_length-2字节）
        uint16_t crc = OID_Encoder_CRC(rx_buffer, expected_length - 2);
        uint16_t received_crc = (rx_buffer[expected_length - 1] << 8) | rx_buffer[expected_length - 2];
        if (crc == received_crc)
        {
            // CRC匹配，解析数据
            if (rx_buffer[1] == OID_ENCODER_FUNCTION_READ_HOLDING_REGISTERS)
            {
                OID_Encoder_Parse_Read_Response(rx_buffer, expected_length, data);
            }
            else if (rx_buffer[1] == OID_ENCODER_FUNCTION_WRITE_SINGLE_REGISTER)
            {
                OID_Encoder_Parse_Write_Response(rx_buffer, expected_length, data);
            }
        }
        // 清空buffer
        buffer_index = 0;
        expected_length = 0;
    }
}