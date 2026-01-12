/**
 * @file OID_Encoder.h
 * @brief OID编码器驱动的头文件。
 * @author Xu ZheXuan
 * @date 2025-12-4
 */

#ifndef OID_ENCODER_H
#define OID_ENCODER_H
#include "CAN_Basic.h"
#include <stdint.h>
#include <stdbool.h>

// MODBUS-RTU 功能码
#define OID_ENCODER_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define OID_ENCODER_FUNCTION_WRITE_SINGLE_REGISTER 0x06

// 编码器模式定义
#define OID_ENCODER_MODE_QUERY                  0x00  // 查询模式
#define OID_ENCODER_MODE_AUTO_SINGLE_TURN       0x01  // 自动回传编码器单圈值
#define OID_ENCODER_MODE_AUTO_MULTI_TURN        0x04  // 自动回传编码器虚拟多圈值
#define OID_ENCODER_MODE_AUTO_ANGULAR_VELOCITY 0x05  // 自动回传编码器角速度值

// 寄存器地址定义
#define OID_ENCODER_REG_SINGLE_TURN_VALUE       0x0000
#define OID_ENCODER_REG_MULTI_TURN_VALUE_LOW    0x0000
#define OID_ENCODER_REG_MULTI_TURN_VALUE_HIGH   0x0001
#define OID_ENCODER_REG_ANGULAR_VELOCITY        0x0003
#define OID_ENCODER_REG_ADDRESS                 0x0004
#define OID_ENCODER_REG_BAUDRATE                0x0005
#define OID_ENCODER_REG_MODE                    0x0006
#define OID_ENCODER_REG_AUTO_TRANSMIT_TIME      0x0007
#define OID_ENCODER_REG_RESET_ZERO_POINT        0x0008
#define OID_ENCODER_REG_INCREMENT_DIRECTION     0x0009
#define OID_ENCODER_REG_ANGULAR_VELOCITY_SAMPLE_TIME 0x000A
#define OID_ENCODER_REG_SET_CURRENT_VALUE       0x000B
#define OID_ENCODER_REG_SET_MIDPOINT            0x000E
#define OID_ENCODER_REG_ANGULAR_VELOCITY2_LOW   0x0020
#define OID_ENCODER_REG_ANGULAR_VELOCITY2_HIGH  0x0021
#define OID_ENCODER_REG_SINGLE_TURN_VALUE2_LOW  0x0025
#define OID_ENCODER_REG_SINGLE_TURN_VALUE2_HIGH 0x0026

// 默认参数
#define OID_ENCODER_DEFAULT_ADDRESS 0x01
#define OID_ENCODER_DEFAULT_BAUDRATE 0x0000  // 9600

// 子结构体定义
typedef struct {
    uint16_t single_turn_value;       // 单圈值 (0x0000)
    uint32_t multi_turn_value;        // 虚拟多圈值 (0x0000-0x0001)
    int16_t angular_velocity;         // 角速度值 (0x0003)
    uint16_t baudrate;                // 波特率 (0x0005)
    uint16_t auto_transmit_time;      // 自动回传时间 (0x0007)
    uint8_t increment_direction;      // 递增方向 (0x0009)
    uint16_t angular_velocity_sample_time; // 角速度采样时间 (0x000A)
} OID_Encoder_RawData;

typedef struct {
    bool offset_initialized;              // 是否已初始化偏置
    bool zero_init_flag;             // 重置零点标志位
    bool mode_init_flag;            // mode初始化标志位
} OID_Encoder_Flags;

// 编码器数据结构体
typedef struct {
    OID_Encoder_RawData raw;          // 原始数据
    // 提出字段
    uint8_t address;                  // 地址 (0x0004)
    uint8_t mode;                     // 模式 (0x0006)
    // 计算结果
    float single_turn_angle;          // 当前单圈角度 (度)
    float multi_turn_angle;           // 当前多圈角度 (度)
    float angular_velocity_rpm;       // 角速度 (转/分钟)
    float angle_offset;               // 角度偏置值 (第一次读取的multi_turn_angle)
    float last_angle;                 // 上一次的角度值，用于计算变化
	
	float PositionMeasure;
	float PositionExpected;
    float PositionIncDeg;             // 累计角度值
	float PositionTarget;	// 目标角度

    OID_Encoder_Flags flags;          // 标志位
} OID_Encoder_Data;

// 函数声明
uint16_t OID_Encoder_CRC(uint8_t *buf, uint8_t len);
void OID_Encoder_Init(UART_HandleTypeDef *huart, uint8_t address);
void OID_Encoder_Init_Address(UART_HandleTypeDef *huart);
void OID_Encoder_Init_Data(OID_Encoder_Data *data);
void OID_Encoder_Send_Read_Command(UART_HandleTypeDef *huart, uint8_t address, uint16_t reg_addr, uint16_t reg_count);
bool OID_Encoder_Parse_Read_Response(uint8_t *buf, uint16_t len, OID_Encoder_Data *data);
bool OID_Encoder_Parse_Write_Response(uint8_t *buf, uint16_t len, OID_Encoder_Data *data);


void OID_Encoder_Send_Write_Command(UART_HandleTypeDef *huart, uint8_t address, uint16_t reg_addr, uint16_t value);
void OID_Encoder_Set_Mode(UART_HandleTypeDef *huart, uint8_t address, uint8_t mode);
void OID_Encoder_Update(UART_HandleTypeDef *huart, OID_Encoder_Data *data);

void OID_Encoder_Process_Byte(uint8_t byte, OID_Encoder_Data *data);

#endif // OID_ENCODER_H
