/**
 * @file Unitree_Motor.h
 * @brief Unitree (宇树) Go1/A1 电机驱动 通讯协议&数据包
 * @version 1.3
 * @date 2026-01-26
 *
 * @copyright Copyright (c) unitree robotics .co.ltd.
 */
 
#ifndef _UNITREE_MOTOR_H
#define _UNITREE_MOTOR_H

/**
 * @brief 宇树电机核心控制公式:
 * T_out = K_P * (Pos_des - Pos_cur) + K_W * (W_des - W_cur) + T_ff
 * 
 * 1. 位置模式: K_P > 0, K_W > 0 (建议 KP约为 0.1, KW约为 2.0)
 * 2. 速度模式: K_P = 0, K_W > 0, W_des = 目标速度
 * 3. 阻尼模式: K_P = 0, K_W > 0, W_des = 0
 * 4. 力矩模式: K_P = 0, K_W = 0, T_ff = 目标力矩
 */

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include "PID.h"

/* ================== 协议常量定义 ================== */
// 协议数据限制 (数据类型量程，适用于 Go1/A1 等采用此协议的电机)
#define UNITREE_MOTOR_ID_MAX         15
#define UNITREE_MOTOR_MODE_MAX       7

// 协议转换系数 (GO1 / M8010)
#define UNITREE_GO1_KP_SCALE       1280.0f  // 32768 / 25.6
#define UNITREE_GO1_KW_SCALE       1280.0f  // 32768 / 25.6
#define UNITREE_GO1_POS_SCALE      5215.18917f // 32768 / 6.28318...
#define UNITREE_GO1_SPD_SCALE      40.743665f  // 256 / 6.28318...
#define UNITREE_GO1_TORQUE_SCALE   256.0f

// 用户安全限制 (GO1)
#define UNITREE_GO1_LIMIT_TORQUE     5.7f     // 调试限制 (Peak: 23.7f)
#define UNITREE_GO1_LIMIT_SPEED      15.0f     // 调试限制 (Peak: 30.0f)

// 协议转换系数 (A1 / 6010)
#define UNITREE_A1_KP_SCALE          2048.0f
#define UNITREE_A1_KW_SCALE          1024.0f
#define UNITREE_A1_POS_SCALE         2607.59f    // 16384 / 6.28318...
#define UNITREE_A1_SPD_SCALE         128.0f
#define UNITREE_A1_TORQUE_SCALE      256.0f

// 用户安全限制 (A1)
#define UNITREE_A1_LIMIT_TORQUE      5.5f     // 调试限制 (Peak: 33.5f)
#define UNITREE_A1_LIMIT_SPEED       15.0f     // 调试限制 (Peak: 21.0f)

// 电机型号枚举
typedef enum {
    UNITREE_MOTOR_TYPE_Unknown = 0,
    UNITREE_MOTOR_TYPE_GO1, // M8010-6
    UNITREE_MOTOR_TYPE_A1,  // A1 Joint Motor (6010?)
} UnitreeMotorType;

/**
 * @brief 宇树电机软件控制模式
 */
typedef enum {
    UNITREE_CTRL_STOP = 0,        ///< 停止控制: 发送模式0
    UNITREE_CTRL_NATIVE,      ///< 默认模式: 使用电机自带 PD (K_P, K_W)
    UNITREE_CTRL_POS,       ///< 软件位置环: 软件 Pos PID -> 目标速度 W (此时电机 K_P=0, K_W>0)
    UNITREE_CTRL_POS_SPD    ///< 软件串级环: 软件 Pos PID -> 软件 Spd PID -> 目标力矩 T (此时电机 K_P=0, K_W=0)
} UnitreeControlMode;

/**
 * @brief 宇树电机 PID 结构体 (仿 DJI 电机设计)
 */
typedef struct {
    PIDStructTypedef SpeedPID;
    PIDStructTypedef PosPID;
} UnitreeMotor_PID_struct;

/**
 * @brief 宇树电机协议转换系数与物理限制
 */
typedef struct {
    float kp_scale;
    float kw_scale;
    float pos_scale;
    float spd_scale;
    float tor_scale;
    
    float tor_limit; // 协议最大力矩
    float spd_limit; // 协议最大速度

    uint8_t tx_msg_len;             // 发送帧长度
    uint8_t rx_msg_len;             // 接收帧长度
    uint8_t rx_head[2];             // 期望包头
} UnitreeParamScale_t;

// 协议缓冲区最大长度
#define UNITREE_PACKET_MAX_LEN       128
#define UNITREE_RX_BUF_SIZE          128

#pragma pack(1)

/* ================== GO1 / M8010 协议 ================== */

/**
 * @brief GO1/M8010 发送数据包格式 (17 Bytes)
 */
typedef struct
{
    uint8_t head[2];       // 包头 0xFE 0xEE
    
    // Mode (1 Byte)
    uint8_t id : 4;      // 电机ID: 0-14, 15为广播
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准
    uint8_t reserve : 1; // 保留位

    // Comd (12 Bytes)
    int16_t T;       // 期望关节输出扭矩 (q8)
    int16_t W;       // 期望关节输出速度 (q8)
    int32_t Pos;     // 期望关节输出位置 (q15)
    int16_t K_P;     // 期望关节刚度系数 (q15)
    int16_t K_W;     // 期望关节阻尼系数 (q15)
    
    uint16_t CRC16;  // CRC校验
} UnitreeGo_ControlData_t;

/**
 * @brief GO1/M8010 接收数据包格式 (16 Bytes)
 */
typedef struct
{
    uint8_t head[2];       // 包头 0xFD 0xEE
    
    // Mode (1 Byte)
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t reserve : 1;

    // Fbk (11 Bytes)
    int16_t torque;      // 实际关节输出扭矩 (q8)
    int16_t speed;       // 实际关节输出速度 (q8)
    int32_t pos;         // 实际关节输出位置 (q15)
    int8_t temp;         // 电机温度
    uint8_t MError : 3;  // 电机错误标识
    uint16_t force : 12; // 足端气压传感器数据
    uint8_t none : 1;    // 保留位

    uint16_t CRC16;  // CRC校验
} UnitreeGo_MotorData_t;

/* ================== A1 / 6010 协议 ================== */

/**
 * @brief A1 电机发送数据包格式 (34 Bytes)
 */
typedef struct
{
    uint8_t head[2];   // 包头 0xFE 0xEE
    uint8_t id;        // 电机ID
    uint8_t reserved1; // 预留位 (0x00)
    uint8_t status;    // 运行模式
    uint8_t ModifyBit; // 控制参数修改位
    uint8_t ReadBit;   // 参数发送位 (0x00)
    uint8_t reserved2; // 预留位 (0x00)
    
    // Modify (4Bytes) - 电机参数修改数据 (Res)
    uint8_t Modify[4]; 

    int16_t T;         // 前馈力矩 (Need * 256)
    int16_t W;         // 期望速度 (Need * 128)
    int32_t Pos;       // 期望位置 (Need * 16384 / 2PI)
    int16_t K_P;       // 位置刚度 (Need * 2048)
    int16_t K_W;       // 速度刚度 (Need * 1024)
    
    uint8_t LowHzIndex;// 低频指令索引
    uint8_t LowHzByte; // 低频指令数据
    uint8_t Res[4];    // 预留位
    
    uint32_t CRC32;    // CRC32 校验
} UnitreeA1_ControlData_t;

/**
 * @brief A1 电机接收数据包格式 (78 Bytes)
 */
typedef struct
{
    uint8_t head[2];        // 包头 0xFE 0xEE (0-1)
    
    uint8_t id;             // 电机ID (2)
    uint8_t reserved;       // 预留位 (3)
    uint8_t status;         // 运行状态 (4)
    uint8_t ReadBit;        // 参数修改标志 (5)
    int8_t  Temp;           // 温度 (6)
    uint8_t MError;         // 错误状态 (7)
    uint8_t Read[4];        // 读取内部参数 (8-11)
    
    int16_t T;              // 输出力矩 (12-13) (x256)
    int16_t W;              // 实际转速 (14-15) (x128)
    int32_t LW;             // 滤波转速 (16-19) (不推荐)
    
    int16_t W2;             // 预留 (20-21)
    int32_t LW2;            // 预留 (22-25)
    
    int16_t Acc;            // 转动加速度 (26-27) (x1)
    int16_t OutAcc;         // 预留 (28-29)
    
    int32_t Pos;            // 电机角度位置 (30-33) (x 16384/2PI)
    int32_t Pos2;           // 预留 (34-37)
    
    int16_t gyro[3];        // IMU角速度 (38-43)
    int16_t acc[3];         // IMU加速度 (44-49)
    
    // Fgyro[0-2], Facc[0-2] (50-61)
    int16_t Fgyro[3];       
    int16_t Facc[3];
    
    int16_t Fmag[3];        // 磁力计 (62-67)
    uint8_t Ftemp;          // 足端温度 (68)
    int16_t Force16;        // 足端压力16位 (69-70)
    uint8_t Force8;         // 足端压力8位 (71)
    uint8_t FError;         // 足端错误 (72)
    uint8_t Res;            // 预留 (73)
    
    uint32_t CRC32;         // CRC32 (74-77)
} UnitreeA1_MotorData_t;

#pragma pack()

/**
 * @brief 宇树电机状态标志位
 */
typedef struct
{
    uint8_t connected;      // 连接状态标识
    uint8_t one_shot_flag;  // 单次发送标志位 (0:普通连续发送, 1:即将单次发送, 2:已单次发送完毕/停止发送)
    uint8_t fbk_ready_flag; // 完整反馈包就绪标志 (1:可解包, 0:处理中/无)
    uint8_t safe_switch;    // 控制模式切换安全保护标志
} UnitreeMotorFlag_t;

/**
 * @brief 宇树电机对象结构体
 * @note  合并了控制与反馈数据，减少冗余，并绑定硬件HAL句柄
 */
typedef struct
{
    // ============ 1. 硬件接口与基本配置 ============
    UART_HandleTypeDef *huart;      // 绑定的串口句柄
    uint8_t id;                     // 电机ID
    UnitreeMotorType type;          // 电机型号
    UnitreeControlMode ctrl_mode;   // 软件控制模式 (环路配置)
    int8_t mode;                    // 底层控制状态 (0:空闲 1:FOC控制 2:标定)

    // ============ 2. 状态标志位 ============
    UnitreeMotorFlag_t Flag;

    // ============ 3. 协议参数系数与限制 ============
    UnitreeParamScale_t param_scale;

    // ============ 4. 控制参数 (用户写入) ============
    float T;                        // 期望前馈力矩 (Nm)
    float W;                        // 期望速度 (rad/s)
    float Pos;                      // 期望位置 (rad)
    float K_P;                      // 刚度系数 (0-25.6)
    float K_W;                      // 阻尼系数 (0-25.6)

    // ============ 5. 反馈参数 (电机返回) ============
    float rx_T;                     // 实际力矩 (Nm)
    float rx_W;                     // 实际速度 (rad/s)
    float rx_Pos;                   // 实际位置 (rad)
    int8_t rx_Temp;                 // 温度 (℃)
    uint8_t rx_MError;              // 错误码
    int16_t rx_footForce;           // 足端力

    // ============ 6. 软件环路 PID (串级环) ============
    UnitreeMotor_PID_struct PID;

    // ============ 7. 内部通讯缓冲区 (协议底层) ============
    uint8_t cmd_buffer[UNITREE_PACKET_MAX_LEN] __attribute__((aligned(32)));  // 发送数据缓冲区
    uint8_t fbk_buffer[UNITREE_PACKET_MAX_LEN] __attribute__((aligned(32)));  // 解析用数据缓冲区 (存放完整的一帧)
    uint8_t rx_stream_buffer[UNITREE_PACKET_MAX_LEN];                         // 状态机流缓冲区 (接收过程)
    
    uint8_t fbk_rx_index;           // 反馈接收索引

    // ============ 8. 统计与诊断计数 ============
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t crc_err_count;

} UnitreeMotor;

// 接口函数
void UnitreeMotor_Init(UnitreeMotor *motor, UnitreeMotorType type, uint8_t id);
void UnitreeMotor_Transmit(UnitreeMotor *motor, UART_HandleTypeDef *huart);
void UnitreeMotor_Control_Update(UnitreeMotor *motor);
void UnitreeMotor_UnpackFbk(UnitreeMotor *motor);
void Process_Unitree_Motor_Byte(UnitreeMotor *motor, uint8_t byte);

#endif

