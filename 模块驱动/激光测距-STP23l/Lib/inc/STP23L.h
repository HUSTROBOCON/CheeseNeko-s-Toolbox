/**
 * @file STP23L.h
 * @author xu zhe xuan
 * @brief STP-23L LiDAR Driver Header
 *
 * Protocol Description (195 Bytes Total):
 * 1. Header (4 Bytes): 0xAA 0xAA 0xAA 0xAA
 * 2. Control (6 Bytes):
 *    - Device Address (1 Byte)
 *    - Command Flag (1 Byte)
 *    - Chunk Offset (2 Bytes, Low-High)
 *    - Data Length (2 Bytes, Low-High)
 * 3. Payload (180 Bytes): 12 Points * 15 Bytes/Point
 *    Each Point Structure (15 Bytes):
 *    - Distance (2 Bytes, Low-High) [mm]
 *    - Noise (2 Bytes, Low-High)
 *    - Peak (4 Bytes, Low-High)
 *    - Confidence (1 Byte)
 *    - Integration (4 Bytes, Low-High)
 *    - RefToF (2 Bytes, Low-High) [Temperature representation]
 * 4. Timestamp (4 Bytes, Low-High)
 * 5. Checksum (1 Byte): Sum of all bytes from Device Address to Timestamp (inclusive).
 *
 * 协议说明 (共195字节):
 * 1. 帧头 (4字节): 0xAA 0xAA 0xAA 0xAA
 * 2. 控制信息 (6字节):
 *    - 设备地址 (1字节)
 *    - 命令标志 (1字节)
 *    - 块偏移 (2字节, 低位在前)
 *    - 数据长度 (2字节, 低位在前)
 * 3. 数据负载 (180字节): 12个点 * 15字节/点
 *    每个点结构 (15字节):
 *    - 距离 (2字节, 低位在前) [mm]
 *    - 噪声 (2字节, 低位在前)
 *    - 峰值 (4字节, 低位在前)
 *    - 置信度 (1字节)
 *    - 积分次数 (4字节, 低位在前)
 *    - RefToF (2字节, 低位在前) [温度表征]
 * 4. 时间戳 (4字节, 低位在前)
 * 5. 校验和 (1字节): 从设备地址到时间戳所有字节的累加和。
 */

#ifndef STP23L_H
#define STP23L_H

#include "stm32f4xx_hal.h"

#define STP23L_HEADER 0xAA
#define STP23L_POINT_COUNT 12

#define STP23L_STEP_DOWN_THRESHOLD 200.0f      // 下台阶阈值 (mm)
#define STP23L_APPROACH_STEP_DISTANCE 500.0f   // 接近台阶距离 (mm)
#define STP23L_DEFAULT_ERRORS 5.0f       // 默认误差 (mm)

typedef struct {
    uint16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
} STP23L_Point;

typedef struct {
    STP23L_Point raw_data[STP23L_POINT_COUNT];
    float distance; // 毫米单位
    uint32_t timestamp;
} STP23L_Data;

typedef struct {
    STP23L_Data data;
    // Internal parsing state
    uint8_t state;
    uint8_t checksum; // Changed from crc to checksum
    uint8_t pack_flag;
    uint16_t data_len;
    uint32_t temp_timestamp;
    STP23L_Point temp_points[STP23L_POINT_COUNT];
} STP23L_Instance;

void STP23L_Init(STP23L_Instance *instance);
void STP23L_Process_Byte(uint8_t byte, STP23L_Instance *instance);
void STP23L_Update(STP23L_Instance *instance);
uint8_t STP23L_IsInRange(STP23L_Instance *instance, float target, float error);
uint8_t STP23L_IsOverThreshold(STP23L_Instance *instance, float threshold);


extern STP23L_Instance stp23l_A; // STP23L实例
extern STP23L_Instance stp23l_B; // STP23L实例
#endif // STP23L_H
