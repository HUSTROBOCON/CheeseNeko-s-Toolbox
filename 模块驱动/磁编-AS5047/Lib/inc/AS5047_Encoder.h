/**
 * @file AS5047_Encoder.h
 * @brief AS5047 磁性编码器驱动的头文件。
 * @author Xu ZheXuan
 * @date 2025-12-4
 */

#ifndef AS5047_ENCODER_H
#define AS5047_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f405xx.h"

/**
 * @brief 片选 (CSN) 引脚和端口定义。
 * 如果未在其他地方定义，默认使用 GPIOA 的 GPIO_PIN_4。
 */
#ifndef CSN_GPIO_Port
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#endif

/**
 * @brief 磁钢定位的稳定角度值。
 */
#define AS5047_STABLE_ANGLE 215.0f

/**
 * @brief 稳定角度检测的容差范围。
 */
#define AS5047_TOLERANCE 1.0f

/**
 * @brief 用于保存 AS5047 编码器数据和配置的结构体。
 */
typedef struct {
    SPI_HandleTypeDef *hspi;    /**< 用于通信的 SPI 句柄指针。 */
    GPIO_TypeDef *csn_port;     /**< 片选引脚的 GPIO 端口。 */
    uint16_t csn_pin;           /**< 片选的 GPIO 引脚号。 */
    float angle;                /**< 以度为单位的编码器角度数据。 */
} AS5047_Encoder_Data;

/**
 * @brief 初始化 AS5047 编码器实例。
 * @param handle 指向 AS5047_Encoder_Data 结构体的指针。
 * @param hspi 指向 SPI 句柄的指针。
 * @param csn_port CSN 引脚的 GPIO 端口。
 * @param csn_pin CSN 的 GPIO 引脚。
 */
void AS5047_Init(AS5047_Encoder_Data *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csn_port, uint16_t csn_pin);

/**
 * @brief 从指定地址读取 AS5047 编码器的角度数据。
 * @param handle 指向 AS5047_Encoder_Data 结构体的指针。
 * @param add 要读取的寄存器地址。
 * @return 以度为单位的角度值。
 */
float AS5047_Read(AS5047_Encoder_Data *handle, uint16_t add);

/**
 * @brief 通过从编码器读取来更新角度数据。
 * @param handle 指向 AS5047_Encoder_Data 结构体的指针。
 */
void AS5047_Angle_Update(AS5047_Encoder_Data *handle);

#endif /* AS5047_ENCODER_H */
