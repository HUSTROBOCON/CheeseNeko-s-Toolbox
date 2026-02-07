/**
 * @file AS5047_Encoder.c
 * @brief AS5047 磁性编码器驱动的源文件。
 * @author Xu ZheXuan
 * @date 2025-12-4
 */

#include "AS5047_Encoder.h"


static uint8_t Parity_bit_Calculate(uint16_t data) {
    uint8_t parity = 0;
    uint16_t temp = data;
    while (temp) {
        parity ^= temp & 1;
        temp >>= 1;
    }
    return parity;
}

static uint16_t SPI_ReadWrite_OneByte(AS5047_Encoder_Data *handle, uint16_t txdata) {
    uint16_t rxdata = 0;
    HAL_GPIO_WritePin(handle->csn_port, handle->csn_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(handle->hspi, (uint8_t*)&txdata, (uint8_t*)&rxdata, 1, 1000);
    HAL_GPIO_WritePin(handle->csn_port, handle->csn_pin, GPIO_PIN_SET);
    return rxdata;
}

void AS5047_Init(AS5047_Encoder_Data *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    handle->hspi = hspi;
    handle->csn_port = csn_port;
    handle->csn_pin = csn_pin;
    handle->angle = 0.0f;
}

float AS5047_Read(AS5047_Encoder_Data *handle, uint16_t add) {
    uint16_t data;
    add |= 0x4000;  // Set read bit
    if (Parity_bit_Calculate(add) == 1) add |= 0x8000;  // Set parity
    do {
        data = SPI_ReadWrite_OneByte(handle, add);
    } while (((data & 0x4000) == 1) || ((data & 0x3FFF) == 0));
    handle->angle = (0x3FFF - (data & 0x3FFF)) * 360.0f / (float)0x3FFF;
    return handle->angle;
}

void AS5047_Angle_Update(AS5047_Encoder_Data *handle) {
    handle->angle = AS5047_Read(handle, 0x3FFF);
}