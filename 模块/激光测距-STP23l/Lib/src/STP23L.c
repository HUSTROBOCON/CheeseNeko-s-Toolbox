/**
 * @file STP23L.c
 * @author xu zhe xuan
 * @brief STP-23L LiDAR Driver Implementation
 */

#include "STP23L.h"
#include <string.h>

// Command Flags (Values inferred or placeholders, adjust if needed)
#define PACK_GET_DISTANCE 0x02
#define PACK_RESET_SYSTEM 0x0D // Example
#define PACK_STOP 0x0F // Example
#define PACK_ACK 0x10 // Example
#define PACK_VERSION 0x01 // Example

void STP23L_Init(STP23L_Instance *instance) {
    memset(instance, 0, sizeof(STP23L_Instance));
}

void STP23L_Process_Byte(uint8_t byte, STP23L_Instance *instance) {
    if (instance->state < 4) {
        if (byte == STP23L_HEADER) {
            instance->state++;
        } else {
            instance->state = 0;
        }
    } else if (instance->state < 10) {
        switch (instance->state) {
            case 4: // Device Address
                instance->state++;
                instance->checksum = byte; // Start checksum calculation
                break;
            case 5: // Command Flag
                instance->pack_flag = byte;
                instance->state++;
                instance->checksum += byte;
                break;
            case 6: // Chunk Offset Low
                instance->state++;
                instance->checksum += byte;
                break;
            case 7: // Chunk Offset High
                instance->state++;
                instance->checksum += byte;
                break;
            case 8: // Data Length Low
                instance->data_len = byte;
                instance->state++;
                instance->checksum += byte;
                break;
            case 9: // Data Length High
                instance->data_len |= (uint16_t)byte << 8;
                instance->state++;
                instance->checksum += byte;
                break;
        }
    } else if (instance->state < 190) {
        // Data Parsing
        uint8_t state_num = (instance->state - 10) % 15;
        // Calculate current point index
        uint8_t point_idx = (instance->state - 10) / 15;
        
        if (point_idx < STP23L_POINT_COUNT) {
            STP23L_Point *point = &instance->temp_points[point_idx];

            switch (state_num) {
                case 0: point->distance = byte; break;
                case 1: point->distance |= (uint16_t)byte << 8; break;
                case 2: point->noise = byte; break;
                case 3: point->noise |= (uint16_t)byte << 8; break;
                case 4: point->peak = byte; break;
                case 5: point->peak |= (uint32_t)byte << 8; break;
                case 6: point->peak |= (uint32_t)byte << 16; break;
                case 7: point->peak |= (uint32_t)byte << 24; break;
                case 8: point->confidence = byte; break;
                case 9: point->intg = byte; break;
                case 10: point->intg |= (uint32_t)byte << 8; break;
                case 11: point->intg |= (uint32_t)byte << 16; break;
                case 12: point->intg |= (uint32_t)byte << 24; break;
                case 13: point->reftof = byte; break;
                case 14: 
                    point->reftof |= (int16_t)byte << 8; 
                    break;
            }
        }
        instance->checksum += byte;
        instance->state++;
    } else {
        // Timestamp and Checksum
        switch (instance->state) {
            case 190: instance->temp_timestamp = byte; instance->checksum += byte; instance->state++; break;
            case 191: instance->temp_timestamp |= (uint32_t)byte << 8; instance->checksum += byte; instance->state++; break;
            case 192: instance->temp_timestamp |= (uint32_t)byte << 16; instance->checksum += byte; instance->state++; break;
            case 193: instance->temp_timestamp |= (uint32_t)byte << 24; instance->checksum += byte; instance->state++; break;
            case 194: // Checksum
                if (byte == instance->checksum) {
                    // Valid Frame
                    memcpy(instance->data.raw_data, instance->temp_points, sizeof(instance->temp_points));
                    instance->data.timestamp = instance->temp_timestamp;
                    STP23L_Update(instance); // Process data immediately
                }
                // Reset
                instance->state = 0;
                instance->checksum = 0;
                break;
        }
    }
}

void STP23L_Update(STP23L_Instance *instance) {
    uint32_t sum_distance = 0;
    uint16_t count = 0;

    for (int i = 0; i < STP23L_POINT_COUNT; i++) {
        if (instance->data.raw_data[i].distance != 0) {
            sum_distance += instance->data.raw_data[i].distance;
            count++;
        }
    }

    if (count > 0) {
        instance->data.distance = (float)sum_distance / count;
    } else {
        instance->data.distance = 0.0f;
    }
}

uint8_t STP23L_IsInRange(STP23L_Instance *instance, float target, float error) {
    float dist = instance->data.distance;
    if (dist >= (target - error) && dist <= (target + error)) {
        return 1;
    }
    return 0;
}

uint8_t STP23L_IsOverThreshold(STP23L_Instance *instance, float threshold) {
    if (instance->data.distance > threshold) {
        return 1;
    }
    return 0;
}