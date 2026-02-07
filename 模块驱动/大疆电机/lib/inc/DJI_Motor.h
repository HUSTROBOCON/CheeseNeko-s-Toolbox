#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "stm32h7xx_hal.h"
#include "Motor.h"
#include "PID.h"

typedef struct {
	PIDStructTypedef SpeedPID;
	PIDStructTypedef PosPID;
	PIDStructTypedef ReservePID;
} DJIMotor_PID_struct;

typedef enum {
    DJI_MOTOR_TYPE_3508,
    DJI_MOTOR_TYPE_2006
} DJIMotorType;

typedef struct {
    MotorTypeDef Motor;
    DJIMotor_PID_struct PID;
    DJIMotorType Type;
} DJIMotorTypeDef;

// New Definitions
#define PLATFORM_3508_ID  1
#define FOREARM_2006_ID   2

// void DJIMotor_Control_Update(MotorTypeDef* Motor, DJIMotor_PID_struct* MotorPID);
void DJIMotor_Control_Update(DJIMotorTypeDef* DJIMotor);
void DJI_Motor_Init(DJIMotorTypeDef* dji_motor, DJIMotorType type, uint32_t can_id);

void dji_can_transmit_eid(uint16_t id, uint8_t bus_id, uint8_t *data, uint8_t len);
void dji_can_set(uint16_t, uint8_t,int16_t, int16_t, int16_t, int16_t);
void dji_can_set_can2(uint16_t controller_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void Process_DJI_Frame(FDCANFrame *Frame_Process, DJIMotorTypeDef* DJIMotor);
uint8_t motor_ready(MotorTypeDef *motor,uint8_t *count,uint8_t boundary);


#endif
