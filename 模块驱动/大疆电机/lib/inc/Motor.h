#ifndef MOTOR_H
#define MOTOR_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "math.h"

#define MOTOR_STABLE_THRESHOLD  1.0f

typedef enum
{
	MOTOR_IDLE,
	MOTOR_POSITION,
	MOTOR_SPEED,
	MOTOR_CURRENT,
	MOTOR_ERROR,
}Motor_mode;

typedef struct
{
	uint8_t enable;
	Motor_mode mode;
	
	short encoder;					
	short encoderPre;				
	float PrePosition;			
	
	float SpeedTorlent;
	float StartPosition;
	
	//记录峰值
	float SpeedMax;					
	float CurrentMAX;	
	
	//电机运行状态
	uint32_t ID;	
	int16_t DIR;	
	
	float PositionExpected;
	float PositionMeasure;
	float SpeedExpected;
	float SpeedMeasure;
	float CurrentExpected;
	float CurrentMeasure;
	
} MotorTypeDef;



// 检查位置误差是否在限制范围内
static inline bool Check_Motor_Position(MotorTypeDef* motor, float maxError)
{
    return fabsf(motor->PositionExpected - motor->PositionMeasure) < maxError;
}

// 检查速度误差是否在限制范围内
static inline bool Check_Motor_Speed(MotorTypeDef* motor, float maxError)
{
    return fabsf(motor->SpeedExpected - motor->SpeedMeasure) < maxError;
}

uint8_t motor_ready(MotorTypeDef *motor, uint8_t *cnt, uint8_t boundary);


#endif
