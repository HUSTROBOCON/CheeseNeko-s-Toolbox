#include "Motor.h"

uint8_t motor_ready(MotorTypeDef *motor, uint8_t *cnt, uint8_t boundary)
{
	uint8_t temp = *cnt;

	// 判断位置是否稳定
	if (fabsf(motor->PositionMeasure - motor->PrePosition) < MOTOR_STABLE_THRESHOLD)
	{
		if (temp < 255)  // 防止溢出
				temp++;
	}
	else
	{
		temp = (temp < 2) ? 0 : (temp >> 1);  // 减半，快速衰减
	}

	// 判断是否达到稳定阈值
	if (temp >= boundary) 
	{
		*cnt = 0;
		return 1;  // 电机已就绪
	}
	else
	{
		*cnt = temp;
		return 0;  // 未就绪
	}
}
