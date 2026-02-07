#include "DJI_Motor.h"
#include "fdcan.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

// DJIMotor_PID_Init is now static helper or inline, but we make a full init func
static void DJIMotor_PID_Set(DJIMotor_PID_struct *MotorPID, float limit_output)
{
	// Common settings
	MotorPID->PosPID.Kp = 5.0f;
	MotorPID->PosPID.Ki = 0.005f;
	MotorPID->PosPID.Kd = 0.10f;
	MotorPID->PosPID.Integral = 0;
	MotorPID->PosPID.LimitIntegral = 50.0f;
	MotorPID->PosPID.LimitOutput = 200.0f;
	MotorPID->PosPID.PreError = 0;

	MotorPID->SpeedPID.Kp = 100.0f;
	MotorPID->SpeedPID.Ki = 0.1f;
	MotorPID->SpeedPID.Kd = 0.0f;
	MotorPID->SpeedPID.Integral = 0;
	MotorPID->SpeedPID.LimitIntegral = 500.0f;
	MotorPID->SpeedPID.LimitOutput = limit_output;
	MotorPID->SpeedPID.PreError = 0;

	// Clear Reserve
	memset(&MotorPID->ReservePID, 0, sizeof(PIDStructTypedef));
}

void DJI_Motor_Init(DJIMotorTypeDef *dji_motor, DJIMotorType type, uint32_t can_id)
{
	if (dji_motor == NULL)
		return;

	// Reset Data
	memset(dji_motor, 0, sizeof(DJIMotorTypeDef));

	// Config ID
	dji_motor->Motor.ID = can_id;
	dji_motor->Type = type;

	// Limit Output based on type
	float limit_out = 0.0f;
	if (type == DJI_MOTOR_TYPE_3508)
		limit_out = 16000.0f;
	else if (type == DJI_MOTOR_TYPE_2006)
		limit_out = 8000.0f;

	// Init PID
	DJIMotor_PID_Set(&dji_motor->PID, limit_out);
}

/***********************************Dj消息发送函数***********************************/
void dji_can_transmit_eid(uint16_t id, uint8_t bus_id, uint8_t *data, uint8_t len)
{
	if (len > 8)
	{
		len = 8;
	}

	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.IdType = FDCAN_STANDARD_ID;	 // 标准帧
	TxHeader.Identifier = id;				 // 11位标准ID（如 0x201）
	TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧（非远程帧）
	TxHeader.DataLength = len;				 // 数据长度（0~8）

	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 关闭 BRS（经典 CAN）
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;	// 经典 CAN 格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

#ifdef USE_FDCAN_1
	if (bus_id == MYFDCAN1)
	{
		HAL_StatusTypeDef status;
		uint16_t can_timeout = 0;

		// 尝试发送，最多重试 1500 次（或直到 FIFO 有空位），FDCAN 无邮箱概念
		do
		{
			status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
			if (status == HAL_BUSY)
			{
				// TX FIFO 满，短暂等待
				// 注意：不要用 HAL_Delay(1) 如果在中断中！
				// 可改用计数延时或让出 CPU
				for (volatile int i = 0; i < 1000; i++)
					; // 简单延时
				can_timeout++;
			}
			else
			{
				break; // 成功或发生错误（非 BUSY）
			}
		} while (status == HAL_BUSY && can_timeout <= 1500);
	}
#endif

#ifdef USE_FDCAN_2
	if (bus_id == MYFDCAN2)
	{
		HAL_StatusTypeDef status;
		uint16_t can_timeout = 0;

		// 尝试发送，最多重试 1500 次（或直到 FIFO 有空位），FDCAN 无邮箱概念
		do
		{
			status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
			if (status == HAL_BUSY)
			{
				// TX FIFO 满，短暂等待
				// 注意：不要用 HAL_Delay(1) 如果在中断中！
				// 可改用计数延时或让出 CPU
				for (volatile int i = 0; i < 1000; i++)
					; // 简单延时
				can_timeout++;
			}
			else
			{
				break; // 成功或发生错误（非 BUSY）
			}
		} while (status == HAL_BUSY && can_timeout <= 1500);
	}
#endif
}

void dji_can_set(uint16_t controller_id, uint8_t bus_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
	uint8_t buffer[8];
	uint16_t id = 0;

	if (controller_id <= 0x08)
	{
		if (controller_id >= 0x05)
		{
			id = 0x1FF;
		}
		else
		{
			id = 0x200;
		}
	}

	buffer[0] = (current1 >> 8) & 0xFF;
	buffer[1] = current1 & 0xFF;
	buffer[2] = (current2 >> 8) & 0xFF;
	buffer[3] = current2 & 0xFF;
	buffer[4] = (current3 >> 8) & 0xFF;
	buffer[5] = current3 & 0xFF;
	buffer[6] = (current4 >> 8) & 0xFF;
	buffer[7] = current4 & 0xFF;

	dji_can_transmit_eid(id, bus_id, buffer, 8);
}

/***********************************Dj消息接收函数***********************************/
float Get_RM2006_Distance(MotorTypeDef Motor)
{
	int Distance = Motor.encoder - Motor.encoderPre;

	if (abs(Distance) > 4000)
	{
		Distance = Distance - Distance / abs(Distance) * 8192;
	}

	return ((float)Distance * 360.0f / 36.0f / 8192.0f);
}

float Get_RM3508_Distance(MotorTypeDef Motor)
{
	int Distance = Motor.encoder - Motor.encoderPre;

	if (abs(Distance) > 4000)
	{
		Distance = Distance - Distance / abs(Distance) * 8192;
	}

	return ((float)Distance * 360.0f / (3591.0f / 187.0f) / 8192.0f);
}

short temp_angle = 0;
short temp_speed = 0;
short temp_current = 0;
void Process_DJI_Frame(FDCANFrame *Frame_Process, DJIMotorTypeDef *DJIMotor)
{
	if (DJIMotor == NULL)
		return;

	uint32_t id = Frame_Process->Id.StdID & 0x00F;
	temp_angle = (short)(Frame_Process->Data.uchars[0] << 8 | Frame_Process->Data.uchars[1]);
	temp_speed = (short)(Frame_Process->Data.uchars[2] << 8 | Frame_Process->Data.uchars[3]);
	temp_current = (short)(Frame_Process->Data.uchars[4] << 8 | Frame_Process->Data.uchars[5]);

	if (id == DJIMotor->Motor.ID)
	{
		if (DJIMotor->Motor.enable == 0)
			DJIMotor->Motor.enable = 1;

		if (DJIMotor->Motor.encoderPre == 0 && DJIMotor->Motor.encoder == 0)
		{
			DJIMotor->Motor.encoderPre = DJIMotor->Motor.encoder = temp_angle;
		}
		else
		{
			DJIMotor->Motor.encoderPre = DJIMotor->Motor.encoder;
			DJIMotor->Motor.encoder = temp_angle;
		}
		DJIMotor->Motor.PrePosition = DJIMotor->Motor.PositionMeasure;

		// Distinguish Motor Type
		if (DJIMotor->Type == DJI_MOTOR_TYPE_3508)
		{
			DJIMotor->Motor.PositionMeasure += Get_RM3508_Distance(DJIMotor->Motor);
			DJIMotor->Motor.SpeedMeasure = (float)(temp_speed / (3591.0f / 187.0f));
		}
		else if (DJIMotor->Type == DJI_MOTOR_TYPE_2006)
		{
			DJIMotor->Motor.PositionMeasure += Get_RM2006_Distance(DJIMotor->Motor);
			DJIMotor->Motor.SpeedMeasure = (float)(temp_speed / 36.0f);
		}

		DJIMotor->Motor.CurrentMeasure = (float)(temp_current);
	}
}

void DJIMotor_Control_Update(DJIMotorTypeDef *DJIMotor)
{
	if (DJIMotor == NULL)
		return; // 防御性编程

	switch (DJIMotor->Motor.mode)
	{
	case MOTOR_POSITION:
		// 位置环控制输出速度期望值
		DJIMotor->Motor.SpeedExpected = Pid_Regulate(
			DJIMotor->Motor.PositionExpected,
			DJIMotor->Motor.PositionMeasure,
			&DJIMotor->PID.PosPID);

	case MOTOR_SPEED:
		// 速度环控制输出电流期望值
		DJIMotor->Motor.CurrentExpected = Pid_Regulate(
			DJIMotor->Motor.SpeedExpected,
			DJIMotor->Motor.SpeedMeasure,
			&DJIMotor->PID.SpeedPID);
		break;

	case MOTOR_CURRENT:
		// 电流模式：直接使用外部设定的 CurrentExpected，不做计算
		break;

	case MOTOR_ERROR:

		break;

	default:
		DJIMotor->Motor.CurrentExpected = 0.0f;
		break;
	}
}
