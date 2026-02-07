#ifndef PID_H_
#define PID_H_

#include "cmsis_os2.h"

/**
 * @brief 全局 PID 控制周期宏定义 (单位: ms)
 * 修改此值会自动影响所有使用 Pid_Delay 的任务，并被 Pid_Regulate_Auto 用于参数缩放
 */
#define GLOBAL_PID_PERIOD 5.0f         // PID 控制周期 (单位: ms) 
#define GLOBAL_BASE_PID_PERIOD 10.0f  // 调参时参考的基础周期 (单位: ms) 

/**
 * @brief PID 结构体定义
 * 用于存储 PID 控制器的参数和状态
 */
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float LimitOutput;      // 输出限幅值
    float LimitIntegral;    // 积分限幅值
    float Integral;         // 积分项累加值
    float Compensate;       // 补偿
    float RangeError;        // 误差范围
    float PreError;         // 上一次误差
} PIDStructTypedef;


float Pid_Regulate(float Reference, float Present_Feedback, PIDStructTypedef* PID_Struct);
float Pid_Regulate_Auto(float Reference, float Present_Feedback, PIDStructTypedef *PID_Struct, float BasePeriod, float ActualPeriod);
void Pid_Delay(float time_ms,float rate);

#endif
