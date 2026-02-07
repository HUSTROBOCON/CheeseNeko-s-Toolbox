#include "PID.h"
#include <math.h> // 需要包含 math.h 使用 fminf/fmaxf 或自定义宏

/**
 * @brief 系统 PID 周期延迟
 * 调用此函数以保持任务周期与 GLOBAL_PID_PERIOD 同步
 */
void Pid_Delay(float time_ms,float rate)
{
    osDelay(time_ms * rate);
}

/**
 * @brief 自动频率补偿的 PID 计算
 * @param Reference 目标值
 * @param Present_Feedback 反馈值
 * @param PID_Struct PID结构体 (内存存储的是基于 BasePeriod 调好的参数)
 * @param BasePeriod 调参时参考的基础周期 (ms)
 * @param ActualPeriod 当前运行的实际周期 (ms)
 * @note  函数会自动缩放 Ki 和 Kd 以保持相同的电机动态响应
 */
float Pid_Regulate_Auto(float Reference, float Present_Feedback, PIDStructTypedef *PID_Struct, float BasePeriod, float ActualPeriod)
{
    // 如果周期参数无效或周期一致，直接计算
    if (BasePeriod <= 0 || ActualPeriod <= 0 || BasePeriod == ActualPeriod)
    {
        return Pid_Regulate(Reference, Present_Feedback, PID_Struct);
    }
    // 计算缩放比例
    float ratio = ActualPeriod / BasePeriod;

    // 备份并缩放参数
    float original_Ki = PID_Struct->Ki;
    float original_Kd = PID_Struct->Kd;
    PID_Struct->Ki = original_Ki * ratio;        // 频率越高(ratio越小)，单步积分越小
    PID_Struct->Kd = original_Kd / ratio;        // 频率越高(ratio越小)，微分增益需越大
    float output = Pid_Regulate(Reference, Present_Feedback, PID_Struct);
    PID_Struct->Ki = original_Ki;
    PID_Struct->Kd = original_Kd;
    return output;
}

/**
 * @brief PID 控制器计算函数
 * @param PID 结构体指针, Reference 目标值, Present_Feedback 当前反馈值
 * @retval float 控制输出值
 */
float Pid_Regulate(float Reference, float Present_Feedback, PIDStructTypedef *PID_Struct)
{
    float Error;
    float Error_Inc;
    float pTerm;
    float iTerm;
    float dTerm;
    float new_integral;
    float Output;
    /*Error computation*/
    Error = Reference - Present_Feedback;

    /*proportional term computation*/
    pTerm = Error * PID_Struct->Kp;

    /*Integral term computation*/
    if (PID_Struct->Ki == 0)
        PID_Struct->Integral = 0;
    else
    {
        iTerm = Error * PID_Struct->Ki;
        new_integral = PID_Struct->Integral + iTerm;

        /*limit integral*/
        if (new_integral > PID_Struct->LimitIntegral)
            PID_Struct->Integral = PID_Struct->LimitIntegral;
        else if (new_integral < -1 * PID_Struct->LimitIntegral)
            PID_Struct->Integral = -1 * PID_Struct->LimitIntegral;
        else
            PID_Struct->Integral = new_integral;
    }

    /*differential term computation*/
    Error_Inc = Error - PID_Struct->PreError;
    dTerm = Error_Inc * PID_Struct->Kd;
    PID_Struct->PreError = Error;
    Output = pTerm + PID_Struct->Integral + dTerm;

    /*compensate*/
    if (Output > 0)
    {
        Output += PID_Struct->Compensate;
        if (Output < 0) Output = 0;
    }
    else if (Output < 0)
    {
        Output -= PID_Struct->Compensate;
        if (Output > 0) Output = 0;
    }

    /*range error check*/
    if (PID_Struct->RangeError < 0) PID_Struct->RangeError = 0;
    if (Error < PID_Struct->RangeError && Error > -PID_Struct->RangeError)
    {
        PID_Struct->Integral = 0;
        Output = 0;
    }

    /*limit output*/
    if (PID_Struct->LimitOutput < 0)PID_Struct->LimitOutput = 0; // 防止负值限幅
    Output = fmaxf(-PID_Struct->LimitOutput, fminf(Output, PID_Struct->LimitOutput));
    
    return Output;
}

