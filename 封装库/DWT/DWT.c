#include "DWT.h"

/* 
 * 性能测量工具 - 变量定义部分
 */

// 全局变量定义
volatile float g_dwt_last_time_us = 0.0f;
volatile float g_dwt_max_time_us = 0.0f;
volatile float g_dwt_min_time_us = 1000000.0f;

// 初始化函数 (只运行一次，无需内联)
void DWT_Init(void) {
    // 检查是否支持 DWT
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 开启 TRC
    }
    
    DWT->CYCCNT = 0;                    // 清空计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 开启 CYCCNT
}

// 重置统计数据 (调试交互用，无需内联)
void DWT_Reset_Stats(void) {
    g_dwt_max_time_us = 0.0f;
    g_dwt_min_time_us = 1000000.0f;
}

/* 
 * 注意: DWT_Start 和 DWT_Stop_To_Us 已移至头文件 DWT.h 
 * 以 "static inline" 方式实现，确保最高性能。
 */
