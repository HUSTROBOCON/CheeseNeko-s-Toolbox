#ifndef DWT_H
#define DWT_H

#include "main.h" 

/* 
 * DWT性能测量工具 
 * 1. 消除函数调用的开销 (Function Call Overhead)。
 *    普通函数调用需要保存寄存器现场(Push)、跳转(Branch)、返回(BX)、恢复现场(Pop)，这会消耗 5~20 个时钟周期。
 *    对于纳秒级的测量，这个开销是不可忽视的误差来源。
 * 
 * 2. 允许编译器优化。
 *    内联后，编译器可以直接将 DWT 读取指令嵌入到被测代码前后，极致情况下误差仅为 1~2 个周期。
 * 
 * 作者：徐哲轩
 * 日期：2026-01
 * Email：zhexuanxu@outlook.com
 * 
 */

// 全局统计变量
extern volatile float g_dwt_last_time_us;
extern volatile float g_dwt_max_time_us;
extern volatile float g_dwt_min_time_us; // 新增最小值

// 时间统计结构体
typedef struct {
    uint32_t cycle_count; // 原始周期数 (ticks)
    float time_ms;        // 转换后的时间 (毫秒)
} DWT_TimeStats;

// 系统 API
void DWT_Init(void);
void DWT_Reset_Stats(void);


// --- 内联核心函数 ---

/**
 * @brief  开始计时 (内联版)
 * @return 当前 DWT 周期数
 * @note   编译为汇编仅需 1~2 条指令，几乎无开销。
 */
static inline uint32_t DWT_Start(void) {
    return DWT->CYCCNT;
}

/**
 * @brief  停止计时并计算微秒 (内联版)
 * @param  start_cycle: 开始时的周期数
 * @return 测量结果结构体 (包含周期数和毫秒数)
 * @note   内部包含数学运算，建议仅在调试时开启，量产时可宏屏蔽。
 */
static inline DWT_TimeStats DWT_Stop_To_Struct(uint32_t start_cycle) {
    uint32_t end_cycle = DWT->CYCCNT;
    DWT_TimeStats stats;
    
    // 处理 32位 计数器溢出
    if (end_cycle >= start_cycle) {
        stats.cycle_count = end_cycle - start_cycle;
    } else {
        stats.cycle_count = (0xFFFFFFFF - start_cycle) + end_cycle + 1;
    }
    
    // 转换为毫秒 (1秒 = 1000毫秒 = SystemCoreClock 周期)
    // 公式: Cycles / (Hz / 1000)
    stats.time_ms = (float)stats.cycle_count / (SystemCoreClock / 1000.0f);
    
    return stats;
}

#endif
