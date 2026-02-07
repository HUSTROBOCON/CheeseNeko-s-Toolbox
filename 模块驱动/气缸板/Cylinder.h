/**
 ******************************************************************************
 * @file    Cylinder.h
 * @author  徐哲轩
 * @date    2026年2月1日
 * @brief   气缸控制模块 (PC控制端)
 ******************************************************************************
 */

#ifndef _CYLINDER_H_
#define _CYLINDER_H_

#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

/* ================== 类型定义 ================== */

/**
 * @brief 气缸枚举标号
 */
typedef enum
{
    // --- 三爪夹爪组 ---
    CYL_CLAW_1 = 0,         // 夹爪1
    CYL_CLAW_2,             // 夹爪2
    CYL_CLAW_3,             // 夹爪3
    CYL_3_CLAW_LIFT,        // 三夹爪抬升

    // --- 取杆机构 ---
    CYL_FLIP_CLAW,          // 取杆翻转夹爪
    CYL_ROD_PUSH,           // 取杆摩擦轮推送

    // --- 方块机构 ---
    CYL_LEFT_CUBE_GRIPPER,  // 左方块夹爪
    CYL_RIGHT_CUBE_GRIPPER, // 右方块夹爪
    CYL_LEFT_CUBE_LIFT,     // 左方块平台抬升
    CYL_RIGHT_CUBE_LIFT,    // 右方块平台抬升
    CYL_LEFT_CUBE_PUSH,     // 左方块推送
    CYL_RIGHT_CUBE_PUSH,    // 右方块推送
    CYL_CENTER_PLATFORM,    // 中央方块平台
    CYL_CENTER_PUSH,        // 中央方块推送

    // --- 中央机构 --- 
    CYL_DOCKING,            // 对接气缸

    // --- 气泵控制 ---
    CYL_PUMP,               // 气泵
    CYL_PUMP_VALVE,         // 气泵溢流阀/气压阀

    CYL_COUNT               // 气缸总数
} Cylinder_ID_Enum;

/**
 * @brief 单个气缸配置结构体
 */
typedef struct
{
    Cylinder_ID_Enum id;    // 气缸标号
    uint8_t i2c_addr;       // 对应的 PCF8574 I2C 设备地址 (8位写地址)
    uint8_t pin_mask;       // 引脚掩码 (如 0x01, 0x02...)
    bool    state;          // 当前状态 (true: 动作/开, false: 复位/关)
    bool    invert_flag;    // 反转标志 (true: 反转高低电平逻辑)
} Cylinder_Unit_TypeDef;

// 扩展板基础地址
#define PCF8574_ADDR_BASE   0x40 

/* ================== 全局变量声明 ================== */
extern Cylinder_Unit_TypeDef Global_Cylinders[CYL_COUNT]; // 全局气缸单元数组

/* ================== 函数声明 ================== */
void Cylinder_Init(void);
void Cylinder_Update(Cylinder_Unit_TypeDef* cylinders);
void Cylinder_Set_State(Cylinder_ID_Enum id, bool state);

#endif
