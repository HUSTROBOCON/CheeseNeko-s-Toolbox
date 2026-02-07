/**
 ******************************************************************************
 * @file    Cylinder.c
 * @author  徐哲轩
 * @date    2026年2月1日
 * @brief   气缸控制模块实现
 ******************************************************************************
 */

#include "Cylinder.h"
#include "i2c.h"

/* ================== 全局变量 ================== */
Cylinder_Unit_TypeDef Global_Cylinders[CYL_COUNT];

/* ================== 辅助宏 ================== */
// PCF8574T 7位基地址高4位固定为 0100 (0x20)
// 实际地址由低3位 A2 A1 A0 决定
#define PCF8574_BASE_7BIT_ADDR 0x20

#define BOARD_1_A2_A1_A0 0x00 // 板1硬件地址设置: 000
#define BOARD_2_A2_A1_A0 0x01 // 板2硬件地址设置: 001
#define BOARD_3_A2_A1_A0 0x02 // 板3硬件地址设置: 010

#define IO_BOARD_1_ADDR ((PCF8574_BASE_7BIT_ADDR | BOARD_1_A2_A1_A0) << 1) // 扩展板1地址
#define IO_BOARD_2_ADDR ((PCF8574_BASE_7BIT_ADDR | BOARD_2_A2_A1_A0) << 1) // 扩展板2地址
#define IO_BOARD_3_ADDR ((PCF8574_BASE_7BIT_ADDR | BOARD_3_A2_A1_A0) << 1) // 扩展板3地址

/* ================== 函数实现 ================== */

/**
 * @brief  初始化气缸组配置
 */
void Cylinder_Init(void)
{
    // --- 板卡 1 配置 (前8个气缸) ---
    Global_Cylinders[CYL_CLAW_1]         = (Cylinder_Unit_TypeDef){CYL_CLAW_1,         IO_BOARD_1_ADDR, 0x02, false, false};
    Global_Cylinders[CYL_CLAW_2]         = (Cylinder_Unit_TypeDef){CYL_CLAW_2,         IO_BOARD_1_ADDR, 0x04, false, false};
    Global_Cylinders[CYL_CLAW_3]         = (Cylinder_Unit_TypeDef){CYL_CLAW_3,         IO_BOARD_2_ADDR, 0x20, false, false};
    Global_Cylinders[CYL_FLIP_CLAW]      = (Cylinder_Unit_TypeDef){CYL_FLIP_CLAW,      IO_BOARD_2_ADDR, 0x40, false, false};
    Global_Cylinders[CYL_ROD_PUSH]       = (Cylinder_Unit_TypeDef){CYL_ROD_PUSH,       IO_BOARD_2_ADDR, 0x80, false, false};
    Global_Cylinders[CYL_LEFT_CUBE_GRIPPER]  = (Cylinder_Unit_TypeDef){CYL_LEFT_CUBE_GRIPPER,  IO_BOARD_3_ADDR, 0x04, false, false};
    Global_Cylinders[CYL_RIGHT_CUBE_GRIPPER] = (Cylinder_Unit_TypeDef){CYL_RIGHT_CUBE_GRIPPER, IO_BOARD_3_ADDR, 0x08, false, false};
    Global_Cylinders[CYL_LEFT_CUBE_LIFT]     = (Cylinder_Unit_TypeDef){CYL_LEFT_CUBE_LIFT,     IO_BOARD_3_ADDR, 0x40, false, true};

    // --- 板卡 2 配置 (后7个气缸) ---
    Global_Cylinders[CYL_RIGHT_CUBE_LIFT]    = (Cylinder_Unit_TypeDef){CYL_RIGHT_CUBE_LIFT,    IO_BOARD_3_ADDR, 0x80, false, false};
    Global_Cylinders[CYL_LEFT_CUBE_PUSH]     = (Cylinder_Unit_TypeDef){CYL_LEFT_CUBE_PUSH,     IO_BOARD_3_ADDR, 0x02, false, false};
    Global_Cylinders[CYL_RIGHT_CUBE_PUSH]    = (Cylinder_Unit_TypeDef){CYL_RIGHT_CUBE_PUSH,    IO_BOARD_3_ADDR, 0x01, false, true};
    Global_Cylinders[CYL_CENTER_PLATFORM]    = (Cylinder_Unit_TypeDef){CYL_CENTER_PLATFORM,    IO_BOARD_3_ADDR, 0x10, false, true};
    Global_Cylinders[CYL_CENTER_PUSH]        = (Cylinder_Unit_TypeDef){CYL_CENTER_PUSH,        IO_BOARD_3_ADDR, 0x20, false, true};

    // --- 气泵与气压阀配置 (板卡 2) ---
    Global_Cylinders[CYL_PUMP]       = (Cylinder_Unit_TypeDef){CYL_PUMP,       IO_BOARD_2_ADDR, 0x08, false, false};
    Global_Cylinders[CYL_PUMP_VALVE] = (Cylinder_Unit_TypeDef){CYL_PUMP_VALVE, IO_BOARD_2_ADDR, 0x01, false, false};

    // 初始化硬件状态
    Cylinder_Update(Global_Cylinders);
}

/**
 * @brief  设置单个气缸状态
 */
void Cylinder_Set_State(Cylinder_ID_Enum id, bool state)
{
    if (id < CYL_COUNT)
    {
        Global_Cylinders[id].state = state;
    }
}

/**
 * @brief  更新所有气缸硬件输出 (I2C发送)
 */
void Cylinder_Update(Cylinder_Unit_TypeDef *cylinders)
{
    // 输出低电平有效，初始值全高 (0xFF)
    uint8_t Board_Data[3] = {0xFF, 0xFF, 0xFF};
    uint8_t Board_Addr[3] = {IO_BOARD_1_ADDR, IO_BOARD_2_ADDR, IO_BOARD_3_ADDR};

    for (int i = 0; i < CYL_COUNT; i++)
    {
        Cylinder_Unit_TypeDef *unit = &cylinders[i];

        // 低电平有效
        if (unit->state ^ unit->invert_flag) // 异或处理反转逻辑
        {
            switch (unit->i2c_addr)
            {
            case IO_BOARD_1_ADDR:
                Board_Data[0] &= ~(unit->pin_mask); // 拉低对应位
                break;
            case IO_BOARD_2_ADDR:
                Board_Data[1] &= ~(unit->pin_mask); // 拉低对应位
                break;
            case IO_BOARD_3_ADDR:
                Board_Data[2] &= ~(unit->pin_mask); // 拉低对应位
                break;
            default:
                break;
            }
        }
    }
    // 批量发送 I2C
    for (int i = 0; i < 3; i++)
    {
        HAL_I2C_Master_Transmit(&hi2c1, Board_Addr[i], &Board_Data[i], 1, 10);
    }
}
