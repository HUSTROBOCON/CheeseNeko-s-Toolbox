#include "Unitree_Motor.h"
#include "CRC.h"
#include "Macro_Func.h"
#include <string.h>

/* ================== 内部辅助函数声明 ================== */
static void UnitreeMotor_PackCmd(UnitreeMotor *motor);

/**
 * @brief Unitree A1 CRC32 Core 算法
 * @param ptr 数据指针 (需转换为 uint32_t*)
 * @param len 校验的 32位字 长度 (例如 30字节数据去掉最后2字节后为 28字节 = 7 words)
 */

/* ================== 接口函数实现 ================== */

/**
 * @brief 电机数据结构初始化
 * @param motor 电机对象指针
 * @param huart 绑定的串口句柄指针 (如 &huart2)
 * @param id 电机ID
 * @param type 电机类型
 */
void UnitreeMotor_Init(UnitreeMotor *motor, UnitreeMotorType type, uint8_t id)
{
    // 清零整个结构体，防止垃圾值
    memset(motor, 0, sizeof(UnitreeMotor));

    motor->id   = id;
    motor->type = type;

    // 根据电机类型设置参数和协议长度
    switch (type)
    {
    case UNITREE_MOTOR_TYPE_GO1: // Go1 (M8010)
        motor->param_scale.tx_msg_len = sizeof(UnitreeGo_ControlData_t);
        motor->param_scale.rx_msg_len = sizeof(UnitreeGo_MotorData_t);
        motor->param_scale.rx_head[0] = 0xFD;
        motor->param_scale.rx_head[1] = 0xEE;

        motor->param_scale.kp_scale  = UNITREE_GO1_KP_SCALE;
        motor->param_scale.kw_scale  = UNITREE_GO1_KW_SCALE;
        motor->param_scale.pos_scale = UNITREE_GO1_POS_SCALE;
        motor->param_scale.spd_scale = UNITREE_GO1_SPD_SCALE;
        motor->param_scale.tor_scale = UNITREE_GO1_TORQUE_SCALE;
        motor->param_scale.tor_limit = UNITREE_GO1_LIMIT_TORQUE;
        motor->param_scale.spd_limit = UNITREE_GO1_LIMIT_SPEED;
        break;

    case UNITREE_MOTOR_TYPE_A1: // A1 (6010)
        motor->param_scale.tx_msg_len = sizeof(UnitreeA1_ControlData_t);
        motor->param_scale.rx_msg_len = sizeof(UnitreeA1_MotorData_t); // 78 Bytes
        motor->param_scale.rx_head[0] = 0xFE;
        motor->param_scale.rx_head[1] = 0xEE;

        motor->param_scale.kp_scale  = UNITREE_A1_KP_SCALE;
        motor->param_scale.kw_scale  = UNITREE_A1_KW_SCALE;
        motor->param_scale.pos_scale = UNITREE_A1_POS_SCALE;
        motor->param_scale.spd_scale = UNITREE_A1_SPD_SCALE;
        motor->param_scale.tor_scale = UNITREE_A1_TORQUE_SCALE;
        motor->param_scale.tor_limit = UNITREE_A1_LIMIT_TORQUE;
        motor->param_scale.spd_limit = UNITREE_A1_LIMIT_SPEED;
        break;

    default:
        break;
    }

    // 设置初始状态 (0力矩，0速度)
    motor->mode         = 0;
    motor->fbk_rx_index = 0;

    // 初始打包一次，确保 buffer 数据就绪
    UnitreeMotor_PackCmd(motor);
}

void UnitreeMotor_Control_Update(UnitreeMotor *motor)
{
    if (motor == NULL)
        return;

    // 检查是否有数据包就绪，如果有则进行解包
    if (motor->Flag.fbk_ready_flag)
    {
        UnitreeMotor_UnpackFbk(motor);
        motor->Flag.fbk_ready_flag = 0;
    }

    switch (motor->ctrl_mode)
    {
    case UNITREE_CTRL_POS_SPD:
        // 位置环控制输出速度期望值
        motor->mode = 1;
        motor->W    = Pid_Regulate_Auto(
            motor->Pos,
            motor->rx_Pos,
            &motor->PID.PosPID,
            GLOBAL_BASE_PID_PERIOD,
            GLOBAL_PID_PERIOD);
        // 速度环控制输出力矩期望值
        motor->T = Pid_Regulate_Auto(
            motor->W,
            motor->rx_W,
            &motor->PID.SpeedPID,
            GLOBAL_BASE_PID_PERIOD,
            GLOBAL_PID_PERIOD);
        break;

    case UNITREE_CTRL_POS:
        // 位置环控制输出速度期望值
        motor->mode = 1;
        motor->W    = Pid_Regulate_Auto(
            motor->Pos,
            motor->rx_Pos,
            &motor->PID.PosPID,
            GLOBAL_BASE_PID_PERIOD,
            GLOBAL_PID_PERIOD);
        break;
    case UNITREE_CTRL_NATIVE:
        motor->mode = 1;
        break;
    case UNITREE_CTRL_STOP:
        motor->mode = 0; // 停止控制
        break;
    default:
        motor->mode = 0; // 停止控制
        break;
    }
}

/**
 * @brief 统一发送电机控制指令
 * @param motor 电机对象指针
 * @param huart 绑定的串口句柄指针
 */
void UnitreeMotor_Transmit(UnitreeMotor *motor, UART_HandleTypeDef *huart)
{
    if (!motor || !huart)
        return;

    // 检查发送锁定 (Flag.one_shot_flag: 1=请求单次并待执行, 2=已完成指令待清除)
    if (motor->Flag.one_shot_flag == 2)
        return;

    // 正常控制指令打包
    UnitreeMotor_PackCmd(motor);

    // 2. 执行发送 (启动阻塞发送，超时 10ms)
    HAL_UART_Transmit(huart, (uint8_t *)&motor->cmd_buffer, motor->param_scale.tx_msg_len, 10);

    motor->tx_count++;

    // 如果处于单次发送请求状态，发送后立即锁定
    if (motor->Flag.one_shot_flag == 1)
    {
        motor->Flag.one_shot_flag = 2;
    }
}

/**
 * @brief 解析电机反馈数据
 * @param motor 电机对象指针 (需确保 fbk_pack 已填充最新数据)
 */
void UnitreeMotor_UnpackFbk(UnitreeMotor *motor)
{
    switch (motor->type)
    {
    case UNITREE_MOTOR_TYPE_GO1: {
        UnitreeGo_MotorData_t *pFbk = (UnitreeGo_MotorData_t *)motor->fbk_buffer;

        // 校验CRC (减去最后2字节的CRC16字段)
        uint16_t calc_crc = crc_ccitt(0, (uint8_t *)pFbk, sizeof(UnitreeGo_MotorData_t) - sizeof(pFbk->CRC16));

        if (pFbk->CRC16 != calc_crc)
        {
            motor->crc_err_count++;
            // CRC错误时不更新数据
            motor->Flag.connected = 0;
            return;
        }
        else
        {
            motor->Flag.connected = 1;
            motor->rx_count++;

            // 提取状态
            motor->rx_Temp      = pFbk->temp;
            motor->rx_MError    = pFbk->MError;
            motor->rx_footForce = pFbk->force;

            // 解析运动数据 (使用浮点转换系数)
            motor->rx_W   = (float)pFbk->speed / motor->param_scale.spd_scale;
            motor->rx_T   = (float)pFbk->torque / motor->param_scale.tor_scale;
            motor->rx_Pos = (float)pFbk->pos / motor->param_scale.pos_scale;
        }
        break;
    }
    case UNITREE_MOTOR_TYPE_A1: {
        UnitreeA1_MotorData_t *pFbk = (UnitreeA1_MotorData_t *)motor->fbk_buffer;

        // CRC 校验 (18 words = 72 bytes)
        uint32_t calc_crc = crc32_core((uint32_t *)pFbk, 18);
        if (pFbk->CRC32 != calc_crc)
        {
            motor->crc_err_count++;
            return;
        }
        else
        {
            motor->Flag.connected = 1;
            motor->rx_count++;

            // 提取状态
            motor->rx_Temp      = pFbk->Temp;
            motor->rx_MError    = pFbk->MError;
            motor->rx_footForce = pFbk->Force16;

            // 解析运动数据 (使用浮点转换系数)
            motor->rx_W   = (float)pFbk->W / motor->param_scale.spd_scale;
            motor->rx_T   = (float)pFbk->T / motor->param_scale.tor_scale;
            motor->rx_Pos = (float)pFbk->Pos / motor->param_scale.pos_scale;
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief 宇树电机协议流解析 (字节对齐状态机)
 * @param motor 电机对象
 * @param byte 单字节数据
 */
void Process_Unitree_Motor_Byte(UnitreeMotor *motor, uint8_t byte)
{
    // 使用流缓冲区进行状态机匹配
    uint8_t *p_stream = motor->rx_stream_buffer;

    // 状态机处理
    switch (motor->fbk_rx_index)
    {
    case 0:
        if (byte == motor->param_scale.rx_head[0])
        {
            p_stream[motor->fbk_rx_index++] = byte;
        }
        break;

    case 1:
        // 寻找包头字节2
        if (byte == motor->param_scale.rx_head[1])
        {
            p_stream[motor->fbk_rx_index++] = byte;
        }
        else
        {
            // 校验错位处理
            motor->fbk_rx_index = (byte == motor->param_scale.rx_head[0]) ? 1 : 0;
        }
        break;

    case 2: {
        // 检查ID是否匹配 (Go1协议ID占低4bit，A1为整字节)
        p_stream[motor->fbk_rx_index++] = byte;
        uint8_t rx_id_val               = byte;
        if (motor->type == UNITREE_MOTOR_TYPE_GO1)
        {
            rx_id_val &= 0x0F;
        }
        if (rx_id_val != motor->id)
        {
            motor->fbk_rx_index = 0;
        }
        break;
    }

    default:
        // 接收剩余数据
        p_stream[motor->fbk_rx_index++] = byte;

        // 检查是否接收完整
        if (motor->fbk_rx_index >= motor->param_scale.rx_msg_len)
        {
            // 搬运数据到解包缓冲区
            memcpy(motor->fbk_buffer, p_stream, motor->param_scale.rx_msg_len);
            motor->Flag.fbk_ready_flag = 1; // 标记数据就绪
            motor->fbk_rx_index   = 0;
        }
        break;
    }
}

/* ================== 内部函数实现 ================== */

/**
 * @brief 将发送给,电机的浮点参数转换为定点类型参数 (打包指令)
 */
static void UnitreeMotor_PackCmd(UnitreeMotor *motor)
{
    float kp = 0.0f, kw = 0.0f, pos = 0.0f, spd = 0.0f, tau = 0.0f;
    uint8_t mode               = motor->mode;
    uint8_t id                 = motor->id;

    // 根据软件控制模式，选择下发给电机的底层参数 (KP, KW, Pos, W, T)
    switch (motor->ctrl_mode)
    {
    case UNITREE_CTRL_POS: // 软件位置环: 下发速度 + 阻尼 (KP=0, KW>0)
        kp          = 0.0f;
        kw          = motor->K_W;
        pos         = 0.0f;
        spd         = motor->W;
        tau         = 0.0f;
        motor->Flag.safe_switch = 1;
        break;

    case UNITREE_CTRL_POS_SPD: // 软件串级环: 纯力矩模式 (KP=0, KW=0)
        kp          = 0.0f;
        kw          = 0.0f;
        pos         = 0.0f;
        spd         = 0.0f;
        tau         = motor->T;
        motor->Flag.safe_switch = 1;
        break;
    default:
        if (motor->Flag.safe_switch)
        {
            // 切换回原生控制模式时，清除残留的速度和力矩指令
            motor->W    = 0.0f;
            motor->T    = 0.0f;
            motor->Pos  = motor->rx_Pos; // 当前位置
            motor->Flag.safe_switch = 0;
        }
        kp  = motor->K_P;
        kw  = motor->K_W;
        pos = motor->Pos;
        spd = motor->W;
        tau = motor->T;
        break;
    }

    // ============ 物理限制 ============
    SATURATE(tau, -motor->param_scale.tor_limit, motor->param_scale.tor_limit);
    SATURATE(spd, -motor->param_scale.spd_limit, motor->param_scale.spd_limit);

    // ============ 协议打包 ============
    switch (motor->type)
    {
    case UNITREE_MOTOR_TYPE_GO1: {
        UnitreeGo_ControlData_t *pCmd = (UnitreeGo_ControlData_t *)motor->cmd_buffer;
        memset(pCmd, 0, sizeof(UnitreeGo_ControlData_t));

        // 2. 设置包头
        pCmd->head[0] = 0xFE;
        pCmd->head[1] = 0xEE;

        // 3. 填充协议数据
        pCmd->id     = id;
        pCmd->status = mode;
        pCmd->K_P    = (int16_t)(kp * motor->param_scale.kp_scale);
        pCmd->K_W    = (int16_t)(kw * motor->param_scale.kw_scale);
        pCmd->Pos    = (int32_t)(pos * motor->param_scale.pos_scale);
        pCmd->W      = (int16_t)(spd * motor->param_scale.spd_scale);
        pCmd->T      = (int16_t)(tau * motor->param_scale.tor_scale);

        uint16_t crc_val = crc_ccitt(0, (uint8_t *)pCmd, sizeof(UnitreeGo_ControlData_t) - sizeof(pCmd->CRC16));
        pCmd->CRC16      = (uint16_t)(crc_val);
        break;
    }

    case UNITREE_MOTOR_TYPE_A1: {
        UnitreeA1_ControlData_t *pCmd = (UnitreeA1_ControlData_t *)motor->cmd_buffer;
        memset(pCmd, 0, sizeof(UnitreeA1_ControlData_t));

        // 设置包头
        pCmd->head[0] = 0xFE;
        pCmd->head[1] = 0xEE;

        pCmd->id        = id;
        pCmd->status    = (mode == 0) ? 0 : 10; // 0:停止, 10:伺服控制
        pCmd->ModifyBit = 0xFF;                 // 允许修改参数
        pCmd->ReadBit   = 0x00;
        pCmd->K_P       = (int16_t)(kp * motor->param_scale.kp_scale);
        pCmd->K_W       = (int16_t)(kw * motor->param_scale.kw_scale);
        pCmd->Pos       = (int32_t)(pos * motor->param_scale.pos_scale);
        pCmd->W         = (int16_t)(spd * motor->param_scale.spd_scale);
        pCmd->T         = (int16_t)(tau * motor->param_scale.tor_scale);
        // CRC Calculation (7 words = 28 bytes, covering up to Res[1])
        pCmd->CRC32 = crc32_core((uint32_t *)pCmd, 7);
        break;
    }

    default:
        break;
    }
}
