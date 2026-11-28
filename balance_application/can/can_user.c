#include "can_user.h"
#include "dm_motor.h"
#include "main.h"
#include "fdcan.h"
#include "string.h"

#define chassis_can hfdcan1   /* 底盘 CAN */
#define gimbal_can  hfdcan2   /* 云台 CAN */

#define ABS(x) (((x) > 0) ? (x) : (-(x)))

/* M3508/C620 反馈解析：机械角、转速、转矩电流、温度 */
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd      = (ptr)->ecd;                              \
        (ptr)->ecd           = (uint16_t)((data)[0] << 8 | (data)[1]);  \
        (ptr)->speed_rpm     = (uint16_t)((data)[2] << 8 | (data)[3]);  \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate     = (data)[6];                               \
    }

/* 超电 0x304 电容电压解析 */
#define get_cap_measure(ptr, data)                                      \
    {                                                                   \
        (ptr)->V_cap_value = (fp32)(((data)[2] << 8) | ((data)[3])) * 0.001f; \
    }

motor_measure_t motor_gimbal[1];
motor_measure_t motor_chassis[4];
volatile uint32_t chassis_motor_rx_count[4];
volatile uint32_t chassis_motor_last_rx_ms[4];
volatile can2_command_diag_t can2_command_diag;

/* Keil Watch 接收诊断。 */
volatile uint32_t chassis_top_rx_bad_frame_count = 0;
volatile uint32_t chassis_can_rx_error_count   = 0;

/* 超时按底盘本地收到 0x307 的时刻计算，和云台的时钟无关。 */
#define GIMBAL_CHASSIS_COMMAND_TIMEOUT_MS 100U
#define CHASSIS_COMMAND_REMOTE_VALID (1U << 2)
#define CHASSIS_COMMAND_CAP (1U << 3)
#define CHASSIS_COMMAND_STOP (1U << 0)
#define CHASSIS_COMMAND_FORMAT 0x01U

/* 中断发布最新一帧。底盘任务复制后再使用，Watch 可直接查看原始解包结果。 */
volatile chassis_c_get_data_t mini_chassis;
static volatile uint32_t gimbal_chassis_command_received_at_ms;

super_cap_t super_cap;

/* 发送缓冲 */
static uint8_t chassis_can_send_data[8];
static uint8_t chassis_send_gimbal_data[8];
static uint8_t chassis_send_cap_data[8];
static uint8_t chassis_send_ramp_pitch[4];

/* 云台 CAN 接收帧计数（调试用） */
volatile uint32_t from_yuntai_can = 0;

/* 与云台 CAN_gimbal_send_chassis() 的 0x307 布局一致。 */
static void receive_gimbal_chassis_command(const uint8_t data[8])
{
    chassis_c_get_data_t command = {0};
    const uint8_t status = data[4];
    const bool stop_requested = (data[5] & CHASSIS_COMMAND_STOP) != 0U;
    const bool remote_valid = (status & CHASSIS_COMMAND_REMOTE_VALID) != 0U;

    if (data[7] != CHASSIS_COMMAND_FORMAT)
    {
        can2_command_diag.format_reject_count++;
        return;
    }

    if (stop_requested)
    {
        can2_command_diag.stop_count++;
    }
    if (!remote_valid)
    {
        can2_command_diag.remote_invalid_count++;
    }

    /* 停车或遥控无效时发布全零指令，使麦轮零力并退出爬阶。 */
    if (!stop_requested && remote_valid)
    {
        const uint8_t dial = status & 3U;

        command.x_rc_data = (int8_t)data[0];
        command.y_rc_data = (int8_t)data[1];
        command.joint_forward = (int8_t)data[2];
        command.joint_lateral = (int8_t)data[3];
        command.chassis_mode = status >> 6;
        command.climb_group = (status >> 4) & 3U;
        command.climb_mode = command.climb_group != CLIMB_NORMAL;
        command.cap_mode = (status & CHASSIS_COMMAND_CAP) != 0U;
        /* 拨轮码 3 保留，收到时按中位处理。 */
        command.climb_dial = dial <= CLIMB_DIAL_REVERSE ? dial : CLIMB_DIAL_CENTER;
    }
    for (uint8_t index = 0U; index < 8U; index++)
    {
        can2_command_diag.last_data[index] = data[index];
    }
    from_yuntai_can++;
    mini_chassis = command;
    gimbal_chassis_command_received_at_ms = HAL_GetTick();
}

/* 9025 电机（云台）编码器解析：只取机械角 */
void get_9025_motor_ecd(motor_measure_t *ptr, uint8_t *data)
{
    ptr->ecd = (uint16_t)((data[2] << 8) | (data[3]));
}

/* FDCAN 发送：经典 CAN 数据帧（1Mbps，BRS 关闭） */
static void fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t dlc_bytes, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    volatile can_bus_diag_t *diag = &can_bus_diag[hfdcan == &hfdcan1 ? 0U : 1U];

    tx.Identifier          = std_id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)dlc_bytes;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        diag->tx_fifo_full++;
        return;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data) == HAL_OK)
    {
        diag->tx_queued++;
    }
    else
    {
        diag->tx_hal_errors++;
        diag->hal_error = hfdcan->ErrorCode;
    }
}

/*
 * FDCAN RX FIFO0 回调。
 * 底盘 CAN：0x201~0x204 电机反馈、0x304 超电；
 * 云台 CAN：0x200 9025 编码器、0x307 云台数据。
 * 第三路 CAN 交由达妙电机驱动处理。
 */
volatile int gimbal_can2_num=0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    /* HAL 按DLC复制数据，留足最大帧长度，异常报文也不能覆盖中断栈。 */
    uint8_t rx_data[64];
    volatile can_bus_diag_t *diag;

    if (hfdcan == &hfdcan3)
    {
        dm_motor_can_rx_irq(hfdcan, RxFifo0ITs);
        return;
    }

    if (hfdcan != &hfdcan1 && hfdcan != &hfdcan2)
    {
        return;
    }
    diag = &can_bus_diag[hfdcan == &hfdcan1 ? 0U : 1U];
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        diag->rx_fifo_lost++;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            chassis_can_rx_error_count++;
            diag->rx_read_errors++;
            break;
        }

        /* 在ID分支和协议检查前计数：从硬件收不到帧与解析拒绝是两种问题。 */
        diag->rx_count++;
        diag->last_rx_ms = HAL_GetTick();
        diag->last_rx_id = rx_header.Identifier;
        if (hfdcan == &hfdcan2 && rx_header.IdType == FDCAN_STANDARD_ID &&
            rx_header.Identifier == CAN2_recive_gimbal_msg)
        {
            can2_command_diag.rx_count++;
            can2_command_diag.last_rx_ms = diag->last_rx_ms;
        }
        if (rx_header.IdType != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME ||
            rx_header.FDFormat != FDCAN_CLASSIC_CAN)
        {
            diag->rx_invalid++;
            continue;
        }

        if (hfdcan == &hfdcan1)  /* 底盘 CAN：不走滑环 */
        {
            if (rx_header.Identifier >= CAN1_3508_M1_ID &&
                rx_header.Identifier <= CAN1_3508_M4_ID)
            {
                const uint8_t index = (uint8_t)(rx_header.Identifier - CAN1_3508_M1_ID);
                if (rx_header.DataLength != FDCAN_DLC_BYTES_8)
                {
                    diag->rx_invalid++;
                    continue;
                }
                get_motor_measure(&motor_chassis[index], rx_data);
                chassis_motor_rx_count[index]++;
                chassis_motor_last_rx_ms[index] = diag->last_rx_ms;
                continue;
            }
            switch (rx_header.Identifier)
            {
            case CAN1_recive_cap_msg:
                if (rx_header.DataLength < FDCAN_DLC_BYTES_4)
                {
                    diag->rx_invalid++;
                    break;
                }
                get_cap_measure(&super_cap, rx_data);
                break;
            default:
                break;
            }
        }
        else if (hfdcan == &hfdcan2)  /* 云台 CAN：走滑环 */
        {
            switch (rx_header.Identifier)
            {
                case 0x200:
                    if (rx_header.DataLength < FDCAN_DLC_BYTES_4)
                    {
                        diag->rx_invalid++;
                        break;
                    }
                    get_9025_motor_ecd(&motor_gimbal[0], rx_data);
                    break;
                case CAN2_recive_gimbal_msg:
                    /* 校验：标准帧 + 数据帧 + 8 字节 */
                    if (rx_header.IdType      != FDCAN_STANDARD_ID ||
                        rx_header.RxFrameType != FDCAN_DATA_FRAME  ||
                        rx_header.DataLength  != FDCAN_DLC_BYTES_8)
                    {
                        chassis_top_rx_bad_frame_count++;
                        diag->rx_invalid++;
                        break;
                    }
                    receive_gimbal_chassis_command(rx_data);
                    gimbal_can2_num++;
                    break;
                default:
                    break;
            }
        }
    }
}

/* ---- 发送 ---- */

int can1_chassis_3508_num=0;
/* 发送 4 个底盘电机电流（0x200，大端） */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_can_send_data[0] = (uint8_t)(motor1 >> 8);
    chassis_can_send_data[1] = (uint8_t)motor1;
    chassis_can_send_data[2] = (uint8_t)(motor2 >> 8);
    chassis_can_send_data[3] = (uint8_t)motor2;
    chassis_can_send_data[4] = (uint8_t)(motor3 >> 8);
    chassis_can_send_data[5] = (uint8_t)motor3;
    chassis_can_send_data[6] = (uint8_t)(motor4 >> 8);
    chassis_can_send_data[7] = (uint8_t)motor4;
    can1_chassis_3508_num++;

    fdcan_send(&chassis_can, CAN1_chassis_flag_id, 8, chassis_can_send_data);
}

/* 发送到云台（0x308）。原代码 DLC=0x0A 对经典 CAN 非法（最大 8），已修正为 8 */
void CAN_chassis_send_gimbal(RM_Referee_system_t *rm_data)
{
    chassis_send_gimbal_data[0] = (uint8_t)(rm_data->robot_state.robot_id);
    chassis_send_gimbal_data[1] = (uint8_t)(rm_data->robot_state.robot_level);
    chassis_send_gimbal_data[2] = (uint8_t)(rm_data->robot_state.shooter_barrel_cooling_value);
    chassis_send_gimbal_data[3] = (uint8_t)(rm_data->shoot_data_t.bullet_speed * 10.0f);
    chassis_send_gimbal_data[4] = (uint8_t)(rm_data->robot_state.shooter_barrel_heat_limit >> 8);
    chassis_send_gimbal_data[5] = (uint8_t)(rm_data->robot_state.shooter_barrel_heat_limit);
    chassis_send_gimbal_data[6] = (uint8_t)(rm_data->power_heat_data_t.shooter_17mm_1_barrel_heat >> 8);
    chassis_send_gimbal_data[7] = (uint8_t)(rm_data->power_heat_data_t.shooter_17mm_1_barrel_heat);

    fdcan_send(&gimbal_can, CAN2_send_gimbal_msg, 8, chassis_send_gimbal_data);
}

/* 发送到超电（0x305） */
void CAN_cmd_cap(RM_Referee_system_t *rm_data)
{
    chassis_send_cap_data[0] = (uint8_t)(rm_data->robot_state.chassis_power_limit >> 8);
    chassis_send_cap_data[1] = (uint8_t)(rm_data->robot_state.chassis_power_limit);
    chassis_send_cap_data[2] = (uint8_t)(rm_data->power_heat_data_t.buffer_energy >> 8);
    chassis_send_cap_data[3] = (uint8_t)(rm_data->power_heat_data_t.buffer_energy);
    chassis_send_cap_data[4] = 0;
    chassis_send_cap_data[5] = 0;
    chassis_send_cap_data[6] = 0;
    chassis_send_cap_data[7] = 0;

    fdcan_send(&chassis_can, CAN1_send_cap_msg, 8, chassis_send_cap_data);
}

/*
 * 调试用：给单个电机发送控制电流（0x140 + id）。
 * 原代码混杂了 hcan1 邮箱判断却往 hcan2 发送的 bug，已改为干净地发到云台 CAN。
 */
void CAN_cmd_chassis_read(uint8_t id, int16_t current)
{
    chassis_can_send_data[0] = 0xA1;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = (uint8_t)current;
    chassis_can_send_data[5] = (uint8_t)(current >> 8);
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    fdcan_send(&gimbal_can, (uint32_t)(0x140u + id), 8, chassis_can_send_data);
}

/* 发送底盘 pitch 到云台（0x306，4 字节 fp32） */
void CAN_chassis_ramp_pitch_to_gimbal(fp32 pitch)
{
    memcpy(chassis_send_ramp_pitch, &pitch, sizeof(pitch));
    fdcan_send(&gimbal_can, CAN2_send_gimbal_pitch, 4, chassis_send_ramp_pitch);
}

/* ---- 接收点获取 ---- */

const motor_measure_t *get_motor_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[i & 3];
}

const motor_measure_t *get_motor_gimbal_yaw_measure_point(void)
{
    return &motor_gimbal[0];
}

const super_cap_t *get_super_cap_measure_point(void)
{
    return &super_cap;
}

void read_gimbal_chassis_command(chassis_c_get_data_t *command)
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    uint32_t command_age_ms;

    __disable_irq();
    *command = mini_chassis;
    command_age_ms = HAL_GetTick() - gimbal_chassis_command_received_at_ms;
    __set_PRIMASK(interrupt_mask);

    /* 断流时清空控制量，使麦轮零力并退出爬阶。 */
    if (command_age_ms >= GIMBAL_CHASSIS_COMMAND_TIMEOUT_MS)
    {
        *command = (chassis_c_get_data_t){0};
    }
}
