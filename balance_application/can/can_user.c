#include "can_user.h"
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

/* 云台板 0x307 数据解析（遥控量 + 各种模式位） */
#define get_gimbal_measure(ptr, data)                                   \
    {                                                                   \
        (ptr)->x_rc_data       = ((data)[0] << 8) | ((data)[1]);        \
        (ptr)->y_rc_data       = ((data)[2] << 8) | ((data)[3]);        \
        (ptr)->x_key_data      = ((data)[4] >> 6) & 3;                  \
        (ptr)->y_key_data      = ((data)[4] >> 4) & 3;                  \
        (ptr)->chassis_mode    = ((data)[5] >> 6) & 3;                  \
        (ptr)->cap_mode        = ((data)[5] >> 5) & 1;                  \
        (ptr)->top_mode        = ((data)[5] >> 4) & 1;                  \
        (ptr)->follow_mode     = ((data)[5] >> 3) & 1;                  \
        (ptr)->follow_mode_last= ((data)[5] >> 2) & 1;                  \
        (ptr)->key_control     = ((data)[5] >> 1) & 1;                  \
    }

/* 超电 0x304 电容电压解析 */
#define get_cap_measure(ptr, data)                                      \
    {                                                                   \
        (ptr)->V_cap_value = (fp32)(((data)[2] << 8) | ((data)[3])) * 0.001f; \
    }

motor_measure_t motor_gimbal[1];
motor_measure_t motor_chassis[4];

/* Keil Watch 诊断计数器：区分收到的 OFF 帧与本地模式变化 */
volatile uint32_t chassis_top_rx_on_count      = 0;
volatile uint32_t chassis_top_rx_off_count     = 0;
volatile uint32_t chassis_top_rx_bad_frame_count = 0;
volatile uint32_t chassis_can_rx_error_count   = 0;
volatile uint8_t  chassis_top_rx_last_off_flags = 0;

chassis_c_get_data_t mini_chassis =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

super_cap_t super_cap;

/* 发送缓冲 */
static uint8_t chassis_can_send_data[8];
static uint8_t chassis_send_gimbal_data[8];
static uint8_t chassis_send_cap_data[8];
static uint8_t chassis_send_ramp_pitch[4];

/* 云台 CAN 接收帧计数（调试用） */
int from_yuntai_can = 0;

/* 9025 电机（云台）编码器解析：只取机械角 */
void get_9025_motor_ecd(motor_measure_t *ptr, uint8_t *data)
{
    ptr->ecd = (uint16_t)((data[2] << 8) | (data[3]));
}

/* FDCAN 发送：经典 CAN 数据帧（1Mbps，BRS 关闭） */
static void fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t dlc_bytes, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef tx = {0};

    tx.Identifier          = std_id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)dlc_bytes;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0u)
    {
        (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data);
    }
}

/*
 * FDCAN RX FIFO0 回调（hfdcan1 / hfdcan2 共用）。
 * 底盘 CAN：0x201~0x204 电机反馈、0x304 超电；
 * 云台 CAN：0x200 9025 编码器、0x307 云台数据。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            chassis_can_rx_error_count++;
            break;
        }

        if (hfdcan == &hfdcan1)  /* 底盘 CAN：不走滑环 */
        {
            switch (rx_header.Identifier)
            {
            case CAN1_3508_M1_ID:
                get_motor_measure(&motor_chassis[0], rx_data);
                break;
            case CAN1_3508_M2_ID:
                get_motor_measure(&motor_chassis[1], rx_data);
                break;
            case CAN1_3508_M3_ID:
                get_motor_measure(&motor_chassis[2], rx_data);
                break;
            case CAN1_3508_M4_ID:
                get_motor_measure(&motor_chassis[3], rx_data);
                break;
            case CAN1_recive_cap_msg:
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
                get_9025_motor_ecd(&motor_gimbal[0], rx_data);
                break;
            case CAN2_recive_gimbal_msg:
                /* 校验：标准帧 + 数据帧 + 8 字节 */
                if (rx_header.IdType      != FDCAN_STANDARD_ID ||
                    rx_header.RxFrameType != FDCAN_DATA_FRAME  ||
                    rx_header.DataLength  != FDCAN_DLC_BYTES_8)
                {
                    chassis_top_rx_bad_frame_count++;
                    break;
                }
                if ((rx_data[5] & 0x10U) != 0U)
                {
                    chassis_top_rx_on_count++;
                }
                else
                {
                    chassis_top_rx_off_count++;
                    chassis_top_rx_last_off_flags = rx_data[5];
                }
                from_yuntai_can++;
                get_gimbal_measure(&mini_chassis, rx_data);
                break;
            default:
                break;
            }
        }
    }
}

/* ---- 发送 ---- */

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

const chassis_c_get_data_t *get_c_gimbal_data_point(void)
{
    return &mini_chassis;
}
