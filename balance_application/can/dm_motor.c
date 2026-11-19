#include "dm_motor.h"

#include <math.h>

//dm电机的id和3508麦轮的id是一一对应的
/*
    id=2----id=1
      |       |
      |       |
    id=3----id=4

*/

/* 点动和遥控共用发送周期；以下固定速度和增益用于 Watch 点动测试。 */
#define DM_TEST_ALL_MOTORS ((1U << DM_MOTOR_COUNT) - 1U)
#define DM_TEST_PERIOD_MS 10U
#define DM_TEST_FEEDBACK_MS 50U
#define DM_TEST_POLL_FRESH_MS 120U
#define DM_TEST_ENABLE_MS 150U
#define DM_TEST_DISABLE_MS 150U
#define DM_TEST_DURATION_MS 5000U
/* 点动输入的偏移上限，以及点动期间的实测位移保护。 */
#define DM_TEST_OFFSET_RAD 1.0f
#define DM_TEST_TRAVEL_RAD 0.18f
#define DM_TEST_RAMP_RAD_S 0.40f    // 速度变化率，rad/s
#define DM_TEST_KP 10.0f
#define DM_TEST_KD 0.25f
/* 遥控推杆时限制目标角与反馈角的差，避免卡住后持续累积误差。 */
#define DM_REMOTE_POSITION_ERROR_RAD 0.20f

/* 重力前馈：力臂用关节轴到触地点的水平投影，不是腿长本身，也不是抬起高度。
 * 前腿皮带按 5% 效率损耗估算；关节角超过约 86° 视为腿已接近竖直、不再支撑。 */
#define DM_GRAVITY_G 9.81f
#define DM_GRAVITY_BELT_EFFICIENCY 0.95f
#define DM_GRAVITY_JOINT_ANGLE_MAX 1.50f
/* 电机使能后前馈力矩的爬升时间，避免使能瞬间产生力矩阶跃。 */
#define DM_GRAVITY_RAMP_MS 300U

/* 遥控机械臂在线调参，四台电机共用。
 * Keil Watch 中修改这些变量即可生效；调好后把记录值写回下面的初始值。
 * Kp、Kd 的协议范围分别为 0～500、0～5；速度使用非负值，单位 rad/s。
 */
volatile float dm_remote_kp = 20.0f;             /* 位置增益，影响跟随力度。 */
volatile float dm_remote_kd = 1.0f;             /* 阻尼增益，影响运动阻尼。 */
volatile float dm_remote_speed_rad_s = 0.50f;    /* 满杆时目标角的变化速度。 */

/* 重力前馈在线调参。
 * enable 默认 1：调试期间保持开启，持位的未选中电机才不会因缺少前馈而下沉。
 * load_kg 是每条腿实际承担的整机重量；总重 20 kg、四腿均匀时为 5.0。
 * 只有前后某一组着地时，该组两条腿要承担更多重量，需要相应加大。
 * scale 是总比例，调试时先给 0.2～0.3 看方向，正常用 1.0。
 */
volatile uint8_t dm_gravity_enable = 1U;
volatile float dm_gravity_scale = 1.0f;
volatile float dm_gravity_load_kg[DM_MOTOR_COUNT] = {5.0f, 5.0f, 5.0f, 5.0f};
volatile float dm_gravity_torque_out[DM_MOTOR_COUNT];

typedef struct
{
    uint16_t can_id;
    uint16_t master_id;
    /* 协议映射半量程。位置 rad，速度 rad/s，扭矩 N·m。 */
    float pmax;
    float vmax;
    float tmax;
    float extension_direction;
    /* 重力前馈用的机械参数。 */
    float reduction;        /* 电机转数 / 关节转数：前腿皮带 1.75，后腿直驱 1.0 */
    float leg_length_m;     /* 关节轴到触地点的距离，单位 m */
    float gravity_sign;     /* 电机正转抬升机体填 +1，反转抬升填 -1 */
} dm_config_t;

/* 下标 0、1 对应前面的两台 4340（ID 1、2），下标 2、3 对应后面的两台 8009（ID 3、4）。
 * pmax、vmax、tmax 必须分别与达妙助手读出的 PMAX、VMAX、TMAX 一致。
 * 最后一列将遥控正向请求换算成角度增量：ID 1、3 为 -1，ID 2、4 为 +1。
 * gravity_sign 先按“摇杆正向 = 抬升”填成与 extension_direction 相同，
 * 上电实测后若某条腿的方向反了，只改该行的 gravity_sign。
 */
static const dm_config_t dm_config[DM_MOTOR_COUNT] =
{
    /* CAN ID  反馈 ID    PMAX       VMAX   TMAX   正向角度增量符号  减速比   腿长    重力符号 */
    {0x01U,    0x11U,    12.5f,     20.0f, 28.0f, -1.0f,            1.75f,  0.20f, -1.0f}, /* 前 4340，ID 1 */
    {0x02U,    0x12U,    12.5f,     20.0f, 28.0f,  1.0f,            1.75f,  0.20f,  1.0f}, /* 前 4340，ID 2 */
    {0x03U,    0x13U,    3.141549f, 45.0f, 54.0f, -1.0f,            1.00f,  0.23f, -1.0f}, /* 后 8009，ID 3 */
    {0x04U,    0x14U,    3.141549f, 45.0f, 54.0f,  1.0f,            1.00f,  0.23f,  1.0f}, /* 后 8009，ID 4 */
};

typedef struct
{
    float a_rad;
    float b_rad;
    float c_rad;
    float d_rad;
} dm_angle_limit_t;

/* 每行对应一台电机，直接使用驱动器原始位置，单位为 rad，不做角度归一化。
 * 要求 -pmax <= a < b < c < d <= pmax，范围按该电机保存零点后的实测角度设置。
 * [a, b] 只允许角度增大，(b, c) 允许双向，[c, d] 只允许角度减小。
 * b、c 是各电机的双向活动区边界，到达边界后只允许朝区间内返回。
 * a、d 取该电机的 -pmax、pmax，使其余可反馈角度范围均可单向返回。
 * [a, d] 之外保持目标角；这些区间按电机反馈角的正负方向定义。
 */
static const dm_angle_limit_t dm_angle_limit[DM_MOTOR_COUNT] =
{
    /*       a        b      c       d       */
    {-12.5f,     -4.0f,  1.0f, 12.5f},     /* 下标 0，前 4340，ID 1 */
    {-12.5f,     -1.0f,  4.0f, 12.5f},     /* 下标 1，前 4340，ID 2 */
    {-3.141549f, -1.1f,  0.0f,  3.141549f}, /* 下标 2，后 8009，ID 3 */
    {-3.141549f,  0.0f,  1.1f,  3.141549f}, /* 下标 3，后 8009，ID 4 */
};

volatile dm_test_t dm_test;
volatile dm_feedback_t dm_feedback[DM_MOTOR_COUNT];
volatile dm_can_diag_t dm_can_diag;
volatile uint8_t dm_remote_active_mask;

static uint32_t phase_start_ms;
static uint32_t last_step_ms;
static uint8_t next_poll_index;
static uint32_t feedback_baseline[DM_MOTOR_COUNT];
static float initial_angle[DM_MOTOR_COUNT];
static float requested_offset[DM_MOTOR_COUNT];
/* 每轴单独保存遥控目标角及失能前的接收计数，用新反馈确认失能。 */
static float remote_target_position[DM_MOTOR_COUNT];
static uint8_t remote_disable_sent_mask;
/* 上一周期真正推进过目标角的电机位，用于区分重新推进的首拍。 */
static uint8_t remote_driven_last;
static uint32_t remote_stop_feedback_count[DM_MOTOR_COUNT];
/* 重力前馈的渐入系数，0→1 对应使能后的爬升过程。 */
static float gravity_ramp[DM_MOTOR_COUNT];

/* 配置 CAN3 接收过滤器、通知和启动状态，用于电机通信。 */
HAL_StatusTypeDef dm_motor_can_start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    const uint32_t interrupts = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_BUS_OFF;

    /* Master ID 是驱动器的反馈帧 ID。配置表按连续 ID 排列。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = dm_config[0].master_id;
    filter.FilterID2 = dm_config[DM_MOTOR_COUNT - 1U].master_id;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan3, interrupts, 0U) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dm_can_diag.started = 1U;
    return HAL_OK;
}

/* 统一从 CAN3 发经典 CAN 帧。HAL_OK 表示报文已进入发送队列。 */
static HAL_StatusTypeDef dm_send(uint16_t id, uint32_t dlc, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (dm_can_diag.started == 0U || dm_can_diag.bus_off != 0U)
    {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
    {
        dm_can_diag.tx_fifo_full++;
        return HAL_BUSY;
    }

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &header, data) != HAL_OK)
    {
        dm_can_diag.tx_hal_error++;
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* 按电机索引发送达妙模式命令；末字节由调用方指定。 */
static HAL_StatusTypeDef dm_send_mode_command(uint8_t index, uint8_t command)
{
    const uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};

    if (index >= DM_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }
    return dm_send(dm_config[index].can_id, FDCAN_DLC_BYTES_8, data);
}

/* 发送指定电机的使能命令。 */
HAL_StatusTypeDef dm_motor_enable(uint8_t index)
{
    return dm_send_mode_command(index, 0xFCU);
}

/* 发送指定电机的失能命令。 */
HAL_StatusTypeDef dm_motor_disable(uint8_t index)
{
    return dm_send_mode_command(index, 0xFDU);
}

/* 状态查询发给 0x7FF，前两字节带目标电机 ID，失能时也能查询。 */
HAL_StatusTypeDef dm_motor_request_feedback(uint8_t index)
{
    uint8_t data[4] = {0U, 0U, 0xCCU, 0U};
    HAL_StatusTypeDef status;

    if (index >= DM_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }

    data[0] = (uint8_t)dm_config[index].can_id;
    data[1] = (uint8_t)(dm_config[index].can_id >> 8);
    status = dm_send(0x7FFU, FDCAN_DLC_BYTES_4, data);
    if (status == HAL_OK)
    {
        dm_can_diag.tx_status_queued[index]++;
    }
    return status;
}

/* 将浮点物理量限幅并线性转换为协议使用的无符号定点数。 */
static uint16_t dm_encode(float value, float min, float max, uint16_t code_max)
{
    if (value <= min)
    {
        return 0U;
    }
    if (value >= max)
    {
        return code_max;
    }
    return (uint16_t)((value - min) * (float)code_max / (max - min));
}

/* 将协议定点数解码为以 0 为中心、范围为 ±max 的物理量。 */
static float dm_decode(uint16_t code, float max, uint16_t code_max)
{
    return (float)code * (2.0f * max / (float)code_max) - max;
}

/* 编码并发送指定电机的 MIT 位置、速度、增益和力矩控制帧。 */
HAL_StatusTypeDef dm_motor_send_mit(uint8_t index, float position, float velocity,
                                   float kp, float kd, float torque)
{
    uint16_t p, v, kp_code, kd_code, t;
    uint8_t data[8];
    HAL_StatusTypeDef status;
    const dm_config_t *config;

    if (index >= DM_MOTOR_COUNT || !isfinite(position) || !isfinite(velocity) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(torque))
    {
        return HAL_ERROR;
    }

    config = &dm_config[index];
    p = dm_encode(position, -config->pmax, config->pmax, 0xFFFFU);
    v = dm_encode(velocity, -config->vmax, config->vmax, 0xFFFU);
    kp_code = dm_encode(kp, 0.0f, 500.0f, 0xFFFU);
    kd_code = dm_encode(kd, 0.0f, 5.0f, 0xFFFU);
    t = dm_encode(torque, -config->tmax, config->tmax, 0xFFFU);

    /* Official DaMiao MIT layout: P16 | V12 | Kp12 | Kd12 | T12. */
    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0xFU) << 4) | (kp_code >> 8));
    data[4] = (uint8_t)kp_code;
    data[5] = (uint8_t)(kd_code >> 4);
    data[6] = (uint8_t)(((kd_code & 0xFU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    status = dm_send(config->can_id, FDCAN_DLC_BYTES_8, data);
    if (status == HAL_OK)
    {
        dm_can_diag.tx_mit_queued[index]++;
    }
    return status;
}

/* 读取 CAN3 接收 FIFO 中的反馈帧，校验后更新对应电机的反馈快照。 */
void dm_motor_can_rx_irq(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64]; /* HAL copies the received DLC before application validation. */

    if ((rx_interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        dm_can_diag.rx_fifo_lost_events++;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
    {
        uint32_t index, now, gap;
        const dm_config_t *config;
        volatile dm_feedback_t *feedback;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            dm_can_diag.rx_read_errors++;
            break;
        }
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8 ||
            header.Identifier < dm_config[0].master_id ||
            header.Identifier > dm_config[DM_MOTOR_COUNT - 1U].master_id)
        {
            dm_can_diag.rx_invalid++;
            continue;
        }

        /* 用连续的反馈 ID 找到下标，再校验帧内的电机编号。 */
        index = header.Identifier - dm_config[0].master_id;
        config = &dm_config[index];
        if ((data[0] & 0xFU) != config->can_id)
        {
            dm_can_diag.rx_invalid++;
            continue;
        }

        feedback = &dm_feedback[index];
        now = HAL_GetTick();
        gap = now - feedback->last_rx_ms;
        if (dm_test.state == DM_TEST_MOVING && (dm_test.active_mask & (1U << index)) != 0U &&
            gap > feedback->max_move_rx_gap_ms)
        {
            feedback->max_move_rx_gap_ms = gap;
        }
        /* 位置占 16 位，速度和扭矩各占 12 位。状态在 data[0] 的高四位。 */
        feedback->position_rad = dm_decode((uint16_t)((data[1] << 8) | data[2]), config->pmax, 0xFFFFU);
        feedback->velocity_rad_s = dm_decode((uint16_t)((data[3] << 4) | (data[4] >> 4)), config->vmax, 0xFFFU);
        feedback->torque_nm = dm_decode((uint16_t)(((data[4] & 0xFU) << 8) | data[5]), config->tmax, 0xFFFU);
        feedback->state = data[0] >> 4;
        feedback->mos_temp_c = data[6];
        feedback->coil_temp_c = data[7];
        feedback->last_rx_ms = now;
        feedback->rx_count++;
    }
}

/* 处理 CAN 协议仲裁错误，累计错误诊断并记录最近的协议错误类型。 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef status;

    if (hfdcan == &hfdcan3 && (hfdcan->ErrorCode & HAL_FDCAN_ERROR_PROTOCOL_ARBT) != 0U)
    {
        /* HAL keeps this bit set across IRQs until software clears it. */
        hfdcan->ErrorCode &= ~HAL_FDCAN_ERROR_PROTOCOL_ARBT;
        HAL_FDCAN_GetProtocolStatus(hfdcan, &status);
        dm_can_diag.protocol_error_events++;
        dm_can_diag.last_error_code = status.LastErrorCode;
        if (status.LastErrorCode == FDCAN_PROTOCOL_ERROR_ACK)
        {
            dm_can_diag.ack_error_events++;
        }
    }
}

/* 处理 CAN 错误状态中断，并记录 CAN3 的 Bus-Off 状态。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    if (hfdcan == &hfdcan3 && (interrupts & FDCAN_IT_BUS_OFF) != 0U)
    {
        dm_can_diag.bus_off = 1U;
    }
}

/* 轮询待查询电机；状态请求使用 0x7FF，每次调用最多发送一帧。 */
static void dm_poll_one(uint32_t mask)
{
    uint8_t scanned;

    /* 从上次轮询位置开始扫描一圈，避免总是优先查询同一台电机。 */
    for (scanned = 0U; scanned < DM_MOTOR_COUNT; scanned++)
    {
        const uint8_t index = next_poll_index;
        /* 先将游标移到下一台；到达末尾后回到 0。 */
        next_poll_index = (uint8_t)((index + 1U) % DM_MOTOR_COUNT);
        /* mask 中置位的电机才需要请求反馈。 */
        if ((mask & (1U << index)) != 0U)
        {
            /* 发送一个反馈请求后立即返回；后续请求留到下一次调用。 */
            (void)dm_motor_request_feedback(index);
            return;
        }
    }
    /* 若 mask 中没有待查询电机，本次扫描不会发送请求。 */
}

/* 停止 Watch 点动，取消排队的运动帧并对已激活电机发起失能流程。 */
static void dm_test_stop(dm_test_stop_t reason, uint32_t mask, uint32_t now,
                         const dm_feedback_t *snapshot)
{
    uint8_t index;

    dm_test.run = 0U;
    dm_test.stop_reason = reason;
    dm_test.fault_mask = mask;
    dm_test.disable_pending_mask = dm_test.active_mask;
    dm_test.state = DM_TEST_DISABLING;
    phase_start_ms = now;

    /* Drop any pending motion commands before sending disable. */
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan3, hfdcan3.Instance->TXBRP);
    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        feedback_baseline[index] = snapshot[index].rx_count;
        if ((dm_test.active_mask & (1U << index)) != 0U)
        {
            (void)dm_motor_disable(index);
        }
    }
}

/* 在空闲态检查点动条件，并锁存参与电机的起始角度和目标偏移。 */
static void dm_test_idle(uint32_t mask, uint32_t now, const dm_feedback_t *snapshot)
{
    uint8_t index;
    uint32_t not_ready = 0U;

    if (dm_test.run != 0U)
    {
        for (index = 0U; index < DM_MOTOR_COUNT; index++)
        {
            float offset;

            if ((mask & (1U << index)) == 0U)
            {
                continue;
            }
            offset = dm_test.offset_rad[index];
            if (snapshot[index].rx_count == 0U || now - snapshot[index].last_rx_ms >= DM_TEST_POLL_FRESH_MS ||
                snapshot[index].state != 0U || !isfinite(offset) || fabsf(offset) > DM_TEST_OFFSET_RAD ||
                fabsf(snapshot[index].position_rad) + DM_TEST_TRAVEL_RAD >= dm_config[index].pmax)
            {
                not_ready |= 1U << index;
            }
            initial_angle[index] = snapshot[index].position_rad;
            requested_offset[index] = offset;
            feedback_baseline[index] = snapshot[index].rx_count;
            dm_feedback[index].max_move_rx_gap_ms = 0U;
        }

        if (mask != 0U && not_ready == 0U && dm_test.disable_pending_mask == 0U)
        {
            dm_test.stop_reason = DM_TEST_NO_STOP;
            dm_test.fault_mask = 0U;
            dm_test.active_mask = mask;
            dm_test.state = DM_TEST_ENABLING;
            phase_start_ms = now;
            for (index = 0U; index < DM_MOTOR_COUNT; index++)
            {
                if ((mask & (1U << index)) != 0U && dm_motor_enable(index) != HAL_OK)
                {
                    dm_test_stop(DM_TEST_STOP_TX_FAILED, 1U << index, now, snapshot);
                    break;
                }
            }
            return;
        }

        dm_test.run = 0U;
        dm_test.stop_reason = DM_TEST_STOP_NOT_READY;
        dm_test.fault_mask = not_ready | dm_test.disable_pending_mask;
    }

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint32_t bit = 1U << index;

        if ((dm_test.disable_pending_mask & bit) != 0U &&
            snapshot[index].rx_count != feedback_baseline[index] && snapshot[index].state == 0U &&
            now - snapshot[index].last_rx_ms < DM_TEST_POLL_FRESH_MS)
        {
            dm_test.disable_pending_mask &= ~bit;
        }
    }
    /* Leave unconfirmed drives silent so their own CAN timeout can expire. */
    dm_poll_one(mask & ~dm_test.disable_pending_mask);
}

static void dm_test_move_step(uint32_t now, const dm_feedback_t *snapshot);

/* 等待所有参与电机返回新的使能反馈，超时或异常时停止测试。 */
static void dm_test_enable_step(uint32_t now, const dm_feedback_t *snapshot)
{
    uint8_t index;
    uint32_t awaiting = 0U;

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint32_t bit = 1U << index;

        if ((dm_test.active_mask & bit) == 0U)
        {
            continue;
        }
        if (snapshot[index].state > 1U)
        {
            dm_test_stop(DM_TEST_STOP_MOTOR_STATE, bit, now, snapshot);
            return;
        }
        if (snapshot[index].rx_count == feedback_baseline[index] || snapshot[index].state != 1U ||
            now - snapshot[index].last_rx_ms >= DM_TEST_POLL_FRESH_MS)
        {
            awaiting |= bit;
        }
    }
    /* 所有参与轴都返回新的使能反馈后，才开始发 MIT 位置指令。 */
    if (awaiting == 0U)
    {
        phase_start_ms = now;
        dm_test.state = DM_TEST_MOVING;
        dm_test_move_step(now, snapshot);
        return;
    }
    if (now - phase_start_ms >= DM_TEST_ENABLE_MS)
    {
        dm_test_stop(DM_TEST_STOP_FEEDBACK_TIMEOUT, awaiting, now, snapshot);
        return;
    }
    dm_poll_one(awaiting);
}

/* 执行点动位置控制，并监测反馈超时、电机状态和行程限制。 */
static void dm_test_move_step(uint32_t now, const dm_feedback_t *snapshot)
{
    uint8_t index;
    float ramp = (float)(now - phase_start_ms) * (DM_TEST_RAMP_RAD_S / 1000.0f);

    if (now - phase_start_ms >= DM_TEST_DURATION_MS)
    {
        dm_test_stop(DM_TEST_STOP_TIME_LIMIT, 0U, now, snapshot);
        return;
    }

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint32_t bit = 1U << index;

        if ((dm_test.active_mask & bit) == 0U)
        {
            continue;
        }
        if (now - snapshot[index].last_rx_ms >= DM_TEST_FEEDBACK_MS)
        {
            dm_test_stop(DM_TEST_STOP_FEEDBACK_TIMEOUT, bit, now, snapshot);
            return;
        }
        if (snapshot[index].state != 1U)
        {
            dm_test_stop(DM_TEST_STOP_MOTOR_STATE, bit, now, snapshot);
            return;
        }
        if (fabsf(snapshot[index].position_rad - initial_angle[index]) > DM_TEST_TRAVEL_RAD)
        {
            dm_test_stop(DM_TEST_STOP_POSITION, bit, now, snapshot);
            return;
        }
    }

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint32_t bit = 1U << index;
        const float offset = fminf(ramp, fmaxf(-ramp, requested_offset[index]));

        if ((dm_test.active_mask & bit) != 0U &&
            dm_motor_send_mit(index, initial_angle[index] + offset, 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f) != HAL_OK)
        {
            dm_test_stop(DM_TEST_STOP_TX_FAILED, bit, now, snapshot);
            return;
        }
    }
}

/* 重发失能命令并等待确认反馈，随后结束点动的失能阶段。 */
static void dm_test_disable_step(uint32_t now, const dm_feedback_t *snapshot)
{
    uint8_t index;

    dm_test.run = 0U;
    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint32_t bit = 1U << index;

        if ((dm_test.disable_pending_mask & bit) == 0U)
        {
            continue;
        }
        if (snapshot[index].rx_count != feedback_baseline[index] && snapshot[index].state == 0U &&
            now - snapshot[index].last_rx_ms < DM_TEST_POLL_FRESH_MS)
        {
            dm_test.disable_pending_mask &= ~bit;
        }
        else if (now - phase_start_ms < DM_TEST_DISABLE_MS)
        {
            (void)dm_motor_disable(index);
        }
    }
    if (now - phase_start_ms < DM_TEST_DISABLE_MS)
    {
        dm_poll_one(dm_test.disable_pending_mask);
    }
    if (dm_test.disable_pending_mask == 0U || now - phase_start_ms >= DM_TEST_DISABLE_MS)
    {
        dm_test.active_mask = 0U;
        dm_test.state = DM_TEST_IDLE;
    }
}
/* 计算一台电机的重力前馈力矩，返回值是电机侧力矩，单位 N·m。
 * motor_angle_rad 使用该轴的目标角（电机原始坐标，rad）。
 *
 * 把电机角换成关节角：theta = gravity_sign * motor_angle / reduction。
 * theta 从 0 增大表示腿往下压、把机体抬高：
 *   竖直高度 h = L * sin(theta)
 *   水平力臂 x = L * cos(theta)
 *   关节保持力矩 = load_kg * g * x
 *   电机侧力矩   = 关节力矩 / (reduction * 皮带效率)
 * theta <= 0（腿在水平面以上）或接近竖直时不承担机体重量，返回 0。
 */
static float dm_gravity_torque(uint8_t index, float motor_angle_rad)
{
    const dm_config_t *config;
    float joint_angle;
    float torque;

    if (dm_gravity_enable == 0U || index >= DM_MOTOR_COUNT)
    {
        if (index < DM_MOTOR_COUNT)
        {
            dm_gravity_torque_out[index] = 0.0f;
        }
        return 0.0f;
    }

    config = &dm_config[index];
    joint_angle = config->gravity_sign * motor_angle_rad / config->reduction;
    if (joint_angle <= 0.0f || joint_angle >= DM_GRAVITY_JOINT_ANGLE_MAX)
    {
        dm_gravity_torque_out[index] = 0.0f;
        return 0.0f;
    }

    torque = dm_gravity_load_kg[index] * DM_GRAVITY_G *
             config->leg_length_m * cosf(joint_angle);
    torque /= config->reduction * DM_GRAVITY_BELT_EFFICIENCY;
    /* 前馈力矩与“抬升”同向；反装电机的 gravity_sign 会把方向一起翻过来。 */
    torque = config->gravity_sign * torque * dm_gravity_scale;
    dm_gravity_torque_out[index] = torque;
    return torque;
}

/* 停止单台遥控电机，并根据新的失能反馈清除其活动标记。 */
static void stop_remote_motor(uint8_t index, bool feedback_is_fresh,
                              const dm_feedback_t *feedback)
{
    const uint8_t motor_bit = 1U << index;

    if ((dm_remote_active_mask & motor_bit) == 0U)
    {
        return;
    }

    /* 等失能发出后的新状态 0，确认后再释放该轴的控制位。 */
    if ((remote_disable_sent_mask & motor_bit) != 0U)
    {
        if (feedback_is_fresh && feedback->state == 0U &&
            feedback->rx_count != remote_stop_feedback_count[index])
        {
            dm_remote_active_mask &= ~motor_bit;
            remote_disable_sent_mask &= ~motor_bit;
        }
        else if (feedback_is_fresh)
        {
            (void)dm_motor_disable(index);
        }
    }
    else if (dm_motor_disable(index) == HAL_OK)
    {
        remote_stop_feedback_count[index] = feedback->rx_count;
        remote_disable_sent_mask |= motor_bit;
    }
}

/* 根据遥控使能集合和选择，更新目标角并控制电机。
 * enabled_motors：本轮保持使能的电机位（左拨杆下的爬阶档为四台）。
 * selected_motors：本轮允许遥控器在目标角上加减的电机位，是 enabled_motors 的子集。
 * 未选中但仍在使能集合内的电机：目标角冻结，持续下发 MIT 帧持位。 */
static void control_remote_motors(uint8_t enabled_motors, uint8_t selected_motors,
                                  int8_t joint_motion_request, uint32_t now,
                                  const dm_feedback_t *feedback)
{
    /* 将摇杆请求（-100～100）换算为本控制周期内的角度增量。 */
    const float motion_fraction = (float)joint_motion_request / 100.0f;
    /* 按配置的角度变化速率和控制周期计算目标角增量。 */
    const float position_step = motion_fraction * dm_remote_speed_rad_s * (DM_TEST_PERIOD_MS / 1000.0f);
    /* 汇总本轮需要请求反馈的电机，循环结束后统一轮询。 */
    uint8_t feedback_request_mask = 0U;
    uint8_t index;
    uint8_t driven_mask = 0U;

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint8_t motor_bit = 1U << index;
        /* 是否在本轮使能集合内。 */
        const bool enabled = (enabled_motors & motor_bit) != 0U;
        /* 是否被遥控选中（允许动目标角）。 */
        const bool selected = (selected_motors & motor_bit) != 0U;
        /* 上一周期是否真正推进过目标角（重新推进的首拍不做相对反馈限幅）。 */
        const bool was_driven = (remote_driven_last & motor_bit) != 0U;
        /* 必须已收到过反馈，且反馈未超过有效期。 */
        const bool feedback_is_fresh = feedback[index].rx_count != 0U &&
            now - feedback[index].last_rx_ms < DM_TEST_FEEDBACK_MS;

        /* 不在使能集合内：走失能流程（退出爬阶或该轴被剔除）。 */
        if (!enabled)
        {
            stop_remote_motor(index, feedback_is_fresh, &feedback[index]);
            /* 仍在等待失能确认的电机需要轮询反馈，确认真实 state==0，避免掩码卡死。 */
            if ((dm_remote_active_mask & motor_bit) != 0U)
            {
                feedback_request_mask |= motor_bit;
            }
            continue;
        }

        /* 使能集合内：取消仍在等待确认的失能。若电机实际已被失能，下面使能边沿
         * 会按反馈 state==0 重新使能并重新对齐目标；若仍处于使能态，则直接沿用
         * 冻结目标继续下发 MIT，避免快速切换拨杆时卡死在待失能状态。 */
        remote_disable_sent_mask &= ~motor_bit;

        /* 使能集合内，但反馈失效或状态异常：不发位置帧，只请求反馈。 */
        if (!feedback_is_fresh || feedback[index].state > 1U)
        {
            feedback_request_mask |= motor_bit;
            continue;
        }

        if ((dm_remote_active_mask & motor_bit) == 0U || feedback[index].state == 0U)
        {
            /* 软件尚未激活或反馈显示电机未使能：用实际位置建立目标，避免使能后跳变。
             * 首次上电还没有遥控时，目标与实际位置自然重合。 */
            remote_target_position[index] = feedback[index].position_rad;
            /* 重新使能时重力前馈从 0 重新爬升。 */
            gravity_ramp[index] = 0.0f;
            if (dm_motor_enable(index) == HAL_OK)
            {
                dm_remote_active_mask |= motor_bit;
            }
            /* 使能后先请求一次反馈，本周期不继续发送位置命令。 */
            feedback_request_mask |= motor_bit;
            continue;
        }

        /* 只有被选中的电机才允许遥控器在目标角上直接加减；
         * 未选中的电机保持 remote_target_position 不变，即持位。 */
        if (selected && joint_motion_request != 0)
        {
            /* 按电机安装方向修正增量，使摇杆方向与关节伸缩方向一致。 */
            const float motor_position_step = position_step * dm_config[index].extension_direction;
            const float current_position = feedback[index].position_rad;
            const dm_angle_limit_t *limit = &dm_angle_limit[index];
            /* 直接比较原始反馈角，保留超过 ±π 时的位置和运动方向。 */
            const bool positive_allowed = current_position >= limit->a_rad && current_position < limit->c_rad;
            const bool negative_allowed = current_position > limit->b_rad && current_position <= limit->d_rad;

            /* 只在允许的方向更新目标；受限方向沿用上一周期的目标角。 */
            if ((motor_position_step > 0.0f && positive_allowed) ||
                (motor_position_step < 0.0f && negative_allowed))
            {
                const float target = remote_target_position[index] + motor_position_step;

                if (was_driven)
                {
                    /* 连续推进：相对反馈角限幅，避免卡死时误差无限累积。 */
                    remote_target_position[index] = fminf(current_position + DM_REMOTE_POSITION_ERROR_RAD,
                                                           fmaxf(current_position - DM_REMOTE_POSITION_ERROR_RAD, target));
                }
                else
                {
                    /* 重新推进的首拍：从冻结目标直接加减，不做相对反馈的限幅，
                     * 避免持位期间的少量下垂把目标拉回反馈角。 */
                    remote_target_position[index] = target;
                }
                driven_mask |= motor_bit;
            }
        }
        /* 摇杆回中或未选中时保持目标不变；速度设为 0，靠 MIT 的 kp/kd 持位。
         * 重力前馈按目标角计算，并乘一个缓慢爬升的系数，避免使能瞬间力矩阶跃。
         * 关闭前馈时把爬升系数清零，这样在 Watch 里临时打开也会从 0 渐入。 */
        if (dm_gravity_enable == 0U)
        {
            gravity_ramp[index] = 0.0f;
        }
        else if (gravity_ramp[index] < 1.0f)
        {
            gravity_ramp[index] = fminf(1.0f, gravity_ramp[index] +
                                        (float)DM_TEST_PERIOD_MS / (float)DM_GRAVITY_RAMP_MS);
        }
        /* 所有使能的电机（含未选中持位的）都持续下发 MIT 帧，
         * 让驱动器内部位置环持续锁位，并避免通信超时自行失能。 */
        (void)dm_motor_send_mit(index, remote_target_position[index], 0.0f,
                                dm_remote_kp, dm_remote_kd,
                                dm_gravity_torque(index, remote_target_position[index]) *
                                gravity_ramp[index]);
    }

    /* 记录本轮真正推进过目标的电机，供下一周期判断重新推进的首拍。 */
    remote_driven_last = driven_mask;

    /* 统一请求本轮需要更新的反馈。 */
    dm_poll_one(feedback_request_mask);
}

/* 周期性更新 CAN 诊断和反馈快照，并调度遥控或点动状态机。 */
void dm_motor_control_step(bool use_remote_control, uint8_t enabled_motors,
                           uint8_t selected_motors, int8_t joint_motion_request)
{
    dm_feedback_t snapshot[DM_MOTOR_COUNT];
    FDCAN_ErrorCountersTypeDef errors;
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    uint8_t index;

    if (dm_can_diag.started == 0U || now - last_step_ms < DM_TEST_PERIOD_MS)
    {
        return;
    }
    last_step_ms = now;
    HAL_FDCAN_GetErrorCounters(&hfdcan3, &errors);
    dm_can_diag.tx_error_counter = errors.TxErrorCnt;
    dm_can_diag.rx_error_counter = errors.RxErrorCnt;

    /* Copy each feedback frame coherently before checking its age and state. */
    primask = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();
    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        snapshot[index] = dm_feedback[index];
    }
    __set_PRIMASK(primask);

    if (dm_can_diag.bus_off != 0U)
    {
        if (dm_test.active_mask != 0U && dm_test.state != DM_TEST_DISABLING)
        {
            dm_test_stop(DM_TEST_STOP_BUS_OFF, dm_test.active_mask, now, snapshot);
        }
        dm_test.run = 0U;
        if (dm_test.state == DM_TEST_DISABLING)
        {
            dm_test_disable_step(now, snapshot);
        }
        return;
    }

    /* V1 遥控优先。正在 Watch 点动时先走完失能流程，再交给遥控。 */
    if (use_remote_control)
    {
        dm_test.run = 0U;
        dm_test.motor_mask = 0U;
        if (dm_test.state == DM_TEST_IDLE && dm_test.disable_pending_mask == 0U)
        {
            control_remote_motors(enabled_motors, selected_motors, joint_motion_request, now, snapshot);
            return;
        }
    }

    if ((dm_test.state == DM_TEST_ENABLING || dm_test.state == DM_TEST_MOVING) && dm_test.run == 0U)
    {
        dm_test_stop(DM_TEST_STOP_REQUEST, 0U, now, snapshot);
        return;
    }

    switch (dm_test.state)
    {
    case DM_TEST_IDLE:
        dm_test_idle(dm_test.motor_mask & DM_TEST_ALL_MOTORS, now, snapshot);
        break;
    case DM_TEST_ENABLING:
        dm_test_enable_step(now, snapshot);
        break;
    case DM_TEST_MOVING:
        dm_test_move_step(now, snapshot);
        break;
    case DM_TEST_DISABLING:
        dm_test_disable_step(now, snapshot);
        break;
    }
}
