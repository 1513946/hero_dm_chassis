#include "dm_motor.h"

#include <math.h>

#define DM_TEST_ALL_MOTORS ((1U << DM_MOTOR_COUNT) - 1U)
#define DM_TEST_PERIOD_MS 10U
#define DM_TEST_FEEDBACK_MS 50U
#define DM_TEST_POLL_FRESH_MS 120U
#define DM_TEST_ENABLE_MS 150U
#define DM_TEST_DISABLE_MS 150U
#define DM_TEST_DURATION_MS 5000U
#define DM_TEST_OFFSET_RAD 1.0f
#define DM_TEST_TRAVEL_RAD 0.18f
#define DM_TEST_RAMP_RAD_S 0.10f
#define DM_TEST_KP 10.0f
#define DM_TEST_KD 0.25f
#define DM_REMOTE_POSITION_ERROR_RAD 0.10f

typedef struct
{
    uint16_t can_id;
    uint16_t master_id;
    float pmax;
    float vmax;
    float tmax;
    float extension_direction;
} dm_config_t;

/* 核对各电机的 MIT 映射值。最后一列是伸出方向，反向安装时改为 -1。
 * 下标 0、1 对应两台 8009P，下标 2、3 对应两台 4340。
 */
static const dm_config_t dm_config[DM_MOTOR_COUNT] =
{
    {0x01U, 0x11U, 12.5f, 45.0f, 54.0f, 1.0f},
    {0x02U, 0x12U, 12.5f, 45.0f, 54.0f, 1.0f},
    {0x03U, 0x13U, 12.5f, 10.0f, 28.0f, 1.0f},
    {0x04U, 0x14U, 12.5f, 10.0f, 28.0f, 1.0f},
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
static float remote_target_position[DM_MOTOR_COUNT];
static uint8_t remote_disable_sent_mask;
static uint32_t remote_stop_feedback_count[DM_MOTOR_COUNT];

HAL_StatusTypeDef dm_motor_can_start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    const uint32_t interrupts = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_BUS_OFF;

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

static HAL_StatusTypeDef dm_send_mode_command(uint8_t index, uint8_t command)
{
    const uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};

    if (index >= DM_MOTOR_COUNT)
    {
        return HAL_ERROR;
    }
    return dm_send(dm_config[index].can_id, FDCAN_DLC_BYTES_8, data);
}

HAL_StatusTypeDef dm_motor_enable(uint8_t index)
{
    return dm_send_mode_command(index, 0xFCU);
}

HAL_StatusTypeDef dm_motor_disable(uint8_t index)
{
    return dm_send_mode_command(index, 0xFDU);
}

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

static float dm_decode(uint16_t code, float max, uint16_t code_max)
{
    return (float)code * (2.0f * max / (float)code_max) - max;
}

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

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    if (hfdcan == &hfdcan3 && (interrupts & FDCAN_IT_BUS_OFF) != 0U)
    {
        dm_can_diag.bus_off = 1U;
    }
}

/* Status requests use 0x7FF. Space them apart to give the higher-priority
 * reply time to finish before sending the next request.
 */
static void dm_poll_one(uint32_t mask)
{
    uint8_t scanned;

    for (scanned = 0U; scanned < DM_MOTOR_COUNT; scanned++)
    {
        const uint8_t index = next_poll_index;
        next_poll_index = (uint8_t)((index + 1U) % DM_MOTOR_COUNT);
        if ((mask & (1U << index)) != 0U)
        {
            (void)dm_motor_request_feedback(index);
            return;
        }
    }
}

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
//
static void stop_remote_motor(uint8_t index, bool feedback_is_fresh,
                              const dm_feedback_t *feedback)
{
    const uint8_t motor_bit = 1U << index;

    if ((dm_remote_active_mask & motor_bit) == 0U)
    {
        return;
    }

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

static void control_remote_motors(uint8_t selected_motors, int8_t joint_motion_request,
                                  uint32_t now, const dm_feedback_t *feedback)
{
    const float motion_fraction = (float)joint_motion_request / 100.0f;
    const float position_step = motion_fraction * DM_TEST_RAMP_RAD_S * (DM_TEST_PERIOD_MS / 1000.0f);
    uint8_t feedback_request_mask = 0U;
    uint8_t index;

    for (index = 0U; index < DM_MOTOR_COUNT; index++)
    {
        const uint8_t motor_bit = 1U << index;
        const bool selected = (selected_motors & motor_bit) != 0U;
        const bool feedback_is_fresh = feedback[index].rx_count != 0U &&
            now - feedback[index].last_rx_ms < DM_TEST_FEEDBACK_MS;

        if (!selected || !feedback_is_fresh || feedback[index].state > 1U ||
            (remote_disable_sent_mask & motor_bit) != 0U)
        {
            stop_remote_motor(index, feedback_is_fresh, &feedback[index]);
            if (selected)
            {
                feedback_request_mask |= motor_bit;
            }
            continue;
        }

        if ((dm_remote_active_mask & motor_bit) == 0U || feedback[index].state == 0U)
        {
            remote_target_position[index] = feedback[index].position_rad;
            if (dm_motor_enable(index) == HAL_OK)
            {
                dm_remote_active_mask |= motor_bit;
            }
            feedback_request_mask |= motor_bit;
            continue;
        }

        if (joint_motion_request != 0)
        {
            const float target = remote_target_position[index] + position_step * dm_config[index].extension_direction;
            const float current_position = feedback[index].position_rad;

            /* 卡住时也不累积过大的位置误差。 */
            remote_target_position[index] = fminf(current_position + DM_REMOTE_POSITION_ERROR_RAD,
                                                   fmaxf(current_position - DM_REMOTE_POSITION_ERROR_RAD, target));
        }
        (void)dm_motor_send_mit(index, remote_target_position[index], 0.0f, DM_TEST_KP, DM_TEST_KD, 0.0f);
    }

    dm_poll_one(feedback_request_mask);
}

void dm_motor_control_step(bool use_remote_control, uint8_t selected_motors,
                           int8_t joint_motion_request)
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

    if (use_remote_control)
    {
        dm_test.run = 0U;
        dm_test.motor_mask = 0U;
        if (dm_test.state == DM_TEST_IDLE && dm_test.disable_pending_mask == 0U)
        {
            control_remote_motors(selected_motors, joint_motion_request, now, snapshot);
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
