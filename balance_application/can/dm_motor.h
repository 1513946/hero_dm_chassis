#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdbool.h>
#include "fdcan.h"

#define DM_MOTOR_COUNT 4U

typedef enum
{
    DM_TEST_IDLE = 0,
    DM_TEST_ENABLING,
    DM_TEST_MOVING,
    DM_TEST_DISABLING
} dm_test_state_t;

typedef enum
{
    DM_TEST_NO_STOP = 0,
    DM_TEST_STOP_REQUEST,
    DM_TEST_STOP_TIME_LIMIT,
    DM_TEST_STOP_NOT_READY,
    DM_TEST_STOP_TX_FAILED,
    DM_TEST_STOP_FEEDBACK_TIMEOUT,
    DM_TEST_STOP_MOTOR_STATE,
    DM_TEST_STOP_POSITION,
    DM_TEST_STOP_BUS_OFF
} dm_test_stop_t;

typedef struct
{
    uint32_t motor_mask;                 /* Watch: bit 0..3 selects motors for polling. */
    uint32_t run;                        /* Watch: 1 starts one bounded test; 0 stops it. */
    float offset_rad[DM_MOTOR_COUNT];      /* Watch: latched when the test starts. */
    dm_test_state_t state;
    dm_test_stop_t stop_reason;
    uint32_t active_mask;
    uint32_t fault_mask;
    uint32_t disable_pending_mask;
} dm_test_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    /* 只在 Watch 点动的 MOVING 阶段统计，单位 ms。 */
    uint32_t max_move_rx_gap_ms;
    uint8_t state;
    uint8_t mos_temp_c;
    uint8_t coil_temp_c;
} dm_feedback_t;

typedef struct
{
    uint32_t started;
    uint32_t tx_fifo_full;
    uint32_t tx_hal_error;
    /* 发送入队次数。按轴查看实际收包时用 dm_feedback[index].rx_count。 */
    uint32_t tx_status_queued[DM_MOTOR_COUNT];
    uint32_t tx_mit_queued[DM_MOTOR_COUNT];
    uint32_t rx_fifo_lost_events;
    uint32_t rx_read_errors;
    uint32_t rx_invalid;
    uint32_t protocol_error_events;
    uint32_t ack_error_events;
    uint32_t last_error_code;
    uint32_t bus_off;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
} dm_can_diag_t;

extern volatile dm_test_t dm_test;
extern volatile dm_feedback_t dm_feedback[DM_MOTOR_COUNT];
extern volatile dm_can_diag_t dm_can_diag;
/* 遥控机械臂在线调参，四台共用；在 Keil Watch 的 Value 列修改。 */
extern volatile float dm_remote_kp;
extern volatile float dm_remote_kd;
extern volatile float dm_remote_speed_rad_s;
/* 重力前馈在线调参：enable 先保持 0，确认每条腿的补偿方向后再打开。
 * load_kg 是这条腿静态承担的整机重量（kg），总重 20 kg、四条腿均匀时填 5.0。
 * 车轮也在承重时，腿实际承担的重量可能远小于 5 kg，需要按实际工况填。 */
extern volatile uint8_t dm_gravity_enable;
extern volatile float dm_gravity_scale;
extern volatile float dm_gravity_load_kg[DM_MOTOR_COUNT];
/* 最近一次算出的重力前馈力矩（电机侧，N·m，未乘渐入系数），Watch 里用来核对方向和大小。 */
extern volatile float dm_gravity_torque_out[DM_MOTOR_COUNT];
/* 遥控已提交使能的电机位，等待失能确认时仍保留。实际使能看反馈 state。 */
extern volatile uint8_t dm_remote_active_mask;

HAL_StatusTypeDef dm_motor_can_start(void);
void dm_motor_can_rx_irq(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_interrupts);

/* Transmit functions take a zero-based motor index. Call from a single task. */
HAL_StatusTypeDef dm_motor_enable(uint8_t index);
HAL_StatusTypeDef dm_motor_disable(uint8_t index);
HAL_StatusTypeDef dm_motor_request_feedback(uint8_t index);
HAL_StatusTypeDef dm_motor_send_mit(uint8_t index, float position, float velocity,
                                   float kp, float kd, float torque);
/* 底盘任务周期调用。selected_motors 的 bit0 到 bit3 对应四台电机。 */
void dm_motor_control_step(bool use_remote_control, uint8_t selected_motors,
                           int8_t joint_motion_request);

#endif
