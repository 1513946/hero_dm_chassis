#ifndef CAN_INIT_H
#define CAN_INIT_H
#include "main.h"
#include "c_data_typedef.h"

/* can_bus_diag[0] 对应 CAN1/3508，[1] 对应 CAN2/云台。
 * init_stage：1滤波器，2全局过滤，3中断线路，4通知，5启动，6初始化成功。
 * started 只表示 HAL 已启动；是否真正通信要看 rx_count 和 bus_off。 */
typedef struct
{
    uint32_t init_stage;
    uint32_t init_status;          /* HAL_OK=0，HAL_ERROR=1，HAL_BUSY=2，HAL_TIMEOUT=3 */
    uint32_t started;
    uint32_t kernel_clock_hz;
    uint32_t nominal_bitrate;
    uint32_t hal_error;
    uint32_t rx_count;             /* 实际取出的帧数，包含未知 ID */
    uint32_t last_rx_ms;
    uint32_t last_rx_id;
    uint32_t rx_read_errors;
    uint32_t rx_invalid;
    uint32_t rx_fifo_lost;
    uint32_t tx_queued;            /* 仅入队，不代表对端收到 */
    uint32_t tx_fifo_full;
    uint32_t tx_hal_errors;
    uint32_t bus_off;
    uint32_t bus_off_count;        /* 周期采样发现的 Bus-off 次数 */
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t last_protocol_error;  /* 保存最近一次非0、非7的 LEC */
    uint32_t recovery_count;       /* HAL Stop/Start 成功次数，不代表已恢复收帧 */
    uint32_t recovery_errors;
} can_bus_diag_t;

extern volatile can_bus_diag_t can_bus_diag[2];

extern void can_filter_init(void);
/* 由底盘任务周期调用，内部每100ms采样状态，并请求 CAN1/2 的 Bus-off 恢复。 */
void can_bus_service(void);

#endif
