#include "can_init.h"
#include "main.h"
#include "fdcan.h"

/*
 * FDCAN 滤波器初始化：把 hfdcan1 / hfdcan2 配置成“接收全部标准帧”，
 * 并启动外设 + 打开 RX FIFO0 新报文中断。
 *
 * 注意：FDCAN1 / FDCAN2 在 CubeMX 里 StdFiltersNbr 都 = 1，
 *       因此 FilterIndex 只能用 0。
 */
void can_filter_init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;                 /* 必须 < StdFiltersNbr */
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;             /* ID */
    filter.FilterID2    = 0x000;             /* 掩码 0：接收全部标准帧 */

    /* 底盘 CAN */
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);
    (void)HAL_FDCAN_Start(&hfdcan1);
    (void)HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* 云台 CAN */
    (void)HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);
    (void)HAL_FDCAN_Start(&hfdcan2);
    (void)HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
