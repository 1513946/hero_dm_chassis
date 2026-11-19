#include "can_init.h"
#include "main.h"
#include "fdcan.h"

volatile can_bus_diag_t can_bus_diag[2];

/* 每一步都检查结果；失败阶段及 HAL 返回值留在 Watch 中，不再静默失败。 */
static void can_start_bus(FDCAN_HandleTypeDef *hfdcan, volatile can_bus_diag_t *diag)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    const uint32_t interrupts = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                FDCAN_IT_RX_FIFO0_MESSAGE_LOST;

    diag->started = 0U;
    diag->kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    diag->nominal_bitrate = diag->kernel_clock_hz /
        (hfdcan->Init.NominalPrescaler *
         (1U + hfdcan->Init.NominalTimeSeg1 + hfdcan->Init.NominalTimeSeg2));

    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;  /* 掩码0：接收所有标准帧，以便发现实际到达的ID。 */

    diag->init_stage = 1U;
    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK) { goto done; }

    diag->init_stage = 2U;
    status = HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
                                         FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE);
    if (status != HAL_OK) { goto done; }

    /* 工程只启用了 IT0 的 NVIC，显式把接收通知分配到线路0。 */
    diag->init_stage = 3U;
    status = HAL_FDCAN_ConfigInterruptLines(hfdcan, interrupts, FDCAN_INTERRUPT_LINE0);
    if (status != HAL_OK) { goto done; }

    diag->init_stage = 4U;
    status = HAL_FDCAN_ActivateNotification(hfdcan, interrupts, 0U);
    if (status != HAL_OK) { goto done; }

    diag->init_stage = 5U;
    status = HAL_FDCAN_Start(hfdcan);
    if (status == HAL_OK)
    {
        diag->started = 1U;
        diag->init_stage = 6U;
    }

done:
    diag->init_status = (uint32_t)status;
    diag->hal_error = hfdcan->ErrorCode;
}

void can_filter_init(void)
{
    can_start_bus(&hfdcan1, &can_bus_diag[0]);
    can_start_bus(&hfdcan2, &can_bus_diag[1]);
}

/* 不在中断里重启，不复位共享 FDCAN 时钟，避免影响另两路 CAN。 */
void can_bus_service(void)
{
    static uint32_t last_service_ms;
    FDCAN_HandleTypeDef *handles[2] = {&hfdcan1, &hfdcan2};
    const uint32_t now = HAL_GetTick();
    uint8_t index;

    if (now - last_service_ms < 100U) { return; }
    last_service_ms = now;

    for (index = 0U; index < 2U; index++)
    {
        FDCAN_HandleTypeDef *hfdcan = handles[index];
        volatile can_bus_diag_t *diag = &can_bus_diag[index];
        FDCAN_ProtocolStatusTypeDef protocol;
        FDCAN_ErrorCountersTypeDef errors;

        if (diag->started == 0U) { continue; }

        HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol);
        HAL_FDCAN_GetErrorCounters(hfdcan, &errors);
        diag->hal_error = hfdcan->ErrorCode;
        diag->tx_error_count = errors.TxErrorCnt;
        diag->rx_error_count = errors.RxErrorCnt;
        if (protocol.LastErrorCode != 0U && protocol.LastErrorCode != 7U)
        {
            diag->last_protocol_error = protocol.LastErrorCode;
        }
        if (protocol.BusOff != 0U && diag->bus_off == 0U)
        {
            diag->bus_off_count++;
        }
        diag->bus_off = protocol.BusOff;

        if (protocol.BusOff != 0U)
        {
            /* 丢掉故障前积压的发送请求，避免恢复后发出旧运动指令。 */
            HAL_StatusTypeDef status = HAL_FDCAN_AbortTxRequest(hfdcan, hfdcan->Instance->TXBRP);
            if (status == HAL_OK) { status = HAL_FDCAN_Stop(hfdcan); }
            if (status == HAL_OK) { status = HAL_FDCAN_Start(hfdcan); }
            if (status == HAL_OK)
            {
                diag->recovery_count++;
            }
            else
            {
                diag->recovery_errors++;
                diag->hal_error = hfdcan->ErrorCode;
            }
        }
    }
}
