"""用主机gcc和模拟HAL验证实际CAN源码；不连接硬件、不烧录。

运行：py tests/test_can_link.py
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

HAL = r'''
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
#include <string.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t TXBRP; } FDCAN_TypeDef;
typedef struct { uint32_t NominalPrescaler, NominalTimeSeg1, NominalTimeSeg2; } Init;
typedef struct { FDCAN_TypeDef *Instance; Init Init; uint32_t ErrorCode, State; } FDCAN_HandleTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2; } FDCAN_FilterTypeDef;
typedef struct { uint32_t LastErrorCode, BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct {
 uint32_t Identifier, IdType, TxFrameType, RxFrameType, DataLength, ErrorStateIndicator;
 uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef FDCAN_TxHeaderTypeDef FDCAN_RxHeaderTypeDef;
#define RCC_PERIPHCLK_FDCAN 1
#define FDCAN_STANDARD_ID 0
#define FDCAN_FILTER_MASK 1
#define FDCAN_FILTER_TO_RXFIFO0 1
#define FDCAN_ACCEPT_IN_RX_FIFO0 0
#define FDCAN_REJECT 2
#define FDCAN_REJECT_REMOTE 1
#define FDCAN_INTERRUPT_LINE0 0
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 8
#define FDCAN_DATA_FRAME 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_DLC_BYTES_4 4
#define FDCAN_DLC_BYTES_8 8
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
uint32_t HAL_GetTick(void);
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
void dm_motor_can_rx_irq(FDCAN_HandleTypeDef *, uint32_t);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
'''

TEST = r'''
#include <assert.h>
#include <stdio.h>
#include "can_user.h"
#include "fdcan.h"
FDCAN_TypeDef regs[3];
FDCAN_HandleTypeDef hfdcan1 = {&regs[0], {4,20,4},0,0};
FDCAN_HandleTypeDef hfdcan2 = {&regs[1], {4,20,4},0,0};
FDCAN_HandleTypeDef hfdcan3 = {&regs[2], {1,14,4},0,0};
extern volatile chassis_c_get_data_t mini_chassis;
extern volatile uint32_t from_yuntai_can;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);
static uint32_t tick, fail_stage, fifo_free=1, bus_off, lec, stop_error;
static uint32_t aborts, stops, starts, dm_calls, read_error;
static FDCAN_HandleTypeDef *pending;
static FDCAN_RxHeaderTypeDef rx;
static uint8_t bytes[64];
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t p) { (void)p; return 100000000; }
static HAL_StatusTypeDef step(FDCAN_HandleTypeDef *h, uint32_t stage) {
 if(h==&hfdcan1 && fail_stage==stage) { h->ErrorCode=0x100; return HAL_ERROR; }
 return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,FDCAN_FilterTypeDef *f) {
 assert(f->FilterIndex==0 && f->FilterID2==0); return step(h,1);
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b,uint32_t c,uint32_t d) {
 assert(a==FDCAN_ACCEPT_IN_RX_FIFO0 && b==FDCAN_REJECT && c==FDCAN_REJECT_REMOTE && d==FDCAN_REJECT_REMOTE);
 return step(h,2);
}
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b) {
 (void)a; assert(b==FDCAN_INTERRUPT_LINE0); return step(h,3);
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b) {
 (void)b; assert(a & FDCAN_IT_RX_FIFO0_MESSAGE_LOST); return step(h,4);
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { starts++; return step(h,5); }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) { (void)h; stops++; return stop_error?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t mask) {
 assert(mask==h->Instance->TXBRP); aborts++; h->Instance->TXBRP=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p) {
 p->BusOff=h==&hfdcan2?bus_off:0; p->LastErrorCode=lec; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *e) {
 (void)h; e->TxErrorCnt=12; e->RxErrorCnt=3; return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { (void)h; return fifo_free; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *t,uint8_t *p) {
 (void)h; (void)p; assert(t->Identifier==0x200 && t->DataLength==8); return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t f) { (void)f; return pending==h; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,uint32_t f,FDCAN_RxHeaderTypeDef *r,uint8_t *p) {
 (void)h; (void)f; pending=0; if(read_error) return HAL_ERROR;
 *r=rx; memcpy(p,bytes,rx.DataLength); return HAL_OK;
}
void dm_motor_can_rx_irq(FDCAN_HandleTypeDef *h,uint32_t flags) { (void)h; (void)flags; dm_calls++; }
static void frame(FDCAN_HandleTypeDef *h,uint32_t id,uint32_t len) {
 rx=(FDCAN_RxHeaderTypeDef){0}; rx.Identifier=id; rx.DataLength=len; pending=h;
 HAL_FDCAN_RxFifo0Callback(h,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
int main(void) {
 for(fail_stage=1;fail_stage<=5;fail_stage++) {
  can_filter_init();
  assert(can_bus_diag[0].init_stage==fail_stage && can_bus_diag[0].init_status==HAL_ERROR);
  assert(!can_bus_diag[0].started && can_bus_diag[1].started);
 }
 fail_stage=0; can_filter_init();
 assert(can_bus_diag[0].nominal_bitrate==1000000 && can_bus_diag[1].nominal_bitrate==1000000);
 assert(can_bus_diag[0].init_stage==6 && can_bus_diag[1].init_status==HAL_OK);
 puts("PASS: all initialization failure stages, independent CAN2 start, live bitrates");

 tick=100; bytes[0]=0x12; bytes[1]=0x34; bytes[6]=23;
 frame(&hfdcan1,0x203,8); frame(&hfdcan1,0x203,8);
 assert(chassis_motor_rx_count[2]==2 && chassis_motor_last_rx_ms[2]==100);
 assert(get_motor_chassis_motor_measure_point(2)->ecd==0x1234);
 bytes[0]=0; frame(&hfdcan1,0x203,2);
 assert(chassis_motor_rx_count[2]==2 && can_bus_diag[0].rx_invalid==1);
 frame(&hfdcan1,0x555,8); assert(can_bus_diag[0].last_rx_id==0x555);
 puts("PASS: stationary 3508 reception, short-frame rejection, unknown-ID diagnostics");

 memset(bytes,0,sizeof bytes);
 bytes[0]=0x81; bytes[1]=100; bytes[2]=0xB0; bytes[3]=25;
 bytes[4]=0x9D; bytes[7]=0x01; /* 不跟随、前臂、超电、遥控有效、另一组正向 */
 frame(&hfdcan2,0x307,8);
 assert(can2_command_diag.rx_count==1 && from_yuntai_can==1 && mini_chassis.climb_mode);
 assert(mini_chassis.x_rc_data==-127 && mini_chassis.y_rc_data==100);
 assert(mini_chassis.joint_forward==-80 && mini_chassis.joint_lateral==25);
 assert(mini_chassis.chassis_mode==2 && mini_chassis.cap_mode==1);
 assert(mini_chassis.climb_group==CLIMB_FRONT_GROUP && mini_chassis.other_group_direction==1);
 bytes[4]=0xA6; frame(&hfdcan2,0x307,8);
 assert(mini_chassis.climb_group==CLIMB_REAR_GROUP && mini_chassis.other_group_direction==-1);
 bytes[4]=0xB4; frame(&hfdcan2,0x307,8);
 assert(mini_chassis.climb_group==CLIMB_MECANUM_WHEELS && mini_chassis.other_group_direction==0);
 tick=199; bytes[7]=0x02; frame(&hfdcan2,0x307,8);
 assert(can2_command_diag.rx_count==4 && from_yuntai_can==3 && can2_command_diag.format_reject_count==1);
 assert(can2_command_diag.last_data[7]==0x01);
 chassis_c_get_data_t command;
 read_gimbal_chassis_command(&command);
 assert(command.climb_mode);
 tick=200; read_gimbal_chassis_command(&command);
 assert(!command.climb_mode && command.x_rc_data==0 && command.joint_forward==0);
 bytes[4]=0x9D; bytes[5]=1; bytes[7]=0x01; frame(&hfdcan2,0x307,8);
 assert(can2_command_diag.stop_count==1 && !mini_chassis.climb_mode && mini_chassis.x_rc_data==0);
 bytes[4]=0x99; bytes[5]=0; frame(&hfdcan2,0x307,8);
 assert(can2_command_diag.remote_invalid_count==1 && !mini_chassis.climb_mode && mini_chassis.joint_forward==0);
 frame(&hfdcan2,0x307,2); assert(chassis_top_rx_bad_frame_count==1 && can2_command_diag.rx_count==7);
 frame(&hfdcan3,0x11,8); assert(dm_calls==1);
 read_error=1; frame(&hfdcan1,0x201,8); read_error=0;
 assert(can_bus_diag[0].rx_read_errors==1);
 HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
 assert(can_bus_diag[0].rx_fifo_lost==1);
 puts("PASS: 0x307 signed axes, groups and dial, format/timeout/STOP/remote checks, CAN3 dispatch, read/lost errors");

 CAN_cmd_chassis(0,0,0,0); fifo_free=0; CAN_cmd_chassis(0,0,0,0);
 assert(can_bus_diag[0].tx_queued==1 && can_bus_diag[0].tx_fifo_full==1);
 bus_off=1; lec=3; regs[1].TXBRP=5;
 uint32_t old_starts=starts;
 can_bus_service();
 assert(aborts==1 && stops==1 && starts==old_starts+1);
 assert(can_bus_diag[1].bus_off_count==1 && can_bus_diag[1].recovery_count==1);
 assert(can_bus_diag[1].last_protocol_error==3 && regs[1].TXBRP==0);
 can_bus_service(); assert(aborts==1);
 tick=300; bus_off=0; lec=7; can_bus_service();
 assert(!can_bus_diag[1].bus_off && can_bus_diag[1].last_protocol_error==3);
 tick=400; bus_off=1; stop_error=1; old_starts=starts; can_bus_service();
 assert(starts==old_starts && can_bus_diag[1].recovery_errors==1);
 puts("PASS: TX diagnostics, Bus-off recovery ordering, rate limit and failure handling");
 return 0;
}
'''


def main():
    # 检查真实CubeMX配置，不能只用模拟HAL中的固定时序代替配置验证。
    fdcan = (ROOT / "Core/Src/fdcan.c").read_text(encoding="utf-8")
    main_c = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    config = dict(line.split("=", 1) for line in
                  (ROOT / "hero_dm_chassis.ioc").read_text(encoding="utf-8").splitlines()
                  if "=" in line)
    def pll_value(field):
        return int(re.search(r"PLL2\.PLL2" + field + r"\s*=\s*(\d+)", main_c)[1])
    clock = int(config["RCC.HSE_VALUE"]) * pll_value("N") // pll_value("M") // pll_value("Q")
    assert clock == int(config["RCC.FDCANFreq_Value"])
    for bus in (1, 2):
        timing = {}
        for field in ("NominalPrescaler", "NominalTimeSeg1", "NominalTimeSeg2"):
            timing[field] = int(re.search(r"hfdcan" + str(bus) + r"\.Init\." + field + r"\s*=\s*(\d+)", fdcan)[1])
            assert timing[field] == int(config[f"FDCAN{bus}.{field}"]), (bus, field)
        denominator = timing["NominalPrescaler"] * (1 + timing["NominalTimeSeg1"] + timing["NominalTimeSeg2"])
        assert clock == 1000000 * denominator, f"CAN{bus} is not exactly 1Mbps"
    print("PASS: actual C and CubeMX CAN1/CAN2 timing agree at exactly 1Mbps", flush=True)
    gcc = shutil.which("gcc")
    if not gcc:
        raise SystemExit("需要主机gcc；该测试不会调用Keil或连接开发板。")
    with tempfile.TemporaryDirectory(prefix="can-link-") as directory:
        work = Path(directory)
        for relative in [
            "balance_bsp/scm_bsp/can_init/can_init.c",
            "balance_bsp/scm_bsp/can_init/can_init.h",
            "balance_application/can/can_user.c",
            "balance_application/can/can_user.h",
        ]:
            shutil.copyfile(ROOT / relative, work / Path(relative).name)
        (work / "mock_hal.h").write_text(HAL, encoding="utf-8")
        for name in ["main.h", "fdcan.h", "dm_motor.h"]:
            (work / name).write_text('#include "mock_hal.h"\n', encoding="utf-8")
        (work / "c_data_typedef.h").write_text(
            '#include <stdint.h>\ntypedef float fp32;\n', encoding="utf-8")
        (work / "referee.h").write_text('''
typedef struct {
 struct { unsigned robot_id, robot_level, shooter_barrel_cooling_value,
          shooter_barrel_heat_limit, chassis_power_limit; } robot_state;
 struct { float bullet_speed; } shoot_data_t;
 struct { unsigned shooter_17mm_1_barrel_heat, buffer_energy; } power_heat_data_t;
} RM_Referee_system_t;
''', encoding="utf-8")
        (work / "test.c").write_text(TEST, encoding="utf-8")
        exe = work / "can_link_test.exe"
        subprocess.run([gcc, "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "can_init.c", "can_user.c", "test.c", "-o", str(exe)],
                       cwd=work, check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
