#ifndef CAN_USER_H
#define CAN_USER_H

#include "c_data_typedef.h"
#include "can_init.h"
#include "referee.h"
#include <stdbool.h>

/* 总线别名：沿用老 C 板划分
 *   hfdcan1 = 底盘 CAN（4 个 M3508）
 *   hfdcan2 = 云台 CAN（云台板通信 + yaw 电机）
 * 具体引脚在 CubeMX 里配置，这里只按逻辑总线区分。 */

typedef enum
{
    /****  hcan1 底盘电机接收报文  ****/
    CAN1_3508_M1_ID = 0x201,   /* 底盘电机 1 反馈 */
    CAN1_3508_M2_ID = 0x202,   /* 底盘电机 2 反馈 */
    CAN1_3508_M3_ID = 0x203,   /* 底盘电机 3 反馈 */
    CAN1_3508_M4_ID = 0x204,   /* 底盘电机 4 反馈 */

    /****  hcan2 云台电机  ****/
    CAN2_yaw_motor_id = 0x141, /* yaw 轴电机反馈，底盘与云台同时解算降低延迟 */

    /****  底盘电机发送报文  ****/
    CAN1_chassis_flag_id = 0x200,

    /****  其它报文  ****/
    CAN2_send_gimbal_msg   = 0x308,  /* 底盘发送到云台 */
    CAN2_recive_gimbal_msg = 0x307,  /* 底盘接收云台（遥控器数据/模式） */
    CAN2_send_gimbal_pitch = 0x306,  /* 底盘发送 pitch */

    CAN1_recive_cap_msg = 0x304,     /* 接收超电信息 */
    CAN1_send_cap_msg   = 0x305,     /* 发送信息到超电 */
} can_msg_id;

typedef struct
{
    uint16_t ecd;           /* 编码器机械数值 */
    int16_t  speed_rpm;     /* 转子转速 rpm */
    int16_t  given_current; /* 转子转矩电流 */
    uint8_t  temperate;     /* 转子温度 */
    int16_t  last_ecd;      /* 上一次编码器值 */
    int16_t  delta_ecd;     /* 增量 */
    int16_t  round;         /* 转的圈数 */
} motor_measure_t;

typedef struct can_user
{
    fp32 V_cap_value;       /* 电容电压 */
} super_cap_t;

/* 云台板经 0x307 下发的数据（底盘模式和遥控量都从这里来） */
typedef struct
{
    int16_t x_rc_data;        /* 16 位 X_rc 摇杆 */
    int16_t y_rc_data;        /* 16 位 Y_rc 摇杆 */
    uint8_t x_key_data;       /* 两位 X_key */
    uint8_t y_key_data;       /* 两位 Y_key */
    uint8_t chassis_mode;     /* 底盘模式 0零力 1静止 2不跟随 3跟随 */
    uint8_t cap_mode;         /* 超电开关 */
    uint8_t top_mode;         /* 小陀螺开关 */
    uint8_t follow_mode;      /* 一键掉头 */
    uint8_t follow_mode_last; /* 一键掉头上一次状态 */
    uint8_t key_control;      /* 键盘控制（比赛模式） */
} chassis_c_get_data_t;

/* 0x307 接收诊断计数器，供 Keil Watch 查看 */
extern volatile uint32_t chassis_top_rx_on_count;
extern volatile uint32_t chassis_top_rx_off_count;
extern volatile uint32_t chassis_top_rx_bad_frame_count;
extern volatile uint32_t chassis_can_rx_error_count;
extern volatile uint8_t  chassis_top_rx_last_off_flags;

/* ---- 发送 ---- */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_cap(RM_Referee_system_t *rm_data);
extern void CAN_chassis_send_gimbal(RM_Referee_system_t *rm_data);
extern void CAN_chassis_ramp_pitch_to_gimbal(fp32 pitch);
extern void CAN_cmd_chassis_read(uint8_t id, int16_t current);

/* ---- 接收点获取 ---- */
extern const motor_measure_t *get_motor_chassis_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_motor_gimbal_yaw_measure_point(void);
extern const super_cap_t *get_super_cap_measure_point(void);
extern const chassis_c_get_data_t *get_c_gimbal_data_point(void);

#endif
