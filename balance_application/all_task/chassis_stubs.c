#include "chassis_stubs.h"

/*
 * 旁路桩：本板还没有移植完整的 INS 姿态解算（依赖 BMI088 + Mahony + IST8310），
 * 这里返回恒 0 的陀螺数据。这样底盘代码里的跟随前馈项
 *     chassis_gyro * CHASSIS_SOLLOW_GIMBAL_FORWARD_KP
 * 恒为 0，不影响四轮运动学验证；以后接好 INS 后删除本文件并链接 ins_task.c 即可。
 */
static fp32 stub_gyro[3] = {0.0f, 0.0f, 0.0f};

const fp32 *get_gyro_data_point(void)
{
    return stub_gyro;
}
