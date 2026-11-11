#ifndef CHASSIS_STUBS_H
#define CHASSIS_STUBS_H

#include "c_data_typedef.h"

/* INS 陀螺数据下标，与原 ins_task.h 保持一致 */
#define INS_GYRO_Z_ADDRESS_OFFSET 2

/* 旁路版陀螺数据获取：本板尚未移植 INS 姿态解算，返回恒 0 */
extern const fp32 *get_gyro_data_point(void);

#endif
