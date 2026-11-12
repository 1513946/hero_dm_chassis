/*
 * ins_task.c —— INS 姿态解算（从老 C 板 F407 工程移植到 DM-MC-Board02 / STM32H723）
 *
 * 本次只做硬件层适配，控制逻辑（Mahony、温度 PID、加速度低通、零漂校准等）保持原样：
 *   - SPI1(PA7/PB3/PB4)        -> SPI2(PC1/PB13/PC2_C)            （DM 板板载 BMI088）
 *   - CS1_ACCEL/CS1_GYRO       -> PC0 / PC3                       （main.h 已定义）
 *   - INT1_ACCEL/INT1_GYRO     -> PE10 / PE12                     （main.h 已定义）
 *   - TIM10_CH1(PF6) 加热      -> TIM3_CH4(PB1)                   （DM 板 IMU 加热）
 *   - F4 寄存器直接操作 DMA    -> HAL_SPI_TransmitReceive_DMA + 完成回调
 *   - IST8310 磁力计 DM 板没有 -> 跳过其初始化，mag 恒为 0（原代码本来也未参与解算）
 *
 * 任务函数名仍为 Imu_Task，与 freertos.c 中的 weak 定义对应。
 */
#include "ins_task.h"
#include "main.h"
#include "spi.h"

#include "can_user.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MahonyAHRS.h"
#include "math.h"

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                    \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

fp32 ist8310_data[3] = {0, 0, 0};
extern TIM_HandleTypeDef htim3;   /* 旧板 TIM10_CH1 -> DM 板 TIM3_CH4(IMU 加热) */

static void imu_pwm_set(uint16_t pwm);
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void imu_temp_control(fp32 temp);
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};


volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   /* 任务运行周期，单位 s */

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

/* 加速度计低通滤波 */
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      /* 欧拉角，单位 rad */
fp32 yaw = 0.0f;
fp32 pitch = 0.0f;
fp32 roll = 0.0f;

void Imu_Task(void *argument)
{
    (void)argument;

    vTaskDelay(INS_TASK_INIT_TIME);
    //L_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while (BMI088_init())
    {
        vTaskDelay(100);
    }
    /*
     * 旧代码在这里还会 while(ist8310_init())：IST8310 是磁力计，DM 板没有该器件，
     * 且下面的解算本来就不使用磁力计（INS_mag 传入即全 0），因此跳过初始化。
     */
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    pid_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);
    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    /* BMI088 初始化完成后提高 SPI 时钟（与原代码一致） */
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    imu_start_dma_flag = 1;   /* DMA 可以开始 */
    while (1)
    {
        /* 等待 SPI DMA 完成 */
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        /* 旋转和零漂 */
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        /* 加速度计低通滤波 */
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
        yaw = *(INS_angle + INS_YAW_ADDRESS_OFFSET);
        pitch = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
        roll = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
        /* 不使用 IST8310，节省时间（保持原逻辑） */
        if (mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
//          ist8310_read_mag(ist8310_real_data.mag);
        }
    }
}

void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])   /* 四元数更新 */
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}

void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)   /* 由四元数求欧拉角 */
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

static void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}

static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_calc(&imu_temp_pid, temp, 45);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        imu_pwm_set(tempPWM);
    }
    else
    {
        /* 还没达到设定温度，一直满功率加热 */
        if (temp > 45)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                /* 达到设定温度，切换为 PID 控制 */
                first_temperate = 1;
                imu_temp_pid.iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

       //mu_pwm_set(MPU6500_TEMP_PWM_MAX - 1);
        imu_pwm_set(0);
    }
}

void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
    (*offset_time_count)++;
}

void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (*time_count == 0)
    {
        gyro_offset[0] = gyro_cali_offset[0];
        gyro_offset[1] = gyro_cali_offset[1];
        gyro_offset[2] = gyro_cali_offset[2];
    }
    gyro_offset_calc(gyro_offset, INS_gyro, time_count);

    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;
}

void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    /* 旧代码这里还有 IST8310_DRDY_Pin 分支：DM 板没有磁力计，已删除 */
}

/*
 * 启动一次 SPI DMA 收发。
 * 老代码在这里直接操作 DMA 寄存器（SPI1_DMA_init/enable），
 * H7 上改用 HAL 接口，完成/错误由 HAL_SPI_TxRxCpltCallback 处理。
 */
static void imu_cmd_spi_dma(void)
{
    /* 陀螺仪分支 */
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(accel_update_flag & (1 << IMU_SPI_SHFITS)) &&
        !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) &&
        (hspi2.State == HAL_SPI_STATE_READY))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive_DMA(&hspi2, gyro_dma_tx_buf, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    /* 加速度计分支 */
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) &&
        !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) &&
        (hspi2.State == HAL_SPI_STATE_READY))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive_DMA(&hspi2, accel_dma_tx_buf, accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }
    /* 温度分支 */
    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) &&
        !(accel_update_flag & (1 << IMU_SPI_SHFITS)) &&
        (hspi2.State == HAL_SPI_STATE_READY))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive_DMA(&hspi2, accel_temp_dma_tx_buf, accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

/* SPI DMA 收发完成：收尾 CS、启动下一笔、唤醒 IMU 任务（替代原 DMA 中断里的处理） */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI2)
    {
        return;
    }

    /* 陀螺仪读取完成 */
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
        gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }

    /* 加速度计读取完成 */
    if (accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
        accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    /* 温度读取完成 */
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
        accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    /* 有新数据就唤醒 INS 任务（原代码通过软件 EXTI GPIO_PIN_0 触发） */
    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
