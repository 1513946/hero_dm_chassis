#include "BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

/*
 * Microsecond delay based on the DWT cycle counter.
 * - Does not depend on SysTick: before osKernelStart() SysTick does not count,
 *   the original SysTick polling version hangs there forever.
 * - Does not hard-code the CPU clock: the original used "us * 168" (F4 @168MHz),
 *   this MCU runs at 480MHz.
 * Requires DWT_CYCCNT enabled in main() (see USER CODE BEGIN 2).
 */
void BMI088_delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (uint32_t)us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

