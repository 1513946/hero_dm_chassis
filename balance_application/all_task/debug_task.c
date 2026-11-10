#include "debug_task.h"
#include "ins_task.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern fp32 yaw;
extern fp32  pitch;
extern fp32 roll;
const fp32 *send_data[CH_COUNT];
fp32  a=0;
void Crg_test_Task(void const * argument)
{
	const fp32 *send_data[CH_COUNT] =
	{
		&yaw,
		&pitch,
		&roll,
		&a
	};
	while(1)
	{
		debug_to_vofa(send_data,CH_COUNT);
		a+=1.0f;
		vTaskDelay(10); //·¢ËÍÆµÂÊ100hz
	}
}
