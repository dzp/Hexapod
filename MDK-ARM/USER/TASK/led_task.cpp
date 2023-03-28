#include "led_task.h"
#include "cmsis_os.h"
#include "debug_uart.h"

extern "C"{
	
	
void LED_Task(void const * argument)
{
	while(1)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		APP_PRINT("HelloWorld\r\n");
		osDelay(100);
	}
}


}
