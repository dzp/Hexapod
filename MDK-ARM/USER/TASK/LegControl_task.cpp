#include "LegControl_task.h"
#include "gait_prg.h"
#include "cmsis_os.h"
#include "leg.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"

using namespace std;

uint32_t LegControl_round;
Leg legs[6];
//º¯Êý
static void Legs_Init(void);
extern "C"
{
	void LegControl_Task(void const *argument)
	{
		Legs_Init();
		//HAL_GPIO_WritePin(LEG1_TXE_GPIO_Port,LEG1_TXE_Pin,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(LEG2_TXE_GPIO_Port,LEG2_TXE_Pin,GPIO_PIN_RESET);
		osDelay(100);
		
		Thetas thetas;	
		while (1)
		{
			LegControl_round = (++LegControl_round) % N_POINTS;
			thetas.angle[0] = LegControl_round/10.0f;
			legs[0].set_time(100);
			//legs[0].load(legs[0].huart);
			legs[0].set_thetas(thetas);
			legs[0].move();
			osDelay(100);
		}
	}
}

static void Legs_Init(void)
{
	legs[0] = Leg(&huart1);
	legs[1] = Leg(&huart2);
	legs[2] = Leg(&huart3);
	legs[3] = Leg(&huart4);
	legs[4] = Leg(&huart5);
	legs[5] = Leg(&huart6);
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
}
