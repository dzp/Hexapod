#include "Servo.h"
#include "gait_prg.h"

volatile uint8_t cmd_send_buffer[12] = {0x55, 0x55};

void Servo::set_angle(float angle)
{
	this->angle = angle / PI * 750 + 500; // 将弧度转换为舵机值
}

void Servo::set_time(uint16_t move_time)
{
	this->move_time = move_time;
}

// 计算校验和
static uint8_t check_sum(uint8_t data_len)
{
	static uint16_t checksum;
	checksum = cmd_send_buffer[2] + cmd_send_buffer[3] + cmd_send_buffer[4];
	for (int i = 0; i < data_len-3; i++)
	{
		checksum += cmd_send_buffer[5 + i];
	}
	uint8_t i;
	i = uint8_t(~checksum); // 取反
	return i; 
}

// 发送舵机移动指令
void Servo::move(UART_HandleTypeDef *huart)
{
	cmd_send_buffer[2] = this->id;
	cmd_send_buffer[3] = SERVO_MOVE_TIME_WRITE_LEN;
	cmd_send_buffer[4] = SERVO_MOVE_TIME_WRITE;
	cmd_send_buffer[5] = this->angle;
	cmd_send_buffer[6] = this->angle >> 8;
	cmd_send_buffer[7] = this->move_time;
	cmd_send_buffer[8] = this->move_time >> 8;
	cmd_send_buffer[9] = check_sum(SERVO_MOVE_TIME_WRITE_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_MOVE_TIME_WRITE_LEN + 3);
	//HAL_UART_Transmit(huart,(uint8_t *)cmd_send_buffer,SERVO_MOVE_TIME_WRITE_LEN + 3,1000);
}

// 设定舵机角度，但是需要等待开始指令才开始移动
void Servo::move_wait(UART_HandleTypeDef *huart)
{
	cmd_send_buffer[2] = this->id;
	cmd_send_buffer[3] = SERVO_MOVE_TIME_WAIT_WRITE;
	cmd_send_buffer[4] = SERVO_MOVE_TIME_WAIT_WRITE_LEN;
	cmd_send_buffer[5] = this->angle;
	cmd_send_buffer[6] = this->angle >> 8;
	cmd_send_buffer[7] = this->move_time;
	cmd_send_buffer[8] = this->move_time >> 8;
	cmd_send_buffer[9] = check_sum(SERVO_MOVE_TIME_WAIT_WRITE_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3);
}

/*************广播指令***********/
void Servo_Broad_Cast::move_start(UART_HandleTypeDef *huart)
{
	cmd_send_buffer[2] = SERVO_BROADCAST_ID;
	cmd_send_buffer[3] = SERVO_MOVE_START;
	cmd_send_buffer[4] = SERVO_MOVE_START_LEN;
	cmd_send_buffer[5] = check_sum(SERVO_MOVE_START_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_MOVE_START_LEN + 3);
}

void Servo_Broad_Cast::move_stop(UART_HandleTypeDef *huart)
{
	cmd_send_buffer[2] = SERVO_BROADCAST_ID;
	cmd_send_buffer[3] = SERVO_MOVE_STOP;
	cmd_send_buffer[4] = SERVO_MOVE_STOP_LEN;
	cmd_send_buffer[5] = check_sum(SERVO_MOVE_STOP_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_MOVE_STOP_LEN + 3);
}

void Servo_Broad_Cast::upload(UART_HandleTypeDef *huart) 
{
	cmd_send_buffer[2] = SERVO_BROADCAST_ID;
	cmd_send_buffer[3] = SERVO_LOAD_OR_UNLOAD_WRITE;
	cmd_send_buffer[4] = SERVO_LOAD_OR_UNLOAD_WRITE_LEN;
	cmd_send_buffer[5] = 0;  //0表示掉电
	cmd_send_buffer[6] = check_sum(SERVO_LOAD_OR_UNLOAD_WRITE_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);	
}

void Servo_Broad_Cast::load(UART_HandleTypeDef *huart) 
{
	cmd_send_buffer[2] = SERVO_BROADCAST_ID;
	cmd_send_buffer[3] = SERVO_LOAD_OR_UNLOAD_WRITE;
	cmd_send_buffer[4] = SERVO_LOAD_OR_UNLOAD_WRITE_LEN;
	cmd_send_buffer[5] = 1;  //1表示上电
	cmd_send_buffer[6] = check_sum(SERVO_LOAD_OR_UNLOAD_WRITE_LEN);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd_send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);	
}
