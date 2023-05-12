#include "user_uart.h"

uint16_t check_countBuffer_uart=0;
uint32_t Get_systick_countBuffer_uart=0;
uint16_t State_systick_countBuffer_uart=0;

int8_t Check_CountBuffer_Complete_Uart(UART_BUFFER *sUart)
{
	uint8_t answer=0;
	if(sUart->countBuffer > 0)
	{
		if(State_systick_countBuffer_uart == 0 )
		{
			Get_systick_countBuffer_uart = HAL_GetTick();
			check_countBuffer_uart = sUart->countBuffer;
			State_systick_countBuffer_uart = 1;
		}
		
		if(HAL_GetTick() - Get_systick_countBuffer_uart > COMPLETE_RECEIVE_UART_TIME_MS && State_systick_countBuffer_uart == 1)	
		{
			if(check_countBuffer_uart == sUart->countBuffer)
			{
				answer = 1;
			}
			else
			{
				Get_systick_countBuffer_uart = HAL_GetTick();
				check_countBuffer_uart = sUart->countBuffer;
			}
		}
	}
	else
	{
		State_systick_countBuffer_uart = 0;
	}
	return answer;
}

void Delete_Buffer(UART_BUFFER *sUart)
{
	sUart->countBuffer=0;
	sUart->buffer=0x00;
	for(uint8_t i = 0; i < LENGTH_BUFFER_UART; i++)
	{
		sUart->sim_rx[i] = 0x00;
	}
}


void Transmit_Data_Uart(UART_BUFFER *sUart, char* command)
{
	HAL_UART_Transmit(sUart->huart, (uint8_t *)command, (uint16_t)strlen(command), 1000);
	HAL_UART_Transmit(sUart->huart,(uint8_t *)"\r",(uint16_t)strlen("\r"),1000);
}

