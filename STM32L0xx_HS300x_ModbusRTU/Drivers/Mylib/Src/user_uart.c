#include "user_uart.h"

uint16_t check_countBuffer_uart=0;
uint32_t Get_systick_countBuffer_uart=0;
uint16_t State_systick_countBuffer_uart=0;

/*
	@brief  Kiem tra da nhan xong Uart hay chua 
	@param  sUart Struct uart muon kiem tra
	@return (0) chua nhan xong
	@return (1) da nhan xong
*/
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
		else
		{
			if(HAL_GetTick() - Get_systick_countBuffer_uart > COMPLETE_RECEIVE_UART_TIME_MS)	
			{
				if(check_countBuffer_uart == sUart->countBuffer)
				{
					//Get_systick_countBuffer_uart = HAL_GetTick();
					answer = 1;
				}
				else
				{
					Get_systick_countBuffer_uart = HAL_GetTick();
					check_countBuffer_uart = sUart->countBuffer;
				}
			}
			else
			{
				if(check_countBuffer_uart != sUart->countBuffer)
				{
					Get_systick_countBuffer_uart = HAL_GetTick();
					check_countBuffer_uart = sUart->countBuffer;
				}
			}
		}
	}
	else
	{
		State_systick_countBuffer_uart = 0;
	}
	return answer;
}

/*
	@brief  Xoa chuoi Uart nhan duoc 
	@retval None
*/
void Delete_Buffer(UART_BUFFER *sUart)
{
	sUart->countBuffer=0;
	sUart->buffer=0x00;
	for(uint8_t i = 0; i < LENGTH_BUFFER_UART; i++)
	{
		sUart->sim_rx[i] = 0x00;
	}
}

/*
	@brief  Truyen chuoi qua Uart 
	@retval None
*/
void Transmit_Data_Uart(UART_BUFFER *sUart, char* command)
{
	HAL_UART_Transmit(sUart->huart, (uint8_t *)command, (uint16_t)strlen(command), 1000);
	HAL_UART_Transmit(sUart->huart,(uint8_t *)"\r",(uint16_t)strlen("\r"),1000);
}

/*
	@brief  Truyen chuoi qua Uart theo length
	@retval None
*/
void Transmit_Data_Uart_Length(UART_BUFFER *sUart, char data[], uint16_t length)																								
{
	HAL_UART_Transmit(sUart->huart,(uint8_t *)data, length, 1000);
	HAL_UART_Transmit(sUart->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
}

