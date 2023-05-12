#ifndef __USER_UART_
#define __USER_UART_

#include "stm32l0xx_hal.h"
#include "string.h"

#define COMPLETE_RECEIVE_UART_TIME_MS 9

#define LENGTH_BUFFER_UART 20

typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t buffer;
	uint8_t countBuffer;
	uint8_t sim_rx[LENGTH_BUFFER_UART];
}UART_BUFFER;

int8_t Check_CountBuffer_Complete_Uart(UART_BUFFER *sUart);

void Transmit_Data_Uart(UART_BUFFER *sUart, char* command);
void Delete_Buffer(UART_BUFFER *sUart);



#endif
