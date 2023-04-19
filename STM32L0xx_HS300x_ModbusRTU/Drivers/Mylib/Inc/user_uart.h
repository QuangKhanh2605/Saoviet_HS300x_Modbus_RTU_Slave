#ifndef __USER_UART_
#define __USER_UART_

#include "stm32l0xx_hal.h"
#include "string.h"

#define COMPLETE_RECEIVE_UART_TIME_MS 9

typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t buffer;
	uint16_t countBuffer;
	uint8_t sim_rx[20];
}UART_BUFFER;

int8_t Check_CountBuffer_Complete_Uart(UART_BUFFER *rx_uart);

void Transmit_Data_Uart(UART_BUFFER *rx_uart, char* command);
void Delete_Buffer(UART_BUFFER *rx_uart);



#endif
