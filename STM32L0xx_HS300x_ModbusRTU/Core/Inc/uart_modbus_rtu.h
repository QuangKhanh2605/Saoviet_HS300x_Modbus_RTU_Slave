#ifndef __UART_MODBUS_RTU
#define __UART_MODBUS_RTU

#define FLASH_STARTPAGE_DATA 0x08000000 + 1024*15

#include "user_modbus_rtu.h"
#include "user_uart.h"

void ModbusRTU_Slave(UART_BUFFER *sUart2, uint8_t *addr_stm32l0xx, uint32_t *baud_rate, int16_t tem, int16_t humi, int16_t drop_Tem, int16_t drop_humi);
void FLASH_WritePage(uint32_t check, uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4);
uint32_t FLASH_ReadData32(uint32_t addr);
void Uart2_Init(UART_BUFFER *sUart2, uint32_t baud_rate);
void Change_Baudrate_AddrSlave_Calib(UART_BUFFER *sUart2, uint8_t *addr_stm32l0xx, uint32_t *baud_rate, int16_t *drop_tem, int16_t *drop_humi);

#endif

