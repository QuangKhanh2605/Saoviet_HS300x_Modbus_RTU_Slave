#include "uart_modbus_rtu.h"

uint32_t baud_rate_value[8]={1200,2400,4800,9600,19200,38400,57600,115200};
char success[]="SUCCESS";
char error[]="ERROR";
char Slave_IF[25];

void Get_Length_Variable(uint8_t *length, uint16_t variable);
int8_t Terminal_Receive(UART_BUFFER *rx_uart);
void Send_Success(UART_BUFFER *rx_uart);
void Send_Error(UART_BUFFER *rx_uart);
void Send_Slave_IF(UART_BUFFER *rx_uart);

void Packing_Frame(uint8_t data_frame[], uint16_t addr_register, uint16_t length, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t Tem, int16_t Humi);
void AT_Command_IF(UART_BUFFER *rx_uart, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t drop_tem, int16_t drop_humi);

void ModbusRTU_Slave(UART_BUFFER *rx_uart, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t tem, int16_t humi, int16_t drop_tem, int16_t drop_humi)
{
	if(rx_uart->sim_rx[0] == addr_stm32l0xx)
	{
		uint8_t frame[128]={0}; 
		sData sFrame;
		sFrame.Data_a8 = frame;
		uint16_t CRC_rx = rx_uart->sim_rx[rx_uart->countBuffer-1] << 8 | rx_uart->sim_rx[rx_uart->countBuffer-2];
		uint16_t CRC_check = ModRTU_CRC(&rx_uart->sim_rx[0], rx_uart->countBuffer-2);
		uint8_t FunCode=rx_uart->sim_rx[1];
		if(CRC_check == CRC_rx)
		{
			uint16_t addr_data = rx_uart->sim_rx[2] << 8 | rx_uart->sim_rx[3];
			uint16_t length_register = rx_uart->sim_rx[4] << 8 | rx_uart->sim_rx[5];
			length_register = length_register*2;
			uint8_t data_frame[20]={0};
			if(FunCode == 0x03)
			{
				if(tem != 0xFF && humi !=0xFF)
				{
					if(addr_data <= 0x07)
					{
						uint8_t length_check = (8 - addr_data) * 2;
						if(length_register >= 1 && length_register <= length_check)
						{
							Packing_Frame(data_frame, addr_data, addr_stm32l0xx, baud_rate, length_register, tem, humi);
							ModRTU_Slave_ACK_Read_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
						}
						else
						{
							Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
						}
					}
					else
					{
						Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else
				{
					Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_I2C);
				}
			}
			else if(FunCode == 0x06)
			{
				for(uint8_t i=0;i<(length_register * 2);i++)
				{
					data_frame[i]= rx_uart->sim_rx[4+i];
				}
				
				if(addr_data == 0x0000)
				{
					if(length_register == 1)
					{
						addr_stm32l0xx = data_frame[0] << 8 | data_frame[1];
						FLASH_WritePage(0xA5, addr_stm32l0xx, baud_rate, drop_tem, drop_humi);
						ModRTU_Slave_ACK_Write_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
					}
//					else if(length_register == 2)
//					{
//						addr_stm32l0xx = data_frame[0] << 8 | data_frame[1];
//						uint16_t tmp_baud_rate = data_frame[2] << 8 | data_frame[3];
//						baud_rate = baud_rate_value[tmp_baud_rate];
//						Uart2_Init(rx_uart,baud_rate);
//						HAL_UART_Receive_IT(rx_uart->huart,&rx_uart->buffer,1);
//						FLASH_WritePage(0xA5, addr_stm32l0xx, baud_rate, drop_tem, drop_humi);
//						ModRTU_Slave_ACK_Write_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
//					}
					else
					{
						Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				
				else if(addr_data == 0x0001)
				{
					if(length_register == 1)
					{
						uint16_t tmp_baud_rate = data_frame[0] << 8 | data_frame[1];
						baud_rate = baud_rate_value[tmp_baud_rate];
						Uart2_Init(rx_uart, baud_rate);
						HAL_UART_Receive_IT(rx_uart->huart,&rx_uart->buffer,1);
						FLASH_WritePage(0xA5, addr_stm32l0xx, baud_rate, drop_tem, drop_humi);
						ModRTU_Slave_ACK_Write_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
					}
					else
					{
						Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else
				{
					Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
				}
			}
			else
			{
				Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_FUNCTION_CODE);
			}
		}
		else
		{
			Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_CHECK_CRC);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_UART_Transmit(rx_uart->huart, sFrame.Data_a8, sFrame.Length_u16, 1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	
}

void Change_Baudrate_AddrSlave(UART_BUFFER *rx_uart, uint8_t *addr_stm32l0xx, uint32_t *baud_rate, int16_t *drop_tem, int16_t *drop_humi)
{
	int8_t receive_ctrl=0;
	receive_ctrl = Terminal_Receive(rx_uart);
	if(receive_ctrl == -1)
	{
		Send_Error(rx_uart);
	}
	
	if(receive_ctrl == 1)
	{
		*addr_stm32l0xx = 0x1A;
		*baud_rate=115200;
		Send_Success(rx_uart);
		
		Uart2_Init(rx_uart, *baud_rate);
		HAL_UART_Receive_IT(rx_uart->huart,&rx_uart->buffer,1);
		FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
	}
	if(receive_ctrl == -2)
	{
		AT_Command_IF(rx_uart, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
	}
	
	if(receive_ctrl == 2)
	{
		uint8_t i=6;
		uint8_t count=0;
		while( rx_uart->sim_rx[i] >= '0' && rx_uart->sim_rx[i] <= '9')
		{
			i++;
			count++;
			if(i == rx_uart->countBuffer) break;
		}
		i--;
		if(count >0 && count <=3)
		{
			uint16_t tmp=0;
			if(count == 1) tmp = (rx_uart->sim_rx[i] -48);
			if(count == 2) tmp = (rx_uart->sim_rx[i] -48) + (rx_uart->sim_rx[i-1] -48)*10 ;
			if(count == 3) tmp = (rx_uart->sim_rx[i] -48) + (rx_uart->sim_rx[i-1] -48)*10 + (rx_uart->sim_rx[i-2] -48)*100;
			if(tmp > 0 && tmp <= 255)
			{
				*addr_stm32l0xx = tmp;
				Send_Success(rx_uart);
				FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
			}
			else
			{
				Send_Error(rx_uart);
			}
		}
		else
		{
			Send_Error(rx_uart);
		}
	}
	
	if(receive_ctrl == 3)
	{
		uint8_t i=6;
		if(rx_uart->sim_rx[i] >= '0' && rx_uart->sim_rx[i] <= '7' )
		{
			uint8_t tmp = rx_uart->sim_rx[i] - 48;
			*baud_rate=baud_rate_value[tmp];
			Send_Success(rx_uart);
			
			Uart2_Init(rx_uart, *baud_rate);
			HAL_UART_Receive_IT(rx_uart->huart,&rx_uart->buffer,1);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Error(rx_uart);
		}
	}
	
	if(receive_ctrl == 4 || receive_ctrl == -4)
	{
		uint8_t i=8;
		uint8_t count=0;
		while( rx_uart->sim_rx[i] >= '0' && rx_uart->sim_rx[i] <= '9')
		{
			i++;
			count++;
			if(i == rx_uart->countBuffer) break;
		}
		i--;
		
		if(count >0 && count <=3)
		{
			uint16_t tmp=0;
			if(count == 1) tmp = (rx_uart->sim_rx[i] -48);
			if(count == 2) tmp = (rx_uart->sim_rx[i] -48) + (rx_uart->sim_rx[i-1] -48)*10 ;
			if(count == 3) tmp = (rx_uart->sim_rx[i] -48) + (rx_uart->sim_rx[i-1] -48)*10 + (rx_uart->sim_rx[i-2] -48)*100;
			if(receive_ctrl == 4)
			{
				*drop_tem = tmp;
			}
			if(receive_ctrl == -4)
			{
				*drop_tem = 0 - tmp;
			}
			Send_Success(rx_uart);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Error(rx_uart);
		}
	}
	
	if(receive_ctrl == 5 || receive_ctrl == -5)
	{
		uint8_t i=8;
		uint8_t count=0;
		while( rx_uart->sim_rx[i] >= '0' && rx_uart->sim_rx[i] <= '9' )
		{
			i++;
			count++;
			if(i == rx_uart->countBuffer) break;
		}
		i--;
		
		if(count >0)
		{
			uint8_t tmp=0;
			tmp = (rx_uart->sim_rx[i] -48);
			if(receive_ctrl == 5)
			{
				*drop_humi = tmp;
			}
			if(receive_ctrl == -5)
			{
				*drop_humi = 0 - tmp;
			}
			
			Send_Success(rx_uart);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Error(rx_uart);
		}
	}
}

void AT_Command_IF(UART_BUFFER *rx_uart, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t drop_tem, int16_t drop_humi)
{
	uint8_t i=0;
	uint8_t j=0;
	uint8_t length_addr=1;
	uint8_t length_baud_rate=1;
	uint8_t length_drop_tem=1;
	uint8_t length_drop_humi=1;
	uint16_t stamp_drop_tem=0;
	uint16_t stamp_drop_humi=0;
	
	if(drop_tem < 0) stamp_drop_tem = (uint16_t)drop_tem*(-1);
	else             stamp_drop_tem = drop_tem;
	
	if(drop_humi < 0) stamp_drop_humi = (uint16_t)drop_humi*(-1);
	else              stamp_drop_humi = drop_humi;
	
	Get_Length_Variable(&length_addr,      addr_stm32l0xx);
	Get_Length_Variable(&length_baud_rate, baud_rate);
	Get_Length_Variable(&length_drop_tem,  stamp_drop_tem);
	Get_Length_Variable(&length_drop_humi, stamp_drop_humi);
	
	Slave_IF[i]='I'; Slave_IF[i+1]='F'; Slave_IF[i+2]= '='; 
	i += 3 + length_addr; j = i - 1;
	while(length_addr > 0)
	{
		Slave_IF[j] = (addr_stm32l0xx % 10) + 48;
		addr_stm32l0xx = addr_stm32l0xx /10;
		length_addr--;
		j--;
	}
	Slave_IF[i]=','  ; Slave_IF[i+1]=' '; i += 2 + length_baud_rate; j = i - 1;
	
	while(length_baud_rate > 0)
	{
		Slave_IF[j] = (baud_rate % 10) + 48;
		baud_rate = baud_rate /10;
		length_baud_rate--;
		j--;
	}
	
	Slave_IF[i]=',';     Slave_IF[i+1]=' ';
	if(drop_tem <0)      Slave_IF[i+2]='-';
	else if(drop_tem >0) Slave_IF[i+2]='+';
	else                 i--;	
	i += 3 + length_drop_tem; j = i - 1;
	while(length_drop_tem > 0)
	{
		Slave_IF[j] = (stamp_drop_tem % 10) + 48;
		stamp_drop_tem = stamp_drop_tem /10;
		length_drop_tem--;
		j--;
	}
	
	Slave_IF[i]=',';      Slave_IF[i+1]=' ';
	if(drop_humi < 0)     Slave_IF[i+2]='-';
	else if(drop_humi >0) Slave_IF[i+2]='+'; 
	else                  i--;     
	i += 3 + length_drop_humi; j = i - 1;
	while(length_drop_humi > 0)
	{
		Slave_IF[j] = (stamp_drop_humi % 10) + 48;
		stamp_drop_humi = stamp_drop_humi /10;
		length_drop_humi--;
		j--;
	}
	while(i<25)
	{
		Slave_IF[i]=' ';
		i++;
	}
	
	Send_Slave_IF(rx_uart);
	Send_Success(rx_uart);
}

void Get_Length_Variable(uint8_t *length, uint16_t variable)
{
	while(variable/10 >=1)
	{
		(*length)++;
		variable = variable / 10;
	}
}

int8_t Terminal_Receive(UART_BUFFER *rx_uart)
{
	uint16_t i=0;
	
	if(rx_uart->sim_rx[i] == 'A' && rx_uart->sim_rx[i+1] == 'T') i=i+2;
	else return 0;
	
	if(rx_uart->sim_rx[i] == '+') i++;
	else return -1;
	
	if(rx_uart->sim_rx[i] == 'R' && rx_uart->sim_rx[i+1] == 'E' && rx_uart->sim_rx[i+2] == 'S' && rx_uart->sim_rx[i+3] == 'E' && rx_uart->sim_rx[i+4] == 'T') return 1;
	if(rx_uart->sim_rx[i] == 'I' && rx_uart->sim_rx[i+1] == 'D')
	{
		i=i+2;
		if(rx_uart->sim_rx[i] == '=') return 2;
		return -1;
	}
	if(rx_uart->sim_rx[i] == 'I' && rx_uart->sim_rx[i+1] == 'F') return -2;
	
	if(rx_uart->sim_rx[i] == 'B' && rx_uart->sim_rx[i+1] == 'R')
	{
		i=i+2;
		if(rx_uart->sim_rx[i] == '=') return 3;
		return -1;
	}
	
	if(rx_uart->sim_rx[i] == 'C' && rx_uart->sim_rx[i+1] == 'L' && rx_uart->sim_rx[i+2] == 'T')
	{
		i=i+3;
		if(rx_uart->sim_rx[i] == '=')
		{
			i=i+1;
			if(rx_uart->sim_rx[i] == '+') return 4;
			if(rx_uart->sim_rx[i] == '-') return -4;
			return -1;
		}
		return -1;
	}
	
	if(rx_uart->sim_rx[i] == 'C' && rx_uart->sim_rx[i+1] == 'L' && rx_uart->sim_rx[i+2] == 'H')
	{
		i=i+3;
		if(rx_uart->sim_rx[i] == '=')
		{
			i=i+1;
			if(rx_uart->sim_rx[i] == '+') return 5;
			if(rx_uart->sim_rx[i] == '-') return -5;
			return -1;
		}
		return -1;
	}
	return 0;
}

void Send_Slave_IF(UART_BUFFER *rx_uart)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)Slave_IF,(uint16_t)strlen(Slave_IF),1000);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	
}

void Send_Success(UART_BUFFER *rx_uart)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)success,(uint16_t)strlen(success),1000);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void Send_Error(UART_BUFFER *rx_uart)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)error,(uint16_t)strlen(error),1000);
	HAL_UART_Transmit(rx_uart->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void Packing_Frame(uint8_t data_frame[], uint16_t addr_register, uint16_t length, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t Tem, int16_t Humi)
{
	uint8_t i=0;
	// address slave
	if(addr_register <=0 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=addr_stm32l0xx;
		i++;
	}
	// baudrate
	if(addr_register <=1 && i<length)
	{
		uint8_t j=0;
		while(j<8)
		{
			if(baud_rate_value[j] == baud_rate)
			{
				break;
			}
			else j++;
		}
		data_frame[i]=0x00;
		i++;
		data_frame[i]=j;
		i++;
	}
	// temperature value
	if(addr_register <=2 && i<length)
	{
		data_frame[i]=Tem >> 8;
		i++;
		data_frame[i]=Tem;
		i++;
	}
	// temperature Unit
	if(addr_register <=3 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=0x01;
		i++;
	}
	// temperature decimal points
	if(addr_register <=4 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=0x02;
		i++;
	}
	// humidity value
	if(addr_register <=5 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=Humi;
		i++;
	}
	// humidity Uint
	if(addr_register <=6 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=0x01;
		i++;
	}
	// humidity decimal points 
	if(addr_register <=7 && i<length)
	{
		data_frame[i]=0x00;
		i++;
		data_frame[i]=0x00;
		i++;
	}
}

void Uart2_Init(UART_BUFFER *rx_uart, uint32_t baud_rate)
{
  rx_uart->huart->Instance = USART2;
  rx_uart->huart->Init.BaudRate = baud_rate;
  rx_uart->huart->Init.WordLength = UART_WORDLENGTH_8B;
  rx_uart->huart->Init.StopBits = UART_STOPBITS_1;
  rx_uart->huart->Init.Parity = UART_PARITY_NONE;
  rx_uart->huart->Init.Mode = UART_MODE_TX_RX;
  rx_uart->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  rx_uart->huart->Init.OverSampling = UART_OVERSAMPLING_16;
  rx_uart->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  rx_uart->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(rx_uart->huart) != HAL_OK)
  {
    while(1);
  }
}

void FLASH_WritePage(uint32_t check, uint32_t data1, uint32_t data2, uint32_t data3, uint32_t data4)
{
  HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = FLASH_STARTPAGE_DATA;
	EraseInit.NbPages = (1024)/FLASH_PAGE_SIZE;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STARTPAGE_DATA , check);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STARTPAGE_DATA + 4, data1); //4 byte dau tien
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STARTPAGE_DATA + 8, data2); //4 byte tiep theo
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STARTPAGE_DATA + 12, data3); //4 byte tiep theo
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STARTPAGE_DATA + 16, data4); //4 byte tiep theo
  HAL_FLASH_Lock();
}

uint32_t FLASH_ReadData32(uint32_t addr)
{
	uint32_t data = *(__IO uint32_t *)(addr);
	return data;
}
