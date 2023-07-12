#include "uart_modbus_rtu.h"

uint32_t baud_rate_value[8]={1200,2400,4800,9600,19200,38400,57600,115200};
char success[]="SUCCESS";
char error[]="ERROR";
char Slave_IF[21];

void Get_Length_Variable(uint8_t *length, uint32_t variable);
int8_t Terminal_Receive(UART_BUFFER *sUart2);
void Send_Data_Terminal(UART_BUFFER *sUart2, void *data);

void Packing_Frame(uint8_t data_frame[], uint8_t addr_stm32l0xx, uint16_t addr_register, uint16_t length, uint32_t baud_rate, int16_t Tem, int16_t Humi);
void AT_Command_IF(UART_BUFFER *sUart2, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t drop_tem, int16_t drop_humi);

/*
	@brief  Kiem tra chuoi Uart nhan duoc va phan hoi theo chuan Modbus RTU
	@param  sUart2 Struct uart
	@param  addr_stm32l0xx dia chi slave
	@param  baud_rate toc do baud rate
	@param  tem nhiet do HS300x
	@param  humi do am HS300x
	@param  drop_tem hieu chuan nhiet do HS300x
	@param  drop_humi hieu chuan do am HS300x
	@retval None
*/
uint8_t ModbusRTU_Slave(UART_BUFFER *sUart2, uint8_t *addr_stm32l0xx, uint32_t *baud_rate, int16_t tem, int16_t humi, int16_t drop_tem, int16_t drop_humi)
{
	uint8_t answer=0;
	if(sUart2->sim_rx[0] == *addr_stm32l0xx)
	{
		uint8_t frame[25]={0}; 
		sData sFrame;
		sFrame.Data_a8 = frame;
		uint16_t CRC_rx = sUart2->sim_rx[sUart2->countBuffer-1] << 8 | sUart2->sim_rx[sUart2->countBuffer-2];
		uint16_t CRC_check = ModRTU_CRC(&sUart2->sim_rx[0], sUart2->countBuffer-2);
		uint8_t FunCode = sUart2->sim_rx[1];
		if(CRC_check == CRC_rx)
		{
			answer=1;
			uint16_t addr_data = sUart2->sim_rx[2] << 8 | sUart2->sim_rx[3];
			uint8_t data_frame[16]={0};
			if(FunCode == 0x03)
			{
				if(tem != 0x7FFF && humi !=0x7FFF)
				{
					if(addr_data <= 0x07)
					{
						uint16_t length_register = (sUart2->sim_rx[4] << 8 | sUart2->sim_rx[5])*2;
						uint8_t length_check = (8 - addr_data) * 2;
						if(length_register >= 1 && length_register <= length_check)
						{
							Packing_Frame(data_frame, *addr_stm32l0xx, addr_data, length_register, *baud_rate, tem, humi);
							ModRTU_Slave_ACK_Read_Frame(&sFrame, *addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
						}
						else
						{
							Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
						}
					}
					else
					{
						Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
						Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else
				{
					Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_I2C_OR_SENSOR);
				}
			}
			else if(FunCode == 0x06)
			{
				if(addr_data == 0x0000)
				{
					int16_t addr = sUart2->sim_rx[4] << 8 | sUart2->sim_rx[5];
					if(addr > 0 && addr <= 255)
					{
						*addr_stm32l0xx = addr;
						FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, drop_tem, drop_humi);
						ModRTU_Slave_ACK_Write_Frame(&sFrame, *addr_stm32l0xx, FunCode, addr_data, 1, data_frame);
					}
					else
					{
						Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else if(addr_data == 0x0001)
				{
					int16_t tmp_baud_rate = sUart2->sim_rx[4] << 8 | sUart2->sim_rx[5];
					if(tmp_baud_rate >= 0 && tmp_baud_rate <= 7)
					{
						*baud_rate = baud_rate_value[tmp_baud_rate];
						Uart2_Init(sUart2, *baud_rate);
						HAL_UART_Receive_IT(sUart2->huart,&sUart2->buffer,1);
						FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, drop_tem, drop_humi);
						ModRTU_Slave_ACK_Write_Frame(&sFrame, *addr_stm32l0xx, FunCode, addr_data, 1, data_frame);
					}
					else
					{
						Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else
				{
					Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
				}
			}
			else
			{
				Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_FUNCTION_CODE);
			}
		}
		else
		{
			Response_Error(&sFrame, *addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_CHECK_CRC);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_UART_Transmit(sUart2->huart, sFrame.Data_a8, sFrame.Length_u16, 1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	return answer;
}

/*
	@brief  Thay doi dia chi, toc do baud rate va hieu chuan sensor tu cong terminal
	@param  sUart2 Struct uart
	@param  addr_stm32l0xx dia chi slave
	@param  baud_rate toc do baud rate
	@param  drop_tem hieu chuan nhiet do HS300x
	@param  drop_humi hieu chuan do am HS300x
	@retval None
*/
void Change_Baudrate_AddrSlave_Calib(UART_BUFFER *sUart2, uint8_t *addr_stm32l0xx, uint32_t *baud_rate, int16_t *drop_tem, int16_t *drop_humi)
{
	int8_t receive_ctrl=0;
	receive_ctrl = Terminal_Receive(sUart2);
	if(receive_ctrl == -1)
	{
		Send_Data_Terminal(sUart2, error);
	}
	
	if(receive_ctrl == 1)
	{
		*addr_stm32l0xx = 0x1A;
		*baud_rate=115200;
		Send_Data_Terminal(sUart2, success);
		
		Uart2_Init(sUart2, *baud_rate);
		HAL_UART_Receive_IT(sUart2->huart,&sUart2->buffer,1);
		FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
	}
	if(receive_ctrl == -2)
	{
		AT_Command_IF(sUart2, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
	}
	
	if(receive_ctrl == 2)
	{
		uint8_t i=6;
		uint8_t count=0;
		while( sUart2->sim_rx[i] >= '0' && sUart2->sim_rx[i] <= '9')
		{
			i++;
			count++;
			if(i == sUart2->countBuffer) break;
		}
		i--;
		if(count >0 && count <=3)
		{
			uint16_t tmp=0;
			if(count == 1) tmp = (sUart2->sim_rx[i] -48);
			if(count == 2) tmp = (sUart2->sim_rx[i] -48) + (sUart2->sim_rx[i-1] -48)*10 ;
			if(count == 3) tmp = (sUart2->sim_rx[i] -48) + (sUart2->sim_rx[i-1] -48)*10 + (sUart2->sim_rx[i-2] -48)*100;
			if(tmp > 0 && tmp <= 255)
			{
				*addr_stm32l0xx = tmp;
				Send_Data_Terminal(sUart2, success);
				FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
			}
			else
			{
				Send_Data_Terminal(sUart2, error);
			}
		}
		else
		{
			Send_Data_Terminal(sUart2, error);
		}
	}
	
	if(receive_ctrl == 3)
	{
		uint8_t i=6;
		uint8_t count=0;
		while( sUart2->sim_rx[i] >= '0' && sUart2->sim_rx[i] <= '7')
		{
			i++;
			count++;
			if(i == sUart2->countBuffer) break;
		}
		i--;
		if(count == 1)
		{
			uint8_t tmp = sUart2->sim_rx[i] - 48;
			*baud_rate=baud_rate_value[tmp];
			Send_Data_Terminal(sUart2, success);
			
			Uart2_Init(sUart2, *baud_rate);
			HAL_UART_Receive_IT(sUart2->huart,&sUart2->buffer,1);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Data_Terminal(sUart2, error);
		}
	}
	
	if(receive_ctrl == 4 || receive_ctrl == -4)
	{
		uint8_t i=8;
		uint8_t count=0;
		while( sUart2->sim_rx[i] >= '0' && sUart2->sim_rx[i] <= '9')
		{
			i++;
			count++;
			if(i == sUart2->countBuffer) break;
		}
		i--;
		
		if(count >0 && count <=3)
		{
			uint16_t tmp=0;
			if(count == 1) tmp = (sUart2->sim_rx[i] -48);
			if(count == 2) tmp = (sUart2->sim_rx[i] -48) + (sUart2->sim_rx[i-1] -48)*10 ;
			if(count == 3) tmp = (sUart2->sim_rx[i] -48) + (sUart2->sim_rx[i-1] -48)*10 + (sUart2->sim_rx[i-2] -48)*100;
			if(receive_ctrl == 4)
			{
				*drop_tem = tmp;
			}
			if(receive_ctrl == -4)
			{
				*drop_tem = 0 - tmp;
			}
			Send_Data_Terminal(sUart2, success);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Data_Terminal(sUart2, error);
		}
	}
	
	if(receive_ctrl == 5 || receive_ctrl == -5)
	{
		uint8_t i=8;
		uint8_t count=0;
		while( sUart2->sim_rx[i] >= '0' && sUart2->sim_rx[i] <= '9')
		{
			i++;
			count++;
			if(i == sUart2->countBuffer) break;
		}
		i--;
		if(count == 1)
		{
			uint8_t tmp=0;
			tmp = (sUart2->sim_rx[i] -48);
			if(receive_ctrl == 5)
			{
				*drop_humi = tmp;
			}
			if(receive_ctrl == -5)
			{
				*drop_humi = 0 - tmp;
			}
			
			Send_Data_Terminal(sUart2, success);
			FLASH_WritePage(0xA5, *addr_stm32l0xx, *baud_rate, *drop_tem, *drop_humi);
		}
		else
		{
			Send_Data_Terminal(sUart2, error);
		}
	}
}

/*
	@brief  Thay doi dia chi, toc do baud rate va hieu chuan sensor tu cong terminal
	@param  sUart2 Struct uart
	@param  addr_stm32l0xx dia chi slave
	@param  baud_rate toc do baud rate
	@param  drop_tem hieu chuan nhiet do HS300x
	@param  drop_humi hieu chuan do am HS300x
	@retval None
*/
void AT_Command_IF(UART_BUFFER *sUart2, uint8_t addr_stm32l0xx, uint32_t baud_rate, int16_t drop_tem, int16_t drop_humi)
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
		Slave_IF[j] = (addr_stm32l0xx % 10) + ASCII_NUMBER_VALUE;
		addr_stm32l0xx = addr_stm32l0xx /10;
		length_addr--;
		j--;
	}
	Slave_IF[i]=','; i += 1 + length_baud_rate; j = i - 1;
	
	while(length_baud_rate > 0)
	{
		Slave_IF[j] = (baud_rate % 10) + ASCII_NUMBER_VALUE;
		baud_rate = baud_rate /10;
		length_baud_rate--;
		j--;
	}
	
	Slave_IF[i]=',';
	if(drop_tem <0)      Slave_IF[i+1]='-';
	else if(drop_tem >0) Slave_IF[i+1]='+';
	else                 i--;	
	i += 2 + length_drop_tem; j = i - 1;
	while(length_drop_tem > 0)
	{
		Slave_IF[j] = (stamp_drop_tem % 10) + ASCII_NUMBER_VALUE;
		stamp_drop_tem = stamp_drop_tem /10;
		length_drop_tem--;
		j--;
	}
	
	Slave_IF[i]=',';
	if(drop_humi < 0)     Slave_IF[i+1]='-';
	else if(drop_humi >0) Slave_IF[i+1]='+'; 
	else                  i--;     
	i += 2 + length_drop_humi; j = i - 1;
	while(length_drop_humi > 0)
	{
		Slave_IF[j] = (stamp_drop_humi % 10) + ASCII_NUMBER_VALUE;
		stamp_drop_humi = stamp_drop_humi /10;
		length_drop_humi--;
		j--;
	}
	while(i<21)
	{
		Slave_IF[i]=' ';
		i++;
	}
	
	Send_Data_Terminal(sUart2, Slave_IF);
	Send_Data_Terminal(sUart2, success);
}

/*
	@brief  Lay do dai chuoi thu duoc
	@retval None
*/
void Get_Length_Variable(uint8_t *length, uint32_t variable)
{
	while(variable/10 >=1)
	{
		(*length)++;
		variable = variable / 10;
	}
}

/*
	@brief  Kiem tra chuoi thu duoc tu Uart
	@param  sUart2 Struct uart
	@return (0)  Khong dung cau truc
	@ruturn (-1) Chuoi bi sai cau truc
	@return (1)  Reset ID va baud rate ve gia tri mac dinh (ID=26, baud_rate=115200)
	@return (2)  Chinh sua ID
	@return (-2) Hoi thong tin cua Slave 
	@return (3)  Chinh sua Baud_rate
	@return (4)  Hieu chuan nhiet do (tang) 
	@return (-4) Hieu chuan nhiet do (giam)
	@return (5)  Hieu chuan do am (tang)
	@return (-5) Hieu chuan do am (giam)
*/
int8_t Terminal_Receive(UART_BUFFER *sUart2)
{
	uint8_t i=0;
	if(sUart2->sim_rx[i] == 'A' && sUart2->sim_rx[i+1] == 'T') i=i+2;
	else return 0;
	
	if(sUart2->sim_rx[i] == '+') i++;
	else return -1;
	
	if(sUart2->sim_rx[i] == 'R' && sUart2->sim_rx[i+1] == 'E' && sUart2->sim_rx[i+2] == 'S' && sUart2->sim_rx[i+3] == 'E' && sUart2->sim_rx[i+4] == 'T') 
	{
		if(i+5 == sUart2->countBuffer) return 1;
		else
		{
			while(i+5 < sUart2->countBuffer)
			{
				if(sUart2->sim_rx[i+5] >32) return -1;
				i++;
			}
			return 1;
		}
	}
	
	if(sUart2->sim_rx[i] == 'I' && sUart2->sim_rx[i+1] == 'D')
	{
		i=i+2;
		if(sUart2->sim_rx[i] == '=') return 2;
		return -1;
	}
	if(sUart2->sim_rx[i] == 'I' && sUart2->sim_rx[i+1] == 'F' && sUart2->sim_rx[i+2] == '?') 
	{
		if(i+3 == sUart2->countBuffer) return -2;
		else
		{
			while(i+3 < sUart2->countBuffer)
			{
				if(sUart2->sim_rx[i+3] >32) return -1;
				i++;
			}
			return -2;
		}
	}
	
	if(sUart2->sim_rx[i] == 'B' && sUart2->sim_rx[i+1] == 'R')
	{
		i=i+2;
		if(sUart2->sim_rx[i] == '=') return 3;
		return -1;
	}
	
	if(sUart2->sim_rx[i] == 'C' && sUart2->sim_rx[i+1] == 'L' && sUart2->sim_rx[i+2] == 'T')
	{
		i=i+3;
		if(sUart2->sim_rx[i] == '=')
		{
			i=i+1;
			if(sUart2->sim_rx[i] == '+') return 4;
			if(sUart2->sim_rx[i] == '-') return -4;
			return -1;
		}
		return -1;
	}
	
	if(sUart2->sim_rx[i] == 'C' && sUart2->sim_rx[i+1] == 'L' && sUart2->sim_rx[i+2] == 'H')
	{
		i=i+3;
		if(sUart2->sim_rx[i] == '=')
		{
			i=i+1;
			if(sUart2->sim_rx[i] == '+') return 5;
			if(sUart2->sim_rx[i] == '-') return -5;
			return -1;
		}
		return -1;
	}
	return 0;
}

/*
	@brief  Gui du lieu den terminal
	@param  sUart2 Struct uart can lam viec
	@param  data du lieu muon gui len server
	@retval None 
*/
void Send_Data_Terminal(UART_BUFFER *sUart2, void *data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_UART_Transmit(sUart2->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_UART_Transmit(sUart2->huart,(uint8_t *)data,(uint16_t)strlen(data),1000);
	HAL_UART_Transmit(sUart2->huart,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	
}

/*
	@brief  Dong goi ban tin	
	@param  data_frame khung ban tin de dong goi
	@param  addr_stm32l0xx dia chi cua slave
	@param  addr_register dia chi thanh ghi du lieu
	@param  length do dai can dong goi
	@param  baud_rate toc do baud rate
	@param  tem nhiet do HS300x
	@param  humi do am HS300x
*/
void Packing_Frame(uint8_t data_frame[], uint8_t addr_stm32l0xx, uint16_t addr_register, uint16_t length, uint32_t baud_rate, int16_t Tem, int16_t Humi)
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

/**
  @brief  Khoi tao Uart2
  @param  sUart2 Struct uart muon lam viec
	@param  baud_rate toc do baud rate
  @retval None
  */
void Uart2_Init(UART_BUFFER *sUart2, uint32_t baud_rate)
{
  sUart2->huart->Instance = USART2;
  sUart2->huart->Init.BaudRate = baud_rate;
  sUart2->huart->Init.WordLength = UART_WORDLENGTH_8B;
  sUart2->huart->Init.StopBits = UART_STOPBITS_1;
  sUart2->huart->Init.Parity = UART_PARITY_NONE;
  sUart2->huart->Init.Mode = UART_MODE_TX_RX;
  sUart2->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  sUart2->huart->Init.OverSampling = UART_OVERSAMPLING_16;
  sUart2->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  sUart2->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(sUart2->huart) != HAL_OK)
  {
     __disable_irq();
		while (1)
		{
		}
  }
}

/*
	@brief  Ghi du lieu vao Flash
	@param  Check sau kiem tra xem du lieu da duoc nap chua
	@param  data1, data2, data3, data4 du lieu muon ghi vao flash
	@retval None
*/
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

/*
	@brief  Doc byte tu Flash
	@param  addr dia chi byte muon doc trong flash
	@return du lieu tai dia chi muon doc
*/
uint32_t FLASH_ReadData32(uint32_t addr)
{
	uint32_t data = *(__IO uint32_t *)(addr);
	return data;
}
