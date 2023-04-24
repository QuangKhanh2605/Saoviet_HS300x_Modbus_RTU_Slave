/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	
/*

Change "baud_rate;" To "MX_USART2_UART_Init();" 

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HS300x.h"
#include "user_modbus_rtu.h"
#include "user_uart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUMBER_OF_REGISTER 8
#define TIME_SAMPLING      500
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t GetTick_Ms=0;
int16_t Tem=0xFF;
int16_t Humi=0xFF;

char Tem_Humi[16];

uint8_t aTemperature[2];
uint8_t aHumidity[2];

uint8_t address=0;

UART_BUFFER sUart2;

uint8_t check_flash=0;
uint32_t FLASH_startPage_data = 0x08000000 + 1024*15;
uint32_t baud_rate      = 115200;
uint8_t addr_stm32l0xx = 0x1A;

uint16_t addr_baud_rate        = 0x01;

uint16_t addr_tem              = 0x02;
uint16_t addr_tem_unit         = 0x03;
uint16_t addr_tem_decimal      = 0x04;

uint16_t addr_humi             = 0X05;
uint16_t addr_humi_uint        = 0X06;
uint16_t addr_humi_decimal     = 0x07;

uint32_t baud_rate_value[8]={1200,2400,4800,9600,19200,38400,57600,115200};
uint8_t calib_sensor[5]={0,0,0,0,0};
char success[]="SUCCESS";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ModbusRTU_Slave(void);
void Change_Baudrate_AddrSlave(void);
void HAL_SYSTICK_Callback(void);
void Packing_Frame(uint8_t data_frame[], uint16_t addr_register, uint16_t length);
void FLASH_WritePage(uint32_t check, uint32_t data1, uint32_t data2);
uint32_t FLASH_ReadData32(uint32_t addr);
uint8_t Terminal_Receive(void);
void send_success(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */




	sUart2.huart = &huart2;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
	for(int i=1;i<128;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, i<<1, 5,5) == HAL_OK)
		{
			address=i;
		}
	}
	check_flash=FLASH_ReadData32(FLASH_startPage_data);
	if(check_flash > 0)
	{
		addr_stm32l0xx=FLASH_ReadData32(FLASH_startPage_data+4);
		baud_rate     =FLASH_ReadData32(FLASH_startPage_data+8); 
		MX_USART2_UART_Init();
		HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(GetTick_Ms > HAL_GetTick()) GetTick_Ms=0;
		if(HAL_GetTick() - GetTick_Ms > TIME_SAMPLING) 
		{
			if(HS300X_Start_Measurement(&hi2c1, (int16_t*)&Tem, (int16_t*)&Humi)==1)
			{
				Tem=0xFF;
				Humi=0xFF;
			}
			aTemperature[0] = Tem >> 8;
			aTemperature[1] = Tem;
			aHumidity[0]    = 0x00;
			aHumidity[1]    = Humi;
			GetTick_Ms = HAL_GetTick();
		}
		ModbusRTU_Slave();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = baud_rate;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ModbusRTU_Slave(void)
{
	if(Check_CountBuffer_Complete_Uart(&sUart2)==1)
	{
		Change_Baudrate_AddrSlave();
		if(sUart2.sim_rx[0] == addr_stm32l0xx)
		{
			uint8_t frame[128];
			sData sFrame;
			sFrame.Data_a8 = frame;
			uint16_t CRC_rx = sUart2.sim_rx[sUart2.countBuffer-1] << 8 | sUart2.sim_rx[sUart2.countBuffer-2];
			uint16_t CRC_check = ModRTU_CRC(&sUart2.sim_rx[0], sUart2.countBuffer-2);
			uint8_t FunCode=sUart2.sim_rx[1];
			if(CRC_check == CRC_rx)
			{
				uint16_t addr_data = sUart2.sim_rx[2] << 8 | sUart2.sim_rx[3];
				uint16_t length_register = sUart2.sim_rx[4] << 8 | sUart2.sim_rx[5];
				length_register = length_register*2;
				uint8_t data_frame[20];
				if(FunCode == 0x03)
				{
					if(addr_data <= 0x07 && length_register >= 1)
					{
						Packing_Frame(data_frame, addr_data, length_register);
						ModRTU_Slave_ACK_Read_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
					}
					else
					{
						Response_Error_CRC(&sFrame, addr_stm32l0xx, (uint16_t) (0x80 + FunCode), ERROR_CODE_ADDRESS_OR_QUANTITY);
					}
				}
				else if(FunCode == 0x06)
				{
					for(uint8_t i=0;i<2;i++)
					{
						data_frame[i]= sUart2.sim_rx[4+i];
					}
					
					if(addr_data == 0x0000 && length_register >= 1)
					{
						addr_stm32l0xx = data_frame[0] << 8 | data_frame[1];
						FLASH_WritePage(1, addr_stm32l0xx, baud_rate);
					}
					
					if(addr_data == 0x0001 && length_register >= 1)
					{
						uint16_t tmp_baud_rate = data_frame[0] << 8 | data_frame[1];
						baud_rate = baud_rate_value[tmp_baud_rate];
						MX_USART2_UART_Init();
						HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
						FLASH_WritePage(1, addr_stm32l0xx, baud_rate);
					}
					
					if(addr_data <= 0x0001 && length_register >= 1)
					{
						ModRTU_Slave_ACK_Write_Frame(&sFrame, addr_stm32l0xx, FunCode, addr_data, length_register/2, data_frame);
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
			HAL_UART_Transmit(&huart2, sFrame.Data_a8, sFrame.Length_u16, 1000);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		}
		Delete_Buffer(&sUart2);
	}
}
void Change_Baudrate_AddrSlave(void)
{
	uint8_t receive_ctrl=0;
	receive_ctrl=Terminal_Receive();
	if(receive_ctrl == 1)
	{
		addr_stm32l0xx = 0x1A;
		baud_rate=115200;
		send_success();
		
		MX_USART2_UART_Init();
		HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
		FLASH_WritePage(1, addr_stm32l0xx, baud_rate);
	}
	
	if(receive_ctrl == 2)
	{
		uint8_t i=0;
		uint8_t count=0;
		while(sUart2.sim_rx[i] != '=')
		{
			i++;
		}
		i++;
		while(sUart2.sim_rx[i] == ' ')
		{
			i++;
		}
		while( sUart2.sim_rx[i] >= '0' && sUart2.sim_rx[i] <= '9' && i<sUart2.countBuffer)
		{
			i++;
			count++;
		}
		i--;
		if(count >0 && count <=3)
		{
			uint8_t tmp=0;
			if(count == 1) tmp = (sUart2.sim_rx[i] -48);
			if(count == 2) tmp = (sUart2.sim_rx[i] -48) + (sUart2.sim_rx[i-1] -48)*10 ;
			if(count == 3) tmp = (sUart2.sim_rx[i] -48) + (sUart2.sim_rx[i-1] -48)*10 + (sUart2.sim_rx[i-2] -48)*100;
			if(tmp > 0 && tmp <= 255)
			{
				addr_stm32l0xx = tmp;
				send_success();
				FLASH_WritePage(1, addr_stm32l0xx, baud_rate);
			}
		}
	}
	
	if(receive_ctrl == 3)
	{
		uint8_t i=0;
		while(sUart2.sim_rx[i] != '=')
		{
			i++;
		}
		i++;
		while(sUart2.sim_rx[i] == ' ')
		{
			i++;
		}
		if(sUart2.sim_rx[i] >= '0' && sUart2.sim_rx[i] <= '7' && i<sUart2.countBuffer)
		{
			uint8_t tmp = sUart2.sim_rx[i] - 48;
			baud_rate=baud_rate_value[tmp];
			send_success();
			
			MX_USART2_UART_Init();
			HAL_UART_Receive_IT(sUart2.huart,&sUart2.buffer,1);
			FLASH_WritePage(1, addr_stm32l0xx, baud_rate);
		}
	}
}

void send_success(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2,(uint8_t *)success,(uint16_t)strlen(success),1000);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n",(uint16_t)strlen("\r\n"),1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

uint8_t Terminal_Receive(void)
{
	uint16_t i=0;
	while(sUart2.sim_rx[i] == ' ' )
	{
		i++;
	}
	if(sUart2.sim_rx[i] == 'A' && sUart2.sim_rx[i+1] == 'T') i=i+2;
	else return 0;
	
	while(sUart2.sim_rx[i] == ' ')
	{
		i++;
	}
	if(sUart2.sim_rx[i] == '+') i++;
	else return 0;
	
	while(sUart2.sim_rx[i] == ' ')
	{
		i++;
	}
	if(sUart2.sim_rx[i] == 'R' && sUart2.sim_rx[i+1] == 'E' && sUart2.sim_rx[i+2] == 'S' && sUart2.sim_rx[i+3] == 'E' && sUart2.sim_rx[i+4] == 'T') return 1;
	if(sUart2.sim_rx[i] == 'I' && sUart2.sim_rx[i+1] == 'D')
	{
		i=i+2;
		while(sUart2.sim_rx[i] == ' ')
		{
			i++;
		}
		if(sUart2.sim_rx[i] == '=') return 2;
	}
	
	if(sUart2.sim_rx[i] == 'B' && sUart2.sim_rx[i+1] == 'R')
	{
		i=i+2;
		while(sUart2.sim_rx[i] == ' ')
		{
			i++;
		}
		if(sUart2.sim_rx[i] == '=') return 3;
	}
	return 0;
}
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	
	if(huart->Instance == huart2.Instance)
	{
		sUart2.sim_rx[(sUart2.countBuffer)++]= sUart2.buffer;
		HAL_UART_Receive_IT(&huart2,&sUart2.buffer,1);
	}
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

void Packing_Frame(uint8_t data_frame[], uint16_t addr_register, uint16_t length)
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
		data_frame[i]=aTemperature[0];
		i++;
		data_frame[i]=aTemperature[1];
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
		data_frame[i]=aHumidity[0];
		i++;
		data_frame[i]=aHumidity[1];
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

void FLASH_WritePage(uint32_t check, uint32_t data1, uint32_t data2)
{
  HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = FLASH_startPage_data;
	EraseInit.NbPages = (1024)/FLASH_PAGE_SIZE;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_startPage_data , check);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_startPage_data + 4, data1); //4 byte dau tien
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_startPage_data + 8, data2); //4 byte tiep theo
  HAL_FLASH_Lock();
}

uint32_t FLASH_ReadData32(uint32_t addr)
{
	uint32_t data = *(__IO uint32_t *)(addr);
	return data;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
