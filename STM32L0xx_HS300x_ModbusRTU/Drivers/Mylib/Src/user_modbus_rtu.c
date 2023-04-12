/*
- Quy dinh giao thuc Modbus RTU cho ca Master và Slave: BKAII
- Giao thuc Modbus RTU: 
   * Mot ban tin Modbus RTU bao gom: 1 byte dia chi  -  1 byte mã hàm - n byte du lieu - 2 byte CRC
   * Chuc nang và vai trò cu the nhu sau:
    + Byte dia chi: 
                  -- xác dinh thiet bi mang dia chi duoc nhan du lieu (doi voi Slave) hoac du lieu nhan duoc tu dia chi nào (doi voi Master). 
                        Ðia chi này duoc quy dinh tu 0 - 254. 
                  -- Cac byte quy dinh ID se lay tu 0x81 - 0xFF.
                       Neu cac byte data roi vao cac byte tren se chuyen doi lai theo: 0x80 0x00-0x7F.
    + Byte mã hàm: duoc quy dinh tu Master, xác dinh yêu cau du lieu tu thiet bi Slave. 
                    Ví du mã 01: doc du lieu luu tru dang Bit, 
                             03: doc du lieu tuc thoi dang Byte, 
                             05: ghi du lieu 1 bit vào Slave, 
                             06: ghi du lieu vào Slave dang byte ...
                             15: ghi du lieu nhieu bit vào Slave ...
    + Byte du lieu: xác dinh du lieu trao doi giua Master và Slave.
            -- Ðoc du lieu:  
                Master:  2 byte dia chi du lieu - 2 byte do dài du lieu            
                Slave: 2 byte dia chi du lieu - 2 byte do dài du lieu - n byte du lieu doc duoc
            -- Ghi du lieu:   
                Master: 2 byte dia chi du lieu  - 2 byte do dài du lieu - n byte du lieu can ghi     
                Slave: 2 byte dia chi du lieu - 2 byte do dài du lieu  
    + Byte CRC: 2 byte kiem tra loi cua hàm truyen. cách tính giá tri cua Byte CRC 16 Bit
*/

#include "user_modbus_rtu.h"


/*======================== Funcion Crc ======================*/

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(uint8_t* buf, int len)
{

  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) 
  {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}


void ModRTU_Convert_Special_Byte (uint8_t* Buff_Source, uint16_t* length)
{
    uint8_t i = 0;
    uint16_t j = 0;
    uint16_t len = *(length);
    
    for(i = 1; i < len; i++)
      if(*(Buff_Source + i) >= 0x80)
      {
           //dich buff sang phia sau 1 byte
           for(j = len; j > (i + 1); j--)
             *(Buff_Source + j) = *(Buff_Source + j - 1);
           
           *(Buff_Source + i + 1) = *(Buff_Source + i) - 0x80;
           *(Buff_Source + i) = 0x80;
           
           len++;
           i++;
      }
    
    *(length) = len;
}

/*======================== Funcion Master ======================*/
void ModRTU_Master_Read_Frame (sData *pFrame, uint8_t Address, uint8_t FunCode, uint16_t Add_Data, uint16_t LengthData)
{
    uint16_t crc;
    uint16_t Count = 0;
    
    // Ðong goi frame
    //1 byte Add Slave
    pFrame->Data_a8[Count++] = Address;
    //1 byte Funcode
    pFrame->Data_a8[Count++] = FunCode;
    //2 byte Add Data
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data & 0xFF);
    //2 byte Data length
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData & 0xFF);
    //Tinh 2 byte Crc
    crc = ModRTU_CRC(&pFrame->Data_a8[0], Count); 
    //them 2 byte crc
    pFrame->Data_a8[Count++] = (uint8_t) (crc & 0xFF);     //dao byte
    pFrame->Data_a8[Count++] = (uint8_t) (crc>>8) & 0xFF;
    
    pFrame->Length_u16 = Count;
}


/*
    FuncTest: Master Read
*/

/*
uint8_t ModRTU_Master_Read (void)
{
//    // Chon chan DE sang send
//    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_SET);
//    HAL_Delay(10);
//    // Send
//    Result = HAL_UART_Transmit(&UART_485, &aTEMP[0], Count, 1000); 
//    //Dua DE ve Receive
//    HAL_GPIO_WritePin(DE_GPIO_PORT, DE_GPIO_PIN, GPIO_PIN_RESET);
//    
//    return Result;
    
    return 1;
}
*/

void ModRTU_Master_Write_Frame (sData *pFrame, uint8_t Address, uint8_t FunCode, uint16_t Add_Data, uint16_t LengthData, uint8_t* aData)
{
    uint16_t crc;
    uint16_t Count = 0;
    uint16_t i = 0;
    
    // Ðong goi frame
    //1 byte Add Slave
    pFrame->Data_a8[Count++] = Address;
    //1 byte Funcode
    pFrame->Data_a8[Count++] = FunCode;
    //2 byte Add Data
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data & 0xFF);
    //2 byte Data length
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData & 0xFF);
    //n byte data
    for(i = 0; i < LengthData; i++)
        pFrame->Data_a8[Count++] = *(aData + i);
    //Tinh 2 byte Crc
    crc = ModRTU_CRC(&pFrame->Data_a8[0], Count); 
    //them 2 byte crc
    pFrame->Data_a8[Count++] = (uint8_t) (crc & 0xFF);
    pFrame->Data_a8[Count++] = (uint8_t) (crc>>8) & 0xFF;
    
    pFrame->Length_u16 = Count;
}


/*======================== Funcion Slave ======================*/
void ModRTU_Slave_ACK_Read_Frame (sData *pFrame, uint8_t Address, uint8_t FunCode, uint16_t Add_Data, uint16_t LengthData, uint8_t* aData)
{
    uint16_t crc;
    uint16_t Count = 0;
    uint16_t i = 0;
    
    // Ðong goi frame
    //1 byte Add Slave
    pFrame->Data_a8[Count++] = Address;
    //1 byte Funcode
    pFrame->Data_a8[Count++] = FunCode;
    //2 byte Add Data
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data & 0xFF);
    //2 byte Data length
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData & 0xFF);
    //n byte data
    for(i = 0; i < LengthData; i++)
    pFrame->Data_a8[Count++] = *(aData + i);
    //Tinh 2 byte Crc
    crc = ModRTU_CRC(&pFrame->Data_a8[0],Count); 
    //them 2 byte crc
    pFrame->Data_a8[Count++] = (uint8_t) (crc & 0xFF);
    pFrame->Data_a8[Count++] = (uint8_t) (crc>>8) & 0xFF;
    
    pFrame->Length_u16 = Count;
}


void ModRTU_Slave_ACK_Write_Frame (sData *pFrame, uint8_t Address, uint8_t FunCode, uint16_t Add_Data, uint16_t LengthData)
{
    uint16_t crc;
    uint16_t Count = 0;

    // Ðong goi frame
    //1 byte Add Slave
    pFrame->Data_a8[Count++] = Address;
    //1 byte Funcode
    pFrame->Data_a8[Count++] = FunCode;
    //2 byte Add Data
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (Add_Data & 0xFF);
    //2 byte Data length
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData>>8) & 0xFF;
    pFrame->Data_a8[Count++] = (uint8_t) (LengthData & 0xFF);
    //Tinh 2 byte Crc
    crc = ModRTU_CRC(&pFrame->Data_a8[0],Count); 
    //them 2 byte crc
    pFrame->Data_a8[Count++] = (uint8_t) (crc & 0xFF);
    pFrame->Data_a8[Count++] = (uint8_t) (crc>>8) & 0xFF;

    pFrame->Length_u16 = Count;
}

void Response_Error_CRC(sData *pFrame, uint8_t Address, uint8_t FunCode, uint8_t Error_Code)
{
	uint16_t Count = 0;
	
	// Ðong goi frame
  //1 byte Add Slave
	pFrame->Data_a8[Count++] = Address;
	//1 byte Funcode
  pFrame->Data_a8[Count++] = FunCode;
	//1 byte Error Code
  pFrame->Data_a8[Count++] = Error_Code;
}



