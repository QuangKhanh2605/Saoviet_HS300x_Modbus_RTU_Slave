
#include "HS300x.h"
//#include "var.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

extern uint32_t RT_Count_Systick_u32;
uint8_t Number_Error_HS300X_u8 = 0;
// uint8_t dataTest[16] = {0x00};
// uint8_t DRDY_INTflag = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/


/* USER CODE BEGIN 2 */
/* This function call after 10ms HS300x resart */
uint8_t HS300X_Init(I2C_HandleTypeDef *hi2c_x, TEMP_RESO Temperature_Resolution_x_bit, TEMP_HUMID Humidity_Resolution_x_bit)
{
    /* Temperature and Humidity are acquired in sequence
     * Default: 00: Temperature resolution = 14 bit,
     *          00: Humidity resolution = 14 bit
     * Obit register
     * Resolution register              Resolution
     *      bit[11:10]                    (bits)
     *          00B                       8
     *          01B                       10
     *          10B                       12
     *          11B                       14
     */
    /* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
    uint8_t configValue[4] = {0x00};
    uint8_t reVal = 0;
    /* goto programming mode - which must be sent within 10ms after applying power to the sensor */
    if (HAL_OK != HAL_I2C_Mem_Write(hi2c_x, HS300X_SENSON_ADDR << 1, HS300X_PROGRAMMING_MODE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&configValue[0], 2, 100))
    {
        return 0;
    }
    HAL_Delay(5);
    /*  Change the Relative Humidity Resolution */
    configValue[0] = HS300X_HUMIDITY_READ_RESO_REG;
    /* Read old config */
    if (HAL_OK == HAL_I2C_Master_Transmit(hi2c_x, HS300X_SENSON_ADDR << 1, (uint8_t *)&configValue[0], 3, 100))
    {
        HAL_Delay(15);
        if (HAL_OK == HAL_I2C_Master_Receive(hi2c_x, HS300X_SENSON_ADDR << 1, (uint8_t *)&configValue[0], 3, 100))
        {
            if (HS300X_STATUS_SUCCESS == configValue[0])
            {
                reVal = 1U;
            }
        }
    }
    /* write new humidity resolution */
    if(1U == reVal)
    {
        configValue[1] = (uint8_t)((configValue[1] & 0xF3) | Humidity_Resolution_x_bit);
        if (HAL_OK != HAL_I2C_Mem_Write(hi2c_x, HS300X_SENSON_ADDR << 1, HS300X_HUMIDITY_WRITE_RESO_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&configValue[1], 2, 100))
        {
            return 0;
        }
        HAL_Delay(15);
    }
    reVal = 0;
    /*  Change the Relative Temperature Resolution */
    configValue[0] = HS300X_TEMP_READ_RESO_REG;
    configValue[1] = 0x00;
    configValue[2] = 0x00;
    /* Read old config */
    if (HAL_OK == HAL_I2C_Master_Transmit(hi2c_x, HS300X_SENSON_ADDR << 1, (uint8_t *)&configValue[0], 3, 100))
    {
        HAL_Delay(15);
        if (HAL_OK == HAL_I2C_Master_Receive(hi2c_x, HS300X_SENSON_ADDR << 1, (uint8_t *)&configValue[0], 3, 100))
        {
            if (HS300X_STATUS_SUCCESS == configValue[0])
            {
                reVal = 1U;
            }
        }
    }
    /* write new temperature resolution */
    if(1U == reVal)
    {
        configValue[1] = (uint8_t)((configValue[1] & 0xF3) | Temperature_Resolution_x_bit);
        if (HAL_OK != HAL_I2C_Mem_Write(hi2c_x, HS300X_SENSON_ADDR << 1, HS300X_TEMP_WRITE_RESO_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&configValue[1], 2, 100))
        {
            return 0;
        }
        HAL_Delay(15);
    }
    return reVal;
}

uint8_t HS300X_Start_Measurement(I2C_HandleTypeDef *hi2c_x, int16_t *temperature, int16_t *humidity)
{
    uint8_t reVal = 0;
    uint8_t receive_data[4] = {0x00, 0x00, 0x00, 0x00};
    uint16_t temp_x,humi_x;

    if (HAL_OK == HAL_I2C_Master_Transmit(hi2c_x, HS300X_SENSON_ADDR << 1, &receive_data[0], 0, 100))
    {
        /*  Resolution Measurement
         *    (bit)     time(ms)
         *      8         1.20
         *     10         2.72
         *     12         9.10
         *     14        33.90
         *                      */
        /* wait sensor wake up */
        HAL_Delay(50);
        /* Read temperature and humidity */
        reVal = HAL_I2C_Master_Receive(hi2c_x, HS300X_SENSON_ADDR << 1, receive_data, 4, 1000);
        if (HAL_OK == reVal)
        {
            humi_x = ((receive_data[0] << 8) | receive_data[1]);
            temp_x = ((receive_data[2] << 8) | receive_data[3]);
            humi_x = humi_x & 0x3FFFU;
            temp_x = ((temp_x >> 2) & 0xFFFFU);
            *humidity = (int16_t)((humi_x / 16383.00) * 100.00);
            *temperature = (int16_t)((((temp_x / 16383.00) * 165.00) - 40.00) * 100);
            if (Number_Error_HS300X_u8 != 0)
            {
                Number_Error_HS300X_u8 = 0;
            }
        }
    }
    else
    {
        if (Number_Error_HS300X_u8 < MAX_READ_SENSOR_RETURN_NULL)
        {
            Number_Error_HS300X_u8++;
        }
        else
        {
            *temperature = 0;
            *humidity = 0;
        }
    }
    return reVal;
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

