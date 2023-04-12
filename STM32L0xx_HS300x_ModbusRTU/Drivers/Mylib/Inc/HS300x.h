#ifndef _HS300X_H_
#define _HS300X_H_


#include "stm32l0xx_hal.h"
//#include "stm32l1xx_hal_i2c.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define HS300X_I2C hi2c1

#define HS300X_SENSON_ADDR              0x44

#define HS300X_PROGRAMMING_MODE         0xA0
#define HS300X_HUMIDITY_READ_RESO_REG   0x06
#define HS300X_HUMIDITY_WRITE_RESO_REG  0x46
#define HS300X_TEMP_READ_RESO_REG       0x11
#define HS300X_TEMP_WRITE_RESO_REG      0x51

#define HS300X_STATUS_SUCCESS           0x81

#define HS300X_MAX_TEMPERATURE 8500
#define HS300X_MIN_TEMPERATURE 0
#define HS300X_MAX_HUMID 99
#define HS300X_MIN_HUMID 0

#define MAX_READ_SENSOR_RETURN_NULL     3    
/*******************************************************************************
 * Typedef
 ******************************************************************************/

typedef enum
{
    TEMP_RESOLUTION_14_BIT = 0x0C,
    TEMP_RESOLUTION_12_BIT = 0x08,
    TEMP_RESOLUTION_10_BIT = 0x04,
    TEMP_RESOLUTION_8_BIT  = 0x00
}TEMP_RESO;

typedef enum
{
    HUMI_RESOLUTION_14_BIT = 0x0C,
    HUMI_RESOLUTION_12_BIT = 0x08,
    HUMI_RESOLUTION_10_BIT = 0x04,
    HUMI_RESOLUTION_8_BIT  = 0x00
}TEMP_HUMID;


// extern uint8_t DRDY_INTflag;
/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief: Uses to check the time out.
 *
 * @param[IN]: None
 *
 * @return: None
 */
uint8_t HS300X_Init(I2C_HandleTypeDef *hi2c_x, TEMP_RESO Temperature_Resolution_x_bit, TEMP_HUMID Humidity_Resolution_x_bit);
uint8_t HS300X_Start_Measurement(I2C_HandleTypeDef *hi2c_x, int16_t *temperature, int16_t *humidity);

#endif /* _HS300X_H_ */

