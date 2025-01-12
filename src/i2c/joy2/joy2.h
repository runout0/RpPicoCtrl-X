/*****************************************************************************/
	/**
* @file joy2.h
*
* Ctrl-X M5-Joystick2 driver.
* ported from: SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
*
* GNU GENERAL PUBLIC LICENSE:
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Errors and commissions should be reported to runout@gmx.de
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---  --------    -----------------------------------------------
* 1.00  tt   02/01/2025  First release
*
* </pre>
******************************************************************************/ 

#ifndef JOY2_H_
#define JOY2_H_

#include "hardware/i2c.h"
#include <string.h>

typedef struct
{
	i2c_inst_t *i2c; /**< I2C Hardware (i2c0 or i2c1). */
	uint8_t		DeviceAddr; /**< IO device address. */		
	bool		IsInitialized; /**< Boolean flag to indicate a successful initialization. */
	
	uint8_t xADC8;
	uint16_t xADC16;
	uint8_t yADC8;
	uint16_t yADC16;
	int8_t xOffset8Bit; //RO
	int16_t xOffset12Bit; //RO
	int8_t yOffset8Bit; //RO
	int16_t yOffset12Bit; //RO
	bool BtnPressed;
	uint8_t ColorR;
	uint8_t ColorG;
	uint8_t ColorB;
	uint16_t x_neg_min;
	uint16_t x_neg_max;
	uint16_t x_pos_min;
	uint16_t x_pos_max;
	uint16_t y_neg_min;
	uint16_t y_neg_max;
	uint16_t y_pos_min;
	uint16_t y_pos_max;
	uint8_t VersionBL;
	uint8_t VersionFW;
	uint8_t I2CAddr;
} joy2_t; 

typedef enum
{
	JOY2_NO_ERROR = 0x00,
	/**< No error. */
	JOY2_INVALID_PARAM = 0x01,
	/**< Invalid parameter passed to function call. */
	JOY2_TIMEOUT = 0x02,
	/**< Communication timeout. */
	JOY2_NOT_INITIALIZED = 0x03,
	JOY2_NOT_READY = 0x04,
	/**< Device function is not ready. Please initialize them first. */
	JOY2_NOT_VALID = 0x05,
	/**< Receive Data DATA1/DATA4 not equal */
	JOY2_COMM_ERROR = 0x06,
	/**< Communication error. */
	JOY2_RDLEN_ERROR = 0x07,
	/**< Read-Lenght mismatch */
	JOY2_WRLEN_ERROR = 0x08,
	/**< Write-Lenght mismatch */
} joy2_error_t; 


#define JOYSTICK2_ADDR                        0x63
#define JOYSTICK2_ADC_VALUE_12BITS_REG        0x00
#define JOYSTICK2_ADC_VALUE_8BITS_REG         0x10
#define JOYSTICK2_BUTTON_REG                  0x20
#define JOYSTICK2_RGB_REG                     0x30
#define JOYSTICK2_ADC_VALUE_CAL_REG           0x40
#define JOYSTICK2_OFFSET_ADC_VALUE_12BITS_REG 0x50
#define JOYSTICK2_OFFSET_ADC_VALUE_8BITS_REG  0x60
#define JOYSTICK2_FIRMWARE_VERSION_REG        0xFE
#define JOYSTICK2_BOOTLOADER_VERSION_REG      0xFC
#define JOYSTICK2_I2C_ADDRESS_REG             0xFF

typedef enum { ADC_8BIT_RESULT = 0, ADC_16BIT_RESULT } adc_mode_t;


joy2_error_t joy2_Init(joy2_t *joy2, i2c_inst_t *i2c, uint8_t DeviceAddr);

joy2_error_t joy2_set_i2c_address(joy2_t *joy2, uint8_t addr);

/**
 * @brief get Unit Joystick2 I2C address
 * @return I2C address
 */
joy2_error_t joy2_get_i2c_address(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 firmware version
 * @return firmware version
 */
joy2_error_t joy2_get_firmware_version(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 bootloader version
 * @return bootloader version
 */
joy2_error_t joy2_get_bootloader_version(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 x-axis ADC value
 * @param adc_bits ADC_8BIT_RESULT or ADC_16BIT_RESULT
 * @return x-axis ADC value
 */
joy2_error_t joy2_get_adc_value_x(joy2_t *joy2, adc_mode_t adc_bits);

/**
 * @brief get Unit Joystick2 y-axis ADC value
 * @param adc_bits ADC_8BIT_RESULT or ADC_16BIT_RESULT
 * @return y-axis ADC value
 */
joy2_error_t joy2_get_adc_value_y(joy2_t *joy2, adc_mode_t adc_bits);

/**
 * @brief get Unit Joystick2 button value
 * @return 0 press, 1 no press
 */
joy2_error_t joy2_get_button_value(joy2_t *joy2);

/**
 * @brief set Unit Joystick2 rgb color
 * @param color rgb color
 */
joy2_error_t joy2_set_rgb_color(joy2_t *joy2, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief get Unit Joystick2 rgb color
 * @return rgb color
 */
joy2_error_t joy2_get_rgb_color(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 mapped cal value
 * @param x_neg_min pointer of x-axis negative minimum value
 * @param x_neg_max pointer of x-axis negative maximum value
 * @param x_pos_min pointer of x-axis positive minimum value
 * @param x_pos_max pointer of x-axis positive maximum value
 * @param y_neg_min pointer of y-axis negative minimum value
 * @param y_neg_max pointer of y-axis negative maximum value
 * @param y_pos_min pointer of y-axis positive minimum value
 * @param y_pos_max pointer of y-axis positive maximum value
 */
joy2_error_t joy2_get_adc_value_cal(joy2_t *joy2);

/**
 * @brief set Unit Joystick2 mapped cal value
 * @param x_neg_min x-axis negative minimum value
 * @param x_neg_max x-axis negative maximum value
 * @param x_pos_min x-axis positive minimum value
 * @param x_pos_max x-axis positive maximum value
 * @param y_neg_min y-axis negative minimum value
 * @param y_neg_max y-axis negative maximum value
 * @param y_pos_min y-axis positive minimum value
 * @param y_pos_max y-axis positive maximum value
 */
joy2_error_t joy2_set_adc_value_cal(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 x-axis 12bits mapped value
 * @return x-axis 12bits mapped value
 */
joy2_error_t joy2_get_adc_12bits_offjoy2_set_value_x(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 y-axis 12bits mapped value
 * @return y-axis 12bits mapped value
 */
joy2_error_t joy2_get_adc_12bits_offjoy2_set_value_y(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 x-axis 8bits mapped value
 * @return x-axis 8bits mapped value
 */
joy2_error_t joy2_get_adc_8bits_offjoy2_set_value_x(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 y-axis 8bits mapped value
 * @return y-axis 8bits mapped value
 */
joy2_error_t joy2_get_adc_8bits_offjoy2_set_value_y(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 x-axis and y-axis 16bits ADC value
 * @param adc_x pointer of x-axis ADC value
 * @param adc_y pointer of y-axis ADC value
 */
joy2_error_t joy2_get_adc_16bits_value_xy(joy2_t *joy2);

/**
 * @brief get Unit Joystick2 x-axis and y-axis 8bits ADC value
 * @param adc_x pointer of x-axis ADC value
 * @param adc_y pointer of y-axis ADC value
 */
joy2_error_t joy2_get_adc_8bits_value_xy(joy2_t *joy2);

#endif /* JOY2_H_ */