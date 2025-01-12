/*****************************************************************************/
/**
* @file joy2.c
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

#include "pico/stdlib.h"
#include "joy2.h"

i2c_inst_t *i2c_int;

joy2_error_t joy2_Init(joy2_t *joy2, i2c_inst_t *i2c, uint8_t DeviceAddr)
{
	joy2->i2c = i2c;
	joy2->DeviceAddr = DeviceAddr;
	i2c_int = i2c;
	joy2_error_t valrc = JOY2_NO_ERROR;
	joy2->IsInitialized = true;
	return valrc;
}


joy2_error_t joy2_write_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)	
{
	uint8_t Temp[20] = {reg };
	int rc;
	joy2_error_t valrc = JOY2_NO_ERROR;
	size_t len = length + 1;

	for (uint8_t i = 0x01; i < (length + 0x01); i++)
	{
		Temp[i] = *(buffer++);
	}

	rc = i2c_write_blocking(i2c_int, addr, Temp, len, false);
	if (rc != len) 
	{
		valrc = JOY2_COMM_ERROR;
	}
	return valrc;
}

joy2_error_t joy2_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
	int rc;
	joy2_error_t valrc = JOY2_NO_ERROR;
		
	rc = i2c_write_blocking(i2c_int, addr, &reg, sizeof(uint8_t), false);	
	if (rc != sizeof(uint8_t))
	{
		valrc = JOY2_COMM_ERROR;
		return valrc;
	}
	
	rc = i2c_read_blocking(i2c_int, addr, buffer, length, false);
	if (rc != length)
	{
		valrc = JOY2_COMM_ERROR;
	}
	return valrc;
}

//
//bool joy2_begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed)
//{
//    _wire  = wire;
//    _addr  = addr;
//    _sda   = sda;
//    _scl   = scl;
//    _speed = speed;
//    _wire->begin(_sda, _scl);
//    _wire->setClock(speed);
//    delay(10);
//    _wire->beginTransmission(_addr);
//    uint8_t error = _wire->endTransmission();
//    if (error == 0) {
//        return true;
//    } else {
//        return false;
//    }
//}

joy2_error_t joy2_get_adc_value_x(joy2_t *joy2, adc_mode_t adc_bits)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen;
	uint8_t data[4], reg;

	if (adc_bits == ADC_16BIT_RESULT) {
		reg = JOYSTICK2_ADC_VALUE_12BITS_REG;
		rdlen = 2;		
		valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
		if (valrc == JOY2_NO_ERROR)
		{
			joy2->xADC16 = data[0] | (data[1] << 8);
		}
	}
	else if (adc_bits == ADC_8BIT_RESULT) {
		reg = JOYSTICK2_ADC_VALUE_8BITS_REG;
		rdlen = 1;
		valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
		if (valrc == JOY2_NO_ERROR)
		{
			joy2->xADC8 = data[0];
		}
	}
	return valrc;
}

joy2_error_t joy2_get_adc_value_y(joy2_t *joy2, adc_mode_t adc_bits)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen;
	uint8_t data[4];

	if (adc_bits == ADC_16BIT_RESULT) {
		uint8_t reg = JOYSTICK2_ADC_VALUE_12BITS_REG + 2;
		rdlen = 2;		
		valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
		if (valrc == JOY2_NO_ERROR)
		{
			joy2->yADC16 = data[0] | (data[1] << 8);
		}
	}
	else if (adc_bits == ADC_8BIT_RESULT) {
		uint8_t reg = JOYSTICK2_ADC_VALUE_8BITS_REG + 1;
		rdlen = 1;
		valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
		if (valrc == JOY2_NO_ERROR)
		{
			joy2->yADC8 = data[0];
		}
	}
	return valrc;
}

joy2_error_t joy2_get_adc_16bits_value_xy(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen = 4;
	uint8_t data[4];

	uint8_t reg = JOYSTICK2_ADC_VALUE_12BITS_REG;

	valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
	if (valrc == JOY2_NO_ERROR)
	{
		memcpy((uint8_t *)&joy2->xADC16, &data[0], 2);
		memcpy((uint8_t *)&joy2->yADC16, &data[2], 2);
	}
	return valrc;
}

joy2_error_t joy2_get_adc_8bits_value_xy(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen = 2;
	uint8_t data[4];

	uint8_t reg = JOYSTICK2_ADC_VALUE_8BITS_REG;

	joy2_read_bytes(joy2->DeviceAddr, reg, data, rdlen);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->xADC8 = data[0];
		joy2->yADC8 = data[1];
	}
	return valrc;
}

joy2_error_t joy2_get_adc_12bits_offset_value_x(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen = 2;
	int16_t value;

	valrc = joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_OFFSET_ADC_VALUE_12BITS_REG, (uint8_t *)&value, rdlen);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->xOffset12Bit = value;
	}
	return valrc;
}

joy2_error_t joy2_get_adc_12bits_offset_value_y(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen = 2;
	int16_t value;

	valrc = joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_OFFSET_ADC_VALUE_12BITS_REG + 2, (uint8_t *)&value, rdlen);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->yOffset12Bit = value;
	}
	return value;
}

joy2_error_t joy2_get_adc_8bits_offset_value_x(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int8_t value;

	valrc = joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_OFFSET_ADC_VALUE_8BITS_REG, (uint8_t *)&value, 1);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->xOffset8Bit = value;
	}
	return valrc;
}

joy2_error_t joy2_get_adc_8bits_offset_value_y(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int8_t value;

	valrc = joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_OFFSET_ADC_VALUE_8BITS_REG + 1, (uint8_t *)&value, 1);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->yOffset8Bit = value;
	}
	return valrc;
}

joy2_error_t joy2_set_adc_value_cal(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int  wrlen = 16;
	uint8_t data[16];

	valrc = joy2_write_bytes(joy2->DeviceAddr, JOYSTICK2_ADC_VALUE_CAL_REG, data, wrlen);
	if (valrc == JOY2_NO_ERROR)
	{	
		memcpy(&data[0],  (uint8_t *)&joy2->x_neg_min, 2);
		memcpy(&data[2],  (uint8_t *)&joy2->x_neg_max, 2);
		memcpy(&data[4],  (uint8_t *)&joy2->x_pos_min, 2);
		memcpy(&data[6],  (uint8_t *)&joy2->x_pos_max, 2);
		memcpy(&data[8],  (uint8_t *)&joy2->y_neg_min, 2);
		memcpy(&data[10], (uint8_t *)&joy2->y_neg_max, 2);
		memcpy(&data[12], (uint8_t *)&joy2->y_pos_min, 2);
		memcpy(&data[14], (uint8_t *)&joy2->y_pos_max, 2);
	}
	return valrc;
}
	
joy2_error_t joy2_get_adc_value_cal(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rdlen = 16;
	uint8_t data[16];

	joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_ADC_VALUE_CAL_REG, data, rdlen);
	if (valrc == JOY2_NO_ERROR)
	{
		memcpy((uint8_t *)&joy2->x_neg_min, &data[0], 2);
		memcpy((uint8_t *)&joy2->x_neg_max, &data[2], 2);
		memcpy((uint8_t *)&joy2->x_pos_min, &data[4], 2);
		memcpy((uint8_t *)&joy2->x_pos_max, &data[6], 2);
		memcpy((uint8_t *)&joy2->y_neg_min, &data[8], 2);
		memcpy((uint8_t *)&joy2->y_neg_max, &data[10], 2);
		memcpy((uint8_t *)&joy2->y_pos_min, &data[12], 2);
		memcpy((uint8_t *)&joy2->y_pos_max, &data[14], 2);
	}
	return valrc;	
}

joy2_error_t joy2_get_button_value(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	uint8_t data[4];

	uint8_t reg = JOYSTICK2_BUTTON_REG;
	valrc = joy2_read_bytes(joy2->DeviceAddr, reg, data, 1);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->BtnPressed = !(bool)data[0];			
	}
	return valrc;	
}

joy2_error_t joy2_set_rgb_color(joy2_t *joy2, uint8_t r, uint8_t g, uint8_t b)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	uint8_t rgb_write_buff[4] = { 0,0,0,0};
	
	joy2->ColorR = r;
	joy2->ColorG = g;
	joy2->ColorB = b;
	
	rgb_write_buff[0] = joy2->ColorB;
	rgb_write_buff[1] = joy2->ColorG;
	rgb_write_buff[2] = joy2->ColorR;
	
	valrc = joy2_write_bytes(joy2->DeviceAddr, JOYSTICK2_RGB_REG, rgb_write_buff, 4);
	return valrc;	
}

joy2_error_t joy2_get_rgb_color(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	uint32_t rgb_read_buff = 0;

	joy2_read_bytes(joy2->DeviceAddr, JOYSTICK2_RGB_REG, (uint8_t *)&rgb_read_buff, 4);
	if (valrc == JOY2_NO_ERROR)
	{
		joy2->ColorB = rgb_read_buff & 0xff; // Register 0x30
		joy2->ColorG = (rgb_read_buff >> 8) & 0xff; // Register 0x31
		joy2->ColorR = (rgb_read_buff >> 16) & 0xff; // Register 0x32		
	}
	return valrc;	
}

joy2_error_t joy2_get_bootloader_version(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rc;
	uint8_t btmp = JOYSTICK2_BOOTLOADER_VERSION_REG;

	rc = i2c_write_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);	
	if (rc != sizeof(uint8_t))
	{
		valrc = JOY2_COMM_ERROR;
		return valrc;
	}
	
	rc = i2c_read_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);
	if (rc == sizeof(uint8_t))
	{
		joy2->VersionBL = btmp;
	}
	else
	{
		valrc = JOY2_COMM_ERROR;
	}
	return valrc;
}

joy2_error_t joy2_get_i2c_address(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rc;
	uint8_t btmp = JOYSTICK2_I2C_ADDRESS_REG;

	rc = i2c_write_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);	
	if (rc != sizeof(uint8_t))
	{
		valrc = JOY2_COMM_ERROR;
		return valrc;
	}
	
	rc = i2c_read_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);
	if (rc == sizeof(uint8_t))
	{
		joy2->I2CAddr = btmp;
	}
	else
	{
		valrc = JOY2_COMM_ERROR;
	}
	return valrc;	
} 

joy2_error_t joy2_set_i2c_address(joy2_t *joy2, uint8_t addr)
{
	joy2_error_t valrc = JOY2_NO_ERROR;  
//	_wire->beginTransmission(_addr);
//	_wire->write(JOYSTICK2_I2C_ADDRESS_REG);
//	_wire->write(addr);
//	_addr = addr;
//	if (_wire->endTransmission() == 0)
//		return rc;
//	else
//		return rc;
	
	return valrc;
}

joy2_error_t joy2_get_firmware_version(joy2_t *joy2)
{
	joy2_error_t valrc = JOY2_NO_ERROR;
	int rc;
	uint8_t btmp = JOYSTICK2_FIRMWARE_VERSION_REG;

	rc = i2c_write_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);	
	if (rc != sizeof(uint8_t))
	{
		valrc = JOY2_COMM_ERROR;
		return valrc;
	}
	
	rc = i2c_read_blocking(i2c_int, joy2->DeviceAddr, &btmp, sizeof(uint8_t), false);
	if (rc == sizeof(uint8_t))
	{
		joy2->VersionFW = btmp;
	}
	else
	{
		valrc = JOY2_COMM_ERROR;
	}
	return valrc;
} 
