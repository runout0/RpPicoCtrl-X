//
// Bit Bang I2C library
// Copyright (c) 2018 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 10/12/2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "BitBang_I2C.h"

//
// Test a specific I2C address to see if a device responds
// returns 0 for no response, 1 for a response
//
uint8_t oled_I2CTest(BBI2C *pI2C, uint8_t addr)
{
	int ret;
	uint8_t rxdata;
	ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);
	return (ret >= 0);
} /* I2CTest() */

// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int oled_I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
	int rc = 0;
  
	rc = i2c_write_blocking(i2c1, iAddr, pData, iLen, true); // true to keep master control of bus
	return rc >= 0 ? iLen : 0;
} /* I2CWrite() */
//
// Read N bytes starting at a specific I2C internal register
//
int oled_I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
	int rc;
  
	rc = i2c_write_blocking(i2c1, iAddr, &u8Register, 1, true); // true to keep master control of bus 
	if (rc >= 0) {
		rc = i2c_read_blocking(i2c1, iAddr, pData, iLen, false);
	}
	return (rc >= 0);
} /* I2CReadRegister() */
//
// Read N bytes
//
int oled_I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
	int rc;
  
	rc = i2c_read_blocking(i2c1, iAddr, pData, iLen, false);
	return (rc >= 0);
} /* I2CRead() */
//
// Figure out what device is at that address
// returns the enumerated value
//
int oled_I2CDiscoverDevice(BBI2C *pI2C, uint8_t i)
{
	uint8_t j, cTemp[8];
	int iDevice = DEVICE_UNKNOWN;

	if (i == 0x3c || i == 0x3d) // Probably an OLED display
	{
		oled_I2CReadRegister(pI2C, i, 0x00, cTemp, 1);
		cTemp[0] &= 0xbf; // mask off power on/off bit
		if (cTemp[0] == 0x8) // SH1106
		iDevice = DEVICE_SH1106;
		else if(cTemp[0] == 3 || cTemp[0] == 6)
		   iDevice = DEVICE_SSD1306;
		return iDevice;
	}
  
	if (i == 0x34 || i == 0x35) // Probably an AXP202/AXP192 PMU chip
	{
		oled_I2CReadRegister(pI2C, i, 0x03, cTemp, 1); // chip ID
		if (cTemp[0] == 0x41)
			return DEVICE_AXP202;
		else if (cTemp[0] == 0x03)
			return DEVICE_AXP192;
	}
  
	if (i >= 0x40 && i <= 0x4f) // check for TI INA219 power measurement sensor
	{
		oled_I2CReadRegister(pI2C, i, 0x00, cTemp, 2);
		if (cTemp[0] == 0x39 && cTemp[1] == 0x9f)
			return DEVICE_INA219;
	}
  
	// Check for Microchip 24AAXXXE64 family serial 2 Kbit EEPROM
	if (i >= 0x50 && i <= 0x57) {
		uint32_t u32Temp = 0;
		oled_I2CReadRegister(pI2C,
			i,
			0xf8,
			(uint8_t *)&u32Temp,
			3); // check for Microchip's OUI
		if (u32Temp == 0x000004a3 || u32Temp == 0x00001ec0 ||
		    u32Temp == 0x00d88039 || u32Temp == 0x005410ec)
			return DEVICE_24AAXXXE64;
	}
  
	//  else if (i == 0x5b) // MLX90615?
	//  {
	//    I2CReadRegister(pI2C, i, 0x10, cTemp, 3);
	//    for (j=0; j<3; j++) Serial.println(cTemp[j], HEX);
	//  }
	  // try to identify it from the known devices using register contents
	{    
		// Check for TI HDC1080
		oled_I2CReadRegister(pI2C, i, 0xff, cTemp, 2);
		if (cTemp[0] == 0x10 && cTemp[1] == 0x50)
			return DEVICE_HDC1080;

		// Check for BME680
		if (i == 0x76 || i == 0x77)
		{
			oled_I2CReadRegister(pI2C, i, 0xd0, cTemp, 1); // chip ID
			if (cTemp[0] == 0x61) // BME680
			return DEVICE_BME680;
		}
		// Check for VL53L0X
		oled_I2CReadRegister(pI2C, i, 0xc0, cTemp, 3);
		if (cTemp[0] == 0xee && cTemp[1] == 0xaa && cTemp[2] == 0x10)
			return DEVICE_VL53L0X;

		// Check for CCS811
		oled_I2CReadRegister(pI2C, i, 0x20, cTemp, 1);
		if (cTemp[0] == 0x81) // Device ID
		return DEVICE_CCS811;

		// Check for LIS3DSH accelerometer from STMicro
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
		if (cTemp[0] == 0x3f) // WHO_AM_I
		return DEVICE_LIS3DSH;

		// Check for LIS3DH accelerometer from STMicro
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
		if (cTemp[0] == 0x33) // WHO_AM_I
		return DEVICE_LIS3DH;

		// Check for LSM9DS1 magnetometer/gyro/accel sensor from STMicro
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
		if (cTemp[0] == 0x68) // WHO_AM_I
		return DEVICE_LSM9DS1;

		// Check for LPS25H pressure sensor from STMicro
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
		if (cTemp[0] == 0xbd) // WHO_AM_I
		return DEVICE_LPS25H;
    
		// Check for HTS221 temp/humidity sensor from STMicro
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1);
		if (cTemp[0] == 0xbc) // WHO_AM_I
		return DEVICE_HTS221;
    
		// Check for MAG3110
		oled_I2CReadRegister(pI2C, i, 0x07, cTemp, 1);
		if (cTemp[0] == 0xc4) // WHO_AM_I
		return DEVICE_MAG3110;

		// Check for LM8330 keyboard controller
		oled_I2CReadRegister(pI2C, i, 0x80, cTemp, 2);
		if (cTemp[0] == 0x0 && cTemp[1] == 0x84) // manufacturer code + software revision
		return DEVICE_LM8330;

		// Check for MAX44009
		if (i == 0x4a || i == 0x4b)
		{
			for (j = 0; j < 8; j++)
				oled_I2CReadRegister(pI2C, i, j, &cTemp[j], 1); // check for power-up reset state of registers
			if ((cTemp[2] == 3 || cTemp[2] == 2) && cTemp[6] == 0 && cTemp[7] == 0xff)
				return DEVICE_MAX44009;
		}
       
		// Check for ADS1115
		oled_I2CReadRegister(pI2C, i, 0x02, cTemp, 2); // Lo_thresh defaults to 0x8000
		oled_I2CReadRegister(pI2C, i, 0x03, &cTemp[2], 2); // Hi_thresh defaults to 0x7fff
		if (cTemp[0] == 0x80 && cTemp[1] == 0x00 && cTemp[2] == 0x7f && cTemp[3] == 0xff)
			return DEVICE_ADS1115;

		// Check for MCP9808
		oled_I2CReadRegister(pI2C, i, 0x06, cTemp, 2); // manufacturer ID && get device ID/revision
		oled_I2CReadRegister(pI2C, i, 0x07, &cTemp[2], 2); // need to read them individually
		if (cTemp[0] == 0 && cTemp[1] == 0x54 && cTemp[2] == 0x04 && cTemp[3] == 0x00)
			return DEVICE_MCP9808;
       
		// Check for BMP280/BME280
		oled_I2CReadRegister(pI2C, i, 0xd0, cTemp, 1);
		if (cTemp[0] == 0x55) // BMP180
		return DEVICE_BMP180;
		else if(cTemp[0] == 0x58)
		   return DEVICE_BMP280;
		else if(cTemp[0] == 0x60) // BME280
		   return DEVICE_BME280;

		// Check for LSM6DS3
		oled_I2CReadRegister(pI2C, i, 0x0f, cTemp, 1); // WHO_AM_I
		if (cTemp[0] == 0x69)
			return DEVICE_LSM6DS3;
       
		// Check for ADXL345
		oled_I2CReadRegister(pI2C, i, 0x00, cTemp, 1); // DEVID
		if (cTemp[0] == 0xe5)
			return DEVICE_ADXL345;
       
		// Check for MPU-60x0i, MPU-688x, and MPU-9250
		oled_I2CReadRegister(pI2C, i, 0x75, cTemp, 1);
		if (cTemp[0] == (i & 0xfe)) // Current I2C address (low bit set to 0)
		return DEVICE_MPU6000;
		else if(cTemp[0] == 0x71)
		   return DEVICE_MPU9250;
		else if(cTemp[0] == 0x19)
		   return DEVICE_MPU6886;

		// Check for DS3231 RTC
		oled_I2CReadRegister(pI2C, i, 0x0e, cTemp, 1); // read the control register
		if (i == 0x68 &&
		    cTemp[0] == 0x1c) // fixed I2C address and power on reset value
		return DEVICE_DS3231;

		// Check for DS1307 RTC
		oled_I2CReadRegister(pI2C, i, 0x07, cTemp, 1); // read the control register
		if (i == 0x68 &&
		    cTemp[0] == 0x03) // fixed I2C address and power on reset value
		return DEVICE_DS1307;
        
	}
	return iDevice;
} /* I2CDiscoverDevice() */
