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
#ifndef __BITBANG_I2C__
#define __BITBANG_I2C__
// supported devices
enum {
  DEVICE_UNKNOWN = 0,
  DEVICE_SSD1306,
  DEVICE_SH1106,
  DEVICE_VL53L0X,
  DEVICE_BMP180,
  DEVICE_BMP280,
  DEVICE_BME280,
  DEVICE_MPU6000,
  DEVICE_MPU9250,
  DEVICE_MCP9808,
  DEVICE_LSM6DS3,
  DEVICE_ADXL345,
  DEVICE_ADS1115,
  DEVICE_MAX44009,
  DEVICE_MAG3110,
  DEVICE_CCS811,
  DEVICE_HTS221,
  DEVICE_LPS25H,
  DEVICE_LSM9DS1,
  DEVICE_LM8330,
  DEVICE_DS3231,
  DEVICE_LIS3DH,
  DEVICE_LIS3DSH,
  DEVICE_INA219,
  DEVICE_SHT3X,
  DEVICE_HDC1080,
  DEVICE_MPU6886,
  DEVICE_BME680,
  DEVICE_AXP202,
  DEVICE_AXP192,
  DEVICE_24AAXXXE64,
  DEVICE_DS1307
};

#ifndef LOW
#define LOW 0
#define HIGH 1
#endif

typedef struct mybbi2c
{
uint8_t iSDABit, iSCLBit; // bit numbers of the ports
uint32_t iDelay;
} BBI2C;
//
// Read N bytes
//
int oled_I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Read N bytes starting at a specific I2C internal register
//
int oled_I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int oled_I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Scans for I2C devices on the bus
// returns a bitmap of devices which are present (128 bits = 16 bytes, LSB first)
//
// Test if an address responds
// returns 0 if no response, 1 if it responds
//
uint8_t oled_I2CTest(BBI2C *pI2C, uint8_t addr);

//
// Figure out what device is at that address
// returns the enumerated value
//
int oled_I2CDiscoverDevice(BBI2C *pI2C, uint8_t i);

#endif //__BITBANG_I2C__

