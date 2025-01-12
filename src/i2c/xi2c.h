/*****************************************************************************/
/**
* @file xi2c.h
*
* Ctrl-X I2C driver.
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
* 1.00  tt   29/05/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XI2C_H_
#define XI2C_H_

// I22C0 - Pico Port-Options
#define PICO_I2C0SET0_SDA_PIN 0
#define PICO_I2C0SET0_SCL_PIN 0
#define PICO_I2C0SET1_SDA_PIN 4
#define PICO_I2C0SET1_SCL_PIN 5
#define PICO_I2C0SET2_SDA_PIN 8
#define PICO_I2C0SET2_SCL_PIN 9
#define PICO_I2C0SET3_SDA_PIN 12
#define PICO_I2C0SET3_SCL_PIN 13
#define PICO_I2C0SET4_SDA_PIN 16
#define PICO_I2C0SET4_SCL_PIN 17
#define PICO_I2C0SET5_SDA_PIN 20
#define PICO_I2C0SET5_SCL_PIN 21

// I22C1 - Pico Port-Optionen
#define PICO_I2C1SET0_SDA_PIN 2
#define PICO_I2C1SET0_SCL_PIN 3
#define PICO_I2C1SET1_SDA_PIN 6
#define PICO_I2C1SET1_SCL_PIN 7
#define PICO_I2C1SET2_SDA_PIN 10
#define PICO_I2C1SET2_SCL_PIN 11
#define PICO_I2C1SET3_SDA_PIN 14
#define PICO_I2C1SET3_SCL_PIN 15
#define PICO_I2C1SET4_SDA_PIN 18
#define PICO_I2C1SET4_SCL_PIN 19
#define PICO_I2C1SET5_SDA_PIN 26
#define PICO_I2C1SET5_SCL_PIN 27


bool i2c0_ResetBus(uint8_t pin_sda, uint8_t pin_scl);
bool i2c1_ResetBus(uint8_t pin_sda, uint8_t pin_scl);

void i2c0_Init(uint8_t pin_sda, uint8_t pin_scl, uint baudrate, bool pullup);
void i2c1_Init(uint8_t pin_sda, uint8_t pin_scl, uint baudrate, bool pullup);
int I2C_Scan(i2c_inst_t *i2c);

#endif /* XI2C_H_ */