/*****************************************************************************/
/**
* @file main.h
*
* Ctrl-X PLC driver.
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
* 1.00  tt   23/11/2023  First release
*
* </pre>
******************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

#include "pico/stdlib.h"
#include <stdio.h> // sprintf()
#include <stdlib.h> // abs(), atoi()
#include <string.h>
#include "uart/tt_uart.h"
#include "time.h"
#include "hardware/i2c.h"
#include "i2c/xi2c.h"
#include "gpio/xgpio.h"
#include "gpio/xpwm.h"
#include "i2c/oled_sh1106/ss_oled.h"
#include "i2c/RV3028/RV3028.h"
#include "i2c/eeprom/xeeprom.h"
#include "i2c/ioexp/ioexp.h"
#include "i2c/joy2/joy2.h"
#include "spi/xspi.h"
#include "spi/max31865/MAX31865.h"
#include "spi/mcp2515/mcp2515.h"
#include "spi/mcp2515/can.h"
#include "spi/sram/23LCV512.h"
#include "adc/xadc.h"
#include "xplc.h"


#define PASSWORD 0x20 // RTC
#define SRAM_ADRPROCVAR 0 // 

// The weekday() method returns the day of the week as an integer, where Monday is 0 and Sunday is 6
struct tm CurrentTime = 
{
	.tm_sec = 0,
	.tm_min = 42,
	.tm_hour = 18,
	.tm_wday = 0,
	.tm_mday = 29,
	.tm_mon = 5,
	.tm_year = 23,
};

	
rv3028_init_t RTC_Init = {
	// Use this settings to enable the battery backup
	.BatteryMode = RV3028_BAT_DSM,
	.Resistance = RV3028_TCT_3K,
	.EnableBSIE = true,
	.EnableCharge = true,
	// TIM

	// Use this settings to configure the time stamp function
	//.TSMode = RV3028_TS_BAT,
	.EnableTS = false,
	// TIM
	//.EnableTSOverwrite = true,

	// Use this settings to enable the clock output
	.Frequency = RV3028_CLKOUT_1HZ,
	//RV3028_CLKOUT_8KHZ,
	.EnableClkOut = true,

	// Use this settings for the event input configuration
	.EnableEventInt = false,
	// TIM
	.EventHighLevel = false,
	.Filter = RV3028_FILTER_256HZ,

	// Set the current time
	.HourMode = RV3028_HOURMODE_24,
	.p_CurrentTime = &CurrentTime,

	// Use this settings for the Power On Reset interrupt
	.EnablePOR = false,

	// Use this settings for the password function
	.Password = PASSWORD,
};
typedef struct
{
	int8_t tst_i2c;	// 0=inactive, 1=I2C-Scan-I2C0_extern,  2=I2C-Scan-I2C1_intern
	int8_t tst_cs;	// disable = 0, pt1 = 1, pt2 = 2, ext_cs1 = 3, ext_cs2 = 4, can = 5, sram = 6, tp206 = 7, tp207 = 8
	int8_t tst_sram; // disable = 0, rd_sram = 1, wr_sram = 2
	int8_t tst_output; // "cil[0/1..4] - LEDs, cir[0/1..4] - Relays, cio[0/1..4] - 24V Output, cib0/1 - Buzzer"
	int8_t tst_joy2;
	bool scan_sensor0;
	bool rtc_dt_rd;
	bool rtc_dt_wr;
	bool can_tx;
} Command_t;
	

typedef struct {
	
	uint8_t ram_content[10]; // remanent
	bool bupdateOled;
} procvar_t;


typedef struct {
	
	uint8_t content[10]; // remanent
} sram_t;

typedef struct {
	
	uint8_t content[10]; // remanent
} ee_t;



#endif /* MAIN_H_ */
