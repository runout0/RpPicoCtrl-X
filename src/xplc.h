/*****************************************************************************/
/**
* @file xplc.h
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

#ifndef XPLC_H_
#define XPLC_H_

#include "pico/stdlib.h"
#include "i2c/ioexp/ioexp.h"
#include "spi/max31865/MAX31865.h"
#include "adc/xadc.h"

#define DI_INTERN0 0
#define DO_INTERN0 0
#define AI_INTERN0 0
#define AO_INTERN0 0

#define DI_1 0 // digital Input 1
#define DI_2 1 // digital Input 2
#define DI_3 2 // digital Input 3
#define DI_4 3 // digital Input 4
#define DI_5 4 // digital Input 5
#define DI_6 5 // digital Input 6
#define DI_7 6 // digital Input 7
#define DI_8 7 // digital Input 8
#define DI_BTN1 8 // Button 1
#define DI_BTN2 9 // Button 1
#define DI_DOSTATUS 10 // Status DO High = OK

#define DO_LED1_GN 0 // digital Output (LED1 gn) :11
#define DO_LED2_GN 1 // digital Output (LED2 gn) :12
#define DO_LED3_RD 2 // digital Output (LED3 rd) :13
#define DO_LED4_RD 3 // digital Output (LED4 rd) :14
#define DO_RELAY1  4 // digital Output (Relay 1) :21
#define DO_RELAY2  5 // digital Output (Relay 2) :22
#define DO_RELAY3  6 // digital Output (Relay 3) :23
#define DO_RELAY4  7 // digital Output (Relay 4) :24
#define DO_OUT1    8 // digital Output (DO1)		 :31
#define DO_OUT2    9 // digital Output (DO2)		 :32
#define DO_OUT3   10 // digital Output (DO3)		 :33
#define DO_OUT4   11 // digital Output (DO4)		 :34
#define DO_BUZZER 12 // digital Output (Buzzer)	 :41

typedef enum
{
	XPLC_MODECONF_0  = 0x00,
	XPLC_MODECONF_RES, // Reset Configuration Mode, set UART in User-Mode, resume PLC-Cycle
	XPLC_MODECONF_SET, // Set Configuration Mode, set UART in Config-Mode, stop PLC-Cycle
	XPLC_MODECONF_REBOOT, // Reboot whole PLC
} xplc_confmode_t;

typedef enum
{
	XPLC_NO_ERROR = 0x00, /**< No error. */
} xplc_error_t;

typedef enum
{
	XPLC_RTCC_SETTIME     = 0x00,
	XPLC_RTCC_SETDATE
} xplc_rtc_cmd_t;

typedef enum
{
	XPLC_RTCS_STARTED = 0x00,
	XPLC_RTCS_RUN,
	XPLC_RTCS_DONE,
	XPLC_RTCS_ERROR		
} xplc_rtc_status_t;

typedef struct
{
	uint8_t DI[16];		
} di_t;

typedef struct
{
	uint8_t DO[16];		
} do_t;

typedef struct
{
	uint16_t AI[4]; // Pico ADC0, ADC1, ADC2
	float PT[2]; // Temperature-PT1, Temperature-PT2
} ai_t;

typedef struct
{
	uint16_t AO[4];		
} ao_t;

/*
https://plcopen.org/sites/default/files/downloads/annex_a_e.pdf 
B.1.2.3.2 Time of day and date
PRODUCTION RULES:
time_of_day ::= ('TIME_OF_DAY' | 'time_of_day' | 'TOD' | 'tod') '#' daytime
daytime ::= day_hour ':' day_minute ':' day_second
day_hour ::= integer
day_minute ::= integer
day_second ::= fixed_point
date ::= ('DATE' | 'date' | 'D' | 'd') '#' date_literal
date_literal ::= year '-' month '-' day
year ::= integer
month ::= integer
day ::= integer
date_and_time ::= ('DATE_AND_TIME' | 'date_and_time' | 'DT' | 'dt') '#' date_literal
'-' daytime
 **/

typedef struct
{
	char date_literal[10]; //date_literal ::= year '-' month '-' day
	int8_t year;
	uint8_t month;
	uint8_t day;	
} plc_date_t;
	
typedef struct
{
	uint8_t cmd;
	int16_t status; // tbd started
	char daytime[8]; //daytime ::= day_hour ':' day_minute ':' day_second
	uint8_t day_hour;
	uint8_t day_minute;
	uint8_t day_second;
} plc_time_t;

// ********* Structure for handling RTC ********* 
typedef struct
{
	xplc_rtc_cmd_t cmd;
	xplc_rtc_status_t status;
	plc_date_t getdate;
	plc_date_t setdate;
	plc_time_t gettime;
	plc_time_t settime;
} xplc_rtc_t;

typedef enum
{
	XPLC_PWMC_TEACHMODE = 0x00,
	XPLC_PWMC_INVERSE, // Servo inverted direction
	XPLC_PWMC_STEPMIN, // step--, stepwidth/accelerated stepwidth is up to the application
	XPLC_PWMC_STEPMAX, // step++, stepwidth/accelerated stepwidth is up to the application
	XPLC_PWMC_TEACHMIN, // teach minimum-Position -> EEPROM
	XPLC_PWMC_TEACHMAX // teach maximum-Position -> EEPROM
} xplc_pwm_cmd_t;

typedef enum
{
	XPLC_PWMS_STARTED = 0x00,
	XPLC_PWMS_RUN,
	XPLC_PWMS_DONE,
	XPLC_PWMS_ERROR		
} xplc_pwm_status_t;

typedef enum
{
	XPLC_PWMM_OFF = 0x00,
	XPLC_PWMM_NATIVE,
	XPLC_PWMM_SERVO, // (Period: 20ms/50Hz, Duty-Cycle-nom.min. 1ms, Duty-Cycle-nom.max. 2ms)
	XPLC_PWMM_FAST_INPUT // special Mode in Ctrl-X (you have to set a solder jumper for fast-input aka interrupt)
} xplc_pwm_mode_t;

// ********* Structure for handling PWM ********* 
typedef struct
{
	xplc_pwm_mode_t mode;
	xplc_pwm_cmd_t cmd;
	xplc_pwm_status_t status;
	uint8_t channel; // to address if more the one PWM-Channel (Ctrl-X only one channel on GPIO20)
	uint32_t freqency; // mode "XPLC_PWMM_NATIVE" [Hz]: pwm-frequncy
	bool force_en; // mode "XPLC_PWMM_NATIVE" [Hz]: force off/0n
	uint8_t force_val;  // mode "XPLC_PWMM_NATIVE" [Hz]: 0..100
} xplc_pwm_t;

typedef struct
{
	di_t di_int[1];
	do_t do_int[1];
	
	ai_t ai_int[1];
	ao_t ao_int[1];
} xplcio_t;

typedef struct
{
	xplc_confmode_t mode;
	xplc_rtc_t plc_rtc;
	xplc_pwm_t plc_pwm;
} xplconf_t;


typedef struct
{
	uint8_t version_interface[3]; // PLC-Interfaceversion <major.minor.revision>
	xplcio_t IO;
	xplconf_t CONF;
} xplc_t;

void cpyCtrlx2PLC(xplc_t *xplc, ioexp_t *ioexp_di, ioexp_t *ioexp_re, ioexp_t *ioexp_dio, Max31865_t  *pt100_1, Max31865_t  *pt100_2, xadc_t *xadc);
void cpyPLC2Ctrlx(xplc_t *xplc, ioexp_t *ioexp_re, ioexp_t *ioexp_dio);
void PLCsetup(xplc_t *xplx, uint32_t millis);
void PLCloop(xplc_t *xplc, uint32_t millis);

#endif /* XPLC_H_ */