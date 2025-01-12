/*****************************************************************************/
/**
* @file RV3028_Defs.h
*
* I/O-Expander PCA9500PW I2C Driver
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
* 1.00  tt   09/06/2023  First release
*
* </pre>
******************************************************************************/

#ifndef IOEXP_DEFS_H_
#define IOEXP_DEFS_H_

//#include <time.h>
#include <stdint.h> /*/ wg. uint8_t*/
//#include <stddef.h>
//#include <stdbool.h>
#include "hardware/i2c.h"

 /**@brief PCA9500PW I2C device slave address.
  */
#define IOEXP_DI_IO_ADDRESS 0x20 // I/O 8x Input 24V/DC
#define IOEXP_RE_IO_ADDRESS 0x23 // I/O 4x Relais, 4x LEDs (2gn/2rt)
#define IOEXP_DIO_IO_ADDRESS 0x24 // I/O 4x HS-Output, 1x RM-Status, 2x Bt., Buzzer
 
#define IOEXP_DI_MEM_ADDRESS 0x50 //EEPROM 8x Input 24V/DC
#define IOEXP_RE_MEM_ADDRESS 0x53 //EEPROM 4x Relais, 4x LEDs (2gn/2rt)
#define IOEXP_DIO_MEM_ADDRESS 0x54 //EEPROM 4x HS-Output, 1x RM-Status, 2x Bt., Buzzer
 
#define IOEXP_DI_BIT_L 0
#define IOEXP_DI_BIT_H 1

#define IOEXP_DO_BIT_L 0
#define IOEXP_DO_BIT_H 1

#define IOEXP_DI_BTN0_MASK (0x40)
#define IOEXP_DI_BTN1_MASK (0x80)
#define IOEXP_DI_STATDO_MASK (0x10)

/**@brief Error codes for the PCA9500PW driver.
  */
typedef enum
{
	IOEXP_NO_ERROR = 0x00, /**< No error. */
	IOEXP_INVALID_PARAM = 0x01, /**< Invalid parameter passed to function call. */
	IOEXP_TIMEOUT = 0x02, /**< Communication timeout. */
	IOEXP_NOT_INITIALIZED = 0x03,
	IOEXP_NOT_READY = 0x04, /**< Device function is not ready. Please initialize them first. */
	IOEXP_NOT_VALID = 0x05, /**< Receive Data DATA1/DATA4 not equal */
	IOEXP_COMM_ERROR = 0x06, /**< Communication error. */
	IOEXP_RDLEN_ERROR = 0x07, /**< Read-Lenght mismatch */
	IOEXP_WRLEN_ERROR = 0x08, /**< Write-Lenght mismatch */
} ioexp_error_t;

/**@brief Accesss modes
 */
typedef enum
{
	IOEXP_MODE_WR = 0x00,
	/**< Schreibzugriff */
	IOEXP_MODE_RD = 0x01, /**< Lesezugriff */
} ioexp_mode_t;
 
/**@brief PCA9500PW device initialization object structure.
 */
typedef struct
{
	bool		DisableSync; /**< Boolean flag to disable the sync for the CLKOUT pin. */
	bool		EnableEventInt; /**< Set to #true to enable the event interrupt function on the INT pin. */
	bool		EventHighLevel; /**< Set to #true to enable the rising edge or high level event detection. */
	bool		EnableTS; /**< Set to #true to enable the time stamp mode. */
	bool		EnableTSOverwrite; /**< Set to #true to enable the overwrite mode in time stamp mode. */
	bool		EnableClkOut; /**< Boolean flag to enable the CLKOUT function. */
	bool		EnablePOR; /**< Boolean flag to enable the POR function on the INT pin. */
	bool		EnableBSIE; /**< Boolean flag to enable the BSIE function on the INT pin. */
	bool		EnableCharge; /**< Boolean flag to enable the trickle charger function.
							 NOTE: Only needed when \ref rv3028_t.BatteryMode is enabled. */
} ioexp_init_t;

#endif /* IOEXP_DEFS_H_ */

/**@brief RV3028 device object structure.
 */
typedef struct
{
	i2c_inst_t *i2c;      /**< I2C Hardware (i2c0 or i2c1). */
	uint8_t		DeviceAddr; /**< IO device address. */	
	uint8_t Data[2];    /**< from Port*/	
	uint8_t DI[8]; /**< DataIn-Bits */	
	uint8_t DO[8]; /**< DataIn-Bits */	
	
	bool		IsInitialized; /**< Boolean flag to indicate a successful initialization. */

} ioexp_t;
 