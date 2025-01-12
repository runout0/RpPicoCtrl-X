/*****************************************************************************/
/**
* @file xeeprom.h
*
* CAT24M01 I2C driver.
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
* 1.00  tt   06/06/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XEEPROM_H_
#define XEEPROM_H_

#include "hardware/i2c.h"

#define EEPROML_ADDRESS                      0x56
#define EEPROMH_ADDRESS                      0x57

/** Number of bytes in EEPROM */
#define EEPROM_24M01_LEN         0x20000

/** Pagesize in EEPROM
 * The CAT24M01 contains 131,072 bytes of data, arranged in 512 pages of 256 bytes each.*/
#define EEPROM_24M01_PAGESIZE    256
#define EEPROM_TEST_PAGESIZE    64 

 /**@brief Error codes for the EEPROM driver.
  */
 typedef enum
 {
    EEPROM_NO_ERROR	    	= 0x00,			/**< No error. */
    EEPROM_INVALID_PARAM	= 0x01,			/**< Invalid parameter passed to function call. */
    EEPROM_NO_ACK  		  = 0x02,			/**< Communication timeout. Acknowledge Polling (while Write)*/
    EEPROM_NOT_INITIALIZED	= 0x03,		/**< Device is not initialized. Please call the EEPROM_Init function. */
    EEPROM_CMD_SKIPPED		  = 0x04,			/**< Device not ready due to ACK Polling */
    EEPROM_WRLEN_ERROR		  = 0x05,			/**Return of "i2c_write_blocking()" does not match */
	  EEPROM_RDLEN_ERROR		  = 0x06,			/**Return of "i2c_read_blocking()" does not match */
    EEPROM_COMM_ERROR		  = 0x07,			/**< Communication error. */
 } eeprom_error_t;

 /**@brief EEPROM device object structure. */
 typedef struct
 {
	 i2c_inst_t *i2c; /**< I2C Hardware (i2c0 or i2c1). */
	 uint8_t		DeviceAddr; /**< IO device address. */
	 uint16_t ee_address;
	 bool initialized;
	 bool ackpolling;
	 uint16_t AckPollCnt;
 } xeeprom_t;
 
eeprom_error_t EE_Init(xeeprom_t *eeprom, i2c_inst_t *i2c, uint8_t DeviceAddr);
eeprom_error_t EE_AcKPoll(xeeprom_t *eeprom, i2c_inst_t *i2c, uint8_t DeviceAddr);
eeprom_error_t EE_ReadImmediate(xeeprom_t *eeprom, uint8_t *rx);
eeprom_error_t EE_ReadSelectiv(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *rx);
eeprom_error_t EE_ReadSequential(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datarx, size_t len);
eeprom_error_t EE_WriteByte(xeeprom_t *eeprom, uint16_t ee_address, uint8_t tx);
eeprom_error_t EE_WritePage(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datatx, size_t len);
eeprom_error_t EE_RandomRead(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datarx, size_t len);
eeprom_error_t EE_RandomWrite(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datatx, size_t len);

#endif /* XEEPROM_H_ */