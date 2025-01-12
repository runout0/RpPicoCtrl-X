
/*****************************************************************************
* @file xeeprom.c
*
* Ctrl-X EEPROM driver.
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

#include "pico/stdlib.h"
#include "xeeprom.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>


/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/*******************************************************************************
 ***************************   LOCAL FUNCTIONS   *******************************
 ******************************************************************************/




/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

uint8_t testrx[4];

eeprom_error_t EE_Init(xeeprom_t *eeprom, i2c_inst_t *i2c, uint8_t DeviceAddr)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	eeprom->i2c = i2c;
	eeprom->DeviceAddr = DeviceAddr;
	eeprom->initialized = true;
	return rc;
}

/* Acknowlegde-Polling after EE-Write Operation*/
eeprom_error_t EE_AcKPoll(xeeprom_t *eeprom, i2c_inst_t *i2c, uint8_t DeviceAddr)
{
	eeprom_error_t rc = EEPROM_NOT_INITIALIZED;
	int ret;
	uint8_t rxdata;

	if (eeprom->initialized && eeprom->ackpolling)
	{		
		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns  -1.
		ret = i2c_read_blocking(i2c, DeviceAddr, &rxdata, 1, false);		
		//return ret < 0 ? EEPROM_NO_ACK : EEPROM_NO_ERROR;
		if (ret < 0) 
		{
			rc = EEPROM_NO_ACK;			
		}
		else
		{
			eeprom->ackpolling = false;	
			rc = EEPROM_NO_ERROR;
		}
	}
	else
	{
		return EEPROM_NOT_INITIALIZED;
	}
	return rc;
}

/* to be done */
eeprom_error_t EE_ReadImmediate(xeeprom_t *eeprom, uint8_t *rx)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	
	return rc;
}

eeprom_error_t EE_ReadSelectiv(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *rx)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	int ret;
	uint8_t tmpdata[2];
	uint8_t rdlen = 1;
	uint16_t sw_address = (ee_address >> 8) | (ee_address << 8);
	if (eeprom->initialized)
	{
		if (eeprom->ackpolling) return EEPROM_CMD_SKIPPED;
			
		memcpy(tmpdata, &sw_address, sizeof(sw_address)); //Data[0..1]
		ret = i2c_write_blocking(i2c1, eeprom->DeviceAddr, tmpdata, sizeof(tmpdata), false);
		if (ret == sizeof(tmpdata))
		{
			ret = i2c_read_blocking(i2c1, eeprom->DeviceAddr, rx, rdlen, false);
			if (ret != rdlen)
			{
				rc = EEPROM_RDLEN_ERROR;	
			}		
		}
		else
		{
			rc = EEPROM_WRLEN_ERROR; /**< Write-Lenght mismatch */
		}		
	}
	else
	{
		return EEPROM_NOT_INITIALIZED;	
	}
	
	return rc;	
}

eeprom_error_t EE_ReadSequential(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datarx, size_t len)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	int ret;
	uint8_t tmpdata[2];
	uint16_t sw_address = (ee_address >> 8) | (ee_address << 8);

	if (eeprom->initialized)
	{
		if (eeprom->ackpolling) return EEPROM_CMD_SKIPPED;
			
		memcpy(tmpdata, &sw_address, sizeof(sw_address)); //Data[0..1]
		ret = i2c_write_blocking(i2c1, eeprom->DeviceAddr, tmpdata, sizeof(tmpdata), false);
		if (ret == sizeof(tmpdata))
		{
			ret = i2c_read_blocking(i2c1, eeprom->DeviceAddr, datarx, len, true);
			if (ret != len)
			{
				rc = EEPROM_RDLEN_ERROR;	
			}		
		}
		else
		{
			rc = EEPROM_WRLEN_ERROR; /**< Write-Lenght mismatch */
		}		
	}
	else
	{
		return EEPROM_NOT_INITIALIZED;	
	}
	
	return rc;	
}

eeprom_error_t EE_WriteByte(xeeprom_t *eeprom, uint16_t ee_address, uint8_t tx)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	int ret;
	uint8_t tmpdata[3];
	uint16_t sw_address = (ee_address >> 8) | (ee_address << 8);

	if (eeprom->initialized)
	{
		if (eeprom->ackpolling) return EEPROM_CMD_SKIPPED;
		
		memcpy(tmpdata, &sw_address, sizeof(sw_address)); //Data[0..1]
		tmpdata[2] = tx;
		ret = i2c_write_blocking(i2c1, eeprom->DeviceAddr, tmpdata, sizeof(tmpdata), false);
		if (ret == sizeof(tmpdata))
		{
			eeprom->ackpolling = true;			
		}
		else
		{
			rc = EEPROM_WRLEN_ERROR; /**< Write-Lenght mismatch */	
		}
	}
	else
	{
		return EEPROM_NOT_INITIALIZED;	
	}

	return rc;
}	

eeprom_error_t EE_WritePage(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datatx, size_t len)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	int ret;
	size_t wrlen = len + 2;
	uint8_t tmpdata[258];
	uint16_t sw_address = (ee_address >> 8) | (ee_address << 8);

	if (eeprom->initialized)
	{
		if (eeprom->ackpolling) return EEPROM_CMD_SKIPPED;
		
		memcpy(tmpdata, &sw_address, sizeof(sw_address)); //Data[0..1]
		memcpy(&tmpdata[2], datatx, len); //Data[2..end]
		ret = i2c_write_blocking(i2c1, eeprom->DeviceAddr, tmpdata, wrlen, false);
		if (ret == wrlen)
		{
			eeprom->ackpolling = true;
		}
		else
		{
			rc = EEPROM_WRLEN_ERROR; /**< Write-Lenght mismatch */	
		}
	}
	else
	{
		return EEPROM_NOT_INITIALIZED;	
	}
	
	return rc;
}


eeprom_error_t EE_RandomRead(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datarx, size_t len)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	
	return rc;
}
eeprom_error_t EE_RandomWrite(xeeprom_t *eeprom, uint16_t ee_address, uint8_t *datatx, size_t len)
{
	eeprom_error_t rc = EEPROM_NO_ERROR;
	
	return rc;		
}


/*
// Test EEPROM-Write Access
EEPROM_CalcWr(0, 64); // = OK
EEPROM_CalcWr(0, 63); // = NOK page_space=first_wr=64=falsch
EEPROM_CalcWr(1, 64); // = OK
EEPROM_CalcWr(1, 63); // = OK
EEPROM_CalcWr(63, 1); // = OK
EEPROM_CalcWr(63, 64); // = OK
EEPROM_CalcWr(32, 64); // = OK
EEPROM_CalcWr(64, 32); // = NOK page_space=first_wr=64=falsch
EEPROM_CalcWr(0, 1); // NOK page_space=first_wr=64=falsch
EEPROM_CalcWr(64, 32); // = NOK page_space=first_wr=64=falsch
while (1);
*/

void EE_Calc(uint32_t eeaddress, uint32_t len)
{
	uint32_t					page = 0;
	uint16_t					page_space;
	uint32_t					address;
	uint32_t					first_write_size;
	uint32_t					last_write_size = 0;
	uint32_t					num_writes;
	uint16_t					write_size = 0;

	// Calculate space available in first page
	page_space = (((eeaddress / EEPROM_24M01_PAGESIZE) + 1)*EEPROM_24M01_PAGESIZE) - eeaddress;

	// Calculate first write size
	if (page_space > EEPROM_24M01_PAGESIZE) {
		first_write_size = page_space - ((page_space / EEPROM_24M01_PAGESIZE)*EEPROM_24M01_PAGESIZE);
		if (first_write_size == 0) first_write_size = EEPROM_24M01_PAGESIZE;
	}
	else first_write_size = page_space;

	// calculate size of last write
	if (len > first_write_size)
		last_write_size = (len - first_write_size) % EEPROM_24M01_PAGESIZE;

	// Calculate how many writes we need
	if (len > first_write_size)

		num_writes = ((len - first_write_size) / EEPROM_24M01_PAGESIZE) + 2;
	else
		num_writes = 1;

	printf("\nadr: %lu,len: %lu, page_space=%u, first_wr_size=%u, last_wr_size=%u, num_writes= %u\r", eeaddress, len, (unsigned int)page_space, (unsigned int)first_write_size, (unsigned int)last_write_size, (unsigned int)num_writes);

	address = eeaddress;

	for (page = 0; page < num_writes; page++)
	{
		if (page == 0)
		{
			if (len < first_write_size) write_size = len;
			else write_size = first_write_size;
		}
		else if (page == (num_writes - 1))
		{
			if (last_write_size) write_size = last_write_size;
			else return;
		}
		else write_size = EEPROM_24M01_PAGESIZE;

		printf("\nWrite to: %u size %u\r", (unsigned int)address, (unsigned int)write_size);
		address += write_size; // Increment address for next write
	}
	printf("\nlast_address= %lu, size= %u\r", (long unsigned int)address - write_size, (unsigned int)write_size);
	return;
}


