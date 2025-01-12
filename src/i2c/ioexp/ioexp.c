/*****************************************************************************/
/**
* @file ioexp.c
*
* Ctrl-X IO-Expander driver.
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
#include "ioexp.h"

ioexp_error_t IoExp_Init(ioexp_t *ioexp)
{
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	return valrc;
}

ioexp_error_t IoExp_ReadDI(ioexp_t *ioexp)
{
	int rc;
	uint8_t mask = 0x1;
	uint8_t rdlen = 2;
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	rc = i2c_read_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, rdlen, false);
	if (rc == rdlen)
	{
		if (ioexp->Data[0] != ioexp->Data[1])
		{
			valrc = IOEXP_NOT_VALID;	
		} 
		else
		{
			for (uint8_t i = 0x0; i < 8; i++)
			{
				bool val = (bool)(mask & ~ioexp->Data[0]);
				ioexp->DI[i] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;
				mask = mask << 1;
			}
		}
	}
	else
	{
		valrc = IOEXP_RDLEN_ERROR;	
	}
	return valrc;
}

ioexp_error_t IoExp_ReadWriteDIO(ioexp_t *ioexp)
{
	int rc;
	uint8_t rdlen = 2;
	uint8_t wrlen = 2;
	uint8_t tmpval = 0;
	bool val;
	
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	rc = i2c_read_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, rdlen, false);
	if (rc == rdlen)
	{
		if (ioexp->Data[0] != ioexp->Data[1])
		{
			valrc = IOEXP_NOT_VALID;	
		} 
		else
		{
			val = (bool)(IOEXP_DI_STATDO_MASK & ioexp->Data[0]);
			ioexp->DI[4] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;
			
			val = (bool)(IOEXP_DI_BTN0_MASK & ioexp->Data[0]);
			ioexp->DI[6] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;

			val = (bool)(IOEXP_DI_BTN1_MASK & ioexp->Data[0]);
			ioexp->DI[7] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;					
		}

		tmpval = ~((ioexp->DO[0] << 0) |  (ioexp->DO[1] << 1) | (ioexp->DO[2] << 2) | (ioexp->DO[3] << 3) | (ioexp->DO[5] << 5)) | (1 << 4) | (1 << 6) | (1 << 7);
		ioexp->Data[0] = tmpval;
		ioexp->Data[1] = ioexp->Data[0];
		rc = i2c_write_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, wrlen, false);
		if (rc != wrlen)
		{
			valrc = IOEXP_WRLEN_ERROR; /**< Write-Lenght mismatch */
		}
	}
	else
	{
		valrc = IOEXP_RDLEN_ERROR;	
	}
	return valrc;
}


ioexp_error_t IoExp_ReadDIO(ioexp_t *ioexp)
{
	int rc;
	uint8_t rdlen = 2;
	bool val;
	
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	rc = i2c_read_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, rdlen, false);
	if (rc == rdlen)
	{
		if (ioexp->Data[0] != ioexp->Data[1])
		{
			valrc = IOEXP_NOT_VALID;	
		} 
		else
		{
			val = (bool)(IOEXP_DI_STATDO_MASK & ioexp->Data[0]);
			ioexp->DI[4] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;
			
			val = (bool)(IOEXP_DI_BTN0_MASK & ioexp->Data[0]);
			ioexp->DI[6] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;

			val = (bool)(IOEXP_DI_BTN1_MASK & ioexp->Data[0]);
			ioexp->DI[7] = (val) ? IOEXP_DI_BIT_H : IOEXP_DI_BIT_L;					
		}
	}
	else
	{
		valrc = IOEXP_RDLEN_ERROR;	
	}
	return valrc;
}

ioexp_error_t IoExp_WriteDIO(ioexp_t *ioexp)
{
	int rc;
	uint8_t wrlen = 2;
	uint8_t tmpval = 0;
	
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	tmpval = ~((ioexp->DO[0] << 0) |  (ioexp->DO[1] << 1) | (ioexp->DO[2] << 2) | (ioexp->DO[3] << 3) | (ioexp->DO[5] << 5)) | (1 << 4) | (1 << 6) | (1 << 7);
	ioexp->Data[0] = tmpval;
	ioexp->Data[1] = ioexp->Data[0];
	rc = i2c_write_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, wrlen, false);
	if (rc != wrlen)
	{
		valrc = IOEXP_WRLEN_ERROR; /**< Write-Lenght mismatch */
	}

	return valrc;
}

ioexp_error_t IoExp_WriteRE(ioexp_t *ioexp)
{
	int rc;
	uint8_t wrlen = 2;
	uint8_t tmpval = 0;
	ioexp_error_t valrc = IOEXP_NO_ERROR;
	
	tmpval = ~((ioexp->DO[0] << 0) |  (ioexp->DO[1] << 1) | (ioexp->DO[2] << 2) | (ioexp->DO[3] << 3)) & 0xF;
	tmpval |= (ioexp->DO[4] << 4) | (ioexp->DO[5] << 5) | (ioexp->DO[6] << 6) | (ioexp->DO[7] << 7);
	ioexp->Data[0] = tmpval;
	ioexp->Data[1] = ioexp->Data[0];
	
	rc = i2c_write_blocking(i2c1, ioexp->DeviceAddr, ioexp->Data, wrlen, false);
	if (rc != wrlen)
	{
		valrc = IOEXP_RDLEN_ERROR; /**< Write-Lenght mismatch */
	}
	return valrc;
}
