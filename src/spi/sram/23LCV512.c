/*****************************************************************************/
/**
* @file 23LCV512.c
*
* Ctrl-X SRAM-driver.
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
* 1.00  tt   30/07/2023  First release
*
* </pre>
******************************************************************************/

#include <string.h> // memset()
#include "23LCV512.h"


enum SRAM_ERROR SRAM_Init(spi_inst_t *spi, enum spi_csn cs_pin, uint8_t mode)
{
	uint8_t data[4] ;
	uint16_t rc ;
	
	CSn_Set(cs_pin) ;
	data[0] = SRAM_INSTR_WRMR ;
	data[1] = mode ;
	rc = spi_write_blocking(spi, data, 2) ;		
	CSn_Set(csn_disable) ;

	if(rc == 2) return SRAM_ERROR_OK ;	
	return SRAM_ERROR_FAIL ;
}

uint8_t SRAM_ReadByte(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, bool initmode)
{
	uint8_t data[4] ;
	CSn_Set(cs_pin) ;

	if(initmode)
{
	data[0] = SRAM_INSTR_WRMR ;
	data[1] = SRAM_MBYTE ;
	spi_write_blocking(spi, data, 2) ;		
	CSn_Set(csn_disable) ;
	asm volatile("nop \n nop \n nop") ;
	asm volatile("nop \n nop \n nop") ;	
	CSn_Set(cs_pin) ;
}
	
data[0] = SRAM_INSTR_READ ;
memcpy(&data[1], &addr, sizeof(uint16_t)) ;
spi_write_blocking(spi, data, 3) ;

uint8_t ret ;
spi_read_blocking(spi, 0x00, &ret, 1) ;
CSn_Set(csn_disable) ;
return ret ;
};

enum SRAM_ERROR SRAM_WriteByte(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t val, bool initmode)
{
	uint8_t data[4] ;
	CSn_Set(cs_pin) ;

	if(initmode)
	{
		data[0] = SRAM_INSTR_WRMR ;
		data[1] = SRAM_MBYTE ;
		spi_write_blocking(spi, data, 2) ;		
		CSn_Set(csn_disable) ;
		asm volatile("nop \n nop \n nop") ;
		asm volatile("nop \n nop \n nop") ;	
		CSn_Set(cs_pin) ;
	}
	
	data[0] = SRAM_INSTR_WRITE ;
	data[3] = val ;
	memcpy(&data[1], &addr, sizeof(uint16_t)) ;
	uint16_t rc = spi_write_blocking(spi, data, 4) ;
	CSn_Set(csn_disable);
	
	if (rc == 4) return SRAM_ERROR_OK;
	return SRAM_ERROR_FAIL ;
}

enum SRAM_ERROR SRAM_ReadPage(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t *buffer, uint8_t n, bool initmode)
{
	uint8_t data[4];
	CSn_Set(cs_pin);

	if(initmode)
	{
		data[0] = SRAM_INSTR_WRMR ;
		data[1] = SRAM_MPAGE ;
		spi_write_blocking(spi, data, 2) ;		
		CSn_Set(csn_disable) ;
		asm volatile("nop \n nop \n nop") ;
		asm volatile("nop \n nop \n nop") ;	
		CSn_Set(cs_pin) ;
	}
	
	data[0] = SRAM_INSTR_READ ;
	memcpy(&data[1], &addr, sizeof(uint16_t)) ;
	spi_write_blocking(spi, data, 3) ;
	uint16_t rc = spi_read_blocking(spi, 0x00, buffer, n) ;
	CSn_Set(csn_disable);

	if (rc == n) return SRAM_ERROR_OK;		
	return SRAM_ERROR_FAIL;
}

enum SRAM_ERROR SRAM_WritePage(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t *buffer, uint8_t n, bool initmode)
{
	uint8_t data[3] ;
	CSn_Set(cs_pin) ;

	if(initmode)
	{
		data[0] = SRAM_INSTR_WRMR ;
		data[1] = SRAM_MPAGE ;
		spi_write_blocking(spi, data, 2) ;		
		CSn_Set(csn_disable) ;
		asm volatile("nop \n nop \n nop") ;
		asm volatile("nop \n nop \n nop") ;	
		CSn_Set(cs_pin) ;
	}
	
	data[0] = SRAM_INSTR_WRITE ;
	memcpy(&data[1], &addr, sizeof(uint16_t)) ;
	uint16_t rc0 = spi_write_blocking(spi, data, 3) ;
	uint16_t rc1 = spi_write_blocking(spi, buffer, n);
	CSn_Set(csn_disable) ;

	if (rc0 + rc1 == n+3) return SRAM_ERROR_OK;		
	return SRAM_ERROR_FAIL;
}