/*****************************************************************************/
/**
* @file 23LCV512.h
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

#ifndef _SRAM_H
#define _SRAM_H

#include "hardware/spi.h"
#include "../xspi.h"

#define	SRAM_INSTR_READ  (0x03)
#define	SRAM_INSTR_WRITE (0x02)
#define	SRAM_INSTR_EDIO  (0x3B)
#define	SRAM_INSTR_RSTIO (0xFF)
#define	SRAM_INSTR_RDMR  (0x05)
#define	SRAM_INSTR_WRMR  (0x01)

enum SRAM_ERROR {
	SRAM_ERROR_OK        = 0,
	SRAM_ERROR_FAIL      = 1,
};

enum SRAM_MODE {
	SRAM_MBYTE   = 0x0,
	SRAM_MPAGE = 0x80,
	SRAM_MSEQUE = 0x40,
};

enum SRAM_ERROR SRAM_Init(spi_inst_t *spi, enum spi_csn cs_pin, uint8_t mode);
uint8_t SRAM_ReadByte(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, bool initmode);
enum SRAM_ERROR SRAM_WriteByte(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t val, bool initmode);
enum SRAM_ERROR SRAM_ReadPage(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t *buffer, uint8_t n, bool initmode);
enum SRAM_ERROR SRAM_WritePage(spi_inst_t *spi, enum spi_csn cs_pin, uint16_t addr, uint8_t *buffer, uint8_t n, bool initmode);

#endif
