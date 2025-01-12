/*****************************************************************************/
/**
* @file xspi.h
*
* Ctrl-X SPI driver.
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
* 1.00  tt   25/06/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XSPI_H_
#define XSPI_H_

#include "hardware/spi.h"
#include "mcp2515/mcp2515.h"

#define PICO_SPI0  (0)
#define PICO_SPI1  (1)

#define PICO_SPI0_SCK_PIN  2
#define PICO_SPI0_MOSI_PIN  3
#define PICO_SPI0_MISO_PIN  4

#define PICO_SPI1_SCK_PIN  10
#define PICO_SPI1_MOSI_PIN  11
#define PICO_SPI1_MISO_PIN  12

// Control of 74HC139, Dual 2-to-4 line decoder/demultiplexer
#define PICO_SPI_CSEN1       6 /*Output: SPI0-/CS - Enable, Low = Enable*/
#define PICO_SPI_CSEN2       7 /*Output: SPI1-/CS - Enable, Low = Enable*/
#define PICO_SPI_A0       13 /*Output: SPI-/CS0*/
#define PICO_SPI_A1       22 /*Output: SPI-/CS1*/

typedef enum
{
	SPI_NO_ERROR = 0x00, /**< No error. */
	SPI_INVALID_PARAM = 0x01, /**< Invalid parameter passed to function call. */
	SPI_TIMEOUT = 0x02, /**< Communication timeout. */
	SPI_NOT_INITIALIZED = 0x03, /**< Device is not initialized. Please call the SPI_Init function. */
	SPI_NOT_READY = 0x04, /**< Device function is not ready. Please initialize them first. */
	SPI_WRONG_BAUD = 0x05, /**< Device Baudrate not set correctly. */
} spi_error_t;

spi_error_t CSn_Init(void);
//void CSsn_Set(enum spi_csn);
spi_error_t CSn_Set(spi_csn_t cs);
uint spi0_Init(uint baudrate_set, spi_cpol_t cpol, spi_cpha_t cpha, MCP2515_t *mcp2515);
uint spi1_Init(uint baudrate_set, spi_cpol_t cpol, spi_cpha_t cpha);	

void spi_write_register(spi_inst_t *spix, enum spi_csn cs, uint8_t reg, const uint8_t data);
int spi_read_registers(spi_inst_t *spix, enum spi_csn cs, uint8_t reg, uint8_t *buf, const uint16_t len);

#endif /* XSPI_H_ */