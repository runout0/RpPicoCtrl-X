/*****************************************************************************/
/**
* @file xspi.c
*
* Ctrl-X SPI-Driver
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

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/resets.h"
#include "xspi.h"
//#include "max31865/MAX31865.h"
#include "sram/23LCV512.h" // SRAM_ReadByte()

#define WRITE_BIT 0x80

static inline void spi_reset(spi_inst_t *spi) {
	invalid_params_if(SPI, spi != spi0 && spi != spi1);
	reset_block(spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);
}

static inline void spi_unreset(spi_inst_t *spi) {
	invalid_params_if(SPI, spi != spi0 && spi != spi1);
	unreset_block_wait(spi == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);
}
spi_error_t CSn_Init(void)
{	
	spi_error_t rc = SPI_NO_ERROR;
	
	// initializing SPI-/CS 
	gpio_init(PICO_SPI_CSEN1); 	
	gpio_set_dir(PICO_SPI_CSEN1, GPIO_OUT);
	gpio_put(PICO_SPI_CSEN1, 1); // passiv setzen
	
	gpio_init(PICO_SPI_CSEN2); 	
	gpio_set_dir(PICO_SPI_CSEN2, GPIO_OUT);
	gpio_put(PICO_SPI_CSEN2, 1); // passiv setzen
	
	gpio_init(PICO_SPI_A0); 	
	gpio_set_dir(PICO_SPI_A0, GPIO_OUT);	
	gpio_init(PICO_SPI_A1); 	
	gpio_set_dir(PICO_SPI_A1, GPIO_OUT);
				
	// Make the CS pin available to picotool
	bi_decl(bi_4pins_with_names(PICO_SPI_CSEN1, "CSN_CSEN1", PICO_SPI_CSEN2, "CSN_CSEN2",PICO_SPI_A0, "CSN_A0", PICO_SPI_A1, "CSN_A1"));
	return rc;
}

uint spi0_Init(uint baudrate_set, spi_cpol_t cpol, spi_cpha_t cpha, MCP2515_t *mcp2515)
{	
	uint baudrate_get;
	
	spi_reset(spi0);
	spi_unreset(spi0);
	
	baudrate_get = spi_init(spi0, baudrate_set);
	
	spi_set_format(spi0, 8, cpol, cpha, SPI_MSB_FIRST); //SPI-Mode 3 = CPOL1/CPHA1
	
	gpio_set_function(PICO_SPI0_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_SPI0_MOSI_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_SPI0_MISO_PIN, GPIO_FUNC_SPI);

	// Make the SPI pins available to picotool
	bi_decl(bi_3pins_with_func(PICO_SPI0_MISO_PIN, PICO_SPI0_MOSI_PIN, PICO_SPI0_SCK_PIN, GPIO_FUNC_SPI));
	
	MCP2515_readRegister(mcp2515, MCP_CANSTAT); //useless read to to force CLK-Pin = High
	return baudrate_get;
}
	
uint spi1_Init(uint baudrate_set, spi_cpol_t cpol, spi_cpha_t cpha)
{	
	uint baudrate_get;

	spi_reset(spi1);
	spi_unreset(spi1);
	
	baudrate_get = spi_init(spi1, baudrate_set);

	spi_set_format(spi1, 8, cpol, cpha, SPI_MSB_FIRST); //SPI-Mode 3 = CPOL1/CPHA1
	
	gpio_set_function(PICO_SPI1_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_SPI1_MOSI_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_SPI1_MISO_PIN, GPIO_FUNC_SPI);

	// Make the SPI pins available to picotool
	bi_decl(bi_3pins_with_func(PICO_SPI1_MISO_PIN, PICO_SPI1_MOSI_PIN, PICO_SPI1_SCK_PIN, GPIO_FUNC_SPI));
	
	SRAM_ReadByte(spi1, csn_sram, 0x00, false); //useless read to to force CLK-Pin = High
	return baudrate_get;
}

/* Kodierung 74HC139 ab HW V1.3
 * 
 * Einggänge:
 * Pin 1A0 = 2A0 = CS0
 * Pin 1A1 = 2A1 = CS1
 * Pin /1E = ChipSelect_extern (CS1, CS2, CAN)
 * Pin /2E = ChipSelect_intern Pt100_1, Pt100_2, SRAM
 *
 * Ausgänge
 * /1Y0 = EXT_A1 -> SPI0, nA0/nA1 = 0/0
 * /1Y1 = EXT_CS2 -> SPI0, nA0/nA1 = 1/0
 * /1Y2 = CAN_CS -> SPI0, nA0/nA1 = 0/1
 * /1Y3 = TP206, nA0/nA1 = 1/1
 * /2Y0 = PT2_CS -> SPI1, nA0/nA1 = 0/0
 * /2Y1 = PT1_CS -> SPI1, nA0/nA1 = 1/0
 * /2Y2 = SRAM_CS -> SPI1, nA0/nA1 = 0/1
 * /2Y3 = TP207, nA0/nA1 = 1/1
 *
 */ 

spi_error_t CSn_Set(spi_csn_t cs)
{
	spi_error_t rc = SPI_NO_ERROR;
	//asm volatile("nop \n nop \n nop");

	switch (cs)
	{
	case csn_ext_cs1: // SPI0/EXT_A1 = 1Y0
		gpio_put(PICO_SPI_A0, 0); 
		gpio_put(PICO_SPI_A1, 0);
		gpio_put(PICO_SPI_CSEN1, 0); 
		gpio_put(PICO_SPI_CSEN2, 1); 
		break;

	case csn_ext_cs2: // SPI0/EXT_A1 = 1Y1
		gpio_put(PICO_SPI_A0, 1); 
		gpio_put(PICO_SPI_A1, 0);		
		gpio_put(PICO_SPI_CSEN1, 0); 
		gpio_put(PICO_SPI_CSEN2, 1); 
		break;

	case csn_can: // SPI0/EXT_CAN = 1Y2
		gpio_put(PICO_SPI_A0, 0); 
		gpio_put(PICO_SPI_A1, 1);	
		gpio_put(PICO_SPI_CSEN1, 0); 
		gpio_put(PICO_SPI_CSEN2, 1); 
		break;

	case csn_tp206: // SPI0/TP206 = 1Y3
		gpio_put(PICO_SPI_A0, 1); 
		gpio_put(PICO_SPI_A1, 1);	
		gpio_put(PICO_SPI_CSEN1, 0); 
		gpio_put(PICO_SPI_CSEN2, 1); 
		break;
		
		case csn_pt1: // SPI1/PT1_CS = 2Y1
		gpio_put(PICO_SPI_A0, 1); 
		gpio_put(PICO_SPI_A1, 0);	
		gpio_put(PICO_SPI_CSEN1, 1); 
		gpio_put(PICO_SPI_CSEN2, 0); 
		break;
	case csn_pt2: // SPI1/PT2_CS = 2Y0
		gpio_put(PICO_SPI_A0, 0); 
		gpio_put(PICO_SPI_A1, 0);	
		gpio_put(PICO_SPI_CSEN1, 1); 
		gpio_put(PICO_SPI_CSEN2, 0); 		
		break;
			
	case csn_sram: // SPI1/SRAM_CS = 2Y2
		gpio_put(PICO_SPI_A0, 0); 
		gpio_put(PICO_SPI_A1, 1);	
		gpio_put(PICO_SPI_CSEN1, 1); 
		gpio_put(PICO_SPI_CSEN2, 0); 
		break;

	case csn_tp207: // SPI1/TP207 = 2Y3
		gpio_put(PICO_SPI_A0, 1); 
		gpio_put(PICO_SPI_A1, 1);			
		gpio_put(PICO_SPI_CSEN1, 1); 
		gpio_put(PICO_SPI_CSEN2, 0); 
		break;
					
	case csn_disable:
		gpio_put(PICO_SPI_CSEN1, 1);
		gpio_put(PICO_SPI_CSEN2, 1);
		gpio_put(PICO_SPI_A0, 1); // -> xY3
		gpio_put(PICO_SPI_A1, 1); // -> xY3
		break;
		
	default:	

		break;		
	}	
	//asm volatile("nop \n nop \n nop");
	return rc;
}

void spi_write_register(spi_inst_t *spix, enum spi_csn cs, const uint8_t reg, const uint8_t data) {
	uint8_t buf[2];
	buf[0] = WRITE_BIT | reg;
	buf[1] = data;

	CSn_Set(cs);
	spi_write_blocking(spix, buf, 2);
	CSn_Set(csn_disable);
	//sleep_ms(1);
}

int spi_read_registers(spi_inst_t *spix, enum spi_csn cs, uint8_t reg, uint8_t *buf, const uint16_t len) {

	int num_bytes_read;
	uint8_t msg = 0x7f & reg;
		
	CSn_Set(cs);
	spi_write_blocking(spix, &msg, 1);
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");
	num_bytes_read = spi_read_blocking(spix, 0, buf, len);
	CSn_Set(csn_disable);
	return num_bytes_read;
}
