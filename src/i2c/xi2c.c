/*****************************************************************************/
/**
* @file xi2c.c
*
* Micro Crystal RV3028 I2C extreme low power RTC driver.
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
#include <stdio.h> // sprintf()
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "xi2c.h"
#include "../uart/tt_uart.h"

  /* from Silabs-SDK: In some situations, after a reset during an I2C transfer, the slave
     device may be left in an unknown state. Send 9 clock pulses to
     set slave in a defined state. */
//for (i = 0; i < 9; i++) {
//	GPIO_PinOutSet(init->sclPort, init->sclPin);
//	UDELAY_Delay(6);
//	GPIO_PinOutClear(init->sclPort, init->sclPin);
//	UDELAY_Delay(6);
//} 

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

static bool SDA_GetVal(uint8_t pin)
{
	return gpio_get(pin);	
}

static bool SCL_GetVal(uint8_t pin)
{
	return gpio_get(pin);	
}

void SCL_ClrVal(uint8_t pin)
{
	gpio_put(pin, 0);
}

static void SCL_SetDir(uint8_t pin, bool Dir)
{
	if (Dir) {

		gpio_init(pin); 	
		gpio_set_dir(pin, GPIO_OUT);	
	}
	else {
		gpio_init(pin); 	
		gpio_set_dir(pin, GPIO_IN);	
	}
} 

static void SDA_SetDir(uint8_t pin, bool Dir)
{
	if (Dir) {

		gpio_init(pin); 	
		gpio_set_dir(pin, GPIO_OUT);	
	}
	else {
		gpio_init(pin); 	
		gpio_set_dir(pin, GPIO_IN);	
	}
} 


bool i2c0_ResetBus(uint8_t pin_sda, uint8_t pin_scl)
{
	char i;

	if (SDA_GetVal(pin_sda) && SCL_GetVal(pin_scl)) {
		return true;
	}
	SCL_SetDir(pin_scl,GPIO_IN);
	SDA_SetDir(pin_sda, GPIO_IN);

	sleep_us(10);
	if (!SCL_GetVal(pin_scl)) {
		return false; /* SCL held low externally, nothing we can do */
	}
	for (i = 0; i < 9; i++) {
		/* up to 9 clocks until SDA goes high */
		SCL_SetDir(pin_scl, GPIO_OUT);
		SCL_ClrVal(pin_scl);
		sleep_us(10);
		SCL_SetDir(pin_scl, GPIO_IN);
		sleep_us(10);
		if (SDA_GetVal(pin_sda)) {
			break; /* finally SDA high so we can generate a STOP */
		}
	} /* for */
	if (!SDA_GetVal(pin_sda)) {
		return false; /* after 9 clocks still nothing */
	}

	return (SDA_GetVal(pin_sda) && SCL_GetVal(pin_scl)); /* both high then we succeeded */
}

bool i2c1_ResetBus(uint8_t pin_sda, uint8_t pin_scl)
{
	char i;

	if (SDA_GetVal(pin_sda) && SCL_GetVal(pin_scl)) {
		return true;
	}
	SCL_SetDir(pin_scl, GPIO_IN);
	SDA_SetDir(pin_sda, GPIO_IN);

	sleep_us(10);
	if (!SCL_GetVal(pin_scl)) {
		return false; /* SCL held low externally, nothing we can do */
	}
	for (i = 0; i < 9; i++) {
		/* up to 9 clocks until SDA goes high */
		SCL_SetDir(pin_scl, GPIO_OUT);
		SCL_ClrVal(pin_scl);
		sleep_us(10);
		SCL_SetDir(pin_scl, GPIO_IN);
		sleep_us(10);
		if (SDA_GetVal(pin_sda)) {
			break; /* finally SDA high so we can generate a STOP */
		}
	} /* for */
	if (!SDA_GetVal(pin_sda)) {
		return false; /* after 9 clocks still nothing */
	}

	return (SDA_GetVal(pin_sda) && SCL_GetVal(pin_scl)); /* both high then we succeeded */
}


void i2c0_Init(uint8_t pin_sda, uint8_t pin_scl, uint baudrate, bool pullup)
{
		
	i2c_init(i2c0, baudrate);
	/* Output value must be set to 1 to not drive lines low.
	 * Set SCL first, to ensure it is high before changing SDA. */ 
	gpio_set_function(pin_scl, GPIO_FUNC_I2C);
	gpio_set_function(pin_sda, GPIO_FUNC_I2C);

	if (pullup)
	{
		gpio_pull_up(pin_scl);
		gpio_pull_up(pin_sda);
	}
	else
	{
		gpio_disable_pulls(pin_scl);
		gpio_disable_pulls(pin_sda);
	}

	if (pin_sda == PICO_I2C0SET0_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET0_SDA_PIN, PICO_I2C0SET0_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C0SET1_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET1_SDA_PIN, PICO_I2C0SET1_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C0SET2_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET2_SDA_PIN, PICO_I2C0SET2_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C0SET3_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET3_SDA_PIN, PICO_I2C0SET3_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C0SET4_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET4_SDA_PIN, PICO_I2C0SET4_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C0SET5_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C0SET5_SDA_PIN, PICO_I2C0SET5_SCL_PIN, GPIO_FUNC_I2C));
	}
	else 
	{
		// Make the I2C pins available to picotool
		bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));		
	}
}
	
void i2c1_Init(uint8_t pin_sda, uint8_t pin_scl, uint baudrate, bool pullup)
{
		
	i2c_init(i2c1, baudrate);
	/* Output value must be set to 1 to not drive lines low.
	 * Set SCL first, to ensure it is high before changing SDA. */ 
	gpio_set_function(pin_scl, GPIO_FUNC_I2C);
	gpio_set_function(pin_sda, GPIO_FUNC_I2C);

	if (pullup)
	{
		gpio_pull_up(pin_scl);
		gpio_pull_up(pin_sda);
	}
	else
	{
		gpio_disable_pulls(pin_scl);
		gpio_disable_pulls(pin_sda);
	}
	
	if (pin_sda == PICO_I2C1SET0_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET0_SDA_PIN, PICO_I2C1SET0_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C1SET1_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET1_SDA_PIN, PICO_I2C1SET1_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C1SET2_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET2_SDA_PIN, PICO_I2C1SET2_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C1SET3_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET3_SDA_PIN, PICO_I2C1SET3_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C1SET4_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET4_SDA_PIN, PICO_I2C1SET4_SCL_PIN, GPIO_FUNC_I2C));
	}
	else if (pin_sda == PICO_I2C1SET5_SDA_PIN)
	{
		bi_decl(bi_2pins_with_func(PICO_I2C1SET5_SDA_PIN, PICO_I2C1SET5_SCL_PIN, GPIO_FUNC_I2C));
	}
	else 
	{
		// Make the I2C pins available to picotool
		bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));		
	}
}

/* Quell: c:\pico-sdk\examples\i2c\bus_scan\ */
int I2C_Scan(i2c_inst_t *i2c) {

	char tmpTx[100];		

	if (i2c == i2c0)
	{
		uart_puts(UART_ID, "\nI2C0 Bus Scan\r\n");		
	}
	
	if (i2c == i2c1)
	{
		uart_puts(UART_ID, "\nI2C1 Bus Scan\r\n");		
	}
	
	uart_puts(UART_ID, "   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");
	 
	for (int addr = 0; addr < (1 << 7); ++addr) {
		if (addr % 16 == 0) {
			sprintf(tmpTx, "%02x ", addr);
			uart_puts(UART_ID, tmpTx);
		}
 
		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.
 
		// Skip over any reserved addresses.
		int ret;
		uint8_t rxdata;
		
		if (reserved_addr(addr))
			ret = PICO_ERROR_GENERIC;
		else
			ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);
							
		sprintf(tmpTx, ret < 0 ? "." : "@");
		uart_puts(UART_ID, tmpTx);
		sprintf(tmpTx, addr % 16 == 15 ? "\r\n" : "  ");
		uart_puts(UART_ID, tmpTx);
	}
	return 0;
}
