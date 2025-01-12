/*****************************************************************************/
/**
* @file gpio.c
*
* Ctrl-X GPIO driver.
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
#include "xgpio.h"

extern xgpio_t XGPIO;

void gpio_callback(uint gpio, uint32_t events) {

	if (gpio == XGPIO_INT_RTC1HZ)		
	{
		XGPIO.p_Int_RTCClk(events);		
	}

	if (gpio == XGPIO_INT_RTCINT)		
	{
		XGPIO.p_Int_RTCInt(events);
	}

	if (gpio == XGPIO_INT_CAN)		
	{
		XGPIO.p_Int_CAN(events);
	}

	if (gpio == XGPIO_PWROK)		
	{
		XGPIO.p_Int_PwrOK(events);
	}
}


xgpio_error_t XGPIO_Init(xgpio_t *xgpio)
{

	// 	PowerOK (Input)
	gpio_init(XGPIO_PWROK);
	gpio_set_dir(XGPIO_PWROK, GPIO_IN);
	gpio_set_pulls(XGPIO_PWROK, 1, 0);
	
	// initializing Onboard-LED (Output)
	gpio_init(PICO_DEFAULT_LED_PIN); 	
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	
	// Enable 24V (Output)
	gpio_init(XGPIO_EN24V); 	
	gpio_set_dir(XGPIO_EN24V, GPIO_OUT);	
	
	// 	DB-Pin-Description CLKOUT: Clock Output; push-pull (Input)
	gpio_init(XGPIO_INT_RTC1HZ);
	gpio_set_dir(XGPIO_INT_RTC1HZ, GPIO_IN);
	
	// 	DB-Pin-Description INT: Interrupt Output; open-drain; active LOW; requires pull-up resistor when used (Input)
	gpio_init(XGPIO_INT_RTCINT);
	gpio_set_dir(XGPIO_INT_RTCINT, GPIO_IN);	
	gpio_pull_up(XGPIO_INT_RTCINT);

	// 	DB-Pin-Description INT:  Interrupt output pin, Low-active (Input)
	gpio_init(XGPIO_INT_CAN);
	gpio_set_dir(XGPIO_INT_CAN, GPIO_IN);
	
	// initializing GPIO-Interrupts
	gpio_set_irq_enabled_with_callback(XGPIO_INT_RTC1HZ, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
	gpio_set_irq_enabled(XGPIO_PWROK, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);	
	gpio_set_irq_enabled(XGPIO_INT_RTCINT, GPIO_IRQ_EDGE_RISE, true);
	gpio_set_irq_enabled(XGPIO_INT_CAN, GPIO_IRQ_EDGE_FALL, true);
		
	return XGPIO_NO_ERROR;
}


