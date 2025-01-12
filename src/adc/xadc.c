/*****************************************************************************/
/**
* @file adc.c
*
* Ctrl-X adc driver.
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
#include "hardware/adc.h"
#include "xadc.h"

#define XADX_UREF (10.0f)

extern xadc_t XADC;

xadc_error_t xadc_Init(xadc_t *xadc)
{
	adc_init();

	adc_gpio_init(XADC_ADC0);
	adc_gpio_init(XADC_ADC1);
	adc_gpio_init(XADC_ADC2);

	xadc->convertion_factor = XADX_UREF / (1 << 12);		
	return XADC_NO_ERROR;
}


xadc_error_t xadc_Read(xadc_t *xadc, xadc_ch_t ch)
{
	adc_select_input(ch);	
	xadc->vraw[ch] = adc_read(); // one single conversion = 2us
	xadc->vfloat[ch] = xadc->vraw[ch] * xadc->convertion_factor;
	return XADC_NO_ERROR;
}


xadc_error_t xadc_Scheduler()
{
	static uint8_t cnt = 0;

	cnt++;
	if (cnt > ADC2) cnt = 0;
	
	if (cnt == ADC0) xadc_Read(&XADC, ADC0);			
	if (cnt == ADC1) xadc_Read(&XADC, ADC1);		
	if (cnt == ADC2) xadc_Read(&XADC, ADC2);		

	return XADC_NO_ERROR;
}
