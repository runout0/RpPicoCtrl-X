/*****************************************************************************/
/**
* @file xadc.h
*
* Ctrl-X ADC driver.
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
* 1.00  tt   04/07/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XADC_H_
#define XADC_H_

#define XADC_ADC0 26 /*ADC-Input 0..10V*/
#define XADC_ADC1 27 /*ADC-Input 0..10V*/
#define XADC_ADC2 28 /*ADC-Input 0..10V*/

typedef enum
{
	XADC_NO_ERROR = 0x00,
	/**< No error. */
	XADC_INVALID_PARAM = 0x01,
	/**< Invalid parameter passed to function call. */
	XADC_TIMEOUT = 0x02,
	/**< Communication timeout. */
	XADC_NOT_INITIALIZED = 0x03,
} xadc_error_t;

typedef enum
{
	ADC0 = 0x00,
	ADC1,
	ADC2,
	TEMP0
} xadc_ch_t;

typedef struct
{
	float convertion_factor;
	uint16_t		vraw[4];
	float		vfloat[4];	
} xadc_t; 

/**@brief		Initialize the Ctrl-X ADC
 * @param 
 * @param
 * @return		GPIO error code.
 */
xadc_error_t xadc_Init(xadc_t *xadc);
xadc_error_t xadc_Scheduler();

#endif /* XADC_H_ */