/*****************************************************************************/
/**
* @file xplc.c
*
* Ctrl-X PLC driver.
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
* 1.00  tt   23/11/2023  First release
*
* </pre>
******************************************************************************/
#include <string.h>
#include "xplc.h"

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void cpyCtrlx2PLC(xplc_t *xplc, ioexp_t *ioexp_di, ioexp_t *ioexp_re, ioexp_t *ioexp_dio, Max31865_t  *pt100_1, Max31865_t  *pt100_2, xadc_t *xadc)
{

	memcpy(xplc->IO.di_int, ioexp_di->DI, 0x8); // copy DI1..DI8
	xplc->IO.di_int[DI_INTERN0].DI[8] = ioexp_dio->DI[7]; // Button 1
	xplc->IO.di_int[DI_INTERN0].DI[9] = ioexp_dio->DI[6]; // Button 2
	xplc->IO.di_int[DI_INTERN0].DI[10] = ioexp_dio->DI[4]; // Status DO
	
	xplc->IO.ai_int[AI_INTERN0].AI[0] = xadc->vraw[0]; // Pico ADC0
	xplc->IO.ai_int[AI_INTERN0].AI[1] = xadc->vraw[1]; // Pico ADC1
	xplc->IO.ai_int[AI_INTERN0].AI[2] = xadc->vraw[2]; // Pico ADC2
	
	xplc->IO.ai_int[AI_INTERN0].PT[0] = pt100_1->tempC; // Temperature-PT1
	xplc->IO.ai_int[AI_INTERN0].PT[1] = pt100_2->tempC; // Temperature-PT2
}

void cpyPLC2Ctrlx(xplc_t *xplc, ioexp_t *ioexp_re, ioexp_t *ioexp_dio)
{

	memcpy(ioexp_re->DO, xplc->IO.do_int[DO_INTERN0].DO, 0x8); // copy LEDs/Relays
	ioexp_dio->DO[3] = 	xplc->IO.do_int[DO_INTERN0].DO[8]; // digital Output (DO1)
	ioexp_dio->DO[2] = 	xplc->IO.do_int[DO_INTERN0].DO[9]; // digital Output (DO2)
	ioexp_dio->DO[1] = 	xplc->IO.do_int[DO_INTERN0].DO[10]; // digital Output (DO3)
	ioexp_dio->DO[0] = 	xplc->IO.do_int[DO_INTERN0].DO[11]; // digital Output (DO4)
	ioexp_dio->DO[5] = 	xplc->IO.do_int[DO_INTERN0].DO[12]; // digital Output (Buzzer)	
}