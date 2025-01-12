/*****************************************************************************/
/**
* @file xgpio.h
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

#ifndef XGPIO_H_
#define XGPIO_H_

#include "xgpio_Defs.h"

#define XGPIO_PWROK      16 /*Input: Power-OK UB24V > 18,2V*/
#define XGPIO_EN24V      17 /*Output: 24V DO Enable*/
#define XGPIO_PWM        20 /*Output: PWM Servo*/

#define XGPIO_INT_RTCINT  5 /*Input: Interrupt RTC_Int*/
#define XGPIO_INT_RTC1HZ 19 /*Input: Interrupt RTC_1Hz*/
#define XGPIO_INT_CAN    21 /*Input: Interrupt CAN-Ctrl*/



 /**@brief		Initialize the Ctrl-X GPIOS
  * @param 
  * @param
  * @return		GPIO error code.
  */
xgpio_error_t XGPIO_Init(xgpio_t *p_xgpio);

 
#endif /* XGPIO_H_ */