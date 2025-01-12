/*****************************************************************************/
/**
* @file xpwm.h
*
* Ctrl-X PWM/Servo driver.
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
* 1.00  tt   31/0/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XPWM_H_
#define XPWM_H_

#include "hardware/pwm.h"

#define XPWM_WRAP   (20000) //Wrap-Counter Value for 20ms@Div. 125
#define XPWM_DIV   (125.f) //Wrap-Counter Value for 20ms
#define XPWM_MIN   (300) // Min = 300µs
#define XPWM_MAX   (2500) // Max = 2,5ms
#define XPWM_NEUTRAL   ((XPWM_MAX-XPWM_MIN)/2 + XPWM_MIN)

 /**@brief Error codes for the XGPIO driver.
  */
 typedef enum
 {
    XPWM_NO_ERROR		= 0x00,			    /**< No error. */
    XPWM_INVALID_PARAM	= 0x01,			    /**< Invalid parameter passed to function call. */
    XPWM_TIMEOUT		= 0x02,			    /**< Communication timeout. */
 } xpwm_error_t;

typedef struct
{
	uint16_t	set_val;
	uint16_t	pwm_slice;
	uint16_t	pwm_channel;
	bool		IsInitialized;
	bool		IsEnabled;
} xpwm_t;


 /**@brief		Initialize the Ctrl-X PWM
  * @param 
  * @param
  * @return		XPWM error code.
  */
xpwm_error_t XPWM_Init(xpwm_t *xpwm, uint8_t gpio);
xpwm_error_t XPWM_SetVal(xpwm_t *xpwm, uint16_t min, uint16_t max, uint16_t set);

#endif /* XPWM_H_ */