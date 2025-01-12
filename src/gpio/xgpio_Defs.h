/*****************************************************************************/
/**
* @file xgpio_Defs.h
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

#ifndef XGPIO_DEFS_H_
#define XGPIO_DEFS_H_

 #include <stdint.h>
 
typedef uint8_t(*xgpio_cb_rtcclk_fptr_t)(uint32_t events);
typedef uint8_t(*xgpio_cb_rtcint_fptr_t)(uint32_t events);
typedef uint8_t(*xgpio_cb_canint_fptr_t)(uint32_t events);
typedef uint8_t(*xgpio_cb_pwrok_fptr_t)(uint32_t events);

typedef struct
{
	uint8_t		HID; /**< Hardware ID from the RTC. */
	uint8_t		VID; /**< Version ID from the RTC. */
	uint8_t		DeviceAddr; /**< RTC device address. */

	bool		IsInitialized; /**< Boolean flag to indicate a successful initialization. */
	bool		IsPOREnabled; /**< Current state of the POR function of the INT pin. */
	bool		IsBSIEEnabled; /**< Current state of the BSIE function of the INT pin. */
	bool		IsEventIntEnabled; /**< Boolean flag to indicate the state of the EIE bit. */
	bool		IsEventHighLevel; /**< Boolean flag to indicate the use of high level events on EVI. */
	bool		IsPasswordEnabled; /**< Current state of the password function. */
	bool		IsAlarmEnabled; /**< Current state of the alarm function. */
	bool		IsTSEnabled; /**< Current state of the time stamp function. */
	bool		IsTSOverwriteEnabled; /**< Current state of the time stamp overwrite function. */
	bool		IsClkOutEnabled; /**< Current state of the CLKOUT function. */
	bool		IsChargeEnabled; /**< Current state of the charging function. */
	
	xgpio_cb_rtcclk_fptr_t	p_Int_RTCClk; /**< Pointer to RTC-Clock -Interruptfunction. */
	xgpio_cb_rtcint_fptr_t	p_Int_RTCInt; /**< Pointer to RTC-Alarm -Interruptfunction. */
	xgpio_cb_canint_fptr_t	p_Int_CAN; /**< Pointer to CAN-Interruptfunction. */
	xgpio_cb_pwrok_fptr_t 	p_Int_PwrOK; /**< Pointer to Power_OK-Interruptfunction. */
	
} xgpio_t;

 /**@brief Error codes for the XGPIO driver.
  */
 typedef enum
 {
    XGPIO_NO_ERROR		= 0x00,			    /**< No error. */
    XGPIO_INVALID_PARAM	= 0x01,			    /**< Invalid parameter passed to function call. */
    XGPIO_TIMEOUT		= 0x02,			    /**< Communication timeout. */
    XGPIO_NOT_INITIALIZED	= 0x03,			    /**< Device is not initialized. Please call the RV3028_Init function. */
    XGPIO_NOT_READY		= 0x04,			    /**< Device function is not ready. Please initialize them first. */
    XGPIO_WP_ACTIVE		= 0x05,			    /**< Device is write protected. Please unprotect the device first. */
    XGPIO_COMM_ERROR		= 0x06,			    /**< Communication error. */
 } xgpio_error_t;

#endif /* XGPIO_DEFS_H_ */
