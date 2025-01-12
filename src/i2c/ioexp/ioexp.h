/*****************************************************************************/
/**
* @file ioexp.h
* 
* I/O-Expander PCA9500PW I2C Driver.
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
* 1.00  tt   06/09/2023  First release
*
* </pre>
******************************************************************************/

#ifndef IOEXP_H_
#define IOEXP_H_

#include "ioexp_Defs.h"

 /**@brief		IO-Expander Init I/O
  * @param ioexp-Structure
  * @return		Communication error code.
  */
ioexp_error_t IoExp_Init(ioexp_t *ioexp);

/**@brief		IO-Expander Read I/O
 * @param ioexp-Structure
 * @return		Communication error code.
 */
ioexp_error_t IoExp_ReadDI(ioexp_t *ioexp);
ioexp_error_t IoExp_ReadWriteDIO(ioexp_t *ioexp);
ioexp_error_t IoExp_ReadDIO(ioexp_t *ioexp);
ioexp_error_t IoExp_WriteDIO(ioexp_t *ioexp);
ioexp_error_t IoExp_WriteRE(ioexp_t *ioexp);

/**@brief		Configure the periodic update interrupt.
 * @param p_Device	Pointer to RV3028 device structure.
 * @param Source	Update source.
 * @param UseInt	Set to #true to enable the INT pin.
 * @return		Communication error code.
 */
 // rv3028_error_t RV3028_InitUpdate(rv3028_t* p_Device, rv3028_ud_src_t Source, bool UseInt);

#endif /* IOEXP_H_ */