/*****************************************************************************/
/**
* @file xspics.h
*
* Ctrl-X SPI driver chip-select.
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
* 1.00  tt   25/06/2023  First release
*
* </pre>
******************************************************************************/

#ifndef XSPICS_H_
#define XSPICS_H_

typedef enum spi_csn {
	csn_disable = 0,
	csn_pt1,
	csn_pt2,
	csn_ext_cs1,
	csn_ext_cs2,
	csn_can,
	csn_sram,
	csn_tp206,
	csn_tp207,
	CSN_COUNT
} spi_csn_t;

#endif /* XSPICS_H_ */