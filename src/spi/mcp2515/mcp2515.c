/*****************************************************************************/
/**
* @file mcp2515.c
*
* Ctrl-X MCP2515driver.
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
* 1.00  tt   06/07/2023  First release
*
* </pre>
******************************************************************************/

#include <string.h> // memset()
#include "mcp2515.h"
#include "../xspi.h" // CSn_Set()

enum MCP2515_ERROR MCP2515_init(MCP2515_t *mcp2515, spi_inst_t *spi, enum spi_csn cs_pin)
{
	mcp2515->spi = spi ;
	mcp2515->cs_pin = cs_pin ;

	TXBn_REGS_t tx0, tx1, tx2;
	RXBn_REGS_t rx0, rx1;
	
	tx0.v[IDX_TXBCTRL] = MCP_TXB0CTRL;
	tx0.v[IDX_TXBSIDH] = MCP_TXB0SIDH;
	tx0.v[IDX_TXBDATA] = MCP_TXB0DATA;
	
	tx1.v[IDX_TXBCTRL] = MCP_TXB1CTRL;
	tx1.v[IDX_TXBSIDH] = MCP_TXB1SIDH;
	tx1.v[IDX_TXBDATA] = MCP_TXB1DATA;
		 
	tx2.v[IDX_TXBCTRL] = MCP_TXB2CTRL;
	tx2.v[IDX_TXBSIDH] = MCP_TXB2SIDH;
	tx2.v[IDX_TXBDATA] = MCP_TXB2DATA;
	
	memcpy(&mcp2515->TXB[TXB0], &tx0, sizeof(TXBn_REGS_t)) ;
	memcpy(&mcp2515->TXB[TXB1], &tx1, sizeof(TXBn_REGS_t)) ;
	memcpy(&mcp2515->TXB[TXB2], &tx2, sizeof(TXBn_REGS_t)) ;
	
	rx0.v[IDX_RXBCTRL] = MCP_RXB0CTRL;
	rx0.v[IDX_RXBSIDH] = MCP_RXB0SIDH;
	rx0.v[IDX_RXBDATA] = MCP_RXB0DATA;
	rx0.v[IDX_RXBIF] = CANINTF_RX0IF;

	rx1.v[IDX_RXBCTRL] = MCP_RXB1CTRL;
	rx1.v[IDX_RXBSIDH] = MCP_RXB1SIDH;
	rx1.v[IDX_RXBDATA] = MCP_RXB1DATA;
	rx1.v[IDX_RXBIF] = CANINTF_RX1IF;

	memcpy(&mcp2515->RXB[RXB0], &rx0, sizeof(RXBn_REGS_t)) ;
	memcpy(&mcp2515->RXB[RXB1], &rx1, sizeof(RXBn_REGS_t)) ;
	return ERROR_OK ;
};


enum MCP2515_ERROR MCP2515_reset(MCP2515_t *mcp2515)
{
	uint8_t result ;

	CSn_Set(mcp2515->cs_pin) ;	
	uint8_t CAN_INSTR = MCP_INSTR_RESET ;
	spi_write_blocking(mcp2515->spi, &CAN_INSTR, 1) ;
	CSn_Set(csn_disable) ;

	//Depends on oscillator & capacitors used
	sleep_ms(10) ;

	uint8_t zeros[14] ;
	memset(zeros, 0, sizeof(zeros)) ;
	MCP2515_setRegisters(mcp2515, MCP_TXB0CTRL, zeros, 14) ; // Sendepuffer 0 loeschen -> bis D7 alles reseten
	MCP2515_setRegisters(mcp2515, MCP_TXB1CTRL, zeros, 14) ; // Sendepuffer 1 loeschen -> bis D7 alles reseten
	MCP2515_setRegisters(mcp2515, MCP_TXB2CTRL, zeros, 14) ; // Sendepuffer 2 loeschen -> bis D7 alles reseten

	MCP2515_setRegister(mcp2515, MCP_RXB0CTRL, 0) ; // Empfangssteuerregister 0 loeschen
	MCP2515_setRegister(mcp2515, MCP_RXB1CTRL, 0) ; // Empfangssteuerregister 1 loeschen

	MCP2515_setRegister(mcp2515, MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF) ; // -> 0xA3

	// receives all valid messages using either Standard or Extended Identifiers that
	// meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
	MCP2515_modifyRegister(mcp2515, MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT) ; // ->0x6704
	MCP2515_modifyRegister(mcp2515, MCP_RXB1CTRL, RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK, RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT) ; // ->0x6701

	// clear filters and masks
	// do not filter any standard frames for RXF0 used by RXB0
	// do not filter any extended frames for RXF1 used by RXB1
		
	// Registerzugrif auf 0x00, 0x04, 0x08, 0x10, 0x14, 0x18
	enum MCP2515_RXF filters[] = { RXF0, RXF1, RXF2, RXF3, RXF4, RXF5 } ;
	for(int i = 0 ; i < 6 ; i++)		
	{
		bool ext = (i == 1) ;
		result = MCP2515_setFilter(mcp2515, filters[i], ext, 0) ;
		if(result != ERROR_OK)
		{
			return result ;
		}
	}

	// Registerzugrif auf 0x20, 0x24
	enum MCP2515_MASK masks[] = { MASK0, MASK1 } ;
	for(int i = 0 ; i < 2 ; i++) 
	{
		result = MCP2515_setFilterMask(mcp2515, masks[i], true, 0) ;
		if(result != ERROR_OK) {
			return result ;
		}
	}  

	return ERROR_OK ;
}

uint8_t MCP2515_readRegister(MCP2515_t *mcp2515, const uint8_t reg)
{
	CSn_Set(mcp2515->cs_pin) ;
	uint8_t data[2] = { MCP_INSTR_READ, reg } ;
	spi_write_blocking(mcp2515->spi, data, 2) ;

	uint8_t ret ;
	spi_read_blocking(mcp2515->spi, 0x00, &ret, 1) ;
	CSn_Set(csn_disable) ;
	return ret ;
}

void MCP2515_readRegisters(MCP2515_t *mcp2515, const uint8_t reg, uint8_t values[], const uint8_t n)
{
	CSn_Set(mcp2515->cs_pin) ;
	uint8_t data[2] = { MCP_INSTR_READ, reg } ;
	spi_write_blocking(mcp2515->spi, data, 2) ;
	spi_read_blocking(mcp2515->spi, 0x00, values, n) ;
	CSn_Set(csn_disable) ;
}

void MCP2515_setRegister(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t value)
{
	CSn_Set(mcp2515->cs_pin) ;
	uint8_t data[3] = { MCP_INSTR_WRITE, reg, value } ;
	spi_write_blocking(mcp2515->spi, data, 3) ;
	CSn_Set(csn_disable) ;
}

void MCP2515_setRegisters(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t values[], const uint8_t n)
{
	CSn_Set(mcp2515->cs_pin) ;

	uint8_t data[2] = { MCP_INSTR_WRITE, reg } ;
	spi_write_blocking(mcp2515->spi, data, 2) ;
	spi_write_blocking(mcp2515->spi, values, n) ;
	CSn_Set(csn_disable) ;
}

void MCP2515_modifyRegister(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t mask, const uint8_t data)
{
	CSn_Set(mcp2515->cs_pin) ;
	uint8_t d[4] = { MCP_INSTR_BITMOD, reg, mask, data } ;
	spi_write_blocking(mcp2515->spi, d, 4) ;
	CSn_Set(csn_disable) ;
}

uint8_t MCP2515_getStatus(MCP2515_t *mcp2515)
{
	CSn_Set(mcp2515->cs_pin) ;

	uint8_t CAN_INSTR = MCP_INSTR_READ_STATUS ;
	spi_write_blocking(mcp2515->spi, &CAN_INSTR, 1) ;

	uint8_t ret ;
	spi_read_blocking(mcp2515->spi, 0x00, &ret, 1) ;
	CSn_Set(csn_disable) ;
	return ret ;
}

enum MCP2515_ERROR MCP2515_setConfigMode(MCP2515_t *mcp2515)
{
	return MCP2515_setMode(mcp2515, CANCTRL_REQOP_CONFIG) ;
}

enum MCP2515_ERROR MCP2515_setListenOnlyMode(MCP2515_t *mcp2515)
{
	return MCP2515_setMode(mcp2515, CANCTRL_REQOP_LISTENONLY) ;
}

enum MCP2515_ERROR MCP2515_setSleepMode(MCP2515_t *mcp2515)
{
	return MCP2515_setMode(mcp2515, CANCTRL_REQOP_SLEEP) ;  
}

enum MCP2515_ERROR MCP2515_setLoopbackMode(MCP2515_t *mcp2515)
{
	return MCP2515_setMode(mcp2515, CANCTRL_REQOP_LOOPBACK) ;
}

enum MCP2515_ERROR MCP2515_setNormalMode(MCP2515_t *mcp2515)
{
	return MCP2515_setMode(mcp2515, CANCTRL_REQOP_NORMAL) ;
}

enum MCP2515_ERROR MCP2515_setMode(MCP2515_t *mcp2515, const uint8_t mode)
{
	MCP2515_modifyRegister(mcp2515, MCP_CANCTRL, CANCTRL_REQOP, mode) ; // Register 0x0F, Maske (0xE), Data (0x80)

	unsigned long endTime = to_ms_since_boot(get_absolute_time()) + 10 ;
	bool modeMatch = false ;
	while(to_ms_since_boot(get_absolute_time()) < endTime) {
  
		uint8_t newmode = MCP2515_readRegister(mcp2515, MCP_CANSTAT);
		newmode &= CANSTAT_OPMOD ;
		modeMatch = newmode == mode ;
		if(modeMatch) {
			break ;
		}
	}
	return modeMatch ? ERROR_OK : ERROR_FAIL ;
}

enum MCP2515_ERROR MCP2515_setBitrate(MCP2515_t *mcp2515, const enum MCP2515_CAN_SPEED canSpeed)
{
	enum MCP2515_ERROR error = MCP2515_setConfigMode(mcp2515) ;
	if(error != ERROR_OK) {
		return error ;
	}

	uint8_t set, cfg1, cfg2, cfg3; 
	set = 1 ;

	switch(canSpeed)
	{
		case(CAN_5KBPS) :                                               //   5Kbps
		cfg1 = MCP_16MHz_5kBPS_CFG1 ;
		cfg2 = MCP_16MHz_5kBPS_CFG2 ;
		cfg3 = MCP_16MHz_5kBPS_CFG3 ;
		break ;

		case(CAN_10KBPS) :                                              //  10Kbps
		cfg1 = MCP_16MHz_10kBPS_CFG1 ;
		cfg2 = MCP_16MHz_10kBPS_CFG2 ;
		cfg3 = MCP_16MHz_10kBPS_CFG3 ;
		break ;

		case(CAN_20KBPS) :                                              //  20Kbps
		cfg1 = MCP_16MHz_20kBPS_CFG1 ;
		cfg2 = MCP_16MHz_20kBPS_CFG2 ;
		cfg3 = MCP_16MHz_20kBPS_CFG3 ;
		break ;

		case(CAN_33KBPS) :                                              //  33.333Kbps
		cfg1 = MCP_16MHz_33k3BPS_CFG1 ;
		cfg2 = MCP_16MHz_33k3BPS_CFG2 ;
		cfg3 = MCP_16MHz_33k3BPS_CFG3 ;
		break ;

		case(CAN_40KBPS) :                                              //  40Kbps
		cfg1 = MCP_16MHz_40kBPS_CFG1 ;
		cfg2 = MCP_16MHz_40kBPS_CFG2 ;
		cfg3 = MCP_16MHz_40kBPS_CFG3 ;
		break ;

		case(CAN_50KBPS) :                                              //  50Kbps
		cfg1 = MCP_16MHz_50kBPS_CFG1 ;
		cfg2 = MCP_16MHz_50kBPS_CFG2 ;
		cfg3 = MCP_16MHz_50kBPS_CFG3 ;
		break ;

		case(CAN_80KBPS) :                                              //  80Kbps
		cfg1 = MCP_16MHz_80kBPS_CFG1 ;
		cfg2 = MCP_16MHz_80kBPS_CFG2 ;
		cfg3 = MCP_16MHz_80kBPS_CFG3 ;
		break ;

		case(CAN_83K3BPS) :                                             //  83.333Kbps
		cfg1 = MCP_16MHz_83k3BPS_CFG1 ;
		cfg2 = MCP_16MHz_83k3BPS_CFG2 ;
		cfg3 = MCP_16MHz_83k3BPS_CFG3 ;
		break ; 

		case(CAN_100KBPS) :                                             // 100Kbps
		cfg1 = MCP_16MHz_100kBPS_CFG1 ;
		cfg2 = MCP_16MHz_100kBPS_CFG2 ;
		cfg3 = MCP_16MHz_100kBPS_CFG3 ;
		break ;

		case(CAN_125KBPS) :                                             // 125Kbps
		cfg1 = MCP_16MHz_125kBPS_CFG1 ;
		cfg2 = MCP_16MHz_125kBPS_CFG2 ;
		cfg3 = MCP_16MHz_125kBPS_CFG3 ;
		break ;

		case(CAN_200KBPS) :                                             // 200Kbps
		cfg1 = MCP_16MHz_200kBPS_CFG1 ;
		cfg2 = MCP_16MHz_200kBPS_CFG2 ;
		cfg3 = MCP_16MHz_200kBPS_CFG3 ;
		break ;

		case(CAN_250KBPS) :                                             // 250Kbps
		cfg1 = MCP_16MHz_250kBPS_CFG1 ;
		cfg2 = MCP_16MHz_250kBPS_CFG2 ;
		cfg3 = MCP_16MHz_250kBPS_CFG3 ;
		break ;

		case(CAN_500KBPS) :                                             // 500Kbps
		cfg1 = MCP_16MHz_500kBPS_CFG1 ;
		cfg2 = MCP_16MHz_500kBPS_CFG2 ;
		cfg3 = MCP_16MHz_500kBPS_CFG3 ;
		break ;

		case(CAN_1000KBPS) :                                            //   1Mbps
		cfg1 = MCP_16MHz_1000kBPS_CFG1 ;
		cfg2 = MCP_16MHz_1000kBPS_CFG2 ;
		cfg3 = MCP_16MHz_1000kBPS_CFG3 ;
		break ;

		default :
		set = 0 ;
		break ;
	}

	if(set) {
		MCP2515_setRegister(mcp2515, MCP_CNF1, cfg1) ;
		MCP2515_setRegister(mcp2515, MCP_CNF2, cfg2) ;
		MCP2515_setRegister(mcp2515, MCP_CNF3, cfg3) ;
		return ERROR_OK ;
	}
	else {
		return ERROR_FAIL ;
	}
}

enum MCP2515_ERROR MCP2515_setClkOut(MCP2515_t *mcp2515, const enum MCP2515_CAN_CLKOUT divisor)
{
	if(divisor == CLKOUT_DISABLE)
	{
		/* Turn off CLKEN */
		MCP2515_modifyRegister(mcp2515, MCP_CANCTRL, CANCTRL_CLKEN, 0x00) ;

		/* Turn on CLKOUT for SOF */
		MCP2515_modifyRegister(mcp2515, MCP_CNF3, CNF3_SOF, CNF3_SOF) ;
		return ERROR_OK ;
	}
  
	/* Set the prescaler (CLKPRE) */
	MCP2515_modifyRegister(mcp2515, MCP_CANCTRL, CANCTRL_CLKPRE, divisor) ;

	/* Turn on CLKEN */
	MCP2515_modifyRegister(mcp2515, MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN) ;

	/* Turn off CLKOUT for SOF, 0 = CLKOUT pin[3] is enabled for clock out function */
	MCP2515_modifyRegister(mcp2515, MCP_CNF3, CNF3_SOF, 0x00) ;
	return ERROR_OK ;
}

void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
	uint16_t canid = (uint16_t)(id & 0x0FFFF) ;

	if(ext)
	{
		buffer[MCP_EID0] = (uint8_t)(canid & 0xFF) ;
		buffer[MCP_EID8] = (uint8_t)(canid >> 8) ;
		canid = (uint16_t)(id >> 16) ;
		buffer[MCP_SIDL] = (uint8_t)(canid & 0x03) ;
		buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3) ;
		buffer[MCP_SIDL] |= TXB_EXIDE_MASK ;
		buffer[MCP_SIDH] = (uint8_t)(canid >> 5) ;
	} else 
	{
		buffer[MCP_SIDH] = (uint8_t)(canid >> 3) ;
		buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5) ;
		buffer[MCP_EID0] = 0 ;
		buffer[MCP_EID8] = 0 ;
	}
}

enum MCP2515_ERROR MCP2515_setFilterMask(MCP2515_t *mcp2515, const enum MCP2515_MASK mask, const bool ext, const uint32_t ulData)
{
	enum MCP2515_ERROR res = MCP2515_setConfigMode(mcp2515) ;
	if(res != ERROR_OK) {
	return res ;
	}
    
	uint8_t tbufdata[4] ;
	MCP2515_prepareId(tbufdata, ext, ulData) ;

	uint8_t reg ;
	switch(mask) {
		case MASK0 : reg = MCP_RXM0SIDH ; break ;
		case MASK1 : reg = MCP_RXM1SIDH ; break ;
		default :
				return ERROR_FAIL ;
	}

	MCP2515_setRegisters(mcp2515, reg, tbufdata, 4) ;  
	return ERROR_OK ;
}

enum MCP2515_ERROR MCP2515_setFilter(MCP2515_t *mcp2515, const enum MCP2515_RXF num, const bool ext, const uint32_t ulData)	
{
	enum MCP2515_ERROR res = MCP2515_setConfigMode(mcp2515) ;
	if(res != ERROR_OK) { return res ; }

	uint8_t reg ;

	switch(num) {
		case RXF0 : reg = MCP_RXF0SIDH ; break ;
		case RXF1 : reg = MCP_RXF1SIDH ; break ;
		case RXF2 : reg = MCP_RXF2SIDH ; break ;
		case RXF3 : reg = MCP_RXF3SIDH ; break ;
		case RXF4 : reg = MCP_RXF4SIDH ; break ;
		case RXF5 : reg = MCP_RXF5SIDH ; break ;
		default :
			return ERROR_FAIL ;
	}

	uint8_t tbufdata[4] ;
	MCP2515_prepareId(tbufdata, ext, ulData) ;
	MCP2515_setRegisters(mcp2515, reg, tbufdata, 4) ;

	return ERROR_OK ;
}

// sendet Tx-Buffer txbn, Aufruf aus "MCP2515_sendMessage1"
enum MCP2515_ERROR MCP2515_sendMessage0(MCP2515_t *mcp2515, const enum MCP2515_TXBn txbn, const struct can_frame *frame)
{
	if(frame->can_dlc > CAN_MAX_DLEN) { return ERROR_FAILTX ; }

	TXBn_REGS_t *ptxbuf = &mcp2515->TXB[txbn];
	uint8_t data[13] ;

	bool ext = (frame->can_id & CAN_EFF_FLAG) ;
	bool rtr = (frame->can_id & CAN_RTR_FLAG) ;
	uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK)) ;
		
	MCP2515_prepareId(data, ext, id) ;

	data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;
	memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);
	MCP2515_setRegisters(mcp2515, ptxbuf->v[IDX_TXBSIDH], data, 5 + frame->can_dlc);
	MCP2515_modifyRegister(mcp2515, ptxbuf->v[IDX_TXBCTRL], TXB_TXREQ, TXB_TXREQ);

	uint8_t ctrl = MCP2515_readRegister(mcp2515, ptxbuf->v[IDX_TXBCTRL]);
	if((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
		return ERROR_FAILTX ;
	}
	return ERROR_OK ;
}

// sendet all drei alle drei Tx-Buffer
enum MCP2515_ERROR MCP2515_sendMessage1(MCP2515_t *mcp2515, const struct can_frame *frame)
{
	if(frame->can_dlc > CAN_MAX_DLEN) { return ERROR_FAILTX; }

	for(int i = 0 ; i < N_TXBUFFERS ; i++) {
		
		TXBn_REGS_t *txbuf = &mcp2515->TXB[i];
		uint8_t ctrlval = MCP2515_readRegister(mcp2515, txbuf->v[IDX_TXBCTRL]);
		if((ctrlval & TXB_TXREQ) == 0) { return MCP2515_sendMessage0(mcp2515, i, frame);}
	}
	return ERROR_ALLTXBUSY ;
}

enum MCP2515_ERROR MCP2515_readMessage0(MCP2515_t *mcp2515, const enum MCP2515_RXBn rxbn, struct can_frame *frame)
{
	RXBn_REGS_t rxb = mcp2515->RXB[rxbn] ;
	uint8_t tbufdata[5] ;

	MCP2515_readRegisters(mcp2515, rxb.v[IDX_RXBSIDH], tbufdata, 5);

	uint32_t id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5) ;

	if((tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK) {
		
		id = (id << 2) + (tbufdata[MCP_SIDL] & 0x03) ;
		id = (id << 8) + tbufdata[MCP_EID8] ;
		id = (id << 8) + tbufdata[MCP_EID0] ;
		id |= CAN_EFF_FLAG ;
	}

	uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK) ;
	if(dlc > CAN_MAX_DLEN) { return ERROR_FAIL;}

	uint8_t ctrl = MCP2515_readRegister(mcp2515, rxb.v[IDX_RXBCTRL]);
		if(ctrl & RXBnCTRL_RTR) {
		id |= CAN_RTR_FLAG ;
	}

	frame->can_id = id ;
	frame->can_dlc = dlc ;

	MCP2515_readRegisters(mcp2515, rxb.v[IDX_RXBDATA], frame->data, dlc);
	MCP2515_modifyRegister(mcp2515, MCP_CANINTF, rxb.v[IDX_RXBIF], 0);

	return ERROR_OK ;
}

enum MCP2515_ERROR MCP2515_readMessage1(MCP2515_t *mcp2515, struct can_frame *frame)
{
	enum MCP2515_ERROR rc ;
	uint8_t stat = MCP2515_getStatus(mcp2515) ;

	if(stat & STAT_RX0IF) {
		rc = MCP2515_readMessage0(mcp2515, RXB0, frame) ;
	} else if(stat & STAT_RX1IF) {
		rc = MCP2515_readMessage0(mcp2515, RXB1, frame) ;
	} else {
		rc = ERROR_NOMSG ;
	}
	return rc ;
}

bool MCP2515_checkReceive(MCP2515_t *mcp2515)
{
	uint8_t res = MCP2515_getStatus(mcp2515) ;
	if(res & STAT_RXIF_MASK)
	{
		return true ;
	} else {
		return false ;
	}
}

bool MCP2515_checkError(MCP2515_t *mcp2515)
{
	uint8_t eflg = MCP2515_getErrorFlags(mcp2515) ;

	if(eflg & EFLG_ERRORMASK) {
		return true ;
	} else {
		return false ;
	}
}

uint8_t MCP2515_getErrorFlags(MCP2515_t *mcp2515)
{
	return MCP2515_readRegister(mcp2515, MCP_EFLG) ;
}

void MCP2515_clearRXnOVRFlags(MCP2515_t *mcp2515)
{
	MCP2515_modifyRegister(mcp2515, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0) ;
}


/////////////////////////////////////////////////////////////////////////////////////////////
void MCP2515_setInterrupts(MCP2515_t *mcp2515, uint8_t intsrc)
{
	MCP2515_setRegister(mcp2515, MCP_CANINTE, intsrc); // Interrupts enable
}

uint8_t MCP2515_getInterrupt(MCP2515_t *mcp2515)
{
	uint8_t ret = MCP2515_readRegister(mcp2515, MCP_CANINTF);
	return ret ;
}

void MCP2515_clrInterrupt(MCP2515_t *mcp2515, uint8_t intsrc)
{
	
	if (intsrc & CANINTF_MERRF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_MERRF, 0);
	if (intsrc & CANINTF_WAKIF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_WAKIF, 0);
	if (intsrc & CANINTF_ERRIF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_ERRIF, 0);
	if (intsrc & CANINTF_TX2IF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_TX2IF, 0);
	if (intsrc & CANINTF_TX1IF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_TX1IF, 0);
	if (intsrc & CANINTF_TX0IF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_TX0IF, 0);
	if (intsrc & CANINTF_RX1IF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_RX1IF, 0);
	if (intsrc & CANINTF_RX0IF) MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_RX0IF, 0);
}
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t MCP2515_getInterrupts(MCP2515_t *mcp2515)
{
	return MCP2515_readRegister(mcp2515, MCP_CANINTF) ;
}

void MCP2515_clearInterrupts(MCP2515_t *mcp2515)
{
	MCP2515_setRegister(mcp2515, MCP_CANINTF, 0) ;
}

uint8_t MCP2515_getInterruptMask(MCP2515_t *mcp2515)
{
	return MCP2515_readRegister(mcp2515, MCP_CANINTE) ;
}

void MCP2515_clearTXInterrupts(MCP2515_t *mcp2515)
{
	MCP2515_modifyRegister(mcp2515, MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0) ;
}

void MCP2515_clearRXnOVR(MCP2515_t *mcp2515)
{
	uint8_t eflg = MCP2515_getErrorFlags(mcp2515) ;
	if(eflg != 0) {
	MCP2515_clearRXnOVRFlags(mcp2515) ;
	MCP2515_clearInterrupts(mcp2515) ;
//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}
	
}

void MCP2515_clearMERR(MCP2515_t *mcp2515)
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_MERRF, 0) ;
}

void MCP2515_clearERRIF(MCP2515_t *mcp2515)
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	MCP2515_modifyRegister(mcp2515, MCP_CANINTF, CANINTF_ERRIF, 0) ;
}

uint8_t MCP2515_errorCountRX(MCP2515_t *mcp2515)                             
{
	return MCP2515_readRegister(mcp2515, MCP_REC) ;
}

uint8_t MCP2515_errorCountTX(MCP2515_t *mcp2515)                             
{
	return MCP2515_readRegister(mcp2515, MCP_TEC) ;
}

//#########################################################################################################################
