/*****************************************************************************/
/**
* @file mcp2515.h
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

#ifndef _MCP2515_H
#define _MCP2515_H

#include "can.h"

#include "hardware/spi.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "boards/pico.h"
#include "../xspics.h"
/*
 *  speed 16M
 */
#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_83k3BPS_CFG1 (0x03)
#define MCP_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP_16MHz_83k3BPS_CFG3 (0x07)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_33k3BPS_CFG1 (0x4E)
#define MCP_16MHz_33k3BPS_CFG2 (0xF1)
#define MCP_16MHz_33k3BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)


// ********** public-Defs *************
#define CANINTF_RX0IF (0x01)
#define CANINTF_RX1IF (0x02)
#define CANINTF_TX0IF (0x04)
#define CANINTF_TX1IF (0x08)
#define CANINTF_TX2IF (0x10)
#define CANINTF_ERRIF (0x20)
#define CANINTF_WAKIF (0x40)
#define CANINTF_MERRF (0x80)

#define	EFLG_RX1OVR (1 << 7)
#define	EFLG_RX0OVR (1 << 6)
#define	EFLG_TXBO   (1 << 5)
#define	EFLG_TXEP   (1 << 4)
#define	EFLG_RXEP   (1 << 3)
#define	EFLG_TXWAR  (1 << 2)
#define	EFLG_RXWAR  (1 << 1)
#define	EFLG_EWARN  (1 << 0)

enum MCP2515_CAN_CLOCK {
	MCP_20MHZ,
	MCP_16MHZ,
	MCP_8MHZ
};

enum MCP2515_CAN_SPEED {
	CAN_5KBPS,
	CAN_10KBPS,
	CAN_20KBPS,
	CAN_31K25BPS,
	CAN_33KBPS,
	CAN_40KBPS,
	CAN_50KBPS,
	CAN_80KBPS,
	CAN_83K3BPS,
	CAN_95KBPS,
	CAN_100KBPS,
	CAN_125KBPS,
	CAN_200KBPS,
	CAN_250KBPS,
	CAN_500KBPS,
	CAN_1000KBPS
};

enum MCP2515_CAN_CLKOUT {
	CLKOUT_DISABLE = -1,
	CLKOUT_DIV1    = 0x0,
	CLKOUT_DIV2    = 0x1,
	CLKOUT_DIV4    = 0x2,
	CLKOUT_DIV8    = 0x3,
};


enum MCP2515_ERROR {
	ERROR_OK        = 0,
	ERROR_FAIL      = 1,
	ERROR_ALLTXBUSY = 2,
	ERROR_FAILINIT  = 3,
	ERROR_FAILTX    = 4,
	ERROR_NOMSG     = 5
};

enum MCP2515_MASK {
	MASK0,
	MASK1
};

enum MCP2515_RXF {
	RXF0 = 0,
	RXF1 = 1,
	RXF2 = 2,
	RXF3 = 3,
	RXF4 = 4,
	RXF5 = 5
};

enum MCP2515_RXBn {
	RXB0 = 0,
	RXB1 = 1
};

enum MCP2515_TXBn {
	TXB0 = 0,
	TXB1 = 1,
	TXB2 = 2
};

enum MCP2515_IDXTX {
	IDX_TXBCTRL = 0,
	IDX_TXBSIDH = 1,
	IDX_TXBDATA = 2	
};

enum MCP2515_IDXRX {
	IDX_RXBCTRL = 0,
	IDX_RXBSIDH = 1,
	IDX_RXBDATA = 2,
	IDX_RXBIF = 3
};


// ********** public-Defs ************* - end

// ********** private-Defs *************
static const uint8_t CANCTRL_REQOP = 0xE0;
static const uint8_t CANCTRL_ABAT = 0x10;
static const uint8_t CANCTRL_OSM = 0x08;
static const uint8_t CANCTRL_CLKEN = 0x04;
static const uint8_t CANCTRL_CLKPRE = 0x03;

#define	CANCTRL_REQOP_NORMAL     0x00
#define	CANCTRL_REQOP_SLEEP      0x20
#define	CANCTRL_REQOP_LOOPBACK   0x40
#define	CANCTRL_REQOP_LISTENONLY 0x60
#define	CANCTRL_REQOP_CONFIG     0x80
#define	CANCTRL_REQOP_POWERUP    0xE0

static const uint8_t CANSTAT_OPMOD = 0xE0;
static const uint8_t CANSTAT_ICOD = 0x0E;

static const uint8_t CNF3_SOF = 0x80;

static const uint8_t TXB_EXIDE_MASK = 0x08;
static const uint8_t DLC_MASK       = 0x0F;
static const uint8_t RTR_MASK       = 0x40;

static const uint8_t RXBnCTRL_RXM_STD    = 0x20;
static const uint8_t RXBnCTRL_RXM_EXT    = 0x40;
static const uint8_t RXBnCTRL_RXM_STDEXT = 0x00;
static const uint8_t RXBnCTRL_RXM_MASK   = 0x60;
static const uint8_t RXBnCTRL_RTR        = 0x08;
static const uint8_t RXB0CTRL_BUKT       = 0x04;
static const uint8_t RXB0CTRL_FILHIT_MASK = 0x03;
static const uint8_t RXB1CTRL_FILHIT_MASK = 0x07;
static const uint8_t RXB0CTRL_FILHIT = 0x00;
static const uint8_t RXB1CTRL_FILHIT = 0x01;

static const uint8_t MCP_SIDH = 0;
static const uint8_t MCP_SIDL = 1;
static const uint8_t MCP_EID8 = 2;
static const uint8_t MCP_EID0 = 3;
static const uint8_t MCP_DLC  = 4;
static const uint8_t MCP_DATA = 5;

#define	STAT_RX0IF (1 << 0)
#define	STAT_RX1IF (1 << 1)

static const uint8_t STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF;

//enum /*class*/ TXBnCTRL : uint8_t {
#define	TXB_ABTF  0x40
#define	TXB_MLOA  0x20
#define	TXB_TXERR 0x10
#define	TXB_TXREQ 0x08
#define	TXB_TXIE  0x04
#define	TXB_TXP   0x03

static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR
                                    | EFLG_RX0OVR
                                    | EFLG_TXBO
                                    | EFLG_TXEP
                                    | EFLG_RXEP;
					
#define	MCP_INSTR_WRITE       0x02
#define	MCP_INSTR_READ        0x03
#define	MCP_INSTR_BITMOD      0x05
#define	MCP_INSTR_LOAD_TX0    0x40
#define	MCP_INSTR_LOAD_TX1    0x42
#define	MCP_INSTR_LOAD_TX2    0x44
#define	MCP_INSTR_RTS_TX0     0x81
#define	MCP_INSTR_RTS_TX1     0x82
#define	MCP_INSTR_RTS_TX2     0x84
#define	MCP_INSTR_RTS_ALL     0x87
#define	MCP_INSTR_READ_RX0    0x90
#define	MCP_INSTR_READ_RX1    0x94
#define	MCP_INSTR_READ_STATUS 0xA0
#define	MCP_INSTR_RX_STATUS   0xB0
#define	MCP_INSTR_RESET       0xC0

#define	MCP_RXF0SIDH 0x00
#define	MCP_RXF0SIDL 0x01
#define	MCP_RXF0EID8 0x02
#define	MCP_RXF0EID0 0x03
#define	MCP_RXF1SIDH 0x04
#define	MCP_RXF1SIDL 0x05
#define	MCP_RXF1EID8 0x06
#define	MCP_RXF1EID0 0x07
#define	MCP_RXF2SIDH 0x08
#define	MCP_RXF2SIDL 0x09
#define	MCP_RXF2EID8 0x0A
#define	MCP_RXF2EID0 0x0B
#define	MCP_CANSTAT  0x0E
#define	MCP_CANCTRL  0x0F
#define	MCP_RXF3SIDH 0x10
#define	MCP_RXF3SIDL 0x11
#define	MCP_RXF3EID8 0x12
#define	MCP_RXF3EID0 0x13
#define	MCP_RXF4SIDH 0x14
#define	MCP_RXF4SIDL 0x15
#define	MCP_RXF4EID8 0x16
#define	MCP_RXF4EID0 0x17
#define	MCP_RXF5SIDH 0x18
#define	MCP_RXF5SIDL 0x19
#define	MCP_RXF5EID8 0x1A
#define	MCP_RXF5EID0 0x1B
#define	MCP_TEC      0x1C
#define	MCP_REC      0x1D
#define	MCP_RXM0SIDH 0x20
#define	MCP_RXM0SIDL 0x21
#define	MCP_RXM0EID8 0x22
#define	MCP_RXM0EID0 0x23
#define	MCP_RXM1SIDH 0x24
#define	MCP_RXM1SIDL 0x25
#define	MCP_RXM1EID8 0x26
#define	MCP_RXM1EID0 0x27
#define	MCP_CNF3     0x28
#define	MCP_CNF2     0x29
#define	MCP_CNF1     0x2A
#define	MCP_CANINTE  0x2B
#define	MCP_CANINTF  0x2C
#define	MCP_EFLG     0x2D
#define	MCP_TXB0CTRL 0x30
#define	MCP_TXB0SIDH 0x31
#define	MCP_TXB0SIDL 0x32
#define	MCP_TXB0EID8 0x33
#define	MCP_TXB0EID0 0x34
#define	MCP_TXB0DLC  0x35
#define	MCP_TXB0DATA 0x36
#define	MCP_TXB1CTRL 0x40
#define	MCP_TXB1SIDH 0x41
#define	MCP_TXB1SIDL 0x42
#define	MCP_TXB1EID8 0x43
#define	MCP_TXB1EID0 0x44
#define	MCP_TXB1DLC  0x45
#define	MCP_TXB1DATA 0x46
#define	MCP_TXB2CTRL 0x50
#define	MCP_TXB2SIDH 0x51
#define	MCP_TXB2SIDL 0x52
#define	MCP_TXB2EID8 0x53
#define	MCP_TXB2EID0 0x54
#define	MCP_TXB2DLC  0x55
#define	MCP_TXB2DATA 0x56
#define	MCP_RXB0CTRL 0x60
#define	MCP_RXB0SIDH 0x61
#define	MCP_RXB0SIDL 0x62
#define	MCP_RXB0EID8 0x63
#define	MCP_RXB0EID0 0x64
#define	MCP_RXB0DLC  0x65
#define	MCP_RXB0DATA 0x66
#define	MCP_RXB1CTRL 0x70
#define	MCP_RXB1SIDH 0x71
#define	MCP_RXB1SIDL 0x72
#define	MCP_RXB1EID8 0x73
#define	MCP_RXB1EID0 0x74
#define	MCP_RXB1DLC  0x75
#define	MCP_RXB1DATA 0x76

static const uint32_t DEFAULT_SPI_CLOCK = 2000000; // 2MHz

#define N_TXBUFFERS 3
#define N_RXBUFFERS 2
	
typedef struct TXBn_REGS {
	uint8_t v[3]; // [0]=CTRL, [1]=SIDH, [2]=DATA
} TXBn_REGS_t;
	
typedef struct RXBn_REGS {
	uint8_t v[4]; // [0]=CTRL, [1]=SIDH, [2]=DATA, [3]=RXnIF
} RXBn_REGS_t;
	
typedef struct
{
	spi_inst_t *spi;
	enum spi_csn cs_pin;

	enum MCP2515_ERROR rctx;
	enum MCP2515_ERROR rcrx;
	uint8_t intsrc;
	const struct TXBn_REGS TXBn;
	const struct RXBn_REGS RXBn;

	TXBn_REGS_t TXB[N_TXBUFFERS];
	RXBn_REGS_t RXB[N_RXBUFFERS];
	
} MCP2515_t;	

// ********** private-Defs ************* - end
	
enum MCP2515_ERROR MCP2515_setMode(MCP2515_t *mcp2515, const uint8_t mode);
uint8_t MCP2515_readRegister(MCP2515_t *mcp2515, const uint8_t reg);
void MCP2515_readRegisters(MCP2515_t *mcp2515, const uint8_t reg, uint8_t values[], const uint8_t n);
void MCP2515_setRegister(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t value);
void MCP2515_setRegisters(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t values[], const uint8_t n);
void MCP2515_modifyRegister(MCP2515_t *mcp2515, const uint8_t reg, const uint8_t mask, const uint8_t data);
void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id);

enum MCP2515_ERROR MCP2515_init(MCP2515_t *mcp2515, spi_inst_t *spi, enum spi_csn cs_pin);
enum MCP2515_ERROR MCP2515_reset(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setConfigMode(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setListenOnlyMode(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setSleepMode(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setLoopbackMode(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setNormalMode(MCP2515_t *mcp2515);
enum MCP2515_ERROR MCP2515_setClkOut(MCP2515_t *mcp2515, const enum MCP2515_CAN_CLKOUT divisor);
enum MCP2515_ERROR MCP2515_setBitrate(MCP2515_t *mcp2515, const enum MCP2515_CAN_SPEED canSpeed);		
enum MCP2515_ERROR MCP2515_setFilterMask(MCP2515_t *mcp2515,const enum MCP2515_MASK num, const bool ext, const uint32_t ulData);
enum MCP2515_ERROR MCP2515_setFilter(MCP2515_t *mcp2515,const enum MCP2515_RXF num, const bool ext, const uint32_t ulData);		
enum MCP2515_ERROR MCP2515_sendMessage0(MCP2515_t *mcp2515, const enum MCP2515_TXBn txbn, const struct can_frame *frame);
enum MCP2515_ERROR MCP2515_sendMessage1(MCP2515_t *mcp2515, const struct can_frame *frame);
enum MCP2515_ERROR MCP2515_readMessage0(MCP2515_t *mcp2515, const enum MCP2515_RXBn rxbn, struct can_frame *frame);
enum MCP2515_ERROR MCP2515_readMessage1(MCP2515_t *mcp2515, struct can_frame *frame);

bool MCP2515_checkReceive(MCP2515_t *mcp2515);
bool MCP2515_checkError(MCP2515_t *mcp2515);
uint8_t MCP2515_getErrorFlags(MCP2515_t *mcp2515);
void MCP2515_clearRXnOVRFlags(MCP2515_t *mcp2515);
uint8_t MCP2515_getInterrupts(MCP2515_t *mcp2515);
uint8_t MCP2515_getInterruptMask(MCP2515_t *mcp2515);
void MCP2515_clearInterrupts(MCP2515_t *mcp2515);
void MCP2515_clearTXInterrupts(MCP2515_t *mcp2515);
uint8_t MCP2515_getStatus(MCP2515_t *mcp2515);
void MCP2515_clearRXnOVR(MCP2515_t *mcp2515);
void MCP2515_clearMERR();
void MCP2515_clearERRIF();
uint8_t MCP2515_errorCountRX(MCP2515_t *mcp2515);
uint8_t MCP2515_errorCountTX(MCP2515_t *mcp2515);	

void MCP2515_setInterrupts(MCP2515_t *mcp2515, uint8_t intsrc);
uint8_t MCP2515_getInterrupt(MCP2515_t *mcp2515);
void MCP2515_clrInterrupt(MCP2515_t *mcp2515, uint8_t intsrc);
		
#endif
