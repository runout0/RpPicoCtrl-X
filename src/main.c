/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Cloffe
 */

#include "main.h"

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDK_VERSION {PICO_SDK_VERSION_STRING} //from "..\build\VisualGDB\Debug\generated\pico_base\pico\version.h"
#define PROJECT_VERSION {PROJECT_VERSION_STRING} //from "CMakeLists.txt -> add_definitions( -DPROJECT_VERSION_STRING="${CMAKE_PROJECT_VERSION}" )	

#define BAUDRATE_SPI0 2000000
#define BAUDRATE_SPI1 2000000

#define JOY2SCALER 10
#define OS_PTX_SCAN 50 // OS_PTX_SCAN * 10ms

static const uint8_t OLEDADDR = 0x3c; 

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

//OsUVFLO set by /XGPIO_PWROK (below 18V), reset bei PowerCycle (20ms after PowerOK)
bool OsUVFLO;

bool 	OS_PtX_Scan; 

uint8_t rd_ctrl;
uint8_t rd_stat;

enum MCP2515_ERROR rc_reset;

bool I2Cbusy;
bool I2CScanIO;

bool boledInitOK;
bool brtcInitOK;

uint8_t rxbuf[2];
oledtxln_t OledTx;
uint32_t Global1ms;

ttUartCmd_t ttUartCmd;
rv3028_error_t ErrorCode;
rv3028_t RTC;
SSOLED OLED;
xgpio_t XGPIO;
xeeprom_t EEPROM;
procvar_t ProcVar;
sram_t SRAM;
ee_t EE;

ioexp_t IOEXP_DI_IO;
ioexp_t IOEXP_RE_IO;
ioexp_t IOEXP_DIO_IO;
xplc_t XPLC;
Command_t Command;
Max31865_t  pt100_1;
Max31865_t  pt100_2;
MCP2515_t MCP2515;
struct can_frame Txmsg;
struct can_frame Rxmsg;
struct tm SetTime;

bool pt1start;
bool pt2start;
uint8_t pttoggle;
uint8_t cnt_joy2Scaler = JOY2SCALER;

uint8_t cancnt = 0;

xadc_t XADC;
xpwm_t XPWM;
joy2_t JOY2;

bool repeating_timer1ms_callback(struct repeating_timer *t);
bool repeating_timer10ms_callback(struct repeating_timer *t);

/* Private function prototypes -----------------------------------------------*/
void HandleContReception(void);

rv3028_error_t RV3028_Write(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length);
rv3028_error_t RV3028_Interface(rv3028_t* p_Device);
rv3028_error_t RV3028_Read(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length);
rv3028_error_t RV3028_DT_Get(rv3028_t* p_Device);
rv3028_error_t RV3028_DT_Set(rv3028_t* p_Device);

eeprom_error_t tee;
eeprom_error_t ree;

xgpio_error_t XGPIO_Interface(xgpio_t *p_xgpio);
xgpio_error_t CbRTCClk(uint32_t events);
xgpio_error_t CbRTCInt(uint32_t events);
xgpio_error_t CbCANInt(uint32_t events);
xgpio_error_t CbPwrOKInt(uint32_t events);

void SRAM_Write(sram_t *sram, procvar_t *procvar);
void SRAM_Read(sram_t *sram, procvar_t *procvar);	
void TSL2591_ReadID(uint8_t iaddr, uint8_t _register);


void RunPLC(void);
void ForceOutput(Command_t *cmd);
void TestJoy2(Command_t *cmd);

int main() {

	char tmpTx[200];
	int rc;
	struct repeating_timer timer1ms;
	struct repeating_timer timer10ms;
	ioexp_error_t rcioexp;
	enum MCP2515_ERROR mcperr;

	Txmsg.can_id  = 0x0F6;
	Txmsg.can_dlc = 8;
	Txmsg.data[0] = 0x00;
	Txmsg.data[1] = 0x87;
	Txmsg.data[2] = 0x32;
	Txmsg.data[3] = 0xFA;
	Txmsg.data[4] = 0x26;
	Txmsg.data[5] = 0x8E;
	Txmsg.data[6] = 0xBE;
	Txmsg.data[7] = 0x86;
	
	fw_version_t fw;
	strcpy(fw.strName, "Debug-Output Pico RP2040 (C) 2025"); // max. Lenght 60
	strcpy(fw.strProject, "UART0 CTRL-X RP2040"); // max. Lenght 20 
	char prjstr[] = PROJECT_VERSION; // e.g. "1.3.0"
	if (strlen(prjstr) == 5)
	{
		fw.major = atoi(&prjstr[0]);
		fw.minor = atoi(&prjstr[2]);
		fw.revision = atoi(&prjstr[4]);
	}

	/* Configure USARTx (USART IP configuration and related GPIO initialization) */
	ttUartCmd.strCode[USART_CMD0] = 'h';
	ttUartCmd.strCode[USART_CMD1] = 'c';
	ttUartCmd.strCode[USART_CMD2] = 's';
	ttUartCmd.strCode[USART_CMD3] = 'j';
	ttUartCmd.strCode[USART_CMD18] = 'x';

	XGPIO_Interface(&XGPIO);	
	XGPIO_Init(&XGPIO);
	
	// 10ms-Timer for DI/DI-Transfer
	// Negative delay so means we will call repeating_timer_callback, and call it again
	// 500ms later regardless of how long the callback took to execute
	add_repeating_timer_ms(-1, repeating_timer1ms_callback, NULL, &timer1ms);
	add_repeating_timer_ms(-10, repeating_timer10ms_callback, NULL, &timer10ms);
	
	ttUART_Configure();
	ttUART_StartReception(&fw);
		
	if (i2c0_ResetBus(PICO_I2C1SET3_SDA_PIN, PICO_I2C1SET3_SCL_PIN))
	{
		strcpy(tmpTx, "I2C0-Reset sucessful\r\n");
		uart_puts(UART_ID, tmpTx); 		
		i2c0_Init(PICO_I2C0SET2_SDA_PIN, PICO_I2C0SET2_SCL_PIN, 220 * 1000, true); //GPIO8/9 05.01.2025 "220" results in 200kHz, "465" results in 400kHz
	}
	else
	{
		strcpy(tmpTx, "I2C0-blocked\r\n");
		uart_puts(UART_ID, tmpTx); 
	}
	
	if (i2c1_ResetBus(PICO_I2C1SET3_SDA_PIN, PICO_I2C1SET3_SCL_PIN))
	{
		strcpy(tmpTx, "I2C1-Reset sucessful\r\n");
		uart_puts(UART_ID, tmpTx); 		
		i2c1_Init(PICO_I2C1SET3_SDA_PIN, PICO_I2C1SET3_SCL_PIN, 465 * 1000, true); //GPIO14/15
	}
	else
	{
		strcpy(tmpTx, "I2C1-blocked\r\n");
		uart_puts(UART_ID, tmpTx); 
	}

	EE_Init(&EEPROM, i2c1, EEPROML_ADDRESS);
	
	// I/O-Expander 8x DI, IO
	IOEXP_DI_IO.i2c = i2c1;
	IOEXP_DI_IO.DeviceAddr = IOEXP_DI_IO_ADDRESS;
	rc = IoExp_Init(&IOEXP_DI_IO);

	// I/O-Expander 4x Relays, Diag-Leds, IO, IO
	IOEXP_RE_IO.i2c = i2c1;
	IOEXP_RE_IO.DeviceAddr = IOEXP_RE_IO_ADDRESS;
	rc = IoExp_Init(&IOEXP_RE_IO);
	
	// I/O-Expander 4x HS-Switch, Buttons, Buzzer, Status IO
	IOEXP_DIO_IO.i2c = i2c1;
	IOEXP_DIO_IO.DeviceAddr = IOEXP_DIO_IO_ADDRESS;
	rc = IoExp_Init(&IOEXP_DIO_IO);

	sleep_ms(100);
	OsUVFLO = !gpio_get(XGPIO_PWROK);
	if (OsUVFLO)
	{
		strcpy(tmpTx, "Undervoltage detected at startup\r\n");
		uart_puts(UART_ID, tmpTx);
	}

	// Pre-Init MCP1525 -> setting SPI-IF (spi0) used in "spi0_Init"
	mcperr = MCP2515_init(&MCP2515, spi0, csn_can);
	
	memset(&Command, 0x0, sizeof(Command_t));

	// Init-SPI0/1	SPI-Mode 3
	CSn_Init();

	spi0_Init(BAUDRATE_SPI0, SPI_CPOL_1, SPI_CPHA_1, &MCP2515); // CAN-IF, extern	
	spi1_Init(BAUDRATE_SPI1, SPI_CPOL_1, SPI_CPHA_1); // SRAM, Pt100-1, PT100-2

	// Init-ADC
	xadc_Init(&XADC);

	ree	= EE_ReadSequential(&EEPROM, 0, &EE.content[0], sizeof(EE.content));
	if (ree ==  EEPROM_NO_ERROR)
	{
		sprintf(tmpTx, "EEPROM-Initial-Read content[0] %u content[9] %u\r\n", EE.content[0], EE.content[9]);
		uart_puts(UART_ID, tmpTx); 						
	}
	else
	{
		sprintf(tmpTx, "EE-RdSeq rc= %u\r\n", ree);
		uart_puts(UART_ID, tmpTx); 								
	}

	joy2_Init(&JOY2, i2c0, JOYSTICK2_ADDR);
	
	rc = oledInit(&OLED, OLED_128x64, OLEDADDR, 1, 0);
	if (rc != OLED_NOT_FOUND)
	{ 
		sleep_ms(100);
		oledFill(&OLED, 0, 1);
		oledSetContrast(&OLED, 127);

		/*Startup Line 0  - *******/
		oledWriteString(&OLED, 0, 0, OLED_LINE0, (char *)"****************", FONT_8x8, 0, 1);
		
		/*Startup Line 1  - Version Firmware*/
		char fwstr[] = PROJECT_VERSION;
		memset(tmpTx, 0x0, sizeof(tmpTx));					
		sprintf(tmpTx, "FW:  %s      ", fwstr);		
		oledWriteString(&OLED, 0, 0, OLED_LINE1, tmpTx, FONT_8x8, 0, 1);

		/*Startup Line 2  - Version SDK*/
		char sdkstr[] = SDK_VERSION;
		memset(tmpTx, 0x0, sizeof(tmpTx));					
		sprintf(tmpTx, "SDK: %s", sdkstr);
		oledWriteString(&OLED, 0, 0, OLED_LINE2, tmpTx, FONT_8x8, 0, 1);
		oledWriteString(&OLED, 0, 0, OLED_LINE6, (char *)"T. Timme 2024", FONT_8x8, 0, 1);
		oledWriteString(&OLED, 0, 0, OLED_LINE7, (char *)"****************", FONT_8x8, 0, 1);

		sleep_ms(2000);
		oledFill(&OLED, 0, 1);
		boledInitOK = true;
	}

	if (boledInitOK)
	{
		if (RV3028_Interface(&RTC) == RV3028_NO_ERROR)
		{

			RV3028_DisableWP(&RTC, PASSWORD);

			//NRF_LOG_INFO("RTC initialized...");
			strcpy(tmpTx, "RTC Interface initialized...\r\n");
			uart_puts(UART_ID, tmpTx); 					
		
			ErrorCode = RV3028_Init(&RTC_Init, &RTC);
			if (ErrorCode == RV3028_NO_ERROR)
			{	
				sprintf(tmpTx, "RTC initialized... HID: %u, VID: %u\r\n", RTC.HID, RTC.VID);
				uart_puts(UART_ID, tmpTx); 					
				sleep_ms(1000);
			
				strcpy(tmpTx, "Unlocking RTC-device...\r\n");			
				RV3028_UnlockWP(&RTC, PASSWORD);
				RV3028_EnableClkOut(&RTC, true, false);
				brtcInitOK = true;			
			}
			else
			{
				brtcInitOK = false;			
				sprintf(tmpTx, "Can not initialize RTC. Error: %u\r\n", ErrorCode);
			}
			uart_puts(UART_ID, tmpTx); 						
		}
	}

	/*********************** Read SRAM *********************/	
	// Prozess-Status  initial off SRAM lesen
	SRAM_Read(&SRAM, &ProcVar);
	sprintf(tmpTx, "SRAM-Initial-Read content[0] %u content[9] %u\r\n", ProcVar.ram_content[0], ProcVar.ram_content[9]);
	uart_puts(UART_ID, tmpTx);	
	
	/***************** Init  pt100_1 ***********************/
	Max31865_init(&pt100_1, spi1, csn_pt1, 3, 50); 
	Max31865_init(&pt100_2, spi1, csn_pt2, 3, 50); 

	/***************** Init  CAN-Bus ***********************/
	mcperr = MCP2515_reset(&MCP2515);
	sprintf(tmpTx, "CAN Reset rc= %02u\r\n", mcperr);
	uart_puts(UART_ID, tmpTx);
	mcperr = MCP2515_setBitrate(&MCP2515, CAN_500KBPS);
	sprintf(tmpTx, "CAN setBitrate rc= %02u\r\n", mcperr);
	uart_puts(UART_ID, tmpTx);
	mcperr = MCP2515_setNormalMode(&MCP2515);
	sprintf(tmpTx, "CAN setNormalMode rc= %02u\r\n", mcperr);
	uart_puts(UART_ID, tmpTx);
	MCP2515_setInterrupts(&MCP2515, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);
	
	// Init-Servo-PWM to 20ms/Div 125
	XPWM_Init(&XPWM, XGPIO_PWM);
	
	// Enable 24V
	gpio_put(XGPIO_EN24V, 1);

	PLCsetup(&XPLC, Global1ms);	
	//main-Loop
	while (true) {
		
		//no cyclic action after undervoltage-event
		if (OsUVFLO) {
			
			while (true) ;			
		}
		
		ttUART_ContReception();
		HandleContReception();	

		// every 1s (triggered from RTC-1Hz-Interrrupt)
		if (brtcInitOK && boledInitOK && ProcVar.bupdateOled)
		{

			//Pt100-Filterung Effectivity 10% = 0,1, after 10s@1s Intervall Filterung complete
			pt100_1.tempC_filtered = Max31865_Filter(pt100_1.tempC, pt100_1.tempC_filtered, 0.1);
			pt100_2.tempC_filtered = Max31865_Filter(pt100_2.tempC, pt100_2.tempC_filtered, 0.1);

			//Pt100 Out-of-Range (disconnected, broken wire)
			bool oor = ((int)pt100_1.tempC_filtered < -99) ||  (int)pt100_2.tempC_filtered < -99;
			
			//Status DI->Display
			memset(tmpTx, 0x0, sizeof(tmpTx));
			sprintf(tmpTx, "DI:%u%u%u%u%u%u%u%u %u %u ", IOEXP_DI_IO.DI[0], IOEXP_DI_IO.DI[1], IOEXP_DI_IO.DI[2], IOEXP_DI_IO.DI[3], IOEXP_DI_IO.DI[4], IOEXP_DI_IO.DI[5], IOEXP_DI_IO.DI[6], IOEXP_DI_IO.DI[7], IOEXP_DIO_IO.DI[7], IOEXP_DIO_IO.DI[6]);
			memcpy(&OledTx.txrstr[OLED_LINE0][0], tmpTx, CHAR_PER_LINE);
			OledTx.txrun[OLED_LINE0] = true;
			
			// Pt100-Input, filtered
			memset(tmpTx, 0x0, sizeof(tmpTx));
					
			if (oor)
			{
				strcpy(tmpTx, "PtX **.** **.** ");
				memcpy(&OledTx.txrstr[OLED_LINE1][0], tmpTx, CHAR_PER_LINE);
			}
			else
			{
				sprintf(tmpTx, "Pt1 %2.1f 2: %2.1f", pt100_1.tempC_filtered, pt100_2.tempC_filtered);
				memcpy(&OledTx.txrstr[OLED_LINE1][0], tmpTx, CHAR_PER_LINE);
			}
			OledTx.txrun[OLED_LINE1] = true;
						
			// ADC-Channel 0..10V -> 0..100
			memset(tmpTx, 0x0, sizeof(tmpTx));
			sprintf(tmpTx, "ADC %03u %03u %03u", (uint8_t)(XADC.vfloat[ADC0] * 10.0f), (uint8_t)(XADC.vfloat[ADC1] * 10.0f), (uint8_t)(XADC.vfloat[ADC2] * 10.0f));
			memcpy(&OledTx.txrstr[OLED_LINE2][0], tmpTx, CHAR_PER_LINE);
			OledTx.txrun[OLED_LINE2] = true;

			// SRAM-Content
			memset(tmpTx, 0x0, sizeof(tmpTx));
			sprintf(tmpTx, "RAM %03u %03u", ProcVar.ram_content[0], ProcVar.ram_content[9]);
			memcpy(&OledTx.txrstr[OLED_LINE3][0], tmpTx, CHAR_PER_LINE);
			OledTx.txrun[OLED_LINE3] = true;
			
			//Status RTC->Display
			RV3028_GetTime(&RTC, &CurrentTime);
			memset(tmpTx, 0x0, sizeof(tmpTx));
			sprintf(tmpTx, "Zeit: %02u:%02u:%02u  ", CurrentTime.tm_hour, CurrentTime.tm_min, CurrentTime.tm_sec);
			memcpy(&OledTx.txrstr[OLED_LINE6][0], tmpTx, CHAR_PER_LINE);
			OledTx.txrun[OLED_LINE6] = true;
	
			ProcVar.bupdateOled	= false;
		}	
		
		// 10ms DI/DO-Scan
		if (I2CScanIO)
		{
			Max31865_Cyclic(&pt100_1);
			Max31865_Cyclic(&pt100_2);
			
			rcioexp = IoExp_ReadDI(&IOEXP_DI_IO);			
			if (rcioexp != IOEXP_NO_ERROR)
			{
				sprintf(tmpTx, "IoExp_ReadIO Error IOEXP_DI_IO %u\r\n", rcioexp);
				uart_puts(UART_ID, tmpTx);
			}
			
			rcioexp = IoExp_ReadDIO(&IOEXP_DIO_IO);
			if (rcioexp != IOEXP_NO_ERROR)
			{
				sprintf(tmpTx, "IoExp_ReadDIO Error IOEXP_DIO_IO %u\r\n", rcioexp);
				uart_puts(UART_ID, tmpTx);
			}

			joy2_error_t rcjoy2 = joy2_get_adc_16bits_value_xy(&JOY2);
			if (rcjoy2 == JOY2_NO_ERROR)
			{
				uint16_t tmppwm = (JOY2.yADC16 / 30) + 300;
				XPWM_SetVal(&XPWM, XPWM_MIN, XPWM_MAX, tmppwm);
			}
			else
			{
				sprintf(tmpTx, "joy2 failed rc= %02u\r\n", rcjoy2);
				uart_puts(UART_ID, tmpTx);
			}
			
//			// every 100ms - default
//			if (cnt_joy2Scaler)
//			{
//				cnt_joy2Scaler--;
//				if (!cnt_joy2Scaler)
//				{
//					cnt_joy2Scaler = JOY2SCALER;
//					//joy2_error_t rcjoy2 = joy2_get_adc_8bits_value_xy(&JOY2);
//					joy2_error_t rcjoy2 = joy2_get_adc_16bits_value_xy(&JOY2);
//					if (rcjoy2 == JOY2_NO_ERROR)
//					{
//						uint16_t tmppwm = (JOY2.yADC16 / 30) + 300;
//						XPWM_SetVal(&XPWM, XPWM_MIN, XPWM_MAX, tmppwm);
//					}
//					else
//					{
//						sprintf(tmpTx, "joy2 failed rc= %02u\r\n", rcjoy2);
//						uart_puts(UART_ID, tmpTx);
//					}
//				}
//			}
			
			// Beremiz-Hook 10ms
			RunPLC();

			rcioexp = IoExp_WriteDIO(&IOEXP_DIO_IO);
			if (rcioexp != IOEXP_NO_ERROR)
			{
				sprintf(tmpTx, "IoExp_WriteDIO Error IOEXP_DIO_IO %u\r\n", rcioexp);
				uart_puts(UART_ID, tmpTx);
			}
				
			rcioexp = IoExp_WriteRE(&IOEXP_RE_IO);			
			if (rcioexp != IOEXP_NO_ERROR)
			{
				sprintf(tmpTx, "IoExp_WriteRE Error IOEXP_RE_IO %u\r\n", rcioexp);
				uart_puts(UART_ID, tmpTx);
			}
			
			for (uint8_t i = 0; i < 8; i++)
			{
				if (OledTx.txrun[i])
				{
					OledTx.txrun[i] = false;
					memcpy(tmpTx, &OledTx.txrstr[i][0], CHAR_PER_LINE);
					oledWriteString(&OLED, 0, 0, i, tmpTx, FONT_8x8, 0, 1);
					break;
				}
			}

			if (Command.rtc_dt_rd)
			{
				Command.rtc_dt_rd = false;
				RV3028_DT_Get(&RTC);
			}
			
			if (Command.rtc_dt_wr)
			{
				Command.rtc_dt_wr = false;
				RV3028_DT_Set(&RTC);
			}
			
			// Read ADC-Channels 0..2
			xadc_Scheduler();
			
			I2Cbusy = false;
			I2CScanIO = false;
		}

		// Testfunction I2C-Scan
		if (!I2Cbusy && !I2CScanIO && Command.tst_i2c)
		{
			if (Command.tst_i2c == 1)
			{
			
				I2C_Scan(i2c0); // extern-iso
			}
			else if (Command.tst_i2c == 2)
			{
				I2C_Scan(i2c1); // intern
			}		
			Command.tst_i2c = 0;
		}

		// Testfunction Read-ID TSL2591
		if (!I2Cbusy && !I2CScanIO && Command.scan_sensor0)
		{
			Command.scan_sensor0 = false;
			TSL2591_ReadID(0x29, 0xB2);
		}

		TestJoy2(&Command);
				
		if (OS_PtX_Scan && !Command.tst_cs)
		{
			OS_PtX_Scan = false;
			pttoggle = ~pttoggle;			
			if (pttoggle) pt1start = true;
			else pt2start = true;			
		}

		if (!Command.tst_cs)
		{
			// Read Temerature Pt100_1, start synchronized with 10ms-Task
			Max31865_readTempC(&pt100_1, pt1start);		
			if (pt100_1.state == st_run0) pt1start = false;
		
			// Read Temerature Pt100_2, start synchronized with 10ms-Task
			Max31865_readTempC(&pt100_2, pt2start);	
			if (pt100_2.state == st_run0) pt2start = false;					
		}
	
		if (Command.can_tx && !Command.tst_cs)
		{
			Command.can_tx = false;
			uint8_t rctx = MCP2515_sendMessage1(&MCP2515, &Txmsg);
			Txmsg.data[0]++;
			sprintf(tmpTx, "sendMessage1 rc= %02u\r\n", rctx);
			uart_puts(UART_ID, tmpTx);
		}

		if (MCP2515.intsrc)
		{					
			MCP2515.intsrc = 0;
			
			if (MCP2515.rcrx == ERROR_OK)
			{
				sprintf(tmpTx, "CANint readMessage1 ID: %lx dlc: %u data: ", Rxmsg.can_id, Rxmsg.can_dlc);
				uart_puts(UART_ID, tmpTx); 										
    
				for (int i = 0; i < Rxmsg.can_dlc; i++) {
					// print the data
					sprintf(tmpTx, "%x", Rxmsg.data[i]);
					uart_puts(UART_ID, tmpTx);
				}
				uart_puts(UART_ID, "\r\n");
			}
		}	

		// SRAM Update, in case of no SPI-Traffic
		if ((pt100_1.state == st_idle) && (pt100_2.state == st_idle))
		{
			if (Command.tst_sram == 1)
			{
				SRAM_Read(&SRAM, &ProcVar);
				sprintf(tmpTx, "SRAM-Command-Read content[0] %u content[9] %u\r\n", ProcVar.ram_content[0], ProcVar.ram_content[9]);
				uart_puts(UART_ID, tmpTx);	
				Command.tst_sram = 0;
			}			
			
			if (Command.tst_sram == 2)
			{
				SRAM_Write(&SRAM, &ProcVar);	
				Command.tst_sram = 0;
			}			
		}

		// Testfunction SPI-Chip-Select
		CSn_Set(Command.tst_cs);

		eeprom_error_t ackpollret = EE_AcKPoll(&EEPROM, i2c1, EEPROML_ADDRESS); //free running, bei EE_WriteByte() ca. 100 Zyklen bis EEPROM_NO_ACK wieder OK
		if (ackpollret == EEPROM_NO_ACK)
		{
			EEPROM.AckPollCnt++;
		}		
	} // end while (true)
} // end main()


void HandleContReception(void)
{
	char tmpTx[200];
	uint8_t tmpn = 0;
	
	if (ttUartCmd.Received) {
		ttUartCmd.Received = false;
		

		// ***************** Spezifier: Show Help ***************** 
		if (ttUartCmd.No == USART_CMD0) // "h"
		{
			sprintf(tmpTx, "Help:\r\n\r\n");
			strcat(tmpTx, "ce[0/1] - 24-V-Enable 0=off, 1=on\r\n");
			uart_puts(UART_ID, tmpTx);			

			sprintf(tmpTx, "ca[0..2] - CAN 0=rd_cnf, 1=rd_ctrL_stat, 2=tx_frame\r\n");
			strcat(tmpTx, "cp[0..2] - set PWM-Servo 0=0%, 1=50%, 2=100%\r\n");
			uart_puts(UART_ID, tmpTx); 			

			sprintf(tmpTx, "cm[0..2] - Memory SRAM 0=disable, 1=rd , 2=rd ++ wr\r\n");
			uart_puts(UART_ID, tmpTx); 			

			sprintf(tmpTx, "co[0..2] - Memory EEPROM 0=disable, 1=rd , 2=rd ++ wr\r\n");
			uart_puts(UART_ID, tmpTx); 			

			sprintf(tmpTx, "cil[0/1..4] - LEDs, cir[0/1..4] - Relays, cio[0/1..4] - 24V Output, cib0/1 - Buzzer\r\n");
			uart_puts(UART_ID, tmpTx); 			
						
			strcpy(tmpTx, "cs[X] - Chip-select disable = 0, pt1 = 1, pt2 = 2, ext_cs1 = 3, ext_cs2 = 4, can = 5, sram = 6, tp206 = 7, tp207 = 8\r\n");
			uart_puts(UART_ID, tmpTx); 			

			sprintf(tmpTx, "crr - RTC - Read Status/DT\r\n");
			strcat(tmpTx, "crw - RTC - Write DT <crwyy.mm.dd,hh:mm:ss>\r\n");
			uart_puts(UART_ID, tmpTx);

			sprintf(tmpTx, "jr[x] - Joy2Read - 0=16bit, 1=8bit 2=Btn, 3=Colour, 4=Lim. Min./Max., 5=BL, 6=FW, 7=I2C-Addr\r\n");
			uart_puts(UART_ID, tmpTx);
			strcpy(tmpTx, "jw[x] - Joy2Write - 0=Colour1, 1=Colour2, 2=Colour3, 3=Lim.Min, 4=Lim.Mid, 5=Lim.Max\r\n");
			uart_puts(UART_ID, tmpTx);
			
			sprintf(tmpTx, "siX - I2C-Scan 0=i2c0(ext), 0=i2c1(int)\r\n");
			strcat(tmpTx, "set - Scan TSL2591-ID\r\n");
			uart_puts(UART_ID, tmpTx); 			
		}
				
		// ***************** Spezifier: Command *****************
		if (ttUartCmd.No == USART_CMD1) // "c"
		{

			//* Command "ce" - 24-V-Enable 0=off, 1=on
			if (ttUartCmd.strRcv[1] == 'e'  && ttUartCmd.Len == 3) {
				
				if (ttUartCmd.strRcv[2] == '0') gpio_put(XGPIO_EN24V, 0);
				if (ttUartCmd.strRcv[2] == '1') gpio_put(XGPIO_EN24V, 1);			
			}
			
			//* Command "ca" -  Choice 0..2
			if (ttUartCmd.strRcv[1] == 'a'  && ttUartCmd.Len == 3) {

				tmpn = ttUartCmd.strRcv[2] - 0x30;
				
				if (tmpn == 0)
				{
					uint8_t conf[] = { 0, 0, 0 };
					conf[0] = MCP2515_readRegister(&MCP2515, MCP_CNF1);
					conf[1] = MCP2515_readRegister(&MCP2515, MCP_CNF2);
					conf[2] = MCP2515_readRegister(&MCP2515, MCP_CNF3);
					sprintf(tmpTx, "CNF1=0x%02x CNF2=0x%02x CNF3=0x%02x\r\n", conf[0], conf[1], conf[2]);
					uart_puts(UART_ID, tmpTx); 					
				}
				
				if (tmpn == 1)
				{
					uint8_t ctrl[] = { 0, 0, 0 };
					uint8_t stat[] = { 0, 0, 0 };
														
					ctrl[0] = MCP2515_readRegister(&MCP2515, 0x3F);
					ctrl[1] = MCP2515_readRegister(&MCP2515, 0x4F);
					ctrl[2] = MCP2515_readRegister(&MCP2515, 0x5F);							
					sprintf(tmpTx, "ctrltx0=0x%02x ctrltx1=0x%02x ctrltx2=0x%02x \r\n", ctrl[0], ctrl[1], ctrl[2]);					
					uart_puts(UART_ID, tmpTx); 					

					stat[0] = MCP2515_readRegister(&MCP2515, 0x3E);
					stat[1] = MCP2515_readRegister(&MCP2515, 0x4E);
					stat[2] = MCP2515_readRegister(&MCP2515, 0x5E);							
					sprintf(tmpTx, "stattx0=0x%02x stattx1=0x%02x stattx2=0x%02x \r\n", stat[0], stat[1], stat[2]);					
					uart_puts(UART_ID, tmpTx); 					
				}
				
				if (tmpn == 2)
				{
					Command.can_tx = true;
				}
			}

			//* Command "cm" - cm[0..2] - Memory SRAM 0=disable, 1=rd , 2=rd
			if (ttUartCmd.strRcv[1] == 'm'  && ttUartCmd.Len == 3) 
			{
				
				tmpn = ttUartCmd.strRcv[2] - 0x30;
				
				if (tmpn >= 0 && tmpn <= 2)
				{
					if (tmpn == 2)
					{
						ProcVar.ram_content[0]++;
					}
					
					Command.tst_sram = 	tmpn;
				}				
			}

			//* Command "cil[0/1..4] - LEDs, cir[0/1..4] - Relays, cio[0/1..4] - 24V Output, cib0/1 - Buzzer"
			if (ttUartCmd.strRcv[1] == 'i'  && ttUartCmd.Len == 4) 
			{				
				tmpn = ttUartCmd.strRcv[3] - 0x30;
								
				if (ttUartCmd.strRcv[2] == 'l')
				{					
					Command.tst_output = tmpn + 10;
				}				

				if (ttUartCmd.strRcv[2] == 'r')
				{
					Command.tst_output = tmpn + 20;
				}				

				if (ttUartCmd.strRcv[2] == 'o')
				{
					Command.tst_output = tmpn + 30;
				}				

				if (ttUartCmd.strRcv[2] == 'b')
				{
					Command.tst_output = tmpn + 40;
				}				
			}
						
			//* Command "cp" -  PWM Choice 0/50/100%
			if (ttUartCmd.strRcv[1] == 'p'  && ttUartCmd.Len == 3) {

				tmpn = ttUartCmd.strRcv[2] - 0x30;

				if (XPWM.IsInitialized && XPWM.IsEnabled)
				{					
					if (tmpn == 0)
					{
						XPWM_SetVal(&XPWM, XPWM_MIN, XPWM_MAX, 300);
					}
				
					if (tmpn == 1)
					{
						XPWM_SetVal(&XPWM, XPWM_MIN, XPWM_MAX, 1500);
					}
				
					if (tmpn == 2)
					{
						XPWM_SetVal(&XPWM, XPWM_MIN, XPWM_MAX, 2500);
					}
				}
			}
			
			//* Command cs[X] - Chip-Select
			if (ttUartCmd.strRcv[1] == 's'  && ttUartCmd.Len == 3) {

				uint8_t cssel = ttUartCmd.strRcv[2] - 0x30;
				
				if (cssel <=  8)
				{
					Command.tst_cs = cssel;
				}				
			}
									
			//* Command "cr" - RTC read/write
			if (ttUartCmd.strRcv[1] == 'r') {
				
				// "crr - RTC_Command Lese Status/DT
				if (ttUartCmd.strRcv[2] == 'r')
				{
					sprintf(tmpTx, "RTC - Lesen Status/DT\r\n");
					uart_puts(UART_ID, tmpTx); 
					Command.rtc_dt_rd = true;
				}

				// "crw - RTC_Command SetTime <cr1yy.mm.dd,hh:mm:ss>\r\n"); Lenght = 20
				if (ttUartCmd.strRcv[2] == 'w')
				{					
					if (ttUartCmd.strRcv[5] == '.' && ttUartCmd.strRcv[8] == '.' &&ttUartCmd.strRcv[11] == ',' && ttUartCmd.strRcv[14] == ':' && ttUartCmd.strRcv[17] == ':' && ttUartCmd.Len == 20)
					{

						SetTime.tm_sec = (ttUartCmd.strRcv[18] - 0x30) * 10 + ttUartCmd.strRcv[19] - 0x30;
						SetTime.tm_min = (ttUartCmd.strRcv[15] - 0x30) * 10 + ttUartCmd.strRcv[16] - 0x30;
						SetTime.tm_hour = (ttUartCmd.strRcv[12] - 0x30) * 10 + ttUartCmd.strRcv[13] - 0x30;
						SetTime.tm_wday = 0;
						SetTime.tm_mday = (ttUartCmd.strRcv[9] - 0x30) * 10 + ttUartCmd.strRcv[10] - 0x30; 
						SetTime.tm_mon = (ttUartCmd.strRcv[6] - 0x30) * 10 + ttUartCmd.strRcv[7] - 0x30;
						SetTime.tm_year = (ttUartCmd.strRcv[3] - 0x30) * 10 + ttUartCmd.strRcv[4] - 0x30;
																		
						sprintf(tmpTx,
							"RTC - Format OK Date: %02u.%02u.%02u Time: %02u:%02u:%02u\r\n",
							SetTime.tm_year,
							SetTime.tm_mon,
							SetTime.tm_mday,
							SetTime.tm_hour,
							SetTime.tm_min,
							SetTime.tm_sec);
						uart_puts(UART_ID, tmpTx); 														
						Command.rtc_dt_wr = true;												
					}
					else
					{
						sprintf(tmpTx, "RTC - Command SetTime Format wrong\r\n");
						uart_puts(UART_ID, tmpTx); 														
					}
				}
			}
		}

		if (ttUartCmd.No == USART_CMD2) // "s"
		{

			//* Command "siX" - I2C-Scan [si0, si1]
			if (ttUartCmd.strRcv[1] == 'i'  && ttUartCmd.Len == 3) 
			{
				
				if (ttUartCmd.strRcv[2] == '0')
				{
					Command.tst_i2c = 1;
				}
				else if (ttUartCmd.strRcv[2] == '1')
				{
					Command.tst_i2c = 2;
				}
			}

			if (ttUartCmd.strRcv[1] == 'e'  && ttUartCmd.strRcv[2] == 't'  && ttUartCmd.Len == 3) 
			{
				Command.scan_sensor0 = true;
			}
		}	
		
		if (ttUartCmd.No == USART_CMD3) // "j"
		{
			//* Command "jrX" - "jr[x] - Joy2Read - 0=16bit, 1=8bit 2=Btn, 3=Colour, 4=Lim. Min./Max., 5=BL, 6=FE, 7= I2C-Addr"
			if (ttUartCmd.strRcv[1] == 'r'  && ttUartCmd.Len == 3) 
			{
				Command.tst_joy2 = ttUartCmd.strRcv[2] - 0x30 + 1;
			}
			//* Command "jwX" - "jw[x] - Joy2Write - 0=Colour1, 1=Colour2, 2=Colour3, 3=Lim.Min, 4=Lim.Mid, 5=Lim.Max");
			else if (ttUartCmd.strRcv[1] == 'w'  && ttUartCmd.Len == 3) 
			{
				Command.tst_joy2 = ttUartCmd.strRcv[2] - 0x30 + 11;				
			}
			else
			{
				Command.tst_joy2 = 0;
			}
		}
	}
}

	

rv3028_error_t RV3028_Write(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t* p_Reg_Data, uint32_t Length)
{
	uint8_t Temp[8] = { Reg_Addr };
	int rc;
	rv3028_error_t valrc = RV3028_NO_ERROR;
	size_t len = Length + 1;

	for (uint8_t i = 0x01; i < (Length + 0x01); i++)
	{
		Temp[i] = *(p_Reg_Data++);
	}

	rc = i2c_write_blocking(i2c1, Device_Addr, Temp, len, false);
	if (rc != len) 
	{
		valrc = RV3028_COMM_ERROR;
	}
	
	return valrc;
}

rv3028_error_t RV3028_Read(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t* p_Reg_Data, uint32_t Length)
{
	int rc;
	rv3028_error_t valrc = RV3028_NO_ERROR;
		
	rc = i2c_write_blocking(i2c1, Device_Addr, &Reg_Addr, sizeof(uint8_t), false);	
	if (rc != sizeof(uint8_t))
	{
		valrc = RV3028_COMM_ERROR;
		return valrc;
	}
	
	rc = i2c_read_blocking(i2c1, Device_Addr, p_Reg_Data, Length, false);
	if (rc != Length)
	{
		valrc = RV3028_COMM_ERROR;
	}

	return valrc;
}

rv3028_error_t RV3028_Interface(rv3028_t* p_Device)
{
	if (p_Device == NULL)
	{
		return RV3028_INVALID_PARAM;
	}

	p_Device->p_Read = (rv3028_read_fptr_t)RV3028_Read;
	p_Device->p_Write = (rv3028_write_fptr_t)RV3028_Write;
	p_Device->DeviceAddr = RV3028_ADDRESS;

	return RV3028_NO_ERROR;
}


xgpio_error_t XGPIO_Interface(xgpio_t *p_xgpio)
{
	if (p_xgpio == NULL)
	{
		return XGPIO_INVALID_PARAM;
	}

	p_xgpio->p_Int_RTCClk = (xgpio_cb_rtcclk_fptr_t)CbRTCClk;
	p_xgpio->p_Int_RTCInt = (xgpio_cb_rtcint_fptr_t)CbRTCInt;
	p_xgpio->p_Int_CAN = (xgpio_cb_canint_fptr_t)CbCANInt;
	p_xgpio->p_Int_PwrOK = (xgpio_cb_canint_fptr_t)CbPwrOKInt;
	return XGPIO_NO_ERROR;
}
	
rv3028_error_t RV3028_DT_Get(rv3028_t* p_Device)
{
	uint8_t Status;
	uint8_t TSCount;
	struct tm LastTS;
	char tmpTx[32];

	// Get the status flags
	RV3028_GetFlags(p_Device, &Status);
	sprintf(tmpTx, "Status: 0x%x\r\n", Status);
	uart_puts(UART_ID, tmpTx); 		

	// Check for a Power On Reset and clear the flag
	if (Status & RV3028_FLAG_POR)
	{
		//NRF_LOG_INFO("  Power On Reset...");
		RV3028_ClearFlags(p_Device, RV3028_FLAG_POR);
	}
	else if (Status & RV3028_FLAG_BATTERY)
	{
		RV3028_ClearFlags(p_Device, RV3028_FLAG_BATTERY);
		strcpy(tmpTx, "Battery switchover occured...\r\n");
		uart_puts(UART_ID, tmpTx); 	

		if (RTC.IsTSEnabled)
		{
			RV3028_GetTS(p_Device, &LastTS, &TSCount);
			sprintf(tmpTx, "Last time stamp: %u:%u:%u\r\n", LastTS.tm_hour, LastTS.tm_min, LastTS.tm_sec);
			uart_puts(UART_ID, tmpTx); 	
		}
	}
	else if (Status & RV3028_FLAG_EVENT)
	{
		strcpy(tmpTx, "Event...\r\n");
		uart_puts(UART_ID, tmpTx); 			
		
		RV3028_ClearFlags(p_Device, RV3028_FLAG_EVENT);

		if (RTC.IsTSEnabled)
		{
			RV3028_GetTS(p_Device, &LastTS, &TSCount);
			sprintf(tmpTx, "Last time stamp: %u:%u:%u\r\n", LastTS.tm_hour, LastTS.tm_min, LastTS.tm_sec);
			uart_puts(UART_ID, tmpTx); 	
		}
	}

	RV3028_GetTime(p_Device, &CurrentTime);
	sprintf(tmpTx, "Current time: %02u:%02u:%02u\r\n", CurrentTime.tm_hour, CurrentTime.tm_min, CurrentTime.tm_sec);
	uart_puts(UART_ID, tmpTx); 	

	sprintf(tmpTx, "Current date: %02u.%02u.%02u\r\n", CurrentTime.tm_mday, CurrentTime.tm_mon, CurrentTime.tm_year + 2000);
	uart_puts(UART_ID, tmpTx); 	
	
	return RV3028_NO_ERROR;	
}


rv3028_error_t RV3028_DT_Set(rv3028_t* p_Device)
{
	
	ErrorCode = RV3028_SetTime(p_Device, &SetTime);
	if (ErrorCode != RV3028_NO_ERROR)
	{
		return ErrorCode;
	}
	
	return RV3028_NO_ERROR;	
}


void RunPLC(void) // Beremiz-Hook 10ms from main-loop
{
	
	// copy process input values to plc
	cpyCtrlx2PLC(&XPLC, &IOEXP_DI_IO, &IOEXP_RE_IO, &IOEXP_DIO_IO, &pt100_1, &pt100_2, &XADC);
		
	// run Beremiz() here
	PLCloop(&XPLC, Global1ms);
	
	// Overwrite from UART
	ForceOutput(&Command);
	// copy plc output values to process
	cpyPLC2Ctrlx(&XPLC, &IOEXP_RE_IO, &IOEXP_DIO_IO);
}


/********************************* write SRAM (if retain variables changed) or if brown-out **********************************/
void SRAM_Write(sram_t *sram, procvar_t *procvar)
{
	char tmpTx[100];

	memcpy(sram->content, procvar->ram_content, sizeof(sram->content));
		
	enum SRAM_ERROR err = SRAM_WritePage(spi1, csn_sram, SRAM_ADRPROCVAR, (unsigned char*)&SRAM, sizeof(sram_t), false);
	sprintf(tmpTx, "SRAM-Write rc= 0x%02x\r\n", err);					
	uart_puts(UART_ID, tmpTx); 												
}

/********************************* read SRAM until Boot (restore retain variables) **********************************/
void SRAM_Read(sram_t *sram, procvar_t *procvar)
{
	char tmp[40];
	memset(sram, 0x0, sizeof(sram_t));
					
	enum SRAM_ERROR err = SRAM_ReadPage(spi1, csn_sram, SRAM_ADRPROCVAR, (unsigned char*)sram, sizeof(sram_t), false);
	
	if (err == SRAM_ERROR_OK)
	{
		memcpy(procvar->ram_content, sram->content, sizeof(procvar->ram_content));
	}
	else
	{
		sprintf(tmp, "SRAM-Initial-Read Error: %u\r\n", err);
		uart_puts(UART_ID, tmp);
	}
}

void ForceOutput(Command_t *cmd)
{

	if (!(bool)cmd->tst_output) return;
	
	if (cmd->tst_output == 10)
	{
		XPLC.IO.do_int[DO_INTERN0].DO[DO_LED1_GN] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_LED2_GN] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_LED3_RD] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_LED4_RD] = false;
		cmd->tst_output = 0;
		return;
	}

	if (cmd->tst_output == 20)
	{
		XPLC.IO.do_int[DO_INTERN0].DO[DO_RELAY1] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_RELAY2] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_RELAY3] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_RELAY4] = false;
		cmd->tst_output = 0;
		return;
	}
	
	if (cmd->tst_output == 30)
	{
		XPLC.IO.do_int[DO_INTERN0].DO[DO_OUT1] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_OUT2] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_OUT3] = false;
		XPLC.IO.do_int[DO_INTERN0].DO[DO_OUT4] = false;
		cmd->tst_output = 0;
		return;
	}
	
	if (cmd->tst_output == 40)
	{
		XPLC.IO.do_int[DO_INTERN0].DO[DO_BUZZER] = false;
		cmd->tst_output = 0;
		return;
	}
		
	if ((cmd->tst_output > 10) && (cmd->tst_output <= 14)) XPLC.IO.do_int[DO_INTERN0].DO[cmd->tst_output - 11] = true; // based 0		
	if ((cmd->tst_output > 20) && (cmd->tst_output <= 24)) XPLC.IO.do_int[DO_INTERN0].DO[cmd->tst_output - 17] = true; // based 4
	if ((cmd->tst_output > 30) && (cmd->tst_output <= 34)) XPLC.IO.do_int[DO_INTERN0].DO[cmd->tst_output - 23] = true; // based 8	
	if (cmd->tst_output == 41) XPLC.IO.do_int[DO_INTERN0].DO[DO_BUZZER] = true;
}


void TestJoy2(Command_t *cmd)
{
	joy2_error_t rc;
	char tmpTx[100];
	
	/* Command "jrX" - "jr[x] - Joy2Read - 0=16bit, 1=8bit 2=Btn, 3=Colour, 4=Lim. Min./Max., 5=BL, 6=FE, 7= I2C-Addr"
     Command "jwX" - "jw[x] - Joy2Write - 0=Colour1, 1=Colour2, 2=Colour3, 3=Lim.Min, 4=Lim.Mid, 5=Lim.Max"); */
	if (cmd->tst_joy2)
	{

		// Read 16bit-Values XY
		if (cmd->tst_joy2 == 1)
		{
			rc = joy2_get_adc_16bits_value_xy(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u ADC16 X:%05u Y:%05u\r\n", rc, JOY2.xADC16, JOY2.yADC16);
			uart_puts(UART_ID, tmpTx);			
	}
		// Read 8bit-Values XY
		if (cmd->tst_joy2 == 2)
		{
			rc = joy2_get_adc_8bits_value_xy(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u ADC8 X:%03u Y:%03u\r\n", rc, JOY2.xADC8, JOY2.yADC8);
			uart_puts(UART_ID, tmpTx);							
		}
		// Read Button-Value (pressed)
		if (cmd->tst_joy2 == 3)
		{
			rc = joy2_get_button_value(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u Btn: %u\r\n", rc, JOY2.BtnPressed);
			uart_puts(UART_ID, tmpTx);	
		}
		// Read Colour RGB
		if (cmd->tst_joy2 == 4)
		{
			rc = joy2_get_rgb_color(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u Color(RGB): %03u/%03u/%03u\r\n", rc, JOY2.ColorR, JOY2.ColorG, JOY2.ColorB);
			uart_puts(UART_ID, tmpTx);				
		}
		// Read Lim. Min./Max
		if (cmd->tst_joy2 == 5)
		{
				
		}
		// Read Version Bootloader
		if (cmd->tst_joy2 == 6)
		{
			rc = joy2_get_bootloader_version(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u VerBL: %03u\r\n", rc, JOY2.VersionBL);
			uart_puts(UART_ID, tmpTx);			
		}
		// Read Version Firmware
		if (cmd->tst_joy2 == 7)
		{
			rc = joy2_get_firmware_version(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u VerFW: %03u\r\n", rc, JOY2.VersionFW);
			uart_puts(UART_ID, tmpTx);							
		}
		// Read I2C-Addr
		if (cmd->tst_joy2 == 8)
		{
			rc = joy2_get_i2c_address(&JOY2);
			sprintf(tmpTx, "joy2 rc= %02u I2C-Addr: 0x%02x\r\n", rc, JOY2.I2CAddr);
			uart_puts(UART_ID, tmpTx);											
		}

		// Write Colour0 
		if (cmd->tst_joy2 == 11)
		{
			rc = joy2_set_rgb_color(&JOY2, 255, 0, 0);
			sprintf(tmpTx, "joy2 rc= %02u Color0(RGB): %03u/%03u/%03u\r\n", rc, JOY2.ColorR, JOY2.ColorG, JOY2.ColorB);
			uart_puts(UART_ID, tmpTx);								
		}
		// Write Colour1
		if (cmd->tst_joy2 == 12)
		{
			rc = joy2_set_rgb_color(&JOY2, 0,255, 0);
			sprintf(tmpTx, "joy2 rc= %02u Color1(RGB): %03u/%03u/%03u\r\n", rc, JOY2.ColorR, JOY2.ColorG, JOY2.ColorB);
			uart_puts(UART_ID, tmpTx);				
		}
		// Write Colour2
		if (cmd->tst_joy2 == 13)
		{
			rc = joy2_set_rgb_color(&JOY2, 0, 0, 255);
			sprintf(tmpTx, "joy2 rc= %02u Color1(RGB): %03u/%03u/%03u\r\n", rc, JOY2.ColorR, JOY2.ColorG, JOY2.ColorB);
			uart_puts(UART_ID, tmpTx);								
		}
					
		cmd->tst_joy2 = 0;
	}
}
void TSL2591_ReadID(uint8_t iaddr, uint8_t _register)
{
	char tmpTx[40];
	uint8_t data;
	int rc;
	/*
	 * .7 = Must write as 1 
	 * .6.5 = 01 = Normal Operation
	 * .4.3.2.1.0 = Addr = 0x12 = ID => Val = 0x50
	 * 10110010 = 0xB2 = Lesezugriff auf TSL2591 ID
	 **/

	rc = i2c_write_blocking(i2c0, iaddr, &_register, 1, false);
	if (rc >= 0) {
		rc = i2c_read_blocking(i2c0, iaddr, &data, 1, false);
		sprintf(tmpTx, "I2C0 TSL2591 rc= %u, ID= %02x\r\n", rc, data);					
		uart_puts(UART_ID, tmpTx);					
	}					
}

bool repeating_timer1ms_callback(struct repeating_timer *t) {

	Global1ms++;
	return true;
}


bool repeating_timer10ms_callback(struct repeating_timer *t) {

	static uint8_t cnt_PtX_Scan = 0;	
	I2Cbusy = true;
	I2CScanIO = true;		
	
	cnt_PtX_Scan++;
	if (cnt_PtX_Scan == OS_PTX_SCAN)
	{
		cnt_PtX_Scan = 0;
		OS_PtX_Scan = true;
	}
	return true;
}

/*RTC-ClkOut Interrupt*/
xgpio_error_t CbRTCClk(uint32_t events)
{
	static uint8_t toggle = 0;
	
	xgpio_error_t rc = XGPIO_NO_ERROR; 

	if (events == GPIO_IRQ_EDGE_RISE && (brtcInitOK && boledInitOK))
	{
		ProcVar.bupdateOled	= true;				
	}		

	toggle = 1 - toggle;		
	gpio_put(PICO_DEFAULT_LED_PIN, toggle);
	return rc;
}

/*RTC-Event/Alarm Interrupt*/
xgpio_error_t CbRTCInt(uint32_t events)
{
	xgpio_error_t rc = XGPIO_NO_ERROR; 
	
	
	return rc;	
}

/*CAN-Controller Interrupt*/
xgpio_error_t CbCANInt(uint32_t events)
{
	xgpio_error_t rc = XGPIO_NO_ERROR; 

	MCP2515.intsrc = MCP2515_getInterrupt(&MCP2515);
	
	if ((MCP2515.intsrc & CANINTF_RX0IF) || (MCP2515.intsrc & CANINTF_RX1IF))
	{
		MCP2515.rcrx = MCP2515_readMessage1(&MCP2515, &Rxmsg);
	}
				
	if (MCP2515.intsrc & CANINTF_ERRIF)
	{
		/*Error-Handling*/
	}
		
	if (MCP2515.intsrc & CANINTF_MERRF)
	{
		/*Error-Handling*/
	}
				
	return rc;	
}

/*PowerOK Interrupt
 *approx 15-25 Changing-Events between Rise and Fall (without 100nF-C1103), HW V1.2 after /PwrOK-detected ->6ms before Voltage Breakdown*/
xgpio_error_t CbPwrOKInt(uint32_t events)
{
	xgpio_error_t rc = XGPIO_NO_ERROR;
	
	if (events == GPIO_IRQ_EDGE_FALL)		
	{
		OsUVFLO = true;
		memset(&IOEXP_RE_IO.DO[0], 0x0, sizeof(IOEXP_RE_IO.DO));			
		IoExp_WriteRE(&IOEXP_RE_IO);
	}
	
	if (events == GPIO_IRQ_EDGE_RISE)
	{
	}		
	return rc;	
}
