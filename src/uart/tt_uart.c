/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "tt_uart.h"

extern ttUartCmd_t ttUartCmd;
static uint8_t RXBuffer[RX_BUFFER_SIZE];
static char RXString[RX_BUFFER_SIZE + 10];
static volatile uint32_t ReceivedChars;
static volatile bool ReadyIndication = false;

#define BAUD_RATE 115200 
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE 

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// RX interrupt handler
void on_uart_rx() {

	uint8_t tmpchar;
	
	/* Read Received character.*/
	while (uart_is_readable(UART_ID)) {
			
		tmpchar  = uart_getc(UART_ID);
		RXBuffer[ReceivedChars++] = tmpchar;
			
		/* Checks if Buffer full indication has been set or CR*/
		if ((ReceivedChars >= RX_BUFFER_SIZE - 1) || (tmpchar == 0x0D))			
		{
			if (tmpchar == 0x0D) ReceivedChars--;
			ttUartCmd.Len = ReceivedChars;
			memcpy(RXString, RXBuffer, ReceivedChars);
			memset(RXBuffer, 0x0, sizeof(RXBuffer));		
			if (ReceivedChars) ReadyIndication = true;
			ReceivedChars	= 0;
		}	
	}
} 	

int ttUART_Configure() {

	int rc = 0;

	// Set up our UART with the required speed.
	uart_init(UART_ID, BAUD_RATE);

	// Set the TX and RX pins by using the function select on the GPIO
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); 
	

	// Set up a RX interrupt
	int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

	// And set up and enable the interrupt handlers
	irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
	irq_set_enabled(UART_IRQ, true);

	// Now enable the UART to send interrupts - RX only
	uart_set_irq_enables(UART_ID, true, false);
	return rc;
}


void ttUART_StartReception(fw_version_t *fw)
{  
	/* Print user info on PC com port */	
	char txtInfoStart[120];

	strcpy(txtInfoStart, "\n...\r");
	uart_puts(UART_ID, txtInfoStart);	
	
	sprintf(txtInfoStart, "\n%s, %s FW-Version %u.%u.%u\r\n", fw->strName, fw->strProject, fw->major, fw->minor, fw->revision);  
	uart_puts(UART_ID, txtInfoStart);	
}


void ttUART_ContReception(void)
{
	bool codenok = true;

	/* Checks if Buffer full indication has been set */
	if (ReadyIndication)
	{
		/* Reset indication */
		ReadyIndication = false;

		for (uint8_t i = 0; i < USART_CMDMAX; i++)
		{
			
			char c = ttUartCmd.strCode[i];
			
			if (!strncmp(RXString, &c, 1))
			{
				ttUartCmd.No = i;
				codenok = false;
				break;				
			}			
		}
		
		if (codenok) ttUartCmd.No = 255;
		ttUartCmd.Received = true;
		memcpy(ttUartCmd.strRcv, RXString, sizeof(ttUartCmd.strRcv));		
		memset(RXString, 0x0, sizeof(RXString));
	}
}

