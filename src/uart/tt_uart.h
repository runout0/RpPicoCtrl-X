#ifndef _TT_UART_H
#define _TT_UART_H

#include "pico/stdlib.h"

/* Exported constants --------------------------------------------------------*/
#define UART_ID uart0
#define RX_BUFFER_SIZE   24 // default 16

#define USART_CMD0 0
#define USART_CMD1 1
#define USART_CMD2 2
#define USART_CMD3 3
#define USART_CMD4 4
#define USART_CMD5 5
#define USART_CMD6 6
#define USART_CMD7 7
#define USART_CMD8 8
#define USART_CMD9 9
#define USART_CMD10 10
#define USART_CMD11 11
#define USART_CMD12 12
#define USART_CMD13 13
#define USART_CMD14 14
#define USART_CMD15 15
#define USART_CMD16 16
#define USART_CMD17 17
#define USART_CMD18 18
#define USART_CMD19 19
#define USART_CMDMAX 20


/* Exported types ------------------------------------------------------------*/
typedef struct {
	uint8_t major; // Firmwareversion Mayor "major.minor.revision"
	uint8_t minor; // Firmwareversion Minor "major.minor.revision"
	uint8_t revision; // Firmwareversion Revision "major.minor.revision"
	char strProject[20];
	char strName[60];  
} fw_version_t;

typedef struct _ttUartCmd
{
	bool Received;
	uint8_t No; // Range 0..19, 255= Number not found	
	uint8_t Len; // effektive Laenge ohne CR/LF
	char strRcv[RX_BUFFER_SIZE + 10];
	char strCode[20]; //Range 0..19
}
ttUartCmd_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
int ttUART_Configure(void);
void ttUART_StartReception(fw_version_t *fw);
void ttUART_ContReception(void);

#endif
