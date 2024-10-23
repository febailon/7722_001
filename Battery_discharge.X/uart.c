//----------------------------------------------------------------------------
//	Copyright © 2020 Instant Care, Inc.
//
//	CONFIDENTIAL INFORMATION OF INSTANT CARE, INC.
//	This document contains information proprietary to
//	Instant Care, Inc, and shall not be disclosed, duplicated,
//	or used as a basis for design, manufacture, or sale of apparatus
//	without express written permission of Instant Care, Inc.
//----------------------------------------------------------------------------

#include "config.h"

/*************************************************
 * LOCAL VARIABLES
 ************************************************/


void initUart(void)
{
    RXD_IN = 1;         // Set RX pin as input
    TXD_IN = 0;         // Set TX pin as output
    
#if BAUD == 19200
		BRG16 = 0;
		BRGH = 1;           // 
		SPBRG = 12;         // 12 = 19200
#else
		SYNC = 0;
		BRG16 = 1;
		BRGH = 1;          // 
		SPBRG = 68;        // // 25 = 9600, 12 = 19200, 68 = 115200
#endif
}

void startUart(void)
{
	PIE1bits.RCIE = 1;	// enable the RX irq
    INTCONbits.PEIE = 1;

	SYNC = 0;           // Config uart as asynchronous 
    SPEN = 1;           // Enable UART module 
    
    // Enable both Rx and Tx ports
    CREN = 1;
    TXEN = 1;
}

#if 0
void terminateUart(void)
{
    SPEN = 0;   // disable uart module
    CREN = 0;   // disable the RX
    TXEN = 0;   // disable the TX
}
#endif


void writeMsg(char *msg, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
		writeUart(*msg++);
}

void writeUart(unsigned char data)
{
    TXREG = data;
    while (!TRMT);              // Wait for UART TX buffer to empty completely
}
