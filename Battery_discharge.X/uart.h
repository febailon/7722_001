//----------------------------------------------------------------------------
//	Copyright © 2020 Instant Care, Inc.
//
//	CONFIDENTIAL INFORMATION OF INSTANT CARE, INC.
//	This document contains information proprietary to
//	Instant Care, Inc, and shall not be disclosed, duplicated,
//	or used as a basis for design, manufacture, or sale of apparatus
//	without express written permission of Instant Care, Inc.
//----------------------------------------------------------------------------

#ifndef UART_H
#define	UART_H

#include "config.h"

#define RXD_IN      TRISC5
#define TXD_IN      TRISC4

// The Carriage Return (CR) character (0x0D, \r) moves the cursor to the beginning of the line without advancing to the next line. 
// The Line Feed (LF) character (0x0A, \n) moves the cursor down to the next line without returning to the beginning of the line.
#define CR 		0x0D    // \r
#define LF 		0x0A    // \n

void initUart(void);
void startUart(void);
void terminate_uart(void);
void writeUart(uint8_t data);
void writeMsg(char *msg, uint8_t len);

#endif	/* UART_H */
