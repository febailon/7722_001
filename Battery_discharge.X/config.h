//----------------------------------------------------------------------------
//	Copyright © 2020 Instant Care, Inc.
//
//	CONFIDENTIAL INFORMATION OF INSTANT CARE, INC.
//	This document contains information proprietary to
//	Instant Care, Inc, and shall not be disclosed, duplicated,
//	or used as a basis for design, manufacture, or sale of apparatus
//	without express written permission of Instant Care, Inc.
//----------------------------------------------------------------------------

#ifndef CONFIG_H
#define	CONFIG_H

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON	    // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF

// SYSTEM INCLUDES
#include <xc.h>
#include <htc.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <pic16lf1829.h>

// LOCAL INCLUDES
#include "uart.h"
#include "spi.h"


#define _XTAL_FREQ  32000000
#define BAUD        115200
//#define START_PB     TRISB4      // was MISO REGISTER SDI_PIN, NOW START PUSH BUTTON
//#define ADC_TEST     TRISC7       // was MOSI REGISTER SDO_PIN, NOW ADC TEST

#define WATCHDOG_SLEEP_256ms    0b01000
#define WATCHDOG_SLEEP_512ms    0b01001
#define WATCHDOG_SLEEP_1S       0b01010
#define WATCHDOG_SLEEP_2S       0b01011
#define WATCHDOG_SLEEP_4S       0b01100
#define WATCHDOG_SLEEP_8S       0b01101
#define WATCHDOG_SLEEP_16S      0b01110
#define WATCHDOG_SLEEP_32S      0b01111
#define WATCHDOG_SLEEP_64S      0b10000
#define WATCHDOG_SLEEP_128S     0b10001
#define WATCHDOG_SLEEP_256S     0b10010

#define TX_TEST_FINAL   RB6     // TX TEST, WAS SCLK
#define RESET_			RA2     //  now RESET button
#define R_LED           RB5     // Red LED on actual 7701 board
#define G_LED_DAUGHTER	RC0     // Green LED on battery discharge daughter board - previously SDN
#define TP_4			RC1		// TEST POINT 4 - gpio 1/CTS
#define TP_5            RC2		// TEST POINT 5 - gpio 0 / 
#define TX_LOAD_RELAY   RC3     // TX_LOAD_TEST - previosuly gpio2
#define R_LED_DAUGHTER	RC6     // Red LED on battery discharge daughter board - previously gpio3

// EEPROM addresses
#define EE_BLANK	0x00	// erased status
#define EE_XOFF		0x02	// offsets
#define EE_YOFF		0x04
#define EE_ZOFF		0x06
#define CAL_STATUS	0x10	// calibration status
#define EE_SUPR		0x12	// supervision mode

#define EE_DEBUG	0x20
#define EE_FREQ		0x22
#define EE_ENCRYPT	0x24

// EE values
#define INIT_OK		0x00
#define EE_ERASED	0xFF
#define CAL_DONE	0x57

void	initPic();
uint16_t CS_rf_protocol_CRC8_byteBuffer(uint8_t buf[], uint8_t len);
void	blinkLED(uint8_t ticks);
void	initEeprom(void);
void	writeEeprom( uint8_t addr, uint8_t val);

#endif	/* CONFIG_H */

