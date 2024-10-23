//----------------------------------------------------------------------------
//	Copyright © 2020 Instant Care, Inc.
//
//	CONFIDENTIAL INFORMATION OF INSTANT CARE, INC.
//	This document contains information proprietary to
//	Instant Care, Inc, and shall not be disclosed, duplicated,
//	or used as a basis for design, manufacture, or sale of apparatus
//	without express written permission of Instant Care, Inc.
//----------------------------------------------------------------------------

#ifndef SPI_H
#define	SPI_H

#include "config.h"

#define START_PB        RB4
#define ADC_TEST        RC7
#define Discharge_battery_relay		RB7   // nCS_RF for RB7, now Discharge relay


/*****************************************************
 * PROTOTYPES
 ****************************************************/

uint8_t spiTransfer(uint8_t data);
void	initSPI(void);

#endif	/* SPI_H */

