/* 
 * File:   battery_discharge.c
 * Author: Dick
 *
 * Created on October 10, 2024, 9:10 AM
 */

#include "config.h"

/*
 * 
 */

#define LOW_VOLTAGE_THRESHOLD   476         // ~2.2v = (1.024 * (2^10 - 1) / 2.2v) = 476.16
#define LOW_VOLTAGE_THRESHOLD_1 460         // ~2.28v = (1.024 * (2^10 - 1) / 2.28v) = 460
#define LOW_BATTERY_CONSECUTIVE_COUNTS  3
#define LOW_BATT	0x20
#define GOOD_BATT_THRESHOLD 332             // ~3.15 = (1.024 * (2^10 - 1) / 3.15v) = 332.55

uint8_t str_voltage = 0;
uint8_t consecutiveLowReadings = 0;
uint16_t startup_voltage = 0;
uint16_t TX_TEST_voltage = 0;
uint8_t prnBuf[128] = {0};
uint8_t rxChar = 0;
uint8_t batt_mode = 0;                      // mode 1 = CR123a, mode 2 = 2477, mode 3 = 2032 etc.
uint8_t discharge_EOL_Complete = 0;         // sets the condition for completed EOL
char buffer_char[128] = {0};
//float float_buffer_val = 0;







static bool isFVReadDone(void)
{
    return (bool)!ADCON0bits.GO_nDONE;
}

uint16_t getVoltage(void)
{
    uint8_t lo, hi;
    uint16_t voltage;
    
    FVRCONbits.ADFVR = 0b01;	// Fixed voltage ref is 1x or (1.024 V)
    FVRCONbits.FVREN = 0b1;     // enable fvr
    while(!FVRCONbits.FVRRDY);  // spin until ready
    
    ////////////////////////////////////////////////////////////////////////////
    ADCON1bits.ADPREF = 0b10;   // read fixed voltage reg - Vdd
    ADCON1bits.ADNREF = 0b1;    // read fixed voltage reg - Vdd   // this is test
    ///////////////////////////////////////////////////////////////////////////
    
    ADCON1bits.ADCS = 0b010;    // Fosc/32 for a 1Mhz clock is in the recommended range 
    ADCON1bits.ADFM = 0b1;      // result is right justified in ADRESH
    ADCON0bits.CHS = 0b01001;   // test for rc7 
    ADCON0bits.ADON = 1;        // enable analog
    __delay_ms(1); // this setup delay is necessary for the ADC hardware
    
    TX_LOAD_RELAY = 1;
	__delay_ms(1);              // this setup delay is necessary for the ADC hardware
    ADCON0bits.GO = 1;          // start read
    while (!isFVReadDone());    // Spin for ~100us
    TX_LOAD_RELAY = 0;
    lo = ADRESLbits.ADRESL;
    hi = (uint8_t)(ADRESHbits.ADRESH & 0x03);
    voltage = (uint16_t)((hi << 8) | (lo & 0xFC));  // We only care about 8 MSB in this 10-bit measurement
    //sprintf(buffer_char, "\r\n BatteryVV: %04u \n\r", voltage); // checking voltage for debug
    //writeMsg(buffer_char, strlen(buffer_char));
    ADCON1 = 0;                         
    ADCON0bits.ADON = 0;        // Disable Analog
    FVRCONbits.FVREN = 0;       // Disable FVR
   	FVRCONbits.ADFVR = 0b00;	// turn FVR Output off
    
    return(voltage);
}







void blinkLED_red(uint8_t ticks)
{
    for (uint8_t i = 0; i < ticks; i++)
{
    R_LED = 0;
    CLRWDT();
    __delay_ms(100);
    R_LED = 1;
    CLRWDT();
    __delay_ms(100);
    }
}

void blinkLED_RED_daughterboard(uint8_t ticks)
{
    for (uint8_t i = 0; i < ticks; i++)
{
    R_LED_DAUGHTER = 0;
    CLRWDT();
    __delay_ms(100);
    R_LED_DAUGHTER = 1;
    CLRWDT();
    __delay_ms(100);
    }
}

void blinkLED_GRN_daughterboard(uint8_t ticks)
{
    for (uint8_t i = 0; i < ticks; i++)
{
    G_LED_DAUGHTER = 0;
    CLRWDT();
    __delay_ms(100);
    G_LED_DAUGHTER = 1;
    CLRWDT();
    __delay_ms(100);
    }
}




void __interrupt() ISR(void){
    
    if(IOCAFbits.IOCAF2) // reset button flag(pressed) || set all relays open
    {
        INTCONbits.IOCIF = 0; 
        IOCAFbits.IOCAF2 = 0;// clear interrupt
        Discharge_battery_relay = 0; // really is open and not engaged
        TX_LOAD_RELAY = 0;           // relay is open and not engaged
        G_LED_DAUGHTER = 1;          // Both LEDs reset to off 
        R_LED_DAUGHTER = 1;
        discharge_EOL_Complete = 0; // reset finished behavior 
        blinkLED_red(3);
        
        // RESET TIMER FOR TRACKING EOL, 1/2, 3/4
        
        strcpy(prnBuf, "\r\n DEBUG 2 \r\n");
        writeMsg(prnBuf, strlen(prnBuf));
        //receivedSync++;
    }

	if (IOCBFbits.IOCBF4)                                       // find start_test button definition and set parameter to Interrupt flag
	{
        IOCBFbits.IOCBF4 = 0;                                   // clear interrupt
        discharge_EOL_Complete = 0;                             // reset finished behavior
        startup_voltage = getVoltage();
        strcpy(prnBuf, "\r\n DEBUG 1 \r\n");
        writeMsg(prnBuf, strlen(prnBuf));
        
        if(startup_voltage >= GOOD_BATT_THRESHOLD)
        {
            R_LED_DAUGHTER = 0;                                      // RED LED = ON FOR BAD BATT
            strcpy(prnBuf, "\r\n Battery below 3.1V - Not GB \r\n"); // COM PORT COMMUNICATION OF NOT GB
            writeMsg(prnBuf, strlen(prnBuf));            
            sprintf(buffer_char, "\r\n Battery: %04u \n\r", startup_voltage);
            writeMsg(buffer_char, strlen(buffer_char));
        }
        
        else {
            strcpy(prnBuf, "\r\n OK GB \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            //sprintf(buffer_char, "Battery: %03d", str_voltage);
            sprintf(buffer_char, "\r\n Battery: %04u \r\n", startup_voltage);
            writeMsg(buffer_char, strlen(buffer_char));
        }
        
       
	}
    
    if(IOCBFbits.IOCBF6)                    // TX Test Button flag || close relay and sample battery. this is after rebound
    {
        IOCBFbits.IOCBF6 = 0;               // clear interrupt
        Discharge_battery_relay = 0;        // relay is not engaged for tx load test
        G_LED_DAUGHTER = 1;                 // Both LEDs reset to off 
        R_LED_DAUGHTER = 1;
        TX_LOAD_RELAY = 1;                  // relay is engaged for load and sample
        TX_TEST_voltage = getVoltage();     // Get voltage during 
        TX_LOAD_RELAY = 0;                  // relay is disengaged 
        //strcpy(prnBuf, "\r\n Battery: %s \r\n",TX_TEST_voltage);
        //writeMsg(prnBuf, strlen(prnBuf));
        
        if(TX_TEST_voltage <= LOW_VOLTAGE_THRESHOLD & TX_TEST_voltage >= LOW_VOLTAGE_THRESHOLD_1 ){     // add a more intricate range
            Discharge_battery_relay = 0;                        // relay is not engaged for tx load test
            TX_LOAD_RELAY = 0;                                  // relay is disengaged 
            strcpy(prnBuf, "\r\n Battery at EOL Voltage \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            discharge_EOL_Complete = 1;
        }
        // RESET TIMER FOR TRACKING EOL, 1/2, 3/4     
        //receivedSync++;
    }
    
    

    	if (PIR1bits.RCIF)
	{
		PIR1bits.RCIF = 0;
		rxChar = RCREG;

		if (rxChar == 'b')
        {
            if (batt_mode == 4 )
            {
                batt_mode = 0;
            }
            batt_mode ++;           
            if(batt_mode == 1){
            strcpy(prnBuf, "\r\n CR123A Discharge \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            // change timing definitions for EOL wait time depending
            // on battery option 
            
            }
            
            if(batt_mode == 2){
            strcpy(prnBuf, "\r\n Panasonic 2477 Discharge \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            // change timing definitions for EOL wait time depending
            // on battery option
            }
            
            if(batt_mode == 3){
            strcpy(prnBuf, "\r\n Panasonic 2032 Discharge \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            // change timing definitions for EOL wait time depending
            // on battery option
            }
            
            if(batt_mode == 4){
            strcpy(prnBuf, "\r\n Panasonic 1632 Discharge \r\n");
            writeMsg(prnBuf, strlen(prnBuf));
            
            // change timing definitions for EOL wait time depending
            // on battery option
            
            
            }                                  
        }
            
            
			//encrypt_mode = !encrypt_mode;
        
        
		else if (rxChar == 't'){    // this function changes between EOL,1/2,3/4 
			//freq_mode = !freq_mode;
            //else if (rxChar == 'd')
			//debug_mode++;
		
		//mode_chng = 1;
        }
	}
}





void main(void) {
    
    initPic();
    blinkLED_red(3);
    initUart();
	startUart();
    CLRWDT();
    
    Discharge_battery_relay = 0; // relay is open / not engaged
    TX_LOAD_RELAY = 0;           // relay is open / not engaged
    
    
    
    strcpy(prnBuf, "\r\nInstant Care Battery Discharge IC\r\n");
	writeMsg(prnBuf, strlen(prnBuf));
    
        
    ////////// Main Sample routine //////////////
    
    while(1)
    {
        // EOL success behavior. If voltage is at LB then set LED to correct blinking to signify complete
        if (discharge_EOL_Complete == 1){
            
            while(1){
                blinkLED_GRN_daughterboard(4);
                }
            
        } 
        
        
        
        //blinkLED_RED_daughterboard(4);
        //blinkLED_GRN_daughterboard(4);
      
    }
}











void initPic(void)
{
    CLRWDT();
    
    // SW control WDT
    WDTCONbits.SWDTEN = 0;
    
   // Config WDT interval
    WDTCONbits.WDTPS = WATCHDOG_SLEEP_1S; 
    INTCON = 0;                         // Disable all interrupts

    OSCTUNE = 0;
	OSCCON = 0b11110000;    // Internal Oscillator 32Mhz 
     
    OPTION_REG = 0b00000111;  // WPU are enabled by individ. WPU latch vals.
    APFCON0 = 0b10000100;   // RC4->USART TX , RC5->USART RX
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0b10000110;    // RC7 = ADC_test which is analog in order to get a voltage
                            // RC0 =  I/O (G_LED), RC 1&2 = TestPoint(ANALOG), RC3 = I/O (DISCHARGE_RELAY),  RC6 = DIGITAL I/O (RED_LED_DAUGHTER)
    
    TRISA = 0b00000100;		// 
    WPUA  = 0b00000100;     // add weak pull up to push buttons to get better defined signals for interrupts
	LATA  = 0b00000000;

	TRISB = 0b01010000;     // 
    WPUB  = 0b00010000;     // add weak pull up to push buttons to get better defined signals for interrupts
    LATB  = 0b00000000;

    TRISC = 0b10101110;     // rc7 was 1
    WPUC  = 0b00101000;     // add weak pull up to push buttons to get better defined signals for interrupts
    LATC  = 0b01000001;
	

    FVRCONbits.ADFVR = 0b01; // Fixed voltage ref is 1x or (1.024 V)
    FVRCONbits.CDAFVR = 0b00; // not using DAC and CPS
    FVRCONbits.TSRNG = 0b0; // not using temp indicator
    FVRCONbits.TSEN = 0b0;  // temp sensor disabled

    ADCON0bits.ADON = 0;        // Disable Analog
    FVRCONbits.FVREN = 0;       // Disable FVR
   	FVRCONbits.ADFVR = 0b00;	// turn FVR Output off

    
    INTCONbits.IOCIF = 0;
    INTCONbits.IOCIE = 1;	// except interrupt-on-change

    IOCAFbits.IOCAF2 = 0;
    IOCANbits.IOCAN2 = 1;	// "Reset" button on RA2 now on Daughter-board     //RA2 - NIRQ
    IOCAPbits.IOCAP2 = 0;
    
    IOCBFbits.IOCBF4 = 0;
    IOCBNbits.IOCBN4 = 1; // Detect "start" push button on falling edge and set flag to observe button press
    IOCBPbits.IOCBP4 = 0;
    
    IOCBFbits.IOCBF6 = 0;
    IOCBNbits.IOCBN6 = 1; // Detect "start" push button on falling edge and set flag to observe button press
    IOCBPbits.IOCBP6 = 0;
    

    T1CONbits.nT1SYNC = 1;
    T1CONbits.TMR1CS = 0b00;
    T1CONbits.T1CKPS = 0b11;

	INTCONbits.GIE = 1;
}



void discharge_relay(uint8_t num)
{
    if (num == 0)
    {
        Discharge_battery_relay = 0; // turns the relay off
    }
    else if (num == 1)
    {
        Discharge_battery_relay = 1; // turns the relay on
    }
     
}
void TX_load_relay(uint8_t num)
{
    if (num == 0)
    {
        TX_LOAD_RELAY = 0; // turns the relay off
    }
    else if (num == 1)
    {
        TX_LOAD_RELAY = 1; // turns the relay on
    }
}


//////////////////////////////////////////////////////////////////////
///// Battery Check Section ////////////////////////
//////////////////////////////////////////////////////
uint8_t isLowBattery(void)
{  
    return (consecutiveLowReadings >= 3 ? LOW_BATT : 0);  
}

void batteryTest(void)
{
    if (isLowBattery())
        return;      // Once battery is detected low, it stays low
    
    if (getVoltage() >= LOW_VOLTAGE_THRESHOLD)
        consecutiveLowReadings++;
    else
        consecutiveLowReadings = 0;

    return;
}




