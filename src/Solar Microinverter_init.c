/////////////////////////////////////////////////////////////////////////////////////////////////
// © 2012 Microchip Technology Inc.
//
// MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any 
// derivatives created by any person or entity by or on your behalf, exclusively with 
// Microchip’s products.  Microchip and its licensors retain all ownership and intellectual 
// property rights in the accompanying software and in all derivatives here to.  
//
// This software and any accompanying information is for suggestion only.  It does not 
// modify Microchip’s standard warranty for its products.  You agree that you are solely 
// responsible for testing the software and determining its suitability.  Microchip has 
// no obligation to modify, test, certify, or support the software.
//
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED 
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, 
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION 
// WITH MICROCHIP’S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
// 
// IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT 
// (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, 
// CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL 
// OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
// SOFTWARE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR 
// THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL 
// LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, 
// IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//
// MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
//
/////////////////////////////////////////////////////////////////////////////////////////////////
 
#include "Solar Microinverter_init.h"

void initClock(void)
{
    // Configure Oscillator to operate the device at 40 MIPS
    //   Fosc = Fin*M/(N1*N2), Fcy = Fosc/2
    //   Fosc = 7.37*(43)/(2*2) = 80MHz for Fosc, Fcy = 40MHz

    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 41; 			    // M = PLLFBD + 2
    CLKDIVbits.PLLPOST = 0;                 // N1 = 2
    CLKDIVbits.PLLPRE = 0;                  // N2 = 2

    // Change oscillator to FRC + PLL 
    __builtin_write_OSCCONH(0x01);		// New Oscillator FRC w/ PLL
    __builtin_write_OSCCONL(0x01);  		// Enable Clock Switch
    
    while(OSCCONbits.COSC != 0b001);		// Wait for new Oscillator to become FRC w/ PLL    
    while(OSCCONbits.LOCK != 1);		// Wait for Pll to Lock

    // Now setup the ADC and PWM clock for 120MHz
    //   ((FRC * 16) / APSTSCLR ) = (7.37 * 16) / 1 = ~ 120MHz

    ACLKCONbits.FRCSEL = 1;                             // FRC provides input for Auxiliary PLL (x16)
    ACLKCONbits.SELACLK = 1;				// Auxiliary Ocillator provides clock source for PWM & ADC
    ACLKCONbits.APSTSCLR = 7;				// Divide Auxiliary clock by 1
    ACLKCONbits.ENAPLL = 1;				// Enable Auxiliary PLL

    while(ACLKCONbits.APLLCK != 1);			// Wait for Aux. PLL to Lock
    
}
void initPWM(void)
{
    // PWM1 and PWM2 Configuration for Interleaved Flyback Drive
    PTCON2bits.PCLKDIV = 0;		// Maximum (1.06nsec) timing resolution

    PTPER = FLYBACKPERIOD;		// Flyback inverter period value
    
    PHASE2 = FLYBACKINTERLEAVEDPHASE;	// 180Deg phase-shift for second converter

    ALTDTR1 = FLYBACKALTDTR;		// Setup Deadtime for PWM1 and PWM2
    DTR1 = FLYBACKDTR;
    
    ALTDTR2 = FLYBACKALTDTR;
    DTR2 = FLYBACKDTR;
                                                                                                                          
    PDC1 = 0;				// Initialize Flyback duty cycle 
    PDC2 = 0;
 
    PWMCON1bits.ITB = 0;		// PTPER register provides the timing for PWM1, PWM2 generator
    PWMCON2bits.ITB = 0;

    PWMCON1bits.IUE = 1;		// Enable immediate updates
    PWMCON2bits.IUE = 1;

    PWMCON1bits.DTC = 0;	        // Positive dead-time enabled
    PWMCON1bits.DTC = 0;
 
    IOCON1bits.PMOD = 0;                // PWM1H and PWM1L is in complementary output mode 
    IOCON2bits.PMOD = 0;                // PWM2H and PWM2L is in complementary output mode 

    IOCON1bits.POLL = 1;                // PWM1L and PWM2L have inverted polarity (active-low)
    IOCON2bits.POLL = 1;                // to drive the P-channel MOSFETs

    IOCON1bits.PENH = 1;                // PWM controls I/O pins
    IOCON1bits.PENL = 1;
    IOCON2bits.PENH = 1;
    IOCON2bits.PENL = 1;

    IOCON1bits.OVRDAT = 0;              // Flyback override data (Safe-State)
    IOCON1bits.OVRENH = 1;              // Enable override (Disable MOSFETs)
    IOCON1bits.OVRENL = 1;                  
    
    IOCON2bits.OVRDAT = 0;              // Flyback override data (Safe-State)
    IOCON2bits.OVRENH = 1;              // Enable override (Disable MOSFETs)
    IOCON2bits.OVRENL = 1;                  

    // Leading Edge Blanking for Current Protection

    // PWM1H rising and falling edges trigger LEB counter (240ns)
    // LEB is applied to current limit input
    LEBCON1 = 0xC4F0;

    // PWM2H rising and falling edges trigger LEB counter (240ns)
    // LEB is applied to current limit input
    LEBCON2 = 0xC4F0;

    // PWM Fault/Current limit Setup
    FCLCON1bits.FLTMOD = 0;		// Latched Fault Mode
    FCLCON1bits.CLMOD = 1;		// Current Limit Enabled
    FCLCON1bits.CLSRC = 1;		// Current Limit Source is Fault 2
    FCLCON1bits.CLPOL = 0;		// Current Limit Source is Active High

    FCLCON2bits.FLTMOD = 0;		// Latched Fault Mode
    FCLCON2bits.CLMOD = 1;		// Current Limit Enabled
    FCLCON2bits.CLSRC = 2;		// Current Limit Source is Fault 3
    FCLCON2bits.CLPOL = 0;		// Current Limit Source is Active High

    // Triggers for ADC Module
    TRGCON1bits.TRGDIV = 0;             // Trigger ADC every PWM Period
    TRGCON1bits.TRGSTRT = 0;            // Wait 0 PWM cycles before generating first PWM trigger

    TRGCON2bits.TRGDIV = 0;		// Trigger ADC every PWM Period
    TRGCON2bits.TRGSTRT = 0;            // Wait 0 PWM cycles before generating first PWM trigger

    TRIG1 = 8;                          // PWM1 used to trigger ADCP4
    TRIG2 = 8;


    // PWM3 Configuration for Full-Bridge Drive
    PWMCON3bits.ITB = 1;		// Independent Time Base

    PHASE3 = FULLBRIDGEPERIOD;		// 2x the switching frequency of the flyback
    SPHASE3 = FULLBRIDGEPERIOD;

    PDC3 = PHASE3 >> 1;			// 50% duty cycle
    SDC3 = SPHASE3 >> 1;

    PWMCON3bits.DTC = 3;		// Dead-time disabled

    IOCON3bits.PMOD = 1;		// Redundant Output Mode

    IOCON3bits.PENH = 1;     		// PWM controls I/O pins
    IOCON3bits.PENL = 1;

    IOCON3bits.OVRDAT = 0;              // Full-Bridge override data (Safe State)
    IOCON3bits.OVRENH = 1;              // Enable override (Disable MOSFETs)
    IOCON3bits.OVRENL = 1;

    // PWM3 Fault/Current Limit Setup
    FCLCON3bits.FLTMOD = 3; 		// Disable Faults

}

void initADC(void)
{
    ADCONbits.SLOWCLK = 1;          // Requirement from ADC Errata
    ADCONbits.FORM = 0;             // Output in Integer Format
    ADCONbits.EIE = 0;              // Disable Early Interrupt
    ADCONbits.ORDER = 0;            // Even channel first
    ADCONbits.SEQSAMP = 0;          // Simultaneus Sampling Enabled
    ADCONbits.ASYNCSAMP = 1;        // Asynchronous Sampling Enableed
    ADCONbits.ADCS = 4;             // Clock Divider is set up for Fadc/5, TAD = 41.6ns
                                    // For simultaneus sampling total conversion time for one pair is 0.58us

    ADSTAT = 0;                     // Clear the ADSTAT register
   
    ADPCFGbits.PCFG0 = 0;           // AN0 PV Panel Voltage Sense
    ADPCFGbits.PCFG1 = 0;           // AN1 PV current sense of Interleaved Flyback 1 converter 
    ADPCFGbits.PCFG2 = 0;           // AN2 PV current sense of Interleaved Flyback 2 converter
    ADPCFGbits.PCFG3 = 0;           // AN3 Output AC Current Sense
    ADPCFGbits.PCFG6 = 0;           // AN6 2.5V Reference
    ADPCFGbits.PCFG7 = 0;           // AN7 Output AC Voltage Sense
    ADPCFGbits.PCFG10 = 0;          // AN10 Temperature Sense
    ADPCFGbits.PCFG11 = 0;          // AN11 12V Drive Supply Sense
   
    ADCPC0bits.TRGSRC0 = 4;         // AN0 and AN1 triggered by PWM1
    ADCPC0bits.TRGSRC1 = 5;         // AN2 and AN3 triggered by PWM2
    ADCPC1bits.TRGSRC3 = 5;         // AN6 and AN7 triggered by PWM2 (use this ISR for Control Loop)
    ADCPC2bits.TRGSRC5 = 5;         // AN10 and AN11 triggered by PWM2

    IPC28bits.ADCP3IP = 6;          // Set ADC Pair3 Interrupt Priority (2nd highest priority)
    IFS7bits.ADCP3IF = 0;	    // Clear ADC Pair3 Interrupt Flag
    IEC7bits.ADCP3IE = 1;           // Enable ADC Pair3 Interrupt at start
}


void initIOPorts(void)
{
    // PWM1 and PWM2 Fault/CMP Remap to Virtual I/O Pins
    RPINR30bits.FLT2R = 32;
    RPINR30bits.FLT3R = 33;

    RPOR16bits.RP32R = 0b101000;
    RPOR16bits.RP33R = 0b101001;

    //LED_DRV1
    TRISCbits.TRISC12 = 0;
    LED_DRV1 = OFF;

    //LED_DRV2
    TRISCbits.TRISC3 = 0;
    LED_DRV2 = OFF;

    //LED_DRV3
    TRISCbits.TRISC8 = 0;
    LED_DRV3 = OFF;

    //OPTO_DRV1
    TRISCbits.TRISC0 = 0;
    OPTO_DRV1 = 0;

    //OPTO_DRV2
    TRISCbits.TRISC13 = 0;
    OPTO_DRV2 = 0;

    //GPIO State when PWM doesn't have ownership
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATBbits.LATB11 = 0;
    LATBbits.LATB12 = 0;
    LATBbits.LATB13 = 0;
    LATBbits.LATB14 = 0;
}

void initStateMachineTimer(void)
{
    T2CONbits.TCKPS = 0;		// Prescaler of 1:1
    PR2 = 4000;				// (100us /25ns) = 4000
    IPC1bits.T2IP = 4;			// Set up Timer interrupt priority
    IFS0bits.T2IF = 0;			// Clear Interrupt Flag
    IEC0bits.T2IE = 1;			// Enable Timer2 interrupt
}

void initLedFaultIndicatorTimer(void)
{
    T3CONbits.TCKPS = 3;		// Prescaler of 1:256
    PR3 = 46875;			// (300ms /(25ns*256)) = 46875
    IPC2bits.T3IP = 5;			// Set up Timer interrupt priority
    IFS0bits.T3IF = 0;			// Clear Interrupt Flag
    IEC0bits.T3IE = 1;			// Enable Timer2 interrupt when fault is present
}

void initCMP(void)
{
    //CMP2C (Flyback Current Phase 1)
    CMPCON2bits.EXTREF = 0;		// Internal Voltage Selected
    CMPCON2bits.RANGE = 1;		// Select Avdd/2 (high-range)
    CMPCON2bits.INSEL = 2;		// Select Comparator 2C

    CMPDAC2bits.CMREF = 700;		// CMREF * (Avdd/2) / 1024 - Volts
    CMPCON2bits.CMPON = 1;		// Enable CMP2

    IPC25bits.AC2IP = 7;		// Highest Priority Interrupt
    IFS6bits.AC2IF = 0;			// Clear Interrupt Flag
    IEC6bits.AC2IE = 1;			// Enable Interrupt to declare fault

    //CMP3B (Flyback Current Phase 2)
    CMPCON3bits.EXTREF = 0;		// Internal Voltage Selected
    CMPCON3bits.RANGE = 1;		// Select Avdd/2 (high-range)
    CMPCON3bits.INSEL = 1;		// Select Comparator 3B

    CMPDAC3bits.CMREF = 700;		// CMREF * (Avdd/2) / 1024 - Volts
    CMPCON3bits.CMPON = 1;		// Enable CMP3

    IPC26bits.AC3IP = 7;		// Highest Priority Interrupt
    IFS6bits.AC3IF = 0;			// Clear Interrupt Flag
    IEC6bits.AC3IE = 1;			// Enable Interrupt to declare fault


    //CMP4C (Flyback Over Voltage Protection)
    CMPCON4bits.EXTREF = 0;		// Internal Voltage Selected
    CMPCON4bits.RANGE = 1;		// Select Avdd/2 (high-range)
    CMPCON4bits.INSEL = 2;		// Select Comparator 4C

    CMPDAC4bits.CMREF = 1000;		// CMREF * (Avdd/2) / 1024 - 1.6 Volts
    CMPCON4bits.CMPON = 1;		// Enable CMP4

    IPC26bits.AC4IP = 7;		// Highest Priority Interrupt
    IFS6bits.AC4IF = 0;			// Clear Interrupt Flag
    IEC6bits.AC4IE = 1;			// Enable Interrupt to disable PWMs and set fault

}
