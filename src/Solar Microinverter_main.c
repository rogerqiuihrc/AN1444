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

#include "Solar Microinverter_main.h"	

// Configuration bits
// Start-up with FRC and switch to FRC w/ Pll
// Configure Watch Dog Timeout (Software Enable/Disable)
// Select ICSP Pair 2 for debugging/programming
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & IOL1WAY_OFF)
_FWDT(FWDTEN_OFF & WDTPOST_PS16 & WDTPRE_PR32 & WINDIS_OFF)
_FPOR(FPWRT_PWR128)
_FICD(ICS_PGD2 & JTAGEN_OFF)


int main()
{
    initClock();                            // Initialize Device Oscillator and Auxiliary Oscillator (PWM/ADC)
    initIOPorts();                          // Initialize all I/O Ports

    initADC();                              // Initialize ADC Module:
  
    ADCONbits.ADON = 1;                     // Enable the ADC Module early for ADC Settling Time

    initCMP();                              // Initialize Comparator Module
    initPWM();                              // Initialize PWM Module 

    initStateMachineTimer();                // Initialize Timer2 (State Machine)
    initLedFaultIndicatorTimer();           // Initialize Timer3 (Fault Indication)

    T2CONbits.TON = 1;                      // Enable State Machine Timer
    PTCONbits.PTEN = 1;                     // Enable PWM Module

    RCONbits.SWDTEN = 1;                    // Enable WDT (cleared at every zero cross)
    
    while(1)
    {
        Nop();
        Nop();
        Nop();
    }
	
    return 0;
}
