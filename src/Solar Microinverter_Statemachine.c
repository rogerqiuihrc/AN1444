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

#include "Solar Microinverter_Statemachine.h"

// System Fault Variables
unsigned char storeFaultState = 0, faultCheckFlag = 0, criticalFaultRestartFlag = 0;
unsigned char pvPanelOverVoltageFlag = 0, pvPanelUnderVoltageFlag = 0, overTemperatureFlag = 0;
unsigned char inverterOverVoltageFlag = 0, inverterUnderVoltageFlag = 0, pvUnderVoltageCounter = 0;
unsigned char inverterUnderVoltageCounter = 0, inverterOverVoltageCounter = 0;
unsigned char tempFaultCnt = 0, driveSupplyFaultCnt = 0, referenceVoltageFaultCnt = 0;
unsigned int criticalFaultCounter = 0, pvPanelMPPOverVoltageCnt = 0;
unsigned int inverterOutputOverCurrent = 0; 

// Start-up and Restart Variables
unsigned char zeroCrossDelay = 45, startFullBridgeFlag = 0;
unsigned char zeroCrossDelayNom = 30, zeroCrossDelayMin = 22, zeroCrossDelayMax = 37;
unsigned char acCurrentOffsetFlag = 0, acCurrentOffsetCounter = 0;
unsigned int systemRestartCounter = 0, acCurrentOffset = 0;
unsigned int inverterPeriodMin = INVERTERPERIOD60HZMIN, inverterPeriodMax = INVERTERPERIOD50HZMAX;
long unsigned int acCurrentOffsetAverage = 0;

// MPPT and Load Balance Variables
unsigned char  loadBalanceCounter = 0, mpptCounter = 0, mpptStartUpFlag = 0;
unsigned int inputVoltageAverage = 0, inputCurrentAverage = 0, openCircuitVoltage = 0;
unsigned int mpptFactor = MPPTFACTORMINIMUM;
int inputPower = 0, prevInputVoltageAverage = 0, deltaDutyCycle = 0;

// Startup in system error, If a fault is detected 
// faultstate will change during the restart counter.
unsigned char systemState = SYSTEMERROR;
unsigned char switchState = SWITCHOFF;
unsigned char faultState = NO_FAULT;

// Externally Defined Variables
extern unsigned char avgInputDataReadyFlag, startupZeroCrossCounter;
extern unsigned char criticalFaultFlag, acCurrentOffsetFlag, ninetyDegreeDetectFlag;
extern unsigned char burstModeActiveFlag;
extern unsigned int mpptFactorMaximum, numberofSamples;
extern unsigned int measuredTemperature, driveSupplyVoltage, referenceVoltage;
extern unsigned int averageFlybackCurrent1, averageFlybackCurrent2;
extern unsigned int inverterPeriod, peakInverterOutputVoltage, burstModeActiveCounter;;
extern int inverterOutputCurrent;

// Variable Declaration for DMCI Debugging Tool
#ifdef DMCI_STATEMACHINE
int array1[100];
int array2[100];
int array3[100];
unsigned char dmciArrayIndex = 0;
#endif

#ifdef DMCI_MPPT 
int array1[100];
int array2[100];
int array3[100];
unsigned char dmciArrayIndex = 0;
#endif


// Timer 3 Interrupt (300ms interrupt) for Fault indication
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt()
{ 
    static unsigned char ledCounter = 0, interruptCounter = 0;

    if(ledCounter < storeFaultState)
    {
        LED_DRV1 ^= ON;		// Blink LED to indicate fault, storeFaultState is 2x to account for toggle Off
        ledCounter++;
    }
    else if (ledCounter < (storeFaultState + 3))	// Wait three interrupts (Clear indication blinking has stopped)
    {
        ledCounter++;
    }
    else
    {
        ledCounter = 0;
    }

    if(systemState == DAYMODE)
    {
        interruptCounter++;
    }

    // InterruptCounter is used to diplay the last known fault for some time after
    // the system restarts (300ms * 200 ~ 1 minute)
    if((faultState == NO_FAULT) && (interruptCounter >= 200))
    {
        T3CONbits.TON = 0;	//Disable this interrupt if the fault is removed and the delay has passed
        ledCounter = 0;
        interruptCounter = 0;
        LED_DRV1 = OFF;
    }

    TMR3 = 0;
    IFS0bits.T3IF = 0;
}

// Timer 2 Interrupt (100us interrupt), performs fault checking and system statemachine
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt()
{	
    // Check Inverter Output Over Voltage Condition
    if((peakInverterOutputVoltage > INVERTER_OVERVOLTAGE_LIMIT) && (inverterOverVoltageFlag == 0))
    {
        inverterOverVoltageCounter++;

        if(inverterOverVoltageCounter >= 2)
        {
            if(faultState == NO_FAULT)
            {
                faultState = INVERTER_VOLTAGE;
                inverterOverVoltageFlag = 1;
            }
        }
    }
    else if((peakInverterOutputVoltage < INVERTER_OVERVOLTAGE_LIMIT_HYS) && (inverterOverVoltageFlag == 1))
    {
        if(faultState == INVERTER_VOLTAGE)
        {
            faultState = NO_FAULT;
        }
        inverterOverVoltageFlag = 0;
    }
    else
    {
        inverterOverVoltageCounter = 0;
    }

    if((peakInverterOutputVoltage < INVERTER_UNDERVOLTAGE_LIMIT) && (inverterUnderVoltageFlag == 0))
    {
        inverterUnderVoltageCounter++;

        if(inverterUnderVoltageCounter >= 2)
        {
            if(faultState == NO_FAULT)
            {
                faultState = INVERTER_VOLTAGE;
                inverterUnderVoltageFlag = 1;
            }

            inverterUnderVoltageCounter = 2;
        }
    }
    else if ((peakInverterOutputVoltage > INVERTER_UNDERVOLTAGE_LIMIT_HYS) && (inverterUnderVoltageFlag == 1))
    {
        if(faultState == INVERTER_VOLTAGE)
        {
            faultState = NO_FAULT;
        }

        inverterUnderVoltageFlag = 0;
    }
    else
    {
        inverterUnderVoltageCounter = 0;
    }

    // Only Check Input Voltage Fault When New Data is Available
    if(avgInputDataReadyFlag == 1)
    {
        // Check PV Panel Voltage Using the Average Input Voltage
        if((inputVoltageAverage > PVPANEL_OVERVOLTAGE_LIMIT) && (pvPanelOverVoltageFlag == 0))
        {
            if(faultState == NO_FAULT)
            {
                faultState = PV_PANEL_VOLTAGE;
                pvPanelOverVoltageFlag = 1;
            }
        }
        else if((inputVoltageAverage < PVPANEL_OVERVOLTAGE_LIMIT_HYS) && (pvPanelOverVoltageFlag == 1))
        {
            if(faultState == PV_PANEL_VOLTAGE)
            {
                faultState = NO_FAULT;
            }

            pvPanelOverVoltageFlag = 0;
        }

        // Check PV Panel Maximum Power Point Voltage Using the Average Input Voltage
        if((inputVoltageAverage > PVPANEL_MPP_LIMIT) && (systemState == DAYMODE))
        {
            pvPanelMPPOverVoltageCnt++;

            // Allow time for system to find MPP
            if(pvPanelMPPOverVoltageCnt > 1000)
            {
                pvPanelOverVoltageFlag = 1;		// Set Flag so System Will Restart
                pvPanelMPPOverVoltageCnt = 0;

                if(faultState == NO_FAULT)
                {
                    faultState = PV_PANEL_VOLTAGE;
                }
            }
        }
        else
        {
            pvPanelMPPOverVoltageCnt = 0;
        }


        // Check PV Panel Minimum Voltage Using Average Input Voltage
        if((inputVoltageAverage < PVPANEL_UNDERVOLTAGE_LIMIT) && (pvPanelUnderVoltageFlag == 0))
        {
            pvUnderVoltageCounter++;

            if(pvUnderVoltageCounter >= 10)
            {
                if(faultState == NO_FAULT)
                {
                    faultState = PV_PANEL_VOLTAGE;
                    pvPanelUnderVoltageFlag = 1;
                }

                pvUnderVoltageCounter = 10;
            }
        }
        else if ((inputVoltageAverage > PVPANEL_UNDERVOLTAGE_LIMIT_HYS) && (pvPanelUnderVoltageFlag == 1))
        {
            if(faultState == PV_PANEL_VOLTAGE)
            {
                faultState = NO_FAULT;
            }

            pvPanelUnderVoltageFlag = 0;
        }
        else
        {
            pvUnderVoltageCounter = 0;
        }
    }

    // Check Over Temperature Fault
    if((measuredTemperature >= MAXTEMPERATURE) && (overTemperatureFlag == 0))
    {
        tempFaultCnt++;

        if(tempFaultCnt > 100)
        {
            tempFaultCnt = 100;

            if(faultState == NO_FAULT)
            {
                faultState = TEMPERATURE;
                overTemperatureFlag = 1;
            }
        }
    }
    else if ((measuredTemperature <= MAXTEMPERATURE_HYS) && (overTemperatureFlag == 1))
    {
        overTemperatureFlag = 0;

        if(faultState == TEMPERATURE)
        {
            faultState = NO_FAULT;
        }
    }
    else
    {
        tempFaultCnt = 0;
    }

    //Check 12V Drive Supply Voltage
    if((driveSupplyVoltage > MAXDRIVEVOLTAGE) || (driveSupplyVoltage < MINDRIVEVOLTAGE))
    {
        driveSupplyFaultCnt++;

        if(driveSupplyFaultCnt > 10)
        {
            driveSupplyFaultCnt = 10;

            if(faultState == NO_FAULT)
            {
                faultState = DRIVE_SUPPLY;
            }
        }
    }
    else
    {
        driveSupplyFaultCnt = 0;

        if(faultState == DRIVE_SUPPLY)
        {
            faultState = NO_FAULT;
        }
    }

    // Check Reference Voltage
    if((referenceVoltage < MINREFERENCEVOLTAGE) || (referenceVoltage > MAXREFERENCEVOLTAGE))
    {
        referenceVoltageFaultCnt++;

        if(referenceVoltageFaultCnt > 10)
        {
            referenceVoltageFaultCnt = 10;

            if(faultState == NO_FAULT)
            {
                faultState = REFERENCE_VOLTAGE;
            }
        }
    }
    else
    {
        referenceVoltageFaultCnt = 0;

        if (faultState == REFERENCE_VOLTAGE)
        {
            faultState = NO_FAULT;
        }
    }


    // Check the fault state, if it is not equal to No Fault then set systemState = SYSTEM_ERROR
    if (faultState != NO_FAULT)
    {
        systemState = SYSTEMERROR;
    }

    // Check the On/Off switch state
    if(PORTCbits.RC11 == 0)
    {
        switchState = SWITCHON;
    }
    else
    {
        //put system state in systemError
        switchState = SWITCHOFF;
        systemState = SYSTEMERROR;
    }

    switch(systemState)
    {
        case SYSTEMSTARTUP:
        {
            if(avgInputDataReadyFlag == 1)
            {
                // During system startup read the PV panel Voltage (open circuit voltage). This information
                // is used to help speed up the time to find MPP
                openCircuitVoltage = inputVoltageAverage;

                avgInputDataReadyFlag = 0;
            }

            // Read AC Current offset during startup mode and verify data
            // Needs to be completed before the full-bridge is enabled during system startup
            if(acCurrentOffsetFlag == 0)
            {
                acCurrentOffsetAverage = acCurrentOffsetAverage + inverterOutputCurrent;

                if(acCurrentOffsetCounter >= 255)
                {
                    acCurrentOffset = (acCurrentOffsetAverage >> 8);
                    inverterOutputOverCurrent = INVERTER_OUTPUTCURRENT_MAX + (16383 - acCurrentOffset);
                    acCurrentOffsetFlag = 1;
                    acCurrentOffsetCounter = 0;
                    acCurrentOffsetAverage = 0;

                    if((acCurrentOffset < MINOFFSETCURRENT) || (acCurrentOffset > MAXOFFSETCURRENT))
                    {
                        if(faultState == NO_FAULT)
                        {
                            faultState = ACCURRENT_OFFSET;
                        }
                    }
                }
                else
                {
                    acCurrentOffsetCounter++;
                }
            }

            // At system Start-up, enable the Full-Bridge circuit (@ peak of AC Cycle) before the flyback circuit is enabled
            // If this is not done, the flyback output will have high DC voltage and when the full-bridge is enabled at the
            // zero cross there is a large dv/dt and the output current will have a large glitch and also trip the flyback
            // OVP circuit

            if((startupZeroCrossCounter >= (ZEROCROSSCOUNT>>1)) && (ninetyDegreeDetectFlag == 1))
            {
                startFullBridgeFlag = 1;				// Set Flag to start full-bridge drive
            }

            // After several consecutive zero crossings switch to Day Mode
            if(startupZeroCrossCounter > ZEROCROSSCOUNT)
            {
                if ((inverterPeriod > INVERTERPERIOD60HZMIN ) && (inverterPeriod <= INVERTERPERIOD60HZMAX))
                {
                    inverterPeriodMin = INVERTERPERIOD60HZMIN;
                    inverterPeriodMax = INVERTERPERIOD60HZMAX;

                    // Delay at the zero crossings to allow AC voltage to reach flyback voltage
                    zeroCrossDelayNom = 20;
                    zeroCrossDelayMax = 22;			// Delay slightly varies with AC voltage
                    zeroCrossDelayMin = 18;

                    zeroCrossDelay = zeroCrossDelayNom;		// Wait (17us*delay) at the zero cross before turning on flyback/full-bridge
                }
                else
                {
                    inverterPeriodMin = INVERTERPERIOD50HZMIN;
                    inverterPeriodMax = INVERTERPERIOD50HZMAX;

                    // Delay at the zero crossings to allow AC voltage to reach flyback voltage
                    zeroCrossDelayNom = 30;
                    zeroCrossDelayMax = 35;			// Delay slightly varies with AC voltage
                    zeroCrossDelayMin = 25;

                    zeroCrossDelay = zeroCrossDelayNom;		// Wait (17us*delay) at the zero cross before turning on flyback/full-bridge
                }

                // Change system state to Day Mode
                systemState = DAYMODE;
            }
        }
        break;

        case DAYMODE:
        {
            // When average input voltage and average input current are available call MPPT Routine
            if(avgInputDataReadyFlag == 1)
            {
                #ifndef BENCHTESTING
                MPPTRoutine();			// Call MPPT Routine when data is ready
                #endif

                // Change Current Reference based on the PV panel voltage
                if(inputVoltageAverage >= PVPANEL_40V)
                {
                    CMPDAC2bits.CMREF = 700;			// Consider current at ~39V
                    CMPDAC3bits.CMREF = 700;
                }
                else if(inputVoltageAverage >= PVPANEL_30V)
                {
                    CMPDAC2bits.CMREF = 850;			// Consider current at ~29V
                    CMPDAC3bits.CMREF = 850;
                }
                else
                {
                    CMPDAC2bits.CMREF = 1000;			// Consider current at ~20V
                    CMPDAC3bits.CMREF = 1000;
                }

                avgInputDataReadyFlag = 0;
            }

            // Load Balance routine to make sure that both flyback converters are sharing the load equally ~50%
            // Execute at a slower rate 100us * LOADBALCOUNT

            if(loadBalanceCounter >= LOADBALCOUNT)
            {
                LoadBalance();				// Call Load Balance Routine
                loadBalanceCounter = 0;
            }

            loadBalanceCounter++;

            // Software for soft-start when using a bench supply as MPP-Tracking is removed
            #ifdef BENCHTESTING

            mpptCounter++;

            if(mpptFactor < MPPTFACTOR_BENCHTESTING)
            {
                if(mpptCounter >= MPPTCOUNT)
                {
                    mpptFactor = mpptFactor + MPPTFACTORINCREMENT;
                    mpptCounter = 0;
                }
            }
            else
            {
                mpptFactor = MPPTFACTOR_BENCHTESTING;
                mpptCounter = 0;
            }

            #endif

        }
        break;

        case SYSTEMERROR:
        {
            IOCON1bits.OVRENH = 1;
            IOCON1bits.OVRENL = 1;
            IOCON2bits.OVRENH = 1;
            IOCON2bits.OVRENL = 1;
            IOCON3bits.OVRENH = 1;
            IOCON3bits.OVRENL = 1;
            OPTO_DRV1 = 0;
            OPTO_DRV2 = 0;
            mpptStartUpFlag = 0;
            startFullBridgeFlag = 0;		// Reset flag to start full-bridge
            burstModeActiveCounter = 0;		// Reset burst mode
            burstModeActiveFlag = 0;
            acCurrentOffset = 0;
            acCurrentOffsetFlag = 0;		// Allow system to re-calculate AC Current Offset
            mpptFactor = MPPTFACTORMINIMUM;
            inputPower = 0;
            prevInputVoltageAverage = 0;

            ClrWdt();				// In fault mode we need to clear WDT

            if((switchState == SWITCHON) && (faultState == NO_FAULT))
            {
                // Switch to system startup after no faults have been detected for ~1s

                systemRestartCounter++;

                if(systemRestartCounter >= RESTARTCOUNT)
                {
                    systemState = SYSTEMSTARTUP;
                    systemRestartCounter = 0;
                }
            }
            else if((switchState == SWITCHON) && (faultState != NO_FAULT))
            {
                // If a fault is present then diplay the fault using T3 and LED D27 on the PCB
                systemRestartCounter = 0;
                storeFaultState = (faultState << 1);		// x2 to account for "off" time
                T3CONbits.TON = 1;

                // Remove the fault and allow system to try to restart
                // If the fault is ac current offset or HW Zero Cross
                if((faultState == ACCURRENT_OFFSET) || (faultState == HARDWAREZEROCROSS))
                {
                    faultState = NO_FAULT;
                }

                // Critical Faults: AC Current, Flyback Over Voltage, and Flyback Over Current
                // Handle faults differently: Allow system to try to restart only once, if fault
                // is still present then disable PWM module.
                if((criticalFaultFlag == 1) && (criticalFaultRestartFlag == 0))
                {
                    criticalFaultCounter++;

                    // After 2s remove the critical fault and allow system to try to restart
                    if(criticalFaultCounter > CRITICALFAULTCOUNT)
                    {
                        criticalFaultRestartFlag = 1;

                        // As the fault for flyback over current is latched the PWM needs to
                        // exit the latched fault mode in order to restart
                        if(faultState == FLYBACK_OVERCURRENT)
                        {
                            FCLCON1bits.FLTMOD = 3;		// Disable Fault Mode
                            FCLCON2bits.FLTMOD = 3;		// Disable Fault Mode
                            FCLCON1bits.FLTMOD = 0;		// Latched Fault Mode
                            FCLCON2bits.FLTMOD = 0;		// Latched Fault Mode
                        }

                        faultState = NO_FAULT;
                        criticalFaultCounter = 0;
                    }
                }
            }
            else if (switchState == SWITCHOFF)
            {
                // Reset Period Limits if switch is off
                // This is only required for testing 50/60 Hz operation
                // without having to cycle power
                inverterPeriodMin = INVERTERPERIOD60HZMIN;
                inverterPeriodMax = INVERTERPERIOD50HZMAX;
                T3CONbits.TON = 0;
                LED_DRV1 = OFF;
            }

        }
        break;
    }


    // Software for debugging purposes
    #ifdef DMCI_STATEMACHINE
    array1 [dmciArrayIndex] = deltaDutyCycle;
    array2 [dmciArrayIndex] = averageFlybackCurrent2 - averageFlybackCurrent1;
    array3 [dmciArrayIndex++] = 0;
    if(dmciArrayIndex >= 100)
    {
        dmciArrayIndex = 0;
    }
    #endif


    TMR2 = 0;
    IFS0bits.T2IF = 0;
}


// This MPPT algorithim implements the Perturbation and Observation method for detecting Maximum
// Power Point. MPPT can be up to ~25000 
void MPPTRoutine(void)
{
    int deltaV = 0, prevInputPower = 0;
    unsigned char mpptScaleFactor = 0;

    //Store off previous inputPower
    prevInputPower = inputPower;

    // Calculate new input power and change in input voltage
    inputPower = (__builtin_mulss((int)inputVoltageAverage ,(int)inputCurrentAverage) >> 15);
    deltaV = inputVoltageAverage - prevInputVoltageAverage;

    // If there is a large drop in voltage decrease mpptFactor at a faster rate
    // Example would be large AC voltage fluctuations
    if(deltaV <= PVPANEL_VOLTAGEDROP)
    {
        mpptFactor = mpptFactor - (mpptFactor >> 2);		// Reset to 75% power
    }

    // To find MPP faster, increase mpptFactor at a faster rate until the operating voltage
    // is less than the opencircuit voltage minus ~3V. Vmp and Voc should always be more
    // than 5-6V difference at any operating temperature or irradiance.
    if((inputVoltageAverage > (openCircuitVoltage - 1650)) && (mpptStartUpFlag == 0))
    {
        mpptFactor += 50;
    }
    else
    {
        mpptStartUpFlag = 1;
        criticalFaultFlag = 0;			// If system gets here without a critical fault, allow system to run as normal
        criticalFaultRestartFlag = 0;
    }


    if(burstModeActiveFlag == 1)
    {
        mpptScaleFactor = 1;
    }
    else
    {
        mpptScaleFactor = 0;
    }

    if (inputPower > prevInputPower)
    {
        if (deltaV < -LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor += MININCREMENTMPPTFACTOR;
        }
        else if (deltaV < 0)
        {
            mpptFactor += MAXINCREMENTMPPTFACTOR;
        }
        else if (deltaV > LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor -= (MAXDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV > 0)
        {
            mpptFactor -= (MINDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
    }
    else if (inputPower < prevInputPower)
    {
        if (deltaV < -LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor -= (MAXDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV < 0)
        {
            mpptFactor -= (MINDECREMENTMPPTFACTOR << mpptScaleFactor);
        }
        else if (deltaV > LARGEVOLTAGEDIFFERENCE)
        {
            mpptFactor += MAXINCREMENTMPPTFACTOR;
        }
        else if (deltaV > 0)
        {
            mpptFactor += MININCREMENTMPPTFACTOR;
        }
    }

    // Saturate the MPPT limit to min and max values
    if(mpptFactor > mpptFactorMaximum)
    {
        mpptFactor = mpptFactorMaximum;
    }
    else if(mpptFactor < MPPTFACTORMINIMUM)
    {
        mpptFactor = MPPTFACTORMINIMUM;
    }

    // Store off last known input power and input voltage
    prevInputVoltageAverage = inputVoltageAverage;

    // Software for debugging purposes
    #ifdef DMCI_MPPT
    array1 [dmciArrayIndex] = mpptFactor;
    array2 [dmciArrayIndex] = deltaV;
    array3 [dmciArrayIndex++] = inputPower;
    if(dmciArrayIndex >= 100)
    {
        dmciArrayIndex = 0;
    }
    #endif
}


void LoadBalance(void) 
{
    static int loadBalIoutput = 0;
    int diffFlybackCurrent = 0, loadBalPoutput = 0, loadBalPIoutput = 0;

    // Difference of the two Flyback MOSFET Currents
    diffFlybackCurrent = averageFlybackCurrent2 - averageFlybackCurrent1;

    // Error * Proportional Gain
    loadBalPoutput = ( (__builtin_mulss(diffFlybackCurrent,(int)KAQ15)) >> 15);

    // Error * Integral Gain
    loadBalIoutput = loadBalIoutput + ( (__builtin_mulss(diffFlybackCurrent,(int)KSAQ15)) >> 15);

    // Check for Integral term exceeding MAXBALANCE, If true, saturate the integral term to MAXBALANCE
    // Check for Integral term going below -MAXBALANCE, If true, saturate the integral term to -MAXBALANCE
    if(loadBalIoutput > MAXBALANCE)
    {
        loadBalIoutput = MAXBALANCE;
    }
    else if(loadBalIoutput < -MAXBALANCE)
    {
        loadBalIoutput = -MAXBALANCE;
    }

    // PI Output = Proportional Term + Integral Term
    loadBalPIoutput = loadBalPoutput + loadBalIoutput;

    // Check for PI Output exceeding MAXBALANCE, If true, saturate PI Output to MAXBALANCE
    // Check for PI Output going below -MAXBALANCE, If true, saturate PI Output to -MAXBALANCE
    if(loadBalPIoutput > MAXBALANCE)
    {
        loadBalPIoutput = MAXBALANCE;
    }
    else if(loadBalPIoutput < -MAXBALANCE)
    {
        loadBalPIoutput = -MAXBALANCE;
    }

    // Delta duty cycle to correct the two Flyback MOSFET duty cycles
    deltaDutyCycle = ( (__builtin_mulss((int)loadBalPIoutput,(int)FLYBACKPERIOD)) >> 15);
}
