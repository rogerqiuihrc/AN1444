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

#include "Solar Microinverter_isr.h" 
	
// System Faults
unsigned char criticalFaultFlag = 0, inverterFrequencyErrorFlag = 0, frequencyFaultCounter = 0;
int maxInverterOutputVoltage = 0, rectifiedInverterOutputVoltage = 0;

// Compensator Variables
int rectifiedInverterOutputCurrent = 0, peakInverterOutputVoltage = 0;
long int Ioutput = 0;
unsigned char fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

// Inverter Current Reference Variables
unsigned char ninetyDegreeDetectFlag = 0;
unsigned int globalAngle = 0, sineAngle = 0, deltaAngle = 0;
unsigned int currentReferenceDynamic = 0;

// ADC Variables
unsigned int pvPanelVoltage = 0, flybackCurrent1 = 0, flybackCurrent2 = 0;
int inverterOutputVoltage = 0, inverterOutputCurrent = 0, prevInverterOutputVoltage = 0;
unsigned int driveSupplyVoltage = 0, measuredTemperature = 0, referenceVoltage = 0;

// Flyback Current Moving Average Variables/Arrays
unsigned char currentArrayCnt = 0;
unsigned int averageFlybackCurrent1 = 0, averageFlybackCurrent2 = 0;
unsigned int flybackCurrent1Array[8] = {0,0,0,0,0,0,0,0};
unsigned int flybackCurrent2Array[8] = {0,0,0,0,0,0,0,0};
long unsigned int flybackCurrent1Sum = 0, flybackCurrent2Sum = 0;

// AC Current Variables and Moving Avg Variables/Array
unsigned int averageRectifiedCurrent = 0, maxInverterOutputCurrent = 0; 
unsigned int rectifiedInverterOutputCurrentArray[8] = {0,0,0,0,0,0,0,0};
long unsigned int rectifiedInverterOutputCurrentSum = 0;

// Peak Power and Burst Mode Variables
unsigned int averagePeakOutputPower = 0, peakInverterOutputCurrent = 0, peakOutputPower = 0;
unsigned char peakOutputPowerArrayCnt = 0;
unsigned int peakOutputPowerArray[8] = {0,0,0,0,0,0,0,0};
long unsigned int peakOutputPowerSum = 0;
unsigned char burstModeActiveFlag = 0;
unsigned int burstModeActiveCounter = 0;

// Zero Cross Variables
unsigned char zeroCrossDetectFlag = 0, zeroCrossCount = 0, firstQuadrantFlag = 0;
unsigned char thirdQuadrantFlag = 0, avgInputDataReadyFlag = 0;
unsigned char zcCounter = 0, startupZeroCrossCounter = 0;
unsigned char hardwareZeroCrossCounter = 0, prevFullBridgeState = 0;
unsigned int inverterPeriod = 0, inverterPeriodCounter = 0, prevInverterPeriod = 0, numberofSamples = 0;

// MPPT Variables
unsigned int mpptFactorMaximum = 0, mpptDeratingFactor = 0;
long unsigned int inputCurrentSum = 0, inputVoltageSum = 0;

// Externally Defined Variables
extern unsigned char switchState, systemState, faultState, zeroCrossDelay;
extern unsigned char inverterFrequencyState, criticalFaultRestartFlag, startFullBridgeFlag;
extern unsigned char zeroCrossDelayNom, zeroCrossDelayMin, zeroCrossDelayMax;
extern unsigned int mpptFactor, inputVoltageAverage, inputCurrentAverage;
extern unsigned int inverterPeriodMin, inverterPeriodMax, inverterOutputOverCurrent;
extern int deltaDutyCycle, acCurrentOffset;


// Variable Declaration for DMCI Debugging Tool
#ifdef DMCI_ISR
int array1[100];
int array2[100];
int array3[100];
unsigned char dmciArrayIndex = 0;
unsigned int dmciCount = 0;
#endif


void __attribute__((interrupt, no_auto_psv)) _CMP2Interrupt()
{
    if((criticalFaultFlag == 1) && (criticalFaultRestartFlag == 1))
    {
        PTCONbits.PTEN = 0;
        RCONbits.SWDTEN = 0;		// Need to disable WDT
    }

    // As PWM is latched we need to set the faultState to Flyback overcurrent no matter what
    // so that the latched fault can be removed
    criticalFaultFlag = 1;
    faultState = FLYBACK_OVERCURRENT;

    IFS6bits.AC2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _CMP3Interrupt()
{
    if((criticalFaultFlag == 1) && (criticalFaultRestartFlag == 1))
    {
        PTCONbits.PTEN = 0;
        RCONbits.SWDTEN = 0;		// Need to disable WDT
    }

    // As PWM is latched we need to set the faultState to Flyback overcurrent no matter what
    // so that the latched fault can be removed
    criticalFaultFlag = 1;
    faultState = FLYBACK_OVERCURRENT;

    IFS6bits.AC3IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _CMP4Interrupt()
{
    // Flyback Output Over Voltage Condition used only for protection against
    // Full-Bridge failure (in normal operating conditions this event should never occur)

    // Over-ride the flyback MOSFETs immediately
    IOCON1bits.OVRENH = 1;
    IOCON1bits.OVRENL = 1;
    IOCON2bits.OVRENH = 1;
    IOCON2bits.OVRENL = 1;

    if((criticalFaultFlag == 1) && (criticalFaultRestartFlag == 1))
    {
        PTCONbits.PTEN = 0;
        RCONbits.SWDTEN = 0;		// Need to disable WDT
    }

    if(faultState == NO_FAULT)
    {
        criticalFaultFlag = 1;
        faultState = FLYBACK_OUTPUT_VOLTAGE;
    }

    IFS6bits.AC4IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _ADCP3Interrupt()
{		
    // Read Flyback Current, PV Voltage, and 2.5V Reference in Q15 Format
    pvPanelVoltage  = (ADCBUF0 << 5); 		// Read PV Panel Voltage
    flybackCurrent1 = (ADCBUF1 << 5); 		// Read PV cell Current at Flyback leg1
    flybackCurrent2 = (ADCBUF2 << 5); 		// Read PV cell Current at Flyback leg2 
    referenceVoltage = (ADCBUF6 << 5);		// Read 2.5V Reference Voltage (used for AC offset)

    // Read inverter output voltage and inverter output current (Q14)
    inverterOutputCurrent = (ADCBUF3 << 5) - acCurrentOffset;
    inverterOutputVoltage = (ADCBUF7 << 5) - referenceVoltage;

    // Rectified Inverter Output Voltage
    if(inverterOutputVoltage >= 0)
    {
        rectifiedInverterOutputVoltage = inverterOutputVoltage;
    }
    else
    {
        rectifiedInverterOutputVoltage = (-inverterOutputVoltage);
    }

    // Find Peak Inverter Output Voltage
    if(rectifiedInverterOutputVoltage > maxInverterOutputVoltage)
    {
        maxInverterOutputVoltage = rectifiedInverterOutputVoltage;
    }

    // Rectify AC current and check for over current condition on the ouput
    if(inverterOutputCurrent >= 0)
    {
        rectifiedInverterOutputCurrent = inverterOutputCurrent;
    }
    else
    {
        rectifiedInverterOutputCurrent = (-inverterOutputCurrent);
    }

    // Find Peak Inverter Output Current
    if(rectifiedInverterOutputCurrent > maxInverterOutputCurrent)
    {
        maxInverterOutputCurrent = rectifiedInverterOutputCurrent;
    }


    // Moving Average of Flyback Currents for load sharing
    flybackCurrent1Sum = flybackCurrent1Sum + flybackCurrent1 - flybackCurrent1Array[currentArrayCnt];
    averageFlybackCurrent1 = flybackCurrent1Sum >> 3;

    flybackCurrent2Sum = flybackCurrent2Sum + flybackCurrent2 - flybackCurrent2Array[currentArrayCnt];
    averageFlybackCurrent2 = flybackCurrent2Sum >> 3;

    flybackCurrent1Array[currentArrayCnt] = flybackCurrent1;
    flybackCurrent2Array[currentArrayCnt] = flybackCurrent2;

    // Moving Average of AC Current for AC Current Fault
    rectifiedInverterOutputCurrentSum = rectifiedInverterOutputCurrentSum + rectifiedInverterOutputCurrent - rectifiedInverterOutputCurrentArray[currentArrayCnt];
    averageRectifiedCurrent = rectifiedInverterOutputCurrentSum >> 3;

    rectifiedInverterOutputCurrentArray[currentArrayCnt++] = rectifiedInverterOutputCurrent;

    if(currentArrayCnt >= 8)
    {
        currentArrayCnt = 0;
    }

    // Check for Over Current Condition
    if((averageFlybackCurrent1 > MAXFLYBACKCURRENT)||(averageFlybackCurrent2 > MAXFLYBACKCURRENT))
    {
        if(faultState == NO_FAULT)
        {
            faultState = FLYBACK_OVERCURRENT;
        }
    }

    // Inverter Over Current Fault Check
    // inverterOutputOverCurrent takes into account the delta in the acCurrentOffset
    if((averageRectifiedCurrent > inverterOutputOverCurrent) && (systemState == DAYMODE))
    {
        if((criticalFaultFlag == 1) && (criticalFaultRestartFlag == 1))
        {
            PTCONbits.PTEN = 0;
            RCONbits.SWDTEN = 0;	// Need to disable WDT
        }

        if(faultState == NO_FAULT)
        {
            criticalFaultFlag = 1;
            faultState = INVERTER_OVERCURRENT;
        }
    }

    // Call zero cross detection function to look for both +ve and -ve zero cross events
    // Routine also keeps track of the number of ADC interrupts over three AC cycles
    // to calculate the average flyback current and PV panel voltage for MPPT

    // To eliminate glitches at the ZC detection only look for a zero cross
    // when inverterPeriodCounter is greater then 200

    if((zeroCrossDetectFlag == 0) && (inverterPeriodCounter > 200))
    {
        zeroCrossDetection();
    }

    // Store current grid voltage to compare with next sample for ZCD
    prevInverterOutputVoltage = inverterOutputVoltage;


    // Counter for verifying the Grid Frequency (number of ADC interrupts per grid half cycle)
    // variable gets reset when finding the zero crossing event
    inverterPeriodCounter++;

    // Accumulate samples for the Average Calculation
    inputCurrentSum = inputCurrentSum + flybackCurrent1 + flybackCurrent2;
    inputVoltageSum = inputVoltageSum + pvPanelVoltage;


    // ZeroCrossDetectFlag is set inside the zeroCrossDetection Routine
    if(zeroCrossDetectFlag == 1)
    {
        zeroCrossDetectFlag = 0;
        startupZeroCrossCounter++;

        if (systemState != SYSTEMSTARTUP)
        {
            startupZeroCrossCounter = 0;
        }

        // Continuously Verify that Grid Frequency is okay
        inverterFrequencyCheck();


        // deltaAngle calculated based on present grid frequency/period measured
        deltaAngle = __builtin_divsd((long)32767,inverterPeriod);

        // Load the newly found Peak Voltage/Current of the Inverter Output (done @ zero crossing)
        peakInverterOutputVoltage = maxInverterOutputVoltage;
        peakInverterOutputCurrent = maxInverterOutputCurrent;
        maxInverterOutputVoltage = 0;
        maxInverterOutputCurrent = 0;

        //Change ZeroCrossDelay based on peak AC Voltage
        if(peakInverterOutputVoltage > 13800)
        {
            zeroCrossDelay = zeroCrossDelayMax;
        }
        else if (peakInverterOutputVoltage < 12000)
        {
            zeroCrossDelay = zeroCrossDelayMin;
        }
        else
        {
            zeroCrossDelay = zeroCrossDelayNom;
        }


        // Maximum MPPT calculation is based on the grid peak voltage
        // mpptFacorMaximum = -(DERATINGSLOPE * peakInverterOutputVoltage) + DERATINGFACTOR
        mpptFactorMaximum = DERATINGFACTOR - ((__builtin_mulss(peakInverterOutputVoltage, DERATINGSLOPE)) >> 14);


        // Power de-rating routine ~7W per Volt when operating below 25Vdc
        if((inputVoltageAverage < POWERDERATINGLIMIT) && (systemState == DAYMODE))
        {
            // Determine MPPT Derating Factor (1V => 584 counts in Q15, 7W => 650 in Q15)
            // Scale voltage properly
            mpptDeratingFactor = POWERDERATINGLIMIT - inputVoltageAverage;
            mpptDeratingFactor = ((__builtin_mulss(mpptDeratingFactor,DERATINGCONSTANT))>>14);

            // Determine New MPPT Factor Maximum
            mpptFactorMaximum = mpptFactorMaximum - mpptDeratingFactor;
        }

        peakOutputPower = ((__builtin_mulss(peakInverterOutputVoltage, peakInverterOutputCurrent)) >> 14);

        // Moving Average for peak power
        peakOutputPowerSum = peakOutputPowerSum + peakOutputPower - peakOutputPowerArray[peakOutputPowerArrayCnt];
        averagePeakOutputPower = peakOutputPowerSum >> 3;
        peakOutputPowerArray[peakOutputPowerArrayCnt++] = peakOutputPower;

        if(peakOutputPowerArrayCnt >= 8)
        {
            peakOutputPowerArrayCnt = 0;
        }

        // Burst Mode Detection (Output Power < ~15% Max Rated Power)
        if((averagePeakOutputPower <= BURSTMODETHRESHOLDLOW) && (burstModeActiveFlag == 0))
        {
            burstModeActiveCounter++;

            if(burstModeActiveCounter >= BURSTMODECOUNT)
            {
                burstModeActiveFlag = 1;
                burstModeActiveCounter = 0;
                mpptFactor = mpptFactor + (mpptFactor << 1);
            }
        }
        else if ((burstModeActiveFlag == 1) && (averagePeakOutputPower > BURSTMODETHRESHOLDHIGH))
        {
            mpptFactor = (mpptFactor >> 1);
            burstModeActiveFlag = 0;
        }
        else
        {
            burstModeActiveCounter = 0;
        }

        // Reset Variables every Zero Cross
        sineAngle = 0;
        globalAngle = 0;
        ninetyDegreeDetectFlag = 0;
    }


    // This portion of SW determines the current reference from the sine lookup table based on
    // deltaAngle which is calculated every Zero Cross
    // sineAngle is used to point to specific location in sine lookup table
    if(ninetyDegreeDetectFlag == 0)
    {
        // if sineAngle is less than 90 degrees, then add deltaAngle to point to next sample
        if (sineAngle < NINETYDEGREE)
        {
            currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
            sineAngle = sineAngle + deltaAngle;
        }
        else
        {
            sineAngle = NINETYDEGREE;
            ninetyDegreeDetectFlag = 1;
        }
    }
    else
    {
        // if sineAngle is greater than 90 degrees, then subtract deltaAngle from it
        currentReferenceDynamic = sineTable512[((sineAngle) >> 5)];
        sineAngle = sineAngle - deltaAngle;
    }

    // globalAngle is used for detecting grid voltage states
    globalAngle = globalAngle + deltaAngle;

	
    // startFullBridgeFlag enabled in system startup when several zero crosses have been detected
    // Flag is disabled when system enter fault mode
    if(startFullBridgeFlag == 1)
    {
        // Routine that determines the state of the Full-Bridge MOSFETs
        fullBridgeDrive();
    }

    // In Day mode Enable/Disbale the PWM drive for the flyback MOSFETs
    if(systemState == DAYMODE)
    {
        if ((fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE) || (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE))
        {
            // If in Burst Mode, only deliver power for one complete AC cycle
            // If not in Burst Mode, remove the PWM override when full-bridge state is active
            if(burstModeActiveFlag == 0)
            {
                IOCON1bits.OVRENH = 0;		/* Remove Override  */
                IOCON2bits.OVRENH = 0;
                IOCON1bits.OVRENL = 0;
                IOCON2bits.OVRENL = 0;

                flybackControlLoop();
            }
            else if ((burstModeActiveFlag == 1) && (zeroCrossCount == 0))
            {
                IOCON1bits.OVRENH = 0;		/* Remove Override  */
                IOCON2bits.OVRENH = 0;
                IOCON1bits.OVRENL = 0;
                IOCON2bits.OVRENL = 0;

                flybackControlLoop();
            }
        }
        else
        {
            IOCON1bits.OVRENH = 1;		/* Enable Override */
            IOCON1bits.OVRENL = 1;
            IOCON2bits.OVRENH = 1;
            IOCON2bits.OVRENL = 1;

            // Give some history to the compensator before starting again at the zero cross
            if(inverterPeriodMin == INVERTERPERIOD50HZMIN)
            {
                Ioutput = -1900;

                if(peakInverterOutputVoltage > 13800)
                {
                    Ioutput = -2500;
                }
            }
            else
            {
                Ioutput = -500;//-1000;
            }
        }

    }

    //Can read AN10 and AN11 here data should be available at this time
    driveSupplyVoltage = (ADCBUF11 << 5);
    measuredTemperature = (ADCBUF10 << 5);


    // Software for debugging purposes
    #ifdef DMCI_ISR
    if(dmciCount == 16)
    {
        array1 [dmciArrayIndex] = 0;
        array2 [dmciArrayIndex] = 0;
        array3 [dmciArrayIndex++] = 0;
        if(dmciArrayIndex >= 100)
        {
            dmciArrayIndex = 0;
        }
        dmciCount = 0;
    }

    dmciCount++;

    #endif

    IFS7bits.ADCP3IF = 0;                   // Clear ADC Interrupt Flag
}

void zeroCrossDetection(void)
{
    // Detect the zero crossing at the 1st Quadrant
    if((prevInverterOutputVoltage < 0) && (inverterOutputVoltage >= 0))
    {
        // As a precaution change state of Full-Bridge if it hasn't already chaged
        fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;

        ClrWdt();

        zeroCrossDetectFlag = 1;
        firstQuadrantFlag = 1;				// Allows Full-Bridge to change State

        // Load counter to Grid Period to check grid frequency
        inverterPeriod = inverterPeriodCounter;
        inverterPeriodCounter = 0;

        // Store accumulated voltage/current sum and counter for avg (avg calculated in T2ISR)
        // The average is calculated every 3rd +ve zero cross event
        zeroCrossCount++;

        if (zeroCrossCount == 1)
        {
            numberofSamples = 0;
        }

        // inverterPeriod is only half cycle so add prevInverterPeriod
        numberofSamples = numberofSamples + inverterPeriod + prevInverterPeriod;

        if(zeroCrossCount >= 3)
        {
            zeroCrossCount = 0;

            inputVoltageAverage = __builtin_divsd((long)inputVoltageSum ,(int)(numberofSamples));
            inputCurrentAverage = __builtin_divsd((long)inputCurrentSum ,(int)(numberofSamples));
            numberofSamples = 0;
            inputVoltageSum = 0;
            inputCurrentSum = 0;

            avgInputDataReadyFlag = 1;
        }
    }

    // Detect the zero crossing at the 3rd Quadrant
    else if ((prevInverterOutputVoltage >= 0) && (inverterOutputVoltage < 0))
    {
        // As a precaution change state of Full-Bridge if it hasn't already chaged
        fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;

        ClrWdt();

        zeroCrossDetectFlag = 1;
        thirdQuadrantFlag = 1;			// Allows Full-Bridge to change State

        // Load counter to Grid Period to check grid frequency
        inverterPeriod = inverterPeriodCounter;
        inverterPeriodCounter = 0;
    }
}

void inverterFrequencyCheck(void)
{
    if ((inverterPeriod > inverterPeriodMin ) && (inverterPeriod < inverterPeriodMax))
    {
        if(inverterFrequencyErrorFlag == 1)
        {
            // Hysteresis for the frequency once a fault has been detected
            if((inverterPeriod > (inverterPeriodMin + INVERTERPERIODHYS)) && (inverterPeriod < (inverterPeriodMax - INVERTERPERIODHYS)))
            {
                if(faultState == INVERTER_FREQUENCY)
                {
                    faultState = NO_FAULT;
                }

                inverterFrequencyErrorFlag = 0;
            }
        }

        frequencyFaultCounter = 0;
    }
    else
    {
        frequencyFaultCounter++;

        if(frequencyFaultCounter >= 2)
        {
            inverterFrequencyErrorFlag = 1;
            frequencyFaultCounter = 2;

            if(faultState == NO_FAULT)
            {
                faultState = INVERTER_FREQUENCY;
            }
        }
    }

    prevInverterPeriod = inverterPeriod;	// Store Inverter Period for average calculation
}

void fullBridgeDrive(void)
{
    // Based on where we are in the sinewave enable/disable PWM3 Gate Drive (Full-Bridge)

    switch(fullBridgeState)
    {
        case FULLBRIDGE_Q3Q4_ACTIVE:
        {
            if(globalAngle < NINETYDEGREE)
            {
                prevFullBridgeState = PORTBbits.RB15;
            }

            if (globalAngle > ONEHUNDREDSEVENTYFIVEDEGREE)
            {
                fullBridgeState = FULLBRIDGE_INACTIVE_2ND_QUADRANT;
            }
        }
        break;

        case FULLBRIDGE_INACTIVE_2ND_QUADRANT:
        {
            // Check for Hardware/Software Zero Cross
            if((PORTBbits.RB15 == 0) && (thirdQuadrantFlag == 1))
            {
                zcCounter++;

                if(zcCounter >= zeroCrossDelay)
                {
                    if((prevFullBridgeState == PORTBbits.RB15) && (systemState == DAYMODE))
                    {
                        if(faultState == NO_FAULT)
                        {
                            faultState = HARDWAREZEROCROSS;
                        }
                    }

                    zcCounter = 0;
                    thirdQuadrantFlag = 0;
                    fullBridgeState = FULLBRIDGE_Q2Q5_ACTIVE;
                }
            }
            else
            {
                zcCounter = 0;
                break;
            }
        }
        break;

        case FULLBRIDGE_Q2Q5_ACTIVE:
        {
            if(globalAngle < NINETYDEGREE)
            {
                prevFullBridgeState = PORTBbits.RB15;
            }

            if (globalAngle > ONEHUNDREDSEVENTYFIVEDEGREE)
            {
                fullBridgeState = FULLBRIDGE_INACTIVE_4TH_QUADRANT;
            }
        }
        break;

        case FULLBRIDGE_INACTIVE_4TH_QUADRANT:
        {
            // Check for Hardware/Software Zero Cross
            if ((PORTBbits.RB15 == 1) && (firstQuadrantFlag == 1))
            {
                zcCounter++;

                if(zcCounter >= zeroCrossDelay)
                {
                    if((prevFullBridgeState == PORTBbits.RB15) && (systemState == DAYMODE))
                    {
                        if(faultState == NO_FAULT)
                        {
                            faultState = HARDWAREZEROCROSS;
                        }
                    }

                    zcCounter = 0;
                    firstQuadrantFlag = 0;
                    fullBridgeState = FULLBRIDGE_Q3Q4_ACTIVE;
                }
            }
            else
            {
                zcCounter = 0;
                break;
            }
        }
        break;
    }

    // Now that we know the state we can modify the PWM outputs

    if(fullBridgeState == FULLBRIDGE_Q3Q4_ACTIVE)
    {
        OPTO_DRV1 = 1;
        OPTO_DRV2 = 0;
        IOCON3bits.OVRENL = 0;
    }
    else if (fullBridgeState == FULLBRIDGE_INACTIVE_2ND_QUADRANT)
    {
        IOCON3bits.OVRENH = 1;
        IOCON3bits.OVRENL = 1;
        OPTO_DRV2 = 1;
        OPTO_DRV1 = 1;
    }
    else if (fullBridgeState == FULLBRIDGE_Q2Q5_ACTIVE)
    {
        OPTO_DRV2 = 1;
        OPTO_DRV1 = 0;
        IOCON3bits.OVRENH = 0;
    }
    else
    {
        IOCON3bits.OVRENH = 1;
        IOCON3bits.OVRENL = 1;
        OPTO_DRV2 = 1;
        OPTO_DRV1 = 1;
    }
}


void flybackControlLoop(void)
{
    unsigned int flybackDutyCycle = 0;
    unsigned int rectifiedVac = 0, decoupleTerm = 0;
    int IoRef = 0, currentError = 0, Poutput = 0;
    long int totalOutput = 0;

    // Compensator Software 
    IoRef = (__builtin_mulss((int)currentReferenceDynamic,(int)mpptFactor) >> 15);
    currentError = IoRef - (rectifiedInverterOutputCurrent << 1);

    Poutput = (__builtin_mulss(currentError,Ra)) >> 15;

    Ioutput  =  Ioutput + (long)((__builtin_mulss((int)currentError,(int)Rsa)) >> 15);

    if(Ioutput > 32767)
    {
        Ioutput = 32767;
    }
    else if(Ioutput < -32767)
    {
        Ioutput = -32767;
    }

    totalOutput = (long)Poutput + Ioutput;

    // Input/Output Voltage Decoupling Software (Vo/(Vin + Vo))
    // rectifiedVac is in Q13 format
    rectifiedVac = (__builtin_mulss((int)currentReferenceDynamic,(int)peakInverterOutputVoltage) >> 16);

    // pvPanel Voltage is multiplied by Q15(0.89) to have the same per unit as the rectifiedVac
    // and both are in Q13 format (56.1 * turns ratio = 392.7, 392/445 = 0.89)
    decoupleTerm = rectifiedVac + (__builtin_mulss(29163,(int)pvPanelVoltage) >> 17);

    // Divide Q26/Q13, result is in Q13
    decoupleTerm = __builtin_divsd(((long)rectifiedVac << 13) ,decoupleTerm);

    // Total Compensator Output, decoupleTerm is scaled back to Q15 as totalOutput is clamped to Q15
    totalOutput = totalOutput + (long)(decoupleTerm << 2);

    // Clamp total output to the maximum Duty Cycle
    if(totalOutput > MAXDUTYCLAMPED)
    {
        totalOutput = MAXDUTYCLAMPED;
    }
    else if(totalOutput < 0)
    {
        totalOutput = 0;
    }

    // Multiply flyback period by total output to get the flyback duty cycle
    flybackDutyCycle = (__builtin_mulss((int)totalOutput,(int)FLYBACKPERIOD) >> 15);

    // If duty cycle is less than dead-time make duty cycle equal to dead-time
    if(flybackDutyCycle < FLYBACKALTDTR)
    {
        FLYBACKDUTY1 = FLYBACKALTDTR;
        FLYBACKDUTY2 = FLYBACKALTDTR;
    }
    else
    {
        FLYBACKDUTY1 = flybackDutyCycle + deltaDutyCycle ;
        FLYBACKDUTY2 = flybackDutyCycle - deltaDutyCycle ;
    }

    // Update ADC Triggers (trigger near end of PWM On-time)
    TRIG1 = FLYBACKDUTY1 - (FLYBACKDUTY1 >> 2);
    TRIG2 = FLYBACKDUTY2 - (FLYBACKDUTY2 >> 2);
}
