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

#define ON 1
#define OFF 0

#define OPTO_DRV1 LATCbits.LATC0
#define OPTO_DRV2 LATCbits.LATC13

#define LED_DRV1 LATCbits.LATC12
#define LED_DRV2 LATCbits.LATC3
#define LED_DRV3 LATCbits.LATC8

#define FLYBACKPERIOD 16864  		 	 	// ~56kHz switching frequency (1/56kHz/1.06ns) = 16864 
#define FULLBRIDGEPERIOD (FLYBACKPERIOD >> 2)		// 228kHz switching frequency (1/228kHz/1.04ns) = 4216 
#define FLYBACKINTERLEAVEDPHASE (FLYBACKPERIOD >> 1)   	// 180Deg Phase-shift (flybackPeriod/2)

#define FLYBACKALTDTR 150				// Dead-time
#define FLYBACKDTR 50

#define FLYBACKDUTY1 PDC1                               // Re-define PWM Duty Cycle Registers
#define FLYBACKDUTY2 PDC2

#define MAXDUTYCLAMPED 24575				// Maximum duty cycle of 75% in Q15 format

// 180 degree is equal to 32767 
#define ONEHUNDREDSEVENTYFIVEDEGREE 31800		// Degree to turn off the Full-Bridge unfolding circuit					 
#define NINETYDEGREE 16383									 
				
// Vac Operating Voltage Limits
#define INVERTER_OVERVOLTAGE_LIMIT 14800		// Inverter Output Voltage ((264 * sqrt(2)) / turns ratio of TR1) * (gain of u6)
							// Converted into ADC counts and Q15 format	minus the offset	
#define INVERTER_OVERVOLTAGE_LIMIT_HYS 14400 		// Inverter Output Over Voltage Hysteresis ~3Vac RMS

#define INVERTER_UNDERVOLTAGE_LIMIT 11400		// Inverter Output Voltage ((210 * sqrt(2)) / turns ratio of TR1) * (gain of u6)
							// Converted into ADC counts and Q15 format minus the offset		
#define INVERTER_UNDERVOLTAGE_LIMIT_HYS 11700		// Inverter Output Under Voltage Hysteresis ~3Vac RMS

// PV Panel Operating Voltage Limits
#define PVPANEL_MPP_LIMIT 26900				// Max MPP Voltage ~46Vdc

#define PVPANEL_OVERVOLTAGE_LIMIT 30370			// ~52V open circuit voltage, 52 * (R74/(R74+R72))
							// Converted into ADC counts and Q15 format
#define PVPANEL_OVERVOLTAGE_LIMIT_HYS 28900		// PV Panel Over Voltage Hysteresis ~2V 	

#define PVPANEL_UNDERVOLTAGE_LIMIT 10900 		// Minimum Operating Voltage ~20Vdc, 18.5 * (R74/(R74+R72))
							// Converted into ADC counts and Q15 format		
#define PVPANEL_UNDERVOLTAGE_LIMIT_HYS 12300		// PV Panel Under Voltage Hysteresis ~2V

// Inverter Output Current Max Rating
#define INVERTER_OUTPUTCURRENT_MAX 13000		// At 210Vac full load, expected peak inverter output current ~1.6A.
							// This current * gain of 185mV/A * gain of U5, measured by the ADC 
							// and converted to Q15. Subtract off the offset 
  
// AC Current Offset Limits 
#define MINOFFSETCURRENT 12896				// Nominal ~1.6V, convert to ADC and Q15
#define MAXOFFSETCURRENT 17856		

// Maximum Output Power Limit
#define DERATINGFACTOR 39600								
#define DERATINGSLOPE 25000//25673			// 1.567 in Q14 format
#define DERATINGCONSTANT 18235				// 1.113 in Q14 format

// Power De-rating Limits
#define POWERDERATINGLIMIT 14300			// Corresponds to 24.5Vdc

// Power Decrement Factor for anti-islanding 
#define POWERDECREMENTFACTOR Q15(.02)			// This is ~2% which is an initial power de-rating 	

// Maximum Average Flyback Current Limit
#define MAXFLYBACKCURRENT 29000				// Max average current through flyback stage ~3V on ADC

// 12V Drive Supply Operating Limits
#define MAXDRIVEVOLTAGE 30766				// 12.5V drive supply * (R2/(R2+R1)), convert to ADC and then Q15
#define MINDRIVEVOLTAGE	28300				// 11.5V in Q15 format

// Temperature Operating Limits
#define MAXTEMPERATURE 		12400			// Vout = (75C * 10mV/C) + .5V = 1.25V, Convert to ADC reading then Q15
#define MAXTEMPERATURE_HYS 	11400			// 65C degrees, Convert to ADC reading and then Q15

// 2.5V Operating Limits
#define MINREFERENCEVOLTAGE	15744			// 2.4V * (R141/(R141+R128)), convert to ADC and then convert to Q15
#define MAXREFERENCEVOLTAGE	17056			// 2.6V in Q15 format

// 50/60Hz Operating Limits
#define INVERTERPERIOD50HZMIN 528			// Frequency set points for 50Hz (47Hz - 53Hz), ADC ISR rate is 17.87us
#define INVERTERPERIOD50HZMAX 595							
#define INVERTERPERIOD60HZMIN 461			// Frequency set points for 60Hz (59.3Hz - 60.5Hz), ADC ISR rate is 17.87us
#define INVERTERPERIOD60HZMAX 473							
#define INVERTERPERIODHYS 3				// Hysteresis for Inverter frequency


// MPPT Definitions
#define MPPTFACTORMINIMUM 700				// Minimum MPPT Factor to start the inverter
#define MININCREMENTMPPTFACTOR 20			// Values should be small enough to avoid large voltage
#define MAXINCREMENTMPPTFACTOR 30			// deviations when operating at MPP
#define MINDECREMENTMPPTFACTOR 20							
#define MAXDECREMENTMPPTFACTOR 30
#define LARGEVOLTAGEDIFFERENCE	8

// Burst Mode Definitions
#define BURSTMODECOUNT	6700				// Burst Mode Count (~1 minute) - 1/(2xVac) * counter
#define BURSTMODETHRESHOLDLOW	1600			// Power to which burst mode is applied < ~15% 
#define BURSTMODETHRESHOLDHIGH	2800			// Power to which burst mode is removed ~18%

// Various #defines
#define ZEROCROSSCOUNT		60			// Number of zero cross events required before switching to Day Mode
#define RESTARTCOUNT		10000			// 10000 * 100us - 1s of no faults to switch to system startup
#define CRITICALFAULTCOUNT 	20000			// 20000 * 100us - 2s before trying to restart the system
#define LOADBALCOUNT		2			// Rate to execute the load balance routine in the state machine (Day mode) 1/10th main compensator
#define PVPANEL_40V		23341			// Panel Voltage 40V, used for changing CMP current reference
#define PVPANEL_30V		17500			// Panel Voltage 30V, used for changing CMP current reference
#define PVPANEL_VOLTAGEDROP	-600			// Change in PV panel average voltage due to large AC Voltage Change
							// PV Voltage change of ~0.5V * (7.5/127.5), convert to ADC and Q15


// Coefficients for PI Controller
#define Ra Q15(0.18) 
#define Rsa Q15(0.02)

// Coefficients for Load Balancing 
#define MAXBALANCE Q15(0.01)							
#define KAQ15 Q15(0.065)
#define KSAQ15 Q15(0.01)

// System State Definitions
#define SYSTEMSTARTUP 0
#define DAYMODE 1
#define SYSTEMERROR 2

// Input Switch State
#define SWITCHOFF 0
#define SWITCHON 1

// Fault State Definitions
#define NO_FAULT 0
#define PV_PANEL_VOLTAGE 1
#define INVERTER_FREQUENCY 2	
#define INVERTER_VOLTAGE 3
#define INVERTER_OVERCURRENT 4
#define FLYBACK_OVERCURRENT 5
#define TEMPERATURE 6
#define DRIVE_SUPPLY 7
#define FLYBACK_OUTPUT_VOLTAGE 8
#define REFERENCE_VOLTAGE 9
#define ACCURRENT_OFFSET 10
#define HARDWAREZEROCROSS 11

// State for Full Bridge Drive
#define FULLBRIDGE_Q3Q4_ACTIVE 1
#define FULLBRIDGE_INACTIVE_2ND_QUADRANT 2
#define FULLBRIDGE_Q2Q5_ACTIVE 3
#define FULLBRIDGE_INACTIVE_4TH_QUADRANT 4


// Include the following #define in the source code to run system on the bench using a DC power supply. 
// This will remove MPP-tracking so we need to define MPPTFactorMaximum ~25000 (max power) and 
// create another way for a controlled softstart

//#define BENCHTESTING

#ifdef BENCHTESTING
#define MPPTFACTOR_BENCHTESTING 9900
#define MPPTFACTORINCREMENT 20		// Softstart increments MPPT by MPPTFACTORINCREMENT every 10ms
#define MPPTCOUNT	100		// Slow down the increment rate to -> 100 * 100us
#endif


// Include only ONE of the following #defines to use any of three arrays (array1, array2, and array3) for DMCI debug purposes
// DMCI_ISR is at a rate of 17us * (dmciCounter)
// DMCI_STATEMACHINE is at a rate of 100us
// DMCI_MPPT is at a rate of 3*inverterFrequency
 
//#define DMCI_ISR
//#define DMCI_STATEMACHINE
//#define DMCI_MPPT


