//#define INCLUDE_TEST_HARNESS
/****************************************************************************
 Module
     PIC32_AD_Lib.c

 Description
     This is a module implementing  the basic functions to use the A/D 
	 converter on the PIC32MX170F256B
 Notes


 History
 When           Who     What/Why
 -------------- ---     --------

 11/03/20 14:55 jec     cleaned up typos and clarified a comment
 10/27/20 16:10 jec     cleaned up the documentation to meet SPDL Standards
 10/20/20 16:38 jec     Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "PIC32_AD_Lib.h"
#include <xc.h>
#include <stdbool.h>

/*--------------------------- External Variables --------------------------*/

/*----------------------------- Module Defines ----------------------------*/
//  We configure the A/D to use 2 8-word buffers, so it is limited to 
//  converting 8 channels. 
#define MAX_CHANNELS 8
/*------------------------------ Module Types -----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
static uint8_t CountBits(uint16_t num2Count);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t numChanInSet;   // used to iterate in read requests

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    ADC_ConfigAutoScan
 Parameters
    uint16_t whichPins specifies which of the ANx pins will be converted
		a 1 in a bit position indicates that that ANx channel is to be
		converted e.g: to convert on AN0, set bit 0
    
 Returns
    bool true if no errors
 Description
    configures the A/D converter subsystem for auto-sampling on a set of pins
 Notes
    This configures the A/D to use 2 8-word buffers, so it is limited to 
    converting 8 channels.
 Author
     J. Edward Carryer, 10/20/20 15:49
****************************************************************************/
bool ADC_ConfigAutoScan( uint16_t whichPins){
  bool returnVal = true; // assume no errors

  // first test to be sure that the number of 1s in whichPins
  // is less than the maximum allowed number of channels
  if (MAX_CHANNELS < (numChanInSet = CountBits(whichPins)))
  {
    returnVal = false;
  }else // OK, looks good
  {
    AD1CON1bits.ON = 0; // disable ADC
 
    // AD1CON1<2>, ASAM     : 1 = Sampling begins immediately after last conversion completes
	  // AD1CON1<4>, CLRASAM : 0 = buffer contents will be overwritten by the next conversion sequence
    // AD1CON1<7:5>, SSRC  : 111 = Internal counter ends sampling and starts conversion (auto convert)
	  // AD1CON1<10:8>, FORM : 000 = unsigned integer data format
    // AD1CON1<13>, SIDL   : 0 = Continue module operation when the device enters Idle mode
    // AD1CON1<15>, ON     : 0 = ADC remains off
	
    AD1CON1bits.ASAM = 1;	// 1 = Sampling begins immediately after last conversion completes
	  AD1CON1bits.CLRASAM = 0;// 0 = buffer contents will be overwritten by the next conversion sequence
	  AD1CON1bits.SSRC = 0b111;// 111 = Internal counter ends sampling and starts conversion (auto convert)
    // AD1CON1SET = 0x00e4;     // to set everything above in one fell swoop
 
    // AD1CON2<0>,  ALTS   : 0 = Always use Sample A input multiplexer settings
	  // AD1CON2<1>, BUFM    : 1 = Buffer configured as two 8-word buffers, ADC1BUF7-ADC1BUF0, ADC1BUFF-ADCBUF8
    // AD1CON2<10>, CSCNA  : 1 = Scan inputs
    // AD1CON2<12>, OFFCAL : 0 = Disable Offset Calibration mode
    // AD1CON2<15:13>,VCFG : 000 = Vrefh = AVDD, Vrefl = AVss

    AD1CON2bits.BUFM = 1;	// 1 = Buffer configured as two 8-word buffers
    AD1CON2bits.CSCNA = 1;	// 1 = Scan inputs
    // AD1CON2 = 0x0402;    // to set everything above in one fell swoop
 
    // AD2CON2<5:2>, SMPI  : Interrupt flag set at after numChanInSet completed conversions
    AD1CON2SET = (numChanInSet-1) << 2;
 
    // AD1CON3<7:0>, ADCS  : 1 = TPB * 2 * (ADCS<7:0> + 1) = 4 * TPB = TAD
    // AD1CON3<12:8>, SAMC : 0x0f = Acquisition time = AD1CON3<12:8> * TAD = 15 * TAD
    // AD1CON3<15>, ADRC   : 0 = Clock derived from Peripheral Bus Clock (PBCLK)
    
    AD1CON3bits.ADCS = 1;	// 1 = TPB * 2 * (ADCS<7:0> + 1) = 4 * TPB = TAD
    AD1CON3bits.SAMC = 0x0f;// 0x0f = Acquisition time = AD1CON3<12:8> * TAD = 15 * TAD    
    // AD1CON3 = 0x0f01;    // to set everything above in one fell swoop
 
    // AD1CHS is ignored in scan mode, but we'll clear it to be sure
    AD1CHS = 0;
 
    // select which pins to use for scan mode, a 1 indicates that the corresponding ANx
    // input will be converted	
    AD1CSSL = whichPins;
    
    AD1CON1bits.ON = 1; // enable ADC
  }  
    return returnVal;
}

/****************************************************************************
 Function
     ADC_MultiRead
 Parameters
    uint32_t *adcResults pointer to array to hold conversion set results
        this must have room for at least as many results as numPins in 
        ADC_ConfigAutoScan
 Returns
    nothing 
 Description
    Reads the most recent conversion results from the channel set and copies
        the results to the array passed as a pointer to this function
        lowest numbered converted channel is in adcResults[0]
 Notes
    None.
 Author
    J. Edward Carryer, 10/20/20 16:39
****************************************************************************/
void ADC_MultiRead(uint32_t *adcResults){
uint8_t i;
uint32_t LocalResult;
volatile uint32_t *resultSet;

// stop automatic sampling during the read to be sure we get a coherent set
AD1CON1bits.ASAM = 0;  

if( AD1CON2bits.BUFS == 1){
// 1 = ADC is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7 
    resultSet = &ADC1BUF0;
}else{ 
// 0 = ADC is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    resultSet = &ADC1BUF8;
}
for (i=0; i < numChanInSet; i++)
{
    // read the results from the ADC1BUFx registers. They are 16 bytes apart in
    // the memory map. That's 4 uint32_t apart, hence *4
    LocalResult= adcResults[i] = *(resultSet+(4*i)); // read the results from the ADC1BUFx registers
    
}
AD1CON1bits.ASAM = 1;  // restart automatic sampling
IFS0CLR = _IFS0_AD1IF_MASK;  // clear ADC interrupt flag, see table 7-1, pg 68
}

// count the number of bits set in v. Algorithm from K&R 
static uint8_t CountBits(uint16_t num2Count)
{
  uint8_t count;           // count accumulates the total bits set in num2Count
  for (count = 0; num2Count; count++)
  {
    num2Count &= num2Count - 1;// clear the least significant bit set
  }
  return count;
}



#ifdef INCLUDE_TEST_HARNESS

// PIC32MX170F256B Configuration Bit Settings for ME218 operating at 40MHz from internal fast oscillator (8Mhz)

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdio.h>
#include "bitdefs.h"
#include "terminal.h"

//these were the problematic ones for XinYi
#define CHANNEL_SET (BIT5HI | BIT9HI|  BIT12HI)

uint32_t ResultsArray[MAX_CHANNELS];

void main(void)
{
  uint8_t numChannelsToTest;
  
  numChannelsToTest = CountBits(CHANNEL_SET);
  Terminal_HWInit();
  clrScrn();
  goHome();
  puts("\rTest harness for PIC32_AD_Lib.c\r");
  if ( false == ADC_ConfigAutoScan(0b111111111))
  {
    printf("ADC_ConfigAutoScan() failed when requesting too many channels (correct behavior)\n\r");
  }else{}
  
  printf("Testing %d channels\n\r", numChannelsToTest);
  
  // configure channels
  if ( false == ADC_ConfigAutoScan(CHANNEL_SET))
  {
    printf("ADC_ConfigAutoScan() failed \n\r");
  }else
  {
  // Now get conversion results and print them to the screen
    while(!IsNewKeyReady())
    {
      uint16_t delayCounter;
      uint8_t channelCounter;
      ADC_MultiRead(ResultsArray);
      clrLine();
      for(channelCounter=0; channelCounter < numChannelsToTest; channelCounter++)
      {
        printf("%d:%d   ", channelCounter, ResultsArray[channelCounter]);     
      }
      printf("\r");
      // now, delay a little bit
      for (delayCounter = 0; delayCounter< 65535; delayCounter++){} 
    }
  }
  while(1){}
}


#endif