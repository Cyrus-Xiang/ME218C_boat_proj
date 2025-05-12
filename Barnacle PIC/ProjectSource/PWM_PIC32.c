//#define TESTING
/****************************************************************************
 Module
     PWM_PIC32.c
 Description
     Implementation file for the PWM Library for the PIC32
 Notes
     Sets the Timer2/3 clock rate to PBClk/8 which in our case gives a 2.5MHz
     clock rate for the PWM/Pulse generation. This still allows at least 1%
     PWM resolution up to PWM frequencies of 25kHz
     
 History
 When           Who     What/Why
 -------------- ---     --------
 10/19/21       jec     Started conversion from Tiva version
 *****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <xc.h>
#include <stdbool.h>
#include "PWM_PIC32.h"

/*----------------------------- Module Defines ----------------------------*/
#define MAX_NUM_CHANNELS 5
// the standard PBClk rate for ME218
#define PBCLK_RATE 20000000L
// TIMERx divisor for PWM, standard value is 8, to give maximum resolution
#define TIMER_DIV 8
// TICS_PER_MS assumes a 20MHz PBClk /8 = 2.5MHz clock rate
#define TICS_PER_MS 2500
// a servo wants to see a 50Hz or 20ms period
#define SERVO_PERIOD  (20*TICS_PER_MS)

// these limits derive from a 20Mhz PBClk divided by 8 and a 16 bit timer
#define MIN_FREQ  24
#define MAX_FREQ 25000

// this limit guarantees 1% resolution in the duty cycle
#define MIN_PERIOD 100

// there are only 5 pins defined for each output on our chip
#define MAX_PINS_PER_OUTPUT 5

/*------------------------------ Module Types -----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
static bool IsPinLegalForChannel(PWM_PinMap_t WhichPin, uint8_t WhichChannel);
static bool IsChannelIllegal( uint8_t whichChannel );

/*---------------------------- Module Variables ---------------------------*/

static int8_t MaxConfiguredChannel = -1; // init to illegal value

// local store of timer periods used when calculating duty cycle                                         
static uint16_t T2Period;
static uint16_t T3Period;

static uint8_t  LocalDuty[MAX_NUM_CHANNELS] = {0,0,0,0,0};

// these arrays are used in mapping between channel number (1-5) and the
// addresses of various registers
static  uint32_t volatile * const ChannelTo_pControlReg[MAX_NUM_CHANNELS]={
                        &OC1CON,&OC2CON,&OC3CON,&OC4CON,&OC5CON};

static  uint32_t volatile * const ChannelTo_pOCRS_Reg[MAX_NUM_CHANNELS]={
                        &OC1RS,&OC2RS,&OC3RS,&OC4RS,&OC5RS};

static  uint32_t volatile * const ChannelTo_pOCR_Reg[MAX_NUM_CHANNELS]={
                        &OC1R,&OC2R,&OC3R,&OC4R,&OC5R};

// which timer is in use by a channel? defaults to timer2
static  uint32_t volatile * ChannelTo_pTimer[MAX_NUM_CHANNELS]={
                        &PR2,&PR2,&PR2,&PR2,&PR2};

// the lists of legal pin numbers for the output channels
static PWM_PinMap_t const LegalOutPins[][5] =  {{ PWM_RPA0, PWM_RPB3, PWM_RPB4, 
                                                  PWM_RPB7, PWM_RPB15 },
                                                { PWM_RPA1, PWM_RPB1, PWM_RPB5, 
                                                  PWM_RPB8,PWM_RPB11 },
                                                { PWM_RPA3, PWM_RPB0, PWM_RPB9, 
                                                  PWM_RPB10,PWM_RPB14 },
                                                { PWM_RPA2, PWM_RPA4, PWM_RPB2, 
                                                  PWM_RPB6, PWM_RPB13 },
                                                { PWM_RPA2, PWM_RPA4, PWM_RPB2, 
                                                  PWM_RPB6, PWM_RPB13 }
};

// these are the output mapping registers indexed by the PWM_PinMap_t value
static volatile uint32_t * const outputMapRegisters[] = { &RPA0R, &RPA1R, 
                      &RPA2R, &RPA3R, &RPA4R, 
                      &RPB0R, &RPB1R, &RPB2R, &RPB3R, &RPB4R, &RPB5R, 
                      &RPB6R, &RPB7R, &RPB8R, &RPB9R, &RPB10R, &RPB11R, &RPB12R,
                      &RPB13R, &RPB14R, &RPB15R 
};

// these are the constants used to map OCx channel constants to output pins
static uint32_t const mapChannel2PinSelConst[] = { 0b0101/*OC1*/, 0b0101/*OC2*/,
                                                   0b0101/*OC3*/, 0b0101/*OC4*/,
                                                   0b0110/*OC5*/
};

static PWM_PinMap_t const LegalOC2OutPins[] = { PWM_RPA1, PWM_RPB1, PWM_RPB5, 
                                                PWM_RPB8,PWM_RPB11
};

static PWM_PinMap_t const LegalOC3OutPins[] = { PWM_RPA3, PWM_RPB0, PWM_RPB9, 
                                                PWM_RPB10,PWM_RPB14
};

static PWM_PinMap_t const LegalOC4_5OutPins[] = { PWM_RPA2, PWM_RPA4, PWM_RPB2, 
                                                  PWM_RPB6,PWM_RPB13
};

/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
    PWMSetup_BasicConfig

   1) Configures both Timer2 & Timer3 for /8 from PBClk as clock source
   2) Sets up both Timer2 & Timer3 to a 50Hz frequency/ 20ms period
   3) Sets the requested OCx channels to PWM mode.
   4) Enables the requested OCx channels and both Timer2 & Timer3
   Further function calls from the PWM HAL will be necessary to complete 
   the setup.
****************************************************************************/
bool PWMSetup_BasicConfig(uint8_t HowMany){    
 uint8_t i;
 bool ReturnVal = true;

//sanity check
  if ((0 == HowMany) || (HowMany > MAX_NUM_CHANNELS))
  {
      ReturnVal = false;
  }else
  {   // Requested number of channels is legal, so let's get things set up
    MaxConfiguredChannel = HowMany; // note how many we have configured
    // start by turning Timer2 off
    T2CONbits.ON = 0;
    // base Timer2 on PBClk/8
    T2CONbits.TCS = 0;  // use PBClk as clock source    
    T2CONbits.TCKPS = 0b011;  // divide by 8
    PR2 = SERVO_PERIOD; // default to the servo rate, 50Hz

    // next, turn Timer3 off
    T3CONbits.ON = 0;
    // base Timer3 on PBClk/8
    T3CONbits.TCS = 0;  // use PBClk as clock source    
    T3CONbits.TCKPS = 0b011;  // divide by 8
    PR3 = SERVO_PERIOD; // default to the servo rate, 50Hz

    // with the Timers configured, move to the PWM setup, 1 loop per channel
    for ( i=0; i<MaxConfiguredChannel; i++)
    {
      // turn off the OC system before making changes
      ((__OC1CONbits_t *)ChannelTo_pControlReg[i])->ON = 0;
      // default to selecting timer 2
      ((__OC1CONbits_t *)ChannelTo_pControlReg[i])->OCTSEL = 0; 
      // set PWM mode with no fault detect
      ((__OC1CONbits_t *)ChannelTo_pControlReg[i])->OCM = 0b110;
      // set the default DC to 0
      *ChannelTo_pOCRS_Reg[i] = 0; // this is the repeating cycle
      *ChannelTo_pOCR_Reg[i] = 0;  // this is the initial cycle
      // finally, turn OC system back on (clock is still off)
      ((__OC1CONbits_t *)ChannelTo_pControlReg[i])->ON = 1;
    }
    //  finish up by turning the clocks to the PWM system on
    T2CONbits.ON = 1; // turn Timer2 back on
    T3CONbits.ON = 1; // turn Timer3 back on
    
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    PWMSetup_MapChannelToOutputPin

****************************************************************************/  
bool PWMSetup_MapChannelToOutputPin(uint8_t WhichChannel, PWM_PinMap_t WhichPin)
{
  bool ReturnVal = true;
  
  // test that BasicConfig has been called and for legal channel
  if ( ( -1 == MaxConfiguredChannel) || IsChannelIllegal(WhichChannel))
  {
    ReturnVal = false;
  }else
  { // now check to see if the requested pin is legal for the requested channel
    if( false == IsPinLegalForChannel(WhichPin, WhichChannel))
    {
      ReturnVal = false;
    }else // OK, everything is legal, now map the pin
    {
      *outputMapRegisters[WhichPin] = mapChannel2PinSelConst[WhichChannel-1];
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    PWMSetup_AssignChannelToTimer
****************************************************************************/
bool PWMSetup_AssignChannelToTimer( uint8_t whichChannel, 
                                    WhichTimer_t whichTimer )
{
  bool ReturnVal = true;
  uint8_t channelIndex;
  
  if( IsChannelIllegal(whichChannel) )
  {
    ReturnVal = false;
  }else
  {
    channelIndex = whichChannel-1;
    
    if( _Timer2_ == whichTimer)
    {
      // save the PR register to use for this channel
      ChannelTo_pTimer[channelIndex] = &PR2;
      // program the channel to use the new timer
      ((__OC1CONbits_t *)ChannelTo_pControlReg[channelIndex])->OCTSEL = 0; 
    } else if (_Timer3_ == whichTimer)
    {
      // save the PR register to use for this channel
      ChannelTo_pTimer[channelIndex] = &PR3;
      // program the channel to use the new timer
      ((__OC1CONbits_t *)ChannelTo_pControlReg[channelIndex])->OCTSEL = 1; 
    }else // if not Timer2 or Timer3, then it is bad
    {
      ReturnVal = false;
    }
  }
  return ReturnVal;
}

/****************************************************************************
 Function
    PWMOperate_SetDutyOnChannel
****************************************************************************/
bool PWMOperate_SetDutyOnChannel( uint8_t dutyCycle, uint8_t whichChannel)
{
  bool  ReturnVal = true;
  uint32_t updateVal;
  uint8_t channelIndex;
  
  channelIndex = whichChannel-1; // convert OC1-5 to 0-4 for indexing
  
  // sanity check, reasonable channel number & DC
  if (IsChannelIllegal(whichChannel) || (100 < dutyCycle))
  {
    ReturnVal = false;
  }else
  {    
    // update local copy of DC used when changing freq or period
    LocalDuty[channelIndex] = dutyCycle;
    
    if (0 == dutyCycle)
    { // don't try to calculate with 0 DC
      updateVal = 0;
    }else
    { // reasonable duty cycle number, so calculate new pulse width
      updateVal = ((*(ChannelTo_pTimer[channelIndex]))*dutyCycle)/100;    
    }
    // 100% DC needs to be handled differently to work with the PWM hardware
    if (100 == dutyCycle)
    {
      // To program 100% DC, simply set the RS reg higher than the period
      updateVal = (*(ChannelTo_pTimer[channelIndex])) + 1;
    }else
    {
    } // no else clause
    // now update the value in the RS register
    *(ChannelTo_pOCRS_Reg[channelIndex]) = updateVal;
  }
  
  return ReturnVal;
}

/****************************************************************************
 Function
   PWMOperate_SetPulseWidthOnChannel

****************************************************************************/
bool PWMOperate_SetPulseWidthOnChannel( uint16_t NewPW, uint8_t whichChannel)
{
  bool ReturnVal = true;
  
  if (IsChannelIllegal(whichChannel) ||
      (NewPW > *ChannelTo_pTimer[whichChannel-1]))
  {
    ReturnVal = false;
  }else
  {
    // everything looks good so update the value in the RS register
    *(ChannelTo_pOCRS_Reg[whichChannel-1]) = NewPW;
  }         
  return ReturnVal;
}

/****************************************************************************
 Function
    PWMSetup_SetPeriodOnTimer

 ****************************************************************************/
bool PWMSetup_SetPeriodOnTimer( uint16_t reqPeriod, WhichTimer_t WhichTimer)
{
  bool ReturnVal = true;
  
  if (MIN_PERIOD > reqPeriod)
  {
    ReturnVal = false;
  }else
  {
    switch  (WhichTimer)
    {
      case  _Timer2_:
        PR2 = T2Period = reqPeriod;
        break;
      
      case  _Timer3_:
        PR3 = T3Period = reqPeriod;
        break;
      
      default: // anything else is illegal
        ReturnVal = false;
        break;
    }
  }
  return ReturnVal;
}


/****************************************************************************
 Function
    PWMSetup_SetFreqOnTimer
****************************************************************************/

bool PWMSetup_SetFreqOnTimer( uint16_t reqFreq, WhichTimer_t WhichTimer)
{
  bool ReturnVal = true;
  uint16_t newPeriod;
  
  // sanity check for legal timers & within range
  if(((_Timer2_ != WhichTimer) && (_Timer3_ != WhichTimer)) ||
    (MIN_FREQ > reqFreq) || (MAX_FREQ < reqFreq))
  { 
    ReturnVal = false;
  }else
  {
    //Use the Frequency (expressed in Hz) to calculate a new period
    newPeriod = PBCLK_RATE/TIMER_DIV /reqFreq;
    // apply the new period
    PWMSetup_SetPeriodOnTimer( newPeriod, WhichTimer);
  }
  
  return ReturnVal;
}

//*********************************
// private functions
//*********************************
static bool IsPinLegalForChannel(PWM_PinMap_t WhichPin, uint8_t WhichChannel)
{
  bool ReturnVal = false;
  uint8_t index;
  
  WhichChannel -= 1;  // convert from 1-5 to 0-4 for indexing into arrays
  for( index = 0; index < MAX_PINS_PER_OUTPUT; index++)
  {
    if ( LegalOutPins[WhichChannel][index] == WhichPin )
    {
      ReturnVal = true;
      break;
    }
  }
  return ReturnVal;
}

static bool IsChannelIllegal( uint8_t whichChannel )
{  
  return ((0 == whichChannel) || (whichChannel > MaxConfiguredChannel));
}

//*********************************
// test harness
//*********************************
#ifdef TESTING
void WaitForRise( void );

void main(void)
{
  // set up as button input on RB12 to stage tests
  ANSELB = 0; //disable analog
  TRISB |= 1<<12; // config pin as input
  
  PWMSetup_BasicConfig(2);
  PWMSetup_SetFreqOnTimer(50, _Timer3_);
  PWMSetup_AssignChannelToTimer(1, _Timer3_);
  PWMSetup_AssignChannelToTimer(2, _Timer3_);  
  PWMSetup_MapChannelToOutputPin(1, PWM_RPB3);
  PWMSetup_MapChannelToOutputPin(2, PWM_RPB5);
  PWMOperate_SetDutyOnChannel(50,1);  
  PWMOperate_SetDutyOnChannel(25,2);
  WaitForRise();
  
  PWMOperate_SetPulseWidthOnChannel((1*TICS_PER_MS),1);
  PWMOperate_SetPulseWidthOnChannel((2*TICS_PER_MS),2);
  WaitForRise();
  
  PWMSetup_SetFreqOnTimer(100, _Timer3_);
  PWMOperate_SetDutyOnChannel(50,1);  
  PWMOperate_SetDutyOnChannel(25,2);    
  
  while(1)
    ;
}

void WaitForRise( void )
{
  bool currentButtonState;
  
  do // wait for low level
  {
    currentButtonState = PORTBbits.RB12;
  } while (1 == currentButtonState);
  
  do // wait for rise
  {
    currentButtonState = PORTBbits.RB12;    
  }while (( 0 == currentButtonState));
  
  return;
}
#endif