#ifndef _PWM_PIC32_H
#define _PWM_PIC32_H

/****************************************************************************
 Module
     PWM_PIC32.h
 Description
     header file to support use of the PWM library on the PIC32
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/19/21       jec     Started port from Tiva version
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>

// typdef to specify which timer is being set
typedef enum {
  _Timer2_,
  _Timer3_
}WhichTimer_t;

// typdef to specify pins for PWM. Implementation code depends on this order
// do not modify!
typedef enum {
        PWM_RPA0 = 0,
        PWM_RPA1,
        PWM_RPA2,
        PWM_RPA3,
        PWM_RPA4,
        PWM_RPB0,
        PWM_RPB1,
        PWM_RPB2,
        PWM_RPB3,
        PWM_RPB4,
        PWM_RPB5,
        PWM_RPB6,
        PWM_RPB7,
        PWM_RPB8,
        PWM_RPB9,
        PWM_RPB10,
        PWM_RPB11,
        PWM_RPB12,
        PWM_RPB13,
        PWM_RPB14,
        PWM_RPB15
} PWM_PinMap_t;

/****************************************************************************
 Function
    PWMSetup_BasicConfig

 Parameters
   uint8_t: How many PWM/Pulse channels to be configured

 Returns
   bool: true if the number of channels requested is legal; otherwise, false

 Description
   Should be the first function called when setting up the PWM HAL.
   1) Configures both Timer2 & Timer3 for /8 from PBClk as clock source
   2) Sets up both Timer2 & Timer3 to a 50Hz frequency/ 20ms period
   3) Sets the requested OCx channels to PWM mode.
   4) Enables the requested OCx channels and both Timer2 & Timer3
   Further function calls from the PWM HAL will be necessary to complete 
   the setup.
   
Example
   PWMSetup_BasicConfig(1);
****************************************************************************/
bool PWMSetup_BasicConfig(uint8_t HowMany);

/****************************************************************************
 Function
    PWMSetup_AssignChannelToTimer

 Parameters
   uint8_t: Which channels to be assigned
   WhichTimer_t: Which timer (Timer2 or Timer3) to assigned to that channel

 Returns
   bool: true if the channel requested and the timer are both legal; 
   otherwise, false

 Description
  Assigns one of the 2 available timers (Timer2 & Timer3) to the specified
  OCx channel.

 Note:
   if used on a running channel whose output is enabled, it will produce
   a glitch output. Should be used before outputs are enabled
   
Example
   PWMSetup_AssignChannelToTimer(1, _Timer2_);
****************************************************************************/
bool PWMSetup_AssignChannelToTimer( uint8_t whichChannel, 
                                    WhichTimer_t whichTimer );

/****************************************************************************
 Function
    PWMSetup_SetPeriodOnTimer

 Parameters
   uint16_t: The new period, specified as the number of 0.4micro-sec ticks
             in the period. i.e. for 1ms use 2500 ticks
   WhichTimer_t: Which timer (Timer2 or Timer3) period will be set.

 Returns
   bool: true if the channel requested and the timer are both legal; 
   otherwise, false

 Note:
   minimum period is 100 ticks, maximum 65535 
 
 Description
  updates the period on one of the 2 available timers (Timer2 & Timer3) to 
  the specified new period.
   
Example
   PWMSetup_SetPeriodOnTimer(2500, _Timer2_); // pardon the magic number :-)
****************************************************************************/
bool PWMSetup_SetPeriodOnTimer( uint16_t reqPeriod, WhichTimer_t WhichTimer );


/****************************************************************************
 Function
    PWMSetup_SetFreqOnTimer

 Parameters
   uint16_t: The new frequency (in Hz) to be set on the specified timer
   WhichTimer_t: Which timer (Timer2 or Timer3) frequency will be set.

 Returns
   bool: true if the frequency requested and the timer are both legal; 
   otherwise, false

 Note:
   minimum frequency is 38Hz, maximum 25,000Hz
  
 Description
   Calculates a new period based on the requested frequency, then uses
   PWMSetup_SetPeriodOnTimer to program the new period.
   
Example
   PWMSetup_SetFreqOnTimer(50, _Timer2_); // pardon the magic number :-)
****************************************************************************/
bool PWMSetup_SetFreqOnTimer( uint16_t reqFreq, WhichTimer_t WhichTimer );

/****************************************************************************
 Function
    PWMSetup_MapChannelToOutputPin

 Parameters
   uint8_t: The OCx channel to be mapped to one of the programmable output pins
   PWM_PinMap_t: Which pin the channel will be mapped to.

 Returns
   bool: true if the pin requested is among the legal choices for the channel; 
   otherwise, false
  
 Description
   Uses the programmable output mapping registers to connect an OCx output to 
   a pin.

 Note:
   generally, this should be the last step in the setup process as nothing
   comes out of the OCx system until a pin is assigned.
   
Example
   PWMSetup_MapChannelToOutputPin(1, PWM_RPA0);
****************************************************************************/
bool PWMSetup_MapChannelToOutputPin( uint8_t channel, PWM_PinMap_t WhichPin);

/****************************************************************************
 Function
    PWMOperate_SetDutyOnChannel

 Parameters
   uint8_t: The requested duty cycle (0-100%)
   uint8_t: The channel whose duty cycle will be set.

 Returns
   bool: true if the requested duty cycle and channel are legal; 
   otherwise, false
  
 Description
   programs the rise time of the output to achieve the specified output duty
   cycle 

Example
   PWMOperate_SetDutyOnChannel(50, 1); // pardon the magic numbers :-)
****************************************************************************/
bool PWMOperate_SetDutyOnChannel( uint8_t dutyCycle, uint8_t channel);

/****************************************************************************
 Function
   PWMOperate_SetPulseWidthOnChannel

 Parameters
   uint16_t: The requested pulse width 1 to the current period of the timer
   uint8_t: The channel whose pulse width will be set.

 Returns
   bool: true if the requested pulse width and channel are legal; 
   otherwise, false
  
 Description
   programs the rise time of the output to achieve the specified pulse width

Example
   PWMOperate_SetPulseWidthOnChannel(2500, 1); // pardon the magic numbers :-)
****************************************************************************/
bool PWMOperate_SetPulseWidthOnChannel( uint16_t NewPW, uint8_t channel);

#endif //_PWM_PIC32_H
