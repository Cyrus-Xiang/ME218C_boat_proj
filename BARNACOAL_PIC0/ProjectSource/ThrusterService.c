/****************************************************************************
 Module
   ThrusterService.c

 Revision
   1.0.1

 Description
   This is a Thruster file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from ThrusterFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ThrusterService.h"
#include "PWM_PIC32.h"
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable

#define low_PW_us 500
#define upper_PW_us 2500
static uint16_t PWM_period_us = 20000;
static float ticks_per_us = 2.5;
// static uint32_t LastAD_Val [Num_AD_Channels];
static uint8_t MyPriority;
static uint8_t DutyCycle;
static uint16_t PulseWidth;
static uint16_t PW_mid_us;
static uint16_t PW_range_us;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitThrusterService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitThrusterService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  DB_printf("start initializing thruster service\n");
  MyPriority = Priority;
  // configure the pins
  //  ANSELBbits.ANSB3 = 0; //set pin 3 to digital
  //  TRISBbits.TRISB3 = 0; //set pin 3 to output
  // Initialize the PWM module

  // code from me218a
  DB_printf("PWM period is initially %d us\n", PWM_period_us);
  DB_printf("ticks_per_us * PWM period is %d\n", ((uint16_t) ticks_per_us*PWM_period_us));
  if (!PWMSetup_BasicConfig(1))
  {
    DB_printf("PWMSetup_BasicConfig failed\n");
  }

  if (!PWMSetup_SetPeriodOnTimer((uint16_t) PWM_period_us * ticks_per_us, _Timer2_))
  {
    DB_printf("PWMSetup_SetPeriodOnTimer failed\n");
  }
  else
  {
    DB_printf("PWM period is set to %d ticks, which is %d us \n", (uint16_t)PWM_period_us * ticks_per_us, PWM_period_us);
    DB_printf("PWM period is initially %d us\n", PWM_period_us);  
  }
  if (!PWMSetup_AssignChannelToTimer(1, _Timer2_))
  {
    DB_printf("PWMSetup_AssignChannelToTimer failed\n");
  }
 
  if (!PWMSetup_MapChannelToOutputPin(1, PWM_RPB3))
  {
    DB_printf("PWMSetup_MapChannelToOutputPin failed\n");
  }
  else
  {
    DB_printf("PWM channel 1 is mapped to pin B3\n");
  }
  PW_range_us = upper_PW_us - low_PW_us;
  PW_mid_us = (uint16_t)PW_range_us / 2 + low_PW_us;
  PulseWidth = PW_mid_us * ticks_per_us;

  PWMOperate_SetPulseWidthOnChannel(PulseWidth, 1);
  DB_printf("PWM pulse width is set to %u ticks \n", PulseWidth);
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostThrusterService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostThrusterService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunThrusterService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunThrusterService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
