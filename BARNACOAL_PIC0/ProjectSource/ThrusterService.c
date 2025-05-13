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
#define thruster_PWM_period_us 20000 //in microseconds
#define ticks_per_us 2.5 //for 40MHz clock
#define numOfOC_channels 2 //total number of OC channels configured
#define OCchannel_4_thruster 1 //should be btw 1 to 5
#define InitialPulseWidth_us 1700//in microseconds 
static WhichTimer_t timer_4_thruster = _Timer2_;
static PWM_PinMap_t OCpin_4_thruster_L = PWM_RPB3;//pin for the left thruster
static uint8_t MyPriority;

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
  //configure the pins
  // ANSELBbits.ANSB3 = 0; //set pin 3 to digital
  // TRISBbits.TRISB3 = 0; //set pin 3 to output
  //Initialize the PWM module
  PWMSetup_BasicConfig(1);
  PWMSetup_AssignChannelToTimer(1, _Timer2_); //assign OC1 to Timer2
  //PWMSetup_SetPeriodOnTimer(thruster_PWM_period_us*ticks_per_us, timer_4_thruster); 
  PWMSetup_SetFreqOnTimer(50, _Timer2_); //50Hz
  PWMSetup_MapChannelToOutputPin(1, PWM_RPB3); //assign OC1 to pin 3
  //set some initial duty cycle for the channels
  //PWMOperate_SetPulseWidthOnChannel(InitialPulseWidth_us*ticks_per_us,OCchannel_4_thruster); 
  PWMOperate_SetDutyOnChannel(50, 1); 
  DB_printf("PWM setup done\n");
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

