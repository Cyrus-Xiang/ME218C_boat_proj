/****************************************************************************
 Module
   ADService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
//This module
#include "ADService.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include "DrivetrainService.h"

/*----------------------------- Module Defines ----------------------------*/
#define AD_CHANNEL BIT12HI //RB12/AN12
#define MAX_STEP_TIME 50

#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWENTY_MS (ONE_SEC * .02)
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint32_t StepTime;
static uint32_t ScaledPotentialValue;
static uint32_t PotentialValue[1];

ES_Event_t ADTimerEvent;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitADService

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
bool InitADService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  // set pin23/RB12 as analog input
  TRISBbits.TRISB12 = 1;
  ANSELBbits.ANSB12 = 1;
  DB_printf("\rStart init AD Service\n");

  //Configure AD channel
  ADC_ConfigAutoScan(AD_CHANNEL);
  
  //Initial read of value
  ADC_MultiRead(PotentialValue);
  StepTime = ((uint32_t)MAX_STEP_TIME * PotentialValue[0] /1024) +1;

  ES_Timer_InitTimer(AD_TIMER, 100);

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
     PostADService

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
bool PostADService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunADService

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
ES_Event_t RunADService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch(ThisEvent.EventType)
  {
    case(ES_TIMEOUT):
    {
      if(ThisEvent.EventParam == AD_TIMER)
      {
        ADC_MultiRead(PotentialValue);
        ScaledPotentialValue = 100 * PotentialValue[0] /1023;
        StepTime = ((uint32_t)MAX_STEP_TIME * PotentialValue[0] /1023) +1;
        ADTimerEvent.EventType = ES_TIMEOUT;
        ADTimerEvent.EventParam = AD_TIMER;
        PostDrivetrainService(ADTimerEvent);
        ES_Timer_InitTimer(AD_TIMER, 200);
      }
    }
    break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
uint32_t GetStepTime(void)
{
  return StepTime;
}

uint32_t GetScaledPotentialValue(void)
{
  return ScaledPotentialValue;
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/