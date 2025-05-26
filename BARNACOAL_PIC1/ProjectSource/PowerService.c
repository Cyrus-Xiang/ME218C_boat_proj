/****************************************************************************
 Module
   PowerService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
//This Module
#include "PowerService.h"

//Hardware
#include <xc.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"
#include "DrivetrainService.h"
#include "BoatComm.h"

/*----------------------------- Module Defines ----------------------------*/
#define ONE_SEC 1000
#define HALF_SEC ONE_SEC/2
#define FIFTH_SEC ONE_SEC/5

#define DECHARGE_PERIOD FIFTH_SEC
#define RECHARGE_PERIOD FIFTH_SEC
#define IDLE_TIME (ONE_SEC*4)
#define FULL_POWER 150
#define NO_POWER 0
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static BarnacleState_t CurrentState = Idle;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// module variable 
uint8_t Power = NO_POWER;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitPowerService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  DB_printf("\rStart init Power Service\n");
  // put us into the Initial PseudoState
  CurrentState = InitPState;
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
     PostTemplateFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostPowerService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunPowerService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        CurrentState = Pairing;
      }
    }
    break;
    case Pairing:       
    {
      if (ThisEvent.EventType == ES_PAIRED)    
      {
        Power = FULL_POWER;
        CurrentState = Idle;
      }
    }
    break;

    case Idle:
    {
      //DB_printf("Entered Idle State\r\n");
      switch (ThisEvent.EventType)
      {
        case ES_UNPAIRED:  
        {  
          CurrentState = Pairing;
        }
        break;

        case ES_IDLE:  
        {  
          // Do nothing
          CurrentState = Idle;
        }
        break;

        case ES_COMMAND:  
        {  
          ES_Timer_InitTimer(POWER_TIMER, DECHARGE_PERIOD);
          CurrentState = Power_On;
        }
        break;

        case ES_CHARGE:
        {
          Power += 6;
          Power = (Power<FULL_POWER)?Power:FULL_POWER; // Limit power to FULL_POWER
          CurrentState = Recharging;
          DB_printf("ES_CHARGE in Idle State\r\n");
        }
        break;

        case ES_NOPWR:
        {
          CurrentState = No_Power;
        }
        break;

        case ES_TIMEOUT:    // POWER_TIMER start in Power_On state, but ES_TIMEOUT at idle state
        {  
          if (ThisEvent.EventParam == POWER_TIMER)
          {
            //Power -= 1;
            if (Power == NO_POWER)
            {
              ES_Event_t Event2Post;
              Event2Post.EventType = ES_NOPWR;
              PostPowerService(Event2Post);
              PostDrivetrainService(Event2Post);
              CurrentState = No_Power;
            }
          }
        }
        break;

        default:
        break;
      }
    }
    break;

    case Power_On:       
    {
      //DB_printf("Entered Power_ON State\r\n");
      switch (ThisEvent.EventType)
      {
        case ES_COMMAND:
        {
          ES_Timer_InitTimer(POWER_TIMER, DECHARGE_PERIOD);
        }
        break;

        case ES_CHARGE:  
        {  
          Power += 6;
          Power = (Power<FULL_POWER)?Power:FULL_POWER; // Limit power to FULL_POWER
          CurrentState = Recharging;
          // DB_printf("ES_CHARGE in Power_On State\r\n");
        }
        break;

        case ES_TIMEOUT:  
        {  
          if (ThisEvent.EventParam == POWER_TIMER)
          {
            ES_Timer_InitTimer(POWER_TIMER, DECHARGE_PERIOD);
            //Power -= 1;
            if (Power == NO_POWER)
            {
              ES_Event_t Event2Post;
              Event2Post.EventType = ES_NOPWR;
              PostPowerService(Event2Post);
              PostDrivetrainService(Event2Post);
              CurrentState = No_Power;
            }
          }
        }
        break;

        case ES_IDLE:
        {
          CurrentState = Idle;
        }
        break;

        case ES_NOPWR:
        {
          CurrentState = No_Power;
        }
        break;

        case ES_UNPAIRED:  
        {  
          CurrentState = Pairing;
        }
        break;

        default:
          break;
      }
    }
    break;

    case Recharging:       
    {
      //DB_printf("Entered Recharging State\r\n");
      switch (ThisEvent.EventType)
      {
        case ES_CHARGE:  
        {  
          Power += 6;
          Power = (Power<FULL_POWER)?Power:FULL_POWER; // Limit power to FULL_POWER
          DB_printf("ES_CHARGE in Recharging State\r\n");
        }
        break;

        case ES_COMMAND:
        {
          ES_Timer_InitTimer(POWER_TIMER, DECHARGE_PERIOD);
          CurrentState = Power_On;
        }
        break;

        case ES_DUMP:
        {
          ES_Timer_InitTimer(POWER_TIMER, DECHARGE_PERIOD);
          CurrentState = Power_On;
        }
        break;

        case ES_IDLE:
        {
          CurrentState = Idle;
        }
        break;

        case ES_UNPAIRED:  
        {  
          CurrentState = Pairing;
        }
        break;

        default:
          break;
      }
    }
    break;

    case No_Power:       
    {
      DB_printf("NO CHARGE!!! CHARGE IMMEDIATELY\r\n");
      switch (ThisEvent.EventType)
      {
        case ES_CHARGE:  
        {  
          Power += 6;
          Power = (Power<FULL_POWER)?Power:FULL_POWER; // Limit power to FULL_POWER
          CurrentState = Recharging;
        }
        break;

        case ES_UNPAIRED:  
        {  
          CurrentState = Pairing;
        }
        break;

        default:
          break;
      }
    }
    break;

    default:
      break;
  }                                   
  return ReturnEvent;
}
/***************************************************************************
 private functions
 ***************************************************************************/