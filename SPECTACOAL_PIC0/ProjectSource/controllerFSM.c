/****************************************************************************
 Module
   controllerFSM.c

 Revision
   1.0.1

 Description
   This is a controller file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RuncontrollerSM()
 10/23/11 18:20 jec      began conversion from SMcontroller.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "controllerFSM.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
#include <xc.h>
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void config_joystick_ADC(void);
static void config_buttons(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static controllerState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
#define ADC_scan_interval 100
static uint8_t MyPriority;
static uint32_t Curr_AD_Val[2];
//static uint32_t Last_AD_Val[] ={0,0};
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitcontrollerFSM

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
bool InitcontrollerFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = Idle_s;
  // configure pins and ADC for X Y information of joysticks
  config_joystick_ADC();
  config_buttons();
  DB_printf("controllerFSM successfully initialized\n");
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
     PostcontrollerFSM

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
bool PostcontrollerFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RuncontrollerFSM

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
ES_Event_t RuncontrollerFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == JoystickScan_TIMER)
  {
    // read the joystick values
    ADC_MultiRead(Curr_AD_Val);
    DB_printf("X: %d Y: %d\n", Curr_AD_Val[0], Curr_AD_Val[1]);
    ES_Timer_InitTimer(JoystickScan_TIMER, ADC_scan_interval);
  }
  switch (CurrentState)
  {
    case Idle_s:        // If current state is initial Psedudo State
    {
      
    }
    break;

    case Pairing_s:        // If current state is state one
    {
      
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QuerycontrollerSM

 Parameters
     None

 Returns
     controllerState_t The current state of the controller state machine

 Description
     returns the current state of the controller state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
controllerState_t QuerycontrollerFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static void config_joystick_ADC(void)
{
  TRISBbits.TRISB12 = 1;
  ANSELBbits.ANSB12 = 1; 
  TRISBbits.TRISB13 = 1;
  ANSELBbits.ANSB13 = 1;
  ADC_ConfigAutoScan(BIT11HI|BIT12HI);//AN11 is for B13. X pos of joystick; AN12 is for B12, Y pos of joystick
  ES_Timer_InitTimer(JoystickScan_TIMER, ADC_scan_interval);
  
  return;
}

static void config_buttons(void)
{
  TRISAbits.TRISA4 = 1; // A4 is the pairing button
  TRISBbits.TRISB4 = 1; // B4 is the drop coal button
  TRISBbits.TRISB9 = 1; // B9 is the drop anchor button
  return;
}