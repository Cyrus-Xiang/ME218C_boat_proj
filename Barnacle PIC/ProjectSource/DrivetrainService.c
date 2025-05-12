/****************************************************************************
 Module
   DrivetrainService.c

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
#include "DrivetrainService.h"

//Hardware
#include <xc.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/
#define FREQUENCY_PBCLK 20000000 //20MHz
#define FREQUENCY_DESIRED 200 // 200HZ
#define PRESCALAR 0b010 // 1:4 
#define PERIOD_DESIRED FREQUENCY_PBCLK/FREQUENCY_DESIRED/(4)
#define PERIOD_COUNT PERIOD_DESIRED-1

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static DrivetrainState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

// parameter for control
static uint32_t DCMotorSpeed = PERIOD_COUNT;
static uint8_t DriveSpeed;
static uint8_t TurnSpeed;
static uint32_t PR = PERIOD_COUNT;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDrivetrainService

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
bool InitTemplateFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  DB_printf("\rStart init Drivetrain Service\n");

  //Configure pins
  ANSELAbits.ANSA0 = 0; //disable analog on RA0, RA1 for digital output
  ANSELAbits.ANSA1 = 0;
  TRISAbits.TRISA0 = 0; //set RA0, RA1 as output for motor control
  TRISAbits.TRISA1 = 0;
  RPA0R = 0b0101; //map RA0, RA1 to output compare mode 
  RPA1R = 0b0101; 
  TRISAbits.TRISA4 = 1; //set RA4 as the digital input for direction switch

  //Configure Timer2
  T2CONbits.ON = 0; //disable time2 during setup
  T2CONbits.TCS = 0; //clear TCS to select internal PB clock source
  T2CONbits.TGATE = 0; //disable gated time accumulation
  T2CONbits.TCKPS = PRESCALAR; //prescalar 1:4
  TMR2 = 0; //clear timer register
  PR2 = PERIOD_COUNT; //Set period based on the desired 200Hz frequency
  IFS0CLR = _IFS0_T2IF_MASK; //clear interrupt flag

  //Configure output compare module
  OC1CONbits.ON = 0; //disable OC1, OC2 for configuration
  OC2CONbits.ON = 0;
  OC1CONbits.OC32 = 0; //set OC1, OC2 for comparison to 16bit timer source
  OC2CONbits.OC32 = 0; 
  OC1CONbits.OCTSEL = 0; //set OC1, OC2 that Timer2 is the source for OC
  OC2CONbits.OCTSEL = 0; 
  OC1CONbits.OCM = 0b110; //set PWM mode on OC1, OC2 and disable fault pin
  OC2CONbits.OCM = 0b110;

  //Finish configuration, enable Timer and Output Compare Module
  T2CONbits.ON = 1; //enable Timer2
  OC1CONbits.ON = 1; //enable OC1, OC2
  OC2CONbits.ON = 1;

  //Initialize PWM with zero duty cycle
  OC1R = DCMotorSpeed;
  OC2R = DCMotorSpeed;
  OC1RS = DCMotorSpeed;
  OC2RS = DCMotorSpeed;

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
     PostDrivetrainService

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
bool PostDrivetrainService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunDrivetrainService

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
ES_Event_t RunDrivetrainService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case Driving:        // If current state is initial Psedudo State
    {
      switch (ThisEvent.EventType)
      {
      case SpeedUpdate:
      {
        DriveSpeed = ThisEvent.EventParam;
        ES_Event_t Event2Post;
        Event2Post.EventType = DriveUpdate;
        Event2Post.EventParam = DriveSpeed;
        PostDrivetrainService();

        }
        PostDrivetrainService(); 
      }
      break;

      case TurnUpdate:
      {
        TurnSpeed = ThisEvent.EventParam;
      }
      break;

      case AjustingPWM
      default:
        break;
      }
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // this is where you would put any actions associated with the
        // transition from the initial pseudo-state into the actual
        // initial state

        // now put the machine into the actual initial state
        CurrentState = UnlockWaiting;
      }
    }
    break;

    case Recharging:        // If current state is state one
    {
      switch (ThisEvent.EventType)
      {
        case ES_LOCK:  //If event is event one

        {   // Execute action function for state one : event one
          CurrentState = Locked;  //Decide what the next state will be
        }
        break;

        // repeat cases as required for relevant events
        default:
          ;
      }  // end switch on CurrentEvent
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
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
TemplateState_t QueryTemplateFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

