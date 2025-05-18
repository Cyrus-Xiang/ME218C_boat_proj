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
// #include "ADService.h"
#include "PowerService.h"


/*----------------------------- Module Defines ----------------------------*/
#define FREQUENCY_PBCLK 20000000 //20MHz
#define FREQUENCY_DESIRED 500 // 2873HZ don't know why, but should be the same as lab 7
#define PRESCALAR 0b010 // 1:4 
#define PERIOD_DESIRED FREQUENCY_PBCLK/FREQUENCY_DESIRED/(4)
#define PERIOD_COUNT PERIOD_DESIRED-1

#define PWM_MIN 60
#define PWM_OFF 75
#define PWM_MAX 90

#define SERVO_OPEN_POS 80
#define SERVO_CLOSE_POS 30

#define ONE_SEC 1000
#define HALF_SEC ONE_SEC/2

#define TURN_WEIGHT 1/4

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static BarnacleState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

static uint32_t PR = PERIOD_COUNT;

// parameter for control
static uint8_t Velocity = 0;
static uint8_t Omega = 0;
static uint8_t ScaledLeft = 127;
static uint8_t ScaledRight = 127;
static uint8_t PWMLeft = 50;
static uint8_t PWMRight = 50;
static uint8_t Servo_POS = SERVO_CLOSE_POS;


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
bool InitDrivetrainService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  DB_printf("\rStart init Drivetrain Service\n");

  //Configure pins
  ANSELAbits.ANSA0 = 0; //disable analog on RA0, RA1 for digital output
  ANSELAbits.ANSA1 = 0;
  TRISAbits.TRISA0 = 0; //set RA0, RA1, RA3 as output for motor control
  TRISAbits.TRISA1 = 0;
  TRISAbits.TRISA3 = 0;
  RPA0R = 0b0101; //map RA0, RA1, RA3 to output compare mode 
  RPA1R = 0b0101;
  RPA3R = 0b0101;
  TRISAbits.TRISA4 = 1; //set RA4 as the digital input for direction switch

  //Configure Timer2
  T2CONbits.ON = 0; //disable time2 during setup
  T2CONbits.TCS = 0; //clear TCS to select internal PB clock source
  T2CONbits.TGATE = 0; //disable gated time accumulation
  T2CONbits.TCKPS = PRESCALAR; //prescalar 1:4
  TMR2 = 0; //clear timer register
  PR2 = PERIOD_COUNT; //Set period based on the desired 200Hz frequency
  IFS0CLR = _IFS0_T2IF_MASK; //clear interrupt flag
  IEC0CLR = _IEC0_T2IE_MASK; //disable interrupt on TMR2

  //Configure output compare module
  OC1CONbits.ON = 0; //disable OC1, OC2, OC3 for configuration
  OC2CONbits.ON = 0;
  OC3CONbits.ON = 0;
  OC1CONbits.OC32 = 0; //set OC1, OC2, OC3 for comparison to 16bit timer source
  OC2CONbits.OC32 = 0; 
  OC3CONbits.OC32 = 0; 
  OC1CONbits.OCTSEL = 0; //set OC1, OC2, OC3 that Timer2 is the source for OC
  OC2CONbits.OCTSEL = 0; 
  OC3CONbits.OCTSEL = 0; 
  OC1CONbits.OCM = 0b110; //set PWM mode on OC1, OC2, OC3 and disable fault pin
  OC2CONbits.OCM = 0b110;
  OC3CONbits.OCM = 0b110;

  //Finish configuration, enable Timer and Output Compare Module
  T2CONbits.ON = 1; //enable Timer2
  OC1CONbits.ON = 1; //enable OC1, OC2, OC3
  OC2CONbits.ON = 1;
  OC3CONbits.ON = 1;

  //Initialize PWM with zero rotation
  OC1R = PR*PWM_OFF/100;
  OC2R = PR*PWM_OFF/100;
  OC3R = PR;
  OC1RS = PR*PWM_OFF/100;
  OC2RS = PR*PWM_OFF/100;
  OC3RS = PR;
  
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
    case InitPState:       
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        CurrentState = Driving;
        // CurrentState = Pairing;
      }
    }
    break;

    case Pairing:
    {
      if(ThisEvent.EventType == ES_PAIRED)
      {
        CurrentState = Idle;
        PairingStateIndicator();
      }
    }
    break;

    case Idle:
    {
      switch (ThisEvent.EventType)
      {
        case ES_COMMAND:
        {
          CurrentState = Driving;
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

    case Driving:  
    {
      switch (ThisEvent.EventType)
      {
        case ES_COMMAND:
        {
          PWMUpdate(Velocity, Omega);
        }
        break;

        case ES_DUMP:
        {
          if(ThisEvent.EventParam == 1)
          {
            OC3RS = PR*SERVO_OPEN_POS/100;
          }
          if(ThisEvent.EventParam == 0)
          {
            OC3RS = PR*SERVO_CLOSE_POS/100;
          }
        }
        break;

        case ES_NOPWR:
        {
          PWMUpdate(0, 0);
          CurrentState = Idle;
        }
        break;

        case ES_UNPAIRED:
        {
          PWMUpdate(0, 0);
          CurrentState = Pairing;
        }
        break;

        case ES_TIMEOUT:
        {
          if(ThisEvent.EventParam == AD_TIMER)
          {
            PWMUpdate(Velocity, Omega);
            DB_printf("\rPWM update\n");
            Servo_POS = PORTAbits.RA4;
            if(Servo_POS == 1)
            {
              OC3RS = PR*SERVO_OPEN_POS/100;
            }
            if(Servo_POS == 0)
            {
              OC3RS = PR*SERVO_CLOSE_POS/100;
            }
          }
        }
        break;

        default:
          break;
      }
    }
    break;

    default:
      break;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

void PWMUpdate(uint8_t Velocity, uint8_t Omega)
{
  ScaledLeft = Velocity - (Omega - 127) * TURN_WEIGHT;
  ScaledRight = Velocity + (Omega - 127) * TURN_WEIGHT;
  if(ScaledLeft > 255)
  {
    ScaledLeft = 255;
  }
  if(ScaledRight > 255)
  {
    ScaledRight = 255;
  }
  if(ScaledLeft < 0)
  {
    ScaledLeft = 0;
  }
  if(ScaledRight < 0)
  {
    ScaledRight = 0;
  }
  
  // uint8_t ADControl = GetScaledPotentialValue();
  uint8_t PWMLeft = PWM_MIN + (PWM_MAX - PWM_MIN) * ScaledLeft/255;
  uint8_t PWMRight = PWM_MIN + (PWM_MAX - PWM_MIN) * ScaledRight/255;
  OC1RS = PR * PWMLeft/100;
  OC2RS = PR * PWMRight/100;
}

uint8_t BoundaryCheck(uint8_t Value)
{
  Value = (Value<255)?Value:255;
  Value = (Value>0)?Value:0;
  return Value;
}

void PairingStateIndicator()
{
  
}