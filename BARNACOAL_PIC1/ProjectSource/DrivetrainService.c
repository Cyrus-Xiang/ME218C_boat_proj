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
//#include "ADService.h"
#include "PowerService.h"
#include "BoatComm.h"

/*----------------------------- Module Defines ----------------------------*/
#define FREQUENCY_PBCLK 20000000 //20MHz
#define FREQUENCY_DESIRED 500 // 2873HZ don't know why, but should be the same as lab 7
#define PRESCALAR 0b010 // 1:4 
#define PERIOD_DESIRED FREQUENCY_PBCLK/FREQUENCY_DESIRED/(4)
#define PERIOD_COUNT PERIOD_DESIRED-1

#define PWM_MIN 60
#define PWM_OFF 75
#define PWM_MAX 90

#define SERVO_OPEN_POS 95
#define SERVO_CLOSE_POS 40

#define ONE_SEC 1000
#define HALF_SEC ONE_SEC/2

#define TURN_WEIGHT 2

#define VELOCITY joystickOneByte
#define OMEGA joystickTwoByte
#define NEUTRAL 127

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
static uint8_t ScaledLeft = 127;
static uint8_t ScaledRight = 127;
static uint8_t PWMLeft = 50;
static uint8_t PWMRight = 50;
static uint8_t Servo_POS = SERVO_CLOSE_POS;
static uint8_t DumpState = 0;


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
  TRISAbits.TRISA2 = 0;
  TRISAbits.TRISA3 = 0;
  RPA0R = 0b0101; //map RA0, RA1, RA3 to output compare mode 
  RPA1R = 0b0101;
  RPA2R = 0b0101;
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
  OC4CONbits.ON = 0;
  OC1CONbits.OC32 = 0; //set OC1, OC2, OC3 for comparison to 16bit timer source
  OC2CONbits.OC32 = 0; 
  OC3CONbits.OC32 = 0; 
  OC4CONbits.OC32 = 0;
  OC1CONbits.OCTSEL = 0; //set OC1, OC2, OC3 that Timer2 is the source for OC
  OC2CONbits.OCTSEL = 0; 
  OC3CONbits.OCTSEL = 0; 
  OC4CONbits.OCTSEL = 0; 
  OC1CONbits.OCM = 0b110; //set PWM mode on OC1, OC2, OC3 and disable fault pin
  OC2CONbits.OCM = 0b110;
  OC3CONbits.OCM = 0b110;
  OC4CONbits.OCM = 0b110;

  //Finish configuration, enable Timer and Output Compare Module
  T2CONbits.ON = 1; //enable Timer2
  OC1CONbits.ON = 1; //enable OC1, OC2, OC3
  OC2CONbits.ON = 1;
  OC3CONbits.ON = 1;
  OC4CONbits.ON = 1;

  //Initialize PWM with zero rotation
  OC1R = PR*PWM_OFF/100;
  OC2R = PR*PWM_OFF/100;
  OC3R = PR;
  OC4R = PR;
  OC1RS = PR*PWM_OFF/100;
  OC2RS = PR*PWM_OFF/100;
  OC3RS = PR*SERVO_CLOSE_POS/100;
  //OC4RS = PR * (2081 - 2080)/8;
  OC4RS = PR * 4.6 / 8; 
  PWMUpdate(NEUTRAL, NEUTRAL);
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
        CurrentState = Pairing;
      }
    }
    break;

    case Pairing:
    {
      //PairingStateIndicator(0x2081);
      PWMUpdate(127, 127); // Disable all actuators
      if(ThisEvent.EventType == ES_PAIRED)            
      {
        PWMUpdate(NEUTRAL, NEUTRAL);
        CurrentState = Idle;
        //PairingStateIndicator(0x2086);
      }
    }
    break;

    case Idle:
    {
      PWMUpdate(127, 127); // Disable all actuators
      switch (ThisEvent.EventType)
      {
        case ES_IDLE:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Idle;
        }
        break; 

        case ES_COMMAND:
        {
          PWMUpdate(VELOCITY, OMEGA);
          CurrentState = Driving;
        }
        break;

        case ES_UNPAIRED:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Pairing;
        }
        break;

        case ES_CHARGE:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Idle;\
          DB_printf("Receive ES_ChARGE in Idle state\r\n");
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
        case ES_IDLE:
        {
          CurrentState = Idle;
        }
        break; 

        case ES_COMMAND:
        {
          PWMUpdate(VELOCITY, OMEGA);
        }
        break;

        case ES_DUMP:
        {
            DB_printf("Detect ES_DUMP in Driving State\r\n");
          if (Power > 0) {
            DumpState = (DumpState + 1) % 2;
            DB_printf("DumpState is = %d\r\n", DumpState);
            if(DumpState == 1)
            {
              OC3RS = PR*SERVO_OPEN_POS/100;
            }
            if(DumpState == 0)
            {
              OC3RS = PR*SERVO_CLOSE_POS/100;
            }
            CurrentState = Driving;
          }
          else {
            DB_printf("Error: DrivetrainService NO Power!!!\r\n");
          }
        }
        break;

        case ES_NOPWR:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = No_Power;
        }
        break;

        case ES_UNPAIRED:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Pairing;
        }
        break;

        case ES_CHARGE:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Idle;
          DB_printf("Receive ES_ChARGE in Driving state\r\n");
        }
        break;
        
        default:
          break;
      }
    }
    break;

    case No_Power:  
    {
      switch (ThisEvent.EventType)
      {
        case ES_UNPAIRED:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Pairing;
        }
        break;

        case ES_CHARGE:
        {
          PWMUpdate(NEUTRAL, NEUTRAL);
          CurrentState = Idle;
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

void PWMUpdate(uint8_t vel, uint8_t om)
{
  DB_printf("vel = %d om = %d\r\n", vel, om);
  if (vel == 127 && om == 127) {
    OC1RS = PR * PWM_OFF/100;
    OC2RS = PR * PWM_OFF/100;
    return;
  }
  else {
    ScaledLeft = vel + (int8_t)(om - 127) / TURN_WEIGHT;
    ScaledRight = vel - (int8_t)(om - 127) / TURN_WEIGHT;

    //
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

    uint8_t PWMLeft = PWM_MIN + (PWM_MAX - PWM_MIN) * ScaledLeft/255;
    uint8_t PWMRight = PWM_MIN + (PWM_MAX - PWM_MIN) * ScaledRight/255;
    DB_printf("PWMLeft = %d PWMRight = %d\r\n", PWMLeft, PWMRight);
    OC1RS = PR * PWMLeft/100;
    OC2RS = PR * PWMRight/100;
    // DB_printf("PWMRight = %d\r\n", PWMRight);
  }
}

uint8_t BoundaryCheck(uint8_t Value)
{
  Value = (Value<255)?Value:255;
  Value = (Value>0)?Value:0;
  return Value;
}


void PairingStateIndicator(uint16_t Address)
{
  /*
  switch sourceAddressLSB:
  {
    case 0x81:

  }
  */
  OC4RS = PR * (Address - 0x2080)/8; // 208x with x = 1-6 by protocol
}
