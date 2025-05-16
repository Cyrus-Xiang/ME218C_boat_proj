/****************************************************************************
 Module
   TestHarnessService0.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:26 jec     moves definition of ALL_BITS to ES_Port.h
 10/19/17 21:28 jec     meaningless change to test updating
 10/19/17 18:42 jec     removed referennces to driverlib and programmed the
                        ports directly
 08/21/17 21:44 jec     modified LED blink routine to only modify bit 3 so that
                        I can test the new new framework debugging lines on PF1-2
 08/16/17 14:13 jec      corrected ONE_SEC constant to match Tiva tick rate
 11/02/13 17:21 jec      added exercise of the event deferral/recall module
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
// This module
#include "../ProjectHeaders/TestHarnessService0.h"

// debugging printf()

// Hardware
#include <xc.h>
//#include <proc/p32mx170f256b.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 10.000mS/tick timing
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)

#define ENTER_POST     ((MyPriority<<3)|0)
#define ENTER_RUN      ((MyPriority<<3)|1)
#define ENTER_TIMEOUT  ((MyPriority<<3)|2)

#define TEST_INT_POST
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

static void InitLED(void);
static void BlinkLED(void);
#ifdef TEST_INT_POST
static void InitTMR2(void);
static void StartTMR2(void);
#endif
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
// add a deferral queue for up to 3 pending deferrals +1 to allow for overhead
static ES_Event_t DeferralQueue[3 + 1];

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService0

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
bool InitTestHarnessService0(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  // When doing testing, it is useful to announce just which program
  // is running.
  clrScrn();
  puts("\rStarting Test Harness for \r");
  DB_printf( "the 2nd Generation Events & Services Framework V2.4\r\n");
  DB_printf( "compiled at %s on %s\n", __TIME__, __DATE__);
  DB_printf( "\n\r\n");
  DB_printf( "Press any key to post key-stroke events to Service 0\n\r");
  DB_printf( "Press 'd' to test event deferral \n\r");
  DB_printf( "Press 'r' to test event recall \n\r");
  DB_printf( "Press 'p' to test posting from an interrupt \n\r");

  /********************************************
   in here you write your initialization code
   *******************************************/
  // initialize deferral queue for testing Deferal function
  ES_InitDeferralQueueWith(DeferralQueue, ARRAY_SIZE(DeferralQueue));
  // initialize LED drive for testing/debug output
  // InitLED();
#ifdef TEST_INT_POST
  InitTMR2();
#endif
  // initialize the Short timer system for channel A
  //ES_ShortTimerInit(MyPriority, SHORT_TIMER_UNUSED);

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
     PostTestHarnessService0

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostTestHarnessService0(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService0

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunTestHarnessService0(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  static char DeferredChar = '1';

#ifdef _INCLUDE_BYTE_DEBUG_
  _HW_ByteDebug_SetValueWithStrobe( ENTER_RUN );
#endif  
  switch (ThisEvent.EventType)
  {
    case ES_INIT:
    {
      ES_Timer_InitTimer(SERVICE0_TIMER, HALF_SEC);
      puts("Service 00:");
      DB_printf("\rES_INIT received in Service %d\r\n", MyPriority);
    }
    break;
    case ES_TIMEOUT:   // re-start timer & announce
    {
      ES_Timer_InitTimer(SERVICE0_TIMER, FIVE_SEC);
      DB_printf("ES_TIMEOUT received from Timer %d in Service %d\r\n",
          ThisEvent.EventParam, MyPriority);
    }
    break;
    case ES_SHORT_TIMEOUT:   // lower the line & announce
    {
      puts("\rES_SHORT_TIMEOUT received\r\n");
    }
    break;
    case ES_NEW_KEY:   // announce
    {
      DB_printf("ES_NEW_KEY received with -> %c <- in Service 0\r\n",
          (char)ThisEvent.EventParam);
      if ('d' == ThisEvent.EventParam)
      {
        ThisEvent.EventParam = DeferredChar++;   //
        if (ES_DeferEvent(DeferralQueue, ThisEvent))
        {
          puts("ES_NEW_KEY deferred in Service 0\r");
        }
      }
      if ('r' == ThisEvent.EventParam)
      {
        ThisEvent.EventParam = 'Q';   // This one gets posted normally
        PostTestHarnessService0(ThisEvent);
        // but we slide the deferred events under it so it(they) should come out first
        if (true == ES_RecallEvents(MyPriority, DeferralQueue))
        {
          puts("ES_NEW_KEY(s) recalled in Service 0\r");
          DeferredChar = '1';
        }
      }
      if ('p' == ThisEvent.EventParam)
      {
        StartTMR2();
      }
    }
    break;
    default:
    {}
     break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
#define LED LATBbits.LATB6
static void InitLED(void)
{
  LED = 0; //start with it off
  TRISBbits.TRISB6 = 0; // set RB6 as an output
}

static void BlinkLED(void)
{
  // toggle state of LED
  LED = ~LED;
}

#ifdef TEST_INT_POST
#include <sys/attribs.h> // for ISR macors

// for testing posting from interrupts.
// Intializes TMR2 to gerenate an interrupt at 100ms
static void InitTMR2(void)
{
  // turn timer off
  T2CONbits.ON = 0;
  // Use internal peripheral clock
  T2CONbits.TCS = 0;
  // setup for 16 bit mode
  T2CONbits.T32 = 0;
  // set prescale to 1:1
  T2CONbits.TCKPS = 0;
  // load period value
  PR2 = 2000-1; // creates a 100ms period with a 20MHz peripheral clock
  // set priority
  IPC2bits.T2IP = 2;
  // clear interrupt flag
  IFS0bits.T2IF = 0;
  // enable the timer interrupt
  IEC0bits.T2IE = 1;
}

// Clears and Starts TMR2
static void StartTMR2(void)
{
  // clear timer
  TMR2 = 0;
  // start timer
  //LATBbits.LATB14 = 0;
  T2CONbits.ON = 1;
}

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2ISR(void)
{
  // clear flag
  IFS0bits.T2IF = 0;
  // post event
  static ES_Event_t interruptEvent = {ES_SHORT_TIMEOUT, 0};
  PostTestHarnessService0(interruptEvent);
  
  // stop timer
  T2CONbits.ON = 0;
  return;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

