/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"
#include "dbprintf.h"
#include <string.h>
// This is the event checking function sample. It is not intended to be
// included in the module. It is only here as a sample to guide you in writing
// your own event checkers
#if 0
/****************************************************************************
 Function
   Check4Lock
 Parameters
   None
 Returns
   bool: true if a new event was detected
 Description
   Sample event checker grabbed from the simple lock state machine example
 Notes
   will not compile, sample only
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Lock(void)
{
  static uint8_t  LastPinState = 0;
  uint8_t         CurrentPinState;
  bool            ReturnVal = false;

  CurrentPinState = LOCK_PIN;
  // check for pin high AND different from last time
  // do the check for difference first so that you don't bother with a test
  // of a port/variable that is not going to matter, since it hasn't changed
  if ((CurrentPinState != LastPinState) &&
      (CurrentPinState == LOCK_PIN_HI)) // event detected, so post detected event
  {
    ES_Event ThisEvent;
    ThisEvent.EventType   = ES_LOCK;
    ThisEvent.EventParam  = 1;
    // this could be any of the service post functions, ES_PostListx or
    // ES_PostAll functions
    ES_PostAll(ThisEvent);
    ReturnVal = true;
  }
  LastPinState = CurrentPinState; // update the state for next time

  return ReturnVal;
}

#endif

/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady()) // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_NEW_KEY;
    ThisEvent.EventParam = GetNewKey();
    PostKeyboardService(ThisEvent);
    return true;
  }
  return false;
}

bool Check4Buttons()
{
#define debounce_time 70
#define numOfButtons 3
  bool toReturn = false;
  volatile uint32_t *port_bit[numOfButtons] = {&PORTA, &PORTB, &PORTB}; // A4, B4, B9
  static uint32_t PortMasks[numOfButtons] = {1L << 4, 1L << 4, 1L << 9};
  static ES_EventType_t corresponding_events[numOfButtons] = { ES_CHOOSE_BOAT_BUTTON_PRESSED, ES_PAIR_BUTTON_PRESSED,
  ES_DROP_COAL_BUTTON_PRESSED, };//events for these buttons
  static bool button_states_last[numOfButtons] = {0};
  static bool button_states_curr[numOfButtons] = {0};
  static uint16_t last_button_down_time[numOfButtons] = {0}; // ES_Timer_GetTime() returns uint16_t
  static uint16_t time_now = 0;

  // read the buttons and check ups and downs
  for (int i = 0; i < numOfButtons; i++)
  {
    button_states_curr[i] = (*port_bit[i] & PortMasks[i]);
    // HIGH = button down, LOW = button up
    // event is posted when the button is released (LOW)
    if (button_states_curr[i] && !button_states_last[i])
    {
      last_button_down_time[i] = ES_Timer_GetTime(); // update the time
      //DB_printf("Button#%d is down at time %d\n", i,last_button_down_time[i]);
    }
    // button event is only possible when the current reading is LOW(released) and previous is HIGH(pressed)
    else if (!button_states_curr[i] && button_states_last[i])
    {
      time_now = ES_Timer_GetTime();
      //DB_printf("Button#%d is up at time %d\n", i, time_now);
      if (time_now - last_button_down_time[i] > debounce_time)
      {
        DB_printf("Button#%d press event sent to controllerFSM\n", i);
        ES_Event_t ThisEvent;
        ThisEvent.EventType = corresponding_events[i];
        PostcontrollerFSM(ThisEvent);
        toReturn = true;
      }
    }
  }

  memcpy(button_states_last, button_states_curr, sizeof(button_states_last));
  return toReturn;
}
