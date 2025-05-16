/****************************************************************************
 Module
   KeyboardService.c

 Revision
   1.0.1

 Description
   This is a Keyboard file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from KeyboardFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "KeyboardService.h"
#include "dbprintf.h"
#include "controllerFSM.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
extern uint8_t txFrame[]; // frame to send to the boat
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitKeyboardService

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
bool InitKeyboardService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  DB_printf("KeyboardService initialized\r\n");
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
     PostKeyboardService

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
bool PostKeyboardService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunKeyboardService

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
ES_Event_t RunKeyboardService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  if (ThisEvent.EventType == ES_NEW_KEY)
  {
    DB_printf("Key Pressed: %c\r\n", ThisEvent.EventParam);
    ES_Event_t Event2Post;
    switch (ThisEvent.EventParam)
    {
    case 'a':
      Event2Post.EventType = ES_BOAT_PAIRED;
      PostcontrollerFSM(Event2Post);
      DB_printf("Keyboard service posts ES_BOAT_PAIRED to controllerFSM\n");
      break;
    case 'b':
      for (size_t i = 0; i < 13; i++)
      {
        DB_printf("txFrame[%d] = %d\r\n", i, txFrame[i]);
      }
      break;
    case 'c':
      DB_printf("%d",check_sum_byte);
      break;
    case 'd':

      break;
    case 'e':

      break;
    case 'f':
      /* code */
      break;
    case 'g':
      /* code */
      break;
    case 'h':
      /* code */
      break;
    case 'i':

      break;
    case 'j':

      break;
    case 'k':

      break;
    case 'l':

      break;
    case 'm':
      /* code */
      break;
    case 'n':
      /* code */
      break;
    case 'o':
      /* code */
      break;
    case 'p':
      /* code */
      break;
    case 'q':
      /* code */
      break;
    case 'r':
      /* code */
      break;
    case 's':

      break;
    case 't':
      /* code */
      break;
    case 'u':
      /* code */
      break;
    case 'v':
      /* code */
      break;
    case 'w':

      break;
    case 'x':

      break;
    case 'y':
      /* code */
      break;
    case 'z':
      /* code */
      break;
    case '0':

      break;
    case '1':

      break;
    case '2':

      break;
    case '3':

      break;
    case '4':

      break;
    case '5':

      break;
    case '6':

      break;
    case '7':

      break;
    case '8':

      break;
    case '9':
      /* code */
      break;
    case 'A':

      break;
    case 'B':
      /* code */
      break;
    case 'C':
      /* code */
      break;
    case 'D':
      /* code */
      break;
    case 'E':
      /* code */
      break;
    case 'F':
      /* code */
      break;
    case 'G':
      /* code */
      break;
    case 'H':
      /* code */
      break;
    case 'I':
      /* code */
      break;
    case 'J':
      /* code */
      break;
    case 'K':
      /* code */
      break;
    case 'L':
      /* code */
      break;
    case 'M':
      /* code */
      break;
    case 'N':
      /* code */
      break;
    case 'O':
      /* code */
      break;
    case 'P':
      /* code */
      break;
    case 'Q':
      /* code */
      break;
    case 'R':
      /* code */
      break;
    case 'S':
      /* code */
      break;
    case 'T':
      /* code */
      break;
    case 'U':
      /* code */
      break;
    case 'V':
      /* code */
      break;
    case 'W':
      /* code */
      break;
    case 'X':
      /* code */
      break;
    case 'Y':
      /* code */
      break;
    case 'Z':
      /* code */
      break;

    default:
      break;
    }
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
