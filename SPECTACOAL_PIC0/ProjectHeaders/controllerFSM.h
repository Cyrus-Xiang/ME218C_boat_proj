/****************************************************************************

  Header file for controller Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FSMcontroller_H
#define FSMcontroller_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, UnlockWaiting, _1UnlockPress,
  _2UnlockPresses, Locked
}controllerState_t;

// Public Function Prototypes

bool InitcontrollerFSM(uint8_t Priority);
bool PostcontrollerFSM(ES_Event_t ThisEvent);
ES_Event_t RuncontrollerFSM(ES_Event_t ThisEvent);
controllerState_t QuerycontrollerSM(void);

#endif /* FSMcontroller_H */

