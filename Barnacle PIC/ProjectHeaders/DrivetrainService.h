/****************************************************************************

  Header file for Drivetrain Service
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef DrivetrainService_H
#define DrivetrainService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, UnlockWaiting, _1UnlockPress,
  _2UnlockPresses, Locked
}DrivetrainState_t;

// Public Function Prototypes

bool InitDrivetrainService(uint8_t Priority);
bool PostDrivetrainService(ES_Event_t ThisEvent);
ES_Event_t RunDrivetrainService(ES_Event_t ThisEvent);
TemplateState_t QueryDrivetrainService(void);

#endif /*DrivetrainService_H */

