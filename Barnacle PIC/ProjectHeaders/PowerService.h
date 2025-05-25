/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef PowerService_H
#define PowerService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"

extern uint8_t Power;
// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, Pairing, Idle, Driving, No_Power, Recharging, Power_On
}BarnacleState_t;

// Public Function Prototypes

bool InitPowerService(uint8_t Priority);
bool PostPowerService(ES_Event_t ThisEvent);
ES_Event_t RunPowerService(ES_Event_t ThisEvent);

#endif /* FSMTemplate_H */

