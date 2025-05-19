/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef UARTService_H
#define UARTService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  Transmitting, Waiting
}UARTState_t;

// Public Function Prototypes

bool InitUARTService(uint8_t Priority);
bool PostUARTService(ES_Event_t ThisEvent);
ES_Event_t RunUARTService(ES_Event_t ThisEvent);

#endif /* FSMTemplate_H */

