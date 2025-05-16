/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ControllerComm_H
#define ControllerComm_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"   
#include "ES_Types.h"

extern uint8_t txFrame[]; 
extern uint8_t txFrame2[]; 
// Public Function Prototypes
bool InitControllerComm(uint8_t Priority);
bool PostControllerComm(ES_Event_t ThisEvent);
ES_Event_t RunControllerComm(ES_Event_t ThisEvent);

// Private Function Prototypes
void SetupUART();
void SendFrame();
#endif /* ServTemplate_H */

