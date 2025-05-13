/****************************************************************************

  Header file for Thruster service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServThruster_H
#define ServThruster_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitThrusterService(uint8_t Priority);
bool PostThrusterService(ES_Event_t ThisEvent);
ES_Event_t RunThrusterService(ES_Event_t ThisEvent);

#endif /* ServThruster_H */

