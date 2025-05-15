/****************************************************************************

  Header file for AD service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ADService_H
#define ADService_H

#include "ES_Types.h"
#include "ES_Events.h"

// Public Function Prototypes

bool InitADService(uint8_t Priority);
bool PostADService(ES_Event_t ThisEvent);
ES_Event_t RunADService(ES_Event_t ThisEvent);
uint32_t GetStepTime(void);
uint32_t GetScaledPotentialValue(void);

#endif /* ADService_H */

