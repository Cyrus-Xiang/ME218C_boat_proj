/****************************************************************************

  Header file for IMU service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServIMU_H
#define ServIMU_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitIMUService(uint8_t Priority);
bool PostIMUService(ES_Event_t ThisEvent);
ES_Event_t RunIMUService(ES_Event_t ThisEvent);

#endif /* ServIMU_H */

