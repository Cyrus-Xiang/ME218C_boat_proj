/****************************************************************************

  Header file for I2C service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServI2C_H
#define ServI2C_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitI2CService(uint8_t Priority);
bool PostI2CService(ES_Event_t ThisEvent);
ES_Event_t RunI2CService(ES_Event_t ThisEvent);

#endif /* ServI2C_H */

