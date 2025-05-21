/****************************************************************************

  Header file for IMU service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServIMU_H
#define ServIMU_H

#include "ES_Types.h"
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData_t;
// Public Function Prototypes

bool InitIMUService(uint8_t Priority);
bool PostIMUService(ES_Event_t ThisEvent);
ES_Event_t RunIMUService(ES_Event_t ThisEvent);

#endif /* ServIMU_H */

