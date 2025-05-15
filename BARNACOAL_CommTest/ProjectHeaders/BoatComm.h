/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef BoatComm_H
#define BoatComm_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"   
#include "ES_Types.h"

// Public Function Prototypes
typedef enum
{
  InitPState, Receiving, Transmitting
}UARTState_t;

bool InitBoatComm(uint8_t Priority);
bool PostBoatComm(ES_Event_t ThisEvent);
ES_Event_t RunBoatComm(ES_Event_t ThisEvent);

// Private Function Prototypes
void SetupUART();
//void SendFrame(const uint8_t *frame, uint8_t len);
//void ProcessUARTByte(uint8_t byte)
#endif /* ServTemplate_H */

