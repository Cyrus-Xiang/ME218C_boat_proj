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

#define STATUS_PAIRING 0x02
#define STATUS_CHARGING 0x01
#define STATUS_COMMAND 0x00

extern uint8_t txFrame[]; 
extern uint8_t powerByte; 
// Public Function Prototypes
bool InitControllerComm(uint8_t Priority);
bool PostControllerComm(ES_Event_t ThisEvent);
ES_Event_t RunControllerComm(ES_Event_t ThisEvent);

// Private Function Prototypes
void SetupUART();
void SendFrame();
void ProcessUARTByte(uint8_t byte); 
void ParseAPIFrame();
void printTxFrame();
#endif /* ServTemplate_H */

