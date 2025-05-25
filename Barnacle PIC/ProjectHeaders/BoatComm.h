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

// Payload variables
extern uint8_t sourceAddressMSB; // rxIndex = 4
extern uint8_t sourceAddressLSB; // rxIndex = 5
extern uint8_t statusByte; // rxIndex = 8
extern uint8_t joystickOneByte; // rxIndex = 9
extern uint8_t joystickTwoByte; // rxIndex = 10
extern uint8_t buttonByte; // rxIndex = 11

// Public Function Prototypes
typedef enum
{
  InitState, Receiving, Transmitting
}UARTState_t;

bool InitBoatComm(uint8_t Priority);
bool PostBoatComm(ES_Event_t ThisEvent);
ES_Event_t RunBoatComm(ES_Event_t ThisEvent);

// Private Function Prototypes
void SetupUART();
void ProcessUARTByte(uint8_t byte);
void ParseAPIFrame();
void updateTxFrame();
void SendFrame();
#endif /* ServTemplate_H */

