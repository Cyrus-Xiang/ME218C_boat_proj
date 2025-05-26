/****************************************************************************

  Header file for Drivetrain Service
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef DrivetrainService_H
#define DrivetrainService_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
#include "ES_Events.h"

// typedefs for the states
// State definitions for use with the query function
// typedef enum
// {
//    
// }DrivetrainState_t;

// Public Function Prototypes

bool InitDrivetrainService(uint8_t Priority);
bool PostDrivetrainService(ES_Event_t ThisEvent);
ES_Event_t RunDrivetrainService(ES_Event_t ThisEvent);
void PWMUpdate(uint8_t vel, uint8_t om);
uint8_t BoundaryCheck(uint8_t Value);
void PairingStateIndicator(uint16_t Address);

#endif /*DrivetrainService_H */

