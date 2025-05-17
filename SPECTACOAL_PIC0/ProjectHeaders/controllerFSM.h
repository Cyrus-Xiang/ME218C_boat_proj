/****************************************************************************

  Header file for controller Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef FSMcontroller_H
#define FSMcontroller_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */
// communication defines
#define pairing_status_msg 0x02
#define driving_status_msg 0x00
#define charging_status_msg 0x01
#define joy_stick_neutral_msg 0x00

enum{
delimiter_byte, length_msb_byte, length_lsb_byte,
frame_type_byte,frame_id_byte,dst_addr_msb_byte, dst_addr_lsb_byte,options_byte,
status_byte,joy_x_byte,joy_y_byte,buttons_byte,check_sum_byte,
};
// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  Idle_s, Pairing_s, DriveMode_s, ChargeMode_s
}controllerState_t;

// Public Function Prototypes

bool InitcontrollerFSM(uint8_t Priority);
bool PostcontrollerFSM(ES_Event_t ThisEvent);
ES_Event_t RuncontrollerFSM(ES_Event_t ThisEvent);
controllerState_t QuerycontrollerSM(void);

#endif /* FSMcontroller_H */

