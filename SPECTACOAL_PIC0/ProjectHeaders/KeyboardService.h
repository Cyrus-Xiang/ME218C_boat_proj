/****************************************************************************

  Header file for Keyboard service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ServKeyboard_H
#define ServKeyboard_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitKeyboardService(uint8_t Priority);
bool PostKeyboardService(ES_Event_t ThisEvent);
ES_Event_t RunKeyboardService(ES_Event_t ThisEvent);

#endif /* ServKeyboard_H */

