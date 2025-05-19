/****************************************************************************
 Module
   controllerFSM.c

 Revision
   1.0.1

 Description
   This is a controller file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RuncontrollerSM()
 10/23/11 18:20 jec      began conversion from SMcontroller.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "controllerFSM.h"
#include "ControllerComm.h"
#include "PIC32_AD_Lib.h"
#include "PWM_PIC32.h"
#include "dbprintf.h"
#include <xc.h>
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void adjust_7seg(uint8_t digit_input);
// functions related to configuration
static void config_joystick_ADC(void);
static void config_buttons(void);
static void config_shift_reg(void);
static void config_charge_indicator(void);
// functions related to state transitions
static void enterDriveMode_s(void);
static void exitDriveMode_s(void);
static void enterChargeMode_s(void);
static void exitChargeMode_s(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static controllerState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
#define ADC_scan_interval 200
static uint8_t MyPriority;
static uint32_t Curr_AD_Val[2];
// variables for the wireless communication
static uint8_t boat_selected = 6; // default to boat 6
static uint8_t max_boat_number = 6;
const static uint8_t boat_addresses_LSB[6] = {0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B};

/*
uint8_t txFrame[] = {
  0x7E,          // Start delimiter
  0x00, 0x09,    // Length (MSB, LSB) = 8 bytes of data after this field
  0x01,          // Frame type = TX (16-bit address)
  0x00,          // Frame ID (0 = no ACK)
  0x20, 0x86,    // TEST: Destination address = 0x2086
  0x01,          // Options = 0x01 to disable ACK
  0x02, 0x00, 0x00, 0x00,// TEST: Pairing: 0x02 (byte 9), 0x00 (byte 10), 0x00 (byte 11), 0x00 (byte 12)
  0x55           // Checksum (computed as 0xFF - sum of bytes after 0x7E)
};
*/
// variables for the 7 seg display
#define SRCLK_port LATAbits.LATA0 // clock pin for SN74HC595 shift register
#define RCLK_port LATAbits.LATA1  // latch pin for SN74HC595 shift register
#define SER_port LATAbits.LATA2   // data pin for SN74HC595 shift register
#define SHORT_DELAY() asm volatile("nop; nop; nop; nop")
// 7-segment patterns (common cathode)
// a–g, dp: MSB = a, LSB = dp
const uint8_t seg_table[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// variables for battery(charge) level indication (servo)
#define charge_byte_full 150
#define charge_update_interval 300 // 300ms
#define OC_channel_4_servo 1
#define PWM_period_us 20000
#define PWM_freq 50 // 50Hz
#define lower_PW_us 500
#define upper_PW_us 2500
#define ticks_per_us 2.5
static uint16_t PW_range_us;
static uint16_t PulseWidth_servo_us;
extern uint8_t powerByte; //set in comm service
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitcontrollerFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitcontrollerFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = Idle_s;
  // configure pins and ADC for X Y information of joysticks
  config_joystick_ADC();
  config_buttons();
  config_shift_reg();
  config_charge_indicator();
  adjust_7seg(boat_selected); // display 8 on the 7-segment display
  DB_printf("controllerFSM successfully initialized\n");
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostcontrollerFSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostcontrollerFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RuncontrollerFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RuncontrollerFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  // executed regardless of the state
  if (ThisEvent.EventType == ES_TIMEOUT)
  {
    if (ThisEvent.EventParam == JoystickScan_TIMER)
    {
      // read the joystick values
      ADC_MultiRead(Curr_AD_Val);
      // update the joystick values in the txFrame
      txFrame[joy_x_byte] = (uint8_t)(Curr_AD_Val[0] >> 2); // right shift to get 8 bits (divide by 4)
      txFrame[joy_y_byte] = (uint8_t)(Curr_AD_Val[1] >> 2); // right shift to get 8 bits (divide by 4)
      DB_printf("joystick X: %d Y: %d\n", txFrame[joy_x_byte], txFrame[joy_y_byte]);
      ES_Timer_InitTimer(JoystickScan_TIMER, ADC_scan_interval);
    }
    if (ThisEvent.EventParam == ServoUpdate_TIMER)
    {
      DB_printf("charge byte is %d\n", powerByte);
      if (powerByte <= charge_byte_full)
      {
        PulseWidth_servo_us = upper_PW_us - (float)(powerByte * PW_range_us / charge_byte_full);
        PWMOperate_SetPulseWidthOnChannel(PulseWidth_servo_us * ticks_per_us, OC_channel_4_servo);
        DB_printf("servo pulse width is set to %u us\n", PulseWidth_servo_us);
        ES_Timer_InitTimer(ServoUpdate_TIMER, charge_update_interval);
      }else {
        DB_printf("charge byte is out of range so we don't update servo\n");
      }
    }
  }

  switch (CurrentState)
  {
  case Idle_s:
  {
    if (ThisEvent.EventType == ES_CHOOSE_BOAT_BUTTON_PRESSED)
    {
      if (boat_selected < max_boat_number)
      {
        boat_selected++;
      }
      else
      {
        boat_selected = 1;
      }
      adjust_7seg(boat_selected);
      // update the boat number in the txFrame
      txFrame[dst_addr_lsb_byte] = boat_addresses_LSB[boat_selected - 1];
      DB_printf("boat address is locked to %d selected\n", txFrame[dst_addr_lsb_byte]);
    }
    else if (ThisEvent.EventType == ES_PAIR_BUTTON_PRESSED)
    {
      CurrentState = Pairing_s;
      // update the pairing status byte in txFrame
      txFrame[status_byte] = pairing_status_msg;
      DB_printf("start pairing with boart number %d\n", boat_selected);
    }
  }
  break;

  case Pairing_s: // If current state is state one
  {
    if (ThisEvent.EventType == ES_BOAT_PAIRED)
    {
      CurrentState = DriveMode_s;
      enterDriveMode_s();
      DB_printf("Pairing successful, entering Drive Mode\n");
    }
  }
  break;
  case DriveMode_s:
  {
    if (ThisEvent.EventType == ES_DROP_COAL_BUTTON_PRESSED)
    {
      DB_printf("Drop coal event received\n");
    }
    else if (ThisEvent.EventType == ES_DROP_ANCHOR_BUTTON_PRESSED)
    {
      DB_printf("Drop anchor event received\n");
    }
    else if (ThisEvent.EventType == ES_IMU_ORIENTATION_SWITCH && ThisEvent.EventParam == 1)
    {
      // event param of 1 means upside down
      CurrentState = ChargeMode_s;
      exitDriveMode_s();
      enterChargeMode_s();
      DB_printf("switch from DriveMode to ChargeMode\n");
    }
  }
  break;
  case ChargeMode_s:
  {
    if (ThisEvent.EventType == ES_IMU_ORIENTATION_SWITCH && ThisEvent.EventParam == 0)
    {
      // event param of 0 means right side up
      CurrentState = DriveMode_s;
      enterDriveMode_s();
      DB_printf("switch from ChargeMode to DriveMode\n");
    }
  }
  break;
  default:
  {
  }
  break;
  } // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QuerycontrollerSM

 Parameters
     None

 Returns
     controllerState_t The current state of the controller state machine

 Description
     returns the current state of the controller state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
controllerState_t QuerycontrollerFSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void config_charge_indicator(void)
{
  TRISBbits.TRISB15 = 0; // output for OC1
  ANSELBbits.ANSB15 = 0; // digital pin
  PWMSetup_BasicConfig(OC_channel_4_servo);
  // PWMSetup_SetPeriodOnTimer(PWM_period_us * ticks_per_us, _Timer2_);
  PWMSetup_SetFreqOnTimer(PWM_freq, _Timer2_); // set the PWM frequency
  DB_printf("servo PWM frequency is set to %u Hz \n", PWM_freq);
  PWMSetup_AssignChannelToTimer(OC_channel_4_servo, _Timer2_);
  PWMSetup_MapChannelToOutputPin(OC_channel_4_servo, PWM_RPB15);
  PW_range_us = upper_PW_us - lower_PW_us;
  // set intial charge to 0
  PulseWidth_servo_us = upper_PW_us; // upper = leftmost servo position
  PWMOperate_SetPulseWidthOnChannel(PulseWidth_servo_us * ticks_per_us, OC_channel_4_servo);
  ES_Timer_InitTimer(ServoUpdate_TIMER, charge_update_interval);
  return;
}
static void config_shift_reg(void)
{
  // configure the shift register pins
  TRISAbits.TRISA0 = 0; // SRCLK,
  TRISAbits.TRISA1 = 0; // RCLK, latch clock
  TRISAbits.TRISA2 = 0; // SER
  // set the pins to low
  SRCLK_port = 0;
  RCLK_port = 0;
  SER_port = 0;

  return;
}
static void config_joystick_ADC(void)
{
  TRISBbits.TRISB12 = 1;
  ANSELBbits.ANSB12 = 1;
  TRISBbits.TRISB13 = 1;
  ANSELBbits.ANSB13 = 1;
  ADC_ConfigAutoScan(BIT11HI | BIT12HI); // AN11 is for B13. X pos of joystick; AN12 is for B12, Y pos of joystick

  return;
}

static void config_buttons(void)
{
  TRISAbits.TRISA4 = 1;
  TRISBbits.TRISB4 = 1;
  TRISBbits.TRISB9 = 1;
  return;
}

static void adjust_7seg(uint8_t digit_input)
{
  // convert the digit to the corresponding 7-segment pattern
  uint8_t data = seg_table[digit_input];
  RCLK_port = 0; // Disable latch during shifting
  for (int i = 7; i >= 0; i--)
  {
    SRCLK_port = 0;
    SHORT_DELAY(); // Ensure clock low period
    SER_port = (data >> i) & 0x01;
    SHORT_DELAY();
    SRCLK_port = 1; // Rising edge shifts in bit
    SHORT_DELAY();
  }
  RCLK_port = 1; // Rising edge latches all bits to output
  DB_printf("7seg is displaying boat number: %d\n", digit_input);
  return;
}

// enter and exit functions of the state machine
static void enterDriveMode_s(void)
{
  DB_printf("Entering Drive Mode\n");
  ES_Timer_InitTimer(JoystickScan_TIMER, ADC_scan_interval); // joystick now is reading values regularly
  txFrame[status_byte] = driving_status_msg;                 // update the pairing status byte in txFrame
  return;
}

static void exitDriveMode_s(void)
{
  DB_printf("Exiting Drive Mode\n");
  ES_Timer_StopTimer(JoystickScan_TIMER);
  return;
}

static void enterChargeMode_s(void)
{
  txFrame[status_byte] = charging_status_msg;  // update the pairing status byte in txFrame
  txFrame[joy_x_byte] = joy_stick_neutral_msg; // set joystick values to neutral
  txFrame[joy_y_byte] = joy_stick_neutral_msg; // set joystick values to neutral
  return;
}
