/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/

#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ControllerComm.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
#define ONE_SEC 1000
#define ONEFIFTH_SEC (ONE_SEC / 5)
#define FRAME_LEN 13

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
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitControllerComm(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // pins I/O setup
  puts("\r Entered InitControllerComm \r");
  // Setup UART module
  SetupUART(); 
  __builtin_enable_interrupts(); // enable interrupts globally
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
     PostTemplateService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostControllerComm(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunControllerComm(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch (ThisEvent.EventType) {
    case ES_INIT:
    {
      ES_Timer_InitTimer(CTRLCOMM_TIMER, ONEFIFTH_SEC);
      puts("Service 00:");
      DB_printf("\rES_INIT received in Service %d\r\n", MyPriority);
    }
    break; 
    case ES_TIMEOUT:
      if (ThisEvent.EventParam == CTRLCOMM_TIMER) { 
        // 1. Build and send txFrame
        SendFrame(txFrame, FRAME_LEN);
        // 2. Restart 200ms timer
        ES_Timer_InitTimer(CTRLCOMM_TIMER, ONEFIFTH_SEC);
      }
    break;
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

void SetupUART() {
  // Disable UART initially
  U2MODEbits.ON = 0; 
  // Set RB11 to be digital input; Map the U2RX to RB11
  TRISBbits.TRISB11 = 1;
  U2RXR = 0b0011;
  // Set RB10 to be digital output; Map the U2TX to RB10
  TRISBbits.TRISB10 = 0;
  RPB10R = 0b0010;
  // Clear SIDL, IREN, RTSMD, UEN, WAKE, LPBACK, ABAUD, RXINV bits
  U2MODEbits.SIDL = 0;
  U2MODEbits.IREN = 0;
  U2MODEbits.RTSMD = 0; 
  U2MODEbits.WAKE = 0;
  U2MODEbits.LPBACK = 0;
  U2MODEbits.ABAUD = 0;
  U2MODEbits.RXINV = 0;
  // Set high speed baud clock
  U2MODEbits.BRGH = 0;
  // Configure CTS and RTS
  U2MODEbits.UEN = 0b00;
  // Set the BRG register to the value required for 9600 baud
  U2BRG = 129;
  // Set 8-bit mode, no parity, and 1 stop bit
  U2MODEbits.PDSEL = 0b00;
  U2MODEbits.STSEL = 0;
  // Configure the UxSTA register to clear the UTXINV, UTXBRK & ADDEN bits
  U2STAbits.UTXINV = 0;
  U2STAbits.UTXBRK = 0;
  U2STAbits.ADDEN = 0;
  // Disable Automatic Address Detect Mode
  U2STAbits.ADM_EN = 0;
  // Configure UTXISEL & URXISEL
  U2STAbits.URXISEL = 0b00; // interrupt on every character received
  U2STAbits.UTXISEL = 0b10; // interrupt while TX buffer is empty
  // Clear URXEN bit and set UTXEN bit (Transmit byte)
  U2STAbits.UTXEN = 1;
  U2STAbits.URXEN = 1;
  // Clear the TX and RX registers
  U2TXREG;
  U2RXREG;
  // setup Rx interrupts
  INTCONbits.MVEC = 1; // enable multi-vector interrupts
  IPC9bits.U2IP = 7; // Ux2 priority 7, highest priority
  IPC9bits.U2IS = 3; // Ux2 subpriority 3, highest subpriority
  IFS1CLR = _IFS1_U2RXIF_MASK; // clear U2Rx flag
  IFS1CLR = _IFS1_U2TXIF_MASK; // clear U2Tx flag
  IEC1SET = _IEC1_U2RXIE_MASK; // enable U2Rx interrupt locally
  // Enable the UART
  U2MODEbits.ON = 1;
}

void SendFrame(const uint8_t *frame, uint8_t len) {
    for (int i = 0; i < len; i++) {
        while (!U2STAbits.TRMT); // Wait until Transmit Register is empty
        U2TXREG = frame[i];
    }
}

// TODO: Implement checkSum function? 
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

