/****************************************************************************
 Module
   UARTService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
//This Module
#include "UARTService.h"

//Hardware
#include <xc.h>

// Event & Services Framework
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/

#define REQUEST_LED LATBbits.LATB13 

#define ONE_SEC (1000) // 1000 ms
#define HALF_SEC ONE_SEC/2 // 500 ms

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static UARTState_t CurrentState;
static volatile uint8_t RxMSG = 0b00000000; // message from Rx
static volatile uint8_t TxMSG = 0b00000000; // message for Tx
static volatile uint8_t InvertBit = 0; // invert bit from RxMSG
static volatile uint8_t SwitchState = 0b000; // message passing between Rx and Tx

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitUARTService

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
bool InitUARTService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;

  DB_printf("Init UART Service\r\n");

  // set pin input and output
  TRISBbits.TRISB8 = 1; // UART Rx, set input pin
  TRISBbits.TRISB9 = 0; // UART Tx, set output pin
  TRISBbits.TRISB13 = 0; // request LED toggle pin
  U2RXR = 0b0100; // assign RB8 to U2Rx
  RPB9R = 0b0010; // assign RB9 to U2Tx

  // UART setup
  U2MODEbits.ON = 0; // disable UART for setup
  U2MODEbits.SIDL = 0; // continue operation in Idle mode
  U2MODEbits.IREN = 0; // IRDA disable
  U2MODEbits.RTSMD = 1; // simplex mode
  U2MODEbits.UEN = 0b00; // use only TX and RX pins
  U2MODEbits.WAKE = 0; // wakeup disabled
  U2MODEbits.LPBACK = 0; // loopback mode disabled
  U2MODEbits.ABAUD = 0; // autobaud disabled
  U2MODEbits.RXINV = 0; // Rx idle high
  U2MODEbits.BRGH = 0; // standard speed mode
  U2MODEbits.PDSEL = 0b00; // 8-bit data, no parity
  U2MODEbits.STSEL = 0; // 1 stop bit
  
  U2STAbits.UTXINV = 0; // Tx idle high, with IREN = 0
  U2STAbits.UTXBRK = 0; // no break
  U2STAbits.ADDEN = 0; // address detect disabled
  U2STAbits.URXISEL = 0b00; // interrupt on every character received
  U2STAbits.UTXISEL = 0b10; // interrupt while TX buffer is empty
  U2STAbits.UTXEN = 1; // enable UART Tx
  U2STAbits.URXEN = 1; // enable UART Rx

  U2BRG = 129; // 9600 baud rate
//  U2BRG = 10; // 115200 baud rate

  // setup Rx interrupts
  INTCONbits.MVEC = 1; // enable multi-vector interrupts
  IPC9bits.U2IP = 7; // Ux2 priority 7, highest priority
  IPC9bits.U2IS = 3; // Ux2 subpriority 3, highest subpriority
  IFS1CLR = _IFS1_U2RXIF_MASK; // clear U2Rx flag
  IFS1CLR = _IFS1_U2TXIF_MASK; // clear U2Tx flag
  IEC1SET = _IEC1_U2RXIE_MASK; // enable U2Rx interrupt locally

  // enable UART
  U2MODEbits.ON = 1;

  __builtin_enable_interrupts(); // enable interrupts globally

  REQUEST_LED = 0; // turn off LED

  // put us into the Initial PseudoState
  CurrentState = Waiting;
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
     PostUARTService

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
bool PostUARTService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunUARTService

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
ES_Event_t RunUARTService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case Waiting:        // If current state is initial Psedudo State
    {
      switch(ThisEvent.EventType)
      {
        case SW1_UP:
        {
          SwitchState |= (1<<0); // set bit 0 for LED1 on
          DB_printf("current switch = %d\r\n", SwitchState);
        }
        break;
        case SW1_DOWN:
        {
          SwitchState &= ~(1<<0); // clear bit 0 for LED1 off
          DB_printf("current switch = %d\r\n", SwitchState); 
        }
        break;
        case SW2_UP:
        {
          SwitchState |= (1<<1); // set bit 0 for LED2 on
          DB_printf("current switch = %d\r\n", SwitchState); 
        }
        break;
        case SW2_DOWN:
        {
          SwitchState &= ~(1<<1); // clear bit 0 for LED2 off  
          DB_printf("current switch = %d\r\n", SwitchState); 
        }
        break;
        case SW3_UP:
        {
          SwitchState |= (1<<2); // set bit 0 for LED3 on   
          DB_printf("current switch = %d\r\n", SwitchState);
        }
        break;
        case SW3_DOWN:
        {
          SwitchState &= ~(1<<2); // clear bit 0 for LED3 off    
          DB_printf("current switch = %d\r\n", SwitchState);
        }
        break;
        case ES_TIMEOUT:
        {
          if (ThisEvent.EventParam == LED_TIMER)
          {
            REQUEST_LED = 0; // turn off request LED
          }
        }
        break;
        case RX_DONE:
        {
          CurrentState = Transmitting;
        }
        break;
      }
    }
    break;

    case Transmitting:
    {
      DB_printf("enter Tx\r\n");
      REQUEST_LED = 1; // turn on request LED 
      ES_Timer_InitTimer(LED_TIMER, HALF_SEC); // start LED timer 
      InvertBit = RxMSG & (1<<3);
      TxMSG = InvertBit | SwitchState;
      DB_printf("TxMSG = %d\r\n", TxMSG);
      U2TXREG = TxMSG; // put the mesg into Tx register
      CurrentState = Waiting; // go to waiting state
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/

/***************************************************************************
 private functions
 ***************************************************************************/
void __ISR(_UART_2_VECTOR, IPL7SOFT) U2RxISR(void)
{
  IFS1CLR = _IFS1_U2RXIF_MASK; // Clear Rx interrupt flag
  RxMSG = U2RXREG; // Read U2RX register into Message
  CurrentState = Transmitting;

  ES_Event_t RxEvent;
  RxEvent.EventType = RX_DONE;
  RxEvent.EventParam = RxMSG;
  PostUARTService(RxEvent);
}
