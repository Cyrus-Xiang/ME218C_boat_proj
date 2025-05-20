/****************************************************************************
 Module
   I2CService.c

 Revision
   1.0.1

 Description
   This is a I2C file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from I2CFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "I2CService.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void configSPI(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitI2CService

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
bool InitI2CService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
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
     PostI2CService

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
bool PostI2CService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunI2CService

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
ES_Event_t RunI2CService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void configSPI(void)
{
    // Step 0: Disable analog function on all SPI pins
    ANSELBbits.ANSB14 = 0;
    // Step 1: Map SPI Outputs to all desired pins
    TRISBbits.TRISB4 = 0;
    RPB4R = 0b0011; // Map SS1 to RB4
    //LATBbits.LATB4 = 1; // Pull SS high
    TRISBbits.TRISB8 = 0;
    RPB8R = 0b0011;        // Map SDO to RB8
    TRISBbits.TRISB14 = 0; // Set SCK1 (RB14) as output
    // Step 2: Map SDI
    TRISBbits.TRISB5 = 1; // Input
    SDI1R = 0b0001;       // Map SDI1 to RB5
    // Step 3: Disable SPI Module
    SPI1CONbits.ON = 0;
    // Step 4: Clear the receive buffer
    uint8_t dummpy = SPI1BUF;
    // Step 5: Enable Enhanced Buffer
    SPI1CONbits.ENHBUF = 0;
    // Step 6: Set Baudrate
    SPI1BRG = 1; //10MHz
    // Step 7: Clear the SPIROV Bit
    SPI1STATbits.SPIROV = 0;
    // Step 8: Write desired settings to SPIxCON
    SPI1CONbits.MSTEN = 1;  // Places in Leader Mode
    SPI1CONbits.MSSEN = 0; // manual CS control by software
    SPI1CONbits.CKE = 0;    // Reads on 2nd edge
    SPI1CONbits.CKP = 1;    // SCK idles high
    SPI1CONbits.FRMPOL = 0; // CS is active low
    SPI1CON2bits.AUDEN = 0;
    SPI1CONbits.MODE16 = 0; // Enable 8 bit transfers
    SPI1CONbits.MODE32 = 0; 
    // Step 9: Initialize Interrupts
    SPI1CONbits.SRXISEL = 0b01; // Interrupt when buffer is full
    IFS1CLR = _IFS1_SPI1RXIF_MASK;
    IPC7bits.SPI1IP = 6;
    IEC1SET = _IEC1_SPI1RXIE_MASK;
    // Step 10: Enable SPI
    SPI1CONbits.ON = 1;
    __builtin_enable_interrupts();
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

