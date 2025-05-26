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
#include "controllerFSM.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include <sys/attribs.h>
#include <stdlib.h>

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
#define ONE_SEC 1000
#define FOUR_SEC (4*ONE_SEC)
#define ONEFIFTH_SEC (ONE_SEC / 5)
#define FRAME_LEN_TX 13
#define FRAME_LEN_RX 10
#define IS_PAIRED_CHARGE 0xFF

#define NO_BUTTON_PRESSED 0x00
#define DUMP_BUTTON_PRESSED 0x01
#define ANCHOR_BUTTON_PRESSED 0x02
#define BOTH_BUTTON_PRESSED 0x03

#define JOYSTICK_NEUTRAL 127
#define JOYSTICK_DEADZONE_MARGIN 15

uint8_t txFrame[] = {
  0x7E,          // Start delimiter
  0x00, 0x09,    // Length (MSB, LSB) = 8 bytes of data after this field
  0x01,          // Frame type = TX (16-bit address)
  0x00,          // Frame ID (0 = no ACK)
  0xFF, 0xFF,    // Destination address = 0xFFFF (uninitialized address)
  0x01,          // Options = 0x01 to disable ACK
  0x00, 0x7F, 0x7F, 0x00,// TEST: Pairing: 0x02 (byte 9), 0x00 (byte 10), 0x00 (byte 11), 0x00 (byte 12)
  0x00           // Checksum (computed as 0xFF - sum of bytes after 0x7E)
};

// Module variables
static bool isPaired = false;
static bool hasAnchorMSGSent = false;
static bool hasDumpMSGSent = false;
static uint8_t buttonMSGCounter = 0; 
static uint8_t MyPriority;

// Variables For Receiving
static volatile uint8_t rxByte = 0xFF; // default to 0xFF
static volatile uint8_t rxBuffer[FRAME_LEN_RX];                                           
static volatile uint8_t rxIndex = 0;
static volatile uint16_t expectedLength = 0;
static volatile bool isReceiving = false;

// Payload variables
static uint8_t targetAddressMSB = 0xFF; // Modified based on input from ControllerFSM.c
static uint8_t targetAddressLSB = 0xFF; // Modified based on input from ControllerFSM.c
static uint8_t sourceAddressMSB = 0xFF; // Modified based on incoming packet from boat
static uint8_t sourceAddressLSB = 0xFF; // Modified based on incoming packet from boat
uint8_t powerByte = 0x00; // default to charge 0


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
    {
      if (ThisEvent.EventParam == CTRLCOMM_TIMER) { 
        // 1. Build and send txFrame if targetAddress has been modified
        //printTxFrame();
        if (!isPaired) {
          if (txFrame[5] != 0xFF && txFrame[6] != 0xFF) {
            // User has selected the boat, but hasn't pressed 'sent' button
            if (txFrame[8] == STATUS_PAIRING) {
              // User pressed 'sent' button, keep sending pairing request
              targetAddressMSB = txFrame[5];
              targetAddressLSB = txFrame[6];
              SendFrame();
              //DB_printf("Sent pairing request\r\n");
            }
            else {
              // Wait until user press 'sent' button
            }
          }
          else {
            // User hasn't select which boat to pair to
          }
        }
        else { // isPaired
          // check txFrame[9] and txFrame[10] respectively
          setDeadZone();
          if (txFrame[11] != NO_BUTTON_PRESSED) {
            SendFrame();
            DB_printf("Reset buttonByte\r\n");
            txFrame[11] = NO_BUTTON_PRESSED; // Reset buttonByte once sent
          }
          else {
            SendFrame();
          }
          // DB_printf("Sent Frame\r\n");
        }
        // 2. Restart 200ms timer
        ES_Timer_InitTimer(CTRLCOMM_TIMER, ONEFIFTH_SEC);
      }
      else if (ThisEvent.EventParam == UNPAIR_TIMER) {
        // Haven't heard back from Boat 4 sec, post ES_BOAT_UNPAIRED to controllerFSM
        ES_Event_t unpairEvent;
        unpairEvent.EventType = ES_BOAT_UNPAIRED;
        PostcontrollerFSM(unpairEvent); 
        DB_printf("Post Boat unpaired event to controllerFSM\r\n");
      }
      else {
        DB_printf("Error: Unknown type behavior\r\n");
      }
    }
    break;

    case ES_PACKET_IN: // Received packet from Boat
      // Restart 4sec timer
      ES_Timer_InitTimer(UNPAIR_TIMER, FOUR_SEC);
      ParseAPIFrame(); // Sanity check and update sourceAddress and powerByte
      //DB_printf("\rsourceAddressMSB = %d\r\n", sourceAddressMSB);
      //DB_printf("\rsourceAddressLSB = %d\r\n", sourceAddressLSB);
      DB_printf("\rpowerByte = %d\r\n", powerByte);
      if (isPaired) {
        //DB_printf("Received a packet from boat\r\n");
        // No further action required, controllerFSM will read updated powerByte by itself
      }
      else { // if not paired
        if (sourceAddressMSB == targetAddressMSB && sourceAddressLSB == targetAddressLSB) {
          if (powerByte == IS_PAIRED_CHARGE) {
            // Pairing confirmed, post ES_BOAT_PAIRED to controllerFSM
            DB_printf("Pairing is successful\r\n");
            isPaired = true; 
            ES_Event_t pairEvent;
            pairEvent.EventType = ES_BOAT_PAIRED;
            PostcontrollerFSM(pairEvent); 
          }
          else {
            DB_printf("Target/Source address match, waiting for powerByte:0XFF\r\n");
          }
        }
        else {
          DB_printf("Error: sourceAddress and targetAddress mismatch\r\n");
        }
      }
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

void __ISR(_UART_2_VECTOR, IPL7SOFT) U2RxISR(void)
{
  rxByte = U2RXREG; // Read U2RX register into Message
  //DB_printf("rxByte = %d\r\n", rxByte);
  ProcessUARTByte(rxByte); // Pass byte to byte-level decoder
  IFS1CLR = _IFS1_U2RXIF_MASK; // Clear Rx interrupt flag
}

void ProcessUARTByte(uint8_t byte) {
  if (!isReceiving) {
    if (byte == 0x7E) {  // Start delimiter
      //DB_printf("Received start delimiter\r\n");
      if (rxIndex == 0) {
        rxBuffer[rxIndex] = byte;
        rxIndex++;
        isReceiving = true;
      }
      else {
        DB_printf("Error: Received start delimiter, but rxIndex not 0\r\n");
      }
      return;
    }
  }
  else { // is receiving
    // Store the byte and then increment
    rxBuffer[rxIndex] = byte;
    rxIndex++;

    if (rxIndex == 3) {
      // We've received the two length bytes (MSB, LSB)
      uint8_t msb = rxBuffer[1];
      uint8_t lsb = rxBuffer[2];
      expectedLength = 4 + ((msb << 8) | lsb);  // Add start, length, checksum
    }

    if (expectedLength > 0 && rxIndex == expectedLength) { // complete a packet
      //DB_printf("Received a packet\r\n");
      ES_Event_t RxEvent;
      RxEvent.EventType = ES_PACKET_IN;
      PostControllerComm(RxEvent); // Post an event to BoatComm FSM
      isReceiving = false;
      rxIndex = 0;
    }

    if (rxIndex > FRAME_LEN_RX) {
      // Overflow or malformed frame, reset
      DB_printf("Error: rxBuffer overflow, check your framing\r\n");
      isReceiving = false;
      rxIndex = 0;
    }
  }
}

void ParseAPIFrame() {
  // Step 1: Sanity check for start byte
  if (rxBuffer[0] != 0x7E) {
    DB_printf("Error: Invalid Start Delimiter\r\n");
    return;
  }

  // Step 2: Verify checksum
  uint8_t calculatedChecksum = 0;
  for (uint16_t i = 3; i < expectedLength - 1; i++) {
    calculatedChecksum += rxBuffer[i];
  }
  calculatedChecksum = 0xFF - calculatedChecksum;
  uint8_t receivedChecksum = rxBuffer[expectedLength - 1];
  if (calculatedChecksum != receivedChecksum) {
    DB_printf("Error: checksum mismatch\r\n");
    return;
  }

  // Step 3: Extract Frame Type, should only consider 0x81
  uint8_t frameType = rxBuffer[3];
  if (frameType != 0x81) {
    DB_printf("Error: unsupported frame type\r\n");
  }
  else {
    // Update sourceAddress and powerByte
    sourceAddressMSB = rxBuffer[4];
    sourceAddressLSB = rxBuffer[5];
    powerByte = rxBuffer[8]; 
  }
}

/*
void updateTxFrame() {
  if (isPaired) {// Prerequisite: sourceAddressMSB and sourceAddressLSB != 0xFF
    // Modify address bytes
    txFrame[5] = sourceAddressMSB; 
    txFrame[6] = sourceAddressLSB; 
    // Modify chargeLevel byte; if hasn't sent pairing message, set to 0XFF
    if (!hasSentPairingMessage) {
      txFrame[8] = IS_PAIRED; 
      pairingMessageCounter++;
      if (pairingMessageCounter == NUM_PAIRING_MESSAGE) {
        hasSentPairingMessage = true; // turn off 0xFF
        pairingMessageCounter = 0; 
      }
    }
    else {
      txFrame[8] = Power; 
    } 
    // checksum will be calculated in SendFrame()
  }
  else {
    DB_printf("Error: Attempting to updateTxFrame() while unpaired\r\n");
  }
}
*/

void SendFrame() {
  // === Calculate checksum ===
  uint8_t sum = 0;
  for (uint8_t i = 3; i < FRAME_LEN_TX - 1; i++) {
    sum += txFrame[i];
  }
  txFrame[FRAME_LEN_TX - 1] = 0xFF - sum;  // Modify checksum at txFrame[12]
  
  // Send entire frame
  for (uint8_t i = 0; i < FRAME_LEN_TX; i++) {
    while (!U2STAbits.TRMT); // Wait until Transmit Register is empty
    U2TXREG = txFrame[i];
  }
}

void printTxFrame() {
  for (uint8_t i = 0; i < FRAME_LEN_TX; i++) {
    DB_printf("txFrame[i] = %d\r\n", txFrame[i]);
  }
}

void setDeadZone() {
  // check txFrame[9] and txFrame[10]. If within deadzone, manually set to NEUTRAL
  if (abs((int)txFrame[9] - JOYSTICK_NEUTRAL) <= JOYSTICK_DEADZONE_MARGIN) {
    txFrame[9] = JOYSTICK_NEUTRAL;
  }
  if (abs((int)txFrame[10] - JOYSTICK_NEUTRAL) <= JOYSTICK_DEADZONE_MARGIN) {
    txFrame[10] = JOYSTICK_NEUTRAL;
  }
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

