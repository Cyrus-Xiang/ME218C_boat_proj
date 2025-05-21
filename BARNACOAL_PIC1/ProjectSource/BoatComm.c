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
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include <sys/attribs.h>

#include "BoatComm.h"
#include "DrivetrainService.h"
#include "PowerService.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
#define ONE_SEC 1000
#define ONEFIFTH_SEC (ONE_SEC / 5)
#define FOUR_SEC (ONE_SEC * 4)
#define FRAME_LEN_RX 13
#define FRAME_LEN_TX 10
#define IS_PAIRED 0xFF
#define JOYSTICK_IDLE 0x7F
#define BUTTON_IDLE 0x00
#define NUM_PAIRING_MESSAGE 5

static uint8_t txFrame[FRAME_LEN_TX] = {
  0x7E,          // Start delimiter
  0x00, 0x06,    // Length (MSB, LSB) = 6 bytes of data after this field
  0x01,          // Frame type = TX (16-bit address)
  0x00,          // Frame ID (0 = no ACK)
  0xFF, 0xFF,    // Destination address = 0x218?
  0x01,          // Options = 0x01 to disable ACK
  0x00,          // ChargeLevel: 0x00 (initialized), 0xFF(Pairing Success)
  0x00           // Checksum (computed as 0xFF - sum of bytes after 0x7E)
};

static uint8_t MyPriority;
static UARTState_t CurrentState = InitState;
static bool isPaired = false; // Default to false 
static bool hasSentPairingMessage = false; // Default to false
static uint8_t pairingMessageCounter = 0; // Default to 0

// Variables For Receiving
static volatile uint8_t rxByte = 0xFF; // default to 0xFF
static volatile uint8_t rxBuffer[FRAME_LEN_RX];
static volatile uint8_t rxIndex = 0;
static volatile uint16_t expectedLength = 0;
static volatile bool isReceiving = false;

// Variables For Transmitting

// Payload variables
uint8_t sourceAddressMSB = 0xFF; // rxIndex = 4
uint8_t sourceAddressLSB = 0xFF; // rxIndex = 5
uint8_t statusByte = 0xFF; // rxIndex = 8
uint8_t joystickOneByte = 0xFF; // rxIndex = 9
uint8_t joystickTwoByte = 0xFF; // rxIndex = 10
uint8_t buttonByte = 0xFF; // rxIndex = 11

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
bool InitBoatComm(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // pins I/O setup
  puts("\r Entered InitBoatComm \r");
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
bool PostBoatComm(ES_Event_t ThisEvent)
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
ES_Event_t RunBoatComm(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  switch (CurrentState)
  {
    case InitState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        ES_Timer_InitTimer(BOATCOMM_TIMER, FOUR_SEC); // start 4 sec unpair timer
        CurrentState = Receiving;
      }
    }
    break;

    case Receiving:
    { 
      // If not receicing ES_PACKET_IN for 4 secs, unpair boat from controller and reset
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BOATCOMM_TIMER)
      { 
        // 1. Post ES_UNPAIRED to BoatFSMs
        ES_Event_t unpairEvent;
        unpairEvent.EventType = ES_UNPAIRED; 
        PostDrivetrainService(unpairEvent);
        PostPowerService(unpairEvent);

        // 2. Reset all boat variables
        isPaired = false; 
        hasSentPairingMessage = false;
        pairingMessageCounter = 0;
        sourceAddressMSB = 0xFF; //IMPORTANT: RESET sourceAddressMSB and sourceAddressLSB !!!!!!
        sourceAddressLSB = 0xFF; 
        statusByte = 0xFF; 
        joystickOneByte = 0xFF; 
        joystickTwoByte = 0xFF; 
        buttonByte = 0xFF; 
      }
      else if (ThisEvent.EventType == ES_PACKET_IN)
      {
        // 1. Restart 4sec Timer
        ES_Timer_InitTimer(BOATCOMM_TIMER, FOUR_SEC);

        // 2. Parse the incoming API Packet, update extern byte variables
        ParseAPIFrame();
        DB_printf("statusByte = %d\r\n", statusByte);
        switch (statusByte) 
        {
          case 0x00: // Driving
          {
            if (isPaired) {
              if (buttonByte == BUTTON_IDLE && joystickOneByte == JOYSTICK_IDLE
                  && joystickTwoByte == JOYSTICK_IDLE) {
                // post idle event
                ES_Event_t idleEvent;
                idleEvent.EventType = ES_IDLE;
                PostDrivetrainService(idleEvent);
                PostPowerService(idleEvent);
              } 
              else {
                ES_Event_t commandEvent;
                commandEvent.EventType = ES_COMMAND;
                PostDrivetrainService(commandEvent);
                PostPowerService(commandEvent);
                DB_printf("Post ES_COMMAND to BoatFSMs\r\n");
                DB_printf("buttonByte = %x\r\n", buttonByte);
                // Handle button messages
                if (buttonByte & (1 << 0)) { // Bit 0 is set, post ES_DUMP
                  ES_Event_t dumpEvent;
                  dumpEvent.EventType = ES_DUMP;
                  PostDrivetrainService(dumpEvent);
                  PostPowerService(dumpEvent);
                  DB_printf("Post ES_DUMP to BoatFSMs\r\n");
                }
                if (buttonByte & (1 << 1)) { // Bit 1 is set, Do nothing
                  // Since no anchor on our boat, do nothing
                }
              }
            }
            else {
              // If boat unpaired, ignore packet
              DB_printf("Error: Boat unpaired, invalid command\r\n");
            }
          }
          break;

          case 0x01: // Charging
          {
            if (isPaired) {
              ES_Event_t chargeEvent;
              chargeEvent.EventType = ES_CHARGE; 
              PostDrivetrainService(chargeEvent);
              PostPowerService(chargeEvent);
            }
            else {
              // If boat unpaired, ignore packet
              DB_printf("Error: Boat unpaired, invalid command\r\n");
            }
          }
          break;

          case 0x02: // Pairing
          {
            if (!isPaired) { 
              isPaired = true; 
              DB_printf("Post ES_PAIRED to BoatFSMs\r\n");
              ES_Event_t pairEvent;
              pairEvent.EventType = ES_PAIRED;
              // sourceAddressMSB and sourceAddressLSB are directly accessible at boatFSMs
              PostDrivetrainService(pairEvent); // Post an event to BoatComm FSM
              PostPowerService(pairEvent);
            }
            else {
              DB_printf("Error: Boat paired, invalid pairing command\r\n");
              // Ignore excessive pairing requests or from other sources
            }
          }
          break;

          default:
          {
            DB_printf("Error: unsupported status byte\r\n");
          }
          break; 
        }
        CurrentState = Transmitting; 
      }
      else {
        // Does not receive ES_TIMEOUT or ES_PACKET_IN, stay in current state
      }
    }
    break;

    case Transmitting:
    {
      updateTxFrame(); // updateTxFrame() accordingly
      SendFrame(); // Send the 10-byte packet 
      CurrentState = Receiving; // Transition back to Receicing State
    }
    break;
    
    // repeat state pattern as required for other states
    default:
      ;
  }                                   // end switch on Current State
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
  // DB_printf("rxByte = %d\r\n", rxByte);
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
      PostBoatComm(RxEvent); // Post an event to BoatComm FSM
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
    if (sourceAddressMSB == 0xFF && sourceAddressLSB == 0xFF) {
      // Only allowed to change sourceAddress when unpaired
      sourceAddressMSB = rxBuffer[4];
      sourceAddressLSB = rxBuffer[5];
    }
    else {
      // If paired, DO NOT change sourceAddressMSB/LSB !!!
    }
    statusByte = rxBuffer[8];
    joystickOneByte = rxBuffer[9];
    joystickTwoByte = rxBuffer[10];
    buttonByte = rxBuffer[11];
  }
}

void updateTxFrame() {
  if (isPaired) {// Prerequisite: sourceAddressMSB and sourceAddressLSB != 0xFF
    // Modify address bytes
    txFrame[5] = sourceAddressMSB; 
    txFrame[6] = sourceAddressLSB; 
    // Modify chargeLevel byte; if hasn't sent pairing message, set to 0XFF
    if (!hasSentPairingMessage) {
      txFrame[8] = IS_PAIRED; // 0xFF
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

void SendFrame() {
  // === Calculate checksum ===
  uint8_t sum = 0;
  for (uint8_t i = 3; i < FRAME_LEN_TX - 1; i++) {
    sum += txFrame[i];
  }
  txFrame[FRAME_LEN_TX - 1] = 0xFF - sum;  // Modify checksum at txFrame[9]
  
  // Send entire frame
  for (uint8_t i = 0; i < FRAME_LEN_TX; i++) {
    while (!U2STAbits.TRMT); // Wait until Transmit Register is empty
    U2TXREG = txFrame[i];
  }
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

