/****************************************************************************
 Module
   ControllerFSM.c

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
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ControllerFSM.h"
#include "ES_Timers.h"
#include <xc.h>
#include <sys/attribs.h>
#include "PIC32_AD_Lib.h"

/*----------------------------- Module Defines ----------------------------*/
//Defines for framework timers at resolution 1ms/tick.
#define ONE_SEC 1000
#define HALF_SEC (ONE_SEC / 2)
#define TWO_SEC (ONE_SEC * 2)
#define FIVE_SEC (ONE_SEC * 5)
#define TWOHUNDRED_MS 200

#define NUM_AD_CHANNELS 3
#define AD_CHANNEL ((1<<5) | (1<<12) | (1<<11))

//Defines for our communication api system
#define MSG_LEN 13 //counting the checksum
#define API_IDENTIFIER_IND 3
#define FRAME_ID_IND 4
#define MSG_ID_IND 8
#define DestAddrMSB_IND 5
#define DestAddrLSB_IND 6
#define Refuel_ind 9
#define EngineControl_ind 9
#define RudderControl_ind 10
#define ButtonsControl_ind 11
#define REFUEL 1
#define PAIR 2
#define CONTROL 0
#define DELIMITER 0x7E

#define PWM_TICKS_PER_MS 1250
#define T4_TICKS_PER_MS 313 //Approx 20MHz at 64 prescalar.
#define RealPR2 PWM_TICKS_PER_MS * 20
//#define FuelServoMax 0.04 * RealPR2
#define FuelServoMax 1050
#define FuelServoMin 3200
//#define FuelServoMin 600
//#define FuelServoMin 0.13 * RealPR2 
#define ServoStep 29

#define NavButton1 PORTAbits.RA2
#define NavButton2 PORTAbits.RA3
#define NavButton3 PORTBbits.RB4


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void UpdateSelectionAddr(void);
static void SendFrame(uint8_t frame_description);
void ResetReceiveVariables(void);
void UpdateControlFrame(void);
bool CheckSumVerify(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static uint32_t MyPriority;
static ControllerFSMState_t CurrentState;
static uint32_t CurrentAddrSelection[NUM_AD_CHANNELS];
//Adapated from 2022-2023 team Lighting PIC_QUEEN, their definition of the frames are neat!
static uint8_t ControlFrame[MSG_LEN];
static uint8_t PairingFrame[MSG_LEN];
static uint8_t RefuelFrame[MSG_LEN];
static uint8_t BoatAddrLSB = 0x84; //defaults to our boat.
volatile uint8_t CheckSum = 0;
volatile uint8_t ByteInd; //To note what the index of the current byte in transmit.
volatile uint8_t RxByteInd = 0;
static bool TransmitPair = false;
static bool TransmitControl = false;
static bool TransmitRefuel = false;
volatile uint8_t RxMsgLen = 7; //The status tx packet length.
static uint8_t ReceivedFrame[MSG_LEN]; 
volatile bool ActualMsgFlag;
volatile bool RefuelFlag;
const uint32_t Timer3Cycle = 0x0000FFFF;
static uint16_t Target_rpm;
static bool FirstEdgeObserved; //A flag to indicate if the IC2 has been any rising edges and start framework timer.
static uint8_t FuelLevel;
static uint32_t CalibratedAnalog[2];
static uint8_t RefuelNum;



/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitControllerFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Baiyu Shi
****************************************************************************/
bool InitControllerFSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
    ControlFrame[0] = 0x7E;//start delimiter
    ControlFrame[1] = 0x00;
    ControlFrame[2] = 0x09; //the LSB of the length is always 9 bytes
    ControlFrame[3] = 0x01; //api identifier for tx.
    ControlFrame[4] = 0x00; //Frame ID
    ControlFrame[DestAddrMSB_IND] = 0x20; //the boat always have addr starting with 0x20.
    ControlFrame[DestAddrLSB_IND] = BoatAddrLSB; 
    ControlFrame[7] = 0x01; //disable XBEE ACK.
    ControlFrame[MSG_ID_IND] = 0x01; 
    ControlFrame[EngineControl_ind] = 0x80; //0x128 netural position.
    ControlFrame[RudderControl_ind] = 0x80; //0x128 netural rudder position.
    ControlFrame[ButtonsControl_ind] = 0x00; //Not shoot.

    RefuelFrame[0] = 0x7E;//start delimiter
    RefuelFrame[1] = 0x00;
    RefuelFrame[2] = 0x09; //the LSB of the length is always 9 bytes
    RefuelFrame[3] = 0x01; //api identifier for tx.
    RefuelFrame[4] = 0x00; //Frame ID
    RefuelFrame[DestAddrMSB_IND] = 0x20; //the boat always have addr starting with 0x20.
    RefuelFrame[DestAddrLSB_IND] = BoatAddrLSB; 
    RefuelFrame[7] = 0x01; //disable XBEE ACK.
    RefuelFrame[MSG_ID_IND] = 0x05; 
    RefuelFrame[Refuel_ind] = 0x01; //refuel bit.
    RefuelFrame[RudderControl_ind] = 0x00; 
    RefuelFrame[ButtonsControl_ind] = 0x00; 

    PairingFrame[0] = 0x7E;//start delimiter
    PairingFrame[1] = 0x00;
    PairingFrame[2] = 0x09; //the LSB of the length is always 9 bytes
    PairingFrame[3] = 0x01; //api identifier for tx.
    PairingFrame[4] = 0x00; //Frame ID
    PairingFrame[DestAddrMSB_IND] = 0x20; //the boat always have addr starting with 0x20.
    PairingFrame[DestAddrLSB_IND] = BoatAddrLSB; 
    PairingFrame[7] = 0x01; //disable XBEE ACK.
    PairingFrame[MSG_ID_IND] = 0x03; 
    PairingFrame[Refuel_ind] = 0x00; 
    PairingFrame[RudderControl_ind] = 0x00; 
    PairingFrame[ButtonsControl_ind] = 0x00; 
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
     PostControllerFSM

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
bool PostControllerFSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunControllerFSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   baiyu shi
****************************************************************************/
ES_Event_t RunControllerFSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == FuelLevelProceedTimer) {
    ES_Timer_InitTimer(FuelLevelProceedTimer, HALF_SEC);
    RefuelFlag = true;
    RefuelNum++;
  }
  else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == InputSenseTimer) {
    UpdateControlFrame();
    if (ControlFrame[EngineControl_ind] != 0x80 | ControlFrame[RudderControl_ind] != 0x80 | ControlFrame[ButtonsControl_ind] != 0x00) {
        RefuelFlag = false;
        printf("movement \n\r");
        ES_Timer_StopTimer(FuelLevelProceedTimer);
        FirstEdgeObserved = false;
    }
    ES_Timer_InitTimer(InputSenseTimer, 100);
    return ReturnEvent;
  }
  switch (CurrentState)
  {
    case InitState:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        TRISAbits.TRISA1 = 1; //Set RA1 to digital input, xbee dout.
        ANSELAbits.ANSA1 = 0;
        TRISBbits.TRISB2 = 1;
        ANSELBbits.ANSB2 = 0; //Set RB2 to digital input, pair button.
        TRISBbits.TRISB14 = 0; //Set RB14 to digital output, for XBEE DIN.
//        TRISBbits.ANSB14 = 0;
        TRISBbits.TRISB12 = 1;
        ANSELBbits.ANSB12 = 1; //Set RB12 to analog input, addr selection pot.
        TRISBbits.TRISB9 = 1; //Set RB9 to digital input.
//        ANSELBbits.ANSB9 = 0;
        TRISBbits.TRISB15 = 0;
        ANSELBbits.ANSB15 = 0; //Set RB15 to digital output, OC1.
        TRISBbits.TRISB13 = 1; 
        ANSELBbits.ANSB13 = 1;//Analog in
        TRISBbits.TRISB3 = 1;
        ANSELBbits.ANSB3 = 1; //Set RB3 to analog input.
        TRISAbits.TRISA2 = 1; //Set RA2 to digital input, navcon button 1 .
//        ANSELAbits.ANSA2 = 0;
        TRISAbits.TRISA3 = 1; //Set RA3 to digital input, navcon button 2 .
//        ANSELAbits.ANSA3 = 0;
        TRISBbits.TRISB4 = 1; //Set RB4 to digital input, navcon button 3 .
//        ANSELBbits.ANSB4 = 0;
        
        //Configure the ADC systems.
        ADC_ConfigAutoScan(AD_CHANNEL);
        printf("\e[1;1H\e[2J");

        __builtin_disable_interrupts();
        //Make sure we are running multivector interrupt;
        INTCONbits.MVEC = 1;
        U2MODEbits.ON = 0; //shutdown the system
        IEC1bits.U2RXIE = 0; //disable interrupt
        U2RXR = 0b0000; //map U2RX to RPA1, XBEEE OUT
        RPB14R = 0b0010; //map to U2TX
        //Clear the following bits.
        U2MODEbits.SIDL = 0; //Don't stop on idle mode.
        U2MODEbits.IREN = 0; //disable IrDA encoder decoder that creates pulses.
        // U2MODEbits.RTSMD = 0; //do not flow control mode, dte-dte.
        U2MODEbits.UEN = 0b00; //CTS, RTS can be freely controlled, only use TX RX.
        U2MODEbits.WAKE = 0; //disable wake up
        U2MODEbits.LPBACK = 0; //disable loopback mode.
        U2MODEbits.ABAUD = 0; //disable auto baud rate.
        U2MODEbits.RXINV = 0; //Idle high.
        U2MODEbits.BRGH = 0; //16x baud clock selected, standard
        U2MODEbits.PDSEL = 0b00; //8 bit data no parity
        U2MODEbits.STSEL = 0; //only have 1 stop bit.

        U2STAbits.UTXINV = 0; //UxTx is still idle high.
        U2STAbits.UTXBRK = 0; //1 is used for auto baud rate mode
        U2STAbits.ADDEN = 0; //No address detection, only 8 bit.
        U2STAbits.UTXISEL = 0b01; //Whenever all THE BUFFER is clr.
        U2STAbits.URXISEL = 0b00; //whenever we receive something get the interrupt.
        U2STAbits.UTXEN = 1; //enable transmitter
        U2STAbits.URXEN = 1;//enable receiver.
        U2STAbits.OERR = 0; //clear over run errors.
        IPC9bits.U2IP = 7; //Set UART2 to highest priority
        IEC1SET = _IEC1_U2RXIE_MASK;
        // IEC1SET = _IFS1_U2TXIE_MASK; //TX IE enabled.
        IEC1CLR = _IEC1_U2EIE_MASK; //no fault for uart2.
        IFS1CLR = _IFS1_U2RXIF_MASK;
        IFS1CLR = _IFS1_U2TXIF_MASK;
        IFS1CLR = _IFS1_U2EIF_MASK; //clear the rx, tx masks.
        while (U2STAbits.URXDA > 0) {
          U2RXREG;   //Read off everything in the receive reg.
        }

        U2BRG = 129; //for a 9600 baud rate.
        U2MODEbits.ON = 1; //turn the system back on.
        ByteInd = 0;
        RxByteInd = 0;
        TransmitPair = false;
        TransmitControl = false;
        TransmitRefuel = false;
        ActualMsgFlag = false;

        T2CONbits.ON = 0;
        //Set the base clock on timer2 default to PBClk with scaler of 16; 
        T2CONbits.TCS=0; 
        T2CONbits.TCKPS = 0b100;
        //Set the timer to run in Idle Mode;
        T2CONbits.SIDL = 0;
        //Set to use as separate 16-bit timer;
        T2CONbits.T32 = 0;
        //Clear the interrupt flag bit for timer2
        IFS0CLR = _IFS0_T2IF_MASK;
        //Turn off interrupt timer2 locally;
        IEC0CLR = _IEC0_T2IE_MASK;
        TMR2 = 0;
        PR2 = RealPR2; //to guarantee a 50Hz PWM frequency;
        OC1CONbits.ON = 0; //turn off OC1
        OC2CONbits.ON = 0;
        OC3CONbits.ON = 0; 
        OC4CONbits.ON = 0;
        OC1CONbits.OCTSEL = 0; //select timer2 for oc1.
        //Set no fault detect pwm for OC1;
        OC1CONbits.OCM = 0b110;
        //Set the period for the initial and the repeating cycles both to 0 for OC3;
        OC1R = 0;
//        OC1RS = 0; //Setoff with max fuel level.
        RPB15R = 0b0101; //Map it onto OC1

      //Then setup input capture system's clock.
        T3CONbits.ON = 0;
        //Set the base clock on timer3 default to PBClk with scaler of 16; 
        T3CONbits.TCS=0; 
        T3CONbits.TCKPS = 0b110;
        T3CONbits.SIDL = 0;
        //Clear the interrupt flag bit for timer3
        IFS0CLR = _IFS0_T3IF_MASK;
        //Set its priority to 6.
        IPC3bits.T3IP = 6;
        PR3 = (uint16_t)Timer3Cycle; //set PR3 to max 2^16-1 to use modulo math.
        TMR3 = 0;
        //Turn off interrupt timer3 locally;
        IEC0CLR = _IEC0_T3IE_MASK;

        //Then setup Timer4 clock for the 75ms nonactive timeout.
        T4CONbits.ON = 0;
        //Set the base clock on timer4 default to PBClk with scaler of 64; 
        T4CONbits.TCS = 0;
        T4CONbits.TCKPS = 0b110;
        //Set the timer4 to run in Idle Mode
        T4CONbits.SIDL = 0;
        //Clear the interrupt flag bit for timer4
        IFS0CLR = _IFS0_T4IF_MASK;
        //Set its priority to 6.
        IPC4bits.T4IP = 7;
        PR4 = 75 * T4_TICKS_PER_MS - 1; //to achieve a period of 75ms;
        //Reset the current timer count register for timer4
        TMR4 = 0;
        //Turn on interrupt timer4 locally;
        IEC0SET = _IEC0_T4IE_MASK;

        //Then we set the input capture.
        IC2CONbits.ON = 0;
        //Set the input capture to run in idle mode;
        IC2CONbits.SIDL = 0;
        //Set the capture to run in 16-bit;
        IC2CONbits.C32=0;
        //Set the timer source to timer3
        IC2CONbits.ICTMR = 0;
        //Set the interrupt to be sent on every capture;
        IC2CONbits.ICI = 0b00;
        //Set the input capture2 to capture on every rising edge;
        IC2CONbits.ICM = 0b011;
        // T4CONbits.ON = 1; //Don't start timer4 yet.
        T3CONbits.ON = 1;
        T2CONbits.ON = 1;
        OC1CONbits.ON=1;
        //Read the buffer to clear them;
        do{
          IC2BUF;
        }while(IC2CONbits.ICBNE != 0);
        //Map the input captures IC2 to RPB9.
        IC2R = 0b0100;
        //Locally enable IC2 interrupts;
        IEC0SET = _IEC0_IC2IE_MASK;
        //Set IC2 to have the highest input priority;
        IPC2bits.IC2IP = 7;
        IFS0CLR = _IFS0_IC2IF_MASK;
        IC2CONbits.ON = 1;
        printf("Large scale motion recorder ready! \n\r");
        __builtin_enable_interrupts();
        printf("Waiting for button \n\r");
        OC1RS = FuelServoMax; //Set off with max fuel level.
//        OC1RS = 1050;
        FirstEdgeObserved = false;
        RefuelFlag = false;
        RefuelNum = 0;
        FuelLevel = 0;
        CalibratedAnalog[0] = 0;
        CalibratedAnalog[1] = 0;
        for (int i=0; i<10;i++) {
           ADC_MultiRead(CurrentAddrSelection);
           CalibratedAnalog[0]+=CurrentAddrSelection[0];
           CalibratedAnalog[1]+=CurrentAddrSelection[1];
        }
        CalibratedAnalog[0] = CalibratedAnalog[0] / 10;
        CalibratedAnalog[1] = CalibratedAnalog[1] / 10;
      }

      else if (ThisEvent.EventType == ES_AddrButtonDown) {
        CurrentState = Pairing;
        UpdateSelectionAddr();
//        SendFrame(PAIR);
        ES_Timer_InitTimer(PairingTimer, TWOHUNDRED_MS);
        ResetReceiveVariables();
      }
    }
    break;

    case Pairing:        // If current state is state pairing
    {
      //This way if it falls back we will not request the pot value and try to reconnect.
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == PairingTimer) {
        SendFrame(PAIR);
        ES_Timer_InitTimer(PairingTimer, TWOHUNDRED_MS);
      }
      else if (ThisEvent.EventType == ES_AddrButtonDown) {
        UpdateSelectionAddr();
        ES_Timer_InitTimer(PairingTimer, TWOHUNDRED_MS);
      }
      else if (ThisEvent.EventType == ES_PAIRED_ACK) {
          printf("paired received \n\r");
        CurrentState = Paired;
        ES_Timer_StopTimer(PairingTimer);
        SendFrame(CONTROL); 
        ResetReceiveVariables();
        RefuelNum = 0;
        ES_Timer_InitTimer(COMM_TIMEOUT_TIMER, 4 * ONE_SEC);
        ES_Timer_InitTimer(ControlTimer, TWOHUNDRED_MS);
        ES_Timer_InitTimer(InputSenseTimer, 100);
      }
    }
    break;

    case Paired:        // If current state is state one
    {
      if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == COMM_TIMEOUT_TIMER) {
        ES_Timer_StopTimer(COMM_TIMEOUT_TIMER);
//        ES_Timer_InitTimer(PairingTimer, TWOHUNDRED_MS);
//        CurrentState = Pairing; //fallback to pairing.
        CurrentState = InitState;
        OC1RS = FuelServoMax; //Set off with max fuel level.
      }
//      else if (ThisEvent.EventType == ES_NavButtonUpdate){
//        ControlFrame[ButtonsControl_ind] = ThisEvent.EventParam; 
//      }
      else if (ThisEvent.EventType == ES_WEIRD_RECEIVE) {
        ResetReceiveVariables();
      }
      else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ControlTimer) {
        if (RefuelNum > 0) {
//          FuelLevel += 15;
//          if (FuelLevel > 75) {
//              FuelLevel = 75;
//          }
//          OC1RS = (uint16_t)FuelServoMin - (FuelLevel * ServoStep);
            if (RefuelNum >= 5) {
                RefuelNum = 5;
            }
          RefuelFrame[Refuel_ind] = RefuelNum;
          SendFrame(REFUEL);
          printf("Refuel sent \n\r");
        }
        else{
          UpdateControlFrame();
          SendFrame(CONTROL); 
        }
        ES_Timer_InitTimer(ControlTimer, TWOHUNDRED_MS);
      }
      else if (ThisEvent.EventType == ES_ValidStatusReceived) {
        ES_Timer_InitTimer(COMM_TIMEOUT_TIMER, 4 * ONE_SEC); //Reinitializes communication timeout.
        if (CheckSumVerify()) {
            FuelLevel = ReceivedFrame[Refuel_ind];
            RefuelNum = 0;
            OC1RS = (uint16_t)FuelServoMin - (FuelLevel * ServoStep);
            printf("Fuel level: %d \n\r", ReceivedFrame[Refuel_ind]);
        }
      }
    }
    break;
    // repeat state pattern as required for other states
    default:
      ;
  }                     
  return ReturnEvent;
}

/***************************************************************************
 interrupt handlers
 ***************************************************************************/

void __ISR(_UART_2_VECTOR, IPL7SOFT) _UART_Handler(void) {
  __builtin_disable_interrupts();
  if (IFS1bits.U2RXIF == 1){
//    while (U2STAbits.URXDA > 0) {
//        NewData = U2RXREG;   //Read off everything in the receive reg.
//    }

     uint16_t NewData = U2RXREG; 
     IFS1CLR = _IFS1_U2RXIF_MASK; //We have to read first before clearing.
     if (NewData == DELIMITER) {
         RxByteInd = 0;
         ActualMsgFlag = true;
         ReceivedFrame[RxByteInd] = NewData;
//         RxByteInd++;
     }
     else if (ActualMsgFlag) {
         ReceivedFrame[RxByteInd] = NewData;
        if (RxByteInd == 2) {
            RxMsgLen = ReceivedFrame[RxByteInd] + 4;
            if (RxMsgLen != MSG_LEN) {
                RxMsgLen = 7; //The tx status bit length for safety.
            }
        }
        else if (RxByteInd >= (RxMsgLen - 1)) {
            if (ReceivedFrame[API_IDENTIFIER_IND] == 0x81) {
              ES_Event_t ReturnEvent;
              ReturnEvent.EventType = ES_WEIRD_RECEIVE;
              if (ReceivedFrame[MSG_ID_IND] == 0x04) {
                ReturnEvent.EventType = ES_PAIRED_ACK;
              }
              else if (ReceivedFrame[MSG_ID_IND] == 0x02) {
                ReturnEvent.EventType = ES_ValidStatusReceived;
              }
              else{
                printf("Corrupted Frame received! \n\r");
              }
              PostControllerFSM(ReturnEvent);
            }
            RxByteInd = 0;
            ActualMsgFlag = false;
//            IEC1CLR = _IEC1_U2RXIE_MASK; //To prevent always stuck.
        }
     }
     RxByteInd++;
//    IFS1CLR = _IFS1_U2TXIF_MASK;
  }
  else if (IFS1bits.U2TXIF == 1) {
    IFS1CLR = _IFS1_U2TXIF_MASK;
//    IFS1CLR = _IFS1_U2RXIF_MASK;
    if (TransmitControl) {
      if (ByteInd < MSG_LEN) {
        U2TXREG = ControlFrame[ByteInd];
        ByteInd++;
      }
      else{
        //Means that we have finished sending everything.
        ByteInd = 0; //reset
        TransmitControl = false;
        //turn off t2if.
        IEC1CLR = _IEC1_U2TXIE_MASK;
        ControlFrame[ButtonsControl_ind] = 0x00; //Reset the button bits.
//        IEC1SET = _IEC1_U2RXIE_MASK;
      }
    }
    else if (TransmitPair) {
//      printf("transmitting \n\r");
      if (ByteInd < MSG_LEN) {
        U2TXREG = PairingFrame[ByteInd];
        ByteInd++;
      }
      else{
        //Means that we have finished sending everything.
        ByteInd = 0; //reset
        TransmitPair = false;
        //turn off t2if.
        IEC1CLR = _IEC1_U2TXIE_MASK;
//        IEC1SET = _IEC1_U2RXIE_MASK;
      }
    }
    else if (TransmitRefuel) {
      if (ByteInd < MSG_LEN) {
        U2TXREG = RefuelFrame[ByteInd];
        ByteInd++;
      }
      else{
        //Means that we have finished sending everything.
        ByteInd = 0; //reset
        TransmitRefuel = false;
        //turn off t2if.
        IEC1CLR = _IEC1_U2TXIE_MASK;
//        IEC1SET = _IEC1_U2RXIE_MASK;
      }
    }
  }
  __builtin_enable_interrupts();
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7SOFT) _Encoder_Handler(void) {
  do{
      IC2BUF;
  } while(IC2CONbits.ICBNE != 0);
  printf("Edge detected \n\r");
  if (FirstEdgeObserved == false) {
      printf("First Edge Detected \n\r");
    ES_Timer_InitTimer(FuelLevelProceedTimer, HALF_SEC);
    FirstEdgeObserved = true;
  }
  //reset nonactive timer.
  T4CONbits.ON = 0; //turn off nonactive timer.
  TMR4 = 0;
  T4CONbits.ON = 1; //turn on nonactive timer.
  //Clear the pending flags;
  IFS0CLR = _IFS0_IC2IF_MASK;
}

void __ISR(_TIMER_4_VECTOR, IPL7SOFT) _Nonactive_Timer_Handler(void) {
  ES_Timer_StopTimer(FuelLevelProceedTimer);
  FirstEdgeObserved = false;
  T4CONbits.ON = 0; //turn off nonactive timer.
  TMR4 = 0;
  //Clear the Interrupt flag of Timer4;
  IFS0CLR = _IFS0_T4IF_MASK;
}

/***************************************************************************
 private functions
 ***************************************************************************/
//Update the addrLSB based on the pot readout.
//static void UpdateSelectionAddr(void) {
//  ADC_MultiRead(CurrentAddrSelection);
//  BoatAddrLSB = 0x81 + (uint8_t)(CurrentAddrSelection[2] / 150);
//  printf("new boat addr selected is :%d \n\r", BoatAddrLSB);
//}


//Update the addrLSB based on the pot readout.
static void UpdateSelectionAddr(void) {
  ADC_MultiRead(CurrentAddrSelection);
  BoatAddrLSB = 0x81 + (uint8_t)(CurrentAddrSelection[2] / 200);
  printf("new boat addr selected is :%d \n\r", BoatAddrLSB);
  ControlFrame[DestAddrLSB_IND] = BoatAddrLSB; 
  RefuelFrame[DestAddrLSB_IND] = BoatAddrLSB; 
  PairingFrame[DestAddrLSB_IND] = BoatAddrLSB; 
}
//Read the static variable's description of frame, set the indices accordingly and send off the first bit.
static void SendFrame(uint8_t frame_description) {
//    IEC1CLR = _IEC1_U2RXIE_MASK; //shutdown the receiver
  ByteInd = 1;
  CheckSum = 0;
  //Calculate these outside of ISR to save time.
  if (frame_description == CONTROL) {
    for (uint8_t current_ind = API_IDENTIFIER_IND; current_ind < MSG_LEN - 1; current_ind++) {
      CheckSum += ControlFrame[current_ind];
    }
    CheckSum = 0xFF - CheckSum;
    ControlFrame[MSG_LEN - 1] = CheckSum;
    U2TXREG = ControlFrame[0];
    TransmitControl = true;
  }

  else if (frame_description == REFUEL) {
    for (uint8_t current_ind = API_IDENTIFIER_IND; current_ind < MSG_LEN - 1; current_ind++) {
      CheckSum += RefuelFrame[current_ind];
    }
    CheckSum = 0xFF - CheckSum;
    RefuelFrame[MSG_LEN - 1] = CheckSum;
    U2TXREG = RefuelFrame[0];
    TransmitRefuel = true;
  }

  else if (frame_description == PAIR) {
      printf("Sent pairing \n\r");
    for (uint8_t current_ind = API_IDENTIFIER_IND; current_ind < MSG_LEN - 1; current_ind++) {
      CheckSum += PairingFrame[current_ind];
    }
    CheckSum = 0xFF - CheckSum;
    PairingFrame[MSG_LEN - 1] = CheckSum;
    U2TXREG = PairingFrame[0];
    TransmitPair = true;
  }
  IEC1SET = _IEC1_U2TXIE_MASK; //enable transmit.
}

void ResetReceiveVariables(void) {
  IEC1SET = _IEC1_U2RXIE_MASK;
  RxMsgLen = 7;
  RxByteInd = 0;
  ActualMsgFlag = false;
  for (int i=0; i < MSG_LEN; i++){
      ReceivedFrame[i] = 0;
  }
}

const uint32_t tolerance = 30;
//Update the control frame through voltage values sampled.
void UpdateControlFrame(void) {
    ADC_MultiRead(CurrentAddrSelection);
    //TODO: add in button control codes.
    if (0 == FuelLevel) {
        ControlFrame[EngineControl_ind] = 0x80; //0d128 netural position.
        ControlFrame[RudderControl_ind] = 0x80; //0d128 netural rudder position.
        ControlFrame[ButtonsControl_ind] = 0x00;
    }
    else {
        uint32_t NewEngineValue = 1023 - CurrentAddrSelection[1];
        uint32_t NewRudderValue = 1023 - CurrentAddrSelection[0];
        if (NewEngineValue > CalibratedAnalog[0] - tolerance && NewEngineValue < CalibratedAnalog[0] + tolerance) {
            NewEngineValue = CalibratedAnalog[0];
        }
        if (NewRudderValue > CalibratedAnalog[1] - tolerance && NewRudderValue < CalibratedAnalog[1] + tolerance) {
            NewRudderValue = CalibratedAnalog[1];
        }
        NewEngineValue = NewEngineValue * 128 / CalibratedAnalog[0];
        NewRudderValue = NewRudderValue * 128 / CalibratedAnalog[1];
        if (NewEngineValue > 255) {
            NewEngineValue = 255;
        }
        if (NewRudderValue > 255) {
            NewRudderValue = 255;
        }
        ControlFrame[EngineControl_ind] = NewEngineValue; //0d128 netural position.
        ControlFrame[RudderControl_ind] = NewRudderValue; //0d128 netural rudder position.
        uint8_t CombinedState = 0x00; //0b0000_0111 to indicate non of the buttons pressed down.
//        CombinedState |= ((~NavButton1 & 0x01) << 0);
//        CombinedState |= ((~NavButton2 & 0x01) << 1); //move to the 2nd last position.
//        CombinedState |= ((~NavButton3 & 0x01) << 2);
        CombinedState |= ((NavButton1 & 0x01) << 0);
        CombinedState |= ((NavButton2 & 0x01) << 1); //move to the 2nd last position.
        CombinedState |= ((NavButton3 & 0x01) << 2);
        ControlFrame[ButtonsControl_ind] = CombinedState;
    }
   
}

bool CheckSumVerify(void) {
  uint8_t CheckSum = 0;
  for (uint8_t current_ind = API_IDENTIFIER_IND; current_ind < MSG_LEN - 1; current_ind++) {
    CheckSum += ReceivedFrame[current_ind];
  }
  CheckSum = 0xFF - CheckSum;
  if (CheckSum != ReceivedFrame[MSG_LEN-1]) {
    return false;
  }
  else {
    return true;
  }
}

