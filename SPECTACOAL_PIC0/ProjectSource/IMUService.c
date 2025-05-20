/****************************************************************************
 Module
   IMUService.c

 Revision
   1.0.1

 Description
   This is a IMU file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from IMUFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "IMUService.h"
#include "terminal.h"
#include "dbprintf.h"
#include <sys/attribs.h>
#include <xc.h>
#include "ControllerComm.h"
#include "controllerFSM.h"
/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void configSPI(void);
static void LSM6DS33_Init(void);
static uint8_t LSM6DS33_ReadReg(uint8_t reg_addr);
static int16_t LSM6DS33_ReadZAccel(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
#define pbclk 20000000 // 20MHz
#define IMU_read_interval 500
#define IMU_boot_interval 1500
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define ACCEL_104HZ_2G 0b01000000  // ODR=104 Hz, FS=2g

static uint32_t SPI_freq;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitIMUService

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
bool InitIMUService(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  configSPI();
  LSM6DS33_Init();
  ES_Timer_InitTimer(IMUUpdate_TIMER, IMU_boot_interval);

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
     PostIMUService

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
bool PostIMUService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunIMUService

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
ES_Event_t RunIMUService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  /********************************************
   in here you write your service code
   *******************************************/
  if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == IMUUpdate_TIMER)
  {
    ES_Timer_InitTimer(IMUUpdate_TIMER, IMU_read_interval);
    #define WHO_AM_I 0x0F
    uint8_t id = LSM6DS33_ReadReg(WHO_AM_I);
    DB_printf("IMU WHO_AM_I: %x\r\n", id);
    // Read the Z-acceleration
    int16_t z_accel = LSM6DS33_ReadZAccel();
    DB_printf("Z-accel: %d\r\n", z_accel);
  }
  else if (ThisEvent.EventType == ES_INIT)
  {
    // Initialization code can go here if needed
  }
  else
  {
    // Handle other events if necessary
  }
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void configSPI(void)
{
    // Step 1: Disable SPI Module
  SPI1CONbits.ON = 0;
  // Step 0: TRIS and0000 ANSEL and mapping
  // B14 is SPI clock
  ANSELBbits.ANSB14 = 0;
  TRISBbits.TRISB14 = 0;
  // A3 is CS pin
  TRISAbits.TRISA3 = 0;
  LATAbits.LATA3= 1; // Deassert CS
  // B5 is SDI
  TRISBbits.TRISB5 = 1;
  SDI1R = 0b0001; // Map SDI1 to RB5
  // B8 is SDO
  TRISBbits.TRISB8 = 0;
  RPB8R = 0b0011; // Map SDO to RB8

  // Step 2: Clear the receive buffer by reading it
  uint8_t dummy = SPI1BUF;
  // Step 3: Enable Enhanced Buffer
  SPI1CONbits.ENHBUF = 0;
  // Step 4: Set Baudrate
  SPI1BRG = 49;                            // for 200kHz SPI freq we use SPI1BRG = 49
  SPI_freq = pbclk / (2 * (SPI1BRG + 1)); // SPI clock frequency
  DB_printf("SPI clock frequency: %d\r\n", SPI_freq);
  // Step 5: Clear the SPIROV Bit
  SPI1STATbits.SPIROV = 0;
  // Step 6: Write desired settings to SPIxCON
  SPI1CONbits.MSTEN = 1; // Places in Leader Mode
  SPI1CONbits.MSSEN = 0; // manual CS control by software
  SPI1CONbits.CKE = 0;   // Reads on 2nd edge
  SPI1CONbits.CKP = 1;   // SCK idles high
  // FRMPOL and AUDEN not needed for manual CS control (manual SPI)
  // SPI1CONbits.FRMPOL = 0; // CS is active low
  // SPI1CON2bits.AUDEN = 0; //might need to delete this line
  SPI1CONbits.MODE16 = 0; // Enable 8 bit transfers
  SPI1CONbits.MODE32 = 0;
  // Step 7: Enable SPI
  SPI1CONbits.ON = 1;
}
static void LSM6DS33_Init(void)
{
  LATAbits.LATA3 = 0; // CS low

  // Send register address with MSB = 0 (write)
  SPI1BUF = CTRL1_XL & 0x7F;
  while (!SPI1STATbits.SPIRBF);
  SPI1BUF; // clear dummy

  // Send configuration byte
  SPI1BUF = ACCEL_104HZ_2G;
  while (!SPI1STATbits.SPIRBF);
  SPI1BUF; // clear dummy

  LATAbits.LATA3 = 1; // CS high
}
uint8_t LSM6DS33_ReadReg(uint8_t reg_addr)
{
  LATAbits.LATA3 = 0;        // CS low
  SPI1BUF = reg_addr | 0x80; // MSB=1 for read
  while (!SPI1STATbits.SPIRBF);//no new byte received yet
  SPI1BUF; // dummy read

  SPI1BUF = 0x00;
  while (!SPI1STATbits.SPIRBF)
    ;
  uint8_t result = SPI1BUF;
  LATAbits.LATA3 = 1; // CS high
  return result;
}

int16_t LSM6DS33_ReadZAccel(void)
{
  LATAbits.LATA3 = 0;         // CS low
  SPI1BUF = OUTZ_L_XL | 0xC0; // Read, auto-increment
  while (!SPI1STATbits.SPIRBF)
    ;
  SPI1BUF; // dummy

  SPI1BUF = 0x00;
  while (!SPI1STATbits.SPIRBF)
    ;
  uint8_t z_l = SPI1BUF;

  SPI1BUF = 0x00;
  while (!SPI1STATbits.SPIRBF)
    ;
  uint8_t z_h = SPI1BUF;

  LATAbits.LATA3 = 1; // CS high

  return (int16_t)((z_h << 8) | z_l);
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
