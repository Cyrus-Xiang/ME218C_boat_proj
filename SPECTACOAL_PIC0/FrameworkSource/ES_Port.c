/****************************************************************************
 Module
   ES_Port.c

 Revision
   1.0.1

 Description
   This is the sample file to demonstrate adding the hardware specific
   functions to the Events & Services Framework. 
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/21 15:43 jec     no changes just a test of using GIT from within MPLABX
 08/06/21 13:04 jec     cleaned things up in preparation for the 2021 AY
 10/05/20 18:52 ram     started work on port to PIC32MX170F256B
 04/18/19 10:17 jec     started work on port to PIC16F15356
 08/21/17 13:47 jec     added functions to init 2 lines for debugging the framework
                        and functions to set & clear those lines.
 03/13/14 10:30	joa		  Updated files to use with Cortex M4 processor core.
                        Specifically, this was tested on a TI TM4C123G mcu.
 03/05/14 13:20	joa		  Began port for TM4C123G
 08/13/13 12:42 jec     moved the hardware specific aspects of the timer here
 08/06/13 13:17 jec     Began moving the stuff from the V2 framework files
 ***************************************************************************/
// PIC32MX170F256B Configuration Bit Settings for ME218 operating at 40MHz from internal fast oscillator (8Mhz)

// 'C' source line config statements

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>             // basic for all projects
#include <cp0defs.h>        // for coprocessor functions
#include <sys/attribs.h>    // for ISR macors

#include <stdint.h>         // for exact size data types
#include <stdbool.h>        // for the bool data type

#include "ES_Port.h"        // the header file for this module
#include "ES_Types.h"       // framework type definitions
#include "ES_Timers.h"      // framework timer prototypes

#include "terminal.h"       // terminal prototypes for init function

// TickCount is used to track the number of timer ints that have occurred
// since the last check. It should really never be more than 1, but just to
// be sure, we increment it in the interrupt response rather than simply
// setting a flag. Using this variable and checking approach we remove the
// need to post events from the interrupt response routine. This is necessary
// for compilers like HTC for the midrange PICs which do not produce re-entrant
// code so cannot post directly to the queues from within the interrupt resp.
static volatile uint8_t TickCount;

// Global tick count to monitor number of SysTick Interrupts
// make uint16_t to maintain backwards compatibility and not overly burden
// 8 and 16 bit processors
static volatile uint16_t SysTickCounter = 0;

// Rate value that needs to be continually added to the compare register to 
// ensure the interrupts occur periodically
static volatile TimerRate_t tickPeriod; 

// This variable is used to store the state of the interrupt mask when
// doing EnterCritical/ExitCritical pairs
// uint8_t _INTCON_temp;


/****************************************************************************
 * Module Level defines
 ***************************************************************************/

//#define LED_DEBUG
/****************************************************************************
 Function
    _HW_PIC32Init
 Parameters
    none
 Returns
     None.
 Description
    Initializes the basic hardware on the PIC. 
 Notes
    The only thing that we need to initialize for the PIC32, is the UART 
 Author
     J. Edward Carryer, 04/18/19 16:17
****************************************************************************/
void _HW_PIC32Init(void)
{
  Terminal_HWInit();
#if 0
  while(1){
    if(kbhit()){
      putch(getchar() & BIT5LO); 
    }  
  }
#endif
}
/****************************************************************************
 Function
     _HW_Timer_Init
 Parameters
     TimerRate_t Rate set to one of the TMR_RATE_XX enum values to set the
     Tick rate
 Returns
     None.
 Description
     Initializes the Core Timer to generate the SysTicks
    
 Notes
     modify as required to port to other timer hardware
 Author
    R. Merchant 10/05/20  18:55
****************************************************************************/
void _HW_Timer_Init(const TimerRate_t Rate)
{
    // If a non-zero rate has been selected
  if(Rate > 0)
  {
    // set the debug halt
    _CP0_SET_DEBUG(_CP0_GET_DEBUG() | _CP0_DEBUG_COUNTDM_MASK);
    // copy over rate value to module var
    tickPeriod = Rate;
        
    // get the current sys clock time
    uint32_t currTime = _CP0_GET_COUNT();
    // add the rate to i1t         
    // place value into compare register
    _CP0_SET_COMPARE(currTime + Rate);
    // Use multivector
    INTCONbits.MVEC = 1;
    // Set Core Timer CT interrupt priority to 3
    IPC0bits.CTIP = 3;
    // Enable the CT interrupt
    IFS0bits.CTIF = 0;
    IEC0bits.CTIE = 1;
    // global enable
    __builtin_enable_interrupts();
    
  }//end if (Rate > 0)

#ifdef LED_DEBUG
  // setup RB15 as debug
  ANSELBbits.ANSB15 = 0;
  LATBbits.LATB15 = 1;
  TRISBbits.TRISB15 = 0;
#endif

  return;
  
}

/****************************************************************************
 Function
     _HW_SysTickIntHandler
 Parameters
     none
 Returns
     None.
 Description
     interrupt response routine for the tick interrupt that will allow the
     framework timers to run.
 Notes
     As currently (4/21/19) implemented this does not actually post events
     but simply increments a counter to indicate that the interrupt has occurred.
     the framework response is handled below in _HW_Process_Pending_Ints
 Author
    R. Merchant, 10/05/20  18:57
****************************************************************************/
void __ISR(_CORE_TIMER_VECTOR, IPL3AUTO ) _HW_SysTickIntHandler(void)
{
  static uint32_t deltaTime; // static for speed
  static uint8_t intsThatShouldHaveHappened;
  
  // clear interrupt flag using the atomic write to the CLR version of the
  // interrupt flag register
  IFS0CLR = _IFS0_CTIF_MASK;
  
  // we create a critical region here in case a higher priority interrupt
  // occurred between the calculation of deltaTime and the test & re-programming
  // of the compare register. If that happened, we could end up programming the 
  // compare for a time that had already passed, resulting in a loss of 
  // tick interrupts until the CoreTimer rolled around.
  EnterCritical();
  // get the time difference since the interrupt
  deltaTime = _CP0_GET_COUNT() - _CP0_GET_COMPARE();
  
  // We need to insure that there are enough cycles left in a tickPeriod to get 
  // the compare register re-programmed before the next interrupt should happen.
  // The -12 accounts for the number of instructions between the calculation
  // of the delta and the re-programming of the Compare register.
  // If there was not enough time available, we would end up programming 
  // the compare for a count/time that had just passed and we would then need to 
  // wait for an entire roll-over cycle before we would get our next tick. 
  // From the outside it would appear that that timer had stopped ticking 
  // for a long time. 
  // 12 cycles is only 6 CoreTimer ticks, so this approach is very conservative.
  // Structure the comparison this way to avoid taking the difference of 2
  // unsigned ints which would require a promotion to signed long to capture
  // the possibility that deltaTime was greater than tickPeriod
  if(deltaTime < (tickPeriod - 12))
  {
    // add the rate back to compare register to set up for the next interrupt
    _CP0_SET_COMPARE(_CP0_GET_COMPARE() + tickPeriod);
    intsThatShouldHaveHappened = 1; // in this case only 1 interrupt happened
  }
  else  // else, interrupts were disabled for a long time
  {
  // We need to calculate how many interrupts that we missed  
  // (_CP0_GET_COUNT() - _CP0_GET_COMPARE()) is the delta time since the last 
  // compare happened, normally a number much less than tickPeriod
  // add 1/2 tickPeriod to the measured delta to move the next compare 
  // out a ways to be sure that we have time to finish this ISR before
  // the next interrupt could occur
  // divide by tickPeriod to get the number of 'missed' interrupts 
  // that should have happened ( normally this will be 0)
  // add 1 for this current interrupt
    intsThatShouldHaveHappened = ((deltaTime + (tickPeriod/2))/tickPeriod) + 1;
    // now update the compare register
    _CP0_SET_COMPARE(_CP0_GET_COMPARE() + 
      (intsThatShouldHaveHappened * tickPeriod));
  }// end if (deltaTime < tickPeriod - 12)
  ExitCritical();
  // and keep our tick counters going
  TickCount += intsThatShouldHaveHappened;
  SysTickCounter += intsThatShouldHaveHappened;

#ifdef LED_DEBUG
  // Toggle debug line
  LATBbits.LATB15 = ~LATBbits.LATB15;
#endif
}

/****************************************************************************
 Function
    _HW_GetTickCount()
 Parameters
    none
 Returns
    uint16_t   count of number of system ticks that have occurred.
 Description
    wrapper for access to SysTickCounter, needed to move increment of tick
    counter to this module to keep the timer ticking during blocking code
 Notes

 Author
    Ed Carryer, 10/27/14 13:55
****************************************************************************/
uint16_t _HW_GetTickCount(void)
{
  return SysTickCounter;
}

/****************************************************************************
 Function
     _HW_Process_Pending_Ints
 Parameters
     none
 Returns
     always true.
 Description
     processes any pending interrupts (actually the hardware interrupt already
     occurred and simply set a flag to tell this routine to execute the non-
     hardware response)
 Notes
     While this routine technically does not need a return value, we always
     return true so that it can be used in the conditional while() loop in
     ES_Run. This way the test for pending interrupts get processed after every
     run function is called and even when there are no queues with events.
     This routine could be expanded to process any other interrupt sources
     that you would like to use to post events to the framework services.
 Author
     J. Edward Carryer, 08/13/13 13:27
****************************************************************************/
bool _HW_Process_Pending_Ints(void)
{
  // in the case where there was a long delay in getting to this function,
  // multiple interrupts may have occurred (TickCount > 1), so process them all
  while (TickCount > 0)
  {
    /* call the framework tick response to actually run the timers */
    ES_Timer_Tick_Resp();
    TickCount--;
  }
  return true;  // always return true to allow loop test in ES_Run to proceed
}

/****************************************************************************
 Function
     _HW_ConsoleInit
 Parameters
     none
 Returns
     none.
 Description
  Initializes the UART for console I/O
 Notes
 real work moved to terminal.c to put all of the terminal functions together
 Author
     J. Edward Carryer, 04/20/19 10:32
 ****************************************************************************/
void _HW_ConsoleInit(void)
{
  Terminal_HWInit();
}

#if 0 // moved to terminal.c
/****************************************************************************
 Function
  putch
 Parameters
  char the character to print
 Returns
     none.
 Description
  send a single character to EUSART1
 Notes
  This function, named in this way is how we connect printf, puts, etc. to
 the serial port on the PIC
 Author
     J. Edward Carryer, 04/20/19 14:20
 ****************************************************************************/
void putch(char data) {
  while( ! TX1IF)   // check if last character finished
  {}                // wait till done
  TX1REG = data;    // send data
}

/****************************************************************************
 Function
  getchar
 Parameters
  none
 Returns
  int The character received (use int return to match C99)  
 Description
  send a single character to EUSART1
 Notes
  The prototype for this is in stdio.h
 Author
  J. Edward Carryer, 04/20/19 14:43
 ****************************************************************************/
int getchar(void) {
  while( ! RC1IF)   // check buffer
  {}                // wait till ready
  return RC1REG;
}

/****************************************************************************
 Function
 kbhit
 Parameters
  none
 Returns
  bool true if character ready, false otherwise.
 Description
  check the state of the RC1IF flag on EUSART1
 Notes

 Author
     J. Edward Carryer, 04/20/19 14:31
 ****************************************************************************/
bool kbhit(void)
{
  /* checks for a character from the terminal channel */
  if (RC1IF)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
#endif