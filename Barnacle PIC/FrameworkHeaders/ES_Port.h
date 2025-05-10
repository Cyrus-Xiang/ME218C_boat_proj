/****************************************************************************
 Module
     ES_Port.h
 Description
     header file to collect all of the hardware/platform dependent info for
     a particular port of the ES framework
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/26/17 18:39 jec     moves definition of ALL_BITS to here
 10/14/15 21:50 jec     added prototype for ES_Timer_GetTime
 01/18/15 13:24 jec     clean up and adapt to use TI driver lib functions
                        for implementing EnterCritical & ExitCritical
 03/13/14		joa		      Updated files to use with Cortex M4 processor core.
                        Specifically, this was tested on a TI TM4C123G mcu.
 08/13/13 12:12 jec     moved the timer rate constants from ES_Timers.h here
 08/05/13 14:24 jec     new version replacing port.h and adding define to
                        capture the C99 compliant behavior of the compiler
*****************************************************************************/
#ifndef ES_PORT_H
#define ES_PORT_H

// pull in the hardware header files that we need
#include <xc.h>

#include <stdio.h>
#include <stdint.h>
#include "bitdefs.h"        /* generic bit defs (BIT0HI, BIT0LO,...) */
#include "Bin_Const.h"      /* macros to specify binary constants in C */
#include "ES_Types.h"

#include "terminal.h"

// macro to control the use of C99 data types (or simulations in case you don't
// have a C99 compiler).
#define COMPILER_IS_C99

// The macro 'ES_FLASH' is needed on some compilers to allocate const data to
// ROM. The macro must be defined, even if it evaluates to nothing.
// for the 'C32 & E128 this is not needed so it evaluates to nothing
#define ES_FLASH

// the macro 'ES_READ_FLASH_BYTE' is needed on some Harvard machines to generate
// code to read a byte from the program space (Flash)
// The macro must be defined, even if it evaluates to nothing.
// for the 'C32 & E128 we don't need special instructions so it evaluates to a
// simple reference to the variable
#define ES_READ_FLASH_BYTE(_flash_var_) (_flash_var_)

// The macro 'REENTRANT' is needed on some compilers to get them to create
// reentrant code. In order to post from an ISR, we need for ES_PostToService,
// ES_EnqueueFIFO, and any service post function that will be called from an
// ISR to be reentrant.
#define REENTRANT __reentrant

// these macros provide the wrappers for critical regions, where ints will be off
// but the state of the interrupt enable prior to entry will be restored.
// allocation of temp var for saving interrupt enable status should be defined
// in ES_Port.c


extern  uint8_t _INTCON_temp;

// for the PIC, at this time, we can not post from within interrupts so keep 
// this definition commented. if you ever get posting from within a int working
// then uncomment it.
// For the PIC32, we *can* post from interrupts
#define POST_FROM_INTS

// in the MIPS architecture, interrupts are not disabled on entry to an ISR
// the interrupt controller simply prevents interrupts from lower or the
// same priority. As a result, we can create a critical region by simply
// disabling interrupts. 
// NOTE: This means that critical regions can not be nested
// I don't think that this should be a serious limitation for the framework
#ifdef POST_FROM_INTS
#define EnterCritical()__builtin_disable_interrupts()
#define ExitCritical() __builtin_enable_interrupts()
#else
#define EnterCritical()
#define ExitCritical()
#endif

/* Rate constants for programming the SysTick Period to generate tick interrupts.
   These assume that we are using the M4K core timer running at 20MHz. Even
   thought the processor clock is 40MHz the core timer increments every other 
   clock edge, resulting in a divide by 2 from the instruction clock.
 */
typedef enum
{
  ES_Timer_RATE_OFF   = (0),
  ES_Timer_RATE_d5mS   = 10000,     /* 0.5ms timer tick */  
  ES_Timer_RATE_1mS  = 20000,       /* 1ms timer tick */
  ES_Timer_RATE_2mS  = 40000,       /* 2ms timer tick */
  ES_Timer_RATE_4mS  = 80000,       /* 4ms timer tick */
  ES_Timer_RATE_5mS  = 100000,       /* 5ms timer tick */
}TimerRate_t;

#if 0 // Moved to terminal.h
// map the generic functions for testing the serial port to actual functions
// for this platform. If the C compiler does not provide functions to test
// and retrieve serial characters, you should write them in ES_Port.c
//#define IsNewKeyReady() (kbhit() != 0)
//#define IsNewKeyReady() (RC1IF)
//#define GetNewKey() getchar()

// this function connects printf & puts to the serial port on the PIC
void putch(char data);

bool kbhit(void);                // is a charcter ready on the EUSART?
#endif

// prototypes for the hardware specific routines
void _HW_PIC32Init(void);
void _HW_Timer_Init(const TimerRate_t Rate);
bool _HW_Process_Pending_Ints(void);
uint16_t _HW_GetTickCount(void);
void _HW_ConsoleInit(void);
void _HW_SysTickIntHandler(void);

// and the one Framework function that we define here
uint16_t ES_Timer_GetTime(void);

#endif
