/* 
 * File:   terminal.h
 * Author: Riyaz Merchant
 *
 * Created on August 29, 2020, 2:44 PM
 */

#ifndef TERMINAL_H
#define	TERMINAL_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include "ES_Port.h"

// a few handy shortcuts for terminal control    
#define clrScrn() printf("\x1b[2J")
#define goHome() printf("\x1b[1,1H")
#define clrLine() printf("\x1b[K")
    
#define XMIT_BUFFER_SIZE 1024
    
// map the generic functions for testing the serial port to actual functions
// for this platform.
#define IsNewKeyReady() (U1STAbits.URXDA)
#define GetNewKey Terminal_ReadByte
//#define putch Terminal_WriteByte
#define kbhit() (U1STAbits.URXDA)
    
void Terminal_HWInit(void);
uint8_t Terminal_ReadByte(void);
void Terminal_WriteByte(uint8_t txByte);
bool Terminal_IsRxData(void);
void Terminal_MoveBuffer2UART( void );

#ifdef __XC16__  // DEPRICATED, USE FOR xc16 of xc32 v1.34 or lower
int write(int handle, void *buffer, unsigned int len);
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* TERMINAL_H */

