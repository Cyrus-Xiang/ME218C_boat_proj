//#define TEST
/****************************************************************************
 Module
   terminal.c

 Revision
   1.0.1

 Description
  File holds functions for printing and receiving characters to/from the serial
  emulator through a UART-USB bridge interface.
 Notes
  For the PIC32 port, we are using UART 1

 History
 When           Who     What/Why
 -------------- ---     --------
 08/29/20 14:46 ram     first pass
 10/05/20 19:38 ram     starting work on PIC32 port
 ***************************************************************************/

/*----------------------------- Include Files -----------------------------*/

// Hardware
#include <xc.h>
#include <stdio.h>

#include "ES_General.h"
#include "ES_Port.h"
#include "circular_buffer.h"
#include "dbprintf.h"

//this module
#include "terminal.h"
/*----------------------------- Module Defines ----------------------------*/
#define BAUD_CONST 42 // sets up baud rate for 115200
//#define BAUD_CONST 21 // sets up baud rate for 230400

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/*---------------------------- Module Variables ---------------------------*/
static uint8_t xmitBuffer[XMIT_BUFFER_SIZE];
static cbuf_handle_t xmitBufferHandle;

/*------------------------------ Module Code ------------------------------*/
/*******************************************************************************
 * Function: TerminalInit
 * Arguments: None
 * Returns nothing
 * 
 * Created by: R. Merchant
 * Description: Sets up UART1 to serve as the terminal interface. Sets the baud
 * rate and maps the needed pins
 ******************************************************************************/
void Terminal_HWInit(void)
{
//#define USE_RB2_3
#ifdef USE_RB2_3
  // This was the original pin choice, though we changed this for the project
  // Set up RB2 as RX and RB3 as TX
  // clear analog control
  ANSELB &= (BIT3LO & BIT2LO);
  // start with TX high
  LATBbits.LATB3 = 1;
  // set RB3 as output
  TRISBbits.TRISB3 = 0;
  // set RB2 as input
  TRISBbits.TRISB2 = 1;
  // Remap the pins
  RPB3R = 0b0001; // U1TX -> RB3
  U1RXR = 0b0100; // U1RX <- RB2
#else
  // this moves the UART pins to RB6 & RB7, freeing up RB2 & RB3 to be used
  // as analog inputs
  ANSELB &= (BIT6LO & BIT7LO);  // clear analog control
  LATBbits.LATB7 = 1;           // start with TX high
  TRISBbits.TRISB7 = 0;         // set RB7 as output
  TRISBbits.TRISB6 = 1;         // set RB6 as input
  RPB7R = 0b0001;               // map U1TX -> RB7
  U1RXR = 0b0001;               // map U1RX <- RB6
#endif  //USE_RB2_3
  
  // disable the UART to be safe
  U1MODEbits.ON = 0;
  // Setup high-speed mode, data = 8 bit, no parity, 1 stop bit
  // Disable: loop-back, autobaud, wake, inversion
  U1MODEbits.BRGH = 1;
  // Diable TX inversion, everything else we don't care about
  U1STA = 0;
  // Set the baud rate based on the constant
  U1BRG = BAUD_CONST;
  
  // redirect printf to UART1 using X32 built in cross over
  __XC_UART = 1; 
  
  // enable the UART module
  U1STAbits.UTXEN = 1; // enable transmit 
  U1STAbits.URXEN = 1; // enable receive
  U1MODEbits.ON = 1; // turn peripheral on
  
  // now initialize the circular buffer for transmitting
  xmitBufferHandle = circular_buf_init( xmitBuffer, ARRAY_SIZE(xmitBuffer) );
  
  return;
}
/*******************************************************************************
 * Function: Terminal_ReadByte
 * Arguments: None
 * Returns byte
 * 
 * Created by: R. Merchant
 * Description: Read the byte from the receive register
 ******************************************************************************/
uint8_t Terminal_ReadByte(void)
{
  // wait for there to be something
  while(!(U1STAbits.URXDA))
  {}
  // if there was an overrun, clear it to reset the unit
  if(U1STAbits.OERR) U1STAbits.OERR = 0;
  // return the content of the receive register
  return U1RXREG;
}
/*******************************************************************************
 * Function: Terminal_Write
 * Arguments: byte to write
 * Returns nothing
 * 
 * Created by: R. Merchant
 * Description: Writes the byte to the transmit register or buffer
 ******************************************************************************/
void Terminal_WriteByte(uint8_t txByte)
{
#ifdef NO_BUFFER
  // wait for the register to empty
  while(U1STAbits.UTXBF)
  {}
  // write the byte to the register
  U1TXREG = txByte;
#else
  circular_buf_put(xmitBufferHandle, txByte);
#endif  
  return;
}
/*******************************************************************************
 * Function: Terminal_IsRxData
 * Arguments: none
 * Returns status
 * 
 * Created by: R. Merchant
 * Description: Returns true if there is data in the receive register, or false
 *              if not
 ******************************************************************************/
bool Terminal_IsRxData(void)
{
  
    if(U1STAbits.FERR != 0 ){
        U1RXREG; // in case of a framing error, read the data reg to clear err
        return false;
    }
    // Return Rx Data bit from status register
    return U1STAbits.URXDA;
}

/*******************************************************************************
 * Function: _mon_putc
 * Arguments: char c
 * Returns none
 * 
 * Created by: Ed Carryer
 * Description: this is the function that connects the output of printf() to
 *              hardware. In our case, we are going to use it to stuff the
 *              characters into the circular buffer.
 ******************************************************************************/
void _mon_putc (char c)
{
  circular_buf_put(xmitBufferHandle, c);
}

/*******************************************************************************
 * Function: Terminal_MoveBuffer2UART
 * Arguments: none
 * Returns none
 * 
 * Created by: Ed Carryer
 * Description: this functions pulls bytes, if any available, from the
 *              circular buffer and stuffs them into the UART1 buffer
 *              until we either run out of bytes in the circular buffer
 *              or we run out of space in the UART FIFO
 ******************************************************************************/
void Terminal_MoveBuffer2UART( void )
{
  while ( (!circular_buf_empty(xmitBufferHandle)) && (!U1STAbits.UTXBF))
  {
    uint8_t byte2Xmit;
    circular_buf_get(xmitBufferHandle, &byte2Xmit);
    U1TXREG = byte2Xmit;
  }
}

void __attribute__((noreturn)) _fassert(int nLineNumber,
                                        const char * sFileName,
                                        const char * sFailedExpression,
                                        const char * sFunction )
{
  DB_printf("Assert \"%s\" Failed at Line: %d, in File: %s \n\r", 
            sFailedExpression, nLineNumber, sFileName, sFunction);
    // now pump the bytes out of the buffer into the UART
    while(1) 
    {
        Terminal_MoveBuffer2UART();
    }
}
/***************************************************************************
 private functions
 ***************************************************************************/
// module test harness:
#ifdef TEST
int main(void)
{
  
  // initialize
  //_HW_Oscillator_Init();
  Terminal_HWInit();
  // test a print
  printf("Hello World! Let's show that it can handle a long string as well\n\r");
  //Terminal_WriteByte('H');
  while(1) //hang out in this loop forever, polling key hits
  {
    Terminal_MoveBuffer2UART(); // move bytes from circ buffer to UART
    // a delay loop to let me see that the buffering is working as expected.
    { volatile uint32_t counter;
      for (counter=0; counter< 3276800; counter++)
      {}
    }
    // test for tracking keys
    if(IsNewKeyReady())
    {
      // make sure we are getting the correct key
      uint8_t key = GetNewKey();
      printf("\rNew Key Received-> %c <- in test loop\r\n", key);
    }
  }
  return 0;
}
#endif
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/
