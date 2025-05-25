//#define TEST
/****************************************************************************
  Module
    dbprintf.c

  Description
    This is a module implementing  a printf() like function that has been
    stripped down to reduce its code size & memory usage.  The only format
    specifiers  recognized are : %d, %x, %u, %c, %s . It can not print
    floats or longs. If it is called with a format specifier other than
    those recognized, it will print BAD. Any values after that are garbage.
    Floats must be explicitly  cast to int before printing.

  Notes
    The maximum line length from a single call to DB_printf() is LINE_LEN
    characters. This is the size of an allocated buffer. If you exceed this,
    you will overrun the stack. The length of any number field in the
    resulting line can not be longer than FIELD_LEN.

 History
 When           Who     What/Why
 -------------- ---     --------
 10/06/20 22:53 ram     updated to use the terminal module. Updated variable 
                        names to make MPLAB happy during parsing
 05/15/02 21:40 jec      converted to use SC1 for use in me218c project master
 02/23/98 21:40 jec      made cl & ch explicitly signed in the test program
                         this is because introl defaults to unsigned chars
 02/22/98 23:32 jec      removed itoa, we only really need uitoa
 02/15/98 10:34 jec      Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include "terminal.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/
// increased line length because the assert() lines can get long)
#define LINE_LEN    120
// increased FIELD_LEN to accommodate 32 bit values for ints
#define FIELD_LEN   11

#define CR 0x0d
#define LF 0x0a
/*---------------------------- Module Functions ---------------------------*/
static void uitoa(char **buf, unsigned int i, unsigned int baseNum);

/*---------------------------- Module Variables ---------------------------*/

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    DB_printf

 Parameters
    a char * format string, followed by a variable number of arguments

 Returns
    None.

 Description
    a printf() like function that has been
    stripped down to reduce its code size & memory usage.  The only format
    specifiers  recognized are : %d, %x, %u, %c, %s . 
 Notes
    It can not print floats or longs. 
    If it is called with a format specifier other than those recognized, 
    it will print BAD. Any values after that are garbage.
    Floats must be explicitly  cast to int before printing.
 Author
    J. Edward Carryer, 05/15/02 21:51
****************************************************************************/
void DB_printf(const char *Format, ...)
{
  va_list Arguments;
  char *pBuffer;
  char *pString;
  int   i;
	unsigned int u;
  char  LineBuffer[LINE_LEN+1];

  va_start(Arguments,Format);
  pBuffer = LineBuffer;
  *pBuffer = 0;                 /* make sure that Line starts out NULL term */
  while (*Format)               /* step through the format string */
  {    
    if (*Format != '%')            /* if not a format specifier */
    {      
      *pBuffer++ = *Format++;  /* simply copy to the output buffer */
    }else
    {
       switch (*++Format)         /* otherwise see what kind of format spec */
       {
          case 'd':               /* %d, decimal signed number */
             i = va_arg(Arguments,int);
             if (i < 0)
             {
                *pBuffer++ = '-'; /* add '-' to the buffer for neg. numbers */
                i = -1*i;         /* and continue with the positive version */
             }
             uitoa(&pBuffer, (unsigned int)i, 10);
             break;
          case 'x':               /* %x, hexadecimal unsigned number */
             u = va_arg(Arguments,unsigned int);
//               *pBuffer++ = '0'; removed to allow cleaner printing of longs
//               *pBuffer++ = 'x';
             uitoa(&pBuffer, u, 16);
             break;
          case 'u':               /* %u, decimal unsigned number */
             u = va_arg(Arguments,unsigned int);
             uitoa(&pBuffer, u, 10);
             break;
          case 'c':               /* %c, a single character */
             *pBuffer++ = (char) va_arg(Arguments,unsigned int);
             break;
          case 's':               /* %s, a string of characters */
             pString = va_arg(Arguments,char *);
             if (!pString)
                pString = "(null)";
             while (*pString)
                *pBuffer++ = *pString++;
             break;
          case '%':               /* quoted % */
             *pBuffer++ = '%';
              break;
          default:                /* anything else is a bad spec. */
              *pBuffer++ = 'B';
              *pBuffer++ = 'A';
              *pBuffer++ = 'D';
              break;
       }
       Format++;
    }
  }
  *pBuffer = 0;                     /* null terminate the output string */

/* now, spit the built up line out 1 character at a time */
   for (pBuffer = LineBuffer; *pBuffer != 0; pBuffer++)
   {
      if (*pBuffer != '\n')
      {   
        putchar(*pBuffer);
      }else
      {
         putchar(CR);
         putchar(LF);
      }
   }
   return;
}
/* integer to ascii conversion for unsigned numbers  */
static void uitoa(char **LineBuffer, unsigned int i, unsigned int baseNum)
{
  char *s;
  uint8_t remainder;
  char FieldBuf[FIELD_LEN + 1];

  FieldBuf[FIELD_LEN] = 0;      /*start by NULL terminating the local buffer */
  
  // we treat a value of 0 as a special case, since it would break the
  // general algorithm
  if (i == 0)
  {
    (*LineBuffer)[0] = '0'; // just copy the '0' to the return buffer
    ++(*LineBuffer);        // and advance the pointer past the '0'
    return;
  }
  // we will fill the FieldBuf from the LS position
  s = &FieldBuf[FIELD_LEN];
  while (i)
  {
    remainder = i % baseNum;
    *--s = "0123456789abcdef"[remainder];
    i /= baseNum;
  }
  while (*s)                    /* copy local buffer into passed buffer */
  {
    (*LineBuffer)[0] = *s++;
    ++(*LineBuffer);
  }
}


#ifdef TEST
#define LONGTEST
#include <xc.h>
#include <limits.h>

#define UINT_MIN 0
#define UCHAR_MIN 0

void main(void)
{
#ifdef LONGTEST
  unsigned char ucl=UCHAR_MIN;
  unsigned char uch=UCHAR_MAX;
  unsigned int  uil=UINT_MIN;
  unsigned int  uih=UINT_MAX;
  int   il = INT_MIN;
  int   ih = INT_MAX;
  signed char  cl = SCHAR_MIN;
  signed char  ch = SCHAR_MAX;
  char  c='A';
  char  String[]="Hello World\n";
  float Floater = 1.23;
  unsigned long LongOne = 123456789;

  Terminal_HWInit();
  DB_printf("\nBeginning DB_printf() test compiled at %s on %s\n\n", __TIME__, __DATE__);

   DB_printf("Printing an unsigned char UCHAR_MIN, Decimal mode(%%u): %u, Hex mode: %x\n\n",ucl, ucl);

   DB_printf("Printing an unsigned char UCHAR_MAX, Decimal mode(%%u): %u, Hex mode: %x\n\n",uch, uch);

   DB_printf("Printing an unsigned int UINT_MIN, Decimal mode(%%u): %u, Hex mode: %x\n\n",uil,uil);

   DB_printf("Printing an unsigned int UINT_MAX, Decimal mode(%%u): %u\n",uih);
   DB_printf("Printing an unsigned int UIINT_MAX, Hex mode: %x\n\n",uih);

   DB_printf("Printing an signed char SCHAR_MIN, Decimal mode(%%d): %d\n",cl);
   DB_printf("Printing an signed char SCHAR_MAX, Decimal mode(%%d):  %d\n\n",ch);
   
   DB_printf("Printing an signed int INT_MIN, Decimal mode(%%d): %d\n",il);
   DB_printf("Printing an signed int INT_MAX, Decimal mode(%%d):  %d\n\n",ih);

   DB_printf("Printing a char as a single character: %c\n", c);
   DB_printf("Printing a string w/ embedded NL: %s\n\n", String);

   DB_printf("Attempting to print a long: %l\n",LongOne);
   DB_printf("Attempting to print a float: %f\n",Floater);
//   DB_printf("A way to print a long value: %x%x\n", HIWORD(LongOne),LOWORD(LongOne));

#else
   Terminal_HWInit();
  DB_printf("\nBeginning DB_printf() test:\n");
  DB_printf( "compiled at %s on %s\n", __TIME__, __DATE__);
   DB_printf("Hello World!\n");
#endif  //LONGTEST

  while(1) //hang out in this loop forever, moving bytes to UART until finished
  {
    Terminal_MoveBuffer2UART(); // move bytes from circ buffer to UART
  }
}
#endif // TEST

