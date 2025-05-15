/****************************************************************************
 Module
     PIC32_AD_Lib.h

 Description
     Header file for the  module implementing  the basic functions to use the A/D 
	 converter on the PIC32MX170F256B
 Notes


 History
 When           Who     What/Why
 -------------- ---     --------

 10/20/20 17:38 jec     Began Coding
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#ifndef PIC_32_Lib_H
#define PIC_32_Lib_H

#include <stdint.h>
#include <stdbool.h>

bool ADC_ConfigAutoScan( uint16_t whichPins);
void ADC_MultiRead(uint32_t *adcResults);

#endif  //PIC_32_Lib_H