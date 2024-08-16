#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here we specify how many samples are combined into one filtered result. Following values are possible:
*  - NUM_SAMPLES = 1: Most recent raw value is returned
*  - NUM_SAMPLES = 3: Median of last 3 values is returned
*  - NUM_SAMPLES = 9: Median of last 3 medians is returned
*  - NUM_SAMPLES = 12: Average of last 4 medians is returned
*/
#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC //Sample&Hold time for each pin. Increases sample time, might increase accuracy 

//Here you specify a list of analog inputs, see main.cpp on how to use them
#define ANA_IN_LIST \
   ANA_IN_ENTRY(ps2_signal, GPIOA, 0) \
   ANA_IN_ENTRY(ps1_signal, GPIOA, 1) \
   ANA_IN_ENTRY(uaux,       GPIOA, 4) \
   ANA_IN_ENTRY(temp_ptin,  GPIOA, 5) \
   ANA_IN_ENTRY(temp_battin,GPIOA, 6) \
   ANA_IN_ENTRY(temp_reserv,GPIOA, 7) \
   ANA_IN_ENTRY(ntc_2,      GPIOB, 0) \
   ANA_IN_ENTRY(ntc_1,      GPIOB, 1) \
   ANA_IN_ENTRY(temp_ps1,   GPIOC, 0) \
   ANA_IN_ENTRY(temp_ps2,   GPIOC, 1) \
   ANA_IN_ENTRY(temp_ps3,   GPIOC, 2) \
   ANA_IN_ENTRY(ps3_signal, GPIOC, 3) \
   ANA_IN_ENTRY(ntc_4,      GPIOC, 4) \
   ANA_IN_ENTRY(ntc_3,      GPIOC, 5) \

#endif // ANAIN_PRJ_H_INCLUDED
