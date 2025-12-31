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
   ANA_IN_ENTRY(pressure_outlet_compressor,     GPIOA, 0) \
   ANA_IN_ENTRY(pressure_pre_evaporator,        GPIOA, 1) \
   ANA_IN_ENTRY(temp_radiator,                  GPIOA, 2) \
   ANA_IN_ENTRY(temp_ambient,                   GPIOA, 3) \
   ANA_IN_ENTRY(temp_battery,                   GPIOA, 4) \
   ANA_IN_ENTRY(temp_powertrain,                GPIOA, 5) \
   ANA_IN_ENTRY(potmeter_cabinr,                GPIOA, 6) \
   ANA_IN_ENTRY(potmeter_cabinl,                GPIOA, 7) \
   ANA_IN_ENTRY(temp_inlet_battery,             GPIOB, 0) \
   ANA_IN_ENTRY(reservoir_level,                GPIOB, 1) \
   ANA_IN_ENTRY(temp_inlet_compressor,          GPIOC, 0) \
   ANA_IN_ENTRY(temp_outlet_compressor,         GPIOC, 1) \
   ANA_IN_ENTRY(temp_pre_evaporator,            GPIOC, 2) \
   ANA_IN_ENTRY(pressure_inlet_compressor,      GPIOC, 3) \
   ANA_IN_ENTRY(uaux,                           GPIOC, 4) \
   ANA_IN_ENTRY(temp_inlet_powertrain,          GPIOC, 5) \

#endif // ANAIN_PRJ_H_INCLUDED
