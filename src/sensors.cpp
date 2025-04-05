#include "sensors.h"
#include "math.h"
#include "Ewma.h"


Ewma ps1_filter(0.5);
Ewma ps2_filter(0.5);
Ewma ps3_filter(0.5);

Ewma ps1t_filter(0.5);
Ewma ps2t_filter(0.5);
Ewma ps3t_filter(0.5);

Ewma battin_filter(0.5);
Ewma ptinfilter   (0.5);
Ewma reserv_filter(0.5);

Ewma ntc1_filter(0.5);
Ewma ntc2_filter(0.5);
Ewma ntc3_filter(0.5);
Ewma ntc4_filter(0.5);

Ewma radiator_filter(0.5);
Ewma ambient_filter(0.5);
Ewma cabin_left_filter(0.5);
Ewma cabin_right_filter(0.5);
Ewma battery_filter(0.5);
Ewma powertrain_filter(0.5);

float HIGHPRESSURE_SENSOR = 37.265; // rated pressure for high side pressure sensor
float LOWPRESSURE_SENSOR = 10.859;  // rated pressure for low side pressure sensor

float temp_ps1_offset = 3;
float temp_ps2_offset = 3;
float temp_ps3_offset = 3;

float temp_ntc1_offset = 0;
float temp_ntc2_offset = 0;
float temp_ntc3_offset = 0;
float temp_ntc4_offset = 0;

float temp_battin_offset = 0;
float temp_ptin_offset = 0;
float temp_reserv_offset = 0;


void GetTemps()
{
    float pps1 = (float)AnaIn::pressure_inlet_compressor.Get(); // low side sensor
    float pps2 = (float)AnaIn::pressure_outlet_compressor.Get(); // high side sensor
    float pps3 = (float)AnaIn::pressure_pre_evaporator.Get(); // low side sensor
    
    // needs an offset because sensor starts measuring above certain treshold
    pps1 = utils::changeFloat(pps1, 590, 4096, 0, LOWPRESSURE_SENSOR); // low side sensor
    pps2 = utils::changeFloat(pps2, 370, 4096, 0, HIGHPRESSURE_SENSOR); // high side sensor
    pps3 = utils::changeFloat(pps3, 590, 4096, 0, LOWPRESSURE_SENSOR); // low side sensor

    Param::SetFloat(Param::pressure_inlet_compressor,   ps1_filter.filter(pps1)); // low side sensor
    Param::SetFloat(Param::pressure_outlet_compressor,  ps2_filter.filter(pps2)); // high side sensor
    Param::SetFloat(Param::pressure_pre_evaporator,     ps3_filter.filter(pps3)); // low side sensor

    // resistance gets lower when hotter (ntc)
    Param::SetFloat(Param::temp_inlet_compressor,   ps1t_filter.filter(TempMeas::Lookup(AnaIn::temp_inlet_compressor.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps1_offset);
    Param::SetFloat(Param::temp_outlet_compressor,  ps2t_filter.filter(TempMeas::Lookup(AnaIn::temp_outlet_compressor.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps2_offset);
    Param::SetFloat(Param::temp_pre_evaporator,     ps3t_filter.filter(TempMeas::Lookup(AnaIn::temp_pre_evaporator.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps3_offset);

    Param::SetFloat(Param::temp_inlet_battery,      battin_filter.filter( TempMeas::Lookup(AnaIn::temp_inlet_battery.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_battin_offset);
    Param::SetFloat(Param::temp_inlet_powertrain,   ptinfilter.filter   ( TempMeas::Lookup(AnaIn::temp_inlet_powertrain.Get()  , TempMeas::TEMP_TESLA_10K) ) - temp_ptin_offset);
    Param::SetFloat(Param::temp_reservoir,          reserv_filter.filter( TempMeas::Lookup(AnaIn::temp_reservoir.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_reserv_offset);

    Param::SetFloat(Param::temp_radiator,       radiator_filter.filter(TempMeas::Lookup(adc.analogRead(0), TempMeas::TEMP_TESLA_10K)));
    Param::SetFloat(Param::temp_ambient,        ambient_filter.filter(TempMeas::Lookup(adc.analogRead(1), TempMeas::TEMP_TESLA_10K)));
    Param::SetFloat(Param::temp_cabin_left,     cabin_left_filter.filter(TempMeas::Lookup(adc.analogRead(2), TempMeas::TEMP_TESLA_10K)));
    Param::SetFloat(Param::temp_cabin_right,    cabin_right_filter.filter(TempMeas::Lookup(adc.analogRead(3), TempMeas::TEMP_TESLA_10K)));
    Param::SetFloat(Param::temp_battery,        battery_filter.filter(TempMeas::Lookup(adc.analogRead(4), TempMeas::TEMP_TESLA_10K)));
    Param::SetFloat(Param::temp_powertrain,     powertrain_filter.filter(TempMeas::Lookup(adc.analogRead(5), TempMeas::TEMP_TESLA_10K)));
}