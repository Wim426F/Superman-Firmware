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

float hs_psensor = 37.265; // max pressure for high side pressure sensor
float ls_psensor = 10.859;

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
    float pps1 = (float)AnaIn::ps1_signal.Get(); // low side sensor
    float pps2 = (float)AnaIn::ps2_signal.Get(); // high side sensor
    float pps3 = (float)AnaIn::ps3_signal.Get(); // low side sensor
    
    // needs an offset because sensor starts measuring above certain treshold
    pps1 = utils::changeFloat(pps1, 590, 4096, 0, ls_psensor); // low side sensor
    pps2 = utils::changeFloat(pps2, 370, 4096, 0, hs_psensor); // high side sensor
    pps3 = utils::changeFloat(pps3, 590, 4096, 0, ls_psensor); // low side sensor

    Param::SetFloat(Param::ps1_pressure, ps1_filter.filter(pps1)); // low side sensor
    Param::SetFloat(Param::ps2_pressure, ps2_filter.filter(pps2)); // high side sensor
    Param::SetFloat(Param::ps3_pressure, ps3_filter.filter(pps3)); // low side sensor

    // resistance gets lower when hotter (ntc)
    Param::SetFloat(Param::ps1_temperature, ps1t_filter.filter(TempMeas::Lookup(AnaIn::temp_ps1.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps1_offset);
    Param::SetFloat(Param::ps2_temperature, ps2t_filter.filter(TempMeas::Lookup(AnaIn::temp_ps2.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps2_offset);
    Param::SetFloat(Param::ps3_temperature, ps3t_filter.filter(TempMeas::Lookup(AnaIn::temp_ps3.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_ps3_offset);

    Param::SetFloat(Param::batt_inlet_temp, battin_filter.filter( TempMeas::Lookup(AnaIn::temp_battin.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_battin_offset);
    Param::SetFloat(Param::pt_inlet_temp,   ptinfilter.filter   ( TempMeas::Lookup(AnaIn::temp_ptin.Get()  , TempMeas::TEMP_TESLA_10K) ) - temp_ptin_offset);
    Param::SetFloat(Param::reservoir_temp,  reserv_filter.filter( TempMeas::Lookup(AnaIn::temp_reserv.Get(), TempMeas::TEMP_TESLA_10K) ) - temp_reserv_offset);

    Param::SetFloat( Param::ext_temp1, TempMeas::Lookup(AnaIn::ntc_1.Get(), (TempMeas::Sensors)Param::GetInt(Param::ext_ntc)) - temp_ntc1_offset);
    Param::SetFloat( Param::ext_temp2, TempMeas::Lookup(AnaIn::ntc_2.Get(), (TempMeas::Sensors)Param::GetInt(Param::ext_ntc)) - temp_ntc2_offset);
    Param::SetFloat( Param::ext_temp3, TempMeas::Lookup(AnaIn::ntc_3.Get(), (TempMeas::Sensors)Param::GetInt(Param::ext_ntc)) - temp_ntc3_offset);
    Param::SetFloat( Param::ext_temp4, TempMeas::Lookup(AnaIn::ntc_4.Get(), (TempMeas::Sensors)Param::GetInt(Param::ext_ntc)) - temp_ntc4_offset);
}