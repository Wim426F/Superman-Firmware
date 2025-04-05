#include "interface.h"



void Interface::handle730(uint32_t data[2])  //params
{
   uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes
}

void Interface::handle731(uint32_t data[2])  // Setpoints and actual temperatures
{
    uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes
  
    // parameters (not saved to flash)
    // only use parameters if CAN-IO is selected
    if(Param::GetInt(Param::canio) == _canio::CAN_IO)
    {
        // sensor values
        Param::SetFloat(Param::temp_battery,   (float)(bytes[0]) - 50);
        Param::SetFloat(Param::temp_cabin_left,   (float)(bytes[2]) - 50);
        Param::SetFloat(Param::temp_cabin_right,   (float)(bytes[4]) - 50);   

        const Param::Attributes *pAtr = Param::GetAttrib((Param::temp_battery_setp));
        Param::SetFloat(Param::temp_battery_setp,   (float)utils::limitVal(bytes[1], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));

        pAtr = Param::GetAttrib((Param::temp_cabinl_setp));
        Param::SetFloat(Param::temp_cabinl_setp,   (float)utils::limitVal(bytes[3], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));

        pAtr = Param::GetAttrib((Param::temp_cabinr_setp));
        Param::SetFloat(Param::temp_cabinr_setp,   (float)utils::limitVal(bytes[5], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));
    }
}


void Interface::SendMessages(CanHardware* can)
{
    uint8_t bytes[8];
    Param::SetInt(Param::compressor_speed, 800);
    int32_t speed = Param::GetInt(Param::compressor_speed);
    
    //Heat pump manifold values 1
    bytes[0] = Param::GetInt(Param::temp_inlet_compressor) + 50;
    bytes[1] = Param::GetInt(Param::temp_outlet_compressor) + 50;
    bytes[2] = Param::GetInt(Param::temp_pre_evaporator) + 50;
    bytes[3] = Param::GetInt(Param::pressure_inlet_compressor) * 5;
    bytes[4] = Param::GetInt(Param::pressure_outlet_compressor) * 5;
    bytes[5] = Param::GetInt(Param::pressure_pre_evaporator) * 5;
    bytes[6] = Param::GetInt(Param::pump_battery_flow) * 10;
    bytes[7] = Param::GetInt(Param::pump_powertrain_flow) * 10;
    can->Send(0x732, (uint32_t*)bytes,8); // every 100ms

    //Heat pump manifold values 2
    bytes[0] = Param::GetInt(Param::temp_inlet_battery) + 50;
    bytes[1] = Param::GetInt(Param::temp_inlet_powertrain) + 50;
    bytes[2] = Param::GetInt(Param::temp_reservoir) + 50;
    bytes[3] = 50;  // pt outlet temp
    bytes[4] = Param::GetInt(Param::radiator_pwm) + 50;
    bytes[5] = Param::GetInt(Param::shutterservo) + 50;
    bytes[6] = Param::GetInt(Param::octo_pos);
    bytes[7] = utils::change(speed, 800,8000, 20,250);
    can->Send(0x733, (uint32_t*)bytes,8); // every 100ms
}