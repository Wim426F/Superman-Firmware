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
        Param::SetFloat(Param::batt_temp,   (float)(bytes[0]) - 50);
        Param::SetFloat(Param::cabinl_temp,   (float)(bytes[2]) - 50);
        Param::SetFloat(Param::cabinr_temp,   (float)(bytes[4]) - 50);   

        const Param::Attributes *pAtr = Param::GetAttrib((Param::batt_setp));
        Param::SetFloat(Param::batt_setp,   (float)utils::limitVal(bytes[1], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));

        pAtr = Param::GetAttrib((Param::cabinl_setp));
        Param::SetFloat(Param::cabinl_setp,   (float)utils::limitVal(bytes[3], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));

        pAtr = Param::GetAttrib((Param::cabinr_setp));
        Param::SetFloat(Param::cabinr_setp,   (float)utils::limitVal(bytes[5], FP_TOINT(pAtr->min), FP_TOINT(pAtr->max)));
    }
}


void Interface::SendMessages(CanHardware* can)
{
    uint8_t bytes[8];
    Param::SetInt(Param::compressor_speed, 800);
    int32_t speed = Param::GetInt(Param::compressor_speed);
    
    //Heat pump manifold values 1
    bytes[0] = Param::GetInt(Param::ps1_temperature) + 50;
    bytes[1] = Param::GetInt(Param::ps2_temperature) + 50;
    bytes[2] = Param::GetInt(Param::ps3_temperature) + 50;
    bytes[3] = Param::GetInt(Param::ps1_pressure) * 5;
    bytes[4] = Param::GetInt(Param::ps2_pressure) * 5;
    bytes[5] = Param::GetInt(Param::ps3_pressure) * 5;
    bytes[6] = Param::GetInt(Param::pumpa_flow) * 10;
    bytes[7] = Param::GetInt(Param::pumpb_flow) * 10;
    can->Send(0x732, (uint32_t*)bytes,8); // every 100ms

    //Heat pump manifold values 2
    bytes[0] = Param::GetInt(Param::batt_inlet_temp) + 50;
    bytes[1] = Param::GetInt(Param::pt_inlet_temp) + 50;
    bytes[2] = Param::GetInt(Param::reservoir_temp) + 50;
    bytes[3] = 50;  // pt outlet temp
    bytes[4] = Param::GetInt(Param::radiator_pwm) + 50;
    bytes[5] = Param::GetInt(Param::shutterservo) + 50;
    bytes[6] = Param::GetInt(Param::octo_pos);
    bytes[7] = utils::change(speed, 800,8000, 20,250);
    can->Send(0x733, (uint32_t*)bytes,8); // every 100ms
}