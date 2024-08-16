#include "pumps.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#define DRV_OFF 1      //Enable Motor Drive

uint8_t drv_address = 0x0;
int flag1 = 0;
int8_t rxLen = 0;
int8_t len = 0;

void Compressor::handle223(uint32_t data[2])
{
    /*
    0x223 - Various status flags

    Bytes 0 and 1 encode the speed of the compressor in RPM. (Little endian as all signals for the compressor are encoded)
    Bytes 2 and 3 encode the output duty cycle in tenths of a percent
    Byte 4 encodes the temperature of the compressor's built-in inverter. This is degrees Centigrade + 40. (So, a reading of 0 means -40C)
    Byte 5 has many status bits. If any of them are 1 you're going to have a bad time.
    Byte 6 continues the error flags. You want this value to also read zero.
    Byte 7 the top bit (0x80 / bit 7) is "Compressor is Ready!" The lower 4 bits are the compressor status but only the bottom 2 bits are used. 0 = None, 1 = Normal, 2 = Wait, 3 = Faulted
    As an example: 0x223 68 05 0E 00 41 00 00 81 means 1384RPM, 1.4% duty cycle, 25C inverter, no faults, compressor is ready, status is normal
    */
    uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes

    Param::SetInt(Param::compressor_speed, byte_to_int(bytes[0], bytes[1]));
    Param::SetInt(Param::compressor_duty, byte_to_int(bytes[2], bytes[3]));
    Param::SetInt(Param::compressor_temp, bytes[4] + 40);
}

void Compressor::handle233(uint32_t data[2])
{
    /*
    0x233 - High voltage status

    Bytes 0 and 1 encode the high voltage reading in tenths of a volt. This signal, as all others, is little endian.
    Byte 2 encodes the 12V input voltage. In theory it should be in tenths of a volt but I have it reporting as 0xFF
    Bytes 3 and 4 encode the compressor current draw in tenths of an amp
    Bytes 5 and 6 encode the actual power draw of the compressor in watts
    As an example: 0x233 65 0E FF 0B 00 94 01 00 means 368.5V, 1.1A, 404 watts (LV stuck at 25.5V)
    */
    uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes

    Param::SetInt(Param::compressor_HV, byte_to_int(bytes[0], bytes[1]));
    Param::SetFloat(Param::compressor_LV, (bytes[2]/10));
    Param::SetFloat(Param::compressor_amps, (byte_to_int(bytes[3], bytes[4]) / 10) );
    Param::SetInt(Param::compressor_power, byte_to_int(bytes[5], bytes[6]));

}

void Compressor::SendMessages(CanHardware* can)
{
    /*
    0x28A - 8 Bytes of data

    Bytes 0 and 1 encode the target duty cycle for the compressor in 0.1% increments. For instance, 40% is then 400 as a value. Values are stored little endian and so byte 0 is the low byte and byte 1 is the high byte. 400 will be encoded as 0x90 0x01
    Bytes 2 and 3 encode the maximum power draw allowable in watts. This is also stored little endian. For instance, 3000W is 0xB8 0x0B
    Byte 4 can be set to 0
    Byte 5 must be set to 1 to enable the compressor
    Bytes 6 and 7 can be set to 0
    */

    int max_power = Param::GetInt(Param::compressor_plim); // power limit in watts
    int compressor_duty = Param::GetInt(Param::compressor_duty_request);

    bool compressor_enable = (compressor_duty > 0) ? true : false;
    
    uint8_t bytes[8];
    bytes[0] = lowByte(compressor_duty); // in 0.1%
    bytes[1] = highByte(compressor_duty);
    bytes[2] = lowByte(max_power); // in watts
    bytes[3] = highByte(max_power);
    bytes[4] = 0x00; // always 0
    bytes[5] = compressor_enable; // must be set to 1 to enable the compressor
    bytes[6] = 0x00; // always 0
    bytes[7] = 0x00; // always 0

    can->Send(0x28A, (uint32_t*)bytes,8); // every 100ms
}

void Compressor::SetDuty(int duty)
{
    duty = utils::limitVal(duty, 0, 100);
    duty *= 10; // canbus message value is in 0.1% increments

    Param::SetInt(Param::compressor_duty_request, duty);
}

void Waterpump::setFlowA(int lpm)
{
    waterpumpA.Control.setSpeed(lpm); // hex:ABCD
    //waterpumpA.getStatus();
    //Param::SetInt(Param::pumpa_udc, waterpumpA.Variables.getVoltageVM());
    //Param::SetInt(Param::pumpa_flow, waterpumpA.Variables.getSpeed());
    //Param::SetInt(Param::pumpb_udc, waterpumpA.Config.setIsdEnable(false)); // for now only get isd register
    //Param::SetInt(Param::pumpa_current, waterpumpA.Variables.getCurrentDC());
}

void Waterpump::setFlowB(int lpm)
{
    //waterpumpB.Control.setSpeed(5000);
//    waterpumpA.getStatus();
//    Param::SetInt(Param::pumpb_flow, waterpumpB.Variables.getSpeed());
//    Param::SetInt(Param::pumpb_udc, waterpumpB.Variables.getVoltageVM());
//    Param::SetInt(Param::pumpb_current, waterpumpB.Variables.getCurrentDC());
}

float Waterpump::getFlowA()
{
    float pumpa_flow = 0;

    // Pump PWM feedback
    if (timer_get_flag(TIM4, TIM_SR_CC3IF))
    {
       pumpa_flow = timer_get_ic_value(TIM4, TIM_IC3);
       Param::SetFloat(Param::pumpa_flow, pumpa_flow);
    }
   
    return pumpa_flow;
}

float Waterpump::getFlowB()
{
    float pumpb_flow = 0;

    // Pump PWM feedback
    if (timer_get_flag(TIM4, TIM_SR_CC4IF))
    {
       pumpb_flow = timer_get_ic_value(TIM4, TIM_IC4);
       Param::SetFloat(Param::pumpb_flow, pumpb_flow);
    }
    return pumpb_flow;
}