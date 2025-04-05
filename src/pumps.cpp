#include "pumps.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#define DRV_OFF 1      //Enable Motor Drive

uint8_t drv_address = 0x0;
int flag1 = 0;
int8_t rxLen = 0;
int8_t len = 0;

void Compressor::handle2A8(uint32_t data[2])
{
    /*
    0x2A8 - Compressor Status (CMPD_state)

    Bytes 0-1: Compressor speed in RPM (bits 0-10), scale 10 RPM/bit
    Bytes 1-2: Speed duty cycle (bits 11-20), scale 0.1%/bit
    Bytes 2-4: Input HV power (bits 21-31), scale 10W/bit
    Bytes 4-5: Input HV current (bits 32-40), scale 0.1A/bit
    Bytes 5-6: Input HV voltage (bits 41-51), scale 0.5V/bit
    Byte 7: Various state flags (bits 55-63)
    */

    uint8_t* bytes = (uint8_t*)data; // Convert the two 32-bit words into bytes

    // Extract RPM
    uint16_t rpm_raw = (bytes[0] | ((bytes[1] & 0x07) << 8)); // 11 bits
    Param::SetInt(Param::compressor_speed, rpm_raw * 10);

    // Extract Duty Cycle
    uint16_t duty_raw = ((bytes[1] >> 3) | (bytes[2] << 5)) & 0x3FF; // 10 bits
    Param::SetFloat(Param::compressor_duty, duty_raw * 0.1);

    // Extract Input HV Power
    uint16_t power_raw = ((bytes[2] >> 6) | (bytes[3] << 2) | ((bytes[4] & 0x01) << 10)); // 11 bits
    Param::SetInt(Param::compressor_power, power_raw * 10);

    // Extract Input HV Current
    uint16_t current_raw = ((bytes[4] >> 1) | ((bytes[5] & 0x01) << 7)); // 9 bits
    Param::SetFloat(Param::compressor_amps, current_raw * 0.1);

    // Extract Input HV Voltage
    uint16_t voltage_raw = ((bytes[5] >> 1) | (bytes[6] << 7)) & 0x7FF; // 11 bits
    Param::SetFloat(Param::compressor_HV, voltage_raw * 0.5);

    // Extract States
    bool powerLimitActive = bytes[6] & 0x80;
    bool powerLimitTooLowToStart = bytes[7] & 0x40;
    bool ready = bytes[7] & 0x80;
    uint8_t state = (bytes[7] >> 0) & 0x0F;
    uint8_t wasteHeatState = (bytes[7] >> 4) & 0x03;

//  Param::SetInt(Param::compressor_powerLimitActive, powerLimitActive);
//  Param::SetInt(Param::compressor_powerLimitTooLowToStart, powerLimitTooLowToStart);
//  Param::SetInt(Param::compressor_ready, ready);
//  Param::SetInt(Param::compressor_state, state);
//  Param::SetInt(Param::compressor_wasteHeatState, wasteHeatState);
}


void Compressor::handle227(uint32_t data[2])
{
    /*
    0x227 - Compressor state feedback (Model 3)

    Byte 0-1: CMP_speedRPM (RPM)
    Byte 2-3: CMP_speedDuty (in 0.1% increments)
    Byte 4:   CMP_inverterTemperature (Celsius, offset by -40)
    Byte 5:   Status Flags (each bit represents a different fault or status)
    Byte 6:   CMP_state (4 bits), other 4 bits reserved
    Byte 7:   CMP_ready (bit 7)

    Fault/Status Flags (Byte 5):
    Bit 0: CMP_HVOverVoltage
    Bit 1: CMP_HVUnderVoltage
    Bit 2: CMP_overTemperature
    Bit 3: CMP_underTemperature
    Bit 4: CMP_VCFRONTCANTimeout
    Bit 5: CMP_overCurrent
    Bit 6: CMP_motorVoltageSat
    Bit 7: CMP_currentSensorCal

    Byte 6 Additional Faults:
    Bit 0: CMP_failedStart
    Bit 1: CMP_shortCircuit
    Bit 2: CMP_repeatOverCurrent
    */

    uint8_t* bytes = (uint8_t*)data; // Convert to byte array

    // Read RPM
    int rpm = (bytes[1] << 8) | bytes[0];
    //Param::SetInt(Param::compressor_rpm, rpm);

    // Read Duty Cycle in 0.1%
    float duty = ((bytes[3] << 8) | bytes[2]) * 0.1;
    Param::SetFloat(Param::compressor_duty, duty);

    // Inverter temperature
    int inverter_temp = bytes[4] - 40;
    Param::SetInt(Param::compressor_temp, inverter_temp);

    // Fault Flags from Byte 5
//  Param::SetInt(Param::compressor_HVOverVoltage, bytes[5] & 0x01);
//  Param::SetInt(Param::compressor_HVUnderVoltage, bytes[5] & 0x02);
//  Param::SetInt(Param::compressor_overTemperature, bytes[5] & 0x04);
//  Param::SetInt(Param::compressor_underTemperature, bytes[5] & 0x08);
//  Param::SetInt(Param::compressor_CANTimeout, bytes[5] & 0x10);
//  Param::SetInt(Param::compressor_overCurrent, bytes[5] & 0x20);
//  Param::SetInt(Param::compressor_motorVoltageSat, bytes[5] & 0x40);
//  Param::SetInt(Param::compressor_currentSensorCal, bytes[5] & 0x80);

    // Additional Fault Flags from Byte 6
//  Param::SetInt(Param::compressor_failedStart, bytes[6] & 0x01);
//  Param::SetInt(Param::compressor_shortCircuit, bytes[6] & 0x02);
//  Param::SetInt(Param::compressor_repeatOverCurrent, bytes[6] & 0x04);

    // Compressor State
    int cmp_state = bytes[6] >> 4; // Upper 4 bits
//    Param::SetInt(Param::compressor_state, cmp_state);

    // Compressor Ready Flag
    bool cmp_ready = bytes[7] & 0x80; // Bit 7
//  Param::SetBool(Param::compressor_ready, cmp_ready);
}


void Compressor::SendMessages(CanHardware* can)
{
    /*
    0x281 - 8 Bytes of data (new DBC format)

    Bytes 0-1: VCFRONT_CMPTargetDuty in 0.1% increments (little endian).
               Example: 40% -> 400 -> 0x90 0x01
    Bytes 2-3: VCFRONT_CMPPowerLimit in watts (little endian).
               Example: 3000W -> 0xB8 0x0B
    Byte 4:    VCFRONT_CMPReset (set to 0)
    Byte 5:    VCFRONT_CMPEnable (1 to enable, 0 to disable)
    Bytes 6-7: Reserved (set to 0)
    */

    int max_power = Param::GetInt(Param::compressor_plim); // Power limit in watts
    int compressor_duty = Param::GetInt(Param::compressor_duty_request); // Duty cycle in 0.1%

    bool compressor_enable = (compressor_duty > 0);

    uint8_t bytes[8];
    bytes[0] = lowByte(compressor_duty);          // VCFRONT_CMPTargetDuty LSB
    bytes[1] = highByte(compressor_duty);         // VCFRONT_CMPTargetDuty MSB
    bytes[2] = lowByte(max_power);                // VCFRONT_CMPPowerLimit LSB
    bytes[3] = highByte(max_power);               // VCFRONT_CMPPowerLimit MSB
    bytes[4] = 0x00;                              // VCFRONT_CMPReset
    bytes[5] = compressor_enable ? 0x01 : 0x00;   // VCFRONT_CMPEnable
    bytes[6] = 0x00;                              // Reserved
    bytes[7] = 0x00;                              // Reserved

    can->Send(0x281, (uint32_t*)bytes, 8); // Send message on VehicleBus
}


void Compressor::SetDuty(int duty)
{
    duty = utils::limitVal(duty, 0, 100);
    duty *= 10; // canbus message value is in 0.1% increments

    Param::SetInt(Param::compressor_duty_request, duty);
}

int Compressor::GetDuty()
{
    return Param::GetInt(Param::compressor_duty);
}

void Waterpump::batterySetFlow(int lpm)
{
    //waterpumpA.Control.setSpeed(lpm); // hex:ABCD
    //waterpumpA.getStatus();
    
}

void Waterpump::powertrainSetFlow(int lpm)
{
    //waterpumpB.Control.setSpeed(5000);
//    waterpumpA.getStatus();

}

float Waterpump::batteryGetFlow()
{
    float pump_battery_flow = 0;

    // Pump PWM feedback
    if (timer_get_flag(TIM4, TIM_SR_CC3IF))
    {
       pump_battery_flow = timer_get_ic_value(TIM4, TIM_IC3);
       Param::SetFloat(Param::pump_battery_flow, pump_battery_flow);
    }
   
    return pump_battery_flow;
}

float Waterpump::powertrainGetFlow()
{
    float pump_powertrain_flow = 0;

    // Pump PWM feedback
    if (timer_get_flag(TIM4, TIM_SR_CC4IF))
    {
       pump_powertrain_flow = timer_get_ic_value(TIM4, TIM_IC4);
       Param::SetFloat(Param::pump_powertrain_flow, pump_powertrain_flow);
    }
    return pump_powertrain_flow;
}