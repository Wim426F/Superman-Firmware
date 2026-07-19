/*
 * This file is part of the Superman heatpump controller project.
 *
 * Copyright (C) 2025 Wim Boone <wim.boone@outlook.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pumps.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>

// Volatile variables updated by TIM4 ISR in main.cpp
extern volatile uint32_t pump_batt_period;
extern volatile bool pump_batt_ready;
extern volatile uint32_t pump_pt_period;
extern volatile bool pump_pt_ready;


/* Tesla checksum: sum of bytes 0..6 plus both CAN ID bytes, truncated to 8 bits.
 * Verified against logged 0x221/0x3A1/0x545 frames from a real Model 3. */
static uint8_t CalcTeslaChecksum(const uint8_t* bytes, uint16_t id)
{
    uint16_t sum = (id & 0xFF) + (id >> 8);
    for (int i = 0; i < 7; i++) sum += bytes[i];
    return sum & 0xFF;
}

/* VCFRONT_CMPPowerLimit must never be sent as 0W - the real car holds it at
 * 8191W permanently, and the compressor apparently refuses to start if it
 * reads 0. Fall back to that value if the param is unset/0. */
static int GetEffectivePowerLimit()
{
    int plim = Param::GetInt(Param::compressor_plim);
    return (plim > 0) ? plim : 8191;
}


void Compressor::handle227(uint32_t data[2])
{
    /*
    0x227 - Compressor state feedback (Model 3)

    Byte 0-1: CMP_speedRPM (RPM)
    Byte 2-3: CMP_speedDuty (in 0.1% increments)
    Byte 4:   CMP_inverterTemperature (Celsius, offset by -40)
    Byte 5:   Status Flags (bits 40-47)
    Byte 6:   Additional Fault Flags
    Byte 7:   CMP_state (bits 0-3), CMP_ready (bit 7)

    Fault/Status Flags (Byte 5, bits 40-47):
    Bit 0: CMP_HVOverVoltage
    Bit 1: CMP_HVUnderVoltage
    Bit 2: CMP_overTemperature
    Bit 3: CMP_underTemperature
    Bit 4: CMP_VCFRONTCANTimeout
    Bit 5: CMP_overCurrent
    Bit 6: CMP_currentSensorCal
    Bit 7: CMP_failedStart

    Byte 6 Additional Faults:
    Bit 0: CMP_motorVoltageSat
    Bit 1: CMP_shortCircuit
    Bit 2: CMP_repeatOverCurrent

    Byte 7:
    Bits 0-3: CMP_state (0=NONE 1=NORMAL 2=WAIT 3=FAULT 4=SOFT_START 5=SOFT_SHUTDOWN 15=SNA)
    Bit 7:    CMP_ready
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

    // Fault Flags from Byte 5 (uncomment as params are added)
//  Param::SetInt(Param::compressor_HVOverVoltage, bytes[5] & 0x01);
//  Param::SetInt(Param::compressor_HVUnderVoltage, bytes[5] & 0x02);
//  Param::SetInt(Param::compressor_overTemperature, bytes[5] & 0x04);
//  Param::SetInt(Param::compressor_underTemperature, bytes[5] & 0x08);
    Param::SetInt(Param::compressor_CANTimeout, (bytes[5] & 0x10) ? 1 : 0);
//  Param::SetInt(Param::compressor_overCurrent, bytes[5] & 0x20);
//  Param::SetInt(Param::compressor_currentSensorCal, bytes[5] & 0x40);
//  Param::SetInt(Param::compressor_failedStart, bytes[5] & 0x80);

    // Additional Fault Flags from Byte 6
//  Param::SetInt(Param::compressor_motorVoltageSat, bytes[6] & 0x01);
//  Param::SetInt(Param::compressor_shortCircuit, bytes[6] & 0x02);
//  Param::SetInt(Param::compressor_repeatOverCurrent, bytes[6] & 0x04);

    // Compressor state and ready flag (byte 7)
    int cmp_state = bytes[7] & 0x0F;
    bool cmp_ready = (bytes[7] & 0x80) != 0;
    Param::SetInt(Param::compressor_state, cmp_state);
    Param::SetInt(Param::compressor_ready, cmp_ready ? 1 : 0);

    // Rough HV power estimate: this compressor variant doesn't report power on CAN.
    // power ~= measured duty% x configured power limit.
    Param::SetInt(Param::compressor_power, (int)(duty * GetEffectivePowerLimit() / 100.0f));
}


/* ---------------------------------------------------------------------
 * VCFRONT emulation & collision arbitration
 *
 * Other open-source controllers on the same bus (e.g. a PCS charger
 * controller) may emulate some of the same VCFRONT ids. Since bxCAN never
 * receives its own transmissions, any RX on one of these ids proves another
 * node owns it - so we stay quiet on that id until it goes silent again.
 * --------------------------------------------------------------------- */

enum EmuMsgIdx { EMU_221 = 0, EMU_2D1, EMU_321, EMU_3A1, EMU_545, EMU_COUNT };
static const uint16_t emuIds[EMU_COUNT] = { 0x221, 0x2D1, 0x321, 0x3A1, 0x545 };
static uint32_t emuLastHeard[EMU_COUNT] = { 0, 0, 0, 0, 0 };
static uint8_t emuActiveMask = 0;

#define EMU_BOOT_LISTEN_TICKS      150 // 1500ms, RTC counts at 10ms/tick (see rtc_setup())
#define EMU_SILENCE_TIMEOUT_TICKS   50 // 500ms

static bool MayTransmit(int idx)
{
    int canemu = Param::GetInt(Param::canemu);
    if (canemu == 1) return true;  // always transmit
    if (canemu == 2) return false; // never transmit

    // auto: stay quiet during the boot listen window, then take over unless
    // we've heard the other node inside the silence timeout.
    uint32_t now = rtc_get_counter_val();
    if (now < EMU_BOOT_LISTEN_TICKS) return false;

    uint32_t lastHeard = emuLastHeard[idx];
    return (lastHeard == 0) || ((now - lastHeard) > EMU_SILENCE_TIMEOUT_TICKS);
}

static void SetActiveBit(int idx, bool active)
{
    if (active) emuActiveMask |= (1 << idx);
    else        emuActiveMask &= ~(1 << idx);
    Param::SetInt(Param::emu_active, emuActiveMask);
}

void Compressor::HandleEmuRx(uint32_t id)
{
    for (int i = 0; i < EMU_COUNT; i++)
    {
        if (emuIds[i] == id)
        {
            emuLastHeard[i] = rtc_get_counter_val();
            break;
        }
    }
}

// VCFRONT_LVPowerState - alternating mux, single shared counter incremented every frame
static void SendMsg221(CanHardware* can)
{
    static uint8_t mux = 0;
    static uint8_t count = 0;

    uint8_t bytes[8] = {0};
    if (!mux) {           // mux 1: CMPDLVState = LV_ON, pcsLVState = LV_ON
        bytes[0]=0x41; bytes[1]=0x01; bytes[2]=0x05; bytes[3]=0x00;
        bytes[4]=0x00; bytes[5]=0x00;
        bytes[6]=(count << 4) | 0x0;
    } else {                 // mux 0: hvacCompLVState = LV_ON
        bytes[0]=0x40; bytes[1]=0x41; bytes[2]=0x05; bytes[3]=0x15;
        bytes[4]=0x00; bytes[5]=0x50;
        bytes[6]=(count << 4) | 0x1;
    }
    bytes[7] = CalcTeslaChecksum(bytes, 0x221);

    bool active = MayTransmit(EMU_221);
    if (active) can->Send(0x221, (uint32_t*)bytes, 8);
    SetActiveBit(EMU_221, active);

    mux = !mux;
    count = (count + 1) & 0x0F;
}

// VCFRONT_okToUseHighPower - static, no counter/checksum
static void SendMsg2D1(CanHardware* can)
{
    bool active = MayTransmit(EMU_2D1);
    if (active)
    {
        uint8_t bytes[8] = {0xFF, 0x01, 0, 0, 0, 0, 0, 0};
        can->Send(0x2D1, (uint32_t*)bytes, 2);
    }
    SetActiveBit(EMU_2D1, active);
}

// VCFRONT_sensors - static, no counter/checksum
static void SendMsg321(CanHardware* can)
{
    bool active = MayTransmit(EMU_321);
    if (active)
    {
        uint8_t bytes[8] = {0xCB, 0x25, 0xA7, 0x65, 0x02, 0x5F, 0x00, 0x00};
        can->Send(0x321, (uint32_t*)bytes, 8);
    }
    SetActiveBit(EMU_321, active);
}

// VCFRONT_vehicleStatus - static body, counter + checksum
static void SendMsg3A1(CanHardware* can)
{
    static uint8_t count = 0;

    uint8_t bytes[8];
    bytes[0]=0x89; bytes[1]=0x42; bytes[2]=0x72; bytes[3]=0x85;
    bytes[4]=0x01; bytes[5]=0x2C;
    bytes[6]=(count << 4) | 0x2;
    bytes[7]=CalcTeslaChecksum(bytes, 0x3A1);

    bool active = MayTransmit(EMU_3A1);
    if (active) can->Send(0x3A1, (uint32_t*)bytes, 8);
    SetActiveBit(EMU_3A1, active);

    count = (count + 1) & 0x0F;
}

// VCFRONT 10Hz mux - alternating mux, single shared counter incremented every frame
static void SendMsg545(CanHardware* can)
{
    static uint8_t mux = 0;
    static uint8_t count = 0;

    uint8_t bytes[8];
    if (!mux) {
        bytes[0]=0x03; bytes[1]=0x19; bytes[2]=0x64; bytes[3]=0x32;
        bytes[4]=0x19; bytes[5]=0x00;
        bytes[6]=(count << 4);
    } else {
        bytes[0]=0x14; bytes[1]=0x00; bytes[2]=0x3F; bytes[3]=0x70;
        bytes[4]=0x9F; bytes[5]=0x01;
        bytes[6]=(count << 4) | 0xA;
    }
    bytes[7] = CalcTeslaChecksum(bytes, 0x545);

    bool active = MayTransmit(EMU_545);
    if (active) can->Send(0x545, (uint32_t*)bytes, 8);
    SetActiveBit(EMU_545, active);

    mux = !mux;
    count = (count + 1) & 0x0F;
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

    int max_power = GetEffectivePowerLimit(); // Power limit in watts; never sent as 0
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

    can->Send(0x281, (uint32_t*)bytes, 8); // Always sent, every 100ms - never arbitrated, compressor times out without it

    SendMsg2D1(can);
    SendMsg3A1(can);

    // 0x321 only needs to appear at 1Hz; throttle our 100ms cadence down by 10.
    static uint8_t divider321 = 0;
    if (++divider321 >= 10)
    {
        divider321 = 0;
        SendMsg321(can);
    }
}

void Compressor::Send50ms(CanHardware* can)
{
    SendMsg221(can);
    SendMsg545(can);
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

void Waterpump::batterySetDuty(uint8_t duty) // in %
{
    Param::SetInt(Param::pump_battery_duty, duty);
    duty = utils::limitVal(duty, 0, 80); // driver doesnt seem to work above 80%
    pwm_write(duty, PWM_PUMP_BATT_TIM, PWM_PUMP_BATT_OC, PWM_PUMP_BATT_ARR);
}

void Waterpump::powertrainSetDuty(uint8_t duty) // in %
{
    Param::SetInt(Param::pump_powertrain_duty, duty);
    duty = utils::limitVal(duty, 0, 80); // driver doesnt seem to work above 80%
    pwm_write(duty, PWM_PUMP_PT_TIM, PWM_PUMP_PT_OC, PWM_PUMP_PT_ARR);
}

// TIM4 runs at 500 kHz (72 MHz / (143+1) prescaler), 2µs per tick
// Period in ticks, convert to RPM: (500,000 / period_ticks) * 60
// Returns last measured value (updated by ISR at input frequency rate)
// TODO: Add RPM-to-flow lookup table for centrifugal pump curve
float Waterpump::batteryGetFlow()
{
    if (pump_batt_period == 0) return 0.0f;
    return 30000000.0f / pump_batt_period;  // RPM (will be converted to LPM later)
}

float Waterpump::powertrainGetFlow()
{
    if (pump_pt_period == 0) return 0.0f;
    return 30000000.0f / pump_pt_period;  // RPM (will be converted to LPM later)
}