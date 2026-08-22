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
#include "errormessage.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>

// Volatile variables updated by TIM4 ISR in main.cpp
extern volatile uint32_t pump_batt_period;
extern volatile bool pump_batt_ready;
extern volatile uint32_t pump_pt_period;
extern volatile bool pump_pt_ready;

#define COMPRESSOR_RX_TIMEOUT_TICKS 50  // 500ms
static uint32_t last2A7Rx = 0;

#define PUMP_STALL_TIMEOUT_TICKS 200 // 2000ms
#define PUMP_STALL_MIN_DUTY 5       // %, below this we don't expect flow


/* Tesla checksum: sum of bytes 0..6 plus both CAN ID bytes, truncated to 8 bits.
 * Verified against logged 0x221/0x3A1/0x545 frames from a real Model 3. */
static uint8_t CalcTeslaChecksum(const uint8_t* bytes, uint16_t id)
{
    uint16_t sum = (id & 0xFF) + (id >> 8);
    for (int i = 0; i < 7; i++) sum += bytes[i];
    return sum & 0xFF;
}

void Compressor::handle2A7(uint32_t data[2])
{
    /*
    0x2A7 - Compressor state, 100ms

    Bit  0, len 11: speedRPM                 x10, RPM
    Bit 11, len 10: speedDuty                x0.1, %   actual duty, not a command echo
    Bit 21, len 11: inputHVPower             x10, W
    Bit 32, len  9: inputHVCurrent           x0.1, A
    Bit 41, len 11: inputHVVoltage           x0.5, V
    Bit 55, len  1: powerLimitActive
    Bit 56, len  4: state : 0=INIT 1=RUNNING 2=STANDBY 3=FAULT 4=IDLE 15=SNA
    Bit 60, len  2: wasteHeatState
    Bit 62, len  1: powerLimitTooLowToStart
    Bit 63, len  1: ready
    */

    const uint8_t* b = (const uint8_t*)data;

    last2A7Rx = rtc_get_counter_val();

    int rpm   = b[0] | ((b[1] & 0x07) << 8);        // bits 0-10
    int duty  = (b[1] >> 3) | ((b[2] & 0x1F) << 5); // bits 11-20
    int power = (b[2] >> 5) | (b[3] << 3);          // bits 21-31
    int amps  = b[4] | ((b[5] & 0x01) << 8);        // bits 32-40
    int volts = (b[5] >> 1) | ((b[6] & 0x0F) << 7); // bits 41-51
    int state = b[7] & 0x0F;                        // bits 56-59

    Param::SetInt(Param::compressor_speed, rpm * 10);
    Param::SetFloat(Param::compressor_duty, duty * 0.1f);
    Param::SetFloat(Param::compressor_amps, amps * 0.1f);
    Param::SetFloat(Param::compressor_HV, volts * 0.5f);
    Param::SetInt(Param::compressor_state, state);
    Param::SetInt(Param::compressor_ready, b[7] >> 7);             // bit 63
    //Param::SetInt(Param::compressor_plim_low, (b[7] >> 6) & 0x01); // bit 62, perhaps error flag: power limit too low to start.

    // The unit meters its own HV input, so this is measured rather than estimated.
    Param::SetInt(Param::compressor_power, power * 10);

    if (state == 3 || state == 15) // FAULT or SNA
        ErrorMessage::Post(ERR_COMPRESSOR_FAULT);
}


void Compressor::handle366(uint32_t data[2])
{
    /*
    0x366 - info, 2.5s, muxed on byte 0. Only the identifying mux is decoded.
    */

    const uint8_t* b = (const uint8_t*)data;

    if (b[0] == 0x0A)
    {
        Param::SetInt(Param::compressor_hwid, b[4] | (b[5] << 8));
        Param::SetInt(Param::compressor_componentid, b[6] | (b[7] << 8));
    }
}


/* 0x2C7 - 100ms. Flags plus temperatures.
 *
 * Byte 1 bit 0: request timeout. clears when request on 0x2A1 arrives.
 * Byte 1 bit 4: HV not present.
 * Byte 5:       temperature, offset -40. Reads 0x00 on some units.
 * Byte 6:       temperature, offset -40.
 */

#define CMP_FLAG_TIMEOUT       0x01 // byte 1 bit 0
#define CMP_FLAG_HV_MISSING    0x10 // byte 1 bit 4
#define CMP_FLAG_TIMEOUT_TICKS 100  // 1000ms the flag must hold before we call it a fault

void Compressor::handle2C7(uint32_t data[2])
{
    static uint32_t flagTimeoutSince = 0;

    const uint8_t* b = (const uint8_t*)data;

    Param::SetInt(Param::compressor_flags, b[1]);
    //Param::SetInt(Param::compressor_hv_missing, (b[1] & CMP_FLAG_HV_MISSING) ? 1 : 0);
    Param::SetInt(Param::compressor_temp1, (int)b[5] - 40);
    Param::SetInt(Param::compressor_temp2, (int)b[6] - 40);

    /* The compressor raises this flag during the gap between its own power up and our
     * first request, so require it to hold before treating it as a fault. Once
     * requests are flowing it clears in the very next frame. */
    if (b[1] & CMP_FLAG_TIMEOUT)
    {
        uint32_t now = rtc_get_counter_val();

        if (flagTimeoutSince == 0)
            flagTimeoutSince = now;
        else if ((now - flagTimeoutSince) > CMP_FLAG_TIMEOUT_TICKS)
            ErrorMessage::Post(ERR_COMPRESSOR_TIMEOUT);
    }
    else
    {
        flagTimeoutSince = 0;
    }
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


/* ---------------------------------------------------------------------
 * Compressor request 0x2A1, DLC 8.
 *
 * Bytes 0-1: target duty in 0.1% increments (little endian).
 *            Example: 40% -> 400 -> 0x90 0x01
 * Bytes 2-3: power limit in watts (little endian).
 *            Example: 3000W -> 0xB8 0x0B
 * Byte 4:    reset (kept at 0)
 * Byte 5:    enable (1 to enable, 0 to disable)
 * Bytes 6-7: reserved (kept at 0)
 * --------------------------------------------------------------------- */

static void SendMsg2A1(CanHardware* can)
{
    int max_power = Param::GetInt(Param::compressor_plim);
    int compressor_duty = Param::GetInt(Param::compressor_duty_request); // Duty cycle in 0.1%

    bool compressor_enable = (compressor_duty > 0);

    uint8_t bytes[8];
    bytes[0] = lowByte(compressor_duty);          // target duty LSB
    bytes[1] = highByte(compressor_duty);         // target duty MSB
    bytes[2] = lowByte(max_power);                // power limit LSB
    bytes[3] = highByte(max_power);               // power limit MSB
    bytes[4] = 0x00;                              // reset
    bytes[5] = compressor_enable ? 0x01 : 0x00;   // enable
    bytes[6] = 0x00;                              // reserved
    bytes[7] = 0x00;                              // reserved

    can->Send(0x2A1, (uint32_t*)bytes, 8); // Every 100ms - never arbitrated, compressor times out without it
}


void Compressor::SendMessages(CanHardware* can)
{
    if (last2A7Rx != 0 && (rtc_get_counter_val() - last2A7Rx) >= COMPRESSOR_RX_TIMEOUT_TICKS)
        ErrorMessage::Post(ERR_COMPRESSOR_TIMEOUT);

    SendMsg2A1(can); // Compressor request
    SendMsg2D1(can);
    SendMsg3A1(can);

    // 0x321 only needs to appear at 1Hz
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

void Waterpump::checkFaults()
{
    static uint32_t battLastEdge = 0;
    static uint32_t ptLastEdge = 0;
    uint32_t now = rtc_get_counter_val();

    if (pump_batt_ready) { pump_batt_ready = false; battLastEdge = now; }
    if (pump_pt_ready)   { pump_pt_ready = false;   ptLastEdge = now; }

    // When commanded on but no tach pulse detected, the rotor isn't spinning.
    if (Param::GetInt(Param::pump_battery_duty) > PUMP_STALL_MIN_DUTY &&
        (now - battLastEdge) >= PUMP_STALL_TIMEOUT_TICKS)
        ErrorMessage::Post(ERR_PUMP_BATTERY_FAULT);

    if (Param::GetInt(Param::pump_powertrain_duty) > PUMP_STALL_MIN_DUTY &&
        (now - ptLastEdge) >= PUMP_STALL_TIMEOUT_TICKS)
        ErrorMessage::Post(ERR_PUMP_POWERTRAIN_FAULT);
}