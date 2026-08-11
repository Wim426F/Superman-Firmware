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

/* Last VCFRONT_CMPPowerLimit the compressor echoed back on 0x2A6. Mirrored into
 * Param::compressor_echo_plim, kept here as well because the request id sweep
 * polls it faster than the UI does. Written from the CAN RX interrupt, read by
 * the 100ms task, hence volatile. */
static volatile uint16_t lastEchoPlim = 0;

#define ECHO_PLIM_NONE 0x1FFF // what the unit reports while it holds no valid request

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

/* VCFRONT_CMPPowerLimit must never be sent as 0W - the real car holds it at
 * 8191W permanently, and the compressor apparently refuses to start if it
 * reads 0. Fall back to that value if the param is unset/0. */
static int GetEffectivePowerLimit()
{
    int plim = Param::GetInt(Param::compressor_plim);
    return (plim > 0) ? plim : 8191;
}


/* Extract a little endian signal, DBC "@1+" convention: bit numbering runs
 * LSB-first within each byte and carries on into the next byte. */
static uint32_t GetBits(const uint8_t* b, int start, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; i++)
    {
        int bit = start + i;
        v |= (uint32_t)((b[bit >> 3] >> (bit & 7)) & 1) << i;
    }
    return v;
}


void Compressor::handle2A7(uint32_t data[2])
{
    /*
    0x2A7 - Compressor state, 100ms

    Bit  0, len 11: speedRPM                 x10, RPM
    Bit 11, len 10: speedDuty                x0.1, %
    Bit 21, len 11: inputHVPower             x10, W
    Bit 32, len  9: inputHVCurrent           x0.1, A   (not stored, no param)
    Bit 41, len 11: inputHVVoltage           x0.5, V
    Bit 55, len  1: powerLimitActive                   (not stored, no param)
    Bit 56, len  4: state
    Bit 60, len  2: wasteHeatState                     (not stored, no param)
    Bit 62, len  1: powerLimitTooLowToStart
    Bit 63, len  1: ready

    state: 0=INIT 1=RUNNING 2=STANDBY 3=FAULT 4=IDLE 15=SNA
    */

    const uint8_t* b = (const uint8_t*)data;

    last2A7Rx = rtc_get_counter_val();

    int state = GetBits(b, 56, 4);

    Param::SetInt(Param::compressor_speed, GetBits(b, 0, 11) * 10);
    Param::SetFloat(Param::compressor_duty, GetBits(b, 11, 10) * 0.1f);
    Param::SetFloat(Param::compressor_HV, GetBits(b, 41, 11) * 0.5f);
    Param::SetInt(Param::compressor_state, state);
    Param::SetInt(Param::compressor_ready, GetBits(b, 63, 1));
    Param::SetInt(Param::compressor_plim_low, GetBits(b, 62, 1));

    // The unit meters its own HV input, so this is measured rather than estimated.
    Param::SetInt(Param::compressor_power, GetBits(b, 21, 11) * 10);

    if (state == 3 || state == 15) // FAULT or SNA
        ErrorMessage::Post(ERR_COMPRESSOR_FAULT);
}


void Compressor::handle2A6(uint32_t data[2])
{
    /*
    0x2A6 - echo of the request frame, 100ms. Same layout as the request:
    bytes 0-1 target duty, bytes 2-3 power limit, byte 4 reset, byte 5 enable.

    The power limit field reads ECHO_PLIM_NONE while the unit holds no valid
    request, which is what makes the request id discoverable - see SweepTick().
    */

    const uint8_t* b = (const uint8_t*)data;

    uint16_t echo = b[2] | (b[3] << 8);
    lastEchoPlim = echo;

    Param::SetInt(Param::compressor_echo_plim, echo);
    Param::SetInt(Param::compressor_cmd_ok, (echo != ECHO_PLIM_NONE) ? 1 : 0);
}


void Compressor::handle366(uint32_t data[2])
{
    /*
    0x366 - info, 2.5s, muxed on byte 0. Only the identifying mux is decoded,
    the rest are ignored.
    */

    const uint8_t* b = (const uint8_t*)data;

    if (b[0] == 0x0A)
    {
        Param::SetInt(Param::compressor_hwid, b[4] | (b[5] << 8));
        Param::SetInt(Param::compressor_componentid, b[6] | (b[7] << 8));
    }
}


/* 0x2C7 - 100ms, alongside 0x2A7/0x2A6. Layout partially identified.
 *
 * Observed over 615 frames on a bench log, only one bit ever changed:
 *
 *   38.74s  00 00 00 00 00 45 46 FF
 *   42.64s  00 01 00 00 00 45 46 FF
 *
 * Byte 1 bit 0 set 100ms before 0x2A7 dropped ready and fell from IDLE to
 * STANDBY, having sat 3.9s with no request arriving. Hypothesis: byte 1 is a
 * fault/status byte and bit 0 is the request timeout, successor to
 * CMP_VCFRONTCANTimeout which lived in 0x227 on the older unit. Unverified -
 * individual bits are not named until a valid request has been seen to clear
 * it. Exposed as a raw byte for now.
 *
 * Bytes 5 and 6 held 0x45 and 0x46 for the whole log. With the -40 offset used
 * throughout Tesla thermal messages that reads 29C and 30C, and ambient at the
 * time was 29C. Assumed to be temperatures, one likely the inverter, the other
 * ambient or suction. Which is which is unverified.
 *
 * Byte 7 constant 0xFF, assumed SNA. Bytes 0, 2, 3, 4 constant 0x00.
 * 
 * Byte 1 with no HV power is 17.
 * After HV powerup byte is 1.
 *
 * Structurally this is closer to the old 0x227 than 0x2A7 is: temperature plus
 * fault flags in one message, with speed and state split out into 0x2A7.
 */
void Compressor::handle2C7(uint32_t data[2])
{
    const uint8_t* b = (const uint8_t*)data;

    Param::SetInt(Param::compressor_flags, b[1]);
    Param::SetInt(Param::compressor_temp1, (int)b[5] - 40);
    Param::SetInt(Param::compressor_temp2, (int)b[6] - 40);
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
 * Compressor request
 *
 * Bytes 0-1: target duty in 0.1% increments (little endian).
 *            Example: 40% -> 400 -> 0x90 0x01
 * Bytes 2-3: power limit in watts (little endian).
 *            Example: 3000W -> 0xB8 0x0B
 * Byte 4:    reset (kept at 0)
 * Byte 5:    enable (1 to enable, 0 to disable)
 * Bytes 6-7: reserved (kept at 0)
 *
 * The id this goes out on isn't known from the logs, so it has to be found
 * with the sweep below and lives in Param::compressor_cmdid. Until that param
 * is set we transmit nothing rather than guess an id.
 * --------------------------------------------------------------------- */

static void SendRequest(CanHardware* can)
{
    int cmdId = Param::GetInt(Param::compressor_cmdid);
    if (cmdId == 0) return; // request id not discovered yet

    int max_power = GetEffectivePowerLimit(); // Power limit in watts; never sent as 0
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

    can->Send(cmdId, (uint32_t*)bytes, 8); // Every 100ms - never arbitrated, compressor times out without it
}


/* ---------------------------------------------------------------------
 * Request id discovery sweep
 *
 * Walk candidate ids sending a probe that carries a power limit no other node
 * would ask for, and watch what 0x2A6 echoes back. The unit only echoes a
 * request it has accepted, so seeing our probe value come back identifies the
 * id it listens on.
 *
 * Duty and enable stay 0 in every probe, so landing on the right id can never
 * start the compressor.
 * --------------------------------------------------------------------- */

#define SWEEP_PROBE_PLIM  6000 // distinctive value we look for coming back on 0x2A6
#define SWEEP_DWELL_TICKS 4    // 400ms per candidate at the 100ms call rate

static bool sweepRunning = false;
static uint16_t sweepId = 0;
static uint8_t sweepDwell = 0;

static void StopSweep()
{
    sweepRunning = false;
    Param::SetInt(Param::compressor_sweep, 0);
}

// Returns true while the sweep owns the request slot.
static bool SweepTick(CanHardware* can)
{
    if (Param::GetInt(Param::compressor_sweep) == 0)
    {
        sweepRunning = false;
        return false;
    }

    if (!sweepRunning)
    {
        sweepRunning = true;
        sweepId = Param::GetInt(Param::compressor_sweep_lo);
        sweepDwell = 0;
        lastEchoPlim = 0;
    }
    else if (++sweepDwell >= SWEEP_DWELL_TICKS)
    {
        sweepDwell = 0;

        if (lastEchoPlim == SWEEP_PROBE_PLIM) // the unit took our probe: this is the id
        {
            Param::SetInt(Param::compressor_cmdid, sweepId);
            StopSweep();
            return false;
        }

        sweepId++;
        lastEchoPlim = 0; // don't let the previous candidate's echo score a hit

        if (sweepId > Param::GetInt(Param::compressor_sweep_hi))
        {
            StopSweep();
            return false;
        }
    }

    Param::SetInt(Param::compressor_sweep_pos, sweepId);

    uint8_t bytes[8] = {0};
    bytes[2] = lowByte(SWEEP_PROBE_PLIM);
    bytes[3] = highByte(SWEEP_PROBE_PLIM);
    // bytes 0-1 duty, byte 4 reset and byte 5 enable all stay 0

    can->Send(sweepId, (uint32_t*)bytes, 8);
    return true;
}


void Compressor::SendMessages(CanHardware* can)
{
    if (last2A7Rx != 0 && (rtc_get_counter_val() - last2A7Rx) >= COMPRESSOR_RX_TIMEOUT_TICKS)
        ErrorMessage::Post(ERR_COMPRESSOR_TIMEOUT);

    // While sweeping, the probe replaces the normal request so we never drive
    // two ids in one tick. The VCFRONT emulation below keeps running either way.
    if (!SweepTick(can))
        SendRequest(can);

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