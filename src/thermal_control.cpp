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

#include "thermal_control.h"
#include "errormessage.h"
#include <cstdint>
#include <limits>
#include <algorithm>

// Constants
const float BATTERY_HEAT_THRESHOLD = 5.0f;  // °C
const float BATTERY_COOL_THRESHOLD = 40.0f;  // °C
const float POWERTRAIN_COOL_THRESHOLD = 50.0f;  // °C
const float HIGH_PRESSURE_LIMIT = 30.0f;  // Bar
const float LOW_PRESSURE_LIMIT = 1.0f;    // Bar
const float SELFHEAT_TEMP_THRESHOLD = -20.0f;  // recirc is heatsource below this temp
const float RANK_HYSTERYSIS = 5.0f;            // °C; hysterysis on source/sink selection to avoid octovalve constant changing

// Cascade: inner EXV PI on T, outer compressor on whether that EXV is saturated.
// 100ms loop. EXV never below 40 in use (stall). Outer loop treats <=50 as "can't close more".
const float EXPV_MIN_POS = 40.0f;
const float EXPV_SAT_POS = 50.0f;        // inner loop saturated closed
const float EXPV_START_POS = 100.0f;
const float EXPV_SLEW = 4.0f;            // counts per 100ms (~5s for 40-255)
const float T_HOLD_BAND = 2.0f;          // °C, "at setpoint"

const float COMPRESSOR_START_DUTY = 65.0f; // blind start, then outer loop walks it
const float COMPRESSOR_MIN_RUN = 15.0f;  // 1-15% is useless; go to 0 instead
const float COMPRESSOR_SLEW = 2.0f;      // % per 100ms, ramp to start
const float COMPRESSOR_TRIM = 0.5f;      // % per 100ms, outer loop
const float PRESSURE_SLEW = 5.0f;

// EXV PI: too hot → close. 10K error → 4 counts/tick (slew-limited).
const float Kp = 4.0f;
const float Ki = 0.8f;
const float dt = 0.1f;
static float last_error = 0.0f;
static float lastDutyCmd = 0.0f;
static float dutyTarget = 0.0f;
static float lastValveCmd = 255.0f;
static bool capacityRunning = false;

// Source and sink types
// Cabin is never a source or sink, as control is from its perspective. It can only need a source or sink.
// Powertrain would be nice as source but octovalve cannot but evaporator in front of powertrain. only evaporator->battery->powertrain.
enum class SourceType {AMBIENT, BATTERY, SELFHEAT};
enum class SinkType {AMBIENT, BATTERY};

// Component struct
struct ThermalComponent {
    bool available;
    float temp;
};

// Demands struct
struct ThermalDemands {
    bool cabinLHeating;
    bool cabinRHeating;
    bool cabinCooling;
    bool batteryHeating;
    bool batteryCooling;
    bool powertrainCooling;
    bool radiatorDefrost;
};

// Source and sink arrays
struct SourceData {
    SourceType type;
    ThermalComponent info;
};

struct SinkData {
    SinkType type;
    ThermalComponent info;
};


// Evaluate thermal demands based on temperature setpoints
ThermalDemands assessDemands() {
    ThermalDemands demands = {false, false, false, false, false, false, false};

    // Cabin demands from digital inputs
    demands.cabinLHeating = Param::GetBool(Param::heat_cabinl);
    demands.cabinRHeating = Param::GetBool(Param::heat_cabinr);
    demands.cabinCooling = Param::GetBool(Param::cool_cabin);

    // Battery and powertrain demands from return-line (outlet) coolant temp
    int batteryTemp = Param::GetInt(Param::temp_outlet_battery);
    static bool bBatHeatLatch = false;
    static bool bBatCoolLatch = false;
    static bool bPtCoolLatch = false;

    if (batteryTemp < Param::GetInt(Param::temp_battery_min)) bBatHeatLatch = true;
    else if (batteryTemp > Param::GetInt(Param::temp_battery_min) + RANK_HYSTERYSIS) bBatHeatLatch = false;
    demands.batteryHeating = bBatHeatLatch;

    if (batteryTemp > Param::GetInt(Param::temp_battery_max)) bBatCoolLatch = true;
    else if (batteryTemp < Param::GetInt(Param::temp_battery_max) - RANK_HYSTERYSIS) bBatCoolLatch = false;
    demands.batteryCooling = bBatCoolLatch;

    int powertrainTemp = Param::GetInt(Param::temp_outlet_powertrain);
    if (powertrainTemp > Param::GetInt(Param::temp_powertrain_max)) bPtCoolLatch = true;
    else if (powertrainTemp < Param::GetInt(Param::temp_powertrain_max) - RANK_HYSTERYSIS) bPtCoolLatch = false;
    demands.powertrainCooling = bPtCoolLatch;

    demands.radiatorDefrost = false;  // Initialize false for now

    int thermal_demands = 0;

    thermal_demands |= demands.cabinCooling ? COOLING_CABIN : NONE;
    thermal_demands |= demands.batteryCooling ? COOLING_BATTERY : NONE;
    thermal_demands |= demands.powertrainCooling ? COOLING_POWERTRAIN : NONE;
    thermal_demands |= demands.cabinLHeating ? HEATING_CABINL : NONE;
    thermal_demands |= demands.cabinRHeating ? HEATING_CABINR : NONE;
    thermal_demands |= demands.batteryHeating ? HEATING_BATTERY : NONE;
    thermal_demands |= demands.radiatorDefrost ? RADIATOR_DEFROST : NONE;
    
    Param::SetInt(Param::thermal_demands, thermal_demands);

    return demands;
}


// Make sure cabin always gets as much flow in dual mode heating as in single mode by reducing the coolant condensor valve
static void adjustCondenserSplit(uint8_t& cabinL, uint8_t& cabinR, uint8_t& coolant, uint8_t compressorDuty) {
    // Reduce coolant condensor valve if cabin needs priority
    if (compressorDuty > 90 && (Param::GetInt(Param::temp_condensor_setp) - Param::GetInt(Param::temp_outlet_compressor) > 2.0f)) {
        coolant = std::max(static_cast<uint8_t>(coolant - 5), static_cast<uint8_t>(128)); // Reduce by 10%, min 50%
    } else if (compressorDuty < 80 && (Param::GetInt(Param::temp_battery_min) - Param::GetInt(Param::temp_outlet_battery) > 2.0f)) {
        coolant = std::min(static_cast<uint8_t>(coolant + 5), static_cast<uint8_t>(255)); // Increase by 10%, max 100%
    }
}

// Prioritize cabin cooling: reduce coolant evap opening if overloaded
static void adjustEvaporatorSplit(uint8_t& cabin, uint8_t& coolant, uint8_t compressorDuty) {
    if (compressorDuty > 90 && (Param::GetInt(Param::temp_inlet_compressor) - Param::GetInt(Param::temp_evaporator_setp) > 2.0f)) {
        coolant = std::max(static_cast<uint8_t>(coolant - 5), static_cast<uint8_t>(EXPV_MIN_POS)); // Close coolant more, divert flow to cabin
    } else if (compressorDuty < 80 && (Param::GetInt(Param::temp_outlet_battery) - Param::GetInt(Param::temp_battery_max) > 2.0f)) {
        coolant = std::min(static_cast<uint8_t>(coolant + 5), static_cast<uint8_t>(255)); // Open coolant more if battery needs it
    }
}

// Gather available heat sources and sinks
void getAvailableSourcesSinks(const ThermalDemands& demands, SourceData sources[3], SinkData sinks[2]) {
    float ambientTemp = Param::GetInt(Param::temp_ambient);
    float batteryTemp = Param::GetInt(Param::temp_outlet_battery);
    const float selfheatTemp = SELFHEAT_TEMP_THRESHOLD; // Dummy ranking temp so self-heat only wins below this

    // Sources – always available when physically present
    sources[0] = {SourceType::BATTERY, {!demands.batteryHeating, batteryTemp}}; // Can't source heat from battery if it needs heating
    sources[1] = {SourceType::AMBIENT, {true, ambientTemp}}; // Always a heat source
    sources[2] = {SourceType::SELFHEAT, {true, selfheatTemp}}; // COP=1, compressor dissipation

    // Sinks – availability based on demand (user wants to use it as heat sink)
    sinks[0] = {SinkType::BATTERY, {!demands.batteryCooling, batteryTemp}}; // Battery is heatsink unless it needs cooling
    sinks[1] = {SinkType::AMBIENT, {true, ambientTemp}}; // Always a heatsink
}

// Select hottest source for best heat transfer
SourceType selectBestSource(float targetTemp, const SourceData sources[3]) {
    SourceType bestSource = SourceType::AMBIENT;
    float minDeltaT = std::numeric_limits<float>::max(); // Start with highest float to ensure first delta updates min
    static SourceType lastSource = (SourceType)-1; // init with no source to avoid hysterysis on startup

    // Check each source for smallest temp difference (hottest source)
    for (int i = 0; i < 3; i++) {
        if (sources[i].info.available) {
            float deltaT = targetTemp - sources[i].info.temp; // Calc delta to rank sources
            if (sources[i].type == lastSource) deltaT -= RANK_HYSTERYSIS; // keep last source unless difference > hysterysis
            if (deltaT < minDeltaT) {
                minDeltaT = deltaT;
                bestSource = sources[i].type; // Update if hotter source found
            }
        }
    }

    lastSource = bestSource;
    Param::SetInt(Param::best_source, (int)bestSource);
    return bestSource;
}

// Select coldest sink for best heat rejection
SinkType selectBestSink(float targetTemp, const SinkData sinks[2]) {
    SinkType bestSink = SinkType::AMBIENT;
    float maxDeltaT = -std::numeric_limits<float>::max(); // Start with lowest float to ensure first delta updates max
    static SinkType lastSink = (SinkType)-1; // init with no sink to avoid hysterysis on startup

    // Check each sink for largest temp difference (best heat rejection)
    for (int i = 0; i < 2; i++) {
        if (sinks[i].info.available) {
            float deltaT = targetTemp - sinks[i].info.temp; // Calc delta to rank sinks
            if (sinks[i].type == lastSink) deltaT += RANK_HYSTERYSIS; // keep last sink unless difference > hysterysis
            if (deltaT > maxDeltaT) {
                maxDeltaT = deltaT;
                bestSink = sinks[i].type; // Update if better sink found
            }
        }
    }
    lastSink = bestSink;
    Param::SetInt(Param::best_sink, (int)bestSink);
    return bestSink;
}


// Inner: evap EXV on T (too hot → close). Outer: compressor ṁ from EXV saturation.
// Returns the shared evap EXV command (40-255).
uint8_t runCapacityControl(float setpoint, float measured) {
    if (!capacityRunning) {
        capacityRunning = true;
        last_error = measured - setpoint;
        lastValveCmd = EXPV_START_POS;
        dutyTarget = COMPRESSOR_START_DUTY;
    }

    float highPressure = Param::GetFloat(Param::pressure_outlet_compressor);
    float lowPressure = Param::GetFloat(Param::pressure_pre_evaporator);
    if (highPressure > HIGH_PRESSURE_LIMIT) ErrorMessage::Post(ERR_REFRIGERANT_HIGHP);
    if (lowPressure < LOW_PRESSURE_LIMIT) ErrorMessage::Post(ERR_REFRIGERANT_LOWP);
    bool pressureCut = (highPressure > HIGH_PRESSURE_LIMIT || lowPressure < LOW_PRESSURE_LIMIT);

    // Inner loop: EXV. error > 0 means too hot.
    float error = measured - setpoint;
    float delta = -(Kp * (error - last_error) + Ki * error * dt);
    last_error = error;
    if (delta > EXPV_SLEW) delta = EXPV_SLEW;
    if (delta < -EXPV_SLEW) delta = -EXPV_SLEW;
    lastValveCmd += delta;
    if (lastValveCmd < EXPV_MIN_POS) lastValveCmd = EXPV_MIN_POS;
    if (lastValveCmd > 255.0f) lastValveCmd = 255.0f;

    // Outer loop: compressor. Slow. Don't sit in 1-15%.
    if (pressureCut) {
        dutyTarget = 0.0f;
    } else {
        bool tooHot = (error > T_HOLD_BAND);
        bool tooCold = (error < -T_HOLD_BAND);
        bool atSet = !tooHot && !tooCold;
        bool satClosed = (lastValveCmd <= EXPV_SAT_POS);
        bool satOpen = (lastValveCmd >= 250.0f);

        if (tooHot && satClosed) dutyTarget += COMPRESSOR_TRIM;
        else if (tooCold && satOpen) dutyTarget -= COMPRESSOR_TRIM;
        else if (atSet && lastValveCmd > EXPV_SAT_POS + 10.0f) dutyTarget -= COMPRESSOR_TRIM;

        if (dutyTarget > 100.0f) dutyTarget = 100.0f;
        if (dutyTarget < COMPRESSOR_MIN_RUN) dutyTarget = 0.0f;
        if (tooHot && satClosed && dutyTarget < COMPRESSOR_MIN_RUN)
            dutyTarget = COMPRESSOR_MIN_RUN;
    }

    float slew = pressureCut ? PRESSURE_SLEW : COMPRESSOR_SLEW;
    if (dutyTarget > lastDutyCmd + slew) lastDutyCmd += slew;
    else if (dutyTarget < lastDutyCmd - slew) lastDutyCmd -= slew;
    else lastDutyCmd = dutyTarget;

    Compressor::SetDuty(static_cast<uint8_t>(lastDutyCmd));
    return static_cast<uint8_t>(lastValveCmd);
}



/*                                          */
/*      Manual coolant purge routine        */ 
/*                                          */

// This routine tries to mimmick Tesla coolant air purge mode.
// We run the pumps at varying speeds to try and "knock" the trapped air pockets away.
// This is done while stepping through all octovalve positions to make sure we hit every branch.

static const int PURGE_POS_SEQ[] = {2, 3, 4, 5, 4, 3}; // Octovalve positions to step
static const uint8_t PURGE_DUTY_SEQ[] = {100, 30, 100, 30}; // switch between high/low pump speeds to break air pockets.
static const uint32_t PURGE_DWELL_MS[] = {15000, 5000, 15000, 5000}; // 15s high, 5s low speed for waterpump
static const int PURGE_POS_COUNT = sizeof(PURGE_POS_SEQ) / sizeof(PURGE_POS_SEQ[0]);
static const int PURGE_PHASE_COUNT = sizeof(PURGE_DUTY_SEQ) / sizeof(PURGE_DUTY_SEQ[0]);

static int purge_pos_idx = 0;      // index into PURGE_POS_SEQ
static int purge_duty_idx = 0;     // phase: 100% / 30% / 100% / 30%
static uint32_t purge_dwell_ms = 0; // remaining ms of the current phase, ticked at 100 ms
static bool purge_low_level = false; // reservoir at/below minimum, waiting for refill
static bool purge_active = false;    // purge ran on the previous cycle (clean exit transient)

static void purgeRoutine()
{
    int level = Param::GetInt(Param::reservoir_level);

    // Reservoir at or below minimum: pumps off, hold the current step.
    // Do not advance and do not exit purge. When level recovers, the
    // current step's timer restarts.
    if (level <= COOLANT_MINIMUM) {
        Waterpump::powertrainSetDuty(0);
        Waterpump::batterySetDuty(0);
        purge_low_level = true;
        return;
    }

    if (purge_low_level) {
        purge_low_level = false;
        purge_dwell_ms = PURGE_DWELL_MS[purge_duty_idx]; // restart the current step
    }

    Valve::octoSetPos(PURGE_POS_SEQ[purge_pos_idx]);

    Waterpump::powertrainSetDuty(PURGE_DUTY_SEQ[purge_duty_idx]);
    Waterpump::batterySetDuty(PURGE_DUTY_SEQ[purge_duty_idx]);

    purge_dwell_ms -= 100; // this runs every 100 ms
    if (purge_dwell_ms == 0) {
        if (++purge_duty_idx >= PURGE_PHASE_COUNT) {
            purge_duty_idx = 0;
            purge_pos_idx = (purge_pos_idx + 1) % PURGE_POS_COUNT;
        }
        purge_dwell_ms = PURGE_DWELL_MS[purge_duty_idx];
    }
}


void thermalControl() {
    // Manual coolant purge, inhibits thermal control while running.
    if (Param::GetInt(Param::purge) == 1) {
        if (!purge_active) {
            purge_active = true;
            // Start a fresh sweep and arm the first phase timer.
            purge_pos_idx = 0;
            purge_duty_idx = 0;
            purge_dwell_ms = PURGE_DWELL_MS[0];
            purge_low_level = false;
            // Reset capacity control state so it starts fresh after purge.
            last_error = 0.0f;
            lastDutyCmd = 0.0f;
            dutyTarget = 0.0f;
            lastValveCmd = 255.0f;
            capacityRunning = false;
        }
        Compressor::SetDuty(0);
        Param::SetInt(Param::opmode, 3); // 3 = "Purge"
        purgeRoutine();
        return;
    }

    // Purge just stopped: stop the pumps, leave the octovalve where it is,
    // normal thermal control resumes from the next cycle.
    if (purge_active) {
        purge_active = false;
        Param::SetInt(Param::opmode, 0);
        Waterpump::powertrainSetDuty(0);
        Waterpump::batterySetDuty(0);
        return;
    }

    ThermalDemands demands = assessDemands();
    SourceData sources[3];
    SinkData sinks[2];
    getAvailableSourcesSinks(demands, sources, sinks);

    SourceType bestSource = selectBestSource(Param::GetInt(Param::temp_condensor_setp), sources);
    SinkType bestSink = selectBestSink(Param::GetInt(Param::temp_evaporator_setp), sinks);

    /* Water pump control */
    // Tie waterpump speeds to compressor with minimum and maximum of 20-100% waterpump duty.
    bool reservoirFull = Param::GetInt(Param::reservoir_level) != 0;
    int requestedDuty = utils::limitVal(Param::GetInt(Param::compressor_duty_request), 20, 100);
    int pumpDuty = reservoirFull ? requestedDuty : 0; // If reservoir is empty, refuse to run pumps.
    Waterpump::powertrainSetDuty(static_cast<uint8_t>(pumpDuty));
    Waterpump::batterySetDuty(static_cast<uint8_t>(pumpDuty));
    
    /* External waterpump control */
    // This is only for option of cabin coolant condensor
    if (demands.cabinLHeating || demands.cabinRHeating)
        // only spin pump when cabin heat required. Tie duty to  
        pwm_write(100, PWM_PUMP_TIM, PWM_PUMP_OC, PWM_PUMP_ARR);
    else
        pwm_write(0, PWM_PUMP_TIM, PWM_PUMP_OC, PWM_PUMP_ARR);

    /* Radiatorfan control */
    // only spin radiator if heat must be rejected or absorbed from ambient
    if (bestSink == SinkType::AMBIENT || bestSource == SourceType::AMBIENT)
        // tie fan speed to pump speed as simple proxy for heat flow rate.
        pwm_write(static_cast<uint8_t>(pumpDuty), PWM_FAN_TIM, PWM_FAN_OC, PWM_FAN_ARR); 
    else
        pwm_write(0, PWM_FAN_TIM, PWM_FAN_OC, PWM_FAN_ARR); // Fan off
    

    // SourceType:  AMBIENT, BATTERY, SELFHEAT
    // SinkType:    AMBIENT, BATTERY

    // OctoPos::POS2_SERIES     Condensor -> Radiator -> Evaporator -> Battery -> Powertrain
    // OctoPos::POS3_AMBIENT    {Condensor -> Battery -> Powertrain}  +  {Evaporator -> Radiator}
    // OctoPos::POS4_RBYPASS    Condensor -> Evaporator -> Battery -> Powertrain
    // OctoPos::POS5_PARALLEL   {Condensor -> Radiator -> Powertrain}  +  {Evaporator -> Battery}


    // Dominant cooling
    if (    (demands.powertrainCooling && !demands.batteryHeating)
        ||  (demands.cabinCooling && bestSink == SinkType::AMBIENT)
        ||  ((demands.cabinLHeating || demands.cabinRHeating) && bestSource == SourceType::BATTERY)
       )
        Valve::octoSetPos(OctoPos::POS2_SERIES);   // Condensor -> Radiator -> Evaporator -> Battery -> Powertrain
        // Condensor dumps heat to radiator. Evaporator cools battery and powertrain. Cabin can be cooled by dumping heat to radiator, cabin can also be heated from battery and powertrain.
        // i.e. this mode is for when battery&powertrain and/or cabin needs cooling

    // Dominant heating
    else if (((demands.cabinLHeating || demands.cabinRHeating || demands.batteryHeating) && bestSource == SourceType::AMBIENT)
            ||(demands.cabinCooling && bestSink == SinkType::BATTERY))
        Valve::octoSetPos(OctoPos::POS3_AMBIENT);  // {Condensor -> Battery -> Powertrain}  +  {Evaporator -> Radiator}
        // Evaporator takes heat from ambient. Condensor CAN heat battery and/or cabin. Powertrain passively cooled

    // Dominant heating
    else if ((demands.cabinLHeating || demands.cabinRHeating || demands.batteryHeating) && bestSource == SourceType::SELFHEAT)
        Valve::octoSetPos(OctoPos::POS4_RBYPASS);  // Condensor -> Evaporator -> Battery -> Powertrain
        // Only when too cold. i.e. when selectBestSource returns self heat as most efficient mode
        // Can heat cabin and battery, also takes in wasteheat from powertrain

    // Dominant cooling
    else if ((demands.cabinCooling || demands.batteryCooling) && !demands.batteryHeating && bestSink == SinkType::AMBIENT)
        Valve::octoSetPos(OctoPos::POS5_PARALLEL); // {Condensor -> Radiator -> Powertrain}  +  {Evaporator -> Battery}
        // Powertrain passive cooling only. Condensor CAN reject heat from cabin and evaporator CAN cool battery
        // i.e. this mode is only for cabin and/or battery active cooling, no powertrain active cooling


    // Dominant heating mode
    if (demands.cabinLHeating || demands.cabinRHeating || (demands.batteryHeating && !demands.cabinCooling))
    {
        Param::SetInt(Param::heat_transfer_mode, DOMINANT_HEATING);

        float setpoint = Param::GetInt(Param::temp_condensor_setp);
        float measured = Param::GetInt(Param::temp_outlet_compressor);
        uint8_t evap = runCapacityControl(setpoint, measured);
        uint8_t compressorDuty = (uint8_t)lastDutyCmd;

        uint8_t cabinL = demands.cabinLHeating ? 255 : 0;
        uint8_t cabinR = demands.cabinRHeating ? 255 : 0;
        uint8_t coolant = demands.batteryHeating ? 255 : 0;

        // All condensors starts with even flow, but if compressor can't keep up/reach setpoint we reduce heat to coolant condensor to prioritize cabin.
        adjustCondenserSplit(cabinL, cabinR, coolant, compressorDuty); // Flow sharing.
        
        // Evaporators are sources
        // Condensors valves are for flow dividing
        Valve::coolantCondensorClose(); // Coolant condensor low restriction valve. only use when no flow sharing is needed.
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, coolant);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, cabinR);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, cabinL);

        // Evaporator valves for heat absorption
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT,(bestSource != SourceType::SELFHEAT) ? evap : 0); // Absorb heat from source
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, 0); // Always 0. Cabin is not a source
        Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC, (bestSource == SourceType::SELFHEAT) ? evap : 0);
    }
    // Dominant cooling mode
    else if (demands.cabinCooling || demands.batteryCooling || demands.powertrainCooling)
    {
        Param::SetInt(Param::heat_transfer_mode, DOMINANT_COOLING);

        float setpoint = Param::GetInt(Param::temp_evaporator_setp);
        float measured = Param::GetInt(Param::temp_inlet_compressor);
        uint8_t evap = runCapacityControl(setpoint, measured);
        uint8_t compressorDuty = (uint8_t)lastDutyCmd;

        uint8_t cabin = demands.cabinCooling ? evap : 0;
        uint8_t coolant = (demands.batteryCooling || demands.powertrainCooling) ? evap : 0;

        adjustEvaporatorSplit(cabin, coolant, compressorDuty);

        // Condensors are sinks
        Valve::coolantCondensorOpen(); // Open the coolant condensor low restriction valve
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, 0); // for cooling any loop we reject through the coolant condensor
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, 0); // Cabin as sink means cabin heating which is handled in heating mode already.
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, 0);

        // Evaporator valves for temperature control
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT, coolant);
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, cabin);
        Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC, 0); // Always 0. Self-cooling is impossible:)    
    }
    // Idle
    else {
        Param::SetInt(Param::heat_transfer_mode, PASSIVE);
        // Set all valves open to not block compressor when starting up the next time.
        Valve::coolantCondensorOpen(); // open the coolant condensor low restriction valve
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, 255);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, 255);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC, 255);

        Compressor::SetDuty(0);
        last_error = 0.0f;
        lastDutyCmd = 0.0f;
        dutyTarget = 0.0f;
        lastValveCmd = 255.0f;
        capacityRunning = false;
    }
}