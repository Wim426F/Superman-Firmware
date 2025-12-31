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
#include <cstdint>
#include <limits>
#include <algorithm>

// Constants
const float BATTERY_HEAT_THRESHOLD = 5.0f;  // °C
const float BATTERY_COOL_THRESHOLD = 40.0f;  // °C
const float POWERTRAIN_COOL_THRESHOLD = 50.0f;  // °C
const float HIGH_PRESSURE_LIMIT = 30.0f;  // Bar
const float LOW_PRESSURE_LIMIT = 1.0f;    // Bar
const float MIN_VALVE_POSITION = 5.0f; // %
const float RECIRC_TEMP_THRESHOLD = -20.0f;  // °C for recirculation mode

// PI control constants
const float Kp = 1.0f;
const float Ki = 0.1f;
static float accumulated_error = 0.0f;
float dt = 0.1f; // Loop time 100ms
float last_error = 0;

// Source and sink types
// Cabin is never a source or sink, as control is from its perspective. It can only need a source or sink.
// Powertrain would be nice as source but octovalve cannot but evaporator in front of powertrain. only evaporator->battery->powertrain.
enum class SourceType {AMBIENT, BATTERY, RECIRCULATION};
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
    ThermalDemands demands;

    // Cabin demands from digital inputs
    demands.cabinLHeating = Param::GetBool(Param::heat_cabinl);
    demands.cabinRHeating = Param::GetBool(Param::heat_cabinr);
    demands.cabinCooling = Param::GetBool(Param::cool_cabin);

    // Battery and powertrain demands are automatic
    float batteryTemp = Param::GetInt(Param::temp_battery);
    demands.batteryHeating = batteryTemp < Param::GetInt(Param::temp_battery_min);
    demands.batteryCooling = batteryTemp > Param::GetInt(Param::temp_battery_max);

    float powertrainTemp = Param::GetInt(Param::temp_powertrain);
    demands.powertrainCooling = powertrainTemp > Param::GetInt(Param::temp_powertrain_max);

    // If radiator coolant returns much cooler coolant than ambient it might we choking (freezing itself in)
    //demands.radiatorDefrost = (heating && Param::GetInt(Param::temp_radiator) < Param::GetInt(Param::temp_ambient) -5 && Compressor::GetDuty() > 80);

    return demands;
}


float estimateCOP(float W_elec_W, uint16_t rpm, float T_in_C, float P_in_bar, float T_out_C, float P_out_bar) {
    // COP Estimation Function
    // First-principles approx: COP = Q_useful / W_comp
    // Q_useful ≈ ṁ_ref * Δh (latent + sensible across cycle)
    // ṁ_ref = (RPM/60 * vol_rot * eta_vol) * ρ_vapor_suction
    // ρ_vapor ≈ P_in * M / (R * T_in)  [ideal gas, good for low-P suction]
    // Δh ≈ h_fg (R1234yf ~170 kJ/kg avg) + c_p_vap * (T_out - T_in)  [rough superheat/sensible]
    // W_comp = W_elec / eta_motor  [~0.9]

    // Constants for R1234yf (from NIST EOS approx)
    const float M = 114.04f / 1000.0f;  // kg/mol
    const float R = 8.314f / M;         // J/kg-K (specific gas const)
    const float h_fg_avg = 170.0f;      // kJ/kg latent (evap/cond range)
    const float c_p_vap = 0.95f;        // kJ/kg-K vapor avg
    const float eta_vol = 0.85f;        // Vol eff (paramize if needed)
    const float eta_motor = 0.90f;      // Motor eff

    // Suction vapor density (ideal gas, bar→Pa, C→K)
    float T_in_K = T_in_C + 273.15f;
    float P_in_Pa = P_in_bar * 1e5f;
    float rho_vap = (P_in_Pa / (R * T_in_K));  // kg/m³

    // Mass flow ṁ (kg/s)
    float vol_rot_m3 = 45e-6f;  // 45 cc/rev = 45e-6 m³/rev
    float m_dot = (rpm / 60.0f * vol_rot_m3 * eta_vol) * rho_vap;

    // Approx Δh across cycle (latent dominant + sensible discharge)
    float delta_h = h_fg_avg + c_p_vap * (T_out_C - T_in_C);  // kJ/kg

    // Q_useful (kW; same approx for heat/cool)
    float Q_useful = m_dot * delta_h / 1000.0f;  // kW (m_dot in kg/s, delta_h kJ/kg)

    // W_comp (kW)
    float W_comp = W_elec_W / 1000.0f / eta_motor;

    // COP (guard div0)
    float cop = (W_comp > 0.1f) ? Q_useful / W_comp : 0.0f;
    return cop;
}


// Make sure cabin always gets as much flow in dual mode heating as in single mode by reducing the coolant condensor valve
static void adjustCondenserSplit(uint8_t& cabinL, uint8_t& cabinR, uint8_t& coolant, uint8_t compressorDuty) {
    // Reduce coolant condensor valve if cabin needs priority
    if (compressorDuty > 90 && (Param::GetInt(Param::temp_condensor_setp) - Param::GetInt(Param::temp_outlet_compressor) > 2.0f)) {
        coolant = std::max(static_cast<uint8_t>(coolant - 5), static_cast<uint8_t>(128)); // Reduce by 10%, min 50%
    } else if (compressorDuty < 80 && (Param::GetInt(Param::temp_battery_min) - Param::GetInt(Param::temp_battery) > 2.0f)) {
        coolant = std::min(static_cast<uint8_t>(coolant + 5), static_cast<uint8_t>(255)); // Increase by 10%, max 100%
    }
}

// Prioritize cabin cooling: reduce coolant evap opening if overloaded
static void adjustEvaporatorSplit(uint8_t& cabin, uint8_t& coolant, uint8_t compressorDuty) {
    if (compressorDuty > 90 && (Param::GetInt(Param::temp_inlet_compressor) - Param::GetInt(Param::temp_evaporator_setp) > 2.0f)) {
        coolant = std::max(static_cast<uint8_t>(coolant - 5), static_cast<uint8_t>(0)); // Close coolant more, divert flow to cabin, -10%
    } else if (compressorDuty < 80 && (Param::GetInt(Param::temp_battery) - Param::GetInt(Param::temp_battery_max) > 2.0f)) {
        coolant = std::min(static_cast<uint8_t>(coolant + 5), static_cast<uint8_t>(255)); // Open coolant more if battery needs it
    }
}

// Gather available heat sources and sinks
void getAvailableSourcesSinks(const ThermalDemands& demands, SourceData sources[3], SinkData sinks[2]) {
    float ambientTemp = Param::GetInt(Param::temp_ambient);
    float batteryTemp = Param::GetInt(Param::temp_battery);
    float recircTemp = Param::GetInt(Param::temp_outlet_compressor); //FIXME not correct!

    // Sources – always available when physically present
    sources[1] = {SourceType::BATTERY, {!demands.batteryHeating, batteryTemp}}; // Can't source heat from battery if it needs heating
    sources[2] = {SourceType::AMBIENT, {true, ambientTemp}}; // Always a heat source
    sources[3] = {SourceType::RECIRCULATION, {true, recircTemp}}; // Always a heat source (COP=1, compressor energy to heat)

    // Sinks – availability based on demand (user wants to use it as heat sink)
    sinks[0] = {SinkType::BATTERY, {!demands.batteryCooling, batteryTemp}}; // Battery is heatsink unless it needs cooling
    sinks[1] = {SinkType::AMBIENT, {true, ambientTemp}}; // Always a heatsink
}

// Select hottest source for best heat transfer
SourceType selectBestSource(float targetTemp, const SourceData sources[3]) {
    SourceType bestSource = SourceType::AMBIENT;
    float minDeltaT = std::numeric_limits<float>::max(); // Start with highest float to ensure first delta updates min

    // Check each source for smallest temp difference (hottest source)
    for (int i = 0; i < 2; i++) {
        if (sources[i].info.available) {
            float deltaT = targetTemp - sources[i].info.temp; // Calc delta to rank sources
            if (deltaT < minDeltaT) {
                minDeltaT = deltaT;
                bestSource = sources[i].type; // Update if hotter source found
            }
        }
    }

    // Use recirculation if ambient too cold and other sources insufficient
    if (Param::GetInt(Param::temp_ambient) < RECIRC_TEMP_THRESHOLD)
        bestSource = SourceType::RECIRCULATION;

    return bestSource;
}

// Select coldest sink for best heat rejection
SinkType selectBestSink(float targetTemp, const SinkData sinks[2]) {
    SinkType bestSink = SinkType::AMBIENT;
    float maxDeltaT = -std::numeric_limits<float>::max(); // Start with lowest float to ensure first delta updates max

    // Check each sink for largest temp difference (best heat rejection)
    for (int i = 0; i < 2; i++) {
        if (sinks[i].info.available) {
            float deltaT = targetTemp - sinks[i].info.temp; // Calc delta to rank sinks
            if (deltaT > maxDeltaT) {
                maxDeltaT = deltaT;
                bestSink = sinks[i].type; // Update if better sink found
            }
        }
    }
    return bestSink;
}


uint8_t runPiControl(float setpoint, float measured) {
    float highPressure = Param::GetInt(Param::pressure_outlet_compressor);
    float lowPressure = Param::GetInt(Param::pressure_pre_evaporator);
    uint8_t currentDuty = Compressor::GetDuty();

    // Pressure overrides (incremental)
    if (highPressure > HIGH_PRESSURE_LIMIT) {
        int newDuty = std::max(0, static_cast<int>(currentDuty - 1)); // Unload compressor
        Compressor::SetDuty(static_cast<uint8_t>(newDuty));
        return static_cast<uint8_t>(newDuty * 2.55f); // Scale for valve
    }
    if (lowPressure < LOW_PRESSURE_LIMIT) {
        int newDuty = std::max(0, static_cast<int>(currentDuty - 1)); // Unload compressor
        Compressor::SetDuty(static_cast<uint8_t>(newDuty));
        return static_cast<uint8_t>(newDuty * 2.55f);
    }

    // PI with anti-windup
    float error = setpoint - measured;
    accumulated_error += error * dt;
    if (error * last_error < 0.0f) accumulated_error *= 0.5f; // Decay on reversal
    last_error = error;

    float controlOutput = Kp * error + Ki * accumulated_error;
    controlOutput = std::max(0.0f, std::min(100.0f, controlOutput));

    // Anti-windup
    if (controlOutput >= 100.0f) accumulated_error = std::min(accumulated_error, (100.0f - Kp * error) / Ki);
    if (controlOutput <= 0.0f) accumulated_error = std::max(accumulated_error, (-Kp * error) / Ki);

    Compressor::SetDuty(static_cast<uint8_t>(controlOutput));

    uint8_t valveOutput = static_cast<uint8_t>(controlOutput * 2.55f);
    valveOutput = utils::limitVal(valveOutput, MIN_VALVE_POSITION, 255);
    return valveOutput;
}

void thermalControl() {
    ThermalDemands demands = assessDemands();
    SourceData sources[3];
    SinkData sinks[2];
    getAvailableSourcesSinks(demands, sources, sinks);

    // Dynamic setpoint for condensor/evaporator to 
    float ambient = Param::GetInt(Param::temp_ambient);
    int32_t ambient_low = RECIRC_TEMP_THRESHOLD;  // e.g., -20
    int32_t ambient_high = 10; // e.g., 10
    int32_t condensor_min = 50;          // e.g., 50
    int32_t condensor_max = 70;          // e.g., 70

    // below -20C ambient the condensor setpoint is 50, above 10C the setpoint is 70CC. between ambient -20C and 10C linear interpolation
    // this is an attempt to improve COP in some conditions
    int32_t condensor_set_int = utils::change(ambient, ambient_low, ambient_high, condensor_min, condensor_max);
    int32_t condensor_setpoint = std::max(condensor_min, std::min(condensor_max, condensor_set_int)); // Clamp

    int32_t evap_min = 5;
    int32_t evap_max = 10;
    int32_t ambient_low_cool = 30;
    int32_t ambient_high_cool = 50;

    // below 30C ambient the evaporator setpoint is 5C, above 50C the setpoint is 10C. between ambient 30C and 50C linear interpolation
    // this is an attempt to improve COP in some conditions
    int32_t evap_set_int = utils::change(ambient, ambient_low_cool, ambient_high_cool, evap_min, evap_max);
    int32_t evap_setpoint = std::max(evap_min, std::min(evap_max, evap_set_int)); // Clamp

    SourceType bestSource = selectBestSource(condensor_setpoint, sources);
    SinkType bestSink = selectBestSink(evap_setpoint, sinks);

    
    // Tie waterpump speeds to compressor with minimum and maximum of 30-80% waterpump duty.
    int pumpDuty = utils::limitVal(Param::GetInt(Param::compressor_duty_request), 30, 80);
    Waterpump::powertrainSetDuty(static_cast<uint8_t>(pumpDuty));
    Waterpump::batterySetDuty(static_cast<uint8_t>(pumpDuty));
    

    /* TODO here, radiatorfan control */
    

    // SourceType:  AMBIENT, BATTERY, RECIRCULATION
    // SinkType:    AMBIENT, BATTERY

    // OctoPos::POS2_SERIES     Condensor -> Radiator -> Evaporator -> Battery -> Powertrain
    // OctoPos::POS3_AMBIENT    {Condensor -> Battery -> Powertrain}  +  {Evaporator -> Radiator}
    // OctoPos::POS4_RBYPASS    Condensor -> Evaporator -> Battery -> Powertrain
    // OctoPos::POS5_PARALLEL   {Condensor -> Radiator -> Powertrain}  +  {Evaporator -> Battery}


    // Dominant cooling
    if ((demands.batteryCooling && demands.powertrainCooling) || (demands.cabinCooling && bestSink == SinkType::AMBIENT) || (demands.cabinLHeating || demands.cabinRHeating && bestSource == SourceType::BATTERY))
        Valve::octoSetPos(OctoPos::POS2_SERIES);   // Condensor -> Radiator -> Evaporator -> Battery -> Powertrain
        // Condensor dumps heat to radiator. Evaporator cools battery and powertrain. Cabin can be cooled by dumping heat to radiator, cabin can also be heated from battery and powertrain.
        // i.e. this mode is for when battery&powertrain and/or cabin needs cooling

    // Dominant heating
    if ((demands.cabinLHeating || demands.cabinRHeating || demands.batteryHeating) && bestSource == SourceType::AMBIENT)
        Valve::octoSetPos(OctoPos::POS3_AMBIENT);  // {Condensor -> Battery -> Powertrain}  +  {Evaporator -> Radiator}
        // Evaporator takes heat from ambient. Condensor CAN heat battery and/or cabin. Powertrain passively cooled

    // Dominant heating
    if ((demands.cabinLHeating || demands.cabinRHeating || demands.batteryHeating) && bestSource == SourceType::RECIRCULATION) 
        Valve::octoSetPos(OctoPos::POS4_RBYPASS);  // Condensor -> Evaporator -> Battery -> Powertrain
        // Only when too cold. i.e. when selectBestSource returns self heat as most efficient mode
        // Can heat cabin and battery, also takes in wasteheat from powertrain

    // Dominant cooling
    if (demands.cabinCooling || demands.batteryCooling && !demands.batteryHeating && bestSink == SinkType::AMBIENT)
        Valve::octoSetPos(OctoPos::POS5_PARALLEL); // {Condensor -> Radiator -> Powertrain}  +  {Evaporator -> Battery}
        // Powertrain passive cooling only. Condensor CAN reject heat from cabin and evaporator CAN cool battery
        // i.e. this mode is only for cabin and/or battery active cooling, no powertrain active cooling


    // Dominant heating mode
    if (demands.cabinLHeating || demands.cabinRHeating || demands.batteryHeating && !demands.cabinCooling)
    {
        float setpoint = Param::GetInt(Param::temp_condensor_setp);
        float measured = Param::GetInt(Param::temp_outlet_compressor);
        uint8_t controlOutput = runPiControl(setpoint, measured);
        uint8_t compressorDuty = Compressor::GetDuty();

        uint8_t cabinL = demands.cabinLHeating ? 255 : 0;
        uint8_t cabinR = demands.cabinRHeating ? 255 : 0;
        uint8_t coolant = demands.batteryHeating ? 255 : 0;

        // All condensors starts with even flow, but if compressor can't keep up/reach setpoint we reduce heat to coolant condensor to prioritize cabin.
        adjustCondenserSplit(cabinL, cabinR, coolant, compressorDuty); // Flow sharing.
        
        // Evaporators are sources
        // Condensors valves are for flow dividing
        Valve::solenoidClose(); // Coolant condensor low restriction valve. only use when no flow sharing is needed.
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, coolant);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, cabinR);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, cabinL);

        // Evaporator valves for heat absorption
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT,(bestSource != SourceType::RECIRCULATION) ? controlOutput : 0); // Absorb heat from source
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, 0); // Always 0. Cabin is not a source
        Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC, (bestSource == SourceType::RECIRCULATION) ? controlOutput : 0);
    }
    // Dominant cooling mode
    else if (demands.cabinCooling || demands.batteryCooling || demands.powertrainCooling)
    {
        float setpoint = Param::GetInt(Param::temp_evaporator_setp);
        float measured = Param::GetInt(Param::temp_inlet_compressor);
        uint8_t controlOutput = runPiControl(setpoint, measured);
        uint8_t compressorDuty = Compressor::GetDuty();

        uint8_t cabin = demands.cabinCooling ? controlOutput : 0;
        uint8_t coolant = (demands.batteryCooling || demands.powertrainCooling) ? controlOutput : 0;

        adjustEvaporatorSplit(cabin, coolant, compressorDuty);

        // Condensors are sinks
        Valve::solenoidOpen(); // Open the coolant condensor low restriction valve
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
        // Set all valves open to not block compressor when starting up the next time.
        Valve::solenoidOpen(); // Release/open the coolant condensor low restriction valve
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, 255);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, 255);
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, 255);
        Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC, 255);

        Compressor::SetDuty(0); // Disable compressor when idle
    }
}