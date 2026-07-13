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

#include "test_control.h"
#include "valves.h"
#include "pumps.h"
#include "params.h"
#include "errormessage.h"

// Minimum expansion-valve opening (0..255) required before the compressor may run.
// Guards against the one destructive bench mistake: running the compressor into a
// closed refrigerant path (overpressure). Conservative on purpose.
static const int COMPRESSOR_OPEN_THRESHOLD = 40;

/*
 * Bench test loop. Called every 100ms from Ms100Task instead of thermalControl().
 * Each CAT_TEST parameter maps directly onto a driver call. Calibration is not
 * handled here; it is triggered from Param::Change (test_calibrate_* buttons).
 */
void testControl()
{
    // --- Expansion valves: direct position 0 (closed) .. 255 (open) ---
    Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT, Param::GetInt(Param::test_expv_evap_coolant));
    Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN,   Param::GetInt(Param::test_expv_evap_cabin));
    Valve::expansionSetPos(EXPV_EVAPORATOR_RECIRC,  Param::GetInt(Param::test_expv_evap_recirc));
    Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT,  Param::GetInt(Param::test_expv_cond_coolant));
    Valve::expansionSetPos(EXPV_CONDENSOR_CABINL,   Param::GetInt(Param::test_expv_cond_cabinl));
    Valve::expansionSetPos(EXPV_CONDENSOR_CABINR,   Param::GetInt(Param::test_expv_cond_cabinr));

    // --- Low-restriction coolant-condensor solenoid (0 = Open, 1 = Closed) ---
    if (Param::GetInt(Param::test_valve_condensor) == 0)
        Valve::coolantCondensorOpen();
    else
        Valve::coolantCondensorClose();

    // --- Octovalve: target position 1..5 (0 = stop) ---
    Valve::octoSetPos(Param::GetInt(Param::test_octo_pos));

    // --- Water pumps: direct duty %. Intentionally bypasses the reservoir run-dry
    //     guard used in thermalControl() - a unit on the bench is dry and we still
    //     want to command the pumps to verify the driver and tach feedback. ---
    Waterpump::batterySetDuty(Param::GetInt(Param::test_pump_battery));
    Waterpump::powertrainSetDuty(Param::GetInt(Param::test_pump_powertrain));

    // --- Compressor: gated behind an open refrigerant path (see threshold above) ---
    int requestedDuty = Param::GetInt(Param::test_compressor_duty);
    bool pathOpen = Param::GetInt(Param::test_expv_evap_coolant) >= COMPRESSOR_OPEN_THRESHOLD &&
                    Param::GetInt(Param::test_expv_cond_coolant) >= COMPRESSOR_OPEN_THRESHOLD;

    if (requestedDuty > 0 && !pathOpen)
    {
        Compressor::SetDuty(0);
    }
    else
    {
        Compressor::SetDuty(requestedDuty);
    }
}
