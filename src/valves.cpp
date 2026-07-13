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

#include "valves.h"
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rtc.h>
#include <algorithm>

// External volatile from main.cpp for direct ISR pulse counter access
extern volatile int octovalve_pulse_count;


int expv_pos = 0;
uint32_t expv_param_id = 0;

int steps_queue[6] = {0,0,0,0,0,0}; // used for non-blocking step taking
bool calibration_active = false; // calibration takes a while, so meanwhile must not set other steps 

int octo_target_position = 0;  // Target position (0=none, 1-5=position)
int octo_current_position = 0;  // Current position (1-5) computed from pulses
int octo_currentpos = 0;
int oct0_pulses = 0;

const int TOTAL_PULSES = 950;  // Measured: 948-1014 depending on direction and mechanical play
const int BAND_SIZE = TOTAL_PULSES / 4;  // 250 pulses between positions (4 gaps for 5 positions)
const int TOLERANCE = 10;
const int pos_centers[] = {0, 0, 240, 450, 670, 950};  // Index 0 unused; POS1=0, POS2=250, POS3=500, POS4=750, POS5=1000

volatile int current_pos = 0;  // From EXTI ISR
volatile enum OctoPos target_pos = POS1;
volatile bool in_transit = false;
volatile uint32_t command_start_ms = 0;  // For stall timeout
const uint32_t STALL_TIMEOUT_MS = 1000;


bool pulse = true;
bool Valve::valve_turning_direction = true;
bool Valve::octo_calibrating = false;

static uint32_t last_pulse_time = 0;
static int last_pulse_count = 0;

DigIo* Valve::pin_dir = nullptr;
DigIo* Valve::pin_en = nullptr;
DigIo* Valve::pin_step = nullptr;


bool Valve::getPins(int valve)
{
    switch (valve)
    {
        case EXPV_EVAPORATOR_COOLANT:
            pin_dir = &DigIo::exp1_dir;
            pin_en = &DigIo::exp1_en;
            pin_step = &DigIo::exp1_step;
            expv_param_id = Param::expv_evaporator_coolant;
            expv_pos = Param::GetInt(Param::expv_evaporator_coolant);
            break;
            
        case EXPV_EVAPORATOR_CABIN:
            pin_dir = &DigIo::exp2_dir;
            pin_en = &DigIo::exp2_en;
            pin_step = &DigIo::exp2_step;
            expv_param_id = Param::expv_evaporator_cabin;
            expv_pos = Param::GetInt(Param::expv_evaporator_cabin);
            break;

        case EXPV_EVAPORATOR_RECIRC:
            pin_dir = &DigIo::exp3_dir;
            pin_en = &DigIo::exp3_en;
            pin_step = &DigIo::exp3_step;
            expv_param_id = Param::expv_recirculation;
            expv_pos = Param::GetInt(Param::expv_recirculation);
            break;

        case EXPV_CONDENSOR_COOLANT:
            pin_dir = &DigIo::exp4_dir;
            pin_en = &DigIo::exp4_en;
            pin_step = &DigIo::exp4_step;
            expv_param_id = Param::expv_condensor_coolant;
            expv_pos = Param::GetInt(Param::expv_condensor_coolant);
            break;

        case EXPV_CONDENSOR_CABINL:
            pin_dir = &DigIo::exp5_dir;
            pin_en = &DigIo::exp5_en;
            pin_step = &DigIo::exp5_step;
            expv_param_id = Param::expv_condensor_cabinl;
            expv_pos = Param::GetInt(Param::expv_condensor_cabinl);
            break;

        case EXPV_CONDENSOR_CABINR:
            pin_dir = &DigIo::exp6_dir;
            pin_en = &DigIo::exp6_en;
            pin_step = &DigIo::exp6_step;
            expv_param_id = Param::expv_condensor_cabinr;
            expv_pos = Param::GetInt(Param::expv_condensor_cabinr);
            break;

        default:
            return false;
    }
    return true;
}

void Valve::expansionRunSteps() // check all valves if any steps are queued
{
    for (int valve = 0; valve < 6; valve++)
    {
        if(!getPins(valve+1)) return;

        if (steps_queue[valve] > 0)
        {
            pin_step->Toggle();
            if (pin_step->Get()) steps_queue[valve] -= 1; // count another step when high again
        }
        else // queue is finished for valve[valve]
        {
            pin_step->Clear();
            pin_en->Set();        // Disable step driver
            calibration_active = (calibration_active == true) ? false : calibration_active;
        }
    }
}


void Valve::expansionSetPos(uint8_t valve, uint8_t new_pos)
{
    if (!getPins(valve) || calibration_active || steps_queue[valve-1] != 0) return; // get all valve specific data (GPIO, params). also ignore setpoints during calibration

    uint8_t current_pos = Param::GetInt((Param::PARAM_NUM)expv_param_id);
    new_pos = utils::limitVal(new_pos, 0, 255);

    if (new_pos > current_pos) // opening valve further
    {
        pin_en->Clear();    // Enable step driver
        pin_dir->Set();     // high is opening expansion valve

        steps_queue[valve-1] = new_pos - current_pos;
        Param::SetInt((Param::PARAM_NUM)expv_param_id, new_pos); // set position to new_pos
        // disabling step driver is done by check function after queue is finished
    }

    if (new_pos < current_pos) // closing valve
    {
        pin_en->Clear();    // Enable step driver
        pin_dir->Clear();   // low is closing expansion valve

        steps_queue[valve-1] = current_pos - new_pos;
        Param::SetInt((Param::PARAM_NUM)expv_param_id, new_pos); // set position to new_pos
        // disabling step driver is done by check function after queue is finished
    }
}

void Valve::expansionCalibrateAll()
{
    calibration_active = true; // All setpoints given to expvalve_setpos() are ignored during calibration

    for (int i = 0; i < 6; i++)
    {
        if(!getPins(i+1)) return;
		steps_queue[i] = 0; // Clear any existing queue just in case
        pin_en->Clear();   // Low is enable
        pin_dir->Clear();    // Low is closing valve
        steps_queue[i] = 255;
        Param::SetInt((Param::PARAM_NUM)expv_param_id, 0); // set position to 0 (closed)
    }
}



void Valve::coolantCondensorOpen()
{
    DigIo::valve_lcc.Clear();
	Param::SetInt(Param::valve_coolant_condensor, 0);
}

void Valve::coolantCondensorClose()
{
    DigIo::valve_lcc.Set();
	Param::SetInt(Param::valve_coolant_condensor, 1);
}

int Valve::octoSetPos(int set_position)
{
	// Don't allow manual positioning during calibration
	if (octo_calibrating) return -1;

	// Validate position range (0-5, where 0=stop)
	if (set_position < 0 || set_position > 5) return -1;

	// Set target position (0 = stop motor)
	octo_target_position = set_position;
	Param::SetInt(Param::octovalve_setpoint, set_position);

	return 0;
}

void Valve::octoRunTask()
{
	const int POSITION_TOLERANCE = 10;  // Stop within +/- 10 pulses of target
	const uint32_t STALL_TIMEOUT = 100; // 200ms (assuming 10ms call rate, 20 * 10ms = 200ms)

	// Get current pulse count directly from ISR counter
	int current_pulses = octovalve_pulse_count;

	// Compute current position (1-5) from pulses
	int closest_pos = 1;
	int min_error = abs(current_pulses - pos_centers[1]);
	for (int pos = 2; pos <= 5; pos++)
	{
		int error = abs(current_pulses - pos_centers[pos]);
		if (error < min_error)
		{
			min_error = error;
			closest_pos = pos;
		}
	}
	octo_current_position = closest_pos;

	// Handle calibration mode
	if (octo_calibrating)
	{
		uint32_t current_time = rtc_get_counter_val();

		// Check if pulses changed
		if (current_pulses != last_pulse_count)
		{
			// Still moving, update tracking
			last_pulse_count = current_pulses;
			last_pulse_time = current_time;
		}
		else
		{
			// No movement detected, check if timeout reached
			uint32_t time_since_pulse = current_time - last_pulse_time;

			if (time_since_pulse >= STALL_TIMEOUT)
			{
				// Stalled against endstop - calibration complete!
				// Stop motor
				DigIo::octo_in1.Clear();
				DigIo::octo_in2.Clear();

				// Set this as position 1 (0 pulses at first endstop)
				octovalve_pulse_count = 0;

				// Clear calibration flag
				octo_calibrating = false;
			}
		}
		return;  // Skip normal positioning during calibration
	}

	// Normal positioning mode
	// If no target set, ensure motor is stopped
	if (octo_target_position == 0)
	{
		DigIo::octo_in1.Clear();
		DigIo::octo_in2.Clear();
		last_pulse_count = current_pulses;  // Reset tracking
		last_pulse_time = rtc_get_counter_val();
		return;
	}

	// Check if we need to move
	int target_pulses = pos_centers[octo_target_position];
	int position_error = target_pulses - current_pulses;

	if (abs(position_error) > POSITION_TOLERANCE)
	{
		// Not at target, check for stall (drift/endstop detection)
		uint32_t current_time = rtc_get_counter_val();

		if (current_pulses != last_pulse_count)
		{
			// Still moving normally, update tracking
			last_pulse_count = current_pulses;
			last_pulse_time = current_time;
		}
		else
		{
			// No movement detected, check if stalled
			uint32_t time_since_pulse = current_time - last_pulse_time;

			if (time_since_pulse >= STALL_TIMEOUT)
			{
				// Stalled! Hit endstop or obstacle
				// Stop motor
				DigIo::octo_in1.Clear();
				DigIo::octo_in2.Clear();

				// If targeting an endstop position (1 or 5), correct drift
				if (octo_target_position == 1)
				{
					// Hit position 1 endstop, correct to 0 pulses
					octovalve_pulse_count = pos_centers[0];
				}
				else if (octo_target_position == 5)
				{
					// Hit position 5 endstop, correct to 1000 pulses
					octovalve_pulse_count = pos_centers[5];
				}
				// For middle positions (2,3,4), accept current position

				// Clear target to stop trying
				octo_target_position = 0;
				return;
			}
		}

		// Keep moving
		if(position_error > 0) // Need to move clockwise
		{
			valve_turning_direction = CLOCKWISE;
			DigIo::octo_in1.Set();
			DigIo::octo_in2.Clear();
		}
		else // Need to move counterclockwise
		{
			valve_turning_direction = COUNTERCLOCKWISE;
			DigIo::octo_in1.Clear();
			DigIo::octo_in2.Set();
		}
	}
	else
	{
		// Position reached - stop motor
		DigIo::octo_in1.Clear();
		DigIo::octo_in2.Clear();
		last_pulse_count = current_pulses;  // Reset tracking
		last_pulse_time = rtc_get_counter_val();
	}
}

int Valve::octoGetPos()
{
	return octo_current_position;
}

void Valve::octoCalibrate()
{
	// Start calibration: move clockwise to position 1 endstop
	octo_calibrating = true;
	octo_target_position = 0;  // Clear any target
	valve_turning_direction = COUNTERCLOCKWISE;

	// Start motor moving clockwise
	DigIo::octo_in1.Clear();
	DigIo::octo_in2.Set();

	// Initialize tracking variables
	last_pulse_time = rtc_get_counter_val();
	last_pulse_count = octovalve_pulse_count;
}

