#include "valves.h"
#include <libopencm3/cm3/systick.h>
#include <algorithm>


int expv_pos = 0;
uint32_t expv_param_id = 0;

int steps_queue[6] = {0,0,0,0,0,0}; // used for non-blocking step taking
bool calibration_active = false; // calibration takes a while, so meanwhile must not set other steps 

int octo_currentpos = 0;
int oct0_pulses = 0;

bool pulse = true;

DigIo* Valve::pin_dir = nullptr;
DigIo* Valve::pin_en = nullptr;
DigIo* Valve::pin_step = nullptr;

bool Valve::getPins(int valve)
{
    switch (valve)
    {
        case EXPV_RECIRCULATION:
            pin_dir = &DigIo::exp1_dir;
            pin_en = &DigIo::exp1_en;
            pin_step = &DigIo::exp1_step;
            expv_param_id = Param::expv_recirculation;
            expv_pos = Param::GetInt(Param::expv_recirculation);
            break;
            
        case EXPV_CONDENSOR_COOLANT:
            pin_dir = &DigIo::exp2_dir;
            pin_en = &DigIo::exp2_en;
            pin_step = &DigIo::exp2_step;
            expv_param_id = Param::expv_condensor_coolant;
            expv_pos = Param::GetInt(Param::expv_condensor_coolant);
            break;

        case EXPV_CONDENSOR_CABINR:
            pin_dir = &DigIo::exp3_dir;
            pin_en = &DigIo::exp3_en;
            pin_step = &DigIo::exp3_step;
            expv_param_id = Param::expv_condensor_cabinr;
            expv_pos = Param::GetInt(Param::expv_condensor_cabinr);
            break;

        case EXPV_CONDENSOR_CABINL:
            pin_dir = &DigIo::exp4_dir;
            pin_en = &DigIo::exp4_en;
            pin_step = &DigIo::exp4_step;
            expv_param_id = Param::expv_condensor_cabinl;
            expv_pos = Param::GetInt(Param::expv_condensor_cabinl);
            break;

        case EXPV_EVAPORATOR_COOLANT:
            pin_dir = &DigIo::exp5_dir;
            pin_en = &DigIo::exp5_en;
            pin_step = &DigIo::exp5_step;
            expv_param_id = Param::expv_evaporator_coolant;
            expv_pos = Param::GetInt(Param::expv_evaporator_coolant);
            break;

        case EXPV_EVAPORATOR_CABIN:
            pin_dir = &DigIo::exp6_dir;
            pin_en = &DigIo::exp6_en;
            pin_step = &DigIo::exp6_step;
            expv_param_id = Param::expv_evaporator_cabin;
            expv_pos = Param::GetInt(Param::expv_evaporator_cabin);
            break;
        
        default:
            return false;
    }
    return true;
}

void Valve::expansionRunSteps() // check all valves if any steps are queued
{
    for (int valve = 0; valve < 5; valve++)
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

    for (int i = 0; i < 5; i++)
    {
        if(!getPins(i+1)) return;
		steps_queue[i] = 0; // Clear any existing queue just in case
        pin_en->Clear();   // Low is enable
        pin_dir->Clear();    // Low is closing valve
        steps_queue[i] = 255;
        Param::SetInt((Param::PARAM_NUM)expv_param_id, 0); // set position to 0 (closed)
    }
}



void Valve::solenoidOpen()
{
    DigIo::valve_lcc.Clear();
	Param::SetInt(Param::valve_lcc, 0);
}

void Valve::solenoidClose()
{
    DigIo::valve_lcc.Set();
	Param::SetInt(Param::valve_lcc, 1);
}

int Valve::octoSetPos(int set_position)
{
	octo_currentpos = Param::GetInt(Param::octo_pos);

	if (octo_currentpos != set_position)
	{
		if(set_position > octo_currentpos) // Move clockwise
		{
			DigIo::octo_in1.Set();
			DigIo::octo_in2.Clear();
		}
		if (set_position < octo_currentpos) // Move counterclockwise
		{
			DigIo::octo_in1.Clear();
			DigIo::octo_in2.Set();
		}
	}
	return 0;
}

int Valve::octoGetPos()
{
	// Octovalve
//	if (timer_get_flag(TIM1, TIM_SR_CC1IF))
//	{
//	   float octo_pos = timer_get_ic_value(TIM1, TIM_IC1);
//	   Param::SetFloat(Param::octo_pos, octo_pos);
//	}
	return 0;
}
