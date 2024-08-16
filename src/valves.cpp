#include "valves.h"
#include <libopencm3/cm3/systick.h>
#include <algorithm>

uint16_t pin_dir = 0;
uint16_t pin_en = 0;
uint16_t pin_step = 0;

uint32_t port_dir = 0;
uint32_t port_en = 0;
uint32_t port_step = 0;

int expv_pos = 0;
uint32_t expv_param_id = 0;

int steps_queue[6] = {0,0,0,0,0,0}; // used for non-blocking step taking
bool calibration_active = false; // calibration takes a while, so meanwhile must not set other steps 

int octo_currentpos = 0;
int oct0_pulses = 0;

bool pulse = true;

bool get_pins(int valve)
{
	switch (valve)
	{
		case EXP1:
            port_dir = GPIOA;   pin_dir = GPIO15;
			port_en = GPIOC;    pin_en = GPIO11;
			port_step = GPIOC;  pin_step = GPIO10;
			expv_param_id = Param::expv1_pos;
			expv_pos = Param::GetInt(Param::expv1_pos);
			break;
            
		case EXP2:
			port_dir = GPIOC;   pin_dir = GPIO12;
			port_en = GPIOD;    pin_en = GPIO1;
			port_step = GPIOD;  pin_step = GPIO0;
			expv_param_id = Param::expv2_pos;
			expv_pos = Param::GetInt(Param::expv2_pos);
			break;

		case EXP3:
			port_dir = GPIOD;   pin_dir = GPIO5;
			port_en = GPIOD;    pin_en = GPIO7;
			port_step = GPIOD;  pin_step = GPIO6;
			expv_param_id = Param::expv3_pos;
			expv_pos = Param::GetInt(Param::expv3_pos);
			break;

		case EXP4:
			port_dir = GPIOD;   pin_dir = GPIO2;
			port_en = GPIOD;    pin_en = GPIO4;
			port_step = GPIOD;  pin_step = GPIO3;
			expv_param_id = Param::expv4_pos;
			expv_pos = Param::GetInt(Param::expv4_pos);
			break;

		case EXP5:
			port_dir = GPIOE;   pin_dir = GPIO2;
			port_en = GPIOE;    pin_en = GPIO4;
			port_step = GPIOE;  pin_step = GPIO3;
			expv_param_id = Param::expv5_pos;
			expv_pos = Param::GetInt(Param::expv5_pos);
			break;

		case EXP6:
			port_dir = GPIOB;   pin_dir = GPIO3;
			port_en = GPIOB;    pin_en = GPIO5;
			port_step = GPIOB;  pin_step = GPIO4;
			expv_param_id = Param::expv6_pos;
			expv_pos = Param::GetInt(Param::expv6_pos);
			break;
		
		default :
			return false;
	}
	return true;
}

void Valve::expansionCheckSteps() // check all valves if any steps are queued
{
	for (int i = 0; i < 5; i++)
	{
		get_pins(i+1);

		if (steps_queue[i] > 0)
		{
			gpio_toggle(port_step, pin_step);
			
			if (gpio_get(port_step, pin_step))	steps_queue[i] -= 1; // count another step when high again
		}
		else // queue is finished for valve[i]
		{
			gpio_clear(port_step, pin_step);
			gpio_set(port_en, pin_en); 		// Disable step driver
			calibration_active = (calibration_active == true) ? false : calibration_active;
		}
	}
}


void Valve::expansionSetPos(uint8_t valve, uint8_t new_pos)
{
	if (!get_pins(valve) || calibration_active) return; // get all valve specific data (GPIO, params). also ignore setpoints during calibration

	uint8_t current_pos = Param::GetInt((Param::PARAM_NUM)expv_param_id);

	if (new_pos > current_pos) // opening valve further
	{
		gpio_clear(port_en, pin_en); 	// Enable step driver
		gpio_set(port_dir, pin_dir); 	// high is opening expansion valve

		steps_queue[valve-1] = new_pos - current_pos;
		Param::SetInt((Param::PARAM_NUM)expv_param_id, new_pos); // set position to 0 (closed)
		// disabling step driver is done by check function after queue is finished
	}

	if (new_pos < current_pos) // closing valve
	{
		gpio_clear(port_en, pin_en); 	// Enable step driver
		gpio_clear(port_dir, pin_dir); 	// low is closing expansion valve

		steps_queue[valve-1] = current_pos - new_pos;
		Param::SetInt((Param::PARAM_NUM)expv_param_id, new_pos); // set position to 0 (closed)
		// disabling step driver is done by check function after queue is finished
	}

}

void Valve::expansionCalibrateAll()
{
	calibration_active = true; // All setpoints given to expvalve_setpos() are ignored during calibration

	for (int i = 0; i < 5; i++)
	{
		get_pins(i+1);
		gpio_clear(port_en, pin_en); // Low is enable
		gpio_set(port_dir, pin_dir); // Low is closing valves
		steps_queue[i] = 255;
		Param::SetInt((Param::PARAM_NUM)expv_param_id, 0); // set position to 0 (closed)
	}
}



void Valve::solenoidOpen()
{
    DigIo::solenoid.Clear();
	Param::SetInt(Param::solenoid_pos, 0);
}

void Valve::solenoidClose()
{
    DigIo::solenoid.Set();
	Param::SetInt(Param::solenoid_pos, 1);
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
}

int Valve::octoGetPos()
{
	// Octovalve
//	if (timer_get_flag(TIM1, TIM_SR_CC1IF))
//	{
//	   float octo_pos = timer_get_ic_value(TIM1, TIM_IC1);
//	   Param::SetFloat(Param::octo_pos, octo_pos);
//	}
}
