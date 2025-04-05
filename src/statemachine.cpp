#include "statemachine.h"
#include <algorithm>


// These flags are set by the state machine and processed by the handler
bool flag_batt_needs_heating = false;
bool flag_batt_needs_cooling = false;
bool flag_cabinl_needs_heating = false;
bool flag_cabinr_needs_heating = false;
bool flag_cabin_needs_cooling = false;
bool flag_powertrain_needs_cooling = false;

uint16_t knob_cabinl_heating = 0;
uint16_t knob_cabinr_heating = 0;
uint16_t knob_cabin_cooling = 0;
uint16_t knob_acpower = 0;

float HIGH_PRESSURE_LIMIT = 30; // bar
float LOW_PRESSURE_LIMIT = 0.5; // bar

// PI Control Variables
float Kp = 0.5;  // Proportional Gain
float Ki = 0.1;  // Integral Gain
float integral_compressor = 0;
float integral_expansion = 0;
float dt = 0.1;  // Time step
float accumulated_error = 0;

void getFlags()
{
    //Flags
    if(Param::GetInt(Param::heat_cabinl))
        flag_cabinl_needs_heating = true;
    else 
        flag_cabinl_needs_heating = false;
    
    if(Param::GetInt(Param::heat_cabinr))
        flag_cabinr_needs_heating = true;
    else 
        flag_cabinr_needs_heating = false;

    if(Param::GetInt(Param::cool_cabin))
        flag_cabin_needs_cooling = true;
    else 
        flag_cabin_needs_cooling = false;

    if(Param::GetInt(Param::heat_cabinl))
        flag_cabinl_needs_heating = true;
    else 
        flag_cabinl_needs_heating = false;

}

void controlLoop()
{
    float setpoint, measured_temp, error, control_output;
    bool isHeating = false, isCooling = false;
    
    if (flag_cabin_needs_cooling)
    {
        isCooling = true;
        setpoint = Param::GetInt(Param::temp_evaporator_setp);
        measured_temp = Param::GetInt(Param::temp_pre_evaporator);
    }
    else if (flag_cabinl_needs_heating || flag_cabinr_needs_heating)
    {
        isHeating = true;
        setpoint = Param::GetInt(Param::temp_condensor_setp);
        measured_temp = Param::GetInt(Param::temp_outlet_compressor);
    }
    else
    {
        Compressor::SetDuty(0);
        return;
    }

    // Safety checks for pressure limits
    float high_side_pressure = Param::GetInt(Param::pressure_outlet_compressor);
    float low_side_pressure = Param::GetInt(Param::pressure_pre_evaporator);
    
    if (high_side_pressure > HIGH_PRESSURE_LIMIT)
    {
        Compressor::SetDuty(Compressor::GetDuty() * 0.8); // Reduce duty cycle to prevent overpressure
    }
    else if (low_side_pressure < LOW_PRESSURE_LIMIT)
    {
        Compressor::SetDuty(Compressor::GetDuty() * 0.8); // Reduce duty cycle
    }
    else
    {
        error = setpoint - measured_temp;
        control_output = Kp * error + Ki * accumulated_error;
        accumulated_error += error;
        control_output = utils::limitVal(control_output, 0, 100);
        
        Compressor::SetDuty(control_output);
    }
    
    if (isCooling)
    {
        Valve::expansionSetPos(EXPV_EVAPORATOR_CABIN, control_output); //reject the heat to coolant / cool cabin
        Valve::expansionSetPos(EXPV_CONDENSOR_COOLANT, 255); // just fully open the valve to condensor without restriction
    }
    else if (isHeating)
    {
        Valve::expansionSetPos(EXPV_EVAPORATOR_COOLANT, control_output); //reject the heat to cabin
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINR, 128); // just fully open the valve to condensor without restriction
        Valve::expansionSetPos(EXPV_CONDENSOR_CABINL, 128); // just fully open the valve to condensor without restriction
    }
}


void StateMachine()
{
    getFlags();
 
    // Check if any flag is enabled
    if (flag_batt_needs_heating + 
        flag_batt_needs_cooling + 
        flag_cabinl_needs_heating + 
        flag_cabinr_needs_heating + 
        flag_cabin_needs_cooling + 
        flag_powertrain_needs_cooling > 0)
    {
        //
    }


/*------------------------------*/
/*      Octovalve settings      */
/*------------------------------*/

    // POS1             Same as POS2
	// POS2_SERIES      Battery and powertrain in series, Cool battery and powertrain, Reject heat to radiator or cabin
	// POS3_AMBIENT     Reject heat to battery or cabin, Get heat from radiator
	// POS4_RBYPASS     Battery and powertrain in series, Cool battery and powertrain, Exclude radiator, Reject heat to cabin
	// POS5_PARALLEL    Battery and powertrain separate, Cool battery with a/c, Powertrain heat to radiator, Reject heat from battery or cabin to radiator

    if (flag_batt_needs_cooling && flag_powertrain_needs_cooling)
    {
        Valve::octoSetPos(OctoPositions::POS2_SERIES);
    }

    if (flag_batt_needs_heating)
    {
        Valve::octoSetPos(OctoPositions::POS3_AMBIENT);
    }

    if (flag_batt_needs_cooling && flag_powertrain_needs_cooling)
    {
        Valve::octoSetPos(OctoPositions::POS4_RBYPASS);
    }

    if (flag_batt_needs_cooling && flag_cabinl_needs_heating || flag_cabinr_needs_heating)
    {
        Valve::octoSetPos(OctoPositions::POS5_PARALLEL);
    }
    
    controlLoop();

}


