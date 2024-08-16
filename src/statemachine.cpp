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


    //Analog settings
    knob_cabinl_heating = utils::change(Param::GetInt(Param::ana_in1), 0,4095, 0,255);
    knob_cabinr_heating = utils::change(Param::GetInt(Param::ana_in2), 0,4095, 0,255);
    knob_cabin_cooling = utils::change(Param::GetInt(Param::ana_in3), 0,4095, 0,255);
    knob_acpower = utils::change(Param::GetInt(Param::ana_in4), 0,4095, 0,255);

}


void StateMachine()
{
    getFlags();

/*  --------------------------------------------------  */
/*       Heating and Cooling controlled manually        */
/*  --------------------------------------------------  */
if (Param::GetInt(Param::control_mode) == ControlMode::Classic)
{   
    // Check if any flag is enabled
    if (flag_batt_needs_heating + 
        flag_batt_needs_cooling + 
        flag_cabinl_needs_heating + 
        flag_cabinr_needs_heating + 
        flag_cabin_needs_cooling + 
        flag_powertrain_needs_cooling > 0)
    {
        Compressor::SetDuty(100); // percentage duty
    }


/*------------------------------*/
/*      Octovalve settings      */
/*------------------------------*/
    //pos1 is the same as pos2

    if (flag_batt_needs_cooling && flag_powertrain_needs_cooling)
    {
        Valve::octoSetPos(OctoPositions::POS2);
    }

    if (flag_batt_needs_heating)
    {
        Valve::octoSetPos(OctoPositions::POS3);
    }

    if (flag_batt_needs_cooling && flag_powertrain_needs_cooling)
    {
        Valve::octoSetPos(OctoPositions::POS4);
    }

    if (flag_batt_needs_cooling && flag_cabinl_needs_heating || flag_cabinr_needs_heating)
    {
        Valve::octoSetPos(OctoPositions::POS5);
    }
    



/*------------------------------------*/
/*      Expansion Valve Settings      */
/*------------------------------------*/

    /*
    exp1:   Recirculation for quick heat
    exp2:   Heat LCC
    exp3:   Heat cabin right
    exp4:   Heat cabin left
    exp5:   Cool chiller
    exp6:   Cool cabin
    */

    if (flag_cabinr_needs_heating || flag_cabinl_needs_heating)
    {
        Valve::expansionSetPos(EXP3, knob_cabinr_heating);
        Valve::expansionSetPos(EXP4, knob_cabinl_heating);

        // still need to get the heat from somewhere..
        // open the chiller expansion valve. 
        // This will typically be taken from the battery. depending on octovalve position.
        Valve::expansionSetPos(EXP5, std::max(knob_cabinr_heating,knob_cabinl_heating));
    }
        
    if (flag_cabin_needs_cooling)
    {
        Valve::expansionSetPos(EXP6, knob_cabin_cooling);

        // still need to get rid of the heat absorbed from the cabin.
        // Open expansion valve to the LCC.
        // This will typically be rejected via radiator,
        // But could also be the battery depending on octovalve position.
        Valve::expansionSetPos(EXP2, knob_cabin_cooling);
    }
        

}




/*  ----------------------------------------------------------------  */
/*    Heating and Cooling controlled automatically with setpoints     */
/*  ----------------------------------------------------------------  */
if (Param::GetInt(Param::control_mode) == ControlMode::ClimateControl)
{
    
}

}