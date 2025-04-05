/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */

 //Define a version string of your firmware here
#define VER 0.01.R
#define name superman

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 16
//Next value Id: 2053
/*             category     name                   unit          min     max      default  id */
#define PARAM_LIST \
   PARAM_ENTRY(CAT_COMM,    canspeed,              CANSPEEDS,    0,      4,       2,       0   ) \
   PARAM_ENTRY(CAT_COMM,    canperiod,             CANPERIODS,   0,      1,       0,       1   ) \
   PARAM_ENTRY(CAT_COMM,    nodeid,                "",           1,      63,      50,      2   ) \
   PARAM_ENTRY(CAT_CTRL,    control_mode,          CTRL_MODE,    0,      3,       0,       3   ) \
   PARAM_ENTRY(CAT_CTRL,    canio,                 CANIO,        0,      1,       0,       4   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_cabinr_src,       HWIO_SRC,     0,      1,       0,       5   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_cabinl_src,       HWIO_SRC,     0,      1,       1,       6   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_battery_src,      HWIO_SRC,     0,      1,       2,       7   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_powertrain_src,   HWIO_SRC,     0,      1,       3,       8   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_cabinl_setp,      "°C",        -30,     100,     20,      9   ) \
   PARAM_ENTRY(CAT_CTRL,    temp_cabinr_setp,      "°C",        -30,     100,     20,      10  ) \
   PARAM_ENTRY(CAT_CTRL,    temp_battery_setp,     "°C",        -30,     100,     20,      11  ) \
   PARAM_ENTRY(CAT_CTRL,    temp_evaporator_setp,  "°C",        -30,     100,     0,       12  ) \
   PARAM_ENTRY(CAT_CTRL,    temp_condensor_setp,   "°C",        -30,     100,     50,      13  ) \
   PARAM_ENTRY(CAT_CTRL,    compressor_plim,       "W",          0,      10000,   5000,    14  ) \
   PARAM_ENTRY(CAT_CTRL,    ext_ntc,               NTC,          0,      6,       5,       15  ) \
   VALUE_ENTRY(octo_pos,                    OCTOPOS,   2005)  \
   VALUE_ENTRY(expv_recirculation ,         "dig",     2006 ) \
   VALUE_ENTRY(expv_condensor_coolant,      "dig",     2007 ) \
   VALUE_ENTRY(expv_condensor_cabinr ,      "dig",     2008 ) \
   VALUE_ENTRY(expv_condensor_cabinl ,      "dig",     2009 ) \
   VALUE_ENTRY(expv_evaporator_coolant,     "dig",     2010 ) \
   VALUE_ENTRY(expv_evaporator_cabin ,      "dig",     2011 ) \
   VALUE_ENTRY(pressure_inlet_compressor,   "bar",     2012 ) \
   VALUE_ENTRY(pressure_outlet_compressor,  "bar",     2013 ) \
   VALUE_ENTRY(pressure_pre_evaporator,     "bar",     2014 ) \
   VALUE_ENTRY(temp_inlet_compressor,       "°C",      2015 ) \
   VALUE_ENTRY(temp_outlet_compressor,      "°C",      2016 ) \
   VALUE_ENTRY(temp_pre_evaporator,         "°C",      2017 ) \
   VALUE_ENTRY(temp_inlet_battery,          "°C",      2018 ) \
   VALUE_ENTRY(temp_inlet_powertrain,       "°C",      2019 ) \
   VALUE_ENTRY(temp_reservoir,              "°C",      2020 ) \
   VALUE_ENTRY(temp_ambient,                "°C",      2021 ) \
   VALUE_ENTRY(temp_radiator,               "°C",      2022 ) \
   VALUE_ENTRY(temp_cabinl,                 "°C",      2023 ) \
   VALUE_ENTRY(temp_cabinr,                 "°C",      2024 ) \
   VALUE_ENTRY(temp_battery,                "°C",      2025 ) \
   VALUE_ENTRY(temp_powertrain,             "°C",      2026 ) \
   VALUE_ENTRY(knob_cabinl,                 "dig",     2027 ) \
   VALUE_ENTRY(knob_cabinr,                 "dig",     2028 ) \
   VALUE_ENTRY(valve_lcc,                   "dig",     2029 ) \
   VALUE_ENTRY(heat_cabinl,                 "dig",     2030 ) \
   VALUE_ENTRY(heat_cabinr,                 "dig",     2031 ) \
   VALUE_ENTRY(cool_cabin,                  "dig",     2032 ) \
   VALUE_ENTRY(heat_battery,                "dig",     2033 ) \
   VALUE_ENTRY(cool_battery,                "dig",     2034 ) \
   VALUE_ENTRY(enable_pumps,                "dig",     2035 ) \
   VALUE_ENTRY(pump_battery_flow,           "lpm",     2036 ) \
   VALUE_ENTRY(pump_battery_duty,           "%",       2037 ) \
   VALUE_ENTRY(pump_powertrain_flow,        "lpm",     2038 ) \
   VALUE_ENTRY(pump_powertrain_duty,        "%",       2039 ) \
   VALUE_ENTRY(compressor_enable,           "dig",     2040 ) \
   VALUE_ENTRY(compressor_speed,            "rpm",     2041 ) \
   VALUE_ENTRY(compressor_duty_request,     "%",       2042 ) \
   VALUE_ENTRY(compressor_duty,             "%",       2043 ) \
   VALUE_ENTRY(compressor_temp,             "°C",      2044 ) \
   VALUE_ENTRY(compressor_power,            "W",       2045 ) \
   VALUE_ENTRY(compressor_HV,               "V",       2046 ) \
   VALUE_ENTRY(compressor_LV,               "V",       2047 ) \
   VALUE_ENTRY(compressor_amps,              "A",      2048 ) \
   VALUE_ENTRY(radiator_pwm,                "%",       2049 ) \
   VALUE_ENTRY(shutterservo,                "°",       2050 ) \
   VALUE_ENTRY(temp_cabin_left,             "°C",      2051 ) \
   VALUE_ENTRY(temp_cabin_right,            "°C",      2052 ) \
   VALUE_ENTRY(opmode,                      OPMODES,   2000   ) \
   VALUE_ENTRY(version,                     VERSTR,    2001   ) \
   VALUE_ENTRY(lasterr,                     errorListString,  2002 ) \
   VALUE_ENTRY(uaux,                        "V",       2003   ) \
   VALUE_ENTRY(cpuload,                     "%",       2004   )


/***** Enum String definitions *****/
#define OPMODES      "0=Off, 1=Run"
#define CANSPEEDS    "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define OCTOPOS      "0=Unknown, 1=Series, 2=Series, 3=Ambient Source, 4=Radiator Bypass, 5=Parallel"
#define NTC          "0=JCurve, 1=KTY81-110, 2=KTY83-110, 3=KTY84-130, 4=Tesla_100K, 5=Tesla_10K, 6=PT1000"
#define CANPERIODS   "0=100ms, 1=10ms"
#define CTRL_MODE    "0=Basic, 1=ClimateControl"
#define SOLENOID     "0=Open, 1=Closed"
#define CANIO        "0=HW-IO, 1=CAN-IO"
#define HWIO_SRC     "0=Ext1, 1=Ext2, 2=Ext3, 3=Ext4"
#define CAT_TEST     "Testing"
#define CAT_COMM     "Communication"
#define CAT_CTRL     "Interface"

#define VERSTR STRINGIFY(4=VER-name)

/***** enums ******/


enum _canperiods
{
   CAN_PERIOD_100MS = 0,
   CAN_PERIOD_10MS,
   CAN_PERIOD_LAST
};

enum _modes
{
   MOD_OFF = 0,
   MOD_RUN,
   MOD_LAST
};

enum _canio
{
   HW_IO = 0,
   CAN_IO = 1
};

enum OctoPositions
{
	POS1 = 1, // same as POS2
	POS2_SERIES = 2,   // Battery and powertrain in series, Cool battery and powertrain, Reject heat to radiator or cabin
	POS3_AMBIENT = 3,  // Reject heat to battery or cabin, Get heat from radiator
	POS4_RBYPASS = 4,  // Battery and powertrain in series, Cool battery and powertrain, Exclude radiator, Reject heat to cabin
	POS5_PARALLEL = 5  // Battery and powertrain separate, Cool battery with a/c, Powertrain heat to radiator, Reject heat from battery or cabin to radiator
};

enum ControlMode
{
   Basic = 0,
   ClimateControl = 1
};

//Generated enum-string for possible errors
extern const char* errorListString;

