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
//Next param id (increase when adding new parameter!): 18
//Next value Id: 2057
/*             category     name             unit        min     max     default  id */
#define PARAM_LIST \
   PARAM_ENTRY(CAT_COMM,    canspeed,        CANSPEEDS,  0,      4,      2,       0   ) \
   PARAM_ENTRY(CAT_COMM,    canperiod,       CANPERIODS, 0,      1,      0,       1   ) \
   PARAM_ENTRY(CAT_COMM,    nodeid,          "",         1,      63,     50,      17  ) \
   PARAM_ENTRY(CAT_CTRL,    control_mode,    CTRL_MODE,  0,      3,      0,       2   ) \
   PARAM_ENTRY(CAT_CTRL,    canio,           CANIO,      0,      1,      0,       12  ) \
   PARAM_ENTRY(CAT_CTRL,    cabinl_tempsrc,  HWIO_SRC,   0,      1,      0,       13  ) \
   PARAM_ENTRY(CAT_CTRL,    cabinr_tempsrc,  HWIO_SRC,   0,      1,      1,       14  ) \
   PARAM_ENTRY(CAT_CTRL,    batt_tempsrc,    HWIO_SRC,   0,      1,      2,       15  ) \
   PARAM_ENTRY(CAT_CTRL,    motor_tempsrc,   HWIO_SRC,   0,      1,      3,       16  ) \
   PARAM_ENTRY(CAT_CTRL,    cabinl_setp,     "°C",       0,      40,     20,      3   ) \
   PARAM_ENTRY(CAT_CTRL,    cabinr_setp,     "°C",       0,      40,     20,      4   ) \
   PARAM_ENTRY(CAT_CTRL,    batt_setp,       "°C",       0,      40,     20,      5   ) \
   PARAM_ENTRY(CAT_CTRL,    batt_heatpower,  "W",        0,      10000,  0,       6   ) \
   PARAM_ENTRY(CAT_CTRL,    batt_coolpower,  "W",        0,      10000,  0,       7   ) \
   PARAM_ENTRY(CAT_CTRL,    cabin_heatpower, "W",        0,      10000,  0,       8   ) \
   PARAM_ENTRY(CAT_CTRL,    cabin_coolpower, "W",        0,      10000,  0,       9   ) \
   PARAM_ENTRY(CAT_CTRL,    compressor_plim, "W",        0,      10000,  0,       10  ) \
   PARAM_ENTRY(CAT_CTRL,    ext_ntc,         NTC,        0,      6,      5,       11  ) \
   VALUE_ENTRY(octo_pos,         "dig",     2005)  \
   VALUE_ENTRY(expv1_pos,        "dig",     2041 ) \
   VALUE_ENTRY(expv2_pos,        "dig",     2042 ) \
   VALUE_ENTRY(expv3_pos,        "dig",     2043 ) \
   VALUE_ENTRY(expv4_pos,        "dig",     2044 ) \
   VALUE_ENTRY(expv5_pos,        "dig",     2045 ) \
   VALUE_ENTRY(expv6_pos,        "dig",     2046 ) \
   VALUE_ENTRY(ps1_pressure,     "bar",     2006 ) \
   VALUE_ENTRY(ps2_pressure,     "bar",     2008 ) \
   VALUE_ENTRY(ps3_pressure,     "bar",     2010 ) \
   VALUE_ENTRY(ps1_temperature,  "°C",      2007 ) \
   VALUE_ENTRY(ps2_temperature,  "°C",      2009 ) \
   VALUE_ENTRY(ps3_temperature,  "°C",      2011 ) \
   VALUE_ENTRY(batt_inlet_temp,  "°C",      2012 ) \
   VALUE_ENTRY(pt_inlet_temp,    "°C",      2013 ) \
   VALUE_ENTRY(reservoir_temp,   "°C",      2014 ) \
   VALUE_ENTRY(outside_temp,     "°C",      2052 ) \
   VALUE_ENTRY(ext_temp1,        "°C",      2015 ) \
   VALUE_ENTRY(ext_temp2,        "°C",      2016 ) \
   VALUE_ENTRY(ext_temp3,        "°C",      2017 ) \
   VALUE_ENTRY(ext_temp4,        "°C",      2018 ) \
   VALUE_ENTRY(ana_in1,          "dig",     2048 ) \
   VALUE_ENTRY(ana_in2,          "dig",     2049 ) \
   VALUE_ENTRY(ana_in3,          "dig",     2050 ) \
   VALUE_ENTRY(ana_in4,          "dig",     2051 ) \
   VALUE_ENTRY(solenoid_pos,     "dig",     2047 ) \
   VALUE_ENTRY(gpi1,             "dig",     2019 ) \
   VALUE_ENTRY(heat_cabinl,      "dig",     2020 ) \
   VALUE_ENTRY(heat_cabinr,      "dig",     2021 ) \
   VALUE_ENTRY(cool_cabin,       "dig",     2022 ) \
   VALUE_ENTRY(heat_battery,     "dig",     2023 ) \
   VALUE_ENTRY(cool_battery,     "dig",     2024 ) \
   VALUE_ENTRY(enable_pumps,     "dig",     2025 ) \
   VALUE_ENTRY(pumpa_flow,       "lpm",     2026 ) \
   VALUE_ENTRY(pumpa_current,    "A",       2037 ) \
   VALUE_ENTRY(pumpa_udc,        "V",       2039 ) \
   VALUE_ENTRY(pumpb_flow,       "lpm",     2027 ) \
   VALUE_ENTRY(pumpb_current,    "A",       2038 ) \
   VALUE_ENTRY(pumpb_udc,        "V",       2040 ) \
   VALUE_ENTRY(compressor_speed, "rpm",     2028 ) \
   VALUE_ENTRY(compressor_duty_request,"%", 2056 ) \
   VALUE_ENTRY(compressor_duty,  "%",       2053 ) \
   VALUE_ENTRY(compressor_temp,  "°C",      2055 ) \
   VALUE_ENTRY(compressor_power, "W",       2029 ) \
   VALUE_ENTRY(compressor_HV,    "V",       2035 ) \
   VALUE_ENTRY(compressor_LV,    "V",       2054 ) \
   VALUE_ENTRY(compressor_amps,   "A",      2036 ) \
   VALUE_ENTRY(radiator_pwm,     "%",       2030 ) \
   VALUE_ENTRY(shutterservo,     "°",       2031 ) \
   VALUE_ENTRY(batt_temp,        "°C",      2032 ) \
   VALUE_ENTRY(cabinl_temp,      "°C",      2033 ) \
   VALUE_ENTRY(cabinr_temp,      "°C",      2034 ) \
   VALUE_ENTRY(opmode,           OPMODES, 2000   ) \
   VALUE_ENTRY(version,          VERSTR,  2001   ) \
   VALUE_ENTRY(lasterr,          errorListString,  2002 ) \
   VALUE_ENTRY(uaux,             "V",     2003   ) \
   VALUE_ENTRY(cpuload,          "%",     2004   )


/***** Enum String definitions *****/
#define OPMODES      "0=Off, 1=Run"
#define CANSPEEDS    "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define OCTOPOS      "0=pos1, 1=pos2, 2=pos3, 3=pos3, 4=pos4, 5=pos5"
#define NTC          "0=JCurve, 1=KTY81-110, 2=KTY83-110, 3=KTY84-130, 4=Tesla_100K, 5=Tesla_10K, 6=PT1000"
#define CANPERIODS   "0=100ms, 1=10ms"
#define CTRL_MODE    "0=Classic, 1=ClimateControl"
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
	POS1 = 1,
	POS2 = 2,
	POS3 = 3,
	POS4 = 4,
	POS5 = 5
};

enum ControlMode
{
   Classic = 0,
   ClimateControl = 1
};

//Generated enum-string for possible errors
extern const char* errorListString;

