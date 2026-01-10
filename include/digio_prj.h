#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    DIG_IO_ENTRY(octo_pulse,            GPIOA, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(exp3_dir,              GPIOA, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(reservoir_level,       GPIOB, GPIO1,  PinMode::INPUT_PU)    \
    DIG_IO_ENTRY(preheat_req,           GPIOB, GPIO2,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(exp2_dir,              GPIOB, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp2_step,             GPIOB, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp2_en,               GPIOB, GPIO5,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pwm_pump_battery,      GPIOB, GPIO6,  PinMode::OUTPUT_ALT)  \
    DIG_IO_ENTRY(pwm_pump_powertrain,   GPIOB, GPIO7,  PinMode::OUTPUT_ALT)  \
    DIG_IO_ENTRY(pump_battery_fb,       GPIOB, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(pump_powertrain_fb,    GPIOB, GPIO9,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(pwm_fan,               GPIOB, GPIO11, PinMode::OUTPUT_ALT)  \
    DIG_IO_ENTRY(led_out,               GPIOB, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp3_step,             GPIOC, GPIO0,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp3_en,               GPIOC, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_dir,              GPIOC, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(octo_in1,              GPIOC, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(octo_in2,              GPIOC, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_step,             GPIOD, GPIO0,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_en,               GPIOD, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_dir,              GPIOD, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_step,             GPIOD, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_en,               GPIOD, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp6_dir,              GPIOD, GPIO5,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp6_step,             GPIOD, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp6_en,               GPIOD, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp1_dir,              GPIOE, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp1_step,             GPIOE, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp1_en,               GPIOE, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cabin_heatr,           GPIOE, GPIO8,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(cabin_cool,            GPIOE, GPIO9,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(cabin_heatl,           GPIOE, GPIO10, PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(valve_lcc,             GPIOE, GPIO11, PinMode::OUTPUT)      \

/*

*/


#endif // PinMode_PRJ_H_INCLUDED
