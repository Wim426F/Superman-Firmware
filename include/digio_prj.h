#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    DIG_IO_ENTRY(pwm_fan,     GPIOA, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pwm_servo,   GPIOA, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(octo_pulse,  GPIOA, GPIO8,  PinMode::INPUT_FLT)      \
    DIG_IO_ENTRY(exp1_dir,    GPIOA, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cabin_heatr, GPIOB, GPIO2,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(exp6_dir,    GPIOB, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp6_step,   GPIOB, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp6_en,     GPIOB, GPIO5,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(i2c1_scl,    GPIOB, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(i2c1_sda,    GPIOB, GPIO7,  PinMode::OUTPUT_OD)   \
    DIG_IO_ENTRY(i2c2_scl,    GPIOB, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(i2c2_sda,    GPIOB, GPIO11, PinMode::OUTPUT_OD)   \
    DIG_IO_ENTRY(exp1_step,   GPIOC, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp1_en,     GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp2_dir,    GPIOC, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(octo_in1,    GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(octo_in2,    GPIOC, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp2_step,   GPIOD, GPIO0,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp2_en,     GPIOD, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_dir,    GPIOD, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_step,   GPIOD, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp4_en,     GPIOD, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp3_dir,    GPIOD, GPIO5,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp3_step,   GPIOD, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp3_en,     GPIOD, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_dir,    GPIOE, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_step,   GPIOE, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp5_en,     GPIOE, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(exp_sleep,   GPIOE, GPIO5,   PinMode::OUTPUT_OD)  \
    DIG_IO_ENTRY(solenoid,    GPIOE, GPIO6,   PinMode::OUTPUT)     \
    DIG_IO_ENTRY(gpi1,        GPIOE, GPIO7,   PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(cabin_heatl, GPIOE, GPIO8,   PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(cabin_cool,  GPIOE, GPIO9,   PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(battery_heat,GPIOE, GPIO10,   PinMode::INPUT_PD)  \
    DIG_IO_ENTRY(battery_cool,GPIOE, GPIO11,   PinMode::INPUT_PD)  \
    DIG_IO_ENTRY(pumps_enable,GPIOE, GPIO12,   PinMode::INPUT_PD)  \


#endif // PinMode_PRJ_H_INCLUDED
