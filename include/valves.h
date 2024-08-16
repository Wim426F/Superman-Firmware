#ifndef VALVES_H
#define VALVES_H

#include <stdint.h>
#include "params.h"
#include <libopencm3/stm32/gpio.h>
#include "digio.h"
#include "utils.h"

#define EXP1 1
#define EXP2 2
#define EXP3 3
#define EXP4 4
#define EXP5 5
#define EXP6 6

class Valve
{
public:
    static void expansionSetPos(uint8_t valve, uint8_t pos); // 0 = closed, 255 = open
    static void expansionCheckSteps(); // takes steps in a non blocking way, must be called every 2ms
    static void expansionCalibrateAll();

    static void solenoidOpen();
    static void solenoidClose();

    static int octoSetPos(int set_position);
    static int octoGetPos();
};





#endif // VALVES_H