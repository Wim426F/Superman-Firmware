#ifndef VALVES_H
#define VALVES_H

#include <stdint.h>
#include "params.h"
#include <libopencm3/stm32/gpio.h>
#include "digio.h"
#include "utils.h"

#define EXPV_RECIRCULATION 1
#define EXPV_CONDENSOR_COOLANT 2
#define EXPV_CONDENSOR_CABINR 3
#define EXPV_CONDENSOR_CABINL 4
#define EXPV_EVAPORATOR_COOLANT 5
#define EXPV_EVAPORATOR_CABIN 6

class Valve
{
private:
    static DigIo* pin_dir;
    static DigIo* pin_en;
    static DigIo* pin_step;
    static bool getPins(int valve);

public:
    static void expansionSetPos(uint8_t valve, uint8_t pos); // 0 = closed, 255 = open
    static void expansionRunSteps(); // takes steps in a non blocking way, must be called every 2ms
    static void expansionCalibrateAll();

    static void solenoidOpen();
    static void solenoidClose();

    static int octoSetPos(int set_position);
    static int octoGetPos();
};





#endif // VALVES_H