#ifndef PUMPS_H
#define PUMPS_H

#include <stdint.h>
#include "stm32_can.h"
#include "params.h"
#include "digio.h"
#include "utils.h"
#include "hwinit.h"

class Compressor
{
public:
    static void handle2A8(uint32_t data[2]);
    static void handle227(uint32_t data[2]);
    static void SendMessages(CanHardware* can);
    static void SetDuty(int duty); // percentage   
    static int GetDuty(); // percentage
};

class Waterpump
{
public:
    static void batterySetDuty(uint8_t duty); // duty in %
    static void powertrainSetDuty(uint8_t duty); // duty in %
    static float batteryGetFlow();
    static float powertrainGetFlow();
};



#endif // PUMPS_H