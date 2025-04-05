#ifndef PUMPS_H
#define PUMPS_H

#include <stdint.h>
#include "stm32_can.h"
#include "params.h"
#include "digio.h"
#include "utils.h"
#include "hwinit.h"
//#include "drv8316.h"

//extern DRV8316Driver waterpumpA;
//extern DRV8316Driver waterpumpB;


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
    static void batterySetFlow(int lpm);
    static void powertrainSetFlow(int lpm);
    static float batteryGetFlow();
    static float powertrainGetFlow();
};



#endif // PUMPS_H