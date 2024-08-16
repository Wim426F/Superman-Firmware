#ifndef PUMPS_H
#define PUMPS_H

#include <stdint.h>
#include "stm32_can.h"
#include "params.h"
#include "digio.h"
#include "utils.h"
#include "hwinit.h"
#include "drv8316.h"

extern DRV8316Driver waterpumpA;
extern DRV8316Driver waterpumpB;


class Compressor
{
public:
    static void handle223(uint32_t data[2]);
    static void handle233(uint32_t data[2]);
    static void SendMessages(CanHardware* can);
    static void SetDuty(int duty); // percentage   
};

class Waterpump
{
public:
    static void setFlowA(int lpm);
    static void setFlowB(int lpm);
    static float getFlowA();
    static float getFlowB();
};



#endif // PUMPS_H