#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "utils.h"
#include "hwinit.h"

class Interface
{

public:
    static void handle6E0(uint32_t data[2]);

    static void SendMessages(CanHardware* can);

    static uint32_t last6E0Rx;
    static bool canBatteryTemp;     // 0x6E0 provided a battery temp (overrides analog)
    static bool canPowertrainTemp;  // 0x6E0 provided a powertrain temp (overrides analog)

};

#endif //INTERFACE_H