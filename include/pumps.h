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
    static void handle227(uint32_t data[2]);
    static void HandleEmuRx(uint32_t id); // collision arbitration: RX on an emulated VCFRONT id means another node owns it
    static void SendMessages(CanHardware* can); // 100ms: 0x281 (always), 0x2D1, 0x3A1, 0x321 (every 10th call)
    static void Send50ms(CanHardware* can);     // 50ms: 0x221, 0x545
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