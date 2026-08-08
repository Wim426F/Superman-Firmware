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
    static void handle2A7(uint32_t data[2]); // compressor state
    static void handle2A6(uint32_t data[2]); // echo of the request we send
    static void handle2C7(uint32_t data[2]); // unidentified, not decoded
    static void handle366(uint32_t data[2]); // hardware/component id and app CRC, muxed on byte 0
    static void HandleEmuRx(uint32_t id); // collision arbitration: RX on an emulated VCFRONT id means another node owns it
    static void SendMessages(CanHardware* can); // 100ms: request (or sweep probe), 0x2D1, 0x3A1, 0x321 (every 10th call)
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
    static void checkFaults();
};



#endif // PUMPS_H