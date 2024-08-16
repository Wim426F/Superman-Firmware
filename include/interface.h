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

static void handle730(uint32_t data[2]);
static void handle731(uint32_t data[2]);

static void SendMessages(CanHardware* can);

};

#endif //INTERFACE_H