#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "params.h"
#include "digio.h"
#include "utils.h"
#include "temp_meas.h"
#include "MCP3208.h"

extern MCP3208 adc;

void GetTemps();


#endif // SENSORS_H