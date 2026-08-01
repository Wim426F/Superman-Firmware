/*
 * This file is part of the Superman heatpump controller project.
 *
 * Copyright (C) 2025 Wim Boone <wim.boone@outlook.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "interface.h"
#include "errormessage.h"
#include <libopencm3/stm32/rtc.h>


uint32_t Interface::last6E0Rx = 0;

void Interface::handle6E0(uint32_t data[2])  // Cabin heat/cool/preheat requests (substitutes digio)
{
    uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes

    last6E0Rx = rtc_get_counter_val();

    Param::SetInt(Param::cool_cabin,  bytes[0]);
    Param::SetInt(Param::heat_cabinl, bytes[1]);
    Param::SetInt(Param::heat_cabinr, bytes[2]);
    Param::SetInt(Param::preheat_req, bytes[3]);

    // Optional direct sensor values, offset 40. 0x00 and 0xFF mean not sent,
    // so a sender may use DLC 4 or pad the unused bytes with 0xFF.
    if(bytes[4] != 0x00 && bytes[4] != 0xFF)
        Param::SetFloat(Param::temp_battery, (float)bytes[4] - 40);

    if(bytes[5] != 0x00 && bytes[5] != 0xFF)
        Param::SetFloat(Param::temp_powertrain, (float)bytes[5] - 40);
}


void Interface::SendMessages(CanHardware* can)
{
    // Superman status message
    uint8_t bytes[8] = {0};
    static uint8_t aliveCounter = 0;

    // Byte 0: opmode (2 bits) | compressor_state (4 bits) | heat_transfer_mode (2 bits)
    uint8_t opmode = Param::GetInt(Param::opmode) & 0x3;
    uint8_t cmpState = Param::GetInt(Param::compressor_state) & 0xF;
    uint8_t heatTransferMode = Param::GetInt(Param::heat_transfer_mode) & 0x3;
    bytes[0] = opmode | (cmpState << 2) | (heatTransferMode << 6);

    // Bytes 1-2: fault bitmask, bit (n-1) set for each currently latched ERR_n (little endian)
    uint16_t faults = 0;
    for (uint8_t i = 0; i < ERROR_BUF_SIZE; i++)
    {
        ERROR_MESSAGE_NUM err = ErrorMessage::GetErrorNum(i);
        if (ErrorMessage::GetErrorTime(i) > 0 && err != ERROR_NONE && err <= 16)
            faults |= (uint16_t)(1u << (err - 1));
    }
    bytes[1] = faults & 0xFF;
    bytes[2] = faults >> 8;

    // Byte 3: active thermal demand/request flags (THERMAL_DEMANDS bitmask)
    bytes[3] = Param::GetInt(Param::thermal_demands) & 0xFF;

    // Byte 4: HV power draw, 0.1kW/count, 0-25.5kW range
    int powerCounts = (Param::GetInt(Param::compressor_power) + 50) / 100;
    bytes[4] = utils::limitVal(powerCounts, 0, 255);

    // Byte 5: alive counter, increments every send
    bytes[5] = aliveCounter++;

    can->Send(0x6E1, (uint32_t*)bytes, 6);
}