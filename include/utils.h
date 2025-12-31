#ifndef UTILS_H
#define UTILS_H


#include "my_fp.h"
#include "my_math.h"
#include "errormessage.h"
#include "params.h"
#include "digio.h"
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/timer.h>
#include "canhardware.h"
#include "anain.h"


#define byte_to_int(MSB,LSB) (((unsigned int) ((unsigned char) MSB)) & 0xff) <<8 | (((unsigned char) LSB) & 0xff) 
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

namespace utils
{
    int32_t change(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
    float changeFloat(float x, float in_min, float in_max, float out_min, float out_max);
    int limitVal(int input, int min, int max);

    double naturalLog(double x, int n);
}

extern void pwm_write(uint8_t duty_pct, uint32_t tim, enum tim_oc_id oc, uint32_t arr); // 0-100%

static void delay_us(int64_t FLASH_DELAY)
{
    int64_t i;
    for (i = 0; i < (FLASH_DELAY * 4); i++)       
    __asm__("nop");
}

static void delay_ms(int64_t FLASH_DELAY)
{
    int64_t i;
    for (i = 0; i < (FLASH_DELAY * 2000); i++)
    __asm__("nop");
}

#endif
