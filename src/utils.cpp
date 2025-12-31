#include "utils.h"
#include <libopencm3/stm32/timer.h>


namespace utils
{

int32_t change(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float changeFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int limitVal(int input, int min, int max)
{
   if(input > max)
      return max;

   if (input < min)
      return min;
   
   return input;
}


double naturalLog(double x, int n) {
    if (x <= 0) {
        return -1;  // Natural logarithm is undefined for non-positive numbers.
    }

    if (x == 1) {
        return 0;  // ln(1) is 0.
    }

    double result = 0;
    double term = 1.0;

    for (int i = 1; i <= n; i++) {
        term *= (x - 1) / x;
        result += term / i;
    }

    return result;
}

} // namespace utils


void pwm_write(uint8_t duty_pct, uint32_t tim, enum tim_oc_id oc, uint32_t arr) {
    if (duty_pct > 100) duty_pct = 100;
    uint32_t ccr = ((uint64_t)duty_pct * (arr + 1UL)) / 100UL;
    if (ccr > arr) ccr = arr;
    timer_set_oc_value(tim, oc, ccr);
}