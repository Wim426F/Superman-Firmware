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


static void write_pwm(uint32_t timer_peripheral, enum tim_oc_id oc_id, uint32_t value)
{
   value = utils::change(value, 0, 2000, 2085, 0);
   timer_set_oc_value(timer_peripheral, oc_id, value); //duty.    1500=28%,      1000 = 52%,     500 = 76%
}