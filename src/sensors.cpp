#include "sensors.h"
#include "errormessage.h"
#include "interface.h"
#include "math.h"
#include "Ewma.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rtc.h>

// A raw reading pinned at its sensor's rail for this many GetSensorReadings()
// cycles (100ms each) means open/short circuit, not a real extreme reading.
#define SENSOR_PIN_FAULT_CYCLES 20 // 2000ms
static inline bool pinned(float raw, float lo, float hi) { return raw <= lo || raw >= hi; }

/* ---------------------------------------------------------------------------
 * Coolant reservoir level (capacitive sensor on PB1 / ADC12_IN9)
 *
 * The sensor is two rods in the reservoir: one to GND, the other via a 10k
 * series resistor to PB1. Submerged in coolant they form a capacitor whose
 * value scales with fill level
 *
 * We measure it by timing an RC charge (time-to-threshold):
 *   1. Drive PB1 low for a while to fully discharge the cap.
 *   2. Drive PB1 high (push-pull) so the cap charges through the 10k, and start
 *      a timer.
 *   3. Every 10ms briefly flip PB1 to analog input and read it with ADC2 (with
 *      the pin high-Z there is no drop across the 10k, so the ADC sees the cap
 *      voltage directly), then resume charging.
 *   4. Measure the time to reach a fixed ADC count (RESERVOIR_ADC_THRESHOLD).
 *
 * NOTE ON THE PHYSICS: the coolant isn't a pure capacitor, it also conducts
 * so the voltage never reaches the 63.2% of VCC.
 * so the classic "time-to-63.2%-of-Vcc == RC" does not hold here
 * RESERVOIR_UF_CAL rescales it to the real capacitance .
 * 
 * ------------------------------------------------------------------------- */
#define RESERVOIR_PORT        GPIOB
#define RESERVOIR_PIN         GPIO1
#define RESERVOIR_ADC_CH      9          // PB1 = ADC12_IN9
#define RESERVOIR_SERIES_R    10000.0f   // series resistor ohm
#define RESERVOIR_DISCHARGE_TICKS  100   // 1000ms
#define RESERVOIR_ADC_THRESHOLD    1500  // empirically chosen
#define RESERVOIR_CHARGE_TIMEOUT   100   // 1000ms

// Empirical calibration against a LCR meter reading ~25uF at the connector with a full reservoir
#define RESERVOIR_UF_CAL       0.82f

// reservoir levels
#define RESERVOIR_UF_MAX      24.0f    // >= this -> COOLANT_MAX       marker at 24
#define RESERVOIR_UF_NOMINAL  21.0f    // >= this -> COOLANT_NOMINAL   marker at 22
#define RESERVOIR_UF_MINIMUM  10.0f    // >= this -> COOLANT_MINIMUM   marker at 15
                                       // else       COOLANT_EMPTY

Ewma ps1_filter(0.5);
Ewma ps2_filter(0.5);
Ewma ps3_filter(0.5);

Ewma ps1t_filter(0.5);
Ewma ps2t_filter(0.5);
Ewma ps3t_filter(0.5);

Ewma battout_filter(0.5);
Ewma ptout_filter (0.5);
Ewma reserv_filter(0.3);

Ewma ntc1_filter(0.5);
Ewma ntc2_filter(0.5);
Ewma ntc3_filter(0.5);
Ewma ntc4_filter(0.5);

/* Ewma radiator_filter(0.5); */
Ewma ambient_filter(0.5);
Ewma cabin_left_filter(0.5);
Ewma cabin_right_filter(0.5);
/* Ewma battery_filter(0.5); */
/* Ewma powertrain_filter(0.5); */

float HIGHPRESSURE_SENSOR = 37.265; // rated pressure for high side pressure sensor
float LOWPRESSURE_SENSOR = 10.859;  // rated pressure for low side pressure sensor

float temp_ps1_offset = 3;
float temp_ps2_offset = 3;
float temp_ps3_offset = 3;

float temp_ntc1_offset = 0;
float temp_ntc2_offset = 0;
float temp_ntc3_offset = 0;
float temp_ntc4_offset = 0;

float temp_battout_offset = 0;
float temp_ptout_offset = 0;
float temp_reserv_offset = 0;


/* Reservoir pin helpers ---------------------------------------------------- */
// Drive PB1 as push-pull output, high=charge the cap, low=discharge it.
static inline void reservoirDrive(bool high)
{
   gpio_set_mode(RESERVOIR_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RESERVOIR_PIN);
   if (high)
      gpio_set(RESERVOIR_PORT, RESERVOIR_PIN);
   else
      gpio_clear(RESERVOIR_PORT, RESERVOIR_PIN);
}

// Switch PB1 to analog input and do one ADC2 conversion.
static uint16_t reservoirRead()
{
   gpio_set_mode(RESERVOIR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, RESERVOIR_PIN);
   adc_start_conversion_regular(ADC2);
   while (!adc_eoc(ADC2));
   return adc_read_regular(ADC2);
}

static uint8_t reservoirBucket(float uF)
{
   if (uF >= RESERVOIR_UF_MAX)     return COOLANT_MAX;
   if (uF >= RESERVOIR_UF_NOMINAL) return COOLANT_NOMINAL;
   if (uF >= RESERVOIR_UF_MINIMUM) return COOLANT_MINIMUM;
   return COOLANT_EMPTY;
}

void UpdateReservoirLevel()
{
   enum ResState { RES_INIT, RES_DISCHARGE, RES_CHARGE };
   static ResState state = RES_INIT;
   static uint32_t phaseStart = 0;

   uint32_t now = rtc_get_counter_val();       // 10ms ticks

   switch (state)
   {
   case RES_INIT:
      // prime the first discharge so we never measure an undrained cap
      reservoirDrive(false);
      phaseStart = now;
      state = RES_DISCHARGE;
      break;

   case RES_DISCHARGE:
      // pin held low; wait until the cap is fully drained
      if ((now - phaseStart) >= RESERVOIR_DISCHARGE_TICKS)
      {
         reservoirDrive(true);
         phaseStart = now;
         state = RES_CHARGE;
      }
      break;

   case RES_CHARGE:
   {
      // brief analog read (charge paused ~20us), then decide
      uint16_t adc   = reservoirRead();
      uint32_t ticks = now - phaseStart;

      if (adc >= RESERVOIR_ADC_THRESHOLD || ticks >= RESERVOIR_CHARGE_TIMEOUT)
      {
         // reached the threshold, or timed out (open circuit / faulted high)
         reservoirDrive(false);                // start draining again immediately
         phaseStart = now;
         state = RES_DISCHARGE;

         float tSec   = ticks * 0.01f;          // seconds
         float raw_uF = (tSec / RESERVOIR_SERIES_R) * 1e6f;
         float uF     = reserv_filter.filter(raw_uF * RESERVOIR_UF_CAL);

         uint8_t level = reservoirBucket(uF);
         Param::SetFloat(Param::reservoir_cap, uF);
         Param::SetInt(Param::reservoir_level, level);
         if (level <= COOLANT_MINIMUM) ErrorMessage::Post(ERR_COOLANT_LOW);
      }
      else
      {
         reservoirDrive(true);                  // keep charging
      }
      break;
   }
   }
}

void GetSensorReadings()
{
    float pps1raw = (float)AnaIn::pressure_inlet_compressor.Get(); // low side sensor
    float pps2raw = (float)AnaIn::pressure_outlet_compressor.Get(); // high side sensor
    float pps3raw = (float)AnaIn::pressure_pre_evaporator.Get(); // low side sensor

    // needs an offset because sensor starts measuring above certain treshold
    float pps1 = utils::changeFloat(pps1raw, 590, 4096, 0, LOWPRESSURE_SENSOR); // low side sensor
    float pps2 = utils::changeFloat(pps2raw, 370, 4096, 0, HIGHPRESSURE_SENSOR); // high side sensor
    float pps3 = utils::changeFloat(pps3raw, 590, 4096, 0, LOWPRESSURE_SENSOR); // low side sensor

    pps1 = utils::limitVal(pps1, 0, LOWPRESSURE_SENSOR);
    pps2 = utils::limitVal(pps2, 0, HIGHPRESSURE_SENSOR);
    pps3 = utils::limitVal(pps3, 0, LOWPRESSURE_SENSOR);

    Param::SetFloat(Param::pressure_inlet_compressor,   ps1_filter.filter(pps1)); // low side sensor
    Param::SetFloat(Param::pressure_outlet_compressor,  ps2_filter.filter(pps2)); // high side sensor
    Param::SetFloat(Param::pressure_pre_evaporator,     ps3_filter.filter(pps3)); // low side sensor

    // resistance gets lower when hotter (ntc)
    float t1 = TempMeas::Lookup(AnaIn::temp_inlet_compressor.Get(),  TempMeas::TEMP_TESLA_10K);
    float t2 = TempMeas::Lookup(AnaIn::temp_outlet_compressor.Get(), TempMeas::TEMP_TESLA_10K);
    float t3 = TempMeas::Lookup(AnaIn::temp_pre_evaporator.Get(),    TempMeas::TEMP_TESLA_10K);
    float t4 = TempMeas::Lookup(AnaIn::temp_outlet_battery.Get(),    TempMeas::TEMP_GE1935);
    float t5 = TempMeas::Lookup(AnaIn::temp_outlet_powertrain.Get(), TempMeas::TEMP_GE1935);
    /* float t6 = TempMeas::Lookup(AnaIn::temp_radiator.Get(),          TempMeas::TEMP_GE1935); */
    float t7 = TempMeas::Lookup(AnaIn::temp_ambient.Get(),           TempMeas::TEMP_GE1935);
    /* float t8 = TempMeas::Lookup(AnaIn::temp_battery.Get(),           TempMeas::TEMP_GE1935); */
    /* float t9 = TempMeas::Lookup(AnaIn::temp_powertrain.Get(),        TempMeas::TEMP_GE1935); */

    Param::SetFloat(Param::temp_inlet_compressor,   ps1t_filter.filter(t1) - temp_ps1_offset);
    Param::SetFloat(Param::temp_outlet_compressor,  ps2t_filter.filter(t2) - temp_ps2_offset);
    Param::SetFloat(Param::temp_pre_evaporator,     ps3t_filter.filter(t3) - temp_ps3_offset);

    if (!Interface::canBatteryTemp)
        Param::SetFloat(Param::temp_outlet_battery,    battout_filter.filter(t4) - temp_battout_offset);
    if (!Interface::canPowertrainTemp)
        Param::SetFloat(Param::temp_outlet_powertrain, ptout_filter.filter(t5)  - temp_ptout_offset);

    /* Param::SetFloat(Param::temp_radiator,       radiator_filter.filter(t6)); */
    Param::SetFloat(Param::temp_ambient,        ambient_filter.filter(t7));
    /* Param::SetFloat(Param::temp_battery,        battery_filter.filter(t8)); */
    /* Param::SetFloat(Param::temp_powertrain,     powertrain_filter.filter(t9)); */

    // Check if sensor reads at rail voltage, in that case throw error.
    bool pressurePinned = pinned(pps1raw, 5, 4090) || pinned(pps2raw, 5, 4090) || pinned(pps3raw, 5, 4090);
    bool tempPinned = pinned(t1, -30, 110) || pinned(t2, -30, 110) || pinned(t3, -30, 110) // TESLA_10K
                    || pinned(t4, -40, 120) || pinned(t5, -40, 120) // GE1935 battery/PT outlet
                    || pinned(t7, -40, 120); // GE1935 ambient
                    /* || pinned(t6, -40, 120) || pinned(t8, -40, 120) || pinned(t9, -40, 120); */

    static uint16_t pressurePinCount = 0;
    static uint16_t tempPinCount = 0;

    pressurePinCount = pressurePinned ? (pressurePinCount + 1) : 0;
    if (pressurePinCount >= SENSOR_PIN_FAULT_CYCLES) ErrorMessage::Post(ERR_PRESSURE_SENSOR_FAULT);

    tempPinCount = tempPinned ? (tempPinCount + 1) : 0;
    if (tempPinCount >= SENSOR_PIN_FAULT_CYCLES) ErrorMessage::Post(ERR_TEMP_SENSOR_FAULT);
}