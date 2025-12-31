/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "sdocommands.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"

#include "valves.h"
#include "interface.h"
#include "temp_meas.h"
#include "sensors.h"
#include "pumps.h"
#include "thermal_control.h"


#define CAN_TIMEOUT 50  //500ms
#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual() { while (1); }

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static Terminal* terminal;


volatile int octovalve_position = 0;
volatile uint32_t pump_batt_high_time = 0, pump_batt_period = 0;
volatile uint32_t pump_pt_high_time = 0, pump_pt_period = 0;
volatile bool pump_batt_ready = false, pump_pt_ready = false;


float GetBatteryPumpDuty() {
    if (!pump_batt_ready) return 0.0f;
    pump_batt_ready = false;
    if (pump_batt_period == 0) return 0.0f;
    return (float)pump_batt_high_time / pump_batt_period * 100.0f;
}

float GetPowertrainPumpDuty() {
    if (!pump_pt_ready) return 0.0f;
    pump_pt_ready = false;
    if (pump_pt_period == 0) return 0.0f;
    return (float)pump_pt_high_time / pump_pt_period * 100.0f;
}

static void Ms1_5Task(void) // used for step drivers, 1.5ms interval for step pulse is very critical!
{
   Valve::expansionRunSteps();
}


static void Ms10Task(void)
{
   static bool canIoActive = false;
   int canio = Param::GetInt(Param::canio);
   canIoActive |= canio != 0;

   if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) >= CAN_TIMEOUT && canIoActive)
   {
      canio = 0;
      Param::SetInt(Param::canio, 0);
      ErrorMessage::Post(ERR_CANTIMEOUT);
   }

   Param::SetInt(Param::heat_cabinl, DigIo::cabin_heatl.Get());
   Param::SetInt(Param::heat_cabinr, DigIo::cabin_heatr.Get());
   Param::SetInt(Param::cool_cabin, DigIo::cabin_cool.Get());
   Param::SetInt(Param::preheat_req, DigIo::preheat_req.Get());

   Param::SetInt(Param::pump_battery_duty, GetBatteryPumpDuty());
   Param::SetInt(Param::pump_powertrain_duty, GetPowertrainPumpDuty());
   Param::SetInt(Param::octo_pos, octovalve_position);


   //If we chose to send CAN messages every 10 ms, do this here.
   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      canMap->SendAll();
}



static void Ms100Task(void)
{  
   iwdg_reset();
   float cpuLoad = scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);

   // Calculate 12V supply voltage from voltage divider
   float uauxGain = 203; // 1k/(5.1k+1k)/3.33v*4095 = 203
   Param::SetFloat(Param::uaux, ((float)AnaIn::uaux.Get()) / uauxGain);
   
   //Set timestamp of error message
   ErrorMessage::SetTime(rtc_get_counter_val());
   
   Interface::SendMessages(can);
   Compressor::SendMessages(can);

   GetSensorReadings();

   thermalControl(); // The whole thermal management happens in here

   //If we chose to send CAN messages every 100 ms, do this here.
   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      canMap->SendAll();
}



static void Ms200Task(void) 
{
   DigIo::led_out.Toggle();
}



static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) // Called when a defined CAN message is received.
{
   dlc=dlc;
   switch (id)
   {
   case 0x730: // Params
         Interface::handle730(data);
      break;

   case 0x731: // Setpoints and actual temperatures
         Interface::handle731(data);
      break;
   
   case 0x227: // AC compressor status flags
         Compressor::handle227(data);
      break;

   case 0x2A8: // AC compressor HV status
         Compressor::handle2A8(data);
      break;

   default:
   
      break;
   }
   return false;
}

//Whenever the user clears mapped can messages or changes the
//CAN interface of a device, this will be called by the CanHardware module
static void SetCanFilters()
{
   //CanHardware* inverter_can = canInterface[Param::GetInt(Param::inv_can)];
   can->RegisterUserMessage(0x730); // Params
   can->RegisterUserMessage(0x731); // Setpoints and actual temperatures
   can->RegisterUserMessage(0x227); // AC compressor status flags
   can->RegisterUserMessage(0x2A8); // AC compressor HV status
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   case Param::canspeed: 
      can->SetBaudrate((CanHardware::baudrates)Param::GetInt(Param::canspeed));
      break;

   case Param::nodeid:
      canSdo->SetNodeId(Param::GetInt(Param::nodeid)); //Set node ID for SDO access
      //can->RegisterUserMessage(0x600 + Param::GetInt(Param::nodeid)); // Dynamic CanSDO request COB-ID (0x600 + Node-ID)
      break;

   default:
      //Handle general parameter changes here. Add paramNum labels for handling specific parameters
      break;
   }
}


//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim3_isr(void)
{
   scheduler->Run();
}

// Octovalve encoder
extern "C" void exti9_5_isr(void) {
    if (exti_get_flag_status(EXTI8)) {
        if (Valve::valve_turning_direction == CLOCKWISE) {
            octovalve_position++;
        } else {
            octovalve_position--;
        }
        exti_reset_request(EXTI8);
    }
}

// Waterpump PWM duty feedback 
extern "C" void tim4_isr(void) {
    if (timer_get_flag(TIM4, TIM_SR_CC3IF)) {  // Pump_BATT CH3
        uint32_t now = timer_get_ic_value(TIM4, TIM_IC3);  // CCR3
        static uint32_t last_rise = 0;
        static enum tim_ic_pol current_pol = TIM_IC_RISING;
        if (current_pol == TIM_IC_RISING) {
            pump_batt_period = now - last_rise;  // From previous rising
            last_rise = now;
            current_pol = TIM_IC_FALLING;
            timer_ic_set_polarity(TIM4, TIM_IC3, TIM_IC_FALLING);
        } else {
            pump_batt_high_time = now - last_rise;
            current_pol = TIM_IC_RISING;
            timer_ic_set_polarity(TIM4, TIM_IC3, TIM_IC_RISING);
            pump_batt_ready = true;
        }
        timer_clear_flag(TIM4, TIM_SR_CC3IF);
    }
    if (timer_get_flag(TIM4, TIM_SR_CC4IF)) {  // Pump_PT CH4
        uint32_t now = timer_get_ic_value(TIM4, TIM_IC4);  // CCR4
        static uint32_t last_rise_b = 0;
        static enum tim_ic_pol current_pol_b = TIM_IC_RISING;
        if (current_pol_b == TIM_IC_RISING) {
            pump_pt_period = now - last_rise_b;
            last_rise_b = now;
            current_pol_b = TIM_IC_FALLING;
            timer_ic_set_polarity(TIM4, TIM_IC4, TIM_IC_FALLING);
        } else {
            pump_pt_high_time = now - last_rise_b;
            current_pol_b = TIM_IC_RISING;
            timer_ic_set_polarity(TIM4, TIM_IC4, TIM_IC_RISING);
            pump_pt_ready = true;
        }
        timer_clear_flag(TIM4, TIM_SR_CC4IF);
    }
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup(); //Must always come first
   rtc_setup();

   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   AnaIn::Start(); //Starts background ADC conversion via DMA

   write_bootloader_pininit(); //Instructs boot loader to initialize certain pins

   nvic_setup();  // Set up some interrupts
   parm_load();   // Load stored parameters
   tim_setup();

   Terminal t(USART1, termCmds);
   terminal = &t;

   //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
   //store a pointer for easier access
   FunctionPointerCallback canCb(CanCallback, SetCanFilters);

   Stm32Can c(CAN1, (CanHardware::baudrates)Param::GetInt(Param::canspeed));
   can = &c;
   can->AddCallback(&canCb);
   SetCanFilters();

   CanMap cm(&c);
   canMap = &cm;
   TerminalCommands::SetCanMap(canMap);

   CanSdo sdo(&c, &cm);
   canSdo = &sdo;
   canSdo->SetNodeId(Param::GetInt(Param::nodeid)); //Set node ID for SDO access e.g. by wifi module
   SdoCommands::SetCanMap(canMap);

   
   Stm32Scheduler s(TIM3); //We never exit main so it's ok to put it on stack
   // manually set to different frequency to have finer control over task duration.
   timer_set_prescaler(TIM3, (2 * rcc_apb1_frequency) / 200000 - 1); //timer now has 5us ticks instead of 10us. thus 0.5ms granularity in tasklength

   scheduler = &s;

   //task length is in 0.5ms so must fill in double the wanted interval (327ms max)
   s.AddTask(Ms1_5Task, 3);     // 1.5ms actual, is very critical for smooth stepping!
   s.AddTask(Ms10Task, 20);   // 10ms actual
   s.AddTask(Ms100Task, 200); // 100ms actual
   s.AddTask(Ms200Task, 400); // 200ms actual


   Param::SetInt(Param::version, 0);
   Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation

   delay_ms(10);
   Valve::expansionCalibrateAll();

   while(1)
   {
      char c = 0;
      CanSdo::SdoFrame* sdoFrame = sdo.GetPendingUserspaceSdo();
      terminal->Run();

      if (canSdo->GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(canSdo, &c);
      }
      if (0 != sdoFrame)
      {
         CanSdo::SdoFrame sdoOrig = *sdoFrame;
         SdoCommands::ProcessStandardCommands(sdoFrame);

         //if (sdoFrame->cmd == SDO_ABORT)
         //{
         //   *sdoFrame = sdoOrig;
         //   ProcessCustomSdoCommands(sdoFrame);
         //}

         sdo.SendSdoReply(sdoFrame);
      }
   }

   return 0;
}