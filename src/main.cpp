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
#include "linbus.h"

#include "valves.h"
#include "interface.h"
#include "temp_meas.h"
#include "sensors.h"
#include "drv8316.h"
#include "pumps.h"
#include "MCP3208.h"
#include "statemachine.h"


#define CAN_TIMEOUT 50  //500ms
#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual() { while (1); }

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static Terminal* terminal;
static LinBus* lin;

DRV8316Driver waterpumpA(I2C1, TARGET_ID);
DRV8316Driver waterpumpB(I2C2, TARGET_ID);

MCP3208 adc(SPI1, DigIo::exp1_dir);

uint16_t exti_line_state;
 

static void Ms2Task(void) // used for step drivers, 2ms interval dictates step pulse and is very critical!
{
   Valve::expansionCheckSteps();
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

   Param::SetInt(Param::cool_battery, DigIo::battery_cool.Get());
   Param::SetInt(Param::heat_battery, DigIo::battery_heat.Get());
   Param::SetInt(Param::heat_cabinl, DigIo::cabin_heatl.Get());
   Param::SetInt(Param::heat_cabinr, DigIo::cabin_heatr.Get());
   Param::SetInt(Param::cool_cabin, DigIo::cabin_cool.Get());
   Param::SetInt(Param::enable_pumps, DigIo::pumps_enable.Get());
   Param::SetInt(Param::gpi1, DigIo::gpi1.Get());


   if(Param::GetInt(Param::cool_battery))
   {
      DigIo::octo_in1.Set();
      DigIo::octo_in2.Clear();
   }

   if(Param::GetInt(Param::heat_battery))
   {
      DigIo::octo_in1.Clear();
      DigIo::octo_in2.Set();
   }

   if(Param::GetInt(Param::cool_battery) == 0 && Param::GetInt(Param::heat_battery) == 0)
   {
      DigIo::octo_in1.Clear();
      DigIo::octo_in2.Clear();
   }

   if(Param::GetInt(Param::gpi1))
   {
      Valve::solenoidClose();
   }  
   else
   {
      Valve::solenoidOpen();
   }

   if(Param::GetInt(Param::heat_cabinr))
   {
      Valve::expansionSetPos(EXP1, 255);
      Valve::expansionSetPos(EXP2, 255);
      Valve::expansionSetPos(EXP3, 255);
      Valve::expansionSetPos(EXP4, 255);
      Valve::expansionSetPos(EXP5, 255);
      Valve::expansionSetPos(EXP6, 255);
   }

   if(Param::GetInt(Param::heat_cabinl))
   {
      Valve::expansionSetPos(EXP1, 0);
      Valve::expansionSetPos(EXP2, 0);
      Valve::expansionSetPos(EXP3, 0);
      Valve::expansionSetPos(EXP4, 0);
      Valve::expansionSetPos(EXP5, 0);
      Valve::expansionSetPos(EXP6, 0);
   }

   if(Param::GetInt(Param::cool_cabin))
   {
      Valve::expansionCalibrateAll();
   }

   


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

   Param::SetInt(Param::octo_pos, Valve::octoGetPos());
   Param::SetFloat(Param::pumpa_flow, Waterpump::getFlowA());
   Param::SetFloat(Param::pumpb_flow, Waterpump::getFlowB());
   //SetFlowPumpA(20000);
   //SetFlowPumpB(5000);

   GetTemps();
   
   Param::SetInt(Param::ana_in1, adc.analogRead(4));
   Param::SetInt(Param::ana_in2, adc.analogRead(5));
   Param::SetInt(Param::ana_in3, adc.analogRead(6));
   Param::SetInt(Param::ana_in4, adc.analogRead(7));

   //If we chose to send CAN messages every 100 ms, do this here.
   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      canMap->SendAll();




   /*  --------------------------------------------------  */
   /*    The whole thermal management happens in here!     */
   /*  --------------------------------------------------  */
   StateMachine(); // 

}



static void Ms500Task(void) 
{

}







void exti0_isr(void)
{
	exti_line_state = GPIOA_IDR;

	//if ((exti_line_state & (1 << 0)) != 0) 
   if (exti_get_flag_status(EXTI8))
   {
		Param::SetInt(Param::octo_pos, 100);
	} else {
		
	}

	exti_reset_request(EXTI8);
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
   
   case 0x223: // AC compressor status flags
         Compressor::handle223(data);
      break;

   case 0x233: // AC compressor HV status
         Compressor::handle233(data);
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
   can->RegisterUserMessage(0x223); // AC compressor status flags
   can->RegisterUserMessage(0x233); // AC compressor HV status
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
      break;

   default:
      //Handle general parameter changes here. Add paramNum labels for handling specific parameters
      break;
   }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
   scheduler->Run();
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

   nvic_setup(); //Set up some interrupts
   parm_load(); //Load stored parameters
   tim_setup(); 
   exti_setup();
   spi1_setup();   //spi 1 used for External ADC
   //usart3_setup(); // used for LIN communication

   Stm32Scheduler s(TIM3); //We never exit main so it's ok to put it on stack
   scheduler = &s;

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
   
   //LinBus l(USART3, 19200);
   //lin = &l;
   
   Terminal t(USART1, termCmds);
   terminal = &t;
   
   scheduler->AddTask(Ms2Task, 2);
   scheduler->AddTask(Ms10Task, 10);
   scheduler->AddTask(Ms100Task, 100);
   scheduler->AddTask(Ms500Task, 500); // max 654

   //waterpumpA.init();
   //waterpumpB.init();

   adc.begin();

   Param::SetInt(Param::version, 4);
   Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation

   DigIo::exp_sleep.Set(); // dont put stepper drivers to sleep
   delay_ms(10);
   Valve::expansionCalibrateAll();


   while(1)
   {
      char c = 0;
      terminal->Run();  
      if (canSdo->GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(&sdo, &c);
      }
   }


   return 0;
}