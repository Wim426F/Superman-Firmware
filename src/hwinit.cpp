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
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/exti.h>
#include "hwdefs.h"
#include "hwinit.h"
#include "stm32_loader.h"
#include "my_string.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_GPIOE);

   rcc_periph_clock_enable(RCC_USART1);

   rcc_periph_clock_enable(RCC_TIM1); // Octovalve encoder
   rcc_periph_clock_enable(RCC_TIM2); // PWM outputs
   rcc_periph_clock_enable(RCC_TIM3); // Scheduler
   rcc_periph_clock_enable(RCC_TIM4); // Waterpumps PWM input/output

   rcc_periph_clock_enable(RCC_DMA1); // ADC, Encoder and UART receive

   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO); // CAN
   rcc_periph_clock_enable(RCC_CAN1); // CAN

   // Remap
   AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_FULL_REMAP;     // CH2=PE11 for solenoid
   AFIO_MAPR |= AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP2; // CH4=PB11 for fan
   
}

/* Some pins should never be left floating at any time
 * Since the bootloader delays firmware startup by a few 100ms
 * We need to tell it which pins we want to initialize right
 * after startup
 */
void write_bootloader_pininit()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* flashCommands = (struct pincommands*)pindefAddr;

   struct pincommands commands;

   memset32((int*)&commands, 0, PINDEF_NUMWORDS);

   //!!! Customize this to match your project !!!
   //Here we specify that PC13 be initialized to ON
   //AND C13 AND C14 be initialized to OFF
   commands.pindef[0].port = GPIOC;
   commands.pindef[0].pin = GPIO3 | GPIO4; // octovalve pin 1 and 2
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 0;


   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)&commands), PINDEF_NUMWORDS);
   commands.crc = crc;

   if (commands.crc != flashCommands->crc)
   {
      flash_unlock();
      flash_erase_page(pindefAddr);

      //Write flash including crc, therefor <=
      for (uint32_t idx = 0; idx <= PINDEF_NUMWORDS; idx++)
      {
         uint32_t* pData = ((uint32_t*)&commands) + idx;
         flash_program_word(pindefAddr + idx * sizeof(uint32_t), *pData);
      }
      flash_lock();
   }
}


/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(NVIC_TIM3_IRQ); //Scheduler
   nvic_set_priority(NVIC_TIM3_IRQ, 0); //Highest priority

   // Enable capture interrupts waterpump feedback
   nvic_enable_irq(NVIC_TIM4_IRQ);
   nvic_set_priority(NVIC_TIM4_IRQ, 1);  // Medium prio

   // Octovalve pulse counter
   nvic_enable_irq(NVIC_EXTI9_5_IRQ);  // For EXTI8 (5-9 shared)
   nvic_set_priority(NVIC_EXTI9_5_IRQ, 2);  // Medium prio
}

void rtc_setup()
{
   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
   //62.5kHz / (624 + 1) = 100Hz
   rtc_auto_awake(RCC_HSE, 624); //10ms tick
   rtc_set_counter_val(0);
}


void tim_setup()
{
   // RCC_TIM1  Octovalve encoder
   // RCC_TIM2  PWM outputs
   // RCC_TIM3  Scheduler
   // RCC_TIM4  Waterpumps PWM input/output

   /*-----------------------------------------------------*/
   /*                  PWM OUTPUTS                        */
   /*-----------------------------------------------------*/


   // TIM1 setup for 20 kHz solenoid_gate (CH2 PE11)
   timer_disable_counter(TIM1);
   timer_set_prescaler(TIM1, 0);
   timer_set_alignment(TIM1, TIM_CR1_CMS_EDGE);
   timer_set_period(TIM1, 3599);  // 72 MHz / 3600 = 20 kHz
   timer_enable_preload(TIM1);

   gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO11);  // PE11=CH2

   timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
   timer_enable_oc_preload(TIM1, TIM_OC2);
   timer_set_oc_polarity_high(TIM1, TIM_OC2);
   timer_enable_oc_output(TIM1, TIM_OC2);
   timer_set_oc_value(TIM1, TIM_OC2, 0);  // 0% init

   timer_enable_counter(TIM1);

   // Timer 2 (PWM outputs, 100 Hz)
   timer_disable_counter(TIM2);
   timer_set_prescaler(TIM2, PWM_FAN_PSC);  // 10
   timer_set_alignment(TIM2, TIM_CR1_CMS_EDGE);
   timer_set_period(TIM2, PWM_FAN_ARR);     // 65535
   timer_enable_preload(TIM2);
   
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3);
   
   // Radiator fan
   timer_set_oc_mode(TIM2, PWM_FAN_OC, TIM_OCM_PWM1);
   timer_enable_oc_preload(TIM2, PWM_FAN_OC);
   timer_set_oc_polarity_high(TIM2, PWM_FAN_OC);
   timer_enable_oc_output(TIM2, PWM_FAN_OC);
   timer_set_oc_value(TIM2, PWM_FAN_OC, 0);  // 0% init (fan off)
   
   timer_enable_counter(TIM2);


   /*-------------------------------------------------*/
   /*    Input pwm capture for waterpumps feedback    */
   /*-------------------------------------------------*/
   timer_disable_counter(TIM4);
   timer_set_prescaler(TIM4, 71); //run at 1 MHz
   timer_set_period(TIM4, 65535);
   timer_direction_up(TIM4);
   timer_slave_set_mode(TIM4, TIM_SMCR_SMS_RM);
   timer_slave_set_polarity(TIM4, TIM_ET_FALLING);
   timer_slave_set_trigger(TIM4, TIM_SMCR_TS_TI1FP1);

   // Waterpump battery
   timer_ic_set_filter(TIM4, TIM_IC3, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_set_input(TIM4, TIM_IC3, TIM_IC_IN_TI3);  // TI3 on PB8
   timer_ic_set_polarity(TIM4, PWMIN_PUMP_BATT_CH, TIM_IC_RISING);
   timer_ic_enable(TIM4, PWMIN_PUMP_BATT_CH);

   // Waterpump powertrain
   timer_ic_set_filter(TIM4, TIM_IC4, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_set_input(TIM4, TIM_IC4, TIM_IC_IN_TI4);  // TI4 on PB9
   timer_ic_set_polarity(TIM4, PWMIN_PUMP_PT_CH, TIM_IC_RISING);
   timer_ic_enable(TIM4, PWMIN_PUMP_PT_CH);

   // Waterpump battery 1 kHz PWM
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
   timer_set_oc_mode(TIM4, PWM_PUMP_BATT_OC, TIM_OCM_PWM1);
   timer_enable_oc_preload(TIM4, PWM_PUMP_BATT_OC);
   timer_set_oc_polarity_high(TIM4, PWM_PUMP_BATT_OC);
   timer_enable_oc_output(TIM4, PWM_PUMP_BATT_OC);
   timer_set_oc_value(TIM4, PWM_PUMP_BATT_OC, 0);  // 0% init (no flow)

   // Waterpump powertrain 1 kHz PWM
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
   timer_set_oc_mode(TIM4, PWM_PUMP_PT_OC, TIM_OCM_PWM1);
   timer_enable_oc_preload(TIM4, PWM_PUMP_PT_OC);
   timer_set_oc_polarity_high(TIM4, PWM_PUMP_PT_OC);
   timer_enable_oc_output(TIM4, PWM_PUMP_PT_OC);
   timer_set_oc_value(TIM4, PWM_PUMP_PT_OC, 0);  // 0% init (no flow)

   timer_generate_event(TIM4, TIM_EGR_UG);
   timer_enable_counter(TIM4);
}
