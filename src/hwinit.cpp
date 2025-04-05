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
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
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
   rcc_periph_clock_enable(RCC_USART3); // LIN Comms

   rcc_periph_clock_enable(RCC_TIM1); // Octovalve encoder
   rcc_periph_clock_enable(RCC_TIM2); // PWM outputs
   rcc_periph_clock_enable(RCC_TIM3); // Scheduler
   rcc_periph_clock_enable(RCC_TIM4); // Waterpumps PWM input/output

   rcc_periph_clock_enable(RCC_DMA1); // ADC, Encoder and UART receive

   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO); // CAN
   rcc_periph_clock_enable(RCC_CAN1); // CAN
   rcc_periph_clock_enable(RCC_SPI1); // SPI for external ADC
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
   commands.pindef[0].pin = GPIO13 | GPIO14; // octovalve pin 1 and 2
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

void spi1_setup()   //spi 1 used for External ADC
{
   spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
   spi_set_standard_mode(SPI1,0);//set mode 0

   spi_enable_software_slave_management(SPI1);
   //spi_enable_ss_output(SPI1);
   spi_set_nss_high(SPI1);
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO5);//MOSI , CLK
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);//MISO
   spi_enable(SPI1);
}

void usart3_setup(void) // For LIN
{
   /* Setup GPIO pin GPIO_USART3_TX and GPIO_USART3_RX. */
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                 GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);
   usart_set_baudrate(USART3, 19200);
   usart_set_databits(USART3, 8);
   usart_set_stopbits(USART3, USART_STOPBITS_1);
   usart_set_mode(USART3, USART_MODE_TX_RX);
   usart_set_parity(USART3, USART_PARITY_NONE);
   usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
   usart_enable(USART3);
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(NVIC_TIM3_IRQ); //Scheduler
   nvic_set_priority(NVIC_TIM3_IRQ, 0); //Highest priority
   //nvic_set_priority(NVIC_TIM2_IRQ, 0xe << 4); //second lowest priority
   //nvic_enable_irq(NVIC_EXTI0_IRQ); // interrupt for octovalve encoder
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

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                 GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2 | GPIO3); // pwm_servo & pwm_fan
   
   // Timer 2
   timer_disable_counter(TIM2);
   timer_set_alignment(TIM2, TIM_CR1_CMS_EDGE); //edge aligned PWM
   timer_enable_preload(TIM2);
   timer_set_oc_mode(TIM2, PWM_FAN,    TIM_OCM_PWM2); 
   timer_set_oc_mode(TIM2, PWM_SERVO,  TIM_OCM_PWM2); 

   timer_enable_oc_preload(TIM2, PWM_FAN);
   timer_enable_oc_preload(TIM2, PWM_SERVO);

   timer_set_oc_polarity_high(TIM2, PWM_FAN);
   timer_set_oc_polarity_high(TIM2, PWM_SERVO);

   timer_enable_oc_output(TIM2, PWM_FAN);
   timer_enable_oc_output(TIM2, PWM_SERVO);

   timer_set_period(TIM2, 2100); // 1khz?

   timer_enable_counter(TIM2);



   /*-----------------------------------------------------*/
   /*    Pulse counting for octovalve position feedback   */
   /*-----------------------------------------------------*/
   
   //We want the timer to run at 1MHz = 72MHz/72
   //Prescaler is div-1 => 71
   timer_set_prescaler(TIM1, 71); //run at 1 MHz
   timer_set_period(TIM1, 65535); // Timer period in the auto-reload register.
   timer_slave_set_mode(TIM1, TIM_SMCR_SMS_RM);
   //timer_slave_set_polarity(TIM1, TIM_ET_RISING);
   //timer_slave_set_trigger(TIM1, TIM_SMCR_TS_TI1FP1);
   //
   //timer_ic_set_filter(TIM1, TIM_IC1, TIM_IC_DTF_DIV_32_N_8);
   //timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);//measure octovalve encoder, input capture channel 1
   //timer_set_oc_polarity_high(TIM1, TIM_OC1);
   //timer_ic_enable(TIM1, TIM_IC1);
   //timer_generate_event(TIM1, TIM_EGR_UG);
   //timer_enable_counter(TIM1); 
   /*
   * Setup timer for measuring waterpump PWM feedback
   */


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

   timer_ic_set_filter(TIM4, PWMIN_PUMPA_CH, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_set_filter(TIM4, PWMIN_PUMPB_CH, TIM_IC_DTF_DIV_32_N_8);

   timer_ic_set_input(TIM4, PWMIN_PUMPA_CH, TIM_IC_IN_TI1); // measure waterpumpA PWM, input capture channel 3
   timer_ic_set_input(TIM4, PWMIN_PUMPB_CH, TIM_IC_IN_TI1); // measure waterpumpB PWM, input capture channel 4

   timer_ic_set_polarity(TIM4, PWMIN_PUMPA_CH, TIM_IC_RISING);
   timer_ic_set_polarity(TIM4, PWMIN_PUMPB_CH, TIM_IC_RISING);

   timer_ic_enable(TIM4, PWMIN_PUMPA_CH);
   timer_ic_enable(TIM4, PWMIN_PUMPB_CH);

   timer_generate_event(TIM4, TIM_EGR_UG);
   timer_enable_counter(TIM4);
}

void exti_setup(void)
{
	exti_select_source(EXTI8, GPIOA);
	exti_set_trigger(EXTI8, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI8);
}

