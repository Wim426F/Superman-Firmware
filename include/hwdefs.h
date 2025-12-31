#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define RCC_CLOCK_SETUP() rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ])
#define OVER_CUR_TIMER     TIM4
#define OCURMAX            4096


// Timer input channels
#define OCTOVALVE_TIM   TIM1 
#define OCTOVALVE_CH    TIM_IC1

#define PWMOUT_PUMP_BATT_TIM   TIM4  
#define PWMOUT_PUMP_BATT_CH    TIM_IC1

#define PWMIN_PUMP_BATT_TIM    TIM4
#define PWMIN_PUMP_BATT_CH     TIM_IC3

#define PWMOUT_PUMP_PT_TIM   TIM4
#define PWMOUT_PUMP_PT_CH    TIM_IC2

#define PWMIN_PUMP_PT_TIM    TIM4
#define PWMIN_PUMP_PT_CH     TIM_IC4

// Fan (TIM2 CH4 PA3, 100 Hz)
#define PWM_FAN_TIM     TIM2
#define PWM_FAN_OC      TIM_OC4
#define PWM_FAN_ARR     65535U  // Max res for granular duty (72 MHz / (11 * 65536) ≈ 100 Hz)
#define PWM_FAN_PSC     10U     // Divides to ~6.55 MHz effective clock for ~100 Hz

// Pumps (TIM4 CH1/2 PB6/7, ~1 kHz)
#define PWM_PUMP_BATT_TIM   TIM4
#define PWM_PUMP_BATT_OC    TIM_OC1  // PB6
#define PWM_PUMP_BATT_ARR   999U     // 1000 ticks @ 1 MHz = 1 kHz
#define PWM_PUMP_BATT_PSC   71U      // Shared w/ inputs (1 MHz clock for PWM meas.)

#define PWM_PUMP_PT_TIM   TIM4
#define PWM_PUMP_PT_OC    TIM_OC2  // PB7
#define PWM_PUMP_PT_ARR   999U
#define PWM_PUMP_PT_PSC   71U 

// Solenoid Gate (TIM1 CH2 PE11, 20 kHz: Fast switching for refrigerant control)
#define PWM_SOLENOID_TIM   TIM1
#define PWM_SOLENOID_OC    TIM_OC2
#define PWM_SOLENOID_ARR   3599U  // 72 MHz / 3600 = 20 kHz
#define PWM_SOLENOID_PSC   0U



//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
