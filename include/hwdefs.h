#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define RCC_CLOCK_SETUP() rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ])
#define OVER_CUR_TIMER     TIM4
#define OCURMAX            4096

// PWM output channels
#define PWM_SERVO   TIM_OC3
#define PWM_FAN     TIM_OC4
#define PWM_PUMPA   TIM_OC1
#define PWM_PUMPB   TIM_OC2

// Timer input channels
#define OCTOVALVE_TIM   TIM1 
#define OCTOVALVE_CH    TIM_IC1

#define PWMIN_PUMPA_TIM    TIM4
#define PWMIN_PUMPA_CH     TIM_IC3

#define PWMIN_PUMPB_CH     TIM4
#define PWMIN_PUMPB_CH     TIM_IC4

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
