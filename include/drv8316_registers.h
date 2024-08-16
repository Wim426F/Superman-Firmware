#ifndef SIMPLEFOC_DRV8316_REGISTERS
#define SIMPLEFOC_DRV8316_REGISTERS

#include "utils.h"
#include "hwinit.h"

#define TARGET_ID 0x01

/*--------------------------*/
/* 		EEPROM registers	*/
/*--------------------------*/

// ALGORITHM_CONFIGURATION Registers
#define ISD_CONFIG 		0x80
#define REV_DRIVE_CONFIG 0x82
#define MOTOR_STARTUP1 	0x84
#define MOTOR_STARTUP2 	0x86
#define CLOSED_LOOP1 	0x88
#define CLOSED_LOOP2 	0x8A
#define CLOSED_LOOP3 	0x8C
#define CLOSED_LOOP4 	0x8E
#define SPEED_PROFILES1 0x94
#define SPEED_PROFILES2 0x96
#define SPEED_PROFILES3 0x98
#define SPEED_PROFILES4 0x9A
#define SPEED_PROFILES5 0x9C
#define SPEED_PROFILES6 0x9E

// FAULT_CONFIGURATION Registers
#define	FAULT_CONFIG1	0x90
#define FAULT_CONFIG2	0x92

// HARDWARE_CONFIGURATION Registers
#define PIN_CONFIG		0xA4
#define DEVICE_CONFIG1	0xA6
#define DEVICE_CONFIG2	0xA8
#define PERI_CONFIG1	0xAA
#define GD_CONFIG1		0xAC
#define GD_CONFIG2		0xAE

// INTERNAL_ALGORITHM_CONFIGURATION
#define INT_ALGO_1		0xA0
#define INT_ALGO_2		0xA2








/*--------------------------*/
/* 		RAM registers		*/
/*--------------------------*/

// FAULT_STATUS Registers
#define GATE_DRIVER_FAULT_STATUS	0xE0
#define CONTROLLER_FAULT_STATUS		0xE2

// SYSTEM_STATUS Registers
#define ALGO_STATUS 		0xE4
#define MTR_PARAMS			0xE6
#define ALGO_STATUS_MPET 	0xE8

// DEVICE_CONTROL Registers
#define DEV_CTRL 0xEA

// ALGORITHM_CONTROL Registers
#define ALGO_CTRL1	0xEC
#define ALGO_CTRL2	0xEE
#define CURRENT_PI 	0xF0
#define SPEED_PI	0xF2

// ALGORITHM_VARIABLES Registers
#define ALGORITHM_STATE			0X210 // Current Algorithm State Register
#define FG_SPEED_FDBK			0X216 // Speed Feedback from FG
#define BUS_CURRENT				0X410 // Calculated Supply Current Register
#define PHASE_CURRENT_A			0X43E // Measured current on Phase A Regist
#define PHASE_CURRENT_B			0X440 // Measured current on Phase B Regist
#define PHASE_CURRENT_C			0X442 // Measured current on Phase C Regist
#define CSA_GAIN_FEEDBACK		0X466 // VM Voltage Register
#define VOLTAGE_GAIN_FEEDBACK	0X476 // Voltage Gain Register
#define VM_VOLTAGE				0X478 // Supply voltage register
#define PHASE_VOLTAGE_VA		0X47E // Phase Voltage Register
#define PHASE_VOLTAGE_VB		0X480 // Phase Voltage Register
#define PHASE_VOLTAGE_VC		0X482 // Phase Voltage Register
#define SIN_COMMUTATION_ANGLE	0X4BA // Sine of Commutation Angle
#define COS_COMMUTATION_ANGLE	0X4BC // Cosine of Commutation Angle
#define IALPHA					0X4D4 // IALPHA Current Register
#define IBETA					0X4D6 // IBETA Current Register
#define VALPHA					0X4D8 // VALPHA Voltage Register
#define VBETA					0X4DA // VBETA Voltage Register
#define ID						0X4E4 // Measured d-axis Current Register
#define IQ						0X4E6 // Measured q-axis Current Register
#define VD						0X4E8 // VD Voltage Register
#define VQ						0X4EA // VQ Voltage Register
#define IQ_REF_ROTOR_ALIGN		0X524 // Align Current Reference
#define SPEED_REF_OPEN_LOOP		0X53A // Speed at which motor transitions to close 
#define IQ_REF_OPEN_LOOP		0X548 // Open Loop Current Reference
#define SPEED_REF_CLOSED_LOOP	0X5CC // Speed Reference Registers
#define ID_REF_CLOSED_LOOP		0X5FC // Reference for Current Loop Register
#define IQ_REF_CLOSED_LOOP		0X5FE // Reference for Current Loop Register
#define ISD_STATE				0X67A // ISD state Register
#define ISD_SPEED				0X684 // ISD Speed Register
#define IPD_STATE				0X6B8 // IPD state Register
#define IPD_ANGLE				0X6FC // Calculated IPD Angle Register
#define ED						0X742 // Estimated BEMF EQ Register
#define EQ						0X744 // Estimated BEMF ED Register
#define SPEED_FDBK				0X752 // Speed Feedback Register
#define THETA_EST				0X756 // Estimated motor position Register



enum DRV8316_Time {
	ms10 = 0x0,
	ms50 = 0x1,
	ms100 = 0x2,
	ms200 = 0x3,
	ms300 = 0x4,
	ms400 = 0x5,
	ms500 = 0x6,
	ms750 = 0x7,
	ms1000  = 0x8,
	ms2000  = 0x9,
	ms3000  = 0xA,
	ms4000  = 0xB,
	ms5000  = 0xC,
	ms7500  = 0xD,
	ms10000 = 0xE,
	ms15000 = 0xF
};

enum DRV8316_BEMF_Thr {
	mV50 =   0x0,
	mV75 =   0x1,
	mV100 =  0x2,
	mV250 =  0x3,
	mV500 =  0x4,
	mV1000 = 0x5,
	MV1500 = 0x6
};

enum DRV8316_Slew {
    Slew_25Vus = 0x00,
    Slew_50Vus = 0x01,
    Slew_150Vus = 0x02,
    Slew_200Vus = 0x03
};

enum DRV8316_SpeedMode {
    Speed_Mode_i2c = 0x02,
};

enum DRV8316_OVP {
    OVP_SEL_32V = 0x0,
    OVP_SEL_20V = 0x1
};

enum DRV8316_OCPMode {
    Latched_Fault = 0x00,
    AutoRetry_Fault = 0x01,
    ReportOnly = 0x02,
    NoAction = 0x03
};


enum DRV8316_OCPLevel {
    Curr_16A = 0x0,
    Curr_24A = 0x1
};

enum DRV8316_OCPRetry {
    Retry5ms = 0x0,
    Retry500ms = 0x1
};

enum DRV8316_OCPDeglitch {
    Deglitch_0us2 = 0x00,
    Deglitch_0us6 = 0x01,
    Deglitch_1us1 = 0x02,
    Deglitch_1us6 = 0x03
};

enum DRV8316_CSAGain {
    Gain_0V15 = 0x00,
    Gain_0V1875 = 0x01,
    Gain_0V25 = 0x02,
    Gain_0V375 = 0x03
};

enum DRV8316_Recirculation {
    BrakeMode = 0x00, // FETs
    CoastMode = 0x01  // Diodes
};

enum DRV8316_BuckVoltage {
    VB_3V3 = 0x00,
    VB_5V = 0x01,
    VB_4V = 0x02,
    VB_5V7 = 0x03
};

enum DRV8316_BuckCurrentLimit {
    Limit_600mA = 0x00,
    Limit_150mA = 0x01
};


enum DRV8316_DelayTarget {
	Delay_0us = 0x0,
	Delay_0us4 = 0x1,
	Delay_0us6 = 0x2,
	Delay_0us8 = 0x3,
	Delay_1us = 0x4,
	Delay_1us2 = 0x5,
	Delay_1us4 = 0x6,
	Delay_1us6 = 0x7,
	Delay_1us8 = 0x8,
	Delay_2us = 0x9,
	Delay_2us2 = 0xA,
	Delay_2us4 = 0xB,
	Delay_2us6 = 0xC,
	Delay_2us8 = 0xD,
	Delay_3us = 0xE,
	Delay_3us2 = 0xF
};

enum DRV8316_Percentage {
	percent5 =  0x1,
	percent10 = 0x2,
	percent15 = 0x3,
	percent20 = 0x4,
	percent25 = 0x5,
	percent30 = 0x6,
	percent35 = 0x7,
	percent40 = 0x8,
	percent45 = 0x9,
	percent50 = 0xA,
	percent55 = 0xB,
	percent60 = 0xB,
	percent70 = 0xC,
	percent80 = 0xD,
	percent90 = 0xE,
	percent100 = 0xF
};

enum BRK_MODE {
	AllHighSideOn = 0x0,
	AllLowSideOn = 0x1
};

enum Current_Lim {
	mA_125 = 0x0,
	mA_250 = 0x1,
	mA_500 = 0x2,
	mA_1000 = 0x3,
	mA_1500 = 0x4, 
	mA_2000 = 0x5, 
	mA_2500 = 0x6, 
	mA_3000 = 0x7, 
	mA_3500 = 0x8, 
	mA_4000 = 0x9, 
	mA_4500 = 0xA, 
	mA_5000 = 0xB,
	mA_5500 = 0xC, 
	mA_6000 = 0xD, 
	mA_7000 = 0xE,
	mA_8000 = 0xF,   
};

enum DRV8316_Angle {
    angle_0 = 0x0,   // 0°
    angle_10 = 0x1,  // 10°
    angle_20 = 0x2,  // 20°
    angle_30 = 0x3,  // 30°
    angle_45 = 0x4,  // 45°
    angle_60 = 0x5,  // 60°
    angle_70 = 0x6,  // 70°
    angle_80 = 0x7,  // 80°
    angle_90 = 0x8,  // 90°
    angle_110 = 0x9, // 110°
    angle_120 = 0xA, // 120°
    angle_135 = 0xB, // 135°
    angle_150 = 0xC, // 150°
    angle_160 = 0xD, // 160°
    angle_170 = 0xE, // 170°
    angle_180 = 0xF, // 180°
    angle_190 = 0x10, // 190°
    angle_210 = 0x11, // 210°
    angle_225 = 0x12, // 225°
    angle_240 = 0x13, // 240°
    angle_250 = 0x14, // 250°
    angle_260 = 0x15, // 260°
    angle_270 = 0x16, // 270°
    angle_280 = 0x17, // 280°
    angle_290 = 0x18, // 290°
    angle_315 = 0x19, // 315°
    angle_330 = 0x1A, // 330°
    angle_340 = 0x1B, // 340°
    angle_350 = 0x1C, // 350°
};

enum DRV8316_Frequency {
    Hertz_0_5 = 0x0,   // 0.5 Hz/s
    Hertz_1 = 0x1,     // 1 Hz/s
    Hertz_2_5 = 0x2,   // 2.5 Hz/s
    Hertz_5 = 0x3,     // 5 Hz/s
    Hertz_7_5 = 0x4,   // 7.5 Hz/s
    Hertz_10 = 0x5,    // 10 Hz/s
    Hertz_20 = 0x6,    // 20 Hz/s
    Hertz_40 = 0x7,    // 40 Hz/s
    Hertz_60 = 0x8,    // 60 Hz/s
    Hertz_80 = 0x9,    // 80 Hz/s
    Hertz_100 = 0xA,   // 100 Hz/s
    Hertz_200 = 0xB,   // 200 Hz/s
    Hertz_300 = 0xC,   // 300 Hz/s
    Hertz_400 = 0xD,   // 400 Hz/s
    Hertz_500 = 0xE,   // 500 Hz/s
    Hertz_600 = 0xF,   // 600 Hz/s
    Hertz_700 = 0x10,  // 700 Hz/s
    Hertz_800 = 0x11,  // 800 Hz/s
    Hertz_900 = 0x12,  // 900 Hz/s
    Hertz_1000 = 0x13, // 1000 Hz/s
    Hertz_2000 = 0x14, // 2000 Hz/s
    Hertz_4000 = 0x15, // 4000 Hz/s
    Hertz_6000 = 0x16, // 6000 Hz/s
    Hertz_8000 = 0x17, // 8000 Hz/s
    Hertz_10000 = 0x18,// 10000 Hz/s
    Hertz_20000 = 0x19,// 20000 Hz/s
    Hertz_30000 = 0x1A,// 30000 Hz/s
    Hertz_40000 = 0x1B,// 40000 Hz/s
    Hertz_50000 = 0x1C,// 50000 Hz/s
    Hertz_60000 = 0x1D,// 60000 Hz/s
    Hertz_70000 = 0x1E,// 70000 Hz/s
    Hertz_No_Limit = 0x1F // No limit
};

enum DRV8316_PWMFrequency {
    PWM_10kHz = 0x0,
    PWM_15kHz = 0x1,
    PWM_20kHz = 0x2,
    PWM_25kHz = 0x3,
    PWM_30kHz = 0x4,
    PWM_35kHz = 0x5,
    PWM_40kHz = 0x6,
    PWM_45kHz = 0x7,
    PWM_50kHz = 0x8,
    PWM_55kHz = 0x9,
    PWM_60kHz = 0xA,
    PWM_65kHz = 0xB,
    PWM_70kHz = 0xC,
    PWM_75kHz = 0xD,
    PWM_NA = 0xE,
    PWM_No_Limit = 0xF
};





/*--------------------------*/
/* 		EEPROM registers	*/
/*--------------------------*/




/* Register to configure initial speed detect settings */
typedef union {
    struct Registerbits{
        uint8_t REV_DRV_OPEN_LOOP_CURRENT:2; // Open loop current limit during speed reversal
        uint8_t REV_DRV_HANDOFF_THR:4; // Speed threshold used to transition to open loop during reverse deceleration
        uint8_t STAT_DETECT_THR:3; // BEMF threshold to detect if motor is stationary
        uint8_t HIZ_TIME:4; // Hi-z time
        uint8_t BRK_TIME:4; // Brake time 
        uint8_t :4;
        uint8_t BRK_MODE:1; // Brake mode
        uint8_t FW_DRV_RESYN_THR:4; // Minimum speed threshold to resynchronize to close loop (% of MAX_SPEED)
        uint8_t RESYNC_EN:1; // Resynchronization enable
        uint8_t RVS_DR_EN:1; // Reverse drive enable
        uint8_t HIZ_EN:1; // Hi-z enable
        uint8_t BRAKE_EN:1; // Brake enable
        uint8_t ISD_EN:1; // ISD enable
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} ISD_Config;




/*	Register to configure reverse drive settings	*/
typedef union {
    struct Registerbits{
        uint16_t ACTIVE_BRAKE_KI:10; // 10-bit value for active braking loop Ki. Ki = ACTIVE_BRAKE_KI / 2^9
        uint16_t ACTIVE_BRAKE_KP:10; // 10-bit value for active braking loop Kp. Kp = ACTIVE_BRAKE_KP / 2^7
        uint8_t ACTIVE_BRAKE_CURRENT_LIMIT:3; // Bus current limit during active braking
        uint8_t REV_DRV_OPEN_LOOP_ACCEL_A2:4; // Open loop acceleration coefficient A2 during reverse drive
        uint8_t REV_DRV_OPEN_LOOP_ACCEL_A1:4; // Open loop acceleration coefficient A1 during reverse drive
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} RevDriveConfig;




/*	Register to configure motor startup settings1	*/
typedef union {
    struct Registerbits{
        uint8_t REV_DRV_CONFIG:1; // Chooses between forward and reverse drive setting for reverse drive
        uint8_t ACTIVE_BRAKE_EN:1; // Active braking enable
        uint8_t IQ_RAMP_EN:1; // Iq ramp down before transition to close loop
        uint8_t OL_ILIMIT_CONFIG:1; // Open loop current limit configuration
        uint8_t IPD_REPEAT:2; // Number of times IPD is executed
        uint8_t IPD_ADV_ANGLE:2; // IPD advance angle
        uint8_t IPD_RLS_MODE:1; // IPD release mode
        uint8_t IPD_CURR_THR:5; // IPD current threshold
        uint8_t IPD_CLK_FREQ:3; // IPD clock frequency
        uint8_t ALIGN_OR_SLOW_CURRENT_ILIMIT:4; // Align or slow first cycle current limit
        uint8_t ALIGN_TIME:4; // Align time
        uint8_t ALIGN_SLOW_RAMP_RATE:4; // Align, slow first cycle and open loop current ramp rate
        uint8_t MTR_STARTUP:2; // Motor start-up method
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} MotorStartup1;




/*	Register to configure motor startup settings2	*/
typedef union {
    struct Registerbits{
        uint8_t THETA_ERROR_RAMP_RATE:3; // Ramp rate for reducing difference between estimated theta and open loop theta
        uint8_t FIRST_CYCLE_FREQ_SEL:1; // First cycle frequency in open loop for align, double align and IPD startup options
        uint8_t SLOW_FIRST_CYC_FREQ:4; // Frequency of first cycle in close loop startup (% of MAX_SPEED)
        uint8_t ALIGN_ANGLE:5; // Align angle
        uint8_t OPN_CL_HANDOFF_THR:5; // Open to close loop handoff threshold (% of MAX_SPEED)
        uint8_t AUTO_HANDOFF_EN:1; // Auto handoff enable
        uint8_t OL_ACC_A2:4; // Open loop acceleration coefficient A2
        uint8_t OL_ACC_A1:4; // Open loop acceleration coefficient A1
        uint8_t OL_ILIMIT:4; // Open loop current limit
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} MotorStartup2;




/*	Register to configure close loop settings1	*/
typedef union {
    struct Registerbits{
        uint8_t LOW_SPEED_RECIRC_BRAKE_EN:1; // Stop mode applied when stop mode is recirculation brake and motor running in align or open loop
        uint8_t SPEED_LOOP_DIS:1; // Speed loop disable
        uint8_t DEADTIME_COMP_EN:1; // Deadtime compensation enable
        uint8_t AVS_EN:1; // AVS enable
        uint8_t FG_BEMF_THR:3; // FG output BEMF threshold
        uint8_t FG_CONFIG:1; // FG output configuration
        uint8_t FG_DIV:4; // FG division factor
        uint8_t FG_SEL:2; // FG select
        uint8_t PWM_MODE:1; // PWM modulation
        uint8_t PWM_FREQ_OUT:4; // Output PWM switching frequency
        uint8_t CL_DEC:5; // Closed loop deceleration
        uint8_t CL_DEC_CONFIG:1; // Closed loop deceleration configuration
        uint8_t CL_ACC:5; // Closed loop acceleration
        uint8_t OVERMODULATION_ENABLE:1; // Overmodulation enable
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} ClosedLoop1;




/*	Register to configure close loop settings2	*/
typedef union {
    struct Registerbits{
        uint8_t MOTOR_IND:8; // 8-bit values for motor phase inductance
        uint8_t MOTOR_RES:8; // 8-bit values for motor phase resistance
        uint8_t BRAKE_SPEED_THRESHOLD:4; // Speed threshold for BRAKE pin and motor stop options
        uint8_t ACT_SPIN_THR:4; // Speed threshold for active spin down (% of MAX_SPEED)
        uint8_t MTR_STOP_BRK_TIME:4; // Brake time during motor stop
        uint8_t MTR_STOP:3; // Motor stop method
        uint8_t PARITY:1; // 
    } bits;
    uint32_t reg;
} ClosedLoop2;




/*	Register to configure close loop settings3	*/
typedef union {
    struct Registerbits{
        uint8_t SPD_LOOP_KP_3MSB:3; // 3 MSB bits for speed loop Kp
        uint16_t CURR_LOOP_KI:10; // 10-bit value for current Iq and Id loop Ki
        uint16_t CURR_LOOP_KP:10; // 10-bit value for current Iq and Id loop Kp
        uint8_t MOTOR_BEMF_CONST:8; // 8-bit values for motor BEMF constant
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} ClosedLoop3;




/*	Register to configure close loop settings4	*/
typedef union {
    struct Registerbits{
        uint16_t MAX_SPEED:14; // 14-bit value for setting maximum value of speed in electrical Hz
        uint16_t SPD_LOOP_KI:10; // 10-bit value for speed loop Ki
        uint8_t SPD_LOOP_KP_7LSB:7; // 7 LSB bits for speed loop Kp
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} ClosedLoop4;





/*	Register to configure speed profile1	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; // 
		uint32_t :31; // 
	} bits;
	uint32_t reg;
} SpeedProfiles1;


/*	Register to configure speed profile2	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; //
		uint32_t :31; // 

	} bits;
	uint32_t reg;
} SpeedProfiles2;


/*	Register to configure speed profile3	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; //
		uint32_t :31; // 
	} bits;
	uint32_t reg;
} SpeedProfiles3;


/*	Register to configure speed profile4	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; //
		uint32_t :31; // 
	} bits;
	uint32_t reg;
} SpeedProfiles4;


/*	Register to configure speed profile5	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; //
		uint32_t :31; // 
	} bits;
	uint32_t reg;
} SpeedProfiles5;


/*	Register to configure speed profile6	*/
typedef union {
	struct Registerbits{
		uint8_t PARITY:1; //
		uint32_t :31; // 
	} bits;
	uint32_t reg;
} SpeedProfiles6;





typedef union {
    struct Registerbits{
        uint8_t SATURATION_FLAGS_EN:1; // Enables indication of current loop and speed loop saturation
        uint8_t IPD_FREQ_FAULT_EN:1; // IPD frequency fault enable
        uint8_t IPD_TIMEOUT_FAULT_EN:1; // IPD timeout fault enable
        uint8_t MTR_LCK_MODE:4; // Motor Lock Mode
        uint8_t LCK_RETRY:4; // Lock detection retry time
        uint8_t LOCK_ILIMIT_DEG:4; // Lock detection current limit deglitch time
        uint8_t LOCK_ILIMIT_MODE:4; // Lock current limit mode
        uint8_t LOCK_ILIMIT:4; // ADC based lock detection current threshold
        uint8_t HW_LOCK_ILIMIT:4; // Comparator based lock detection current limit
        uint8_t ILIMIT:4; // Reference for torque PI loop
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} FaultConfig1;



typedef union {
    struct Registerbits{
        uint8_t AUTO_RETRY_TIMES:3; // Automatic retry attempts
        uint8_t MAX_VM_MODE:1; // Overvoltage fault mode
        uint8_t MAX_VM_MOTOR:3; // Maximum voltage for running motor
        uint8_t MIN_VM_MODE:1; // Undervoltage fault mode
        uint8_t MIN_VM_MOTOR:3; // Minimum voltage for running motor
        uint8_t HW_LOCK_ILIMIT_DEG:4; // Hardware lock detection current limit deglitch time
        uint8_t HW_LOCK_ILIMIT_MODE:4; // Hardware lock detection current mode
        uint8_t NO_MTR_THR:3; // No motor lock threshold
        uint8_t ABNORMAL_BEMF_THR:3; // Abnormal BEMF lock threshold (% of expected BEMF)
        uint8_t LOCK_ABN_SPEED:3; // Abnormal speed lock threshold (% of MAX_SPEED)
        uint8_t LOCK3_EN:1; // Lock 3 : No motor enable
        uint8_t LOCK2_EN:1; // Lock 2 : Abnormal BEMF enable
        uint8_t LOCK1_EN:1; // Lock 1 : Abnormal speed enable
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} FaultConfig2;





typedef union {
    struct Registerbits{
        uint8_t SPEED_MODE:2; // Configure speed control mode from speed pin
        uint8_t BRAKE_INPUT:2; // Brake pin override
        uint8_t ALIGN_BRAKE_ANGLE_SEL:1; // Align brake angle select
        uint8_t BRAKE_PIN_MODE:1; // Brake pin mode
        uint32_t :25; // 
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} PinConfig;




/*	Register to configure device	*/
typedef union {
    struct Registerbits{
        uint8_t BUS_VOLT:2; // Maximum bus voltage configuration
        uint32_t :18; // 
        uint8_t I2C_TARGET_ADDR:7; // I2C target address
        uint8_t :1; // 
        uint8_t PIN_38_CONFIG:2; // Pin 38 configuration
        uint8_t :1; // 
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} DeviceConfig1;



/*	Register to configure device	*/
typedef union {
    struct Registerbits{
        uint8_t EXT_WD_FAULT:1; // External watchdog fault mode
        uint8_t EXT_WD_INPUT:1; // External watchdog input mode
        uint8_t EXT_WD_CONFIG:2; // Time between watchdog tickles
        uint8_t EXT_WD_EN:1; // External watchdog enable
        uint8_t EXT_CLK_CONFIG:3; // External clock configuration
        uint8_t EXT_CLK_EN:1; // External clock mode enable
        uint8_t CLK_SEL:2; // Clock source
        uint8_t DEV_MODE:1; // Device mode select
        uint8_t DYNAMIC_VOLTAGE_GAIN_EN:1; // Adjust voltage gain at 1ms rate for optimal voltage resolution
        uint8_t DYNAMIC_CSA_GAIN_EN:1; // Adjust CSA gain at 1ms rate for optimal current resolution
        uint8_t SLEEP_ENTRY_TIME:2; // Device enters sleep mode when speed input is held continuously below the speed threshold for SLEEP_ENTRY_TIME
        uint16_t INPUT_MAXIMUM_FREQ:15; // Input frequency on speed pin for speed control mode
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} DeviceConfig2;





/*	Register to peripheral1		*/
typedef union {
    struct Registerbits{
        uint8_t :8; // 
        uint8_t ALARM_PIN_DIS:1; // Alarm pin disable
        uint8_t SPEED_RANGE_SEL:1; // Speed range selection for digital speed
        uint8_t ACTIVE_BRAKE_MOD_INDEX_LIMIT:3; // Modulation index limit beyond which active braking will be applied
        uint8_t ACTIVE_BRAKE_SPEED_DELTA_LIMIT:4; // Difference between final speed and present speed beyond which active braking will be applied
        uint8_t SELF_TEST_ENABLE:1; // Self-test on power up enable
        uint8_t DIR_CHANGE_MODE:1; // Response to change of DIR pin status
        uint8_t DIR_INPUT:2; // DIR pin override
        uint8_t BUS_CURRENT_LIMIT_ENABLE:1; // Bus current limit enable
        uint8_t BUS_CURRENT_LIMIT:4; // Bus current limit
        uint8_t :4; // 
        uint8_t SPREAD_SPECTRUM_MODULATION_DIS:1; // Spread spectrum modulation disable
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} PeriConfig1;





/*	Register to configure gated driver settings1	*/
typedef union {
    struct Registerbits{
        uint8_t CSA_GAIN:2; // Current Sense Amplifier (CSA) gain
        uint8_t :6; //
        uint8_t OCP_MODE:2; // OCP fault mode
        uint8_t OCP_LVL:1; // OCP level
        uint8_t OCP_RETRY:1; // OCP retry time
        uint8_t OCP_DEG:2; // OCP deglitch time
        uint8_t :2; // 
        uint8_t OTW_REP:1; //  Overtemperature warning reporting on nFAULT
        uint8_t :1; // 
        uint8_t OVP_EN:1; // Overvoltage protection enable
        uint8_t OVP_SEL:1; // Overvoltage protection level
        uint8_t :6; // 
        uint8_t SLEW_RATE:2; // Slew rate
        uint8_t :3; // 
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} GdConfig1;



/*	Register to configure gated driver settings2	*/
typedef union {
    struct Registerbits{
        uint32_t :20; // 
        uint8_t BUCK_DIS:1; // Buck disable
        uint8_t BUCK_SEL:2; // Buck voltage selection
        uint8_t BUCK_CL:1; // Buck current limit
        uint8_t BUCK_PS_DIS:1; // Buck power sequencing disable
        uint8_t BUCK_SR:1; // Buck slew rate
        uint8_t TARGET_DELAY:4; // Target delay
        uint8_t DELAY_COMP_EN:1; // Driver delay compensation enable
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} GdConfig2;





/*	Register to configure internal algorithm parameters1	*/
typedef union {
    struct Registerbits{
        uint8_t REV_DRV_OPEN_LOOP_DEC:3; // % of open loop acceleration to be applied during open loop deceleration in reverse drive
        uint8_t MPET_OPEN_LOOP_SLEW_RATE:3; // Open loop slew rate for MPET (Hz/s)
        uint8_t MPET_OPEN_LOOP_SPEED_REF:2; // Open loop speed reference for MPET (% of MAXIMUM_SPEED)
        uint8_t MPET_OPEN_LOOP_CURRENT_REF:3; // Open loop current reference
        uint8_t MPET_IPD_FREQ:2; // Number of times IPD is executed for MPET
        uint8_t MPET_IPD_CURRENT_LIMIT:2; // IPD current limit for MPET
        uint8_t :2; // 
        uint8_t AUTO_HANDOFF_MIN_BEMF:3; // Minimum BEMF for handoff
        uint8_t ISD_TIMEOUT:2; // Timeout in case ISD is unable to reliably detect speed or direction
        uint8_t ISD_RUN_TIME:2; // Persistence time for declaring motor is running
        uint8_t ISD_STOP_TIME:2; // Persistence time for declaring motor has stopped
        uint8_t FAST_ISD_EN:1; // Fast initial speed detection enable
        uint8_t SPEED_PIN_GLITCH_FILTER:2; // Glitch filter applied on speed pin input
        uint8_t FG_ANGLE_INTERPOLATE_EN:1; // Angle interpolation for FG enable
        uint8_t :1; // 
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} IntAlgo1;


/*	Register to configure internal algorithm parameters2	*/
typedef union {
    struct Registerbits{
        uint8_t IPD_HIGH_RESOLUTION_EN:1; // IPD high resolution enable
        uint8_t MPET_KE_MEAS_PARAMETER_SELECT:1; // Selection between MPET_OPEN_LOOP_SLEW_RATE for slew rate
        uint8_t MPET_IPD_SELECT:1; // Selection between MPET_IPD_CURRENT_LIMIT for IPD current limit
        uint8_t ACTIVE_BRAKE_BUS_CURRENT_SLEW_RATE:3; // Bus current slew rate during active braking
        uint8_t CL_SLOW_ACC:4; // Close loop acceleration when estimator is not yet fully aligned
        uint32_t :21; // 
        uint8_t PARITY:1; //
    } bits;
    uint32_t reg;
} IntAlgo2;







/*--------------------------*/
/* 		RAM registers		*/
/*--------------------------*/




/*	Status of various gate driver faults 	*/
typedef union {
    struct Registerbits{
        uint16_t :11; // 
        uint8_t VCP_UV:1; // Charge pump undervoltage status
        uint8_t BUCK_UV:1; // Buck regulator undervoltage status
        uint8_t BUCK_OCP:1; // Buck regulator overcurrent status
        uint8_t OTP_ERR:1; // One-time programmable (OTP) error
        uint8_t :1; //
        uint8_t OCP_LA:1; // Overcurrent status on low-side switch of OUTA
        uint8_t OCP_HA:1; // Overcurrent status on high-side switch of OUTA
        uint8_t OCP_LB:1; // Overcurrent status on low-side switch of OUTB
        uint8_t OCP_HB:1; // Overcurrent status on high-side switch of OUTB
        uint8_t OCP_LC:1; // Overcurrent status on low-side switch of OUTC
        uint8_t OCP_HC:1; // Overcurrent status on high-side switch of OUTC
        uint8_t OTS:1; // Overtemperature shutdown status
        uint8_t OTW:1; // Overtemperature warning status
        uint8_t :1; // 
        uint8_t OT:1; // Overtemperature fault status
        uint8_t OVP:1; // Supply overvoltage protection status
        uint8_t NPOR:1; // Supply power on reset
        uint8_t OCP:1; // Overcurrent protection status
        uint8_t :1; // 
        uint8_t :1; // 
        uint8_t BK_FLT:1; // Buck fault
        uint8_t DRIVER_FAULT:1; // Logic OR of driver fault registers
    } bits;
    uint32_t reg;
} GateDriverFaultStatus;




/*	Status of various controller faults 	*/
typedef union {
    struct Registerbits{
        uint16_t :14; // 
        uint8_t CURRENT_LOOP_SATURATION:1; // 
        uint8_t SPEED_LOOP_SATURATION:1; // 
        uint8_t MTR_OVER_VOLTAGE:1; // 
        uint8_t MTR_UNDER_VOLTAGE:1; // 
        uint8_t HW_LOCK_ILIMIT:1; // 
        uint8_t LOCK_ILIMIT:1; // 
        uint8_t MTR_LCK:1; // 
        uint8_t NO_MTR:1; // 
        uint8_t ABN_BEMF:1; // 
        uint8_t ABN_SPEED:1; // 
        uint8_t MPET_BEMF_FAULT:1; // 
        uint8_t MPET_IPD_FAULT:1; // 
        uint8_t BUS_CURRENT_LIMIT_STATUS:1; // 
        uint8_t IPD_T2_FAULT:1; // 
        uint8_t IPD_T1_FAULT:1; // 
        uint8_t IPD_FREQ_FAULT:1; // 
        uint8_t :1; // 
        uint8_t CONTROLLER_FAULT:1; // Logic OR of controller fault status registers
    } bits;
    uint32_t reg;
} ControllerFaultStatus;




/*	Status of various system and algorithm parameters	*/
typedef union {
    struct Registerbits{
        uint16_t :16; // 
        uint16_t VOLT_MAG:16; // 16-bit value indicating output voltage magnitude
    } bits;
    uint32_t reg;
} AlgoStatus;




/*	Status of various motor parameters	*/
typedef union {
    struct Registerbits{
        uint8_t :8; // 
        uint8_t MOTOR_L:8; // 8-bit value indicating measured motor inductance
        uint8_t MOTOR_BEMF_CONST:8; // 8-bit value indicating measured BEMF constant
        uint8_t MOTOR_R:8; // 8-bit value indicating measured motor resistance
    } bits;
    uint32_t reg;
} MtrParams;




/*	Status of various MPET parameters	*/ 
typedef union {
    struct Registerbits{
        uint32_t :24; // 
        uint8_t MPET_PWM_FREQ:4; // 4-bit value indicating MPET recommended PWM switching frequency based on electrical time constant
        uint8_t MPET_MECH_STATUS:1; // Indicates status of mechanical parameter measurement
        uint8_t MPET_KE_STATUS:1; // Indicates status of BEMF constant measurement
        uint8_t MPET_L_STATUS:1; // Indicates status of inductance measurement
        uint8_t MPET_R_STATUS:1; // Indicates status of resistance measurement
    } bits;
    uint32_t reg;
} AlgoStatusMpet;





/*	Register for device control 	*/
typedef union {
    struct Registerbits {
        uint16_t :10; // 
        uint8_t WATCHDOG_TICKLE:1; // RAM bit to tickle watchdog in I2C mode
        uint16_t FORCED_ALIGN_ANGLE:9; // 9-bit value (in °) used during forced align state
        uint8_t EEPROM_WRITE_ACCESS_KEY:8; // EEPROM write access key
        uint8_t CLR_FLT_RETRY_COUNT:1; // Clears fault retry count
        uint8_t CLR_FLT:1; // Clears all faults
        uint8_t EEPROM_READ:1; // Read the default configuration from EEPROM 
        uint8_t EEPROM_WRT:1; // Write the configuration to EEPROM
    } bits;
    uint32_t reg;
} DevCtrl;



/*	Algorithm control register for debug	*/
typedef union {
	struct Registerbits{
		uint16_t FORCE_IQ_REF_SPEED_LOOP_DIS:10; // Sets Iq_ref when speed loop is disabled If SPEED_LOOP_DIS = 1b
		uint16_t FORCE_ALIGN_ANGLE_SRC_SEL:1; // Force align angle state source select
		uint16_t FORCE_ISD_EN:1; // Force ISD enable
		uint16_t FORCE_IPD_EN:1; // Force IPD enable
		uint16_t FORCE_SLOW_FIRST_CYCLE_EN:1; // Force slow first cycle enable
		uint16_t FORCE_ALIGN_EN:1; // Force align state enable
		uint16_t CLOSED_LOOP_DIS:1; // Use to disable closed loop
		uint16_t DIGITAL_SPEED_CTRL:15; // Digital speed control If OVERRIDE = 1b, then SPEED_CMD is control using DIGITAL_SPEED_CTRL
		uint16_t OVERRIDE:1; // Use to control the SPD_CTRL bits. If OVERRIDE = 1b, speed command can be written by the user through serial interface.	
	} bits;
	uint32_t reg;
} AlgoCtrl1;



/*	Algorithm control register for debug	*/
typedef union {
    struct Registerbits{
        uint8_t MPET_WRITE_SHADOW:1; // Write measured parameters to shadow register when set to 1b
        uint8_t MPET_MECH:1; // Enables motor mechanical parameter measurement during motor parameter measurement routine
        uint8_t MPET_KE:1; // Enables motor BEMF constant measurement during motor parameter measurement routine
        uint8_t MPET_L:1; // Enables motor inductance measurement during motor parameter measurement routine
        uint8_t MPET_R:1; // Enables motor resistance measurement during motor parameter measurement routine
        uint8_t MPET_CMD:1; // Initiates motor parameter measurement routine when set to 1b
        uint16_t FORCE_VQ_CURRENT_LOOP_DIS:10; // Sets Vq_ref when current loop speed loop are disabled If CURRENT_LOOP_DIS = 1b
        uint16_t FORCE_VD_CURRENT_LOOP_DIS:10; // Sets Vd_ref when current loop and speed loop are disabled If CURRENT_LOOP_DIS = 1b
        uint8_t CURRENT_LOOP_DIS:1; // Use to control the FORCE_VD_CURRENT_LOOP_DIS
        uint8_t :5; // 
    } bits;
    uint32_t reg;
} AlgoCtrl2;



/*	Current PI controller used	*/
typedef union {
    struct Registerbits{
        uint16_t :12; // 
        uint16_t CURRENT_LOOP_KI:10; // 10-bit value for current loop Ki; same scaling as CURR_LOOP_KI
        uint16_t CURRENT_LOOP_KP:10; // 10-bit value for current loop Kp; same scaling as CURR_LOOP_KP
    } bits;
    uint32_t reg;
} CurrentPI;



/*	Speed PI controller used	*/
typedef union {
    struct Registerbits{
        uint16_t :12; // 
        uint16_t SPEED_LOOP_KI:10; // 10-bit value for speed loop Ki; same scaling as SPD_LOOP_KI
        uint16_t SPEED_LOOP_KP:10; // 10-bit value for speed loo Kp; same scaling as SPD_LOOP_KP
    } bits;
    uint32_t reg;
} SpeedPI;


#endif
