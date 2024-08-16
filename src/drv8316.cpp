#include "drv8316.h"

// Define the static members outside the class
uint32_t DRV8316Driver::i2c;
uint16_t DRV8316Driver::address;


static bool lock = false;
#define I2C_DELAY       50 // 30
#define READ            true
#define WRITE           false

DigIo gpio;

static void BitBangI2CStartAddress(uint32_t i2c, uint8_t address)
{	
	if (i2c == I2C1)
	{
		DigIo::i2c1_sda.Clear();
		for (volatile int i = 0; i < I2C_DELAY*2; i++);
		DigIo::i2c1_scl.Clear();

		for (int i = 16; i >= 0; i--)
		{
		   if (address & 0x80 || i < 1) DigIo::i2c1_sda.Set();
		   else DigIo::i2c1_sda.Clear();
		   for (volatile int j = 0; j < I2C_DELAY; j++);
		   DigIo::i2c1_scl.Toggle();;
		   if (i & 1) address <<= 1;
		}
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c1_sda.Set();
	}
	if (i2c == I2C2)
	{
		DigIo::i2c2_sda.Clear();
		for (volatile int i = 0; i < I2C_DELAY*2; i++);
		DigIo::i2c2_scl.Clear();

		for (int i = 16; i >= 0; i--)
		{
		   if (address & 0x80 || i < 1) DigIo::i2c2_sda.Set();
		   else DigIo::i2c2_sda.Clear();
		   for (volatile int j = 0; j < I2C_DELAY; j++);
		   DigIo::i2c2_scl.Toggle();;
		   if (i & 1) address <<= 1;
		}
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c2_sda.Set();
	}
}
	

static uint8_t BitBangI2CByte(uint32_t i2c, uint8_t byte, bool ack)
{
 	uint8_t byteRead = 0;

	if (i2c == I2C1)	
	{
		DigIo::i2c1_scl.Clear();
 		for (volatile int i = 0; i < I2C_DELAY; i++);

 		for (int i = 16; i >= 0; i--)
 		{	// byte & 0x80 checks if MSB=1
 		   	if (byte & 0x80 || (i < 1 && !ack)) DigIo::i2c1_sda.Set();
 		   	else DigIo::i2c1_sda.Clear();
			
 		   	for (volatile int j = 0; j < I2C_DELAY; j++);
 		   	DigIo::i2c1_scl.Toggle();;
 		   	if (i & 1) {
				byte <<= 1;
				byteRead <<= 1;
				byteRead |= DigIo::i2c1_sda.Get();
 		   	}
 		}
 		for (volatile int i = 0; i < I2C_DELAY; i++);
	}

	if (i2c == I2C2)	
	{
		DigIo::i2c2_scl.Clear();
 		for (volatile int i = 0; i < I2C_DELAY; i++);
	
 		for (int i = 16; i >= 0; i--)
 		{
			if (byte & 0x80 || (i < 1 && !ack)) DigIo::i2c2_sda.Set();
			else DigIo::i2c2_sda.Clear();
			for (volatile int j = 0; j < I2C_DELAY; j++);
			DigIo::i2c2_scl.Toggle();;
			if (i & 1) {
				byte <<= 1;
				byteRead <<= 1;
				byteRead |= DigIo::i2c2_sda.Get();
			}
 		}
 		for (volatile int i = 0; i < I2C_DELAY; i++);
	}
 	
 	return byteRead;
}


static void BitBangI2CStop(uint32_t i2c)
{
	if (i2c == I2C1)
	{
		DigIo::i2c1_scl.Clear();
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c1_sda.Clear(); //data low
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c1_scl.Set();
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c1_sda.Set(); //data high -> STOP
		for (volatile int i = 0; i < I2C_DELAY; i++);
	}
	if (i2c == I2C2)
	{
		DigIo::i2c2_scl.Clear();
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c2_sda.Clear(); //data low
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c2_scl.Set();
		for (volatile int i = 0; i < I2C_DELAY; i++);
		DigIo::i2c2_sda.Set(); //data high -> STOP
		for (volatile int i = 0; i < I2C_DELAY; i++);
	}
}



static void BitBangI2CTransfer(uint32_t i2c, uint8_t address, uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
	if (lock) return;

	lock = true;
	address <<= 1;

	if (wn) // write
	{
		address |= WRITE;
		BitBangI2CStartAddress(i2c, address);
		delay_us(100); // required according to datasheet

		for (int i = 0; i < wn; i++)
		{
			w[i] = BitBangI2CByte(i2c, w[i], i != (rn - 1) && WRITE);
			delay_us(100); // required according to datasheet
		}
		BitBangI2CStop(i2c);
	}

	if (rn) // read
	{
		address |= READ;
		delay_us(500); 
		BitBangI2CStartAddress(i2c, address);
		delay_us(100); // required according to datasheet


		for (int i = 0; i < rn; i++)
		{
			r[i] = BitBangI2CByte(i2c, 0xff, i != (rn - 1) && READ); // ack on read
			delay_us(100); // required according to datasheet
		}
		BitBangI2CStop(i2c);
	}

	

	lock = false;
}





union controlWord { // top of struct is lsb
    struct RegisterBits {
        uint16_t MEM_ADDR:12;
        uint8_t MEM_PAGE:4;
        uint8_t MEM_SEC:4;
        uint8_t DLEN:2;
        uint8_t CRC_EN:1;
        uint8_t OP_RW:1;
        uint8_t :8; // Padding
    } bits;
    uint8_t bytes[4];
};





uint32_t DRV8316Driver::write_i2c(uint16_t addr, uint32_t value)  
{
	//uint8_t* reg_bytes = (uint8_t*)value;// this converts the two 32bit array into bytes.

	// Set values using bit fields
	controlWord word;
    word.bits.OP_RW = 0x0;      // 0x0=write    0x1=read
    word.bits.CRC_EN = 0x0;     // 0x0=off      0x1=crc
    word.bits.DLEN = 0x01;       // 0x00=16bit   0x01=32bit   0x02=64bit
    word.bits.MEM_SEC = 0x0;
    word.bits.MEM_PAGE = 0x0;
    word.bits.MEM_ADDR = addr; // 0xEC

	uint8_t reg_bytes[4];
    for (int i = 0; i < 4; ++i) {
        reg_bytes[i] = (value >> (i * 8)) & 0xFF;
    }
	
	uint8_t message[7] = 
	{
		word.bytes[2], 
		word.bytes[1],
		word.bytes[0],
		reg_bytes[3], // Databyte 3
		reg_bytes[2], // Databyte 2	
		reg_bytes[1], // Databyte 1	
		reg_bytes[0]  // Databyte 0	
	};

	uint8_t result[4];

	BitBangI2CTransfer(I2C1, TARGET_ID, message, 7, result, 0);

	delay_ms(1);

	uint32_t convertedValue = *(reinterpret_cast<uint32_t*>(result)); 
	return convertedValue;
}




uint32_t DRV8316Driver::read_i2c(uint16_t addr)   							
{
	// Set values using bit fields
	controlWord word;
    word.bits.OP_RW = 0x1;      // 0x0=write    0x1=read
    word.bits.CRC_EN = 0x0;     // 0x0=off      0x1=crc
    word.bits.DLEN = 0x1;       // 0x00=16bit   0x01=32bit   0x02=64bit
    word.bits.MEM_SEC = 0x0;
    word.bits.MEM_PAGE = 0x0;
    word.bits.MEM_ADDR = addr;

	uint8_t message[3] = 
	{
		// skip word.bytes[3] because control word is only 24bits
		word.bytes[2], 
		word.bytes[1],
		word.bytes[0],
	};

	uint8_t result[4];

	BitBangI2CTransfer(I2C1, TARGET_ID, message, 3, result, 4);

	delay_ms(1); //

	uint32_t convertedValue = *(reinterpret_cast<uint32_t*>(result)); 
	return convertedValue;
}




/*
	EEPROM read procedure is as follows
	1. Write 0x40000000 into register 0x0000EA to read the EEPROM data into the shadow registers 
	(0x000080-0x0000AE).
	2. Wait for 100ms for the EEPROM read operation to complete.
	3. Read any register

	EEPROM write procedure is as follows
	1. Write any register
	2. Write 0x8A500000 into register 0x0000EA to write the shadow register(0x000080-0x0000AE) values into the EEPROM.
	3. Wait for 100ms for the EEPROM write operation to complete
*/
void DRV8316Driver::saveToEEPROM() // unfinished
{
	controlWord word = {
		word.bits.OP_RW = 0x0,       // 0x0=write    0x1=read
		word.bits.CRC_EN = 0x0,      // 0x0=off      0x1=crc
		word.bits.DLEN = 0x01,       // 0x00=16bit   0x01=32bit   0x02=64bit
		word.bits.MEM_SEC = 0x0,     
		word.bits.MEM_PAGE = 0x0,    
		word.bits.MEM_ADDR = 0xEA // full address is 0x0000EA
	};
	
	uint8_t message[7] = 
	{
		// skip bytes[0] because control word is only 24bits
		word.bytes[1], 
		word.bytes[2],
		word.bytes[3],
		0x00,	
		0x00,	
		0x50,	
		0x8A, // full value is 0x8A500000
	};

	uint8_t result; 

	i2c_send_start(I2C1);
	i2c_transfer7(I2C1, TARGET_ID, message, 7, &result, 0);
	i2c_send_stop(I2C1);

	
}


	





/*------------------------------*/
/* 		DRV8316Configuration	*/
/*------------------------------*/






uint32_t DRV8316Configuration::setIsdEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	return result;
//	ISD_Config data;
//	data.reg = result;
//	data.bits.ISD_EN = enable;
//	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setBrakeEnable(bool mode)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.BRAKE_EN = mode;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setBrakeTime(DRV8316_Time time)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.BRK_TIME = time;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setHizEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.HIZ_EN = enable;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setHizTime(DRV8316_Time time)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.HIZ_TIME = time;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setReverseEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.RVS_DR_EN = enable;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setResyncEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.RESYNC_EN = enable;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setResyncThreshold(DRV8316_Percentage percentage)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.FW_DRV_RESYN_THR = percentage;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setBemfStationaryThreshold(DRV8316_BEMF_Thr voltage)
{
	uint32_t result = DRV8316Driver::read_i2c(ISD_CONFIG);
	ISD_Config data;
	data.reg = result;
	data.bits.STAT_DETECT_THR = voltage;
	result = DRV8316Driver::write_i2c(ISD_CONFIG, data.reg);
}

void DRV8316Configuration::setActiveBrakeEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(MOTOR_STARTUP1);
	MotorStartup1 data;
	data.reg = result;
	data.bits.ACTIVE_BRAKE_EN = enable;
	result = DRV8316Driver::write_i2c(MOTOR_STARTUP1, data.reg);
}

void DRV8316Configuration::setOpenLoopCurrentLimit(Current_Lim current) // unfinished
{
	uint32_t result = DRV8316Driver::read_i2c(MOTOR_STARTUP2);
	MotorStartup2 data;
	data.reg = result;
	data.bits.OL_ILIMIT = current;
	result = DRV8316Driver::write_i2c(MOTOR_STARTUP2, data.reg);
}

void DRV8316Configuration::setAlignAngle(DRV8316_Angle degrees)
{
	uint32_t result = DRV8316Driver::read_i2c(MOTOR_STARTUP2);
	MotorStartup2 data;
	data.reg = result;
	data.bits.ALIGN_ANGLE = degrees;
	result = DRV8316Driver::write_i2c(MOTOR_STARTUP2, data.reg);
}

void DRV8316Configuration::setOvermodulationEnable(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(CLOSED_LOOP1);
	ClosedLoop1 data;
	data.reg = result;
	data.bits.OVERMODULATION_ENABLE = enable;
	result = DRV8316Driver::write_i2c(CLOSED_LOOP1, data.reg);
}

void DRV8316Configuration::setClosedLoopAccel(DRV8316_Frequency dps)
{
	uint32_t result = DRV8316Driver::read_i2c(CLOSED_LOOP1);
	ClosedLoop1 data;
	data.reg = result;
	data.bits.CL_ACC = dps;
	result = DRV8316Driver::write_i2c(CLOSED_LOOP1, data.reg);
}

void DRV8316Configuration::setClosedLoopDecel(DRV8316_Frequency dps)
{
	uint32_t result = DRV8316Driver::read_i2c(CLOSED_LOOP1);
	ClosedLoop1 data;
	data.reg = result;
	data.bits.CL_DEC = dps;
	result = DRV8316Driver::write_i2c(CLOSED_LOOP1, data.reg);
}

void DRV8316Configuration::setPWMfreq(DRV8316_PWMFrequency khz)
{
	uint32_t result = DRV8316Driver::read_i2c(CLOSED_LOOP1);
	ClosedLoop1 data;
	data.reg = result;
	data.bits.PWM_FREQ_OUT = khz;
	result = DRV8316Driver::write_i2c(CLOSED_LOOP1, data.reg);
}

void DRV8316Configuration::setPWMmode(uint8_t mode)
{
	
}

void DRV8316Configuration::setAvsEnable(bool enable)
{
	
}

void DRV8316Configuration::setDeadtimeCompensationEnable(bool enable)
{
	
}

void DRV8316Configuration::setSpeedloopDisable(bool enable)
{
	
}

void DRV8316Configuration::setMotorStartupMethod(uint8_t method)
{
	
}

void DRV8316Configuration::setMotorStopMethod(uint8_t method)
{
	
}

void DRV8316Configuration::setMotorStopTime(uint8_t time)
{
	
}

void DRV8316Configuration::setMotorMaxSpeed(uint16_t hertz)
{
	ClosedLoop4 data;
	data.reg = DRV8316Driver::read_i2c(CLOSED_LOOP4);
	data.bits.MAX_SPEED = hertz;
	DRV8316Driver::write_i2c(CLOSED_LOOP4, data.reg);
}

void DRV8316Configuration::setMotorResistance(uint8_t ohm)
{
	
}

void DRV8316Configuration::setMotorInductance(uint8_t henry)
{
	
}

void DRV8316Configuration::setSpeedProfileConfig(uint8_t mode)
{
	
}

void DRV8316Configuration::setBrakePinMode(uint8_t mode)
{
	
}

void DRV8316Configuration::setBrakeInput(uint8_t input)
{
	
}

void DRV8316Configuration::setSpeedMode(uint8_t mode)
{
	
}

void DRV8316Configuration::setPin38Config(uint8_t config)
{
	
}

void DRV8316Configuration::setI2C_Address(uint8_t address)
{
	uint32_t result = DRV8316Driver::read_i2c(DEVICE_CONFIG1);
	DeviceConfig1 data;
	data.reg = result;
	data.bits.I2C_TARGET_ADDR = address;
	result = DRV8316Driver::write_i2c(DEVICE_CONFIG1, data.reg);
}

void DRV8316Configuration::setMaxVoltage(uint8_t voltage)
{
	
}

void DRV8316Configuration::setMaxInputFreq(uint16_t freq)
{
	
}

void DRV8316Configuration::setSleepEntryTime(uint8_t time)
{
	
}

void DRV8316Configuration::setDeviceMode(uint8_t mode)
{
	
}

void DRV8316Configuration::setClockSource(uint8_t source)
{
	
}

void DRV8316Configuration::setExtClockEnable(bool enable)
{
	
}

void DRV8316Configuration::setExtClockFreq(uint8_t freq)
{
	
}

void DRV8316Configuration::setExtWDenable(bool enable)
{
	
}

void DRV8316Configuration::setExtWDinterval(uint8_t interval)
{
	
}

void DRV8316Configuration::setExtWDfaultMode(uint8_t mode)
{
	
}

void DRV8316Configuration::setCurrentLimitEnable(bool enable)
{
	
}

void DRV8316Configuration::setDirInput(uint8_t input)
{
	
}

void DRV8316Configuration::setSelfTestEnable(bool enable)
{
	
}

void DRV8316Configuration::setSpeedRange(uint8_t range)
{
	
}

void DRV8316Configuration::setSlewRate(DRV8316_Slew slewRate)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.SLEW_RATE = slewRate;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOvervoltageLevel(DRV8316_OVP voltage)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OVP_SEL = voltage;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOvervoltageProtection(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OVP_EN = enable;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOvertemperatureReporting(bool reportFault)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OTW_REP = reportFault;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOCPDeglitchTime(DRV8316_OCPDeglitch ms)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OCP_DEG = ms;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOCPRetryTime(DRV8316_OCPRetry ms)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OCP_RETRY = ms;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOCPLevel(DRV8316_OCPLevel amps)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OCP_LVL = amps;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setOCPMode(DRV8316_OCPMode ocpMode)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.OCP_MODE = ocpMode;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setCurrentSenseGain(DRV8316_CSAGain gain)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG1);
	GdConfig1 data;
	data.reg = result;
	data.bits.CSA_GAIN = gain;
	result = DRV8316Driver::write_i2c(GD_CONFIG1, data.reg);
}

void DRV8316Configuration::setDelayCompensationEnabled(bool enabled)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.DELAY_COMP_EN = enabled;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setDelayTarget(DRV8316_DelayTarget us)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.TARGET_DELAY = us;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setBuckSlewRate(DRV8316_Slew slewRate)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.BUCK_SR = slewRate;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setBuckPowerSequencingEnabled(bool enabled)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.BUCK_PS_DIS = enabled;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.BUCK_CL = mamps;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setBuckVoltage(DRV8316_BuckVoltage volts)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.BUCK_SEL = volts;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}

void DRV8316Configuration::setBuckEnabled(bool enabled)
{
	uint32_t result = DRV8316Driver::read_i2c(GD_CONFIG2);
	GdConfig2 data;
	data.reg = result;
	data.bits.BUCK_DIS = enabled;
	result = DRV8316Driver::write_i2c(GD_CONFIG2, data.reg);
}





/*--------------------------*/
/* 		DRV8316Status		*/
/*--------------------------*/





DRV8316Status DRV8316Driver::getStatus() {

	GateDriverFaultStatus 	data;
	ControllerFaultStatus 	data1;
	AlgoStatus 				data2;
	MtrParams 				data3;
	AlgoStatusMpet 			data4; 


	uint32_t result = read_i2c(GATE_DRIVER_FAULT_STATUS);
	data.reg = result;
	
	result = read_i2c(CONTROLLER_FAULT_STATUS);
	data1.reg = result;

	result = read_i2c(ALGO_STATUS);
	data2.reg = result;

	result = read_i2c(MTR_PARAMS);
	data3.reg = result;

	result = read_i2c(ALGO_STATUS_MPET);
	data4.reg = result;

	return DRV8316Status(data, data1, data2, data3, data4); // , data1, data2, data3, data4);
}






/*--------------------------*/
/* 		DRV8316Control		*/
/*--------------------------*/





void DRV8316Control::setSpeed(uint16_t speed)
{
	uint32_t result = 0;
	//uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;

	data.reg = result;

	data.bits.OVERRIDE = 0x1;	// 0x0 = hardware speed input		0x1 = i2c speed input
	data.bits.DIGITAL_SPEED_CTRL = speed; // max value 32768 = 15bit
	data.bits.CLOSED_LOOP_DIS = 0;
	data.bits.FORCE_ALIGN_EN = 0;
	data.bits.FORCE_SLOW_FIRST_CYCLE_EN	 = 0;
	data.bits.FORCE_IPD_EN = 0;
	data.bits.FORCE_ISD_EN = 0;
	data.bits.FORCE_ALIGN_ANGLE_SRC_SEL = 0;
	data.bits.FORCE_IQ_REF_SPEED_LOOP_DIS = 0; // 10 bits
	

	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setClosedloop(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;
	data.reg = result;
	data.bits.CLOSED_LOOP_DIS = !enable;
	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setForceAlign(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;
	data.reg = result;
	data.bits.FORCE_ALIGN_EN = enable;
	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setSlowFirstCycle(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;
	data.reg = result;
	data.bits.FORCE_SLOW_FIRST_CYCLE_EN = enable;
	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setIPD(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;
	data.reg = result;
	data.bits.FORCE_IPD_EN = enable;
	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setISD(bool enable)
{
	uint32_t result = DRV8316Driver::read_i2c(ALGO_CTRL1);
	AlgoCtrl1 data;
	data.reg = result;
	data.bits.FORCE_ISD_EN = enable;
	result = DRV8316Driver::write_i2c(ALGO_CTRL1, data.reg);
}

void DRV8316Control::setCurrentLoopKP(uint16_t Kp)
{
	
}

void DRV8316Control::setCurrentLoopKI(uint16_t Ki)
{
	
}

void DRV8316Control::setSpeedLoopKP(uint16_t Kp)
{
	
}

void DRV8316Control::setSpeedLoopKI(uint16_t Ki)
{
	
}






/*--------------------------*/
/* 		DRV8316Variables	*/
/*--------------------------*/




uint32_t DRV8316Variables::getAlgoState()
{	
	return DRV8316Driver::read_i2c(ALGORITHM_STATE);
}

float DRV8316Variables::getSpeedFG()
{
	//ClosedLoop4 max_speed;
	//return DRV8316Driver::read_i2c(FG_SPEED_FDBK) / 134217728 * max_speed.MAX_SPEED; // Hertz = value / 2^27 * MAX_SPEED
	return DRV8316Driver::read_i2c(FG_SPEED_FDBK); 
}

float DRV8316Variables::getCurrentDC()
{	
	return DRV8316Driver::read_i2c(BUS_CURRENT) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentPhaseA()
{
	return DRV8316Driver::read_i2c(PHASE_CURRENT_A) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentPhaseB()
{
	return DRV8316Driver::read_i2c(PHASE_CURRENT_B) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentPhaseC()
{
	return DRV8316Driver::read_i2c(PHASE_CURRENT_C) / 107374182; // Ampere = value / 2^27 * 1.25
}

uint32_t DRV8316Variables::getCSAgain()
{
	return DRV8316Driver::read_i2c(CSA_GAIN_FEEDBACK);
}

uint32_t DRV8316Variables::getVoltageGain()
{
	return DRV8316Driver::read_i2c(VOLTAGE_GAIN_FEEDBACK);
}

float DRV8316Variables::getVoltageVM()
{
	return DRV8316Driver::read_i2c(VM_VOLTAGE) / 2236962; // Volts = value * 60 / 2^27
}

float DRV8316Variables::getVoltagePhaseA()
{
	return DRV8316Driver::read_i2c(PHASE_VOLTAGE_VA) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getVoltagePhaseB()
{
	return DRV8316Driver::read_i2c(PHASE_VOLTAGE_VB) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getVoltagePhaseC()
{
	return DRV8316Driver::read_i2c(PHASE_VOLTAGE_VC) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getSineAngle()
{
	return DRV8316Driver::read_i2c(SIN_COMMUTATION_ANGLE) / 134217728; // Angle = value / 2^27;
}

float DRV8316Variables::getCosineAngle()
{
	return DRV8316Driver::read_i2c(COS_COMMUTATION_ANGLE) / 134217728; // Angle = value / 2^27;
}

float DRV8316Variables::getCurrentAlpha()
{
	return DRV8316Driver::read_i2c(IALPHA) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentBeta()
{
	return DRV8316Driver::read_i2c(IBETA) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentDaxis()
{
	return DRV8316Driver::read_i2c(ID) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentQaxis()
{
	return DRV8316Driver::read_i2c(IQ) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getVoltageAlpha()
{
	return DRV8316Driver::read_i2c(VALPHA) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getVoltageBeta()
{
	return DRV8316Driver::read_i2c(VBETA) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getVoltageDaxis()
{
	return DRV8316Driver::read_i2c(VD) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getVoltageQaxis()
{
	return DRV8316Driver::read_i2c(VQ) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getAlignCurrent()
{
	return DRV8316Driver::read_i2c(IQ_REF_ROTOR_ALIGN) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getSpeedOpenloopReference()
{
	//ClosedLoop4 max_speed;
	//return DRV8316Driver::read_i2c(SPEED_REF_OPEN_LOOP) / 134217728 * max_speed.MAX_SPEED; // Hertz = value / 2^27 * MAX_SPEED
	return DRV8316Driver::read_i2c(SPEED_REF_OPEN_LOOP); 
}

float DRV8316Variables::getSpeedClosedloopReference()
{
	//ClosedLoop4 max_speed;
	//return DRV8316Driver::read_i2c(SPEED_REF_CLOSED_LOOP) / 134217728 * max_speed.MAX_SPEED; // Hertz = value / 2^27 * MAX_SPEED
	return DRV8316Driver::read_i2c(SPEED_REF_CLOSED_LOOP);
}

float DRV8316Variables::getCurrentDaxisReference()
{
	return DRV8316Driver::read_i2c(ID_REF_CLOSED_LOOP) / 107374182; // Ampere = value / 2^27 * 1.25
}

float DRV8316Variables::getCurrentQaxisReference()
{
	return DRV8316Driver::read_i2c(IQ_REF_CLOSED_LOOP) / 107374182; // Ampere = value / 2^27 * 1.25
}

uint32_t DRV8316Variables::getISDstate()
{
	return DRV8316Driver::read_i2c(ISD_STATE);
}

float DRV8316Variables::getISDspeed()
{
	//ClosedLoop4 max_speed;
	//return DRV8316Driver::read_i2c(ISD_SPEED) / 134217728 * max_speed.MAX_SPEED; // Hertz = value / 2^27 * MAX_SPEED
	return DRV8316Driver::read_i2c(ISD_SPEED);
}

uint32_t DRV8316Variables::getIPDstate()
{
	return DRV8316Driver::read_i2c(IPD_STATE);
}

float DRV8316Variables::getIPDangle()
{
	return DRV8316Driver::read_i2c(IPD_ANGLE) / 372827; // Angle(°) = value / 2^27 * 360
}

float DRV8316Variables::getBEMFqAxis()
{
	return DRV8316Driver::read_i2c(EQ) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getBEMFdAxis()
{
	return DRV8316Driver::read_i2c(ED) / 3874532; // Volts = value * 60 / (sqrt(3) * 2^27)
}

float DRV8316Variables::getSpeed()
{
	//ClosedLoop4 max_speed;
	//return DRV8316Driver::read_i2c(SPEED_FDBK) / 134217728 * max_speed.MAX_SPEED; // Hertz = value / 2^27 * MAX_SPEED;
	return DRV8316Driver::read_i2c(SPEED_FDBK);
}

float DRV8316Variables::getMotorPosition()
{
	return DRV8316Driver::read_i2c(THETA_EST) / 372827; // Angle(°) = value / 2^27 * 360
}







/*--------------------------*/
/* 		DRV8316Driver		*/
/*--------------------------*/






void DRV8316Driver::clearFault()
{
	uint32_t result = read_i2c(DEV_CTRL);
	DevCtrl data;
	data.reg = result;
	data.bits.CLR_FLT = 1;
	result = write_i2c(DEV_CTRL, data.reg);
}



void DRV8316Driver::init() {
	delay_us(1);
}
