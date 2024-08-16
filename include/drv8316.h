#ifndef SIMPLEFOC_DRV8316
#define SIMPLEFOC_DRV8316

#include "drv8316_registers.h"
#include <libopencm3/stm32/i2c.h>
#include <digio.h>


class DRV8316Configuration {
public:
	DRV8316Configuration() {};

	uint32_t setIsdEnable(bool enable);

	void setBrakeEnable(bool enable);
	void setBrakeMode(uint8_t mode);
	void setBrakeTime(DRV8316_Time time);
	
	void setHizEnable(bool enable);
	void setHizTime(DRV8316_Time time);

	void setReverseEnable(bool enable);

	void setResyncEnable(bool enable);
	void setResyncThreshold(DRV8316_Percentage percentage);
	
	void setBemfStationaryThreshold(DRV8316_BEMF_Thr voltage);
	
	void setActiveBrakeEnable(bool enable);
	void setOpenLoopCurrentLimit(Current_Lim current);

	void setAlignAngle(DRV8316_Angle degrees);
	void setOvermodulationEnable(bool enable);

	void setClosedLoopAccel(DRV8316_Frequency dps);
	void setClosedLoopDecel(DRV8316_Frequency dps);

	void setPWMfreq(DRV8316_PWMFrequency khz);
	void setPWMmode(uint8_t mode);

	void setAvsEnable(bool enable);
	void setDeadtimeCompensationEnable(bool enable);
	void setSpeedloopDisable(bool enable);

	void setMotorStartupMethod(uint8_t method);
	void setMotorStopMethod(uint8_t method);
	void setMotorStopTime(uint8_t time);
	void setMotorMaxSpeed(uint16_t hertz);
	void setMotorResistance(uint8_t ohm);
	void setMotorInductance(uint8_t henry);

	void setSpeedProfileConfig(uint8_t mode);

	void setBrakePinMode(uint8_t mode);
	void setBrakeInput(uint8_t input);
	void setSpeedMode(uint8_t mode);

	void setPin38Config(uint8_t config);
	void setI2C_Address(uint8_t address);
	void setMaxVoltage(uint8_t voltage);

	void setMaxInputFreq(uint16_t freq);
	void setSleepEntryTime(uint8_t time);
	void setDeviceMode(uint8_t mode);
	void setClockSource(uint8_t source);
	void setExtClockEnable(bool enable);
	void setExtClockFreq(uint8_t freq);
	void setExtWDenable(bool enable);
	void setExtWDinterval(uint8_t interval);
	void setExtWDfaultMode(uint8_t mode);
	

	void setCurrentLimitEnable(bool enable);
	void setDirInput(uint8_t input);

	void setSelfTestEnable(bool enable);
	void setSpeedRange(uint8_t range);

	void setSlewRate(DRV8316_Slew slewRate);

	void setOvervoltageLevel(DRV8316_OVP voltage);
	void setOvervoltageProtection(bool enable);
	void setOvertemperatureReporting(bool reportFault);

	void setOCPDeglitchTime(DRV8316_OCPDeglitch ms);
	void setOCPRetryTime(DRV8316_OCPRetry ms);
	void setOCPLevel(DRV8316_OCPLevel amps);
	void setOCPMode(DRV8316_OCPMode ocpMode);

	void setCurrentSenseGain(DRV8316_CSAGain gain);

	void setDelayCompensationEnabled(bool enabled);
	void setDelayTarget(DRV8316_DelayTarget us);
	void setBuckSlewRate(DRV8316_Slew slewRate);
	void setBuckPowerSequencingEnabled(bool enabled);
	void setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps);
	void setBuckVoltage(DRV8316_BuckVoltage volts);
	void setBuckEnabled(bool enabled);
	
};

// GateDriverFaultStatus
class DRV8316ICStatus {
private:
	GateDriverFaultStatus gateDriverStatus;

public:
	DRV8316ICStatus(GateDriverFaultStatus status) : gateDriverStatus(status) {};
	~DRV8316ICStatus() {};

	bool isBuckError() { return gateDriverStatus.bits.BK_FLT == 0x01; }
	bool isOverCurrent() { return gateDriverStatus.bits.OCP == 0x01; }
	bool isPowerOnReset() { return gateDriverStatus.bits.NPOR == 0x01; }
	bool isOverVoltage() { return gateDriverStatus.bits.OVP == 0x01; }
	bool isOverTemperature() { return gateDriverStatus.bits.OT == 0x01; }
	bool isOverTemperatureWarning() { return gateDriverStatus.bits.OTW == 0x01; }
	bool isOverTemperatureShutdown() { return gateDriverStatus.bits.OTS == 0x01; }

	bool isOverCurrent_Ah() { return gateDriverStatus.bits.OCP_HA == 0x01; }
	bool isOverCurrent_Al() { return gateDriverStatus.bits.OCP_LA == 0x01; }
	bool isOverCurrent_Bh() { return gateDriverStatus.bits.OCP_HB == 0x01; }
	bool isOverCurrent_Bl() { return gateDriverStatus.bits.OCP_LB == 0x01; }
	bool isOverCurrent_Ch() { return gateDriverStatus.bits.OCP_HC == 0x01; }
	bool isOverCurrent_Cl() { return gateDriverStatus.bits.OCP_LC == 0x01; }

	bool isOneTimeProgrammingError() { return gateDriverStatus.bits.OTP_ERR == 0x01; }
	bool isBuckOverCurrent() { return gateDriverStatus.bits.BUCK_OCP == 0x01; }
	bool isBuckUnderVoltage() { return gateDriverStatus.bits.BUCK_UV == 0x01; }
	bool isChargePumpUnderVoltage() { return gateDriverStatus.bits.VCP_UV == 0x01; }


};

// ControllerFaultStatus
class DRV8316Status1 {
private:
	ControllerFaultStatus status1;

public:
	DRV8316Status1(ControllerFaultStatus status1) : status1(status1) {};
	~DRV8316Status1() {};
};

// AlgoStatus
class DRV8316Status2 {
private:
	AlgoStatus status2;

public:
	DRV8316Status2(AlgoStatus status2) : status2(status2) {};
	~DRV8316Status2() {};
};

// MtrParams
class DRV8316Status3 {
private:
	MtrParams status3;

public:
	DRV8316Status3(MtrParams status3) : status3(status3) {};
	~DRV8316Status3() {};
};

// AlgoStatusMpet
class DRV8316Status4 {
private:
	AlgoStatusMpet status4;

public:
	DRV8316Status4(AlgoStatusMpet status4) : status4(status4) {};
	~DRV8316Status4() {};
};


class DRV8316Status : public DRV8316ICStatus, public DRV8316Status1, public DRV8316Status2, public DRV8316Status3, public DRV8316Status4 {
	public:
		DRV8316Status(GateDriverFaultStatus status, ControllerFaultStatus status1, AlgoStatus status2, MtrParams status3, AlgoStatusMpet status4) : 
		DRV8316ICStatus(status), DRV8316Status1(status1), DRV8316Status2(status2), DRV8316Status3(status3), DRV8316Status4(status4) {};
		~DRV8316Status() {};
}; 


class DRV8316Control {
public:
	DRV8316Control(){};
	
	void setSpeed(uint16_t speed);
	void setClosedloop(bool enable);
	void setForceAlign(bool enable);
	void setSlowFirstCycle(bool enable);
	void setIPD(bool enable);
	void setISD(bool enable);
	//void setIqRefSpeed(uint16_t);
	void setCurrentLoopKP(uint16_t Kp);
	void setCurrentLoopKI(uint16_t Ki);
	void setSpeedLoopKP(uint16_t Kp);
	void setSpeedLoopKI(uint16_t Ki);
};


class DRV8316Variables {
public:
	DRV8316Variables(){};
	uint32_t getAlgoState();
	float getSpeedFG();
	float getCurrentDC();
	float getCurrentPhaseA();
	float getCurrentPhaseB();
	float getCurrentPhaseC();
	uint32_t getCSAgain();
	uint32_t getVoltageGain();
	float getVoltageVM();
	float getVoltagePhaseA();
	float getVoltagePhaseB();
	float getVoltagePhaseC();
	float getSineAngle();
	float getCosineAngle();
	float getCurrentAlpha();
	float getCurrentBeta();
	float getCurrentDaxis();
	float getCurrentQaxis();
	float getVoltageAlpha();
	float getVoltageBeta();
	float getVoltageDaxis();
	float getVoltageQaxis();
	float getAlignCurrent();
	float getSpeedOpenloopReference();
	float getSpeedClosedloopReference();
	float getCurrentDaxisReference();
	float getCurrentQaxisReference();
	uint32_t getISDstate();
	float getISDspeed();
	uint32_t getIPDstate();
	float getIPDangle();
	float getBEMFqAxis();
	float getBEMFdAxis();
	float getSpeed();
	float getMotorPosition();
};



class DRV8316Driver {
private: 
	static uint32_t i2c;
    static uint16_t address;

public:
	DRV8316Driver(uint32_t i2c, uint16_t address){
		DRV8316Driver::i2c = i2c;
        DRV8316Driver::address = address;
	};

	DRV8316Variables Variables;
	DRV8316Configuration Config;
	DRV8316Control Control;
	//DRV8316Status Status;

	void init();
	void clearFault(); // TODO check for fault condition methods
	void saveToEEPROM();
	DRV8316Status getStatus();

	static uint32_t write_i2c(uint16_t addr, uint32_t value);
	static uint32_t read_i2c(uint16_t addr);
};


#endif
