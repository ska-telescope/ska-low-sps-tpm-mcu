/*
 * ADT7408.cpp
 *
 * Created: 21/12/2018 16:05:45
 *  Author: luca
 */ 

#include "ADT7408.h"
#include "TwiFpga.h"
#include <stdbool.h>

/**
This function read the board temperature sensors.
@param Temp1 is a pointer for the temperature of the sensor number 1
@param Temp2 is a pointer for the temperature of the sensor number 2
@param Temp1Regs is a pointer array [3] for the alarm registers of sensor number 1; [0] Above Cristical; [1] Above Alarm; [2] Below Alarm
@param Temp2Regs is a pointer array [3] for the alarm registers of sensor number 2; [0] Above Cristical; [1] Above Alarm; [2] Below Alarm
@returns Void - Return no value.
*/
void readBoardTemp(uint16_t* Temp1, uint16_t* Temp2, bool* Temp1Regs[3], bool* Temp2Regs[3]){
	volatile uint16_t temp1, temp2;
	bool temp1regs[3], temp2regs[3];
	volatile float  temp1out, temp2out;
	
	
	temp1 = twiFpgaRead16(ADT7408_ADDRESS1, ADT7408_REG_TEMPERATURE, i2c1);
	//temp1 = readi2cRegister16(ADT7408_REG_TEMPERATURE, ADT7408_ADDRESS1);
	if (temp1 & ADT7408_ABOVE_CRITICAL) { // Above Critical Flags is setted - DO SOMETHING!!!
		temp1 = temp1 - ADT7408_ABOVE_CRITICAL;
		temp1regs[0] =  true;
	}
	else temp1regs[0] =  false;
	if (temp1 & ADT7408_ABOVE_ALARM) { // Above Alarm Flags is setted - DO SOMETHING!!!
		temp1 = temp1 - ADT7408_ABOVE_ALARM;
		temp1regs[1] =  true;
	}
	else temp1regs[1] =  false;
	if (temp1 & ADT7408_BELOW_ALARM) { // Above Alarm Flags is setted - DO SOMETHING!!!
		temp1 = temp1 - ADT7408_BELOW_ALARM;
		temp1regs[2] =  true;
	}
	else temp1regs[2] =  false;
	if (temp1 & ADT7408_TEMP_SIGN) { // Oh, the baord is freezing
		temp1 = temp1 - ADT7408_TEMP_SIGN;
		temp1out = temp1;
		temp1out = (temp1out / 16) * (-1);
	}
	else {
		temp1out = temp1;
		temp1out = temp1out / 16 ;
	}
	
	temp2 = twiFpgaRead16(ADT7408_ADDRESS2, ADT7408_REG_TEMPERATURE, i2c1);
	//temp2 = readi2cRegister16(ADT7408_REG_TEMPERATURE, ADT7408_ADDRESS2);
	if (temp2 & ADT7408_ABOVE_CRITICAL) { // Above Critical Flags is setted - DO SOMETHING!!!
		temp2 = temp2 - ADT7408_ABOVE_CRITICAL;
		temp2regs[0] =  true;
	}
	else temp2regs[0] =  false;
	if (temp2 & ADT7408_ABOVE_ALARM) { // Above Alarm Flags is setted - DO SOMETHING!!!
		temp2 = temp2 - ADT7408_ABOVE_ALARM;
		temp2regs[1] =  true;
	}
	else temp2regs[1] =  false;
	if (temp2 & ADT7408_BELOW_ALARM) { // Above Alarm Flags is setted - DO SOMETHING!!!
		temp2 = temp2 - ADT7408_BELOW_ALARM;
		temp2regs[2] =  true;
	}
	else temp2regs[2] =  false;
	if (temp2 & ADT7408_TEMP_SIGN) { // Oh, the baord is freezing
		temp2 = temp2 - ADT7408_TEMP_SIGN;
		temp2out = temp2;
		temp2out = (temp2out / 16) * (-1);
	}
	else {
		temp2out = temp2;
		temp2out = temp2out / 16 ;
	}
	
	*Temp1 = temp1out;
	*Temp2 = temp2out;
	*Temp1Regs = temp1regs;
	*Temp2Regs = temp2regs;
	
}

/**
This function read the alarm boundary of temperature sensor 1.
@param No parmameters
@returns atub1 uinti16_t
*/
uint16_t readAlarmTempUpperBoundary1(){
	volatile uint16_t atub1;
	volatile float out;
	atub1 = twiFpgaRead16(ADT7408_ADDRESS1, ADT7408_REG_ALARM_TEMP_UPPER, i2c1);
	out = (float)atub1/4;
	return atub1;
}

/**
This function write the alarm boundary of temperature sensor 1.
@param atub1
@returns Void - Return no value.
*/
void writeAlarmTempUpperBoundary1(uint16_t atub1){
	atub1 = atub1 << 2;
	twiFpgaWrite16(ADT7408_ADDRESS1, atub1, ADT7408_REG_ALARM_TEMP_UPPER, i2c1);
}

/**
This function read the alarm boundary of temperature sensor 2.
@param No parmameters
@returns atub1 uinti16_t
*/
uint16_t readAlarmTempUpperBoundary2(){
	volatile uint16_t atub2;
	volatile float out;
	atub2 = twiFpgaRead16(ADT7408_ADDRESS2, ADT7408_REG_ALARM_TEMP_UPPER, i2c1);
	out = (float)atub2/4;
	return atub2;
}

/**
This function write the alarm boundary of temperature sensor 1.
@param atub1
@returns Void - Return no value.
*/
void writeAlarmTempUpperBoundary2(uint16_t atub2){
	atub2 = atub2 << 2;
	twiFpgaWrite16(ADT7408_ADDRESS2, atub2, ADT7408_REG_ALARM_TEMP_UPPER, i2c1);
}

/**
This function read the configuration of board temperature sensors.
@param config1 is a pointer for return the register configuration of sensor 1
@param config2 is a pointer for return the register configuration of sensor 2
@returns Void - Return no value.
*/
void readADTConfiguration(uint16_t* config1, uint16_t* config2){
	//volatile uint16_t config1, config2;
	*config1 = twiFpgaRead16(ADT7408_ADDRESS1, ADT7408_REG_CONFIGURATION, i2c1);
	*config2 = twiFpgaRead16(ADT7408_ADDRESS2, ADT7408_REG_CONFIGURATION, i2c1);
}

/**
This function write the configuration of board temperature sensors.
@param config1 Config for the register configuration of sensor 1
@param config2 Config for the register configuration of sensor 2
@returns Void - Return no value.
*/
void writeADTConfiguration(uint16_t config1, uint16_t config2){
	twiFpgaWrite16(ADT7408_ADDRESS1, config1, ADT7408_REG_CONFIGURATION, i2c1);
	twiFpgaWrite16(ADT7408_ADDRESS2, config2, ADT7408_REG_CONFIGURATION, i2c1);
}