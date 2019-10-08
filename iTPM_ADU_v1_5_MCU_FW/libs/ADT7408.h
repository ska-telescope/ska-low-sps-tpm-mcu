/*
 * ADT7408.h
 *
 * Created: 21/12/2018 16:04:45
 *  Author: luca
 */ 


#ifndef ADT7408_H_
#define ADT7408_H_

// ------------- ADT7408 FLAGS ---------------
#define ADT7408_ADDRESS1 0x18

#define ADT7408_REG_CAPABILTY 0x00
#define ADT7408_REG_CONFIGURATION 0x01
#define ADT7408_REG_ALARM_TEMP_UPPER 0x02
#define ADT7408_REG_ALARM_TEMP_LOWER 0x03
#define ADT7408_REG_CRITICAL_TEMP_TRIP 0x04
#define ADT7408_REG_TEMPERATURE 0x05

#define ADT7408_ABOVE_CRITICAL 32768 // BIT 15
#define ADT7408_ABOVE_ALARM 16384 // BIT 14
#define ADT7408_BELOW_ALARM 8192 // BIT 13
#define ADT7408_TEMP_SIGN 4096 // BIT 12
// -------------------------------------------


#endif /* ADT7408_H_ */