/*
 * iTPM ADU V1.5 MCU FW (BIOS)
 *
 * Sanitas EG
 *
 * Created: 07/10/2019 09:12:42 ~ Luca Schettini
 *
 * Copyright (c) 2017-2019 Sanitas EG srl.  All right reserved.
 *
 */ 

#include <atmel_start.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "driver_init.h"
#include "utils.h"

#include "Settings_itpm.h"
#include "build_def.h"
#include "SpiRouter.h"
#include "regfile.h"
#include "TwiFpga.h"
#include "ADT7408.h"

// WARNING - Proper undefine DEBUG in Project properties
#ifdef DEBUG
char bufferOut[512];
#undef DEBUG // Undefine only for remove warning
#define DEBUG 2 // Possible Choice: 3: Log Level - 2: Warning Level - 1: Error Level - Only #def: Status
#endif

#if defined(DEBUG)
#define DEBUG_PRINT(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print();} while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 0
#define DEBUG_PRINT1(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x1);} while( false )
#else
#define DEBUG_PRINT1(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 1
#define DEBUG_PRINT2(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x2);} while( false )
#else
#define DEBUG_PRINT2(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 2
#define DEBUG_PRINT3(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x3);} while( false )
#else
#define DEBUG_PRINT3(...) do{ } while ( false )
#endif

const uint32_t _build_version = 0xb0000024;
const uint32_t _build_date = ((((BUILD_YEAR_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH1 & 0xFF - 0x30)) << 24) | (((BUILD_YEAR_CH2 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH3 & 0xFF - 0x30)) << 16) | (((BUILD_MONTH_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_MONTH_CH1 & 0xFF - 0x30)) << 8) | (((BUILD_DAY_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_DAY_CH1 & 0xFF - 0x30))));
//const uint32_t _build_time = (0x00 << 24 | (((__TIME__[0] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[1] & 0xFF - 0x30)) << 16) | (((__TIME__[3] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[4] & 0xFF - 0x30)) << 8) | (((__TIME__[6] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[7] & 0xFF - 0x30))));

#define ADCCOLUMNS 14
#define TEMPS_SENSOR 3
struct ADCstruct {
	int ADCpin;
	uint16_t ADCread;
	float divider;
	uint16_t alarmTHRupper;
	uint16_t alarmTHRdowner;
	uint16_t warningTHRupper;
	uint16_t warningTHRdowner;
	bool enabled;
	};
	
struct ADCstruct VoltagesTemps[ADCCOLUMNS+TEMPS_SENSOR]; // +3 Temperatures
int anaReadPos = 0;
bool anaNotReady = true;

/* --------- VAR -------------------- */

static const float ADC_STEP = (2.5/65536); // Ext Ref Voltage (2.5V) / 16Bit ADC 2^16

bool irqTimerSlow = false;
bool irqTimerFast = false;
bool irqExternalFPGA = false;

uint32_t ADT7408_temp_raw;
uint16_t ADT7408_temp;
bool ADT7408Regs[3];
uint32_t pollingHz;
uint32_t reg_ThresholdEnable = 0;
uint16_t reg_ThresholdVals [2][17];
uint32_t pollingOld = 1000;
uint32_t pollingNew = 1000;

bool TPMpowerLock = false;

/* -----------------------------------*/

#ifdef DEBUG
void deb_print(uint8_t debug_level = 0){
	char * str;
	str = const_cast<char *>(bufferOut);
	struct io_descriptor *uartDebug;
	usart_sync_get_io_descriptor(&USART_0, &uartDebug);
	usart_sync_enable(&USART_0);
	switch (debug_level) {
		case 0x1:
			io_write(uartDebug, (uint8_t *)"d1:", (unsigned)3);
			break;
		case 0x2:
			io_write(uartDebug, (uint8_t *)"d2:", (unsigned)3);
			break;
		case 0x3:
			io_write(uartDebug, (uint8_t *)"d3:", (unsigned)3);
			break;
	}
	io_write(uartDebug, (uint8_t *)str, (unsigned)strlen(str));
}
#endif

/* -----------------------------------*/

void framRead(uint32_t fram_register, uint32_t* readback){
	uint32_t intreadback;
	XO3_Read(itpm_cpld_bram_cpu + fram_register, &intreadback);
	*readback = intreadback;
}

void framWrite(uint32_t fram_register, uint32_t writedata){
	XO3_WriteByte(itpm_cpld_bram_cpu + fram_register, writedata);
}

static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync() {
	while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void SKAPower(bool ADCpwr, bool FRONTENDpwr, bool FPGApwr, bool SYSRpwr, bool VGApwr) {
	uint8_t tmp = 0;
	if (ADCpwr)			tmp += 0x1;
	if (FRONTENDpwr)	tmp += 0x2;
	if (FPGApwr)		tmp += 0x4;
	if (SYSRpwr)		tmp += 0x8;
	if (VGApwr)			tmp += 0x10;
	
	XO3_WriteByte(itpm_cpld_regfile_enable_shadow, tmp);
	//XO3_WriteByte(itpm_cpld_regfile_enable, tmp); // OLD
	
	DEBUG_PRINT("Powered devices ADC %d - Frontend %d - FPGA %d - SYSR %d - VGA %d\n", ADCpwr, FRONTENDpwr, FPGApwr, SYSRpwr, VGApwr);
}

void SKAalarmUpdate(void){
		uint32_t risp;
		
		// WARNING
		
		framRead(FRAM_WARN_THR_SW_AVDD1, &risp);
		VoltagesTemps[int(SWAVDD1)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD1)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_SW_AVDD2, &risp);
		VoltagesTemps[int(SWAVDD2)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD2)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_AVDD3, &risp);
		VoltagesTemps[int(SWAVDD3)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD3)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MAN_1V2, &risp);
		VoltagesTemps[int(MAN1V2)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V2)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_DDR0_VREF, &risp);
		VoltagesTemps[int(DDR0VREF)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR0VREF)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_DDR1_VREF, &risp);
		VoltagesTemps[int(DDR1VREF)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR1VREF)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_DRVDD, &risp);
		VoltagesTemps[int(VMDRVDD)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(VMDRVDD)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VIN_SCALED, &risp);
		VoltagesTemps[int(VINSCALED)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(VINSCALED)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_MAN3V3, &risp);
		VoltagesTemps[int(MAN3V3)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN3V3)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_MAN1V8, &risp);
		VoltagesTemps[int(MAN1V8)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V8)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MON_5V0, &risp);
		VoltagesTemps[int(MON5V0)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MON5V0)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MGT_AV, &risp);
		VoltagesTemps[int(MGTAV)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MGTAV)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MGT_AVTT, &risp);
		VoltagesTemps[int(MGAVTT)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MGAVTT)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_INTERNAL_MCU_TEMP, &risp);
		VoltagesTemps[int(INTTEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(INTTEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_BOARD_TEMP, &risp);
		VoltagesTemps[int(BOARDTEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(BOARDTEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_FPGA0_TEMP, &risp);
		VoltagesTemps[int(FPGA0TEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA0TEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_FPGA1_TEMP, &risp);
		VoltagesTemps[int(FPGA1TEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA1TEMP)].warningTHRupper = uint16_t(risp>>16);
		
		// ALARM
		
		framRead(FRAM_ALARM_THR_SW_AVDD1, &risp);
		VoltagesTemps[int(SWAVDD1)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD1)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_SW_AVDD2, &risp);
		VoltagesTemps[int(SWAVDD2)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD2)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_AVDD3, &risp);
		VoltagesTemps[int(SWAVDD3)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD3)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MAN_1V2, &risp);
		VoltagesTemps[int(MAN1V2)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V2)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_DDR0_VREF, &risp);
		VoltagesTemps[int(DDR0VREF)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR0VREF)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_DDR1_VREF, &risp);
		VoltagesTemps[int(DDR1VREF)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR1VREF)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_DRVDD, &risp);
		VoltagesTemps[int(VMDRVDD)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(VMDRVDD)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VIN_SCALED, &risp);
		VoltagesTemps[int(VINSCALED)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(VINSCALED)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_MAN3V3, &risp);
		VoltagesTemps[int(MAN3V3)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN3V3)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_MAN1V8, &risp);
		VoltagesTemps[int(MAN1V8)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V8)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MON_5V0, &risp);
		VoltagesTemps[int(MON5V0)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MON5V0)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MGT_AV, &risp);
		VoltagesTemps[int(MGTAV)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MGTAV)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MGT_AVTT, &risp);
		VoltagesTemps[int(MGAVTT)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MGAVTT)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_INTERNAL_MCU_TEMP, &risp);
		VoltagesTemps[int(INTTEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(INTTEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_BOARD_TEMP, &risp);
		VoltagesTemps[int(BOARDTEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(BOARDTEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_FPGA0_TEMP, &risp);
		VoltagesTemps[int(FPGA0TEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA0TEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_FPGA1_TEMP, &risp);
		VoltagesTemps[int(FPGA1TEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA1TEMP)].alarmTHRupper = uint16_t(risp>>16);

		
		
		framWrite(FRAM_WARN_ALARM_UPDATE, 0x0); //Reset Registers

}

void SKAalarmManage(){
	if ((VoltagesTemps[anaReadPos].ADCread > VoltagesTemps[anaReadPos].warningTHRupper) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_WARNING
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,itpm_cpld_regfile_global_status_voltage_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC WARNING %d too high, val %d expected max %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].warningTHRupper);
		//delay_ms(500); // ONLY FOR TEST
	}	
	if ((VoltagesTemps[anaReadPos].ADCread < VoltagesTemps[anaReadPos].warningTHRdowner) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_WARNING
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,itpm_cpld_regfile_global_status_voltage_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC WARNING %d too low, val %d expected min %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].warningTHRdowner);
		//delay_ms(500); // ONLY FOR TEST
	}
	if ((VoltagesTemps[anaReadPos].ADCread > VoltagesTemps[anaReadPos].alarmTHRupper) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_ALARM
		//SKAPower(0,0,0,0,0);
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,itpm_cpld_regfile_global_status_temperature_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC ALARM %d too high, val %d expected max %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].alarmTHRupper);
		//delay_ms(500); // ONLY FOR TEST
	}
	if ((VoltagesTemps[anaReadPos].ADCread < VoltagesTemps[anaReadPos].alarmTHRdowner) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_ALARM
		//SKAPower(0,0,0,0,0);
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,itpm_cpld_regfile_global_status_temperature_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC ALARM %d too low, val %d expected min %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].alarmTHRdowner);
		//delay_ms(500); // ONLY FOR TEST
	}
}

int32_t SAMinternalTempConv(uint32_t raw) {
	int32_t adc = raw;
	//use device factory calibration values for temperature conversion (simplified)
	uint32_t* tmpLogBase = (uint32_t*)0x00806030;
	uint32_t tmpLog0 = tmpLogBase[0];
	uint32_t tmpLog1 = tmpLogBase[1];
	uint8_t roomInt = tmpLog0 & 0xff;
	uint8_t roomFrac = (tmpLog0 >> 8) & 0x0f;
	uint8_t hotInt = (tmpLog0 >> 12) & 0xff;
	uint8_t hotFrac = (tmpLog0 >> 20) & 0x0f;
	int32_t roomADC = ((tmpLog1 >> 8) & 0xfff) << 4;
	int32_t hotADC = ((tmpLog1 >> 20) & 0xfff) << 4;
	int32_t roomMdeg = 1000 * roomInt + 100 * roomFrac;
	int32_t hotMdeg = 1000 * hotInt + 100 * hotFrac;
	int32_t mdeg = roomMdeg + ((hotMdeg-roomMdeg) * (adc-roomADC)) / (hotADC-roomADC);
	return mdeg;
}

void exchangeDataBlock(){
	uint32_t res = 0x0;
	
	framWrite(FRAM_ADC_SW_AVDD1,			VoltagesTemps[0].ADCread);
	framWrite(FRAM_ADC_SW_AVDD2,			VoltagesTemps[1].ADCread);
	framWrite(FRAM_ADC_AVDD3,				VoltagesTemps[2].ADCread);
	framWrite(FRAM_ADC_MAN_1V2,				VoltagesTemps[3].ADCread);
	framWrite(FRAM_ADC_DDR0_VREF,			VoltagesTemps[4].ADCread);
	framWrite(FRAM_ADC_DDR1_VREF,			VoltagesTemps[5].ADCread);
	framWrite(FRAM_ADC_VM_DRVDD,			VoltagesTemps[6].ADCread);
	framWrite(FRAM_ADC_VIN_SCALED,			VoltagesTemps[7].ADCread);
	framWrite(FRAM_ADC_VM_MAN3V3,			VoltagesTemps[8].ADCread);
	framWrite(FRAM_ADC_VM_MAN1V8,			VoltagesTemps[9].ADCread);
	framWrite(FRAM_ADC_MON_5V0,				VoltagesTemps[10].ADCread);
	framWrite(FRAM_ADC_MGT_AV,				VoltagesTemps[11].ADCread);
	framWrite(FRAM_ADC_MGT_AVTT,			VoltagesTemps[12].ADCread);	
	framWrite(FRAM_ADC_INTERNAL_MCU_TEMP,	SAMinternalTempConv(uint32_t(VoltagesTemps[13].ADCread)));
	
	framWrite(FRAM_BOARD_TEMP, ADT7408_temp_raw);
	
	framRead(FRAM_WARN_ALARM_UPDATE, &res);
	if (res == 0x1) SKAalarmUpdate();
	

	
// 	framRead(FRAM_THRESHOLD_ENABLE_MASK, &reg_ThresholdEnable);
// 	if (reg_ThresholdEnable && 0x80000000) {
// 		int x = 0;
// 		for (uint32_t i = FRAM_ALARM_THR_SW_AVDD1; i < 0x1D8; i += 4){
// 			uint32_t temp;
// 			framRead(i, &temp);
// 			reg_ThresholdVals[0][x] = (temp && 0xFFFF); // High Threshold
// 			reg_ThresholdVals[1][x] = (temp && 0xFFFF0000) >> 16; // Low Threshold
// 			x+;+
// 		}
// 	}
	
}

void ADCreadSingle() { // Single read, much FASTER
	if (ADC->INTFLAG.bit.RESRDY == 1){
		//uint32_t valueRead = ADC->RESULT.reg;
		//adcArrgh[1][anaReadPos] = ADC->RESULT.reg; // Save ADC read to the array
		
		float ADC_voltage = (ADC_STEP * ADC->RESULT.reg) * 1000;
		///??? float Voltage = (ADC_voltage * ADC_STEP) * 1000;
		
		if (VoltagesTemps[anaReadPos].divider == 0) VoltagesTemps[anaReadPos].ADCread = (uint16_t)ADC_voltage;
		else VoltagesTemps[anaReadPos].ADCread = (uint16_t)(ADC_voltage * VoltagesTemps[anaReadPos].divider);
		
		//if (adcDivider[anaReadPos] == 0) adcArrgh[1][anaReadPos] = (uint16_t)ADC_voltage;
		//else adcArrgh[1][anaReadPos] = (uint16_t)(ADC_voltage * adcDivider[anaReadPos]);
		
		DEBUG_PRINT3("ANALOG_READ > %d %i\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread);
		
		ADCsync();
		ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
		
		SKAalarmManage();
		
		if (anaReadPos >= ADCCOLUMNS-1){
			anaReadPos = 0;
			anaNotReady = false;
		}
		else anaReadPos++;
		ADC->INPUTCTRL.bit.MUXPOS = VoltagesTemps[anaReadPos].ADCpin;
		ADC->INTFLAG.bit.RESRDY = 1;
		ADC->SWTRIG.bit.START = 1;
	}
}

void ADCstart() { // Single read, much FASTER
	DEBUG_PRINT("Analog Start\n");
	//REG_PM_APBCMASK |= 0x10000; // Enable bus Clock
	//REG_GCLK_CLKCTRL = 0x4001E;
	//REG_ADC_SAMPCTRL = 5;
	ADC->INPUTCTRL.bit.MUXNEG = 0x18; // Mux NEG GND
	SYSCTRL->VREF.bit.TSEN = 1; // Enable TSENOR
	//REG_ADC_REFCTRL = 0;
	//REG_ADC_CTRLA = 2;
	
	ADCsync();
	ADC->CTRLA.bit.ENABLE = 0x01;              // Enable ADC

	ADC->INTFLAG.bit.RESRDY = 1;               // Data ready flag cleared

	ADCsync();
	ADC->SWTRIG.bit.START = 1;                 // Start ADC conversion
	
	for (int i=0; i < ADCCOLUMNS; i++) ADCreadSingle();
	//while (anaReadPos < ADCCOLUMNS) ADCreadSingle();
	anaReadPos = 0;
}

void TWIdataBlock(void){
	int status;
	uint32_t retvalue = 0xffffffff;

	// i2c1
   //readBoardTemp(&ADT7408_temp, &ADT7408Regs[3]); // Disabled for errors
	status = twiFpgaWrite(0x30, 1, 2, 0x05, &ADT7408_temp_raw, i2c1); //temp_value 0x30
	//XO3_WriteByte(fram_ADT7408_M_1_temp_val + fram_offset, retvalue);
	retvalue = 0xffffffff;

	
}

void StartupLoadSettings(void){
	DEBUG_PRINT("Push Settings to CPLD\n");
	
	framWrite(FRAM_MCU_POOLING_INTERVAL, DEFAULT_POLLING_INTERVAL);
	
	framWrite(FRAM_WARN_ALARM_UPDATE, SETTING_WARN_ALARM_UPDATE);
	
	framWrite(FRAM_WARN_THR_SW_AVDD1			, SETTING_WARN_THR_SW_AVDD1				);
	framWrite(FRAM_WARN_THR_SW_AVDD2			, SETTING_WARN_THR_SW_AVDD2				);
	framWrite(FRAM_WARN_THR_AVDD3				, SETTING_WARN_THR_AVDD3				);
	framWrite(FRAM_WARN_THR_MAN_1V2				, SETTING_WARN_THR_MAN_1V2				);
	framWrite(FRAM_WARN_THR_DDR0_VREF			, SETTING_WARN_THR_DDR0_VREF			);
	framWrite(FRAM_WARN_THR_DDR1_VREF			, SETTING_WARN_THR_DDR1_VREF			);
	framWrite(FRAM_WARN_THR_VM_DRVDD			, SETTING_WARN_THR_VM_DRVDD				);
	framWrite(FRAM_WARN_THR_VIN_SCALED			, SETTING_WARN_THR_VIN_SCALED			);
	framWrite(FRAM_WARN_THR_VM_MAN3V3			, SETTING_WARN_THR_VM_MAN3V3			);
	framWrite(FRAM_WARN_THR_VM_MAN1V8			, SETTING_WARN_THR_VM_MAN1V8			);
	framWrite(FRAM_WARN_THR_MON_5V0				, SETTING_WARN_THR_MON_5V0				);
	framWrite(FRAM_WARN_THR_MGT_AV				, SETTING_WARN_THR_MGT_AV				);
	framWrite(FRAM_WARN_THR_MGT_AVTT			, SETTING_WARN_THR_MGT_AVTT				);
	framWrite(FRAM_WARN_THR_INTERNAL_MCU_TEMP	, SETTING_WARN_THR_INTERNAL_MCU_TEMP	);
	framWrite(FRAM_WARN_THR_BOARD_TEMP			, SETTING_WARN_THR_BOARD_TEMP			);
	framWrite(FRAM_WARN_THR_FPGA0_TEMP			, SETTING_WARN_THR_FPGA0_TEMP			);
	framWrite(FRAM_WARN_THR_FPGA1_TEMP			, SETTING_WARN_THR_FPGA1_TEMP			);
	
	framWrite(FRAM_ALARM_THR_SW_AVDD1			, SETTING_ALARM_THR_SW_AVDD1			);
	framWrite(FRAM_ALARM_THR_SW_AVDD2			, SETTING_ALARM_THR_SW_AVDD2			);
	framWrite(FRAM_ALARM_THR_AVDD3				, SETTING_ALARM_THR_AVDD3				);
	framWrite(FRAM_ALARM_THR_MAN_1V2			, SETTING_ALARM_THR_MAN_1V2				);
	framWrite(FRAM_ALARM_THR_DDR0_VREF			, SETTING_ALARM_THR_DDR0_VREF			);
	framWrite(FRAM_ALARM_THR_DDR1_VREF			, SETTING_ALARM_THR_DDR1_VREF			);
	framWrite(FRAM_ALARM_THR_VM_DRVDD			, SETTING_ALARM_THR_VM_DRVDD			);
	framWrite(FRAM_ALARM_THR_VIN_SCALED			, SETTING_ALARM_THR_VIN_SCALED			);
	framWrite(FRAM_ALARM_THR_VM_MAN3V3			, SETTING_ALARM_THR_VM_MAN3V3			);
	framWrite(FRAM_ALARM_THR_VM_MAN1V8			, SETTING_ALARM_THR_VM_MAN1V8			);
	framWrite(FRAM_ALARM_THR_MON_5V0			, SETTING_ALARM_THR_MON_5V0				);
	framWrite(FRAM_ALARM_THR_MGT_AV				, SETTING_ALARM_THR_MGT_AV				);
	framWrite(FRAM_ALARM_THR_MGT_AVTT			, SETTING_ALARM_THR_MGT_AVTT			);
	framWrite(FRAM_ALARM_THR_INTERNAL_MCU_TEMP	, SETTING_ALARM_THR_INTERNAL_MCU_TEMP	);
	framWrite(FRAM_ALARM_THR_BOARD_TEMP			, SETTING_ALARM_THR_BOARD_TEMP			);
	framWrite(FRAM_ALARM_THR_FPGA0_TEMP			, SETTING_ALARM_THR_FPGA0_TEMP			);
	framWrite(FRAM_ALARM_THR_FPGA1_TEMP			, SETTING_ALARM_THR_FPGA1_TEMP			);
	
}

void SKAsystemMonitorStart(){
	// Create Struct
	
	DEBUG_PRINT("Load Voltages and Temperature Settings\n");
	
	for (int i = 0; i < (ADCCOLUMNS + TEMPS_SENSOR); i++){
		VoltagesTemps[i].enabled = false;
	}
	
	// Voltages	
	// ADC4 - PA04 - SW_AVDD1	
	VoltagesTemps[0].ADCpin				= 4;
	VoltagesTemps[0].divider			= 0;
	VoltagesTemps[0].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_SW_AVDD1>>16);
	VoltagesTemps[0].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_SW_AVDD1);
	VoltagesTemps[0].warningTHRupper	= uint16_t(SETTING_WARN_THR_SW_AVDD1>>16);
	VoltagesTemps[0].warningTHRdowner	= uint16_t(SETTING_WARN_THR_SW_AVDD1);
	
	// ADC5 - PA05 - SW_AVDD2
	VoltagesTemps[1].ADCpin				= 5;
	VoltagesTemps[1].divider			= 0;
	VoltagesTemps[1].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_SW_AVDD2>>16);
	VoltagesTemps[1].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_SW_AVDD2);
	VoltagesTemps[1].warningTHRupper	= uint16_t(SETTING_WARN_THR_SW_AVDD2>>16);
	VoltagesTemps[1].warningTHRdowner	= uint16_t(SETTING_WARN_THR_SW_AVDD2);
	
	// ADC6 - PA06 - AVDD3
	VoltagesTemps[2].ADCpin				= 6;
	VoltagesTemps[2].divider			= 0;
	VoltagesTemps[2].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_AVDD3>>16);
	VoltagesTemps[2].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_AVDD3);
	VoltagesTemps[2].warningTHRupper	= uint16_t(SETTING_WARN_THR_AVDD3>>16);
	VoltagesTemps[2].warningTHRdowner	= uint16_t(SETTING_WARN_THR_AVDD3);
	
	// ADC7 - PA07 - MAN_1V2
	VoltagesTemps[3].ADCpin				= 7;
	VoltagesTemps[3].divider			= 0;
	VoltagesTemps[3].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_MAN_1V2>>16);
	VoltagesTemps[3].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_MAN_1V2);
	VoltagesTemps[3].warningTHRupper	= uint16_t(SETTING_WARN_THR_MAN_1V2>>16);
	VoltagesTemps[3].warningTHRdowner	= uint16_t(SETTING_WARN_THR_MAN_1V2);
	VoltagesTemps[3].enabled			= true;
	
	// ADC16 - PA08 - DDR0_VREF
	VoltagesTemps[4].ADCpin				= 16;
	VoltagesTemps[4].divider			= 0;
	VoltagesTemps[4].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_DDR0_VREF>>16);
	VoltagesTemps[4].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_DDR0_VREF);
	VoltagesTemps[4].warningTHRupper	= uint16_t(SETTING_WARN_THR_DDR0_VREF>>16);
	VoltagesTemps[4].warningTHRdowner	= uint16_t(SETTING_WARN_THR_DDR0_VREF);
	
	// ADC17 - PA09 - DDR1_VREF
	VoltagesTemps[5].ADCpin				= 17;
	VoltagesTemps[5].divider			= 0;
	VoltagesTemps[5].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_DDR1_VREF>>16);
	VoltagesTemps[5].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_DDR1_VREF);
	VoltagesTemps[5].warningTHRupper	= uint16_t(SETTING_WARN_THR_DDR1_VREF>>16);
	VoltagesTemps[5].warningTHRdowner	= uint16_t(SETTING_WARN_THR_DDR1_VREF);
	
	// ADC18 - PA10 - VM_DRVDD
	VoltagesTemps[6].ADCpin				= 18;
	VoltagesTemps[6].divider			= 0;
	VoltagesTemps[6].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VM_DRVDD>>16);
	VoltagesTemps[6].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_DRVDD);
	VoltagesTemps[6].warningTHRupper	= uint16_t(SETTING_WARN_THR_VM_DRVDD>>16);
	VoltagesTemps[6].warningTHRdowner	= uint16_t(SETTING_WARN_THR_VM_DRVDD);
	VoltagesTemps[6].enabled			= true;
	
	// ADC8 - PB00 - VIN_SCALED
	VoltagesTemps[7].ADCpin				= 8;
	VoltagesTemps[7].divider			= 12.5; // 12000/960
	VoltagesTemps[7].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VIN_SCALED>>16);
	VoltagesTemps[7].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VIN_SCALED);
	VoltagesTemps[7].warningTHRupper	= uint16_t(SETTING_WARN_THR_VIN_SCALED>>16);
	VoltagesTemps[7].warningTHRdowner	= uint16_t(SETTING_WARN_THR_VIN_SCALED);
	VoltagesTemps[7].enabled			= true;
	
	// ADC9 - PB01 - VM_MAN3V3
	VoltagesTemps[8].ADCpin				= 9;
	VoltagesTemps[8].divider			= 0;
	VoltagesTemps[8].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VM_MAN3V3>>16);
	VoltagesTemps[8].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_MAN3V3);
	VoltagesTemps[8].warningTHRupper	= uint16_t(SETTING_WARN_THR_VM_MAN3V3>>16);
	VoltagesTemps[8].warningTHRdowner	= uint16_t(SETTING_WARN_THR_VM_MAN3V3);
	
	// ADC10 - PB02 - VM_MAN1V8
	VoltagesTemps[9].ADCpin				= 10;
	VoltagesTemps[9].divider			= 0;
	VoltagesTemps[9].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VM_MAN1V8>>16);
	VoltagesTemps[9].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_MAN1V8);
	VoltagesTemps[9].warningTHRupper	= uint16_t(SETTING_WARN_THR_VM_MAN1V8>>16);
	VoltagesTemps[9].warningTHRdowner	= uint16_t(SETTING_WARN_THR_VM_MAN1V8);
	
	// ADC11 - PB03 - MON_5V0
	VoltagesTemps[10].ADCpin			= 11;
	VoltagesTemps[10].divider			= 2.739726; // 5000/1825)
	VoltagesTemps[10].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_MON_5V0>>16);
	VoltagesTemps[10].alarmTHRdowner	= uint16_t(SETTING_ALARM_THR_MON_5V0);
	VoltagesTemps[10].warningTHRupper	= uint16_t(SETTING_WARN_THR_MON_5V0>>16);
	VoltagesTemps[10].warningTHRdowner	= uint16_t(SETTING_WARN_THR_MON_5V0);
	VoltagesTemps[10].enabled			= true;
	
	// ADC14 - PB06 - MGT_AV
	VoltagesTemps[11].ADCpin			= 14;
	VoltagesTemps[11].divider			= 0;
	VoltagesTemps[11].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_MGT_AV>>16);
	VoltagesTemps[11].alarmTHRdowner	= uint16_t(SETTING_ALARM_THR_MGT_AV);
	VoltagesTemps[11].warningTHRupper	= uint16_t(SETTING_WARN_THR_MGT_AV>>16);
	VoltagesTemps[11].warningTHRdowner	= uint16_t(SETTING_WARN_THR_MGT_AV);
	
	// ADC15 - PB07 - MGT_AVTT
	VoltagesTemps[12].ADCpin			= 15;
	VoltagesTemps[12].divider			= 0;
	VoltagesTemps[12].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_MGT_AVTT>>16);
	VoltagesTemps[12].alarmTHRdowner	= uint16_t(SETTING_ALARM_THR_MGT_AVTT);
	VoltagesTemps[12].warningTHRupper	= uint16_t(SETTING_WARN_THR_MGT_AVTT>>16);
	VoltagesTemps[12].warningTHRdowner	= uint16_t(SETTING_WARN_THR_MGT_AVTT);
	
	// Temperatures	
	// ADC24 - INT 0x18 - Internal Temperature (Need option to enable)
	VoltagesTemps[13].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_INTERNAL_MCU_TEMP>>16);
	VoltagesTemps[13].warningTHRupper	= uint16_t(SETTING_WARN_THR_INTERNAL_MCU_TEMP>>16);
	VoltagesTemps[13].enabled			= true;
	
	VoltagesTemps[14].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_BOARD_TEMP>>16);
	VoltagesTemps[14].warningTHRupper	= uint16_t(SETTING_WARN_THR_BOARD_TEMP>>16);
	
	VoltagesTemps[15].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA0_TEMP>>16);
	VoltagesTemps[15].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA0_TEMP>>16);
	
	VoltagesTemps[16].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA1_TEMP>>16);
	VoltagesTemps[16].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA1_TEMP>>16);
	
	DEBUG_PRINT2("Temps and Voltages Loaded:\n");
	for (int i = 0; i < (ADCCOLUMNS + TEMPS_SENSOR); i++){
		VoltagesTemps[i].ADCread = 0;
		DEBUG_PRINT2("PIN %d - Divider %f\n", VoltagesTemps[i].ADCpin, VoltagesTemps[i].divider);
	}
	
	ADCstart();
	
}

int SKAenableCheck(void){
	uint32_t ret, bypass;
	XO3_Read(itpm_cpld_regfile_enable, &ret);
	framRead(FRAM_TPM_ENABLE_BYPASS, &bypass);
	
	if (!TPMpowerLock){
		XO3_WriteByte(itpm_cpld_regfile_enable_shadow, ret);
		DEBUG_PRINT("Powered devices - %x\n", ret);
		return 0;
	}
	else if (bypass == ENABLE_BYPASS_MAGIC){
		XO3_WriteByte(itpm_cpld_regfile_enable_shadow, ret);
		DEBUG_PRINT("Powered devices - %x - BYPASS ENFORCED\n", ret);
		return 1;
	}
	else {
		XO3_WriteByte(itpm_cpld_regfile_enable, 0x0);
		DEBUG_PRINT("Power request DENIED for %x - Power Locked\n", ret);
		return -1;
	}
}

static void IRQfromFPGA(void){
		irqExternalFPGA = true;
}

void IRQinternalFPGAhandler(void){
		uint32_t irq_status, irq_mask;
		
		XO3_Read(itpm_cpld_intc_status, &irq_status);
		XO3_Read(itpm_cpld_intc_mask, &irq_mask);
		DEBUG_PRINT2("IRQ FPGA Val: %x - Mask %x\n", irq_status, irq_mask);
		
		// Call
		
		if (!(irq_mask & (irq_status & ENABLE_UPDATE_int))) SKAenableCheck(); // Verify if Enable can be 
		
		
		XO3_WriteByte(itpm_cpld_intc_ack, MASK_default_int); // Clean FPGA IRQ
		
		irqExternalFPGA = false;
}

void StartupStuff(void){
	uint32_t res;
	
	DEBUG_PRINT("\nSKA iTPM 1.6 - Debug Enabled\n");
	DEBUG_PRINT("Debug level: %d\n", (int) DEBUG);
	
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xF0, &res, i2c2);
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xF0, &res, i2c3);	

	DEBUG_PRINT("Version: %x\n", _build_version);
	DEBUG_PRINT("Date: %x\n", _build_date);
	DEBUG_PRINT("GIT Hash: %x\n", BUILD_GIT_HASH);
	framWrite(FRAM_MCU_VERSION, _build_version);
	framWrite(FRAM_MCU_COMPILE_DATE, _build_date);
	framWrite(FRAM_MCU_GIT_HASH, BUILD_GIT_HASH);
	
	XO3_Read(itpm_cpld_regfile_date_code, &res);
	DEBUG_PRINT("CPLD Version: %x\n", res);
	DEBUG_PRINT("-------------------------------\n\n");
	
	gpio_set_pin_level(USR_LED1, true);
	

	
	SKAsystemMonitorStart();
	
	StartupLoadSettings();
	
	// Interrupt Enable
	XO3_WriteByte(itpm_cpld_intc_mask, (itpm_cpld_intc_mask_M - ENABLE_UPDATE_int));
	XO3_WriteByte(itpm_cpld_intc_ack, MASK_default_int);	
	ext_irq_register(XO3_LINK0, IRQfromFPGA);	
	
	
	
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xFE, &res, i2c2);
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xFE, &res, i2c3);
	
	DEBUG_PRINT("Startup Done\n");	
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;

static void IRQtimerSlow(const struct timer_task *const timer_task){
	irqTimerSlow = true; // Enable Task
}

void taskSlow(){
	uint32_t res;
	gpio_toggle_pin_level(USR_LED0);
	
	framRead(FRAM_MCU_VERSION, &res);
	if (res == _build_version){
		TWIdataBlock();
		exchangeDataBlock();
		
		framRead(FRAM_MCU_POOLING_INTERVAL, &pollingNew);
	}
	else DEBUG_PRINT1("ERROR no SPI bus comunication. Expected MCU version %x read %x\n", _build_version, res);
	

	//DEBUG_PRINT3("FRAM_MCU_POOLING_INTERVAL > %x\n", pollingNew);
	if(pollingNew > 2000){
		pollingNew = 2000;
	}
	if (pollingOld != pollingNew){
		timer_stop(&TIMER_0);
		TIMER_0_task1.interval = pollingNew;
		timer_start(&TIMER_0);
		pollingOld = pollingNew;
		DEBUG_PRINT("Pooling Time Changed to %d\n", pollingNew);
	}
	
	irqTimerSlow = false; // Disable Task untile next IRQ
}

void WDT_Handler(void){
	asm("nop");
}

int main(void)
{
	// Cheap delay startup ~1000 ms total
	for (int i = 0; i < 0xffff; i++) asm("nop");
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	//SysTick_Config(1);
	
	StartupStuff();
	
	uint32_t vers;
	
	
	
	
	
	uint32_t mtime, xil;
	uint32_t xil_done = 0xfeffffff;

	//XO3_WriteByte(itpm_cpld_regfile_enable, 0x1f);
	
	//delay_ms(1000);
	
// 	while (1){
// 		XO3_WriteByte(itpm_cpld_regfile_user_reg0, 0x12345678);
// 		XO3_Read(itpm_cpld_regfile_user_reg0, &xil_done);
// 	}
	
	XO3_Read(itpm_cpld_regfile_xilinx, &xil);
	XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
	
	//ExtFlash_SRAMErase(0x01);
	//ExtFlash_FPGA_Prog(0x0f, 0x1, true);
	
	xil_done = 0xffffffff;	
	XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
	XO3_Read(itpm_cpld_regfile_xilinx, &xil);

	/* Replace with your application code */
	
	
	TIMER_0_task1.interval = 1000;
	TIMER_0_task1.cb       = IRQtimerSlow;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
// 	TIMER_0_task2.interval = 200;
// 	TIMER_0_task2.cb       = TIMER_0_task2_cb;
// 	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
/*	timer_add_task(&TIMER_0, &TIMER_0_task2);*/
	timer_start(&TIMER_0);
	
	DEBUG_PRINT("Default Polling Time set to %x\n", pollingOld);	

	
	while (1) {
		uint32_t vers;
		gpio_toggle_pin_level(USR_LED1);

		ADCreadSingle();
		
		if (irqExternalFPGA) IRQinternalFPGAhandler();
		if (irqTimerSlow) taskSlow();
		
	}
}
