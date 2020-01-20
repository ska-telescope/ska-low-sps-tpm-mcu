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

#include "build_def.h"
#include "SpiRouter.h"
#include "regfile.h"
#include "TwiFpga.h"
#include "ADT7408.h"

// WARNING - Proper undefine DEBUG in Project properties
#ifdef DEBUG
char bufferOut[512];
#undef DEBUG
#define DEBUG 3 // Possible Choice: 3: Maximum Level - 2: Medium Level - 1: Minimum Level - Only #def: Status
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

const uint32_t _build_version = 0xb0000012;
const uint32_t _build_date = ((((BUILD_YEAR_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH1 & 0xFF - 0x30)) << 24) | (((BUILD_YEAR_CH2 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH3 & 0xFF - 0x30)) << 16) | (((BUILD_MONTH_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_MONTH_CH1 & 0xFF - 0x30)) << 8) | (((BUILD_DAY_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_DAY_CH1 & 0xFF - 0x30))));
const uint32_t _build_time = (0x00 << 24 | (((__TIME__[0] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[1] & 0xFF - 0x30)) << 16) | (((__TIME__[3] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[4] & 0xFF - 0x30)) << 8) | (((__TIME__[6] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[7] & 0xFF - 0x30))));

#define ADCCOLUMNS 14
uint16_t adcArrgh[2][ADCCOLUMNS] = {
	{
		4,  // PA04 -SW_AVDD1
		5,  // PA05 - SW_AVDD2
		6,  // PA06 - AVDD3
		7,  // PA07 - MAN_1V2
		16, // PA08 - DDR0_VREF
		17, // PA09 - DDR1_VREF
		18, // PA10 - VM_DRVDD
		
		8,  // PB00 - VIN_SCALED
		9,  // PB01 - VM_MAN3V3
		10, // PB02 - VM_MAN1V8	
		11, // PB03 - MON_5V0	
		14, // PB06 - MGT_AV
		15, // PB07 - MGT_AVTT	
		
		24, //INT 0x18 - INTERNAL TEMP
	},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}   /*  initializers for row indexed by 1 */
};

float adcDivider[ADCCOLUMNS] = {0, 0, 0, 0, 0, 0, 0, (float)12000/960, 0, 0, (float)5000/1825, 0, 0, 0};

int anaReadPos = 0;
bool anaNotReady = true;

/* --------- CONST------------------- */

const uint32_t DEFAULT_POLLING_INTERVAL = 1000;

/* ---------------------------------- */

/* --------- VAR -------------------- */

static const float ADC_STEP = (2.5/65536); // Ext Ref Voltage (2.5V) / 16Bit ADC 2^16

bool irqTimerSlow = false;
bool irqTimerFast = false;

uint32_t ADT7408_temp_raw;
uint16_t ADT7408_temp;
bool ADT7408Regs[3];
uint32_t pollingHz;
uint32_t reg_ThresholdEnable = 0;
uint16_t reg_ThresholdVals [2][17];
uint32_t pollingOld = 1000;
uint32_t pollingNew = 1000;

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

void exchangeDataBlock(){
	framWrite(FRAM_ADC_SW_AVDD1,			adcArrgh[1][0]);
	framWrite(FRAM_ADC_SW_AVDD2,			adcArrgh[1][1]);
	framWrite(FRAM_ADC_AVDD3,				adcArrgh[1][2]);
	framWrite(FRAM_ADC_MAN_1V2,				adcArrgh[1][3]);
	framWrite(FRAM_ADC_DDR0_VREF,			adcArrgh[1][4]);
	framWrite(FRAM_ADC_DDR1_VREF,			adcArrgh[1][5]);
	framWrite(FRAM_ADC_VM_DRVDD,			adcArrgh[1][6]);
	framWrite(FRAM_ADC_VIN_SCALED,			adcArrgh[1][7]);
	framWrite(FRAM_ADC_VM_MAN3V3,			adcArrgh[1][8]);
	framWrite(FRAM_ADC_VM_MAN1V8,			adcArrgh[1][9]);
	framWrite(FRAM_ADC_MON_5V0,				adcArrgh[1][10]);
	framWrite(FRAM_ADC_MGT_AV,				adcArrgh[1][11]);
	framWrite(FRAM_ADC_MGT_AVTT,			adcArrgh[1][12]);
	framWrite(FRAM_ADC_INTERNAL_MCU_TEMP,	adcArrgh[1][13]);
	
	framWrite(FRAM_BOARD_TEMP, ADT7408_temp_raw);
	
// 	framRead(FRAM_THRESHOLD_ENABLE_MASK, &reg_ThresholdEnable);
// 	if (reg_ThresholdEnable && 0x80000000) {
// 		int x = 0;
// 		for (uint32_t i = FRAM_ALARM_THR_SW_AVDD1; i < 0x1D8; i += 4){
// 			uint32_t temp;
// 			framRead(i, &temp);
// 			reg_ThresholdVals[0][x] = (temp && 0xFFFF); // High Threshold
// 			reg_ThresholdVals[1][x] = (temp && 0xFFFF0000) >> 16; // Low Threshold
// 			x++;
// 		}
// 	}
	
}

void analogRead() { // Single read, much FASTER
	if (ADC->INTFLAG.bit.RESRDY == 1){
		//uint32_t valueRead = ADC->RESULT.reg;
		//adcArrgh[1][anaReadPos] = ADC->RESULT.reg; // Save ADC read to the array
		
		float ADC_voltage = (ADC_STEP * ADC->RESULT.reg) * 1000;
		///??? float Voltage = (ADC_voltage * ADC_STEP) * 1000;
		
		if (adcDivider[anaReadPos] == 0) adcArrgh[1][anaReadPos] = (uint16_t)ADC_voltage;
		else adcArrgh[1][anaReadPos] = (uint16_t)(ADC_voltage * adcDivider[anaReadPos]);
		
		
		ADCsync();
		ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
		if (anaReadPos > ADCCOLUMNS -1){
			anaReadPos = 0;
			anaNotReady = false;
		}
		else anaReadPos++;
		ADC->INPUTCTRL.bit.MUXPOS = adcArrgh[0][anaReadPos];
		ADC->INTFLAG.bit.RESRDY = 1;
		ADC->SWTRIG.bit.START = 1;
	}
}

void analogStart() { // Single read, much FASTER
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
	
	while (anaReadPos < ADCCOLUMNS) analogRead();
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

void StartupStuff(void){
	uint32_t res;
	
	DEBUG_PRINT("\nSKA iTPM 1.6 - Debug Enabled\n");
	DEBUG_PRINT("Debug level: %d\n", (int) DEBUG);
	DEBUG_PRINT("Version: %x\n", _build_version);
	DEBUG_PRINT("Date: %x\n", _build_date);
	DEBUG_PRINT("Time: %x\n", _build_time);
	DEBUG_PRINT("-------------------------------\n\n");
	
	framWrite(FRAM_MCU_VERSION, _build_version);
	framWrite(FRAM_MCU_COMPILE_DATE, _build_date);
	framWrite(FRAM_MCU_COMPILE_TIME, _build_time);
	
	gpio_set_pin_level(USR_LED1, true);
	
	analogStart();
	
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xCE, NULL, i2c2); // Da provare
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xCE, NULL, i2c3); // Da provare
	
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;

static void timerSlow(const struct timer_task *const timer_task){
	irqTimerSlow = true; // Enable Task
}

void taskSlow(){
	gpio_toggle_pin_level(USR_LED0);
	
	TWIdataBlock();
	exchangeDataBlock();
	
	framRead(FRAM_MCU_POOLING_INTERVAL, &pollingNew);
	DEBUG_PRINT3("FRAM_MCU_POOLING_INTERVAL > %x\n", pollingNew);
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

int main(void)
{
	// Cheap delay startup ~1000 ms total
	for (int i = 0; i < 0xffff; i++) asm("nop");
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	//SysTick_Config(1);
	
	StartupStuff();
	framWrite(FRAM_MCU_POOLING_INTERVAL, DEFAULT_POLLING_INTERVAL);
	
	uint32_t vers;
	
	
	
	
	
	uint32_t mtime, xil;
	uint32_t xil_done = 0xfeffffff;

	//XO3_WriteByte(itpm_cpld_regfile_enable, 0x1f);
	
	delay_ms(1000);
	
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
	TIMER_0_task1.cb       = timerSlow;
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

		analogRead();
		
		if (irqTimerSlow) taskSlow();
		
	}
}
