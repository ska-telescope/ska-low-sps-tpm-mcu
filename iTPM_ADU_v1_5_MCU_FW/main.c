#include <atmel_start.h>
#include "SpiRouter.h"
#include "regfile.h"
#include "TwiFpga.h"
#include "ADT7408.h"

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

int anaReadPos = 0;
bool anaNotReady = true;


static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync() {
	while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void framRead(uint32_t fram_register, uint32_t* readback){
	uint32_t intreadback;
	XO3_Read(itpm_cpld_bram_cpu + fram_register, &intreadback);
	*readback = intreadback;
}

void framWrite(uint32_t fram_register, uint32_t writedata){
	XO3_WriteByte(itpm_cpld_bram_cpu + fram_register, writedata);
}


void analogStart() { // Single read, much FASTER
	//REG_PM_APBCMASK |= 0x10000; // Enable bus Clock
	//REG_GCLK_CLKCTRL = 0x4001E;
	REG_ADC_SAMPCTRL = 5;
	ADC->INPUTCTRL.bit.MUXNEG = 0x18; // Mux NEG GND
	SYSCTRL->VREF.bit.TSEN = 1; // Enable TSENOR
	//REG_ADC_REFCTRL = 0;
	//REG_ADC_CTRLA = 2;
	
	ADCsync();
	ADC->CTRLA.bit.ENABLE = 0x01;              // Enable ADC

	ADC->INTFLAG.bit.RESRDY = 1;               // Data ready flag cleared

	ADCsync();
	ADC->SWTRIG.bit.START = 1;                 // Start ADC conversion
}


void analogRead() { // Single read, much FASTER
	if (ADC->INTFLAG.bit.RESRDY == 1){
		//uint32_t valueRead = ADC->RESULT.reg;
		adcArrgh[1][anaReadPos] = ADC->RESULT.reg; // Save ADC read to the array
		ADCsync();
		ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
		if (anaReadPos >= ADCCOLUMNS){
			anaReadPos = 0;
			anaNotReady = false;
		}
		else anaReadPos++;
		ADC->INPUTCTRL.bit.MUXPOS = adcArrgh[0][anaReadPos];
		//ADC->INTFLAG.bit.RESRDY = 1;
		ADC->SWTRIG.bit.START = 1;
	}
}

void TWIdataBlock(void){
	int status;
	uint32_t retvalue = 0xffffffff;

	// i2c1
	status = twiFpgaWrite(0x30, 1, 2, 0x05, &retvalue, i2c1); //temp_value 0x30
	//XO3_WriteByte(fram_ADT7408_M_1_temp_val + fram_offset, retvalue);
	retvalue = 0xffffffff;

	
}


int main(void)
{
	// Cheap delay startup ~1000 ms total
	for (int i = 0; i < 0xffff; i++) asm("nop");
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	framWrite(FRAM_MCU_VERSION, 0xb0000001);
	
	uint32_t vers;
	
	gpio_set_pin_level(USR_LED1, true);

	/* Replace with your application code */
	while (1) {
		
		gpio_toggle_pin_level(USR_LED0);
		XO3_Read(0x30000010, &vers);
		//XO3_WriteByte(itpm_cpld_regfile_enable, 0x1);
		XO3_Read(0x30000010, &vers);
		XO3_WriteByte(itpm_cpld_i2c_transmit, 0x7777777);
		XO3_Read(itpm_cpld_i2c_transmit, &vers); 
		framRead(FRAM_MCU_VERSION, &vers);
		delay_ms(100);
		TWIdataBlock();
		delay_ms(100);
	}
}
