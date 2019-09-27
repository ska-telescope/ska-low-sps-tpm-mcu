#include <atmel_start.h>
#include "SpiRouter.h"
#include "regfile.h"

uint16_t adcArrgh[2][18] = {
	{
		11, //PB03 - LED0_ILL
		10, //PB02 - LED0_TEMP
		
		8, //PB00 - LED1_ILL
		9, //PB01 - LED1_TEMP
		
		12, //PB04 - LED2_ILL
		13, //PB05 - LED2_TEMP
		
		14, //PB06 - LED3_ILL
		15, //PB07 - LED3_TEMP
		
		2, //PB08 - LED4_ILL
		3, //PB09 - LED4_TEMP
		
		6, //PA06 - LED5_ILL
		19, //PA11 - LED5_TEMP
		
		7, //PA07 - VIN_MON
		18, //PA10 - MON_5V
		26, //INT 0x1A - SCALED CORE VCC
		27, //INT 0x1B - SCALED IO VCC
		24, //INT 0x18 - INTERNAL TEMP
		
		5, //PA05 - USER_PTC_READ
	},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}   /*  initializers for row indexed by 1 */
};

int anaReadPos = 0;
bool anaNotReady = true;


static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync() {
	while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
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
		if (anaReadPos >= 16){
			anaReadPos = 0;
			anaNotReady = false;
		}
		else anaReadPos++;
		ADC->INPUTCTRL.bit.MUXPOS = adcArrgh[0][anaReadPos];
		//ADC->INTFLAG.bit.RESRDY = 1;
		ADC->SWTRIG.bit.START = 1;
	}
}


int main(void)
{
	// Cheap delay startup ~1000 ms total
	for (int i = 0; i < 0xffff; i++) asm("nop");
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	uint32_t vers;
	
	gpio_set_pin_level(USR_LED1, true);

	/* Replace with your application code */
	while (1) {
		
		gpio_toggle_pin_level(USR_LED0);
		uint32_t vers;
		XO3_Read(0x30000010, &vers);
		XO3_WriteByte(0x30000010, 0x01234567);
		XO3_Read(0x30000010, &vers);
		XO3_WriteByte(0x30000010, 0x76543210);
		XO3_Read(0x30000010, &vers);
		delay_ms(100);
	}
}
