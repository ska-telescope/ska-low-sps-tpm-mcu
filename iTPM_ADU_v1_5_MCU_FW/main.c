#include <atmel_start.h>
#include "SpiRouter.h"
#include "regfile.h"

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
