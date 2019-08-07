#include <atmel_start.h>

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/* Replace with your application code */
	while (1) {
		
		gpio_toggle_pin_level(USR_LED0);
		delay_ms(1000);
	}
}
