/*
 * SKA_iTPM.cpp
 *
 * Created: 06/08/2019 14:14:48
 * Author : luca
 */ 


#include "sam.h"
#include "atmel_start.h"
#include "regfile.h"
#include "SpiRouter.h"
#include "SystemInit.h"

int main(void)
{
    /* Initialize the SAM system */
    SystemInitP();
	
	
	atmel_start_init();
	
	uint32_t vers;
	//XO3_Read(0x0, &vers);	
	
	gpio_set_pin_level(USR_LED0, true);
	
    /* Replace with your application code */
    while (1) 
    {
		gpio_toggle_pin_level(USR_LED0);
		gpio_toggle_pin_level(USR_LED1);
		delay_ms(1000);
    }
}
