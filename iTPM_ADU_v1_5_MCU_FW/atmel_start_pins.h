/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define SW_AVDD1 GPIO(GPIO_PORTA, 4)
#define SW_AVDD2 GPIO(GPIO_PORTA, 5)
#define AVDD3 GPIO(GPIO_PORTA, 6)
#define MAN_1V2 GPIO(GPIO_PORTA, 7)
#define DDR0_VREF GPIO(GPIO_PORTA, 8)
#define DDR1_VREF GPIO(GPIO_PORTA, 9)
#define VM_DRVDD GPIO(GPIO_PORTA, 10)
#define USART0_RX GPIO(GPIO_PORTA, 17)
#define USART0_TX GPIO(GPIO_PORTA, 18)
#define XO3_LINK0 GPIO(GPIO_PORTA, 19)
#define XO3_LINK1 GPIO(GPIO_PORTA, 20)
#define PG_FPGA GPIO(GPIO_PORTA, 21)
#define XO3_UART_TX GPIO(GPIO_PORTA, 22)
#define XO3_UART_RX GPIO(GPIO_PORTA, 23)
#define PG_FE GPIO(GPIO_PORTA, 24)
#define PG_AVDD GPIO(GPIO_PORTA, 25)
#define USR_LED0 GPIO(GPIO_PORTA, 27)
#define VIN_SCALED GPIO(GPIO_PORTB, 0)
#define VM_MAN3V3 GPIO(GPIO_PORTB, 1)
#define VM_MAN1V8 GPIO(GPIO_PORTB, 2)
#define MON_5V0 GPIO(GPIO_PORTB, 3)
#define MGT_AV GPIO(GPIO_PORTB, 6)
#define MGT_AVTT GPIO(GPIO_PORTB, 7)
#define FPGA_MISO GPIO(GPIO_PORTB, 10)
#define FPGA_MOSI GPIO(GPIO_PORTB, 12)
#define FPGA_CLK GPIO(GPIO_PORTB, 13)
#define BACKPL_RESET GPIO(GPIO_PORTB, 17)
#define BACKPL_PWR_ALERT GPIO(GPIO_PORTB, 22)
#define USR_LED1 GPIO(GPIO_PORTB, 23)
#define PG_MAN GPIO(GPIO_PORTB, 30)
#define PG_ADC GPIO(GPIO_PORTB, 31)

#endif // ATMEL_START_PINS_H_INCLUDED
