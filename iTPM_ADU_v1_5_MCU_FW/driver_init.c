/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

/* The channel amount for ADC */
#define ADC_0_CH_AMOUNT 1

/* The buffer size for ADC */
#define ADC_0_BUFFER_SIZE 16

/* The maximal channel number of enabled channels */
#define ADC_0_CH_MAX 0

struct adc_async_descriptor         ADC_0;
struct adc_async_channel_descriptor ADC_0_ch[ADC_0_CH_AMOUNT];
struct spi_m_sync_descriptor        SPI_0;
struct timer_descriptor             TIMER_0;

static uint8_t ADC_0_buffer[ADC_0_BUFFER_SIZE];
static uint8_t ADC_0_map[ADC_0_CH_MAX + 1];

struct usart_sync_descriptor USART_0;

struct usart_sync_descriptor USART_XO3;

struct wdt_descriptor WDT_0;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
void ADC_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
	adc_async_init(&ADC_0, ADC, ADC_0_map, ADC_0_CH_MAX, ADC_0_CH_AMOUNT, &ADC_0_ch[0], (void *)NULL);
	adc_async_register_channel_buffer(&ADC_0, 0, ADC_0_buffer, ADC_0_BUFFER_SIZE);

	// Disable digital pin circuitry
	gpio_set_pin_direction(SW_AVDD1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(SW_AVDD1, PINMUX_PA04B_ADC_AIN4);

	// Disable digital pin circuitry
	gpio_set_pin_direction(SW_AVDD2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(SW_AVDD2, PINMUX_PA05B_ADC_AIN5);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AVDD3, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AVDD3, PINMUX_PA06B_ADC_AIN6);

	// Disable digital pin circuitry
	gpio_set_pin_direction(MAN_1V2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MAN_1V2, PINMUX_PA07B_ADC_AIN7);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VIN_SCALED, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VIN_SCALED, PINMUX_PB00B_ADC_AIN8);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VM_MAN3V3, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VM_MAN3V3, PINMUX_PB01B_ADC_AIN9);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VM_MAN1V8, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VM_MAN1V8, PINMUX_PB02B_ADC_AIN10);

	// Disable digital pin circuitry
	gpio_set_pin_direction(MON_5V0, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MON_5V0, PINMUX_PB03B_ADC_AIN11);

	// Disable digital pin circuitry
	gpio_set_pin_direction(MGT_AV, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MGT_AV, PINMUX_PB06B_ADC_AIN14);

	// Disable digital pin circuitry
	gpio_set_pin_direction(MGT_AVTT, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MGT_AVTT, PINMUX_PB07B_ADC_AIN15);

	// Disable digital pin circuitry
	gpio_set_pin_direction(DDR0_VREF, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(DDR0_VREF, PINMUX_PA08B_ADC_AIN16);

	// Disable digital pin circuitry
	gpio_set_pin_direction(DDR1_VREF, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(DDR1_VREF, PINMUX_PA09B_ADC_AIN17);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VM_DRVDD, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VM_DRVDD, PINMUX_PA10B_ADC_AIN18);
}

void EXTERNAL_IRQ_0_init(void)
{
	_gclk_enable_channel(EIC_GCLK_ID, CONF_GCLK_EIC_SRC);

	// Set pin direction to input
	gpio_set_pin_direction(XO3_LINK0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(XO3_LINK0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(XO3_LINK0, PINMUX_PA19A_EIC_EXTINT3);

	// Set pin direction to input
	gpio_set_pin_direction(XO3_LINK1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(XO3_LINK1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(XO3_LINK1, PINMUX_PA20A_EIC_EXTINT4);

	// Set pin direction to input
	gpio_set_pin_direction(PG_FPGA, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PG_FPGA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PG_FPGA, PINMUX_PA21A_EIC_EXTINT5);

	// Set pin direction to input
	gpio_set_pin_direction(PG_FE, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PG_FE,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PG_FE, PINMUX_PA24A_EIC_EXTINT12);

	// Set pin direction to input
	gpio_set_pin_direction(PG_AVDD, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PG_AVDD,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PG_AVDD, PINMUX_PA25A_EIC_EXTINT13);

	// Set pin direction to input
	gpio_set_pin_direction(PG_MAN, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PG_MAN,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PG_MAN, PINMUX_PB30A_EIC_EXTINT14);

	// Set pin direction to input
	gpio_set_pin_direction(PG_ADC, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PG_ADC,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PG_ADC, PINMUX_PB31A_EIC_EXTINT15);

	ext_irq_init();
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(USART0_RX, PINMUX_PA17C_SERCOM1_PAD1);

	gpio_set_pin_function(USART0_TX, PINMUX_PA18C_SERCOM1_PAD2);
}

void USART_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM1, (void *)NULL);
	USART_0_PORT_init();
}

void USART_XO3_PORT_init(void)
{

	gpio_set_pin_function(XO3_UART_TX, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_function(XO3_UART_RX, PINMUX_PA23C_SERCOM3_PAD1);
}

void USART_XO3_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
}

void USART_XO3_init(void)
{
	USART_XO3_CLOCK_init();
	usart_sync_init(&USART_XO3, SERCOM3, (void *)NULL);
	USART_XO3_PORT_init();
}

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(FPGA_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(FPGA_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(FPGA_MOSI, PINMUX_PB12C_SERCOM4_PAD0);

	gpio_set_pin_level(FPGA_CLK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(FPGA_CLK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(FPGA_CLK, PINMUX_PB13C_SERCOM4_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(FPGA_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(FPGA_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(FPGA_MISO, PINMUX_PB10D_SERCOM4_PAD2);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM4);
	SPI_0_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC7);
	_gclk_enable_channel(TC7_GCLK_ID, CONF_GCLK_TC7_SRC);

	timer_init(&TIMER_0, TC7, _tc_get_timer());
}

void WDT_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, WDT);
	_gclk_enable_channel(WDT_GCLK_ID, CONF_GCLK_WDT_SRC);
}

void WDT_0_init(void)
{
	WDT_0_CLOCK_init();
	wdt_init(&WDT_0, WDT);
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA27

	gpio_set_pin_level(USR_LED0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(USR_LED0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(USR_LED0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB16

	gpio_set_pin_level(FPGA_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(FPGA_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(FPGA_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB17

	gpio_set_pin_level(BACKPL_RESET,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BACKPL_RESET, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BACKPL_RESET, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	gpio_set_pin_level(BACKPL_PWR_ALERT,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(BACKPL_PWR_ALERT, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(BACKPL_PWR_ALERT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23

	gpio_set_pin_level(USR_LED1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(USR_LED1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(USR_LED1, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();
	EXTERNAL_IRQ_0_init();

	USART_0_init();

	USART_XO3_init();

	SPI_0_init();

	delay_driver_init();

	TIMER_0_init();

	WDT_0_init();
}
