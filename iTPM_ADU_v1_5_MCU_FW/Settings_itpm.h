/*
 * Settings_itpm.h
 *
 * Created: 20/01/2020 16:49:20
 *  Author: luca
 */ 


#ifndef SETTINGS_ITPM_H_
#define SETTINGS_ITPM_H_

/*
 * ----- RAM BOOT Map ----------------------------
 */
static uint32_t *ram = (uint32_t *)HMCRAMC0_ADDR;

#define RAM_BOOT_TYPE				ram[0]
#define RAM_BOOT_TYPE_SHIFT			ram[1]
#define RAM_DATA1					ram[2]
#define RAM_DATA2					ram[3]
#define RAM_DATA3					ram[4]
#define RAM_BL_VERSION				ram[6]
#define RAM_STATUS					ram[7]

#define BL_REQUEST_COPYNV			0xFEEDB007
#define BL_REQUEST_BOOT				0xB007B007

const uint16_t DEFAULT_POLLING_INTERVAL = 1000;

// ---------------------------------------------------------------------------
enum VoltTemps{SWAVDD1, SWAVDD2, SWAVDD3, MAN1V2, DDR0VREF, DDR1VREF, VMDRVDD, VINSCALED, MAN3V3, MAN1V8, MON5V0, MGTAV, MGAVTT, INTTEMP, BOARDTEMP, FPGA0TEMP, FPGA1TEMP, FPGA0FEVA, FPGA1FEVA}; 
	

enum IRQfpgaMask{
		I2C_int = 0x1,
		UART_RX_int = 0x2,
		WDT_SEM_EXPIRED_int = 0x4,
		FRAM_UPDATE_int = 0x8,
		ENABLE_UPDATE_int = 0x10,
		TMP_EVENT_INT = 0x20,
		XIL0_int = 0x40,
		XIL1_int = 0x80,
		SingleWireErr_int = 0x100,
		MASK_default_int = 0xfff
	};
	
enum EnableBitmask{
	EN_ADC	= 0x1,
	EN_FE	= 0x2,
	EN_FPGA	= 0x4,
	EN_SYSR	= 0x8,
	EN_VGA	= 0x10
	};
	
enum PowerGoodBitmaks{
	PG_MAN_irq	= 0x1,
	PG_FPGA_irq = 0x2,
	PG_FE_irq	= 0x4,
	PG_AVDD_irq	= 0x8,
	PG_ADC_irq	= 0x10
	};
	
// VOLTAGE DIVIDERS - MULTIPLIERS
const float VIN_SCALED_DIVIDER						= 12.5; // 12000/960
const float VM_MAN3V3_DIVIDER						= 3.74782; // 3300 / 880.51
const float VM_MAN1V8_DIVIDER						= 2.73914; // 1800/657,14
const float MON_5V0_DIVIDER							= 2.739726; // 5000/1825
const float FPGA_FE_CURRENT_MULTPLIER				= (1/65536/0.4);
const float FPGA_TEMP_MULTIPLIER					= ((501.37/65536)-273.677);

// THREASHOLDS = 32 bit - [31:15]Upper Threshold - [14:0] Lower Threshold

const uint8_t SETTING_WARN_ALARM_UPDATE			= 0x0;
const uint32_t ENABLE_BYPASS_MAGIC				= 0xDEAD70CC;

// Supply

const uint32_t SETTING_WARN_THR_SW_AVDD1			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_SW_AVDD2			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_AVDD3				= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_MAN_1V2				= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_DDR0_VREF			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_DDR1_VREF			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_VM_DRVDD			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_VIN_SCALED			= 0x2EE02710; //12 V, 10V
const uint32_t SETTING_WARN_THR_VM_MAN3V3			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_VM_MAN1V8			= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_MON_5V0				= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_MGT_AV				= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_MGT_AVTT			= 0xFFFF0000;

const uint32_t SETTING_ALARM_THR_SW_AVDD1			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_SW_AVDD2			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_AVDD3				= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_MAN_1V2			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_DDR0_VREF			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_DDR1_VREF			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_VM_DRVDD			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_VIN_SCALED			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_VM_MAN3V3			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_VM_MAN1V8			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_MON_5V0			= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_MGT_AV				= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_MGT_AVTT			= 0xFFFF0000;

// Temps
const uint32_t SETTING_WARN_THR_INTERNAL_MCU_TEMP	= 0xEEEE0000;
const uint32_t SETTING_WARN_THR_BOARD_TEMP			= 0x04b00000; // 75 °C
const uint32_t SETTING_WARN_THR_FPGA0_TEMP			= 0xB9B10000; // 90 C° 
const uint32_t SETTING_WARN_THR_FPGA1_TEMP			= 0xB9B10000; // 90 C° 

const uint32_t SETTING_ALARM_THR_INTERNAL_MCU_TEMP	= 0xEEEE0000;
const uint32_t SETTING_ALARM_THR_BOARD_TEMP			= 0x05000000; // 80 °C
const uint32_t SETTING_ALARM_THR_FPGA0_TEMP			= 0xBC3F0000; // 95 C°
const uint32_t SETTING_ALARM_THR_FPGA1_TEMP			= 0xBC3F0000; // 95 C°

// FPGA FE_CURRENTS
const uint32_t SETTING_WARN_THR_FPGA0_FE_CURRENT	= 0xFFFF0000;
const uint32_t SETTING_WARN_THR_FPGA1_FE_CURRENT	= 0xFFFF0000;

const uint32_t SETTING_ALARM_THR_FPGA0_FE_CURRENT	= 0xFFFF0000;
const uint32_t SETTING_ALARM_THR_FPGA1_FE_CURRENT	= 0xFFFF0000;

// ---------------------------------------------------------------------------

#endif /* SETTINGS_ITPM_H_ */