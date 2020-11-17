/*
 * Settings_itpm.h
 *
 * Created: 20/01/2020 16:49:20
 *  Author: luca
 */ 


#ifndef SETTINGS_ITPM_H_
#define SETTINGS_ITPM_H_

const uint16_t DEFAULT_POLLING_INTERVAL = 1000;

// ---------------------------------------------------------------------------
enum VoltTemps{SWAVDD1, SWAVDD2, SWAVDD3, MAN1V2, DDR0VREF, DDR1VREF, VMDRVDD, VINSCALED, MAN3V3, MAN1V8, MON5V0, MGTAV, MGAVTT, INTTEMP, BOARDTEMP, FPGA0TEMP, FPGA1TEMP}; 
	

enum IRQfpgaMask{
		I2C_int = 0x1,
		UART_RX_int = 0x2,
		WDT_SEM_EXPIRED_int = 0x4,
		ALARM_UPDATE_int = 0x8,
		ENABLE_UPDATE_int = 0x10,
		TMP_EVENT_INT = 0x20,
		XIL0_int = 0x40,
		XIL1_int = 0x80,
		MASK_default_int = 0xfff
	};
	
// VOLTAGE DIVIDERS
const float VIN_SCALED_DIVIDER						= 12.5; // 12000/960
const float VM_MAN3V3_DIVIDER						= 3.74782; // 3300 / 880.51
const float VM_MAN1V8_DIVIDER						= 2.73914; // 1800/657,14
const float MON_5V0_DIVIDER							= 2.739726; // 5000/1825

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
const uint32_t SETTING_WARN_THR_VIN_SCALED			= 0xFFFF0000;
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
const uint32_t SETTING_WARN_THR_BOARD_TEMP			= 0xEEEE0000;
const uint32_t SETTING_WARN_THR_FPGA0_TEMP			= 0xEEEE0000;
const uint32_t SETTING_WARN_THR_FPGA1_TEMP			= 0xEEEE0000;

const uint32_t SETTING_ALARM_THR_INTERNAL_MCU_TEMP	= 0xEEEE0000;
const uint32_t SETTING_ALARM_THR_BOARD_TEMP			= 0xEEEE0000;
const uint32_t SETTING_ALARM_THR_FPGA0_TEMP			= 0xEEEE0000;
const uint32_t SETTING_ALARM_THR_FPGA1_TEMP			= 0xEEEE0000;
// ---------------------------------------------------------------------------

enum power_settings {sunday = 1, monday, tuesday = 5,
wednesday, thursday = 10, friday, saturday};


#endif /* SETTINGS_ITPM_H_ */