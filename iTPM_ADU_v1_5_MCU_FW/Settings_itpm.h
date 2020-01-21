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
// THREASHOLDS = 32 bit - [31:15]Upper Threshold - [14:0] Lower Threshold

const uint8_t SETTING_WARN_ALARM_UPDATE			= 0x0;

// Supply

const uint32_t SETTING_WARN_THR_SW_AVDD1			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_SW_AVDD2			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_AVDD3				= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_MAN_1V2				= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_DDR0_VREF			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_DDR1_VREF			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_VM_DRVDD			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_VIN_SCALED			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_VM_MAN3V3			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_VM_MAN1V8			= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_MON_5V0				= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_MGT_AV				= 0xEEEEEEEE;
const uint32_t SETTING_WARN_THR_MGT_AVTT			= 0xEEEEEEEE;

const uint32_t SETTING_ALARM_THR_SW_AVDD1			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_SW_AVDD2			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_AVDD3				= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_MAN_1V2			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_DDR0_VREF			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_DDR1_VREF			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_VM_DRVDD			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_VIN_SCALED			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_VM_MAN3V3			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_VM_MAN1V8			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_MON_5V0			= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_MGT_AV				= 0xEEEEEEEE;
const uint32_t SETTING_ALARM_THR_MGT_AVTT			= 0xEEEEEEEE;

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