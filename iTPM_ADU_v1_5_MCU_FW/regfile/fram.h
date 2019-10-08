/*
 * fram.h
 *
 * Sanitas EG - SKA Management Board
 *
 * Created: 08/10/2019 10:37:19 ~ Luca Schettini
 *
 * Copyright (c) 2017-2019 Sanitas EG srl.  All right reserved.
 *
 */ 



#ifndef FRAM_H_
#define FRAM_H_

// All registers are 32bit spaced. The size in comment refers to the shadow TWI register

// i2c1
#define FRAM_MCU_VERSION					0x000000000 //R  - 32 bit
#define FRAM_MCU_COMPILE_DATE				0x000000004 //R  - 32 bit

#define FRAM_BOARD_STATUS					0x000000008 //RW - 32 bit
#define FRAM_BOARD_ALARM					0x00000000C //RW - 32 bit

#define FRAM_MCU_INTERNAL_COMMAND			0x000000010 //RW - 32 bit
#define	FRAM_MCU_GPR_0						0x000000014 //RW - 32 bit
#define	FRAM_MCU_GPR_1						0x000000018 //RW - 32 bit
#define	FRAM_MCU_GPR_2						0x00000001C //RW - 32 bit
#define	FRAM_MCU_GPR_3						0x000000020 //RW - 32 bit

#define FRAM_ADC_SW_AVDD1					0x000000020 //RW - 16 bit
#define FRAM_ADC_SW_AVDD2					0x000000024 //RW - 16 bit
#define FRAM_ADC_AVDD3						0x000000028 //RW - 16 bit
#define FRAM_ADC_MAN_1V2					0x00000002C //RW - 16 bit
#define FRAM_ADC_DDR0_VREF					0x000000030 //RW - 16 bit
#define FRAM_ADC_DDR1_VREF					0x000000034 //RW - 16 bit
#define FRAM_ADC_VM_DRVDD					0x000000038 //RW - 16 bit
#define FRAM_ADC_VIN_SCALED					0x00000003C //RW - 16 bit
#define FRAM_ADC_VM_MAN3V3					0x000000040 //RW - 16 bit
#define FRAM_ADC_VM_MAN1V8					0x000000044 //RW - 16 bit
#define FRAM_ADC_MON_5V0					0x000000048 //RW - 16 bit
#define FRAM_ADC_MGT_AV						0x00000004C //RW - 16 bit
#define FRAM_ADC_MGT_AVTT					0x000000050 //RW - 16 bit
#define FRAM_ADC_INTERNAL_MCU_TEMP			0x000000054 //RW - 16 bit
#define FRAM_BOARD_TEMP						0x000000058 //RW - 32 bit
#define FRAM_FPGA0_TEMP						0x00000005C //RW - 16 bit
#define FRAM_FPGA1_TEMP						0x000000060 //RW - 16 bit

#define THRESHOLD_ENABLE_MASK				0x000000064 //RW - 17 bit
#define FRAM_ALARM_THR_SW_AVDD1				0x000000020 //RW - 32 bit
#define FRAM_ALARM_THR_SW_AVDD2				0x000000024 //RW - 32 bit
#define FRAM_ALARM_THR_AVDD3				0x000000028 //RW - 32 bit
#define FRAM_ALARM_THR_MAN_1V2				0x00000002C //RW - 32 bit
#define FRAM_ALARM_THR_DDR0_VREF			0x000000030 //RW - 32 bit
#define FRAM_ALARM_THR_DDR1_VREF			0x000000034 //RW - 32 bit
#define FRAM_ALARM_THR_VM_DRVDD				0x000000038 //RW - 32 bit
#define FRAM_ALARM_THR_VIN_SCALED			0x00000003C //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN3V3			0x000000040 //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN1V8			0x000000044 //RW - 32 bit
#define FRAM_ALARM_THR_MON_5V0				0x000000048 //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AV				0x00000004C //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AVTT				0x000000050 //RW - 32 bit
#define FRAM_ALARM_THR_INTERNAL_MCU_TEMP	0x000000054 //RW - 32 bit
#define FRAM_ALARM_THR_BOARD_TEMP			0x000000058 //RW - 32 bit
#define FRAM_ALARM_THR_FPGA0_TEMP			0x00000005C //RW - 32 bit
#define FRAM_ALARM_THR_FPGA1_TEMP			0x000000060 //RW - 32 bit 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 





#endif /* FRAM_H_ */