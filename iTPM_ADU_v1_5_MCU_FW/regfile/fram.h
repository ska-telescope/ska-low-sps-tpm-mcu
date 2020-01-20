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

// All registers are 32bit spaced. The size in comment refers to the shadow register (256 Regs MAX - 0x400)

#define FRAM_MCU_VERSION					0x000000000 //R  - 32 bit
#define FRAM_MCU_COMPILE_DATE				0x000000004 //R  - 32 bit
#define FRAM_MCU_COMPILE_TIME				0x000000008 //R  - 32 bit
#define	FRAM_MCU_GPR_0						0x00000000C //RW - 32 bit
#define	FRAM_MCU_GPR_1						0x000000010 //RW - 32 bit
#define	FRAM_MCU_GPR_2						0x000000014 //RW - 32 bit
#define	FRAM_MCU_GPR_3						0x000000018 //RW - 32 bit
#define FRAM_MCU_POOLING_INTERVAL			0x00000001C //RW - 32 bit
#define FRAM_MCU_INTERNAL_COMMAND			0x000000020 //RW - 32 bit

#define FRAM_BOARD_STATUS					0x0000000E0 //RW - 32 bit
#define FRAM_BOARD_ALARM					0x0000000E4 //RW - 32 bit 

#define FRAM_ADC_SW_AVDD1					0x000000100 //RW - 16 bit
#define FRAM_ADC_SW_AVDD2					0x000000104 //RW - 16 bit
#define FRAM_ADC_AVDD3						0x000000108 //RW - 16 bit
#define FRAM_ADC_MAN_1V2					0x00000010C //RW - 16 bit
#define FRAM_ADC_DDR0_VREF					0x000000110 //RW - 16 bit
#define FRAM_ADC_DDR1_VREF					0x000000114 //RW - 16 bit
#define FRAM_ADC_VM_DRVDD					0x000000118 //RW - 16 bit
#define FRAM_ADC_VIN_SCALED					0x00000011C //RW - 16 bit
#define FRAM_ADC_VM_MAN3V3					0x000000120 //RW - 16 bit
#define FRAM_ADC_VM_MAN1V8					0x000000124 //RW - 16 bit
#define FRAM_ADC_MON_5V0					0x000000128 //RW - 16 bit
#define FRAM_ADC_MGT_AV						0x00000012C //RW - 16 bit
#define FRAM_ADC_MGT_AVTT					0x000000130 //RW - 16 bit
#define FRAM_ADC_INTERNAL_MCU_TEMP			0x000000134 //RW - 16 bit
#define FRAM_BOARD_TEMP						0x000000138 //RW - 32 bit
#define FRAM_FPGA0_TEMP						0x00000013C //RW - 16 bit
#define FRAM_FPGA1_TEMP						0x000000140 //RW - 16 bit

#define FRAM_WARN_ALARM_UPDATE				0x000000190 //RW - 32 bit

#define FRAM_WARN_THR_SW_AVDD1				0x000000194 //RW - 32 bit
#define FRAM_WARN_THR_SW_AVDD2				0x000000198 //RW - 32 bit
#define FRAM_WARN_THR_AVDD3					0x00000019C //RW - 32 bit
#define FRAM_WARN_THR_MAN_1V2				0x0000001A0 //RW - 32 bit
#define FRAM_WARN_THR_DDR0_VREF				0x0000001A4 //RW - 32 bit
#define FRAM_WARN_THR_DDR1_VREF				0x0000001A8 //RW - 32 bit
#define FRAM_WARN_THR_VM_DRVDD				0x0000001AC //RW - 32 bit
#define FRAM_WARN_THR_VIN_SCALED			0x0000001B0 //RW - 32 bit
#define FRAM_WARN_THR_VM_MAN3V3				0x0000001B4 //RW - 32 bit
#define FRAM_WARN_THR_VM_MAN1V8				0x0000001B8 //RW - 32 bit
#define FRAM_WARN_THR_MON_5V0				0x0000001BC //RW - 32 bit
#define FRAM_WARN_THR_MGT_AV				0x0000001C0 //RW - 32 bit
#define FRAM_WARN_THR_MGT_AVTT				0x0000001C4 //RW - 32 bit
#define FRAM_WARN_THR_INTERNAL_MCU_TEMP		0x0000001C8 //RW - 32 bit
#define FRAM_WARN_THR_BOARD_TEMP			0x0000001CC //RW - 32 bit
#define FRAM_WARN_THR_FPGA0_TEMP			0x0000001D0 //RW - 32 bit
#define FRAM_WARN_THR_FPGA1_TEMP			0x0000001D4 //RW - 32 bit

#define FRAM_ALARM_THR_SW_AVDD1				0x000000220 //RW - 32 bit
#define FRAM_ALARM_THR_SW_AVDD2				0x000000224 //RW - 32 bit
#define FRAM_ALARM_THR_AVDD3				0x000000228 //RW - 32 bit
#define FRAM_ALARM_THR_MAN_1V2				0x00000022C //RW - 32 bit
#define FRAM_ALARM_THR_DDR0_VREF			0x000000230 //RW - 32 bit
#define FRAM_ALARM_THR_DDR1_VREF			0x000000234 //RW - 32 bit
#define FRAM_ALARM_THR_VM_DRVDD				0x000000238 //RW - 32 bit
#define FRAM_ALARM_THR_VIN_SCALED			0x00000023C //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN3V3			0x000000240 //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN1V8			0x000000244 //RW - 32 bit
#define FRAM_ALARM_THR_MON_5V0				0x000000248 //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AV				0x00000024C //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AVTT				0x000000250 //RW - 32 bit
#define FRAM_ALARM_THR_INTERNAL_MCU_TEMP	0x000000254 //RW - 32 bit
#define FRAM_ALARM_THR_BOARD_TEMP			0x000000258 //RW - 32 bit
#define FRAM_ALARM_THR_FPGA0_TEMP			0x00000025C //RW - 32 bit
#define FRAM_ALARM_THR_FPGA1_TEMP			0x000000260 //RW - 32 bit


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 





#endif /* FRAM_H_ */