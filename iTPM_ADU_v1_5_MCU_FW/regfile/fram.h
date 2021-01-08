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

#define FRAM_MCU_VERSION					0x00000000 //R  - 32 bit
#define FRAM_MCU_COMPILE_DATE				0x00000004 //R  - 32 bit
#define FRAM_MCU_GIT_HASH					0x00000008 //R  - 32 bit
#define	FRAM_MCU_GPR_0						0x0000000C //RW - 32 bit
#define	FRAM_MCU_GPR_1						0x00000010 //RW - 32 bit
#define	FRAM_MCU_GPR_2						0x00000014 //RW - 32 bit
#define	FRAM_MCU_GPR_3						0x00000018 //RW - 32 bit
#define FRAM_MCU_POOLING_INTERVAL			0x0000001C //RW - 32 bit
#define FRAM_MCU_INTERNAL_COMMAND			0x00000020 //RW - 32 bit
#define FRAM_MCU_COUNTER					0x00000024 //RW - 32 bit
#define FRAM_MCU_COMPLETE_ADC_COUNTER		0x00000028 //RW - 32 bit
#define FRAM_MCU_BOOTLOADER_VERSION			0x0000002C //R  - 32 bit
#define	FRAM_MCU_BOOTLOADER_COMMANDS		0x00000030 //RW - 32 bit

#define FRAM_BOARD_STATUS					0x000000E0 //RW - 32 bit
#define FRAM_BOARD_ALARM					0x000000E4 //RW - 32 bit 
#define FRAM_BOARD_WARNING					0x000000E8 //RW - 32 bit

#define FRAM_ADC_SW_AVDD1					0x00000100 //RW - 16 bit
#define FRAM_ADC_SW_AVDD2					0x00000104 //RW - 16 bit
#define FRAM_ADC_AVDD3						0x00000108 //RW - 16 bit
#define FRAM_ADC_MAN_1V2					0x0000010C //RW - 16 bit
#define FRAM_ADC_DDR0_VREF					0x00000110 //RW - 16 bit
#define FRAM_ADC_DDR1_VREF					0x00000114 //RW - 16 bit
#define FRAM_ADC_VM_DRVDD					0x00000118 //RW - 16 bit
#define FRAM_ADC_VIN_SCALED					0x0000011C //RW - 16 bit
#define FRAM_ADC_VM_MAN3V3					0x00000120 //RW - 16 bit
#define FRAM_ADC_VM_MAN1V8					0x00000124 //RW - 16 bit
#define FRAM_ADC_MON_5V0					0x00000128 //RW - 16 bit
#define FRAM_ADC_MGT_AV						0x0000012C //RW - 16 bit
#define FRAM_ADC_MGT_AVTT					0x00000130 //RW - 16 bit
#define FRAM_ADC_INTERNAL_MCU_TEMP			0x00000134 //RW - 16 bit
#define FRAM_BOARD_TEMP						0x00000138 //RW - 32 bit
#define FRAM_FPGA0_TEMP						0x0000013C //RW - 16 bit
#define FRAM_FPGA1_TEMP						0x00000140 //RW - 16 bit
#define FRAM_FPGA0_FE_CURRENT				0x00000144 //RW - 16 bit
#define FRAM_FPGA1_FE_CURRENT				0x00000148 //RW - 16 bit

#define FRAM_WARN_ALARM_UPDATE				0x00000190 //RW - 32 bit

#define FRAM_WARN_THR_SW_AVDD1				0x00000194 //RW - 32 bit
#define FRAM_WARN_THR_SW_AVDD2				0x00000198 //RW - 32 bit
#define FRAM_WARN_THR_AVDD3					0x0000019C //RW - 32 bit
#define FRAM_WARN_THR_MAN_1V2				0x000001A0 //RW - 32 bit
#define FRAM_WARN_THR_DDR0_VREF				0x000001A4 //RW - 32 bit
#define FRAM_WARN_THR_DDR1_VREF				0x000001A8 //RW - 32 bit
#define FRAM_WARN_THR_VM_DRVDD				0x000001AC //RW - 32 bit
#define FRAM_WARN_THR_VIN_SCALED			0x000001B0 //RW - 32 bit
#define FRAM_WARN_THR_VM_MAN3V3				0x000001B4 //RW - 32 bit
#define FRAM_WARN_THR_VM_MAN1V8				0x000001B8 //RW - 32 bit
#define FRAM_WARN_THR_MON_5V0				0x000001BC //RW - 32 bit
#define FRAM_WARN_THR_MGT_AV				0x000001C0 //RW - 32 bit
#define FRAM_WARN_THR_MGT_AVTT				0x000001C4 //RW - 32 bit
#define FRAM_WARN_THR_INTERNAL_MCU_TEMP		0x000001C8 //RW - 32 bit
#define FRAM_WARN_THR_BOARD_TEMP			0x000001CC //RW - 32 bit
#define FRAM_WARN_THR_FPGA0_TEMP			0x000001D0 //RW - 32 bit
#define FRAM_WARN_THR_FPGA1_TEMP			0x000001D4 //RW - 32 bit
#define FRAM_WARN_THR_FPGA0_FE_CURRENT		0x000001D8 //RW - 32 bit
#define FRAM_WARN_THR_FPGA1_FE_CURRENT		0x000001E0 //RW - 32 bit

#define FRAM_ALARM_THR_SW_AVDD1				0x00000220 //RW - 32 bit
#define FRAM_ALARM_THR_SW_AVDD2				0x00000224 //RW - 32 bit
#define FRAM_ALARM_THR_AVDD3				0x00000228 //RW - 32 bit
#define FRAM_ALARM_THR_MAN_1V2				0x0000022C //RW - 32 bit
#define FRAM_ALARM_THR_DDR0_VREF			0x00000230 //RW - 32 bit
#define FRAM_ALARM_THR_DDR1_VREF			0x00000234 //RW - 32 bit
#define FRAM_ALARM_THR_VM_DRVDD				0x00000238 //RW - 32 bit
#define FRAM_ALARM_THR_VIN_SCALED			0x0000023C //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN3V3			0x00000240 //RW - 32 bit
#define FRAM_ALARM_THR_VM_MAN1V8			0x00000244 //RW - 32 bit
#define FRAM_ALARM_THR_MON_5V0				0x00000248 //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AV				0x0000024C //RW - 32 bit
#define FRAM_ALARM_THR_MGT_AVTT				0x00000250 //RW - 32 bit
#define FRAM_ALARM_THR_INTERNAL_MCU_TEMP	0x00000254 //RW - 32 bit
#define FRAM_ALARM_THR_BOARD_TEMP			0x00000258 //RW - 32 bit
#define FRAM_ALARM_THR_FPGA0_TEMP			0x0000025C //RW - 32 bit
#define FRAM_ALARM_THR_FPGA1_TEMP			0x00000260 //RW - 32 bit
#define FRAM_ALARM_THR_FPGA0_FE_CURRENT		0x00000264 //RW - 32 bit
#define FRAM_ALARM_THR_FPGA1_FE_CURRENT		0x00000268 //RW - 32 bit


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 





#endif /* FRAM_H_ */