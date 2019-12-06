/*
 * ExtFlash.c
 *
 * Created: 05/12/2019 14:23:53
 *  Author: luca
 */ 

#include <SpiRouter.h>
#include <atmel_start.h>
#include "regfile.h"

#define FLASH0_SLID 1
#define FLASH1_SLID 2

void ExtFlash_SRAMErase(uint8_t fpgaid){
	if (fpgaid == 0x1){
		XO3_WriteByte(itpm_cpld_smap_xil_0, 0);
		
	}
	if (fpgaid == 0x2){
		XO3_WriteByte(itpm_cpld_smap_xil_1, 0);
	}
}

void ExtFlash_FPGA_Prog(uint8_t fpgaid, uint8_t flashid){
	
}