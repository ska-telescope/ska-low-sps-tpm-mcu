/*
 * ExtFlash.c
 *
 * Created: 05/12/2019 14:23:53
 *  Author: luca
 */ 

#include <SpiRouter.h>
#include <atmel_start.h>
#include "regfile.h"

#include <stdbool.h>
#include <memory.h>
#include <string.h>
#include <stdlib.h>

#define FLASH0_SLID 0x1
#define FLASH1_SLID 0x2
#define MAX_TRY 500 // 500 x 10ms = 5sec

#define EFERR_CPLD_WRONG_VERSION 1
#define EFERR_NO_ERASE 2

#ifndef __cplusplus
#define nullptr ((void*)0)
#endif

int ExtFlash_SRAMErase(uint8_t fpgaid){
	if (fpgaid == 0x1){
		XO3_WriteByte(itpm_cpld_smap_xil_0, 0);		
	}
	if (fpgaid == 0x2){
		XO3_WriteByte(itpm_cpld_smap_xil_1, 0);
	}
	uint32_t xil0, xil1;
	uint8_t trya = 0;
	XO3_Read(itpm_cpld_smap_xil_0, &xil0); 
	XO3_Read(itpm_cpld_smap_xil_1, &xil1); 
	while ((xil0 & 0x01 == 0) && (xil1 & 0x01 == 0)){
		delay_ms(10);
		XO3_Read(itpm_cpld_smap_xil_0, &xil0);
		XO3_Read(itpm_cpld_smap_xil_1, &xil1);
		trya++;
		if (trya == MAX_TRY) return EFERR_NO_ERASE;		
	}
	XO3_WriteByte(itpm_cpld_smap_global, 0x3);
	return 0;
}

int ExtFlash_FPGA_Prog(uint8_t fpgaid, uint8_t flashid, bool EraseBefore){
	int status;
	// TODO Check CPLD Version
	
	// Erase
	if (EraseBefore) status = ExtFlash_SRAMErase(fpgaid);
	if (status != 0 ) return status; 
	
	// Select SPI Mux
	XO3_WriteByte(itpm_cpld_regfile_spi_mux, flashid);
		
	
	
}

void FlashSPI_WriteReg(uint8_t devicespi, uint8_t regs){
	uint8_t txBuffer[4];
	uint8_t	cmd[4];
	
	memset(txBuffer, 0, 4);
	
	cmd[0] = regs;	
}

void FlashSPI_Trans(uint8_t slaveId, const uint8_t* txBuffer, uint8_t* rxBuffer, uint8_t length){
	uint8_t* rxbuf = NULL;
	uint8_t* tmp  = nullptr;
	static const int latency = 0;
	
	void* buffer = NULL;
	void* rxbuffer = NULL;
	
	uint8_t  offset = 0;
	
	buffer = malloc(length + 8); // Make sure we have some (4) spare bytes at the end...
	rxbuffer = malloc(length + 8); // Make sure we have some (4) spare bytes at the end...
	tmp    = (uint8_t*)buffer;
	rxbuf  = (uint8_t*)rxbuffer;
	
	uint8_t rxlenght = 0;	
	
	memcpy(&tmp[offset], txBuffer, length);
	
	XO3_WriteByte(itpm_cpld_confspi_rxtx_buffer, tmp);
	XO3_WriteByte(itpm_cpld_regfile_spi_tx_byte, length);
	
	while((length - rxlenght) > 0) XO3_Read(itpm_cpld_regfile_spi_rx_byte, &rxlenght);
	
			XO3_WriteByte(itpm_cpld_regfile_spi_fifo_addr, 0x0);
	
	for (int i = 0; i < rxlenght, i++){
		XO3_Read(itpm_cpld_confspi_rxtx_buffer+i, &buffer);
		buffer++;
	}
	
	memcpy(rxBuffer, &rxbuf[offset+latency], length);
	free(buffer);
	free(rxbuffer);	
	return;	
}