/*
 * ExtFlash.c
 *
 * Created: 05/12/2019 14:23:53
 *  Author: luca
 */ 

#include <ExtFlash.h>
#include <SpiRouter.h>
#include <atmel_start.h>
#include "regfile.h"

#include <stdbool.h>
#include <memory.h>
#include <string.h>
#include <stdlib.h>

#define MAX_TRY 500 // 500 x 10ms = 5sec

#ifndef __cplusplus
#define nullptr ((void*)0)
#endif

int ExtFlash_SRAMErase(uint8_t fpgaid){
	if (fpgaid & 0x1){
		XO3_WriteByte(itpm_cpld_smap_xil_0, 0);		
	}
	if (fpgaid & 0x2){
		XO3_WriteByte(itpm_cpld_smap_xil_1, 0);
	}
	XO3_WriteByte(itpm_cpld_smap_global, 0x1);
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
	uint8_t txBuffer[4];
	uint8_t rxBuffer[16];
	memset(txBuffer, 0, 4);
	memset(txBuffer, 0, 8);
	uint32_t address = 0x00000000;
	txBuffer[1] = (address >> 16) & 0xFF;
	txBuffer[2] = (address >> 8) & 0xFF;
	txBuffer[3] = address & 0xFF;

	int status;
	// TODO Check CPLD Version

	// Erase
	if (EraseBefore) status = ExtFlash_SRAMErase(fpgaid);
	if (status != 0 ) return status;

	// Select SPI Mux
	XO3_WriteByte(itpm_cpld_regfile_spi_mux, flashid);

	txBuffer[0]=0x6;  //write enable command
	FlashSPI_Sync(0,txBuffer,rxBuffer,1);

	//Read bitstream lenght from flashid
	txBuffer[0]=0x3;  //write enable command
	FlashSPI_Sync(0,txBuffer,rxBuffer,8);
	//uint32_t lengthbit=(rxBuffer[7]<<24)|(rxBuffer[6]<<16)|(rxBuffer[5]<<8)|(rxBuffer[4]);
	uint32_t lengthbit=(rxBuffer[4]<<24)|(rxBuffer[5]<<16)|(rxBuffer[6]<<8)|(rxBuffer[7]);


	//Send FastRead command to flash with CS forced to low at end of command
	txBuffer[0]=0x0B;  //write enable command
	
	uint32_t test;
	XO3_Read(itpm_cpld_regfile_spi_cs, &test);
	
	XO3_BitfieldRMWrite(itpm_cpld_regfile_spi_cs, itpm_cpld_regfile_spi_cs_ow_M, itpm_cpld_regfile_spi_cs_ow_B, 0); //write 0 on OW flag of CS register
	//XO3_WriteByte(itpm_cpld_regfile_spi_cs_ow_M, 0);  //write 0 on OW flag of CS register
	FlashSPI_Sync(0,txBuffer,rxBuffer,8);
	
	XO3_Read(itpm_cpld_regfile_spi_cs, &test);
	
	//XO3_WriteByte(itpm_cpld_regfile_spi_tx_byte, 4);
	
	//set router to connect Flash out with FPGA in
	//XO3_WriteByte(itpm_cpld_regfile_spi_route, 1);  //write 1 on spi_route register

	//set register to provide the number of clk pulse equal to bistream lenght
	XO3_WriteByte(itpm_cpld_regfile_spi_tx_byte, lengthbit);
	//poll done bit to wait operation complete or check timeout

	while(1)
	{
		uint32_t xil_done;
		XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
		if (xil_done==0x3) break;
		XO3_Read(itpm_cpld_regfile_xilinx, &xil_done);
		delay_ms(10);
	}
	
	XO3_BitfieldRMWrite(itpm_cpld_regfile_spi_cs, itpm_cpld_regfile_spi_cs_ow_M, itpm_cpld_regfile_spi_cs_ow_B, 1); //write 1 on OW flag of CS register
	//XO3_WriteByte(itpm_cpld_regfile_spi_cs_ow_M ,1);  //write 1 on OW flag of CS register
	XO3_WriteByte(itpm_cpld_regfile_spi_route, 0);  //write 0 on spi_route register
	
}

void FlashSPI_WriteReg(uint8_t devicespi, uint8_t regs){
	uint8_t txBuffer[4];
	uint8_t	cmd[4];
	
	memset(txBuffer, 0, 4);
	
	cmd[0] = regs;	
	
	FlashSPI_Sync(devicespi, txBuffer, &cmd, 1);
}

// void FlashSPI_ReadReg(uint8_t devicespi, uint8_t regs){
// }

void FlashSPI_Sync(uint8_t slaveId, const uint8_t* txBuffer, uint8_t* rxBuffer, uint8_t length_spi){
	//uint8_t* rxbuf = NULL;
	uint8_t* tmp  = nullptr;
	static const int latency = 0;
	
	uint32_t rxTmp[16];
	
	void* buffer = NULL;
	//void* rxbuffer = NULL;
	
	uint8_t  offset = 0;
	
	buffer = malloc(length_spi + 8); // Make sure we have some (4) spare bytes at the end...
	//rxbuffer = malloc(length + 8); // Make sure we have some (4) spare bytes at the end...
	tmp = malloc(length_spi + 8); // Make sure we have some (4) spare bytes at the end...
	tmp = (uint8_t*)txBuffer;
	//rxbuf  = (uint8_t*)rxbuffer;
	
	uint32_t rxlenght = 0;	
	
	memcpy(&tmp[offset], txBuffer, length_spi);
	XO3_WriteByte(itpm_cpld_regfile_spi_cs, 0x10001);	
	
	uint32_t txt = tmp[0];
	txt += tmp[1] << 8;
	txt += tmp[2] << 16;
	txt += tmp[3] << 24;
	
	XO3_WriteByte(itpm_cpld_confspi_rxtx_buffer, txt);
	XO3_WriteByte(itpm_cpld_regfile_spi_tx_byte, length_spi);
		
	while(rxlenght > 0) {
		XO3_Read(itpm_cpld_regfile_spi_rx_byte, &rxlenght);
		asm("nop");
	}
	
	XO3_WriteByte(itpm_cpld_regfile_spi_fifo_addr, 0x0);
	
	int rxlen = length_spi/4;
  	if ((length_spi % 4) > 0) rxlen++; 
// 
// 	for (int i = 0; i < rxlen; i++){
// 		XO3_Read(itpm_cpld_confspi_rxtx_buffer, &val);
// 		rxTmp[i] = val;
// 		asm("nop");
// 	}
	
	int res = 0;
	
	for (int i = 0; i < length_spi -1; i++){
		uint32_t val = 0xaa;
		res = XO3_Read(itpm_cpld_confspi_rxtx_buffer, &val);
		rxTmp[i] = val;

		//asm("nop");
		if (i >= rxlen-1) break; // Soluzione dirty per problema (lenght_spi - 1); se sostituito con qualsiasi altra variabile o valore, il valore letto dalla fifo sarà sempre 0x0
		
	}
	
	memcpy(rxBuffer, rxTmp, length_spi);
	free(buffer);

	asm("nop");
	//free(rxbuffer);	
/*	free(tmp);	*/
	return;	
}