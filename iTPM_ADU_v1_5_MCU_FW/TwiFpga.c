/*
 * TwiFpga.cpp
 *
 * Created: 18/12/2018 14:19:42
 *  Author: luca
 */ 

#include <TwiFpga.h>
#include <SpiRouter.h>

#include "regfile.h"

int twiFpgaWrite (uint8_t ICaddress, uint8_t byte2write, uint8_t byte2read, uint32_t datatx, uint32_t* datarx, twiFPGAadd address)
{
	uint32_t twi_ctrl_data = 0;
	uint32_t dataIN;
	uint32_t statusIN; //0x0 ok - 0x1 
	uint8_t busyRetry = 0;
	uint8_t tempbyte0, tempbyte1, tempbyte2, tempbyte3;
	
	ICaddress = ICaddress >> 1; // Shift 8bit to 7bit address
	
	if ((byte2write > 1) || (address != i2c3)) { // Chiedere ad ale di invertire le scritture
		tempbyte0 = (uint8_t)datatx;
		tempbyte1 = (uint8_t)datatx >> 8;
		tempbyte2 = (uint8_t)datatx >> 16;
		tempbyte3 = (uint8_t)datatx >> 24;
		datatx = 0x0;
		
		if (byte2write == 2) datatx = ((tempbyte0 << 8 ) + tempbyte1);
		if (byte2write == 3) datatx = ((tempbyte0 << 16) + (tempbyte1 << 8 ) + tempbyte2);
		if (byte2write == 4) datatx = ((tempbyte0 << 24) + (tempbyte1 << 16) + (tempbyte2 << 8) + tempbyte3);
	}
	
	twi_ctrl_data += (byte2read << 24); // [31:24] byte number to read
	twi_ctrl_data += (byte2write << 16); // [23:16] byte number to write
	twi_ctrl_data += (ICaddress); // [9:0] command - [6:0] IC address
	twi_ctrl_data += address; // [9:0] command - [9:8] FPGA router TWI address
	
	XO3_WriteByte(twi_offset + twi_wrdata, datatx);
	XO3_WriteByte(twi_offset + twi_command, twi_ctrl_data);
	for (int i = 0; i < 0xffff; i++) asm("nop");
    XO3_Read(twi_offset + twi_status, &statusIN);
	while (statusIN == (0x1 || 0x3)) {
		busyRetry++;
		if (busyRetry >= MAX_BUSY_RETRY) return (int)statusIN;
		XO3_Read(twi_offset + twi_status, &statusIN);
	}
	XO3_Read(twi_offset + tiw_rdata, &dataIN);
	
	if ((byte2write > 1) || (address != i2c3)) { // Chiedere ad ale di invertire le letture
		tempbyte0 = (uint8_t)dataIN;
		tempbyte1 = (uint8_t)dataIN >> 8;
		tempbyte2 = (uint8_t)dataIN >> 16;
		tempbyte3 = (uint8_t)dataIN >> 24;
		dataIN = 0x0;
		
		if (byte2read == 2) dataIN = ((tempbyte0 << 8 ) + tempbyte1);
		if (byte2read == 3) dataIN = ((tempbyte0 << 16) + (tempbyte1 << 8 ) + tempbyte2);
		if (byte2read == 4) dataIN = ((tempbyte0 << 24) + (tempbyte1 << 16) + (tempbyte2 << 8) + tempbyte3);
	}
	
	*datarx = dataIN;
	return (int)statusIN;
}

uint8_t twiFpgaRead8 (uint8_t ICaddress, uint32_t TwiRegister, twiFPGAadd address)
{
	uint32_t data;
		
	twiFpgaWrite(ICaddress, R8BIT, R8BIT, TwiRegister, &data, address);
	
	return data;
}

uint16_t twiFpgaRead16 (uint8_t ICaddress, uint32_t TwiRegister, twiFPGAadd address)
{
	uint32_t data;
	
	twiFpgaWrite(ICaddress, R16BIT, R16BIT, TwiRegister, &data, address);
	
	return data;
}

uint8_t twiFpgaWrite8 (uint8_t ICaddress, uint8_t dataOut, uint32_t TwiRegister, twiFPGAadd address)
{
	uint32_t data;
	
	TwiRegister = (TwiRegister << 8) + dataOut;
	
	twiFpgaWrite(ICaddress, W8BIT, W8BIT, TwiRegister, &data, address);
	
	return data;
}

uint16_t twiFpgaWrite16 (uint8_t ICaddress, uint16_t dataOut, uint32_t TwiRegister, twiFPGAadd address)
{
	uint32_t data;
	
	TwiRegister = (TwiRegister << 16) + dataOut;
	
	twiFpgaWrite(ICaddress, W16BIT, W16BIT, TwiRegister, &data, address);
	
	return data;
}