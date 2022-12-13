/*
 * TwiFpga.cpp
 *
 * Created: 18/12/2018 14:19:42
 *  Author: luca
 */ 

#include <TwiFpga.h>
#include <SpiRouter.h>

#include <memory.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <atmel_start.h>



#include "regfile.h"

extern uint32_t cpld_fw_vers;
#define DEBUG_TWI

#ifdef DEBUG_TWI
char bufferOut_twi[512];
#undef DEBUG_TWI // Undefine only for remove warning
#define DEBUG_TWI 2 // Possible Choice: 3: Log Level - 2: Warning Level - 1: Error Level - Only #def: Status
#endif

#ifdef DEBUG_TWI
void deb_print_twi(uint8_t debug_level = 0){
	char * str;
	str = const_cast<char *>(bufferOut_twi);
	struct io_descriptor *uartDebug;
	usart_sync_get_io_descriptor(&USART_0, &uartDebug);
	usart_sync_enable(&USART_0);
	switch (debug_level) {
		case 0x1:
		io_write(uartDebug, (uint8_t *)"d1:", (unsigned)3);
		break;
		case 0x2:
		io_write(uartDebug, (uint8_t *)"d2:", (unsigned)3);
		break;
		case 0x3:
		io_write(uartDebug, (uint8_t *)"d3:", (unsigned)3);
		break;
	}
	io_write(uartDebug, (uint8_t *)str, (unsigned)strlen(str));
}
#endif

#if defined(DEBUG_TWI)
#define DEBUG_PRINT_TWI(...) do{sprintf(bufferOut_twi, __VA_ARGS__); deb_print_twi();} while( false )
#else
#define DEBUG_PRINT_SPI(...) do{ } while ( false )
#endif


int twiFpgaWrite (uint8_t ICaddress, uint8_t byte2write, uint8_t byte2read, uint32_t datatx, uint32_t* datarx, twiFPGAadd address)
{
		
	uint32_t twi_ctrl_data = 0;
	uint32_t dataIN;
	uint32_t statusIN; //0x0 ok - 0x1 
	uint8_t busyRetry = 0;
	uint8_t tempbyte0, tempbyte1, tempbyte2, tempbyte3;
	uint8_t timeout = 0;
	uint32_t res;
	uint32_t ticket;
	uint32_t fwver=0;
	
	bool i2c_ack = false;
	//XO3_Read(itpm_cpld_regfile_date_code, &ticket);
	
	XO3_Read(itpm_cpld_regfile_date_code, &fwver);
	
	//DEBUG_PRINT_TWI("TWI: CPLD_FW_VER 0x%x\n",fwver);
	if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
	{
		XO3_Read(itpm_cpld_lock_queue_number, &ticket);
		do{
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, ticket); // Request I2C Ownership
			XO3_Read(itpm_cpld_lock_lock_i2c, &res);
			if (res == ticket){ // Check ownership
				DEBUG_PRINT_TWI("CPLD MCU Lock: I2C LOCKED OK from MCU\n");
				i2c_ack = true;
				break;
			}
			else timeout++;
		} while (timeout < 20 );
		if (timeout == 20)
		{
			DEBUG_PRINT_TWI("CPLD MCU Lock: I2C LOCKED Fails Timeout Occours\n");
			return -3;
		}
	}
	ICaddress = ICaddress >> 1; // Shift 8bit to 7bit address
	
	if (byte2write > 1) { // Chiedere ad ale di invertire le scritture
		tempbyte0 = (uint8_t)datatx;
		tempbyte1 = (uint8_t)datatx >> 8;
		tempbyte2 = (uint8_t)datatx >> 16;
		tempbyte3 = (uint8_t)datatx >> 24;
		datatx = 0x0;
		
		if (byte2write == 2) datatx = ((tempbyte0 << 8 ) + tempbyte1);
		if (byte2write == 3) datatx = ((tempbyte0 << 16) + (tempbyte1 << 8 ) + tempbyte2);
		if (byte2write == 4) datatx = ((tempbyte0 << 24) + (tempbyte1 << 16) + (tempbyte2 << 8) + tempbyte3);
	}
	
	twi_ctrl_data += (address << 16); // [9:0] command - [9:8] FPGA router TWI address
	twi_ctrl_data += (byte2read << 12); // [31:24] byte number to read
	twi_ctrl_data += (byte2write << 8); // [23:16] byte number to write
	twi_ctrl_data += (ICaddress); // [9:0] command - [6:0] IC address
	
	//check that passwd isn't changed
	//XO3_Read(0x40000024, &res); 
	//if(res &0x10000 == 0 )
	//{
	//		DEBUG_PRINT_TWI("Password not accepted\n");
	//		return -2;
	//}

	//check that I2C isn't BUSY
	if (XO3_Read(itpm_cpld_i2c_status, &statusIN) !=0)
	{
		if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
		return -1;
	}
	while (statusIN & 0x1 != 0) {
		busyRetry++;
		if (busyRetry >= MAX_BUSY_RETRY)
		{
			DEBUG_PRINT_TWI("I2C busy \n");
			if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
				XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
			return (int)statusIN;
		}
		
		if (XO3_Read(itpm_cpld_i2c_status, &statusIN) !=0)
		{
			if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
				XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
			return -1;
		}
	}
	if (XO3_WriteByte(itpm_cpld_i2c_transmit, datatx) != 0)
	{
		if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
		return -1;
	}
	if (XO3_WriteByte(itpm_cpld_i2c_command, twi_ctrl_data) != 0)
	{
		if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
		return -1;
	}
	
	
	for (int i = 0; i < 0xffff; i++) asm("nop");
	//delay_ms(100);
    
	
	
	if (XO3_Read(itpm_cpld_i2c_status, &statusIN)!= 0)
	{
		if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
		return -1;
	}
	
	while (statusIN == 0x1 || statusIN == 0x3) {
		busyRetry++;
		if (busyRetry >= MAX_BUSY_RETRY) 
		{
			DEBUG_PRINT_TWI("I2C busy or not ack\n");
			if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
				XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
			return (int)statusIN;
		}
		
		if (XO3_Read(itpm_cpld_i2c_status, &statusIN) !=0)
		{
			if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
				XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
			return -1;
		}
	}
	if (XO3_Read(itpm_cpld_i2c_receive, &dataIN) != 0)
	{
		if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
		return -1;
	}
	
	if (byte2read > 1) { // Chiedere ad ale di invertire le letture
		tempbyte0 = dataIN;
		tempbyte1 = dataIN >> 8;
		tempbyte2 = dataIN >> 16;
		tempbyte3 = dataIN >> 24;
		dataIN = 0x0;
		
		if (byte2read == 2) dataIN = ((tempbyte0 << 8 ) + tempbyte1);
		if (byte2read == 3) dataIN = ((tempbyte0 << 16) + (tempbyte1 << 8 ) + tempbyte2);
		if (byte2read == 4) dataIN = ((tempbyte0 << 24) + (tempbyte1 << 16) + (tempbyte2 << 8) + tempbyte3);
	}
	
	*datarx = dataIN;
	if(fwver>CPLD_FW_VERSION_LOCK_CHANGE)
		XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); // Clear I2C Ownership
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