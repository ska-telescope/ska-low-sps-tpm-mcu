/*
 * Sanitas EG SPI Router
 * 
 * V1.3 - 17/01/18 ~ Luca Schettini
 * 
 * ~ CHANGELOG ~ 
 * V1.0  - Initial Version.
 * V1.1b - XO3 SPI read with left shifting for mitigation XO3 SPI design error.
 * V1.2  - XO3 SPI ok, remove mitigation from MCU side.
 * V1.3  - Added free to buffer and rxbuffer to free memory. [OoF Issue] 
 * V1.4  - New SPI version
 * V2.0  - Rewrite code to support SAM4S, 32bit read
 *
 * Copyright (c) 2017 Sanitas EG srl.  All right reserved.
 * 
 */

#include <memory.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <SpiRouter.h>
#include <atmel_start.h>

// Registers
// #include "intctrl.h"
// #include "mdio.h"
// #include "onewire.h"
// #include "regfile.h"
// #include "sam.h"
// #include "uart.h"
#include "regfile.h"

/*#define MYSPI			SPI*/


#define DEBUG_SPI

#ifdef DEBUG_SPI
char bufferOut_spi[512];
#undef DEBUG_SPI // Undefine only for remove warning
#define DEBUG_SPI 2 // Possible Choice: 3: Log Level - 2: Warning Level - 1: Error Level - Only #def: Status
#endif

#ifdef DEBUG_SPI
void deb_print_spi(uint8_t debug_level = 0){
	char * str;
	str = const_cast<char *>(bufferOut_spi);
	struct io_descriptor *uartDebug;
	usart_sync_get_io_descriptor(&USART_XO3, &uartDebug);
	usart_sync_enable(&USART_XO3);
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




#if defined(DEBUG_SPI)
#define DEBUG_PRINT_SPI(...) do{sprintf(bufferOut_spi, __VA_ARGS__); deb_print_spi();} while( false )
#else
#define DEBUG_PRINT_SPI(...) do{ } while ( false )
#endif


#ifndef __cplusplus
#define nullptr ((void*)0)
#endif


int
SPI_sync(
		uint32_t       slaveId,
		const uint8_t* txBuffer,
		uint8_t*       rxBuffer,
		uint16_t    length
)
{
	uint8_t*  tmp           = nullptr;
	uint16_t* txbuf         = NULL;
 	uint8_t* rxbuf         = NULL;
	//std::vector<uint16_t> rxbuf;
	uint32_t  offset        = 0;
	bool      last          = true;
	bool      little_endian = true;
	static const int latency = 1;

	void* buffer = NULL;
	void* rxbuffer = NULL;

	uint16_t _length = length;
	uint16_t _count  = length;
	
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);
	spi_m_sync_enable(&SPI_0);
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
	
	uint16_t count_delay=0;
	
// 	struct spi_device device = {
// 		
// 		device.id = 0
// 		
// 	}		
		buffer = malloc(length + 8); // Make sure we have some (4) spare bytes at the end...
		rxbuffer = malloc(length + 8); // Make sure we have some (4) spare bytes at the end...
		tmp    = (uint8_t*)buffer;
		txbuf  = (uint16_t*)buffer;
 		rxbuf  = (uint8_t*)rxbuffer;
		slaveId >>= 8;
		slaveId  &= 0x000000FF;

/*		if (slaveId) */{ 
			if (length < 128) {
				tmp[0] = (uint8_t)(slaveId);
				tmp[1] = length & ~0x80;

//			tmp[length + 3] = 0x00;

				_length = length + 3;
				_count  = length + 3;

				offset = 2;
			} else {
				tmp[0] = (uint8_t)(slaveId);
				tmp[1] = 0x80;

				tmp[2] = (length >> 8) & 0xFF;
				tmp[3] = length & 0xFF;

//			tmp[length + 4] = 0x00;

				_length = length + 5;
				_count  = length + 5;

				offset = 4;
			}

			if (last) {
				tmp[0] &= ~0x80;
			} else {
				tmp[0] |= 0x80;
			}
		}

		if (txBuffer) {
			memcpy(&tmp[offset], txBuffer, length);
		}

		if (_count & 0x00000001) {
			_count++;
		}

		if (!little_endian) {
			uint16_t* ptr = txbuf;

			for (int i = 0; i < _count; i += 2) {
				uint16_t x = *ptr;
				x    = ((x & 0xff00) >> 8) | ((x & 0xff) << 8);
				*ptr = x;
				ptr++;
			}
		}
		
		uint8_t scoda_tx[4] = {0xff,0xff,0xff,0xff};		
		struct spi_xfer scoda;		
		struct spi_xfer transfer;
		
		if (tmp[2] != 0x05){
			transfer.rxbuf = rxbuf;
			transfer.txbuf = tmp;
			transfer.size  = _count;
			
			for (int i = 0; i < _count-1; i++) rxbuf[i] = 0xaa;
			
			gpio_set_pin_level(FPGA_CS, false); // Select Device and pulldown CS	
			int32_t risp = spi_m_sync_transfer(&SPI_0, &transfer);
		    

				
				scoda.rxbuf = rxbuf;
				scoda.txbuf = scoda_tx;
				scoda.size  = 1; 
				//wait acknowledge
				//if (rxbuf[0] != 0x0)
				//{
			 
				while (1){
					risp = spi_m_sync_transfer(&SPI_0, &scoda);
					if (rxbuf[0] == 0x0) break;
					else if (rxbuf[0] == 0x11)
					{
						gpio_set_pin_level(FPGA_CS, true); // Deselect Device and pullup CS
						DEBUG_PRINT_SPI("SPI Timeout cmd received in write op\n");
						return -1;
					}
					count_delay++;
				}
			//}
			gpio_set_pin_level(FPGA_CS, true); // Deselect Device and pullup CS
			memcpy(rxBuffer, &rxbuf[offset+latency], length);
		}
		else {
			
			for (int i = 0; i < 8; i++) rxbuf[i] = 0xaa;
			
			transfer.rxbuf = rxbuf;
			transfer.txbuf = tmp;
			transfer.size  = 7;
			
			gpio_set_pin_level(FPGA_CS, false); // Select Device and pulldown CS
			int32_t risp = spi_m_sync_transfer(&SPI_0, &transfer);
			
			scoda.rxbuf = rxbuf;
			scoda.txbuf = scoda_tx;
			scoda.size  = 1;
			
			while (1){
				risp = spi_m_sync_transfer(&SPI_0, &scoda);
				if (rxbuf[0] == 0x0) break;	
				else if (rxbuf[0] == 0x11)
				{
					gpio_set_pin_level(FPGA_CS, true); // Deselect Device and pullup CS
					DEBUG_PRINT_SPI("SPI Timeout cmd received in read op\n");
					return -1;
				}
				count_delay++;

			}
			
			scoda.size  = 4;
			
			risp = spi_m_sync_transfer(&SPI_0, &scoda);
			gpio_set_pin_level(FPGA_CS, true); // Deselect Device and pullup CS
			if (count_delay > 4) DEBUG_PRINT_SPI("Delay Count %d\n", count_delay);
			memcpy(rxBuffer, rxbuf, 4);
		}		
		
		if (!little_endian) {
			uint16_t* ptr = (uint16_t*)rxBuffer;

			for (int i = 0; i < length; i += 2) {
				uint16_t x = *ptr;
				x    = ((x & 0xff00) >> 8) | ((x & 0xff) << 8);
				*ptr = x;
				ptr++;
			}
		}
		free(buffer);
		free(rxbuffer);
		return 0;
} // SPI_sync





/*int
XO3_Write(
    void*    context,
    uint32_t offset,
    uint32_t value,
    void*    privateData
)*/
int 
XO3_WriteByte(
    uint32_t regs,
    uint32_t value
)
{
	uint8_t txBuffer[10];
	uint8_t rxBuffer[10];
	uint8_t retry=5;
	int success=0;
	memset(txBuffer, 0, 8);

	txBuffer[0] = 0x01;
	txBuffer[1] = 0xFF & (regs >> 24);
	txBuffer[2] = 0xFF & (regs >> 16);
	txBuffer[3] = 0xFF & (regs >> 8);
	txBuffer[4] = 0xFF & (regs);
	txBuffer[5] = 0xFF & (value >> 24);
	txBuffer[6] = 0xFF & (value >> 16);
	txBuffer[7] = 0xFF & (value >> 8);
	txBuffer[8] = 0xFF & (value);

	//SPI_sync(1, txBuffer, rxBuffer, 9);
	while(retry>0) 
	{
		success = SPI_sync(1, txBuffer, rxBuffer, 9);
		if (success == -1)
			retry=retry-1;
		else break;
	} 
	
	//uint32_t veryfy_data=0;
	//if (XO3_Read(regs,&veryfy_data) != 0)
	//	return -1;
	//else
	//	if (veryfy_data != value )
	//	{
	//		//DEBUG_PRINT_SPI("Error in verify regs %x, exp %x, read %x \n",regs,value,veryfy_data);
	//		//return -1;
	//	}
	return 0;	
} // XO3_Write
/*
int
XO3_Read(
    void*     context,
    uint32_t  offset,
    uint32_t* value,
    void*     privateData
)*/
int
XO3_Read3(
    uint32_t  regs,
    uint32_t* value
)
{
  uint8_t txBuffer[10];
  uint8_t rxBuffer[10];
  uint32_t dato=0;
  memset(txBuffer, 0, 8);
  int success=0;
  uint8_t retry=20;

  txBuffer[0] = 0x03;
  txBuffer[1] = 0xFF & (regs >> 24);
  txBuffer[2] = 0xFF & (regs >> 16);
  txBuffer[3] = 0xFF & (regs >> 8);
  txBuffer[4] = 0xFF & (regs);

  //int success = SPI_sync(1, txBuffer, rxBuffer, 10);
  
   while(retry>0)
   {
	success = SPI_sync(1, txBuffer, rxBuffer, 10);
	if (success == -1) retry=retry-1;
	else break;
    }
  if (retry != 20)
  	DEBUG_PRINT_SPI("Retry number %d \n",20-retry );

  //dato = (((rxBuffer[0] & 0xFF) << 24) | ((rxBuffer[1] & 0xFF) << 16) | ((rxBuffer[2] & 0xFF) << 8) | rxBuffer[3]);
  dato = (((rxBuffer[6] & 0xFF) << 24) | ((rxBuffer[7] & 0xFF) << 16) | ((rxBuffer[8] & 0xFF) << 8) | rxBuffer[9]);
  //memcpy(dato, rxBuffer[6], 4);
  //dato=dato&0x0000ffff;
  *value=dato;
  return success;
} // XO3_Read

/*
int
XO3_Read(
    void*     context,
    uint32_t  offset,
    uint32_t* value,
    void*     privateData
)*/
int
XO3_Read(
    uint32_t  regs,
    uint32_t* value
)
{
  uint8_t txBuffer[10];
  uint8_t rxBuffer[10];
  uint32_t dato=0;
  uint8_t retry=20;
  int success =0;
  memset(txBuffer, 0, 8);

  txBuffer[0] = 0x05;
  txBuffer[1] = 0xFF & (regs >> 24);
  txBuffer[2] = 0xFF & (regs >> 16);
  txBuffer[3] = 0xFF & (regs >> 8);
  txBuffer[4] = 0xFF & (regs);

  //int success = SPI_sync(1, txBuffer, rxBuffer, 10);
while(retry>0)
{
	success = SPI_sync(1, txBuffer, rxBuffer, 10);
	if (success == -1)
	{
		 retry=retry-1;
	}
	else break;
}
if (retry != 20)
	DEBUG_PRINT_SPI("Retry number %d \n",20-retry );
  dato = (((rxBuffer[0] & 0xFF) << 24) | ((rxBuffer[1] & 0xFF) << 16) | ((rxBuffer[2] & 0xFF) << 8) | rxBuffer[3]);

  //dato = (((rxBuffer[6] & 0xFF) << 24) | ((rxBuffer[7] & 0xFF) << 16) | ((rxBuffer[8] & 0xFF) << 8) | rxBuffer[9]);
  //memcpy(dato, rxBuffer[6], 4);
  //dato=dato&0x0000ffff;
  *value=dato;
  return success;
} // XO3_Read

/*
int
XO3_Read(
    void*     context,
    uint32_t  offset,
    uint32_t* value,
    void*     privateData
)*/
int
XO3_ReadXilinx(
    uint32_t  regs,
    uint32_t* value
)
{
  uint8_t txBuffer[10];
  uint8_t rxBuffer[12];
  uint32_t dato=0;
    uint8_t retry=20;
    int success =0;
  memset(txBuffer, 0, 8);

  txBuffer[0] = 0x05;
  txBuffer[1] = 0xFF & (regs >> 24);
  txBuffer[2] = 0xFF & (regs >> 16);
  txBuffer[3] = 0xFF & (regs >> 8);
  txBuffer[4] = 0xFF & (regs);




  //int success = SPI_sync(1, txBuffer, rxBuffer, 30); //11
  
  while(retry>0)
  {
	  success = SPI_sync(1, txBuffer, rxBuffer, 30); //11
	  if (success == -1) retry=retry-1;
	  else break;
  }
  if (retry != 20)
	DEBUG_PRINT_SPI("Retry number %d \n",20-retry );
  dato = (((rxBuffer[0] & 0xFF) << 24) | ((rxBuffer[1] & 0xFF) << 16) | ((rxBuffer[2] & 0xFF) << 8) | rxBuffer[3]);
  //dato = (((rxBuffer[7] & 0xFF) << 24) | ((rxBuffer[8] & 0xFF) << 16) | ((rxBuffer[9] & 0xFF) << 8) | rxBuffer[10]);
  //memcpy(dato, rxBuffer[6], 4);
  //dato=dato&0x0000ffff;
  *value=dato;
  return success;
} // XO3_Read


int
XO3_BitfieldExtract(
	uint32_t RegBA,
	uint32_t RegMask,
	uint32_t shift,
	uint32_t* value
)
{
	int success;
	uint32_t tmp = 0;	
	
	success = XO3_Read(RegBA, &tmp);	
	tmp = (tmp & RegMask) >> shift;
	
	*value = tmp;
	
	return success;
}

int
XO3_BitfieldRMWrite(
uint32_t RegBA,
uint32_t RegMask,
uint32_t shift,
uint32_t value
)
{
	int success;
	uint32_t tmp;
	
	success = XO3_Read(RegBA, &tmp);
	tmp &= ~RegMask;
	tmp += (value << shift);
		
	XO3_WriteByte(RegBA, tmp);
	
	return success;
}

/*
int
XO3_Address(
    void*     context,
    uint32_t  offset,
    uint32_t* value,
    void*     privateData
)
{
  *value = offset;
  return 0;//success;
} // XO3_Read
*/

/*void send_spi(uint8_t data){
	
	struct spi_device device = {
		
		device.id = 0
		
	};
	spi_select_device(MYSPI, &device);
	data_buffer[0] = data;
	
	//spi_write_packet(MYSPI, data_buffer, sizeof(data_buffer));
	spi_transceive_packet(MYSPI, data_buffer, data_rx_buffer, sizeof(data_buffer));
	
	spi_deselect_device(MYSPI, &device);
	delay_us(10);
	
	
}*/