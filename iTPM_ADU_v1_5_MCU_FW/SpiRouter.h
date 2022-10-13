/*
 * Sanitas EG SPI Router
 * 
 * V1.0 - 04/01/18 ~ Luca Schettini
 * 
 * Copyright (c) 2017 Sanitas EG srl.  All right reserved.
 * 
 */

#ifndef SPIROUTER_H_
#define SPIROUTER_H_

#include <stdint.h>
//#include <cstddef>

#define SPI_FIFO_SIZE 1024

int
SPI_sync(
	uint32_t       slaveId,
	const uint8_t* txBuffer,
	uint8_t*       rxBuffer,
	uint16_t    length
);

/*
int
XO3_Write(
    void*    context,
    uint32_t offset,
    uint32_t value,
    void*    privateData
);*/

int
XO3_WriteByte(
    uint32_t regs,
    uint32_t value
);

int
XO3_Read(
    uint32_t  regs,
    uint32_t* value
);

int
XO3_Read3(
	uint32_t  regs,
	uint32_t* value
);


int
XO3_ReadXilinx(
uint32_t  regs,
uint32_t* value
);

int
XO3_Address(
    void*     context,
    uint32_t  regs,
    uint32_t* value,
    void*     privateData
);

int
XO3_BitfieldExtract(
	uint32_t RegBA,
	uint32_t RegMask,
	uint32_t shift,
	uint32_t* value
);

int
XO3_BitfieldRMWrite(
	uint32_t RegBA,
	uint32_t RegMask,
	uint32_t shift,
	uint32_t value
);

/*void
send_spi(
	uint8_t data
);*/

#endif // ifndef SPIROUTER_H_
