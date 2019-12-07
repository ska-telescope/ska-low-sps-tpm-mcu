/*
 * ExtFlash.h
 *
 * Created: 05/12/2019 14:24:11
 *  Author: luca
 */ 


#ifndef EXTFLASH_H_
#define EXTFLASH_H_

#include <stdint.h>
#include <stdbool.h>

#define FLASH0_SLID 0x1
#define FLASH1_SLID 0x2

#define EFERR_CPLD_WRONG_VERSION 1
#define EFERR_NO_ERASE 2

int ExtFlash_SRAMErase(uint8_t fpgaid);
int ExtFlash_FPGA_Prog(uint8_t fpgaid, uint8_t flashid, bool EraseBefore);
void FlashSPI_WriteReg(uint8_t devicespi, uint8_t regs);
void FlashSPI_Sync(uint8_t slaveId, const uint8_t* txBuffer, uint8_t* rxBuffer, uint8_t length);



#endif /* EXTFLASH_H_ */