/*
 * xil_sysmon_fpga.h
 *
 * Created: 08/01/2021 11:02:05
 *  Author: luca
 */ 


#ifndef XIL_SYSMON_FPGA_H_
#define XIL_SYSMON_FPGA_H_

#define XIL_SYSMON_FPGA0_OFFSET		0x00000010 // Register with sys monitor base address FPGA0
#define XIL_SYSMON_FPGA1_OFFSET		0x10000010 // Register with sys monitor base address FPGA1

#define XIL_SYSMON_FPGA0_FE_CURRENT_OFF		0x448
#define XIL_SYSMON_FPGA1_FE_CURRENT_OFF		0x460

#define XIL_SYSMON_FPGA0_TEMP				0x400
#define XIL_SYSMON_FPGA1_TEMP				0x400




#endif /* XIL_SYSMON_FPGA_H_ */