/*
 * iTPM ADU V1.5 MCU FW (BIOS)
 *
 * Sanitas EG
 *
 * Created: 07/10/2019 09:12:42 ~ Luca Schettini
 *
 * Copyright (c) 2017-2019 Sanitas EG srl.  All right reserved.
 *
 */ 

#include <atmel_start.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "driver_init.h"
#include "utils.h"

#include "Settings_itpm.h"
#include "build_def.h"
#include "SpiRouter.h"
#include "regfile.h"
#include "TwiFpga.h"
#include "ADT7408.h"

#define NO_XILINX_DEBUG_TEXT

// WARNING - Proper undefine DEBUG in Project properties
#ifdef DEBUG
char bufferOut[512];
#undef DEBUG // Undefine only for remove warning
#define DEBUG 2 // Possible Choice: 3: Log Level - 2: Warning Level - 1: Error Level - Only #def: Status
#endif

#if defined(DEBUG)
#define DEBUG_PRINT(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print();} while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 0
#define DEBUG_PRINT1(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x1);} while( false )
#else
#define DEBUG_PRINT1(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 1
#define DEBUG_PRINT2(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x2);} while( false )
#else
#define DEBUG_PRINT2(...) do{ } while ( false )
#endif

#if defined(DEBUG) && DEBUG > 2
#define DEBUG_PRINT3(...) do{sprintf(bufferOut, __VA_ARGS__); deb_print(0x3);} while( false )
#else
#define DEBUG_PRINT3(...) do{ } while ( false )
#endif

const uint32_t _build_version = 0xb000011A;

const uint32_t _build_date = ((((BUILD_YEAR_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH1 & 0xFF - 0x30)) << 24) | (((BUILD_YEAR_CH2 & 0xFF - 0x30) * 0x10 ) + ((BUILD_YEAR_CH3 & 0xFF - 0x30)) << 16) | (((BUILD_MONTH_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_MONTH_CH1 & 0xFF - 0x30)) << 8) | (((BUILD_DAY_CH0 & 0xFF - 0x30) * 0x10 ) + ((BUILD_DAY_CH1 & 0xFF - 0x30))));
//const uint32_t _build_time = (0x00 << 24 | (((__TIME__[0] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[1] & 0xFF - 0x30)) << 16) | (((__TIME__[3] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[4] & 0xFF - 0x30)) << 8) | (((__TIME__[6] & 0xFF - 0x30) * 0x10 ) + ((__TIME__[7] & 0xFF - 0x30))));

#define ADCCOLUMNS 14
#define TEMPS_SENSOR 3
#define FPGA_FE_CURRENT 2
#define I2C_CONNECTION_ERR_MAX 25
#define ETH_REG_NUM 6
#define WDT_CPLD_REG 0x4 //250ms * 4 = 1 s

/* EXECUTION STEP DEFINES */
enum mcu_exec_steps_t{
	post_internal_periph_init=1,
	pre_startup_stuff=2,
	startup_stuff=3,
	eth_reg_init=4,
	checkpowergood=5,
	systemmonitorstart=6,
	startuploadsettings=7,
	post_startup_stuff=8,
	pre_main_loop=9,
	i2c_manage=10,
	adcreadsingle=11,
	alarmmanage=12,
	irqinternalcpldhandler=13,
	taskslow=14,
	irqinternalpghandler=15,
	twidatablock=16,
	exchangedatablock=17,
	exchangedatablockxilinx=18,
	smaplock=19,
	smapunlock=20,
	exchangedatablockxilinx2=21,
	exchangedatablockxilinx3=22,
	exchangedatablockxilinx4=23,
	exchangedatablockxilinx5=24,
	exchangedatablockxilinx6=25,
	taskslow1=26,
	taskslow2=27,
	taskslow3=28
	};

enum i2c_control_status_t{
	waiting_first_req=0,
	request_received,
	ack_send
	};


struct ADCstruct {
	int ADCpin;
	uint16_t ADCread;
	float divider;
	uint16_t alarmTHRupper;
	uint16_t alarmTHRdowner;
	uint16_t warningTHRupper;
	uint16_t warningTHRdowner;
	bool alarmTriggered;
	bool enabled;
	uint8_t objectType;
	};
	
struct ADCstruct VoltagesTemps[ADCCOLUMNS+TEMPS_SENSOR+FPGA_FE_CURRENT]; // +3 Temperatures + 2 FE_CURRENT
int anaReadPos = 0;
bool anaNotReady = true;

/* ----- GLOBAL FLAGS -------*/
volatile bool spi_timeout = false;

volatile bool irqTimerSlow = false;
volatile bool irqTimerFast = false;
volatile bool irqExternalFPGA = false;
volatile bool check_pg_en	= false;


/* --------- VAR -------------------- */

static const float ADC_STEP = (2.5/65536); // Ext Ref Voltage (2.5V) / 16Bit ADC 2^16



uint8_t irqPG = 0x0;

uint32_t ADT7408_temp_raw=0;
uint16_t ADT7408_temp;
uint16_t ADT7408_temp_prev;

bool ADT7408Regs[3];
uint32_t pollingHz;
uint32_t reg_ThresholdEnable = 0;
uint16_t reg_ThresholdVals [2][17];
uint32_t pollingOld = 1000;
uint32_t pollingNew = 1000;

uint32_t EnableShadowRegister = 0;

bool TPMpowerLock = false;
bool TPMoverride = false;
bool TPMoverrideAutoShutdown = false;

uint32_t InternalCounter_CPLD_update = 0;
uint32_t InternalCounter_ADC_update = 0;

uint32_t xil_sysmon_fpga0_offset, xil_sysmon_fpga1_offset;
bool XilinxBlockNewInfo = false;
volatile i2c_control_status_t i2c_ctrl_status=waiting_first_req;
volatile mcu_exec_steps_t mcu_exec_step=post_internal_periph_init; 

volatile i2c_control_status_t i2c_ctrl_status_last=waiting_first_req;

uint32_t i2c_connection_error=0;
uint32_t cpld_fw_vers=0;
uint32_t mcu_heartbit=0;
uint32_t pgood_reg=0;
uint32_t ena_reg=0;
volatile uint32_t last_i2c_ack=0;
volatile uint32_t last_i2c_req=0;



/* -----------------------------------*/

#ifdef DEBUG
void deb_print(uint8_t debug_level = 0){
	char * str;
	str = const_cast<char *>(bufferOut);
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

/* -------function prototypes ----------------------------*/

void framRead(uint32_t fram_register, uint32_t* readback);
void framWrite(uint32_t fram_register, uint32_t writedata);
int copyeep_ram();
void smap_lock(bool *xil_ack, uint32_t *retry);
void smap_unlock();
void tpm_wd_init(uint8_t time);
void tpm_wd_enable(bool enable);
void tpm_wd_update();
int set_i2c_pwd(void);
int init_eth_regs_from_eep();
void SKAPower(bool ADCpwr, bool FRONTENDpwr, bool FPGApwr, bool SYSRpwr, bool VGApwr);
void SKAalarmUpdate(void);
void SKAalarmManage();
int32_t SAMinternalTempConv(uint32_t raw);
void exchangeDataBlockXilinx();
void exchangeDataBlock();
void ADCreadSingle();
void ADCstart();
void TWIdataBlock(void);
void check_bus_access();

/*******************************************************************/


void framRead(uint32_t fram_register, uint32_t* readback){
	XO3_Read(itpm_cpld_bram_cpu + fram_register, readback);
}

void framWrite(uint32_t fram_register, uint32_t writedata){
	XO3_WriteByte(itpm_cpld_bram_cpu + fram_register, writedata);
}


void check_bus_access()
{
	uint32_t rreg=0; 
	uint32_t errors_det=0;
	DEBUG_PRINT("XO3 ACCESS Check\n");
	XO3_WriteByte(itpm_cpld_regfile_xo3_link, 0);
	for(uint32_t i=0;i<1000;i++)
	{
		//gpio_set_pin_level(XO3_LINK1, true);
		//gpio_set_pin_level(XO3_LINK1, false);
		rreg=0;
		framWrite(DBG_ACCESS_CHECK,i);
		framRead(DBG_ACCESS_CHECK,&rreg);

		if(rreg!=i)
		{
			//gpio_set_pin_level(XO3_LINK1, true);
			//gpio_set_pin_level(XO3_LINK1, false);
			//set_i2c_pwd();
			//XO3_WriteByte(itpm_cpld_i2c_ip,0x0);
			framRead(DBG_ACCESS_CHECK,&rreg);
			if(rreg!=i)
			{
				DEBUG_PRINT("Error Detected, exp %x, read %x \n", i, rreg);
				errors_det++;
			}
			else{
				DEBUG_PRINT("Error Detected but recovered\n", i, rreg);
				errors_det++;
			}
		}	
	}
	if(errors_det == 0)
		DEBUG_PRINT("Check PASSED no errors\n");
	else
		DEBUG_PRINT("Check FAILED, detected %d errors\n", errors_det );
	
}

/*
int copyeep_ram()
{
	int status;
	uint32_t eep_add=0;
	uint32_t fram_add=0;
	uint32_t fram_data=0;
	uint32_t res=0;
	int i,k;

	XO3_Read(0x40000020, &res);
	XO3_WriteByte(0x4000003c,res);
	XO3_Read(0x40000024, &res);
	XO3_WriteByte(0x40000038,res);
	XO3_Read(0x40000024, &res);
	
	if(res &0x10000 == 0 )
		DEBUG_PRINT("Password not accepted\n");
	else
		DEBUG_PRINT("Password accepted\n");

	fram_add=FRAM_EEP_START;
	for (eep_add=0;eep_add<FRAM_EEP_SIZE;eep_add=eep_add+4)
	{
		status = twiFpgaWrite(EEP_PHY_ADD, 1, 4,eep_add , &fram_data, i2c1);
		if(status == 0)
		{
			i2c_connection_error=0;
			
			framWrite(fram_add,fram_data);
			fram_data=0;
			fram_add=fram_add+0x4;
		}
		else if (status == 2)
		{
			DEBUG_PRINT("I2C read failed, ACK_Error detected\n");
			i2c_connection_error++;
			return -1;
		}
		else if (status == -3)
		{
			DEBUG_PRINT("I2C read failed, MCU Lock Failed\n");
			i2c_connection_error++;
			return -1;
		}
		else
		{
			DEBUG_PRINT("I2C read failed\n");
			i2c_connection_error++;
			return -1;
		}
	}
	fram_add=FRAM_EEP_START+0xF0;
	for (eep_add=0xf0;eep_add<0x100;eep_add=eep_add+4)
	{
		status = twiFpgaWrite(EEP_PHY_ADD, 1, 4,eep_add , &fram_data, i2c1);
		if(status == 0)
		{
			i2c_connection_error=0;
			
			framWrite(fram_add,fram_data);
			fram_data=0;
			fram_add=fram_add+0x4;
		}
		else if (status == 2)
		{
			DEBUG_PRINT("I2C read failed, ACK_Error detected\n");
			i2c_connection_error++;
			return -1;
		}
		else if (status == -3)
		{
			DEBUG_PRINT("I2C read failed, MCU Lock Failed\n");
			i2c_connection_error++;
			return -1;
		}
		else
		{
			DEBUG_PRINT("I2C read failed\n");
			i2c_connection_error++;
			return -1;
		}
	}
	fram_add=EEPMAPPEDADD;
	fram_data=itpm_cpld_bram_cpu+FRAM_EEP_START;
	framWrite(fram_add,fram_data);
	return 0;
	
}*/

void smap_lock(bool *xil_ack, uint32_t *retry)
{
	*xil_ack=false;
	uint32_t received_d=0;
	uint32_t ticket=0;
	uint32_t retry_num=0;
	mcu_exec_step=smaplock;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	if(cpld_fw_vers > CPLD_FW_VERSION_LOCK_CHANGE )
	{	
		tpm_wd_update();
		XO3_Read(itpm_cpld_lock_queue_number,&ticket);
		DEBUG_PRINT3("MCU SMAP LOCK Ticket 0x%x\n",ticket);
		do{
			XO3_WriteByte(itpm_cpld_lock_lock_smap, ticket); // Request Xilinx Bus Ownership
			XO3_Read(itpm_cpld_lock_lock_smap, &received_d);
			if (received_d == ticket){ // Check ownership
				*xil_ack = true;
				DEBUG_PRINT3("CPLD SMAP LOCK MCU - Xilinx OK - Retry %i\n", timeout);
				break;
			}
			else
			{
				retry_num++; 	
			}
		} while(retry_num<20);
	}
	else
	{
			XO3_WriteByte(itpm_cpld_lock_mlock0, itpm_cpld_smap_global); // Request Xilinx Bus Ownership
			do{
				XO3_Read(itpm_cpld_lock_mlock0, &received_d);
				if (received_d == itpm_cpld_smap_global){ // Check ownership
					*xil_ack = true;
					DEBUG_PRINT3("CPLD SMAP LOCK MCU - Xilinx OK - Retry %i\n", timeout);
					retry_num = 10;
				}
				else { retry_num++; }
			} while (retry_num < 10);
	}
	*retry=retry_num;
}

void smap_unlock()
{
		mcu_exec_step=smapunlock;
		framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		if(cpld_fw_vers > CPLD_FW_VERSION_LOCK_CHANGE )
			XO3_WriteByte(itpm_cpld_lock_lock_smap, 0x0); // Clear Xilinx Bus Ownership
		else
			XO3_WriteByte(itpm_cpld_lock_mlock0, 0xffffffff); // Clear Xilinx Bus Ownership
}


//initiale wd timer vale in multiple of 150 ms
void tpm_wd_init(uint8_t time)
{
	uint32_t readdata; 
	XO3_Read(itpm_cpld_regfile_wdt_mcu, &readdata);
	readdata = (readdata & 0xffff) | (time<<16);
	XO3_WriteByte(itpm_cpld_regfile_wdt_mcu,readdata);
}

void tpm_wd_enable(bool enable)
{
	uint32_t readdata;
	XO3_Read(itpm_cpld_regfile_wdt_mcu, &readdata);
	if (enable)
		readdata = readdata | 0x1;
	else
		readdata = readdata & 0xfffffffe;
	XO3_WriteByte(itpm_cpld_regfile_wdt_mcu,readdata);

}

void tpm_wd_update()
{
	uint32_t readdata;
	//if(mcu_heartbit == 0)
	//XO3_Read(itpm_cpld_regfile_wdt_mcu, &readdata);
	mcu_heartbit=~mcu_heartbit;
	XO3_WriteByte(itpm_cpld_regfile_mcu_heartbeat,mcu_heartbit);	  
}


/**
 * \brief set i2c pwd 
 * \details set i2c password
 * \return return operation status 0 ok  else failed
 */
int set_i2c_pwd(void)
{
		uint32_t readval=0;
		XO3_Read(itpm_cpld_i2c_mac_hi, &readval);
		XO3_WriteByte(itpm_cpld_i2c_password,readval);
		XO3_Read(itpm_cpld_i2c_mac_lo, &readval);
		XO3_WriteByte(itpm_cpld_i2c_password_lo,readval);
		XO3_Read(itpm_cpld_i2c_password, &readval);
		
		if((readval & 0x10000) == 0 )
		{
			DEBUG_PRINT("Password not accepted: %x\n",readval);
			return -1;	
		}
		else
		{		
			return 0;
		}
		//DEBUG_PRINT("Password accepted\n");
}


/**
 * \brief initialization of CPLD ETH registers
 * \details Initialize CPLD ETH registers (ip, netmask, gateway, mac) reading from MAC EEP
 * \return return operation status 0 ok  else failed
 */
int init_eth_regs_from_eep()
{
		int status;
		uint32_t eep_add=0;
		uint32_t cpld_add=0;
		uint32_t eep_data=0;
		uint32_t res=0;
		eep_add = 0;
		int i=0;
		int pw_acc=0;
		uint32_t retry_op=0;
		uint32_t cpld_adds[ETH_REG_NUM] = {itpm_cpld_i2c_ip, itpm_cpld_i2c_netmask, itpm_cpld_i2c_gateway,itpm_cpld_i2c_key, itpm_cpld_i2c_mac_hi,itpm_cpld_i2c_mac_lo};
		uint32_t mac_s[2]={0,0};
		mcu_exec_step=eth_reg_init;
		framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		pw_acc=set_i2c_pwd();		
		if (pw_acc != 0)
			DEBUG_PRINT("Password not accepted\n");
		else
			DEBUG_PRINT("Password accepted\n");
		
		
		eep_add=0;
		for(i=0;i<ETH_REG_NUM - 2; i++)
		{
			status = twiFpgaWrite(EEP_PHY_ADD, 1, 4,eep_add , &eep_data, i2c1);
			if(status == 0)
			{
				//DEBUG_PRINT("EEP Read data %x \n",eep_data);
				i2c_connection_error=0;
				if(cpld_adds[i]==itpm_cpld_i2c_ip)
				{  
					if (eep_data!=0xffffffff)
						XO3_WriteByte((uint32_t)cpld_adds[i],eep_data);
				}
				else if(cpld_adds[i]=itpm_cpld_i2c_key)
				{
					XO3_WriteByte((uint32_t)cpld_adds[i],eep_data>>8);
				}
				else
					XO3_WriteByte((uint32_t)cpld_adds[i],eep_data);
				DEBUG_PRINT("Write at 0x%x EEP Read data %x  at EEPadd 0x%x \n",(uint32_t)cpld_adds[i],eep_data, eep_add);
				eep_add=eep_add+4;
			}
			else
			{	
				DEBUG_PRINT("I2C Operation Error, status = %x \n",status);
				i2c_connection_error++;
				return -1;								
			} 	
		}
		eep_add=0xF8;
		int m_ind=0;
		for(i=(ETH_REG_NUM - 2);i<ETH_REG_NUM; i++)
		{
			status = twiFpgaWrite(EEP_PHY_ADD, 1, 4,eep_add , &eep_data, i2c1);
			if(status == 0)
			{
				i2c_connection_error=0;
				//XO3_WriteByte((uint32_t)cpld_adds[i],eep_data);
				mac_s[m_ind]=eep_data;
				DEBUG_PRINT("EEP Read data %x at eepadd 0x%x \n",eep_data,eep_add);
				eep_add=eep_add+4;
				m_ind++;
			}
			else
			{
				i2c_connection_error++;
				return -1;
			}
		}
	
		for (i=0;i<2;i++)
		{
			XO3_WriteByte((uint32_t)cpld_adds[i+4],mac_s[i]);
		}

		//debug
		for(i=0;i<ETH_REG_NUM;i++)
		{
			XO3_Read(cpld_adds[i],&eep_data);
			DEBUG_PRINT("CPLD ETH REGS: 0x%x = 0x%x\n",cpld_adds[i],eep_data);
		}
		return 0;
}




static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync() {
	while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void SKAPower(bool ADCpwr, bool FRONTENDpwr, bool FPGApwr, bool SYSRpwr, bool VGApwr) {
	uint8_t tmp = 0;
	if (ADCpwr)			tmp += EN_ADC;
	if (FRONTENDpwr)	tmp += EN_FE;
	if (FPGApwr)		tmp += EN_FPGA;
	if (SYSRpwr)		tmp += EN_SYSR;
	if (VGApwr)			tmp += EN_VGA;
	
	XO3_WriteByte(itpm_cpld_regfile_enable_shadow, tmp);
	XO3_WriteByte(itpm_cpld_regfile_enable, tmp);
	
	DEBUG_PRINT("Powered devices ADC %d - Frontend %d - FPGA %d - SYSR %d - VGA %d\n", ADCpwr, FRONTENDpwr, FPGApwr, SYSRpwr, VGApwr);
}

void GetTPMENABLEState(uint32_t *tpm_enablereg_state)
{
	uint32_t readreg=0;
	XO3_Read(itpm_cpld_regfile_enable, &readreg);
	*tpm_enablereg_state=readreg;
	ena_reg=readreg;
}

void CheckPowerGoodandEnable()
{
	uint32_t enable_reg=0;
	uint32_t pgoodreg=0;
	uint32_t pgood_err_reg=0;
	uint32_t error=0;
	mcu_exec_step=checkpowergood;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	
	GetTPMENABLEState(&enable_reg);
	framRead(FRAM_POWERGOOD,&pgoodreg);
	framRead(FRAM_POWERGOOD_ERR,&pgood_err_reg);
	DEBUG_PRINT("PGOOD CHECK\n");
	if(enable_reg&EN_ADC == EN_ADC)
		if(pgoodreg&PG_ADC_irq != PG_ADC_irq)
		{
			error++;
			pgood_err_reg=pgood_err_reg|PG_ADC_irq;
			DEBUG_PRINT("ERROR: ADC is enabled but pgood of ADC PS is Low\n");
		}
	if(enable_reg&EN_FE == EN_FE)
		if(pgoodreg&PG_FE_irq != PG_FE_irq)
		{
			error++;
			pgood_err_reg=pgood_err_reg|PG_FE_irq;
			DEBUG_PRINT("ERROR: FE is enabled but pgood of FE PS is Low\n");
		}	
	if(enable_reg&EN_FPGA == EN_FPGA)
		if(pgoodreg&PG_FPGA_irq != PG_FPGA_irq)
		{
			error++;
			pgood_err_reg=pgood_err_reg|PG_FPGA_irq;
			DEBUG_PRINT("ERROR: FPGA is enabled but pgood of FPGA PS is Low\n");
		}
	if(enable_reg&EN_FPGA == EN_FPGA)
		if(pgoodreg&PG_FPGA_irq != PG_FPGA_irq)
		{
			error++;
			pgood_err_reg=pgood_err_reg|PG_FPGA_irq;
			DEBUG_PRINT("ERROR: FPGA is enabled but pgood of FPGA PS is Low\n");
		}
	if(pgoodreg&(PG_MAN_irq | PG_AVDD_irq) != (PG_MAN_irq | PG_AVDD_irq))
	{
		error++;
		pgood_err_reg=pgood_err_reg|(PG_MAN_irq | PG_AVDD_irq);
		DEBUG_PRINT("ERROR: Board is powered on but pgood of MAN and AVDD PS are Low\n");
	}
	check_pg_en=false;
	// TO DO: add powering off of PS and setting global_alarm_status_register
		
}



/**
 * \brief update thresholds alarms register in CPLD
 * \details update thresholds alarms and warning register in FRAM in CPLD
 */
void SKAalarmUpdate(void){
		uint32_t risp;
		
		// WARNING
		
		framRead(FRAM_WARN_THR_SW_AVDD1, &risp);
		VoltagesTemps[int(SWAVDD1)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD1)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_SW_AVDD2, &risp);
		VoltagesTemps[int(SWAVDD2)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD2)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_AVDD3, &risp);
		VoltagesTemps[int(SWAVDD3)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD3)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MAN_1V2, &risp);
		VoltagesTemps[int(MAN1V2)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V2)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_DDR0_VREF, &risp);
		VoltagesTemps[int(DDR0VREF)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR0VREF)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_DDR1_VREF, &risp);
		VoltagesTemps[int(DDR1VREF)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR1VREF)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_DRVDD, &risp);
		VoltagesTemps[int(VMDRVDD)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(VMDRVDD)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VIN_SCALED, &risp);
		VoltagesTemps[int(VINSCALED)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(VINSCALED)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_MAN3V3, &risp);
		VoltagesTemps[int(MAN3V3)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN3V3)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_VM_MAN1V8, &risp);
		VoltagesTemps[int(MAN1V8)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V8)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MON_5V0, &risp);
		VoltagesTemps[int(MON5V0)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MON5V0)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MGT_AV, &risp);
		VoltagesTemps[int(MGTAV)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MGTAV)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_MGT_AVTT, &risp);
		VoltagesTemps[int(MGAVTT)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(MGAVTT)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_INTERNAL_MCU_TEMP, &risp);
		VoltagesTemps[int(INTTEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(INTTEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_BOARD_TEMP, &risp);
		VoltagesTemps[int(BOARDTEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(BOARDTEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_FPGA0_TEMP, &risp);
		VoltagesTemps[int(FPGA0TEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA0TEMP)].warningTHRupper = uint16_t(risp>>16);
		
		framRead(FRAM_WARN_THR_FPGA1_TEMP, &risp);
		VoltagesTemps[int(FPGA1TEMP)].warningTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA1TEMP)].warningTHRupper = uint16_t(risp>>16);
		
		// ALARM
		
		framRead(FRAM_ALARM_THR_SW_AVDD1, &risp);
		VoltagesTemps[int(SWAVDD1)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD1)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_SW_AVDD2, &risp);
		VoltagesTemps[int(SWAVDD2)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD2)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_AVDD3, &risp);
		VoltagesTemps[int(SWAVDD3)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(SWAVDD3)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MAN_1V2, &risp);
		VoltagesTemps[int(MAN1V2)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V2)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_DDR0_VREF, &risp);
		VoltagesTemps[int(DDR0VREF)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR0VREF)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_DDR1_VREF, &risp);
		VoltagesTemps[int(DDR1VREF)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(DDR1VREF)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_DRVDD, &risp);
		VoltagesTemps[int(VMDRVDD)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(VMDRVDD)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VIN_SCALED, &risp);
		VoltagesTemps[int(VINSCALED)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(VINSCALED)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_MAN3V3, &risp);
		VoltagesTemps[int(MAN3V3)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN3V3)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_VM_MAN1V8, &risp);
		VoltagesTemps[int(MAN1V8)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MAN1V8)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MON_5V0, &risp);
		VoltagesTemps[int(MON5V0)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MON5V0)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MGT_AV, &risp);
		VoltagesTemps[int(MGTAV)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MGTAV)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_MGT_AVTT, &risp);
		VoltagesTemps[int(MGAVTT)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(MGAVTT)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_INTERNAL_MCU_TEMP, &risp);
		VoltagesTemps[int(INTTEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(INTTEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_BOARD_TEMP, &risp);
		VoltagesTemps[int(BOARDTEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(BOARDTEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_FPGA0_TEMP, &risp);
		VoltagesTemps[int(FPGA0TEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA0TEMP)].alarmTHRupper = uint16_t(risp>>16);
				
		framRead(FRAM_ALARM_THR_FPGA1_TEMP, &risp);
		VoltagesTemps[int(FPGA1TEMP)].alarmTHRdowner = uint16_t(risp); VoltagesTemps[int(FPGA1TEMP)].alarmTHRupper = uint16_t(risp>>16);

		
		
		framWrite(FRAM_WARN_ALARM_UPDATE, 0x0); //Reset Registers

}

void SKAalarmManage(){
	mcu_exec_step=alarmmanage;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	
	/// -------------- ADC -----------------
	if ((VoltagesTemps[anaReadPos].ADCread!=0xffff) && (VoltagesTemps[anaReadPos].ADCread > VoltagesTemps[anaReadPos].alarmTHRupper) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_ALARM
		#ifndef DISABLE_AUTO_SHUTDOWN		
		if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
		#endif
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,itpm_cpld_regfile_global_status_voltage_B,0x2); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("-----\nADC ALARM %d too high, val %d expected max %d\n-----\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].alarmTHRupper);
		framWrite(FRAM_WRN_ERR_VALUE,VoltagesTemps[anaReadPos].ADCread);
		TPMpowerLock = true;
		//delay_ms(500); // ONLY FOR TEST
	}
	else if ((VoltagesTemps[anaReadPos].ADCread!=0xffff) && (VoltagesTemps[anaReadPos].ADCread < VoltagesTemps[anaReadPos].alarmTHRdowner) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_ALARM
		#ifndef DISABLE_AUTO_SHUTDOWN
		if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
		#endif
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,itpm_cpld_regfile_global_status_voltage_B,0x2); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("-----\nADC ALARM %d too low, val %d expected min %d\n-----\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].alarmTHRdowner);
		framWrite(FRAM_ALM_ERR_VALUE,VoltagesTemps[anaReadPos].ADCread);
		TPMpowerLock = true;
		//delay_ms(500); // ONLY FOR TEST
	}
	else if ((VoltagesTemps[anaReadPos].ADCread!=0xffff) && (VoltagesTemps[anaReadPos].ADCread > VoltagesTemps[anaReadPos].warningTHRupper) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_WARNING
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,itpm_cpld_regfile_global_status_voltage_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC WARNING %d too high, val %d expected max %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].warningTHRupper);
		framWrite(FRAM_WRN_ERR_VALUE,VoltagesTemps[anaReadPos].ADCread);
		//delay_ms(500); // ONLY FOR TEST
	}	
	else if ((VoltagesTemps[anaReadPos].ADCread!=0xffff) && (VoltagesTemps[anaReadPos].ADCread < VoltagesTemps[anaReadPos].warningTHRdowner) && ((VoltagesTemps[anaReadPos]).enabled)){
		XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,anaReadPos),anaReadPos,1); // Write bit on FRAM_BOARD_WARNING
		XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,itpm_cpld_regfile_global_status_voltage_B,0x1); // Write bit on itpm_cpld_regfile_global_status
		DEBUG_PRINT1("ADC WARNING %d too low, val %d expected min %d\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread, VoltagesTemps[anaReadPos].warningTHRdowner);
		framWrite(FRAM_ALM_ERR_VALUE,VoltagesTemps[anaReadPos].ADCread);
		//delay_ms(500); // ONLY FOR TEST
	}

	/// -------------- ADC -----------------
	
	/// -------------- Other Voltage/Current/Temps -----------------------
	if (XilinxBlockNewInfo){
		for (int i = BOARDTEMP; i < FPGA1FEVA+1; i++)
		{
			if (i == BOARDTEMP)
			{
				if (i2c_connection_error > I2C_CONNECTION_ERR_MAX)
				{
					set_i2c_pwd();
					int status = twiFpgaWrite(0x30, 1, 2, 0x05, &ADT7408_temp_raw, i2c1); //temp_value 0x30
					if(status == 0)
					{
						i2c_connection_error=0;
					}
					else
					{
						DEBUG_PRINT("I2C Unreachable\n");
						XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,i),i,1); // Write bit on FRAM_BOARD_ALARM
						#ifndef DISABLE_AUTO_SHUTDOWN
						if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
						#endif
						XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,0x30000,16,0x2); // Write bit on itpm_cpld_regfile_global_status
						//DEBUG_PRINT1("-----\nERROR I2C Unreachable for %d times, powered off same supplies\n-----\n", I2C_CONNECTION_ERR_MAX");
						framWrite(FRAM_I2C_UNREACH_ERR,i2c_connection_error);
						TPMpowerLock = true;
					}
				}
				else if ((VoltagesTemps[i].ADCread&0x8000 != 0x8000) && (VoltagesTemps[i].ADCread&0xfff > VoltagesTemps[i].alarmTHRupper) && ((VoltagesTemps[i]).enabled))
				{
					XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,i),i,1); // Write bit on FRAM_BOARD_ALARM
					#ifndef DISABLE_AUTO_SHUTDOWN
					if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
					#endif
					XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,uint32_t(VoltagesTemps[i].objectType),0x2); // Write bit on itpm_cpld_regfile_global_status
					DEBUG_PRINT1("-----\nADC ALARM_ %d too high, val %x expected max %x\n-----\n", i, VoltagesTemps[i].ADCread, VoltagesTemps[i].alarmTHRupper);
					framWrite(FRAM_ALM_ERR_VALUE,VoltagesTemps[i].ADCread);
					TPMpowerLock = true;
					//delay_ms(500); // ONLY FOR TEST
				}
			}
			else
			{ 
				if ((VoltagesTemps[i].ADCread!=0xffff) && (VoltagesTemps[i].ADCread > VoltagesTemps[i].alarmTHRupper) && ((VoltagesTemps[i]).enabled)){
						XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,i),i,1); // Write bit on FRAM_BOARD_ALARM
						#ifndef DISABLE_AUTO_SHUTDOWN
						if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
						#endif
						XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,uint32_t(VoltagesTemps[i].objectType),0x2); // Write bit on itpm_cpld_regfile_global_status
						DEBUG_PRINT1("-----\nADC ALARM_ %d too high, val %x expected max %x\n-----\n", i, VoltagesTemps[i].ADCread, VoltagesTemps[i].alarmTHRupper);
						framWrite(FRAM_ALM_ERR_VALUE,VoltagesTemps[i].ADCread);
						TPMpowerLock = true;
						//delay_ms(500); // ONLY FOR TEST
					}
				else if ((VoltagesTemps[i].ADCread!=0xffff) && (VoltagesTemps[i].ADCread < VoltagesTemps[i].alarmTHRdowner) && ((VoltagesTemps[i]).enabled)){
					XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_ALARM),pow(2,i),i,1); // Write bit on FRAM_BOARD_ALARM
					#ifndef DISABLE_AUTO_SHUTDOWN
					if (!TPMoverrideAutoShutdown) SKAPower(0,0,0,0,0);
					#endif
					XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_temperature_M,uint32_t(VoltagesTemps[i].objectType),0x2); // Write bit on itpm_cpld_regfile_global_status
					DEBUG_PRINT1("-----\nADC ALARM %d too low, val %d expected min %d\n-----\n", i, VoltagesTemps[i].ADCread, VoltagesTemps[i].alarmTHRdowner);
					framWrite(FRAM_ALM_ERR_VALUE,VoltagesTemps[i].ADCread);
					TPMpowerLock = true;
					//delay_ms(500); // ONLY FOR TEST
				}
				else if ((VoltagesTemps[i].ADCread!=0xffff) && (VoltagesTemps[i].ADCread > VoltagesTemps[i].warningTHRupper) && ((VoltagesTemps[i]).enabled)){
					XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,i),i,1); // Write bit on FRAM_BOARD_WARNING
					XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,uint32_t(VoltagesTemps[i].objectType),0x1); // Write bit on itpm_cpld_regfile_global_status
					DEBUG_PRINT1("ADC WARNING %d too high, val %d expected max %d\n", i, VoltagesTemps[i].ADCread, VoltagesTemps[i].warningTHRupper);
					framWrite(FRAM_WRN_ERR_VALUE,VoltagesTemps[i].ADCread);
					//delay_ms(500); // ONLY FOR TEST
				}
				else if ((VoltagesTemps[i].ADCread!=0xffff) && (VoltagesTemps[i].ADCread < VoltagesTemps[i].warningTHRdowner) && ((VoltagesTemps[i]).enabled)){
					XO3_BitfieldRMWrite((itpm_cpld_bram_cpu+FRAM_BOARD_WARNING),pow(2,i),i,1); // Write bit on FRAM_BOARD_WARNING
					XO3_BitfieldRMWrite(itpm_cpld_regfile_global_status,itpm_cpld_regfile_global_status_voltage_M,uint32_t(VoltagesTemps[i].objectType),0x1); // Write bit on itpm_cpld_regfile_global_status
					DEBUG_PRINT1("ADC WARNING %d too low, val %d expected min %d\n", i, VoltagesTemps[i].ADCread, VoltagesTemps[i].warningTHRdowner);
					framWrite(FRAM_WRN_ERR_VALUE,VoltagesTemps[i].ADCread);
					//delay_ms(500); // ONLY FOR TEST
				}
		}
		XilinxBlockNewInfo = false;
	}
		}
	
	/// -------------- Other Voltage/Current/Temps ---------------------------	
	
}

int32_t SAMinternalTempConv(uint32_t raw) {
	int32_t adc = raw;
	//use device factory calibration values for temperature conversion (simplified)
	uint32_t* tmpLogBase = (uint32_t*)0x00806030;
	uint32_t tmpLog0 = tmpLogBase[0];
	uint32_t tmpLog1 = tmpLogBase[1];
	uint8_t roomInt = tmpLog0 & 0xff;
	uint8_t roomFrac = (tmpLog0 >> 8) & 0x0f;
	uint8_t hotInt = (tmpLog0 >> 12) & 0xff;
	uint8_t hotFrac = (tmpLog0 >> 20) & 0x0f;
	int32_t roomADC = ((tmpLog1 >> 8) & 0xfff) << 4;
	int32_t hotADC = ((tmpLog1 >> 20) & 0xfff) << 4;
	int32_t roomMdeg = 1000 * roomInt + 100 * roomFrac;
	int32_t hotMdeg = 1000 * hotInt + 100 * hotFrac;
	int32_t mdeg = roomMdeg + ((hotMdeg-roomMdeg) * (adc-roomADC)) / (hotADC-roomADC);
	return mdeg;
}

void exchangeDataBlockXilinx(){
	//#define XILINX_DEBUG_TEXT
	mcu_exec_step=exchangedatablockxilinx;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	if (TPMoverrideAutoShutdown) return;
	
	uint32_t timeout = 0;
	bool xil_ack = false;
	static bool offset_read0 = false;
	static bool offset_read1 = false;
	static uint32_t timer = 0;
	uint32_t res;
	uint32_t ticket;
	static bool xil0_sm_disabled = false;
	static bool xil1_sm_disabled = false;
	smap_lock(&xil_ack,&timeout);
	if ((timeout >= 20) && !xil_ack){
		DEBUG_PRINT2("CPLD LOCK MCU - Xilinx Busy (Maybe someone is programming the FPGAs?)\n");  // Timeout
		offset_read0 = false;
		offset_read1 = false;
		timer = 0;
		xil0_sm_disabled = false;
		xil1_sm_disabled = false;
		VoltagesTemps[FPGA0TEMP].enabled = false;
		VoltagesTemps[FPGA0FEVA].enabled = false;
		VoltagesTemps[FPGA1TEMP].enabled = false;
		VoltagesTemps[FPGA1FEVA].enabled = false;
	}
	if (xil0_sm_disabled && xil1_sm_disabled) xil_ack = false;
	if (xil_ack){ // If bus is mine...
		uint32_t xil;
		uint32_t xil_done = 0xfffffff;
		uint32_t xil_init = 0xfffffff;
		mcu_exec_step=exchangedatablockxilinx2;
		framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		XO3_Read(itpm_cpld_regfile_xilinx, &xil);
		XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
		XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_init_M, itpm_cpld_regfile_xilinx_init_B,&xil_init);	
		XO3_Read(itpm_cpld_regfile_enable_shadow, &res);
		//DEBUG_PRINT2("Xilinx Done - %x - Xilinx Init %x - Enable Xilinx - %x\n", xil_done, xil_init, res);
		uint32_t res2;
		if (res & EN_FPGA) { // EnableShadow Reg
			if (((xil_done == 1) && (xil_init == 1)) || ((xil_done == 3) && (xil_init == 3))){
				if (timer > 4){
					mcu_exec_step=exchangedatablockxilinx3;
					framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
					// Xilinx System Monitor load base address from CPLD
					XO3_ReadXilinx(XIL_SYSMON_FPGA0_OFFSET, &xil_sysmon_fpga0_offset);
					// Check version
					XO3_ReadXilinx(itpm_cpld_wb_c2c0, &res2);
					if (res2 < 0x1021752) {
						offset_read0 = false;
						DEBUG_PRINT1("Xil0 FW Ver 0x%x too old. System Monitor 0 disabled\n", res2);
						xil0_sm_disabled = true;
						//XO3_WriteByte(itpm_cpld_regfile_enable, res+EN_ADC);
						}
					else {
						offset_read0 = true;
						VoltagesTemps[FPGA0TEMP].enabled = true;
						VoltagesTemps[FPGA0FEVA].enabled = true;
					}
					//delay_ms(1000);
				}
				else {
					timer++;
#ifdef XILINX_DEBUG_TEXT
					DEBUG_PRINT2("Timer 0 Hit\n");
#endif
					}
				if(offset_read0){
					mcu_exec_step=exchangedatablockxilinx4;
					framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
					XilinxBlockNewInfo = true;
					XO3_ReadXilinx((xil_sysmon_fpga0_offset+XIL_SYSMON_FPGA0_FE_CURRENT_OFF), &res);
					framWrite(FRAM_FPGA0_FE_CURRENT, res);
					VoltagesTemps[FPGA0FEVA].ADCread = uint16_t(res);
					XO3_ReadXilinx((xil_sysmon_fpga0_offset+XIL_SYSMON_FPGA0_TEMP), &res);
					framWrite(FRAM_FPGA0_TEMP, res);
					VoltagesTemps[FPGA0TEMP].ADCread = uint16_t(res);
#ifdef XILINX_DEBUG_TEXT
					DEBUG_PRINT2("Xilinx SysMon FE Current 0 - %x - SysMon OFFSET %x\n", res, xil_sysmon_fpga0_offset);
#endif
				}
			}
			if (((xil_done == 2) && (xil_init == 2)) || ((xil_done == 3) && (xil_init == 3))){
				if (timer > 5){
					mcu_exec_step=exchangedatablockxilinx5;
					framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
					// Xilinx System Monitor load base address from CPLD
					XO3_ReadXilinx(XIL_SYSMON_FPGA1_OFFSET, &xil_sysmon_fpga1_offset);
					// Check version
					XO3_ReadXilinx(itpm_cpld_wb_c2c1, &res2);
					if (res2 < 0x1021752) {
						offset_read1 = false;
						DEBUG_PRINT1("Xil1 FW Ver 0x%x too old. System Monitor 1 disabled\n", res2);
						xil1_sm_disabled = true;
						//XO3_WriteByte(itpm_cpld_regfile_enable, res+EN_ADC);
						} 
					else {
						offset_read1 = true;
						VoltagesTemps[FPGA1TEMP].enabled = true;
						VoltagesTemps[FPGA1FEVA].enabled = true;
					}
					//delay_ms(1000);
				}
				else {
					timer++;
#ifdef XILINX_DEBUG_TEXT
					DEBUG_PRINT2("Timer 1 Hit\n");
#endif
					}
				if (offset_read1){
					mcu_exec_step=exchangedatablockxilinx6;
					framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
					XilinxBlockNewInfo = true;
					XO3_ReadXilinx((xil_sysmon_fpga0_offset+XIL_SYSMON_FPGA1_FE_CURRENT_OFF+itpm_cpld_wb_c2c1), &res);
					framWrite(FRAM_FPGA1_FE_CURRENT, res);
					VoltagesTemps[FPGA1FEVA].ADCread = uint16_t(res);
					XO3_ReadXilinx((xil_sysmon_fpga1_offset+XIL_SYSMON_FPGA1_TEMP), &res);
					framWrite(FRAM_FPGA1_TEMP, res);
					VoltagesTemps[FPGA1TEMP].ADCread = uint16_t(res);
#ifdef XILINX_DEBUG_TEXT					
					DEBUG_PRINT2("Xilinx SysMon FE Current 1 - %x - SysMon OFFSET %x\n", res, xil_sysmon_fpga1_offset);
#endif
				}
			}
		}
	}
	smap_unlock();
	//XO3_WriteByte(itpm_cpld_lock_lock_smap, 0x0); // Clear Xilinx Bus Ownership
}

void exchangeDataBlock(){
	uint32_t res = 0x0;
	mcu_exec_step=exchangedatablock;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	framWrite(FRAM_ADC_SW_AVDD1,			VoltagesTemps[0].ADCread);
	framWrite(FRAM_ADC_SW_AVDD2,			VoltagesTemps[1].ADCread);
	framWrite(FRAM_ADC_AVDD3,				VoltagesTemps[2].ADCread);
	framWrite(FRAM_ADC_MAN_1V2,				VoltagesTemps[3].ADCread);
	framWrite(FRAM_ADC_DDR0_VREF,			VoltagesTemps[4].ADCread);
	framWrite(FRAM_ADC_DDR1_VREF,			VoltagesTemps[5].ADCread);
	framWrite(FRAM_ADC_VM_DRVDD,			VoltagesTemps[6].ADCread);
	framWrite(FRAM_ADC_VIN_SCALED,			VoltagesTemps[7].ADCread);
	framWrite(FRAM_ADC_VM_MAN3V3,			VoltagesTemps[8].ADCread);
	framWrite(FRAM_ADC_VM_MAN1V8,			VoltagesTemps[9].ADCread);
	framWrite(FRAM_ADC_MON_5V0,				VoltagesTemps[10].ADCread);
	framWrite(FRAM_ADC_MGT_AV,				VoltagesTemps[11].ADCread);
	framWrite(FRAM_ADC_MGT_AVTT,			VoltagesTemps[12].ADCread);	
	framWrite(FRAM_ADC_INTERNAL_MCU_TEMP,	SAMinternalTempConv(uint32_t(VoltagesTemps[13].ADCread)));
	
	//framWrite(FRAM_BOARD_TEMP, ADT7408_temp_raw);
	framWrite(FRAM_BOARD_TEMP, ADT7408_temp);
	
	framWrite(FRAM_MCU_COMPLETE_ADC_COUNTER, InternalCounter_ADC_update);	
	
	framRead(FRAM_WARN_ALARM_UPDATE, &res);
	if (res == 0x1) SKAalarmUpdate();
	
	exchangeDataBlockXilinx();
	
// 	framRead(FRAM_THRESHOLD_ENABLE_MASK, &reg_ThresholdEnable);
// 	if (reg_ThresholdEnable && 0x80000000) {
// 		int x = 0;
// 		for (uint32_t i = FRAM_ALARM_THR_SW_AVDD1; i < 0x1D8; i += 4){
// 			uint32_t temp;
// 			framRead(i, &temp);
// 			reg_ThresholdVals[0][x] = (temp && 0xFFFF); // High Threshold
// 			reg_ThresholdVals[1][x] = (temp && 0xFFFF0000) >> 16; // Low Threshold
// 			x+;+
// 		}
// 	}
	
}

void ADCreadSingle() { // Single read, much FASTER
	mcu_exec_step=adcreadsingle;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	if (ADC->INTFLAG.bit.RESRDY == 1){
		//uint32_t valueRead = ADC->RESULT.reg;
		//adcArrgh[1][anaReadPos] = ADC->RESULT.reg; // Save ADC read to the array
		
		float ADC_voltage = (ADC_STEP * ADC->RESULT.reg) * 1000;
		///??? float Voltage = (ADC_voltage * ADC_STEP) * 1000;
		
		if (VoltagesTemps[anaReadPos].divider == 0) VoltagesTemps[anaReadPos].ADCread = (uint16_t)ADC_voltage;
		else VoltagesTemps[anaReadPos].ADCread = (uint16_t)(ADC_voltage * VoltagesTemps[anaReadPos].divider);
		
		//if (adcDivider[anaReadPos] == 0) adcArrgh[1][anaReadPos] = (uint16_t)ADC_voltage;
		//else adcArrgh[1][anaReadPos] = (uint16_t)(ADC_voltage * adcDivider[anaReadPos]);
		
		DEBUG_PRINT3("ANALOG_READ > %d %i\n", anaReadPos, VoltagesTemps[anaReadPos].ADCread);
		
		ADCsync();
		ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
		
		SKAalarmManage();
		
		if (anaReadPos >= ADCCOLUMNS-1){
			anaReadPos = 0;
			anaNotReady = false;
			InternalCounter_ADC_update++;
		}
		else anaReadPos++;
		ADC->INPUTCTRL.bit.MUXPOS = VoltagesTemps[anaReadPos].ADCpin;
		ADC->INTFLAG.bit.RESRDY = 1;
		ADC->SWTRIG.bit.START = 1;
	}
}

void ADCstart() { // Single read, much FASTER
	DEBUG_PRINT("Analog Start\n");
	//REG_PM_APBCMASK |= 0x10000; // Enable bus Clock
	//REG_GCLK_CLKCTRL = 0x4001E;
	//REG_ADC_SAMPCTRL = 5;
	ADC->INPUTCTRL.bit.MUXNEG = 0x18; // Mux NEG GND
	SYSCTRL->VREF.bit.TSEN = 1; // Enable TSENOR
	//REG_ADC_REFCTRL = 0;
	//REG_ADC_CTRLA = 2;
	
	ADCsync();
	ADC->CTRLA.bit.ENABLE = 0x01;              // Enable ADC

	ADC->INTFLAG.bit.RESRDY = 1;               // Data ready flag cleared

	ADCsync();
	ADC->SWTRIG.bit.START = 1;                 // Start ADC conversion
	
	for (int i=0; i < ADCCOLUMNS; i++) ADCreadSingle();
	//while (anaReadPos < ADCCOLUMNS) ADCreadSingle();
	anaReadPos = 0;
}

void TWIdataBlock(void){
	int status;
	uint32_t retvalue = 0xffffffff;
	mcu_exec_step=twidatablock;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	DEBUG_PRINT3("TWI Data Block\n");
	// i2c1
   //readBoardTemp(&ADT7408_temp, &ADT7408Regs[3]); // Disabled for errors
	set_i2c_pwd();
	status = twiFpgaWrite(0x30, 1, 2, 0x05, &ADT7408_temp_raw, i2c1); //temp_value 0x30
	if(status == 0)
	{
		i2c_connection_error=0;
		if ((ADT7408_temp_raw & 0x1000) == 0x1000)
		{
			DEBUG_PRINT("TWI Data Read neg Val %x\n",ADT7408_temp_raw);
			ADT7408_temp=0x8000;
		}
		else
			ADT7408_temp=ADT7408_temp_raw&0xfff;		
		VoltagesTemps[BOARDTEMP].ADCread = (uint16_t)ADT7408_temp;
	}
	else if (status == 2)
	{
		DEBUG_PRINT("I2C read failed, ACK_Error detected\n");
		XO3_Read(itpm_cpld_i2c_password,&retvalue);
		DEBUG_PRINT("I2C password high %x \n",retvalue);
		i2c_connection_error++;
		//VoltagesTemps[BOARDTEMP].ADCread = (ADT7408_temp | 0x8000);
	}
	else if (status == -3)
	{
			DEBUG_PRINT("I2C read failed, MCU Lock Failed\n");
			i2c_connection_error++;
			//VoltagesTemps[BOARDTEMP].ADCread = (ADT7408_temp | 0x8000);
	}
	else
	{		
		DEBUG_PRINT("I2C read failed\n");
		i2c_connection_error++;
		//VoltagesTemps[BOARDTEMP].ADCread = (ADT7408_temp | 0x8000);
	}
	
	//XO3_WriteByte(fram_ADT7408_M_1_temp_val + fram_offset, retvalue);
	retvalue = 0xffffffff;
	
	
}

void StartupLoadSettings(void){
	mcu_exec_step=startuploadsettings;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	DEBUG_PRINT("Push Settings to CPLD\n");
	
	framWrite(FRAM_MCU_POOLING_INTERVAL, DEFAULT_POLLING_INTERVAL);
	
	framWrite(FRAM_WARN_ALARM_UPDATE, SETTING_WARN_ALARM_UPDATE);
	
	framWrite(FRAM_POWERGOOD, 0x0);
	
	framWrite(FRAM_WARN_THR_SW_AVDD1			, SETTING_WARN_THR_SW_AVDD1				);
	framWrite(FRAM_WARN_THR_SW_AVDD2			, SETTING_WARN_THR_SW_AVDD2				);
	framWrite(FRAM_WARN_THR_AVDD3				, SETTING_WARN_THR_AVDD3				);
	framWrite(FRAM_WARN_THR_MAN_1V2				, SETTING_WARN_THR_MAN_1V2				);
	framWrite(FRAM_WARN_THR_DDR0_VREF			, SETTING_WARN_THR_DDR0_VREF			);
	framWrite(FRAM_WARN_THR_DDR1_VREF			, SETTING_WARN_THR_DDR1_VREF			);
	framWrite(FRAM_WARN_THR_VM_DRVDD			, SETTING_WARN_THR_VM_DRVDD				);
	framWrite(FRAM_WARN_THR_VIN_SCALED			, SETTING_WARN_THR_VIN_SCALED			);
	framWrite(FRAM_WARN_THR_VM_MAN3V3			, SETTING_WARN_THR_VM_MAN3V3			);
	framWrite(FRAM_WARN_THR_VM_MAN1V8			, SETTING_WARN_THR_VM_MAN1V8			);
	framWrite(FRAM_WARN_THR_MON_5V0				, SETTING_WARN_THR_MON_5V0				);
	framWrite(FRAM_WARN_THR_MGT_AV				, SETTING_WARN_THR_MGT_AV				);
	framWrite(FRAM_WARN_THR_MGT_AVTT			, SETTING_WARN_THR_MGT_AVTT				);
	framWrite(FRAM_WARN_THR_INTERNAL_MCU_TEMP	, SETTING_WARN_THR_INTERNAL_MCU_TEMP	);
	framWrite(FRAM_WARN_THR_BOARD_TEMP			, SETTING_WARN_THR_BOARD_TEMP			);
	framWrite(FRAM_WARN_THR_FPGA0_TEMP			, SETTING_WARN_THR_FPGA0_TEMP			);
	framWrite(FRAM_WARN_THR_FPGA1_TEMP			, SETTING_WARN_THR_FPGA1_TEMP			);
	framWrite(FRAM_WARN_THR_FPGA0_FE_CURRENT	, SETTING_WARN_THR_FPGA0_FE_CURRENT		);
	framWrite(FRAM_WARN_THR_FPGA1_FE_CURRENT	, SETTING_WARN_THR_FPGA1_FE_CURRENT		);
	
	framWrite(FRAM_ALARM_THR_SW_AVDD1			, SETTING_ALARM_THR_SW_AVDD1			);
	framWrite(FRAM_ALARM_THR_SW_AVDD2			, SETTING_ALARM_THR_SW_AVDD2			);
	framWrite(FRAM_ALARM_THR_AVDD3				, SETTING_ALARM_THR_AVDD3				);
	framWrite(FRAM_ALARM_THR_MAN_1V2			, SETTING_ALARM_THR_MAN_1V2				);
	framWrite(FRAM_ALARM_THR_DDR0_VREF			, SETTING_ALARM_THR_DDR0_VREF			);
	framWrite(FRAM_ALARM_THR_DDR1_VREF			, SETTING_ALARM_THR_DDR1_VREF			);
	framWrite(FRAM_ALARM_THR_VM_DRVDD			, SETTING_ALARM_THR_VM_DRVDD			);
	framWrite(FRAM_ALARM_THR_VIN_SCALED			, SETTING_ALARM_THR_VIN_SCALED			);
	framWrite(FRAM_ALARM_THR_VM_MAN3V3			, SETTING_ALARM_THR_VM_MAN3V3			);
	framWrite(FRAM_ALARM_THR_VM_MAN1V8			, SETTING_ALARM_THR_VM_MAN1V8			);
	framWrite(FRAM_ALARM_THR_MON_5V0			, SETTING_ALARM_THR_MON_5V0				);
	framWrite(FRAM_ALARM_THR_MGT_AV				, SETTING_ALARM_THR_MGT_AV				);
	framWrite(FRAM_ALARM_THR_MGT_AVTT			, SETTING_ALARM_THR_MGT_AVTT			);
	framWrite(FRAM_ALARM_THR_INTERNAL_MCU_TEMP	, SETTING_ALARM_THR_INTERNAL_MCU_TEMP	);
	framWrite(FRAM_ALARM_THR_BOARD_TEMP			, SETTING_ALARM_THR_BOARD_TEMP			);
	framWrite(FRAM_ALARM_THR_FPGA0_TEMP			, SETTING_ALARM_THR_FPGA0_TEMP			);
	framWrite(FRAM_ALARM_THR_FPGA1_TEMP			, SETTING_ALARM_THR_FPGA1_TEMP			);
	framWrite(FRAM_ALARM_THR_FPGA0_FE_CURRENT	, SETTING_ALARM_THR_FPGA0_FE_CURRENT	);
	framWrite(FRAM_ALARM_THR_FPGA1_FE_CURRENT	, SETTING_ALARM_THR_FPGA1_FE_CURRENT	);
	
}

void SKAsystemMonitorStart(){
	
	mcu_exec_step=systemmonitorstart;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	
	// Create Struct
	
	DEBUG_PRINT("Load Voltages and Temperature Settings\n");
	
	for (int i = 0; i < (ADCCOLUMNS + TEMPS_SENSOR); i++){
		VoltagesTemps[i].enabled = false;
	}
	
	// Voltages	
	// ADC4 - PA04 - SW_AVDD1	
	VoltagesTemps[SWAVDD1].ADCpin				= 4;
	VoltagesTemps[SWAVDD1].divider				= 0;
	VoltagesTemps[SWAVDD1].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_SW_AVDD1>>16);
	VoltagesTemps[SWAVDD1].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_SW_AVDD1);
	VoltagesTemps[SWAVDD1].warningTHRupper		= uint16_t(SETTING_WARN_THR_SW_AVDD1>>16);
	VoltagesTemps[SWAVDD1].warningTHRdowner		= uint16_t(SETTING_WARN_THR_SW_AVDD1);
	VoltagesTemps[SWAVDD1].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC5 - PA05 - SW_AVDD2
	VoltagesTemps[SWAVDD2].ADCpin				= 5;
	VoltagesTemps[SWAVDD2].divider				= 0;
	VoltagesTemps[SWAVDD2].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_SW_AVDD2>>16);
	VoltagesTemps[SWAVDD2].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_SW_AVDD2);
	VoltagesTemps[SWAVDD2].warningTHRupper		= uint16_t(SETTING_WARN_THR_SW_AVDD2>>16);
	VoltagesTemps[SWAVDD2].warningTHRdowner		= uint16_t(SETTING_WARN_THR_SW_AVDD2);
	VoltagesTemps[SWAVDD2].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC6 - PA06 - AVDD3
	VoltagesTemps[SWAVDD3].ADCpin				= 6;
	VoltagesTemps[SWAVDD3].divider				= 0;
	VoltagesTemps[SWAVDD3].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_AVDD3>>16);
	VoltagesTemps[SWAVDD3].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_AVDD3);
	VoltagesTemps[SWAVDD3].warningTHRupper		= uint16_t(SETTING_WARN_THR_AVDD3>>16);
	VoltagesTemps[SWAVDD3].warningTHRdowner		= uint16_t(SETTING_WARN_THR_AVDD3);
	VoltagesTemps[SWAVDD3].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC7 - PA07 - MAN_1V2
	VoltagesTemps[MAN1V2].ADCpin				= 7;
	VoltagesTemps[MAN1V2].divider				= 0;
	VoltagesTemps[MAN1V2].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_MAN_1V2>>16);
	VoltagesTemps[MAN1V2].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_MAN_1V2);
	VoltagesTemps[MAN1V2].warningTHRupper		= uint16_t(SETTING_WARN_THR_MAN_1V2>>16);
	VoltagesTemps[MAN1V2].warningTHRdowner		= uint16_t(SETTING_WARN_THR_MAN_1V2);
	VoltagesTemps[MAN1V2].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	VoltagesTemps[MAN1V2].enabled				= true;
	
	// ADC16 - PA08 - DDR0_VREF
	VoltagesTemps[DDR0VREF].ADCpin				= 16;
	VoltagesTemps[DDR0VREF].divider				= 0;
	VoltagesTemps[DDR0VREF].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_DDR0_VREF>>16);
	VoltagesTemps[DDR0VREF].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_DDR0_VREF);
	VoltagesTemps[DDR0VREF].warningTHRupper		= uint16_t(SETTING_WARN_THR_DDR0_VREF>>16);
	VoltagesTemps[DDR0VREF].warningTHRdowner	= uint16_t(SETTING_WARN_THR_DDR0_VREF);
	VoltagesTemps[DDR0VREF].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC17 - PA09 - DDR1_VREF
	VoltagesTemps[DDR1VREF].ADCpin				= 17;
	VoltagesTemps[DDR1VREF].divider				= 0;
	VoltagesTemps[DDR1VREF].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_DDR1_VREF>>16);
	VoltagesTemps[DDR1VREF].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_DDR1_VREF);
	VoltagesTemps[DDR1VREF].warningTHRupper		= uint16_t(SETTING_WARN_THR_DDR1_VREF>>16);
	VoltagesTemps[DDR1VREF].warningTHRdowner	= uint16_t(SETTING_WARN_THR_DDR1_VREF);
	VoltagesTemps[DDR1VREF].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC18 - PA10 - VM_DRVDD
	VoltagesTemps[VMDRVDD].ADCpin				= 18;
	VoltagesTemps[VMDRVDD].divider				= 0;
	VoltagesTemps[VMDRVDD].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VM_DRVDD>>16);
	VoltagesTemps[VMDRVDD].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_DRVDD);
	VoltagesTemps[VMDRVDD].warningTHRupper		= uint16_t(SETTING_WARN_THR_VM_DRVDD>>16);
	VoltagesTemps[VMDRVDD].warningTHRdowner		= uint16_t(SETTING_WARN_THR_VM_DRVDD);
	VoltagesTemps[VMDRVDD].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	VoltagesTemps[VMDRVDD].enabled				= true;
	
	// ADC8 - PB00 - VIN_SCALED
	VoltagesTemps[VINSCALED].ADCpin				= 8;
	VoltagesTemps[VINSCALED].divider			= VIN_SCALED_DIVIDER;  // 12.5; // 12000/960
	VoltagesTemps[VINSCALED].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_VIN_SCALED>>16);
	VoltagesTemps[VINSCALED].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VIN_SCALED);
	VoltagesTemps[VINSCALED].warningTHRupper	= uint16_t(SETTING_WARN_THR_VIN_SCALED>>16);
	VoltagesTemps[VINSCALED].warningTHRdowner	= uint16_t(SETTING_WARN_THR_VIN_SCALED);
	VoltagesTemps[VINSCALED].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	VoltagesTemps[VINSCALED].enabled			= true;
	
	// ADC9 - PB01 - VM_MAN3V3
	VoltagesTemps[MAN3V3].ADCpin				= 9;
	VoltagesTemps[MAN3V3].divider				= VM_MAN3V3_DIVIDER; // 3.74782; // 3300 / 880.51
	VoltagesTemps[MAN3V3].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_VM_MAN3V3>>16);
	VoltagesTemps[MAN3V3].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_MAN3V3);
	VoltagesTemps[MAN3V3].warningTHRupper		= uint16_t(SETTING_WARN_THR_VM_MAN3V3>>16);
	VoltagesTemps[MAN3V3].warningTHRdowner		= uint16_t(SETTING_WARN_THR_VM_MAN3V3);
	VoltagesTemps[MAN3V3].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC10 - PB02 - VM_MAN1V8
	VoltagesTemps[MAN1V8].ADCpin				= 10;
	VoltagesTemps[MAN1V8].divider				= VM_MAN1V8_DIVIDER; // 2.73914; // 1800/657,14
	VoltagesTemps[MAN1V8].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_VM_MAN1V8>>16);
	VoltagesTemps[MAN1V8].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_VM_MAN1V8);
	VoltagesTemps[MAN1V8].warningTHRupper		= uint16_t(SETTING_WARN_THR_VM_MAN1V8>>16);
	VoltagesTemps[MAN1V8].warningTHRdowner		= uint16_t(SETTING_WARN_THR_VM_MAN1V8);
	VoltagesTemps[MAN1V8].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC11 - PB03 - MON_5V0
	VoltagesTemps[MON5V0].ADCpin				= 11;
	VoltagesTemps[MON5V0].divider				= MON_5V0_DIVIDER; // 2.739726; // 5000/1825
	VoltagesTemps[MON5V0].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_MON_5V0>>16);
	VoltagesTemps[MON5V0].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_MON_5V0);
	VoltagesTemps[MON5V0].warningTHRupper		= uint16_t(SETTING_WARN_THR_MON_5V0>>16);
	VoltagesTemps[MON5V0].warningTHRdowner		= uint16_t(SETTING_WARN_THR_MON_5V0);
	VoltagesTemps[MON5V0].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	VoltagesTemps[MON5V0].enabled				= true;
	
	// ADC14 - PB06 - MGT_AV
	VoltagesTemps[MGTAV].ADCpin					= 14;
	VoltagesTemps[MGTAV].divider				= 0;
	VoltagesTemps[MGTAV].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_MGT_AV>>16);
	VoltagesTemps[MGTAV].alarmTHRdowner			= uint16_t(SETTING_ALARM_THR_MGT_AV);
	VoltagesTemps[MGTAV].warningTHRupper		= uint16_t(SETTING_WARN_THR_MGT_AV>>16);
	VoltagesTemps[MGTAV].warningTHRdowner		= uint16_t(SETTING_WARN_THR_MGT_AV);
	VoltagesTemps[MGTAV].objectType				= itpm_cpld_regfile_global_status_voltage_B;
	
	// ADC15 - PB07 - MGT_AVTT
	VoltagesTemps[MGAVTT].ADCpin				= 15;
	VoltagesTemps[MGAVTT].divider				= 0;
	VoltagesTemps[MGAVTT].alarmTHRupper			= uint16_t(SETTING_ALARM_THR_MGT_AVTT>>16);
	VoltagesTemps[MGAVTT].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_MGT_AVTT);
	VoltagesTemps[MGAVTT].warningTHRupper		= uint16_t(SETTING_WARN_THR_MGT_AVTT>>16);
	VoltagesTemps[MGAVTT].warningTHRdowner		= uint16_t(SETTING_WARN_THR_MGT_AVTT);
	VoltagesTemps[MGAVTT].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// Temperatures	
	// ADC24 - INT 0x18 - Internal Temperature (Need option to enable)
	VoltagesTemps[INTTEMP].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_INTERNAL_MCU_TEMP>>16);
	VoltagesTemps[INTTEMP].warningTHRupper		= uint16_t(SETTING_WARN_THR_INTERNAL_MCU_TEMP>>16);
	VoltagesTemps[INTTEMP].objectType			= itpm_cpld_regfile_global_status_temperature_B;
	VoltagesTemps[INTTEMP].enabled				= true;
	
	VoltagesTemps[BOARDTEMP].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_BOARD_TEMP>>16);
	VoltagesTemps[BOARDTEMP].warningTHRupper	= uint16_t(SETTING_WARN_THR_BOARD_TEMP>>16);
	VoltagesTemps[BOARDTEMP].objectType			= itpm_cpld_regfile_global_status_temperature_B;
	
	// FPGA TEMPS
	VoltagesTemps[FPGA0TEMP].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA0_TEMP>>16);
	VoltagesTemps[FPGA0TEMP].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA0_TEMP>>16);
	VoltagesTemps[FPGA0TEMP].objectType			= itpm_cpld_regfile_global_status_temperature_B;
	
	VoltagesTemps[FPGA1TEMP].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA1_TEMP>>16);
	VoltagesTemps[FPGA1TEMP].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA1_TEMP>>16);
	VoltagesTemps[FPGA1TEMP].objectType			= itpm_cpld_regfile_global_status_temperature_B;
	
	// FPGA FE_CURRENTS
	// FPGA0 FE_CURRENT
	VoltagesTemps[FPGA0FEVA].divider			= 0;
	VoltagesTemps[FPGA0FEVA].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA0_FE_CURRENT>>16);
	VoltagesTemps[FPGA0FEVA].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_FPGA0_FE_CURRENT);
	VoltagesTemps[FPGA0FEVA].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA0_FE_CURRENT>>16);
	VoltagesTemps[FPGA0FEVA].warningTHRdowner	= uint16_t(SETTING_WARN_THR_FPGA0_FE_CURRENT);
	VoltagesTemps[FPGA0FEVA].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	// FPGA0 FE_CURRENT
	VoltagesTemps[FPGA1FEVA].divider			= 0;
	VoltagesTemps[FPGA1FEVA].alarmTHRupper		= uint16_t(SETTING_ALARM_THR_FPGA1_FE_CURRENT>>16);
	VoltagesTemps[FPGA1FEVA].alarmTHRdowner		= uint16_t(SETTING_ALARM_THR_FPGA1_FE_CURRENT);
	VoltagesTemps[FPGA1FEVA].warningTHRupper	= uint16_t(SETTING_WARN_THR_FPGA1_FE_CURRENT>>16);
	VoltagesTemps[FPGA1FEVA].warningTHRdowner	= uint16_t(SETTING_WARN_THR_FPGA1_FE_CURRENT);
	VoltagesTemps[FPGA1FEVA].objectType			= itpm_cpld_regfile_global_status_voltage_B;
	
	DEBUG_PRINT2("Temps, Voltages and FE Currents Loaded:\n");
	for (int i = 0; i < (ADCCOLUMNS + TEMPS_SENSOR + FPGA_FE_CURRENT); i++){
		VoltagesTemps[i].ADCread = 0;
		VoltagesTemps[i].alarmTriggered = false;
		DEBUG_PRINT2("PIN %d - Divider %f\n", VoltagesTemps[i].ADCpin, VoltagesTemps[i].divider);
	}
	
	ADCstart();
	
}

int SKAenableCheck(void){
	int ret;
	uint32_t enable, enableshadow, bypass, alarm_state, safety_override;
	
	XO3_Read(itpm_cpld_regfile_enable, &enable);
	XO3_Read(itpm_cpld_regfile_enable_shadow, &enableshadow);
	XO3_Read(itpm_cpld_regfile_safety_override, &bypass);
	framRead(FRAM_MCU_SA_OV, &safety_override);
	
	if (bypass == 0x1) { TPMoverride = true; }
	else if (safety_override == 0xBADC0DE) { TPMoverrideAutoShutdown = true; DEBUG_PRINT("SAFETY OVERRIDE ENABLED\n"); }
	else if (safety_override == 0x0) { TPMoverrideAutoShutdown = false; DEBUG_PRINT("SAFETY OVERRIDE DISABLED\n");}
	else { TPMoverride = false; }
	
	if (enable != enableshadow){
		if (!TPMpowerLock){
			XO3_WriteByte(itpm_cpld_regfile_enable_shadow, enable);
			EnableShadowRegister = enable;
			DEBUG_PRINT("Powered devices - %x\n", enable);
			ret = 0;
		}
		else if (TPMoverride){
			XO3_WriteByte(itpm_cpld_regfile_enable_shadow, enable);
			DEBUG_PRINT("Powered devices - %x - BYPASS ENFORCED\n", enable);
			ret = 1;
		}
		else if (enable == 0x0){
			XO3_WriteByte(itpm_cpld_regfile_enable_shadow, enable);
			DEBUG_PRINT("Un-powered alle devices");
			if (TPMpowerLock) DEBUG_PRINT(" allowed even if board is in locked state.");
			ret = 2;
		}
		else {
			//XO3_WriteByte(itpm_cpld_regfile_enable_shadow, enable);
			DEBUG_PRINT("Power request DENIED for %x - Power Locked\n", enable);
			SKAPower(0,0,0,0,0);
			ret = -1;
		}
	}
	
	if (enable & EN_ADC){ // Enable ADC
		VoltagesTemps[SWAVDD1].enabled = true;
		VoltagesTemps[SWAVDD2].enabled = true;
		VoltagesTemps[SWAVDD3].enabled = true;
		VoltagesTemps[VMDRVDD].enabled = true;
		VoltagesTemps[BOARDTEMP].enabled = true;
		DEBUG_PRINT2("Enabled warnings and alarms for SWAVDD1, SWAVVD2, SWAVDD3, DRVdd, BoardTemp\n");
	}
	else {
		VoltagesTemps[SWAVDD1].enabled = false;
		VoltagesTemps[SWAVDD2].enabled = false;
		VoltagesTemps[SWAVDD3].enabled = false;
		VoltagesTemps[VMDRVDD].enabled = false;
		VoltagesTemps[BOARDTEMP].enabled = false;
		DEBUG_PRINT2("Disabled warnings and alarms for SWAVDD1, SWAVVD2, SWAVDD3, DRVdd, BoardTemp\n");
	}
	
	// FE Wanring and Alarms are enabled and disabled in exchangeDataBlockXilinx() function, due checks for Xilixn system monitor
	
	if (enable & EN_FPGA){ // Enable FPGA
		VoltagesTemps[MGAVTT].enabled = true;
		VoltagesTemps[MGTAV].enabled = true;
		VoltagesTemps[DDR0VREF].enabled = true;
		VoltagesTemps[DDR1VREF].enabled = true;
		DEBUG_PRINT2("Enabled warnings and alarms for MGT_Avtt, MGT_Avcc, DDR0Vref, DDR1Vref\n");
	}
	else {
		VoltagesTemps[MGAVTT].enabled = false;
		VoltagesTemps[MGTAV].enabled = false;
		VoltagesTemps[DDR0VREF].enabled = false;
		VoltagesTemps[DDR1VREF].enabled = false;
		DEBUG_PRINT2("Disabled warnings and alarms for MGT_Avtt, MGT_Avcc, DDR0Vref, DDR1Vref\n");
	}
	
	return ret;
	
}

static void IRQfromCPLD(void){
	irqExternalFPGA = true;
}

static void IRQpgFPGA(void){
	if(irqPG == 0) irqPG = PG_FPGA_irq;
}

static void IRQpgFE(void){
	if(irqPG == 0) irqPG = PG_FE_irq;
}

static void IRQpgAVDD(void){
	if(irqPG == 0) irqPG = PG_AVDD_irq;
}

static void IRQpgMAN(void){
	if(irqPG == 0) irqPG = PG_MAN_irq;
}

static void IRQpgADC(void){
	if(irqPG == 0) irqPG = PG_ADC_irq;
}

void IRQinternalCPLDhandler(void){
		uint32_t irq_status, irq_mask, res;
		
		mcu_exec_step=irqinternalcpldhandler;
		framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		
		framRead(FRAM_MCU_VERSION, &res);		
		if (res != _build_version){
			DEBUG_PRINT1("CRITICAL ERROR: IRQ NOT HANDLED, no SPI bus communication. Expected %x read %x\n", _build_version, res);
			return;
		}		
		
		XO3_Read(itpm_cpld_intc_status, &irq_status);
		XO3_Read(itpm_cpld_intc_mask, &irq_mask);
		DEBUG_PRINT2("IRQ CPLD Val: %x - Mask %x\n", irq_status, irq_mask);
		if (irq_status == 0xFFFFFFFF){
			DEBUG_PRINT1("CRITICAL ERROR: IRQ NOT HANDLED, no SPI bus communication. IRQ %x is not a valid value\n", irq_status);
			return;
		}
		// Call	
		if ((irq_status & ENABLE_UPDATE_int) == ENABLE_UPDATE_int)  { SKAenableCheck(); } // Enable Interrupt
		if ((irq_status & FRAM_UPDATE_int) == FRAM_UPDATE_int) { SKAalarmUpdate(); DEBUG_PRINT("FRAM Data Updated IRQ\n"); }
		XO3_WriteByte(itpm_cpld_intc_ack, MASK_default_int); // Clean FPGA IRQ
		irqExternalFPGA = false;
}

void IRQinternalPGhandler(void){
	uint32_t powergood_register = 0;
	mcu_exec_step=irqinternalpghandler;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	if(gpio_get_pin_level(PG_FPGA)) powergood_register += PG_FPGA_irq;
	if(gpio_get_pin_level(PG_FE  )) powergood_register += PG_FE_irq;
	if(gpio_get_pin_level(PG_AVDD)) powergood_register += PG_AVDD_irq;
	if(gpio_get_pin_level(PG_MAN )) powergood_register += PG_MAN_irq;
	if(gpio_get_pin_level(PG_ADC )) powergood_register += PG_ADC_irq;
	
	framWrite(FRAM_POWERGOOD, powergood_register);
	
	DEBUG_PRINT("IRQ PowerGood status changed - 0x%x\n", powergood_register);
	
	pgood_reg=powergood_register;
	irqPG = 0;
	if (check_pg_en == false)
		check_pg_en=true;
}

void StartupStuff(void){
	uint32_t res;
	int copyeep_ret=0;
	int eth_init_reply=0;
	int eth_init_status=0;
	XO3_Read(itpm_cpld_regfile_date_code, &cpld_fw_vers);
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	mcu_exec_step=startup_stuff;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	res = PM->RCAUSE.reg;
	 DEBUG_PRINT("\nRESET REASON 0x%x", res);
	if (PM->RCAUSE.bit.POR) { DEBUG_PRINT(" - Power On Reset\n", res); }
	if (PM->RCAUSE.bit.BOD12) { DEBUG_PRINT(" - BrownOut 12\n", res); }
	if (PM->RCAUSE.bit.BOD33) { DEBUG_PRINT(" - BrownOut 33\n", res); }
	if (PM->RCAUSE.bit.EXT) { DEBUG_PRINT(" - External Reset (PIN)\n", res); }
	//if (PM->RCAUSE.bit.WDT) { DEBUG_PRINT("\nRESET REASON 0x%x - WatchDog\n", res); }
	if (PM->RCAUSE.bit.SYST) { DEBUG_PRINT(" - System Reset Request (JTAG or MCU)\n", res); }
	//delay_ms(100);

	XO3_Read(itpm_cpld_regfile_date_code, &cpld_fw_vers);
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();

	if (cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
	{
		DEBUG_PRINT("Init CPLD ETH Regs\n");
		do 
		{
			eth_init_status=init_eth_regs_from_eep();
			if (eth_init_status!=0)
			{
				DEBUG_PRINT("Error in init_eth_regs_from_eep\n");
				eth_init_reply++;
			}
			else 
				break;	
		} while(eth_init_reply<10);
		if(eth_init_reply==10)
			DEBUG_PRINT("Init CPLD ETH Regs failed, reply=%d\n",eth_init_reply);
		else
			DEBUG_PRINT("Init CPLD ETH Regs complete, reply=%d\n",eth_init_reply);	
	}
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	DEBUG_PRINT("\nSKA iTPM 1.6 - Debug Enabled\n");
	DEBUG_PRINT("Debug level: %d\n", (int) DEBUG);
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xF0, &res, i2c2);
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xF0, &res, i2c3);	
	DEBUG_PRINT("Version: %x\n", _build_version);
	DEBUG_PRINT("Date: %x\n", _build_date);
	DEBUG_PRINT("GIT Hash: %x\n", BUILD_GIT_HASH);
	DEBUG_PRINT("Bootloader Version: %x\n", RAM_BL_VERSION);
	framWrite(FRAM_MCU_VERSION, _build_version);
	framWrite(FRAM_MCU_COMPILE_DATE, _build_date);
	framWrite(FRAM_MCU_GIT_HASH, BUILD_GIT_HASH);
	framWrite(FRAM_MCU_BOOTLOADER_VERSION, RAM_BL_VERSION);
	DEBUG_PRINT("CPLD Version: %x\n", cpld_fw_vers);
	DEBUG_PRINT("-------------------------------\n\n");

	gpio_set_pin_level(USR_LED1, true);

	framWrite(I2C_PASSWORD_HI_S,0x0);
	framWrite(I2C_PASSWORD_LO_S,0x0);
	framWrite(I2C_REQUEST_S,0x0);
	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();

	
	CheckPowerGoodandEnable();
	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	
	SKAsystemMonitorStart();
	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	
	StartupLoadSettings();
	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	// Interrupt Enable
	XO3_WriteByte(itpm_cpld_intc_mask, (itpm_cpld_intc_mask_M - ENABLE_UPDATE_int - FRAM_UPDATE_int));
	XO3_WriteByte(itpm_cpld_intc_ack, MASK_default_int);	
	ext_irq_register(XO3_LINK0, IRQfromCPLD); // Interrupt from CPLD
	ext_irq_register(PG_FPGA, IRQpgFPGA);
	ext_irq_register(PG_FE, IRQpgFE);
	ext_irq_register(PG_AVDD, IRQpgAVDD);
	ext_irq_register(PG_MAN, IRQpgMAN);
	ext_irq_register(PG_ADC, IRQpgADC);
	
	XO3_WriteByte(itpm_cpld_regfile_safety_override, 0x0);
	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		tpm_wd_update();
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xFE, &res, i2c2);
	twiFpgaWrite(IOEXPANDER, 1, 2, 0xFE, &res, i2c3);
	
	framWrite(FRAM_MCU_COUNTER, InternalCounter_CPLD_update);
	framWrite(FRAM_MCU_COUNTER, InternalCounter_ADC_update);
	framWrite(FRAM_MCU_BOOTLOADER_COMMANDS, 0xFFFFFFFF);
	DEBUG_PRINT("Startup Done\n");	
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;

static void IRQtimerSlow(const struct timer_task *const timer_task){
	irqTimerSlow = true; // Enable Task
}

void taskSlow(){
	static uint8_t errorSPI = 0;
	uint32_t res, res2;
	mcu_exec_step=taskslow;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	
	
	gpio_toggle_pin_level(USR_LED0);
	
	framRead(FRAM_MCU_VERSION, &res);
	if (res == _build_version){
		if (errorSPI > 0){
			DEBUG_PRINT("INFO: SPI Bus revived. Comunication OK\n");
			errorSPI = 0;
		}
		XO3_Read(itpm_cpld_regfile_enable_shadow, &res);
		if(res&EN_ADC == EN_ADC)
			TWIdataBlock();
		else{
			framRead(FRAM_BOARD_ALARM, &res);
			if(res!=0)
			{	
				ADT7408_temp=0x4000 | (0x0fff&ADT7408_temp); 
				VoltagesTemps[BOARDTEMP].ADCread = (uint16_t)ADT7408_temp;
			}
		}
		exchangeDataBlock();
		//mcu_exec_step=taskslow1;
		//framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		framRead(FRAM_MCU_POOLING_INTERVAL, &pollingNew);
		InternalCounter_CPLD_update++;
		if(InternalCounter_CPLD_update % 10 == 0)
			DEBUG_PRINT("Internal Counter %d\n", InternalCounter_CPLD_update); 
		framWrite(FRAM_MCU_COUNTER, InternalCounter_CPLD_update);
		framRead(FRAM_ADC_MGT_AVTT, &res2);

		//DEBUG_PRINT3("FRAM_MCU_POOLING_INTERVAL > %x\n", pollingNew);
		if(pollingNew > 2000){
			pollingNew = 2000;
			framWrite(FRAM_MCU_POOLING_INTERVAL, 2000);
		}
		if ((pollingOld != pollingNew) && (pollingNew != 0) ){
			DEBUG_PRINT("New polling time detected: %x \n", pollingNew);
			timer_stop(&TIMER_0);
			TIMER_0_task1.interval = pollingNew;
			timer_start(&TIMER_0);
			pollingOld = pollingNew;
			DEBUG_PRINT("Pooling Time Changed to %x\n", pollingNew);
		}
		//mcu_exec_step=taskslow2;
		//framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		framRead(FRAM_MCU_BOOTLOADER_COMMANDS, &res2);
		if (res2 < 0xffffffff){
			if (res2 == BL_REQUEST_BOOT){
				RAM_BOOT_TYPE = BL_REQUEST_BOOT;
				RAM_BOOT_TYPE_SHIFT = (BL_REQUEST_BOOT >> 1);
			
				framWrite(FRAM_MCU_BOOTLOADER_COMMANDS, 0x1);
				DEBUG_PRINT("Requested Jump to Bootloader, Goodbye!");
			
				delay_ms(5);
				NVIC_SystemReset();
			}
			else{
				framWrite(FRAM_MCU_BOOTLOADER_COMMANDS, 0xFFFFFFFF);
			}
		}
	}
	else{
		//mcu_exec_step=taskslow3;
		//framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
		DEBUG_PRINT1("CRITICAL ERROR: no SPI bus comunication. Expected %x read %x\n", _build_version, res);
		errorSPI++;
		if (errorSPI > 10){
			DEBUG_PRINT("\n\n!!! CRITICAL ERROR: REBOOT DUE SPI BUS CRITICAL STATE !!!\n");
			NVIC_SystemReset();
		}
	}
	
	irqTimerSlow = false; // Disable Task untile next IRQ
}

void WDT_Handler(void){
	asm("nop");
}






int i2c_manager(void)
{
	uint32_t read_data;
	uint32_t pwd_h=0;
	uint32_t pwd_l=0;
	uint32_t dataIN=0;
	uint32_t statusIN; //0x0 ok - 0x1
	uint8_t busyRetry = 0;
	uint8_t tempbyte0, tempbyte1, tempbyte2, tempbyte3;
	uint8_t timeout = 0;
	uint32_t write_error=0;
	uint32_t read_error=0;
	bool nack_received=false;
	
	mcu_exec_step=i2c_manage;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	
	framRead(I2C_ACNOWLEDGE_S,&read_data);
	if (read_data!=last_i2c_ack)
	{
		DEBUG_PRINT("I2C ACK Change %x, i2c state %x \n",read_data,i2c_ctrl_status);
		last_i2c_ack=read_data;
	}
	framRead(I2C_REQUEST_S,&read_data);
	if (read_data!=last_i2c_req)
	{
		DEBUG_PRINT("I2C REQ Change %x, i2c state %x \n",read_data,i2c_ctrl_status);
		last_i2c_req=read_data;
	}
	if (i2c_ctrl_status_last!=i2c_ctrl_status)
	{
		DEBUG_PRINT("I2C Status Change i2c last %x, current state %x \n",i2c_ctrl_status_last,i2c_ctrl_status);
		i2c_ctrl_status_last=i2c_ctrl_status;
	}
	
	
	
	
	if (i2c_ctrl_status==waiting_first_req)
	{
		framRead(I2C_REQUEST_S,&read_data);
		if (read_data!=0)
		{
			framRead(I2C_REQUEST_S,&read_data);
			if (read_data==0)
			{
				DEBUG_PRINT("Error request low after request detection\n");
				return 0;
			}
			DEBUG_PRINT("request_received: %x \n",read_data);	
			//manage I2 Password
			timeout=0;
			do 
			{
				framRead(I2C_PASSWORD_HI_S,&pwd_h);
				DEBUG_PRINT3("I2C_PWD_HI_S %x\n",pwd_h);
				XO3_WriteByte(itpm_cpld_i2c_password, pwd_h);
				framRead(I2C_PASSWORD_LO_S,&pwd_l);
				DEBUG_PRINT3("I2C_PWD_LO_S %x\n",pwd_l);
				XO3_WriteByte(itpm_cpld_i2c_password_lo, pwd_l);
				//delay_us(1);
				XO3_Read(itpm_cpld_i2c_password, &read_data);
				framWrite(I2C_PASSWORD_HI_S,read_data);
				framRead(I2C_PASSWORD_HI_S,&pwd_h);
				if(pwd_h!=read_data){
					gpio_set_pin_level(XO3_LINK1, true);
					gpio_set_pin_level(XO3_LINK1, false);
					DEBUG_PRINT("I2C I2C_PASSWORD_HI_S write error\n");
				
				}
				DEBUG_PRINT3("I2C_PWD %x\n",read_data);
				if((read_data&0x10000)==0x10000)
					break;
				timeout++;
			} while (timeout < 1);
			if (timeout==1)
			{
				DEBUG_PRINT("I2C Password not accepted\n");	
				DEBUG_PRINT("I2C_PASSWORD_HI_S %x\n",pwd_h);
				DEBUG_PRINT("I2C_PWD_H %x\n",read_data);
				DEBUG_PRINT("I2C_PWD_L %x\n",pwd_l);
				return -1;
			}
				
			framRead(I2C_TRANSMIT_S,&read_data);	
			if (XO3_WriteByte(itpm_cpld_i2c_transmit, read_data) != 0)
			{
				//return -1;
				write_error++;
			}
			
			framRead(I2C_COMMAND_S,&read_data);
			if (XO3_WriteByte(itpm_cpld_i2c_command, read_data) != 0)
			{
				//return -1;
				write_error++;
			}
			//for (int i = 0; i < 0xffff; i++) asm("nop");
			delay_us(10);
			if (XO3_Read(itpm_cpld_i2c_status, &statusIN)!= 0)
			{
				//return -1;
				read_error++;
			}
			while (statusIN == 0x1 || statusIN == 0x3) {
				busyRetry++;
				if (busyRetry >= MAX_BUSY_RETRY)
				{
					DEBUG_PRINT("I2C busy or not ack\n");
					//return (int)statusIN;
					nack_received=true;
				}
				
				if (XO3_Read(itpm_cpld_i2c_status, &statusIN) !=0)
				{
					//XO3_WriteByte(itpm_cpld_lock_mlock0, 0xffffffff); // Clear I2C Ownership
					//return -1;
					read_error++;
				}
			}
			if (write_error==0 && read_error==0 )
			{
				if(nack_received)
				{
					framWrite(I2C_STATUS_S,statusIN);
					framWrite(I2C_RECEIVE_S,dataIN);
					framWrite(I2C_ACNOWLEDGE_S,0x1);
					i2c_ctrl_status=ack_send;
					DEBUG_PRINT("I2C ACK SEND OP NAK\n");
				}
				else
				{
					if (XO3_Read(itpm_cpld_i2c_receive, &dataIN) != 0)
					{
						//XO3_WriteByte(itpm_cpld_lock_mlock0, 0xffffffff); // Clear I2C Ownership
						read_error++;
					}	
					framWrite(I2C_STATUS_S,statusIN);
					framWrite(I2C_RECEIVE_S,dataIN);
					framWrite(I2C_ACNOWLEDGE_S,0x1);
					framRead(I2C_ACNOWLEDGE_S,&read_data);
					if(read_data!=1)
					{
						gpio_set_pin_level(XO3_LINK1, true);
						gpio_set_pin_level(XO3_LINK1, false);
						DEBUG_PRINT("I2C_ACNOWLEDGE_S write 1 ERROR\n");
					}
					//framWrite(I2C_REQUEST_S,0x0);
					i2c_ctrl_status=ack_send;
					DEBUG_PRINT("I2C ACK SEND OP OK\n");
				}
			}
		}
	}
	else if(i2c_ctrl_status==ack_send)
	{

		framRead(I2C_REQUEST_S,&read_data);
		if(read_data==0)
		{	
			DEBUG_PRINT("I2C REQ DEASSERT RECEIVED\n");	
			XO3_WriteByte(itpm_cpld_i2c_password, 0);
			XO3_WriteByte(itpm_cpld_i2c_password_lo, 0);
			framWrite(I2C_PASSWORD_HI_S,0x0);
			framWrite(I2C_PASSWORD_LO_S,0x0);
			framWrite(I2C_ACNOWLEDGE_S,0x0);
			framRead(I2C_ACNOWLEDGE_S,&read_data);
			if(read_data!=0)
			{
				gpio_set_pin_level(XO3_LINK1, true);
				gpio_set_pin_level(XO3_LINK1, false);
				DEBUG_PRINT("I2C_ACNOWLEDGE_S write 0 ERROR\n");
			}
			
			i2c_ctrl_status=waiting_first_req;		
		}
	}
	
	framRead(ENABLE_ACCESS_CHECK,&read_data);
	if(read_data != 0)
		check_bus_access();
	
	return 0;
}

void i2c_manager_cb(const struct timer_task *const timer_task)
{
	//i2c_manager();
	
}

int main(void)
{
	// Cheap delay startup ~1000 ms total
	//for (int i = 0; i < 0xffff; i++) asm("nop");
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	uint32_t data=0;
	
	//SysTick_Config(1);
	bool pippo = true;
	
	//if (pollingOld == 0x25) pippo = false;
	/*
	while (pippo){
		asm("nop");
	}
	*/
	uint32_t pippo2=0;

	/*
	uint32_t res;
	uint32_t ticket;
	uint32_t to=0;
	
	for (int z=0;z<100;z++)
	{
		DEBUG_PRINT("DEBUG: Check lock, iter %d \n",z);
		bool i2c_ack = false;
		to=0;
		XO3_Read(itpm_cpld_lock_queue_number, &ticket);
		do{
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, ticket); // Request I2C Ownership
			XO3_Read(itpm_cpld_lock_lock_i2c, &res);
			if (res == ticket){ // Check ownership
				DEBUG_PRINT("DEBUG:CPLD MCU Lock: I2C LOCKED from MCU\n");
				i2c_ack = true;
				break;
			}
			else to++;
		} while (to < 20 );
		if (to==20)
			DEBUG_PRINT("DEBUG:CPLD MCU Lock: I2C LOCKED failed\n");
		else
			XO3_WriteByte(itpm_cpld_lock_lock_i2c, 0); 
	}
	*/

	
	/*
	while(data < 3000000)
	{
		XO3_WriteByte(0x30000014,data);
		XO3_Read(0x30000014, &pippo2);
		if(pippo2!=data)
		{
			DEBUG_PRINT("Error detected expected %d, read %d\n", data,pippo2);	
		}
		data=data+1;
		if (data%1000==0) DEBUG_PRINT("Op num %d\n", data);	
	}
	*/
		
	/*
	framWrite(FRAM_MCU_VERSION, _build_version);				
	XO3_Read(itpm_cpld_regfile_date_code, &pippo2);
	XO3_Read(itpm_cpld_regfile_date_code, &pippo2);
	XO3_Read(itpm_cpld_regfile_date_code, &pippo2);*/
	
	//XO3_Read(itpm_cpld_regfile_wdt_sem, &pippo2);
	//DEBUG_PRINT("WD SEM REG VAL 0x%x\n",pippo2);
	
	//XO3_Read(itpm_cpld_regfile_wdt_mcu, &pippo2);
	//DEBUG_PRINT("WD REG VAL 0x%x\n",pippo2);
	//tpm_wd_update();
	mcu_exec_step=pre_startup_stuff;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	StartupStuff();
	mcu_exec_step=post_startup_stuff;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	//gpio_set_pin_level(XO3_LINK1, false); //debug only


	uint32_t mtime, xil;
	uint32_t xil_done = 0xfeffffff;

	//XO3_WriteByte(itpm_cpld_regfile_enable, 0x1f);
	
	//delay_ms(1000);
	
// 	while (1){
// 		XO3_WriteByte(itpm_cpld_regfile_user_reg0, 0x12345678);
// 		XO3_Read(itpm_cpld_regfile_user_reg0, &xil_done);
// 	}
	
	XO3_Read(itpm_cpld_regfile_xilinx, &xil);
	XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
	
	//ExtFlash_SRAMErase(0x01);
	//ExtFlash_FPGA_Prog(0x0f, 0x1, true);
	
	xil_done = 0xffffffff;	
	XO3_BitfieldExtract(itpm_cpld_regfile_xilinx, itpm_cpld_regfile_xilinx_done_M, itpm_cpld_regfile_xilinx_done_B,&xil_done);
	XO3_Read(itpm_cpld_regfile_xilinx, &xil);

	/* Replace with your application code */
	
	
	TIMER_0_task1.interval = 1000;
	TIMER_0_task1.cb       = IRQtimerSlow;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
 	//TIMER_0_task2.interval = 100;
 	//TIMER_0_task2.cb       = i2c_manager_cb;
 	//TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	//timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
	
	DEBUG_PRINT("Default Polling Time set to %x\n", pollingOld);	
	if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
	{	
		//tpm_wd_update();
		tpm_wd_init(WDT_CPLD_REG);
		tpm_wd_update();
	}
	framWrite(ENABLE_ACCESS_CHECK, 0);	//disable access check
	mcu_exec_step=pre_main_loop;
	framWrite(FRAM_MCU_STEP, (uint32_t)mcu_exec_step);
	while (1) {
		uint32_t i2creg;
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
				
		}
		gpio_toggle_pin_level(USR_LED1);
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
		}
		ADCreadSingle();
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
		}
		if (irqExternalFPGA) IRQinternalCPLDhandler();
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
		}
		if (irqTimerSlow) taskSlow();
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
		}
		if (irqPG > 0) IRQinternalPGhandler();
		if(cpld_fw_vers>CPLD_FW_VERSION_LOCK_CHANGE)
		{
			tpm_wd_update();
			i2c_manager();
		}
		if (check_pg_en) CheckPowerGoodandEnable();	
	}
}
