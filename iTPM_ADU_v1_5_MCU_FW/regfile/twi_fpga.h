/*
 * twi_fpga.h
 *
 * Created: 18/12/2018 10:54:41
 *  Author: luca
 */ 


#ifndef TWI_FPGA_H_
#define TWI_FPGA_H_

#define twi_offset				0x00010000
#define twi_command_byte_len	0x0  // R - Max 16B
#define twi_command				0x4  // RW [9:0] command
#define twi_wrbyte				0x4  // RW [23:16] byte number to write
#define twi_rbyte				0x4	 // RW [31:24] byte number to read
#define twi_status				0x8  // R
#define twi_irq					0xc  // RW
#define twi_irq_en				0x10 // RW

#define twi_wrdata				0x100 // Data W reg 32b (Add + data)
#define tiw_rdata				0x200 // Data R reg 32b


#endif /* TWI_FPGA_H_ */

/*
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

package wb_i2c_registers_pkg is
constant WB_FPGA_REGNUM  : integer := 8;  --              Max Number of registers per bank
constant I2C_CMD_BYTELEN : integer := 16;  --             Max Number of registers per bank

type i2c_reg_type is record
--I2C_CMD_BYTELEN  : std_logic_vector(15 downto 0);  --         0x0         R             (others =>'-')
command    : std_logic_vector(9 downto 0);  --                  0x4         RW            (others =>'-')
wrbyte     : std_logic_vector(23 downto 16);  --                0x4         RW            (others =>'-')
rdbyte     : std_logic_vector(31 downto 24);  --                0x4         RW            (others =>'-')
command_we : std_logic_vector(0 downto 0);  --                  non mappato
status     : std_logic_vector(1 downto 0);  --                  0x8         R             (others =>'-')
irq        : std_logic_vector(1 downto 0);  --                  0xc         RW            (others =>'-')
irq_en     : std_logic_vector(1 downto 0);  --                  0x10        RW            (others =>'-')
end record i2c_reg_type;

type i2c_data_type is array (0 to I2C_CMD_BYTELEN/4-1) of std_logic_vector(31 downto 0);

end package wb_i2c_registers_pkg;
*/