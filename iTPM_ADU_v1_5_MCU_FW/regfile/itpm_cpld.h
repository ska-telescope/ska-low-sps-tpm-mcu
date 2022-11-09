#define itpm_cpld_wb_c2c0  	0x00000000	//RW
#define itpm_cpld_wb_c2c0_M	0xFFFFFFFF
#define itpm_cpld_wb_c2c0_R	0x0
#define itpm_cpld_wb_c2c0_D	"Chip 2 Chip Bridge 0"

#define itpm_cpld_wb_c2c1  	0x10000000	//RW
#define itpm_cpld_wb_c2c1_M	0xFFFFFFFF
#define itpm_cpld_wb_c2c1_R	0x0
#define itpm_cpld_wb_c2c1_D	"Chip 2 Chip Bridge 1"

#define itpm_cpld_spi_address  	0x20000000	//RW
#define itpm_cpld_spi_address_M	0x0000FFFF
#define itpm_cpld_spi_address_R	0x0
#define itpm_cpld_spi_address_D	"spi address"

#define itpm_cpld_spi_write_data  	0x20000004	//RW
#define itpm_cpld_spi_write_data_M	0x0000FFFF
#define itpm_cpld_spi_write_data_R	0x0
#define itpm_cpld_spi_write_data_D	"data to be written"

#define itpm_cpld_spi_read_data  	0x20000008	//RW
#define itpm_cpld_spi_read_data_M	0x0000FFFF
#define itpm_cpld_spi_read_data_R	0x0
#define itpm_cpld_spi_read_data_D	"read data"

#define itpm_cpld_spi_chip_select  	0x2000000C	//RW
#define itpm_cpld_spi_chip_select_M	0x0000FFFF
#define itpm_cpld_spi_chip_select_R	0x0
#define itpm_cpld_spi_chip_select_D	"chip select"

#define itpm_cpld_spi_sclk  	0x20000010	//RW
#define itpm_cpld_spi_sclk_M	0x0000FFFF
#define itpm_cpld_spi_sclk_R	0x0
#define itpm_cpld_spi_sclk_D	"serial clock enable"

#define itpm_cpld_spi_cmd  	0x20000014	//RW
#define itpm_cpld_spi_cmd_M	0xFFFFFFFF
#define itpm_cpld_spi_cmd_R	0x0
#define itpm_cpld_spi_cmd_D	"command type"

#define itpm_cpld_spi_cmd_rnw_M	0x00000002
#define itpm_cpld_spi_cmd_rnw_B	1
#define itpm_cpld_spi_cmd_rnw_R	0x0
#define itpm_cpld_spi_cmd_rnw_D	"0x1 = read , 0x0 = write"

#define itpm_cpld_spi_cmd_start_M	0x00000001
#define itpm_cpld_spi_cmd_start_B	0
#define itpm_cpld_spi_cmd_start_R	0x0
#define itpm_cpld_spi_cmd_start_D	"start"

#define itpm_cpld_regfile_date_code  	0x30000000	//R
#define itpm_cpld_regfile_date_code_M	0xffffffff
#define itpm_cpld_regfile_date_code_R	0x00000000
#define itpm_cpld_regfile_date_code_D	"Compile date"

#define itpm_cpld_regfile_ucp_last_psn  	0x30000004	//RW
#define itpm_cpld_regfile_ucp_last_psn_M	0xffffffff
#define itpm_cpld_regfile_ucp_last_psn_R	0x00000000
#define itpm_cpld_regfile_ucp_last_psn_D	"UCP Last PSN"

#define itpm_cpld_regfile_master_rst  	0x30000008	//RW
#define itpm_cpld_regfile_master_rst_M	0x00008000
#define itpm_cpld_regfile_master_rst_R	0x00000000
#define itpm_cpld_regfile_master_rst_D	"Global XO3 Reset"

#define itpm_cpld_regfile_jtag_sel  	0x3000000C	//RW
#define itpm_cpld_regfile_jtag_sel_M	0x00000003
#define itpm_cpld_regfile_jtag_sel_R	0x00000000
#define itpm_cpld_regfile_jtag_sel_D	"JTAG Demux Selector: 0x1 = FPGA0 , 0x2 = FPGA1 , others = daisy chain"

#define itpm_cpld_regfile_user_reg0  	0x30000010	//RW
#define itpm_cpld_regfile_user_reg0_M	0xffffffff
#define itpm_cpld_regfile_user_reg0_R	0xdeadbeef
#define itpm_cpld_regfile_user_reg0_D	"User Register0"

#define itpm_cpld_regfile_user_reg1  	0x30000014	//RW
#define itpm_cpld_regfile_user_reg1_M	0xffffffff
#define itpm_cpld_regfile_user_reg1_R	0x00000000
#define itpm_cpld_regfile_user_reg1_D	"User Register1"

#define itpm_cpld_regfile_ena_stream  	0x30000018	//RW
#define itpm_cpld_regfile_ena_stream_M	0x00000001
#define itpm_cpld_regfile_ena_stream_R	0x00000000
#define itpm_cpld_regfile_ena_stream_D	"Enable C2C Stream"

#define itpm_cpld_regfile_mcu_heartbeat  	0x3000001C	//RW
#define itpm_cpld_regfile_mcu_heartbeat_M	0x00000001
#define itpm_cpld_regfile_mcu_heartbeat_R	0x00000000
#define itpm_cpld_regfile_mcu_heartbeat_D	"MCU Heatbeat"

#define itpm_cpld_regfile_pll  	0x30000020	//RW
#define itpm_cpld_regfile_pll_M	0xFFFFFFFF
#define itpm_cpld_regfile_pll_R	0x0
#define itpm_cpld_regfile_pll_D	"Missing description"

#define itpm_cpld_regfile_pll_status_M	0x00000030
#define itpm_cpld_regfile_pll_status_B	4
#define itpm_cpld_regfile_pll_status_R	0x00000000
#define itpm_cpld_regfile_pll_status_D	"PLL AD9528 Status"

#define itpm_cpld_regfile_pll_resetn_M	0x00000001
#define itpm_cpld_regfile_pll_resetn_B	0
#define itpm_cpld_regfile_pll_resetn_R	0x00000000
#define itpm_cpld_regfile_pll_resetn_D	"PLL AD9528 and AD9550 Reset"

#define itpm_cpld_regfile_xo3_link  	0x30000024	//RW
#define itpm_cpld_regfile_xo3_link_M	0x00000001
#define itpm_cpld_regfile_xo3_link_R	0x00000001
#define itpm_cpld_regfile_xo3_link_D	"XO3_LINK pin status"

#define itpm_cpld_regfile_c2c_pll  	0x30000028	//RW
#define itpm_cpld_regfile_c2c_pll_M	0xFFFFFFFF
#define itpm_cpld_regfile_c2c_pll_R	0x0
#define itpm_cpld_regfile_c2c_pll_D	"Missing description"

#define itpm_cpld_regfile_c2c_pll_phasedir_M	0x00000010
#define itpm_cpld_regfile_c2c_pll_phasedir_B	4
#define itpm_cpld_regfile_c2c_pll_phasedir_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasedir_D	"c2c PLL Phase Direction"

#define itpm_cpld_regfile_c2c_pll_phasestep_M	0x00000001
#define itpm_cpld_regfile_c2c_pll_phasestep_B	0
#define itpm_cpld_regfile_c2c_pll_phasestep_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasestep_D	"c2c PLL Phase Step"

#define itpm_cpld_regfile_c2c_pll_phasesel_M	0x00000300
#define itpm_cpld_regfile_c2c_pll_phasesel_B	8
#define itpm_cpld_regfile_c2c_pll_phasesel_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasesel_D	"c2c PLL Phase Select"

#define itpm_cpld_regfile_c2c_ctrl  	0x3000002C	//RW
#define itpm_cpld_regfile_c2c_ctrl_M	0xFFFFFFFF
#define itpm_cpld_regfile_c2c_ctrl_R	0x0
#define itpm_cpld_regfile_c2c_ctrl_D	"Missing description"

#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_M	0x00000001
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_B	0
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_R	0x00000000
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_D	"Enable C2C Management"

#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_M	0x00000002
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_B	1
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_R	0x00000000
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_D	"Enable C2C Burst"

#define itpm_cpld_regfile_eth10ge  	0x30000030	//RW
#define itpm_cpld_regfile_eth10ge_M	0xFFFFFFFF
#define itpm_cpld_regfile_eth10ge_R	0x0
#define itpm_cpld_regfile_eth10ge_D	"Missing description"

#define itpm_cpld_regfile_eth10ge_lock_M	0x00000030
#define itpm_cpld_regfile_eth10ge_lock_B	4
#define itpm_cpld_regfile_eth10ge_lock_R	0x00000000
#define itpm_cpld_regfile_eth10ge_lock_D	"PLL AD9550 Lock"

#define itpm_cpld_regfile_eth10ge_psnt_M	0x00000003
#define itpm_cpld_regfile_eth10ge_psnt_B	0
#define itpm_cpld_regfile_eth10ge_psnt_R	0x00000000
#define itpm_cpld_regfile_eth10ge_psnt_D	"10GE QSFP Present"

#define itpm_cpld_regfile_test_error  	0x30000040	//RW
#define itpm_cpld_regfile_test_error_M	0x00000030
#define itpm_cpld_regfile_test_error_R	0x00000000
#define itpm_cpld_regfile_test_error_D	"C2C Test Error"

#define itpm_cpld_regfile_ethled  	0x30000050	//R
#define itpm_cpld_regfile_ethled_M	0x00000007
#define itpm_cpld_regfile_ethled_R	0x00000000
#define itpm_cpld_regfile_ethled_D	"Etherned LED status"

#define itpm_cpld_regfile_xilinx  	0x30000060	//RW
#define itpm_cpld_regfile_xilinx_M	0xFFFFFFFF
#define itpm_cpld_regfile_xilinx_R	0x0
#define itpm_cpld_regfile_xilinx_D	"Missing description"

#define itpm_cpld_regfile_xilinx_program_M	0x00000030
#define itpm_cpld_regfile_xilinx_program_B	4
#define itpm_cpld_regfile_xilinx_program_R	0x00000003
#define itpm_cpld_regfile_xilinx_program_D	"Xilinx Program"

#define itpm_cpld_regfile_xilinx_reset_M	0x00000001
#define itpm_cpld_regfile_xilinx_reset_B	0
#define itpm_cpld_regfile_xilinx_reset_R	0x00000000
#define itpm_cpld_regfile_xilinx_reset_D	"Xilinx Reset"

#define itpm_cpld_regfile_xilinx_done_M	0x00003000
#define itpm_cpld_regfile_xilinx_done_B	12
#define itpm_cpld_regfile_xilinx_done_R	0x00000000
#define itpm_cpld_regfile_xilinx_done_D	"Xilinx Done"

#define itpm_cpld_regfile_xilinx_init_M	0x00000300
#define itpm_cpld_regfile_xilinx_init_B	8
#define itpm_cpld_regfile_xilinx_init_R	0x00000000
#define itpm_cpld_regfile_xilinx_init_D	"Xilinx Init"

#define itpm_cpld_regfile_amp  	0x30000070	//RW
#define itpm_cpld_regfile_amp_M	0xFFFFFFFF
#define itpm_cpld_regfile_amp_R	0x0
#define itpm_cpld_regfile_amp_D	"Missing description"

#define itpm_cpld_regfile_amp_pmode_M	0x00000002
#define itpm_cpld_regfile_amp_pmode_B	1
#define itpm_cpld_regfile_amp_pmode_R	0x00000000
#define itpm_cpld_regfile_amp_pmode_D	"AMP Power Mode"

#define itpm_cpld_regfile_amp_fa_M	0x00000001
#define itpm_cpld_regfile_amp_fa_B	0
#define itpm_cpld_regfile_amp_fa_R	0x00000000
#define itpm_cpld_regfile_amp_fa_D	"AMP Fast Attack"

#define itpm_cpld_regfile_enable  	0x30000080	//RW
#define itpm_cpld_regfile_enable_M	0xFFFFFFFF
#define itpm_cpld_regfile_enable_R	0x0
#define itpm_cpld_regfile_enable_D	"Missing description"

#define itpm_cpld_regfile_enable_fpga_M	0x00000004
#define itpm_cpld_regfile_enable_fpga_B	2
#define itpm_cpld_regfile_enable_fpga_R	0x00000000
#define itpm_cpld_regfile_enable_fpga_D	"Enable FPGA"

#define itpm_cpld_regfile_enable_sysr_M	0x00000008
#define itpm_cpld_regfile_enable_sysr_B	3
#define itpm_cpld_regfile_enable_sysr_R	0x00000000
#define itpm_cpld_regfile_enable_sysr_D	"Enable SYSR"

#define itpm_cpld_regfile_enable_vga_M	0x00000010
#define itpm_cpld_regfile_enable_vga_B	4
#define itpm_cpld_regfile_enable_vga_R	0x00000000
#define itpm_cpld_regfile_enable_vga_D	"Enable VGA"

#define itpm_cpld_regfile_enable_fe_M	0x00000002
#define itpm_cpld_regfile_enable_fe_B	1
#define itpm_cpld_regfile_enable_fe_R	0x00000000
#define itpm_cpld_regfile_enable_fe_D	"Enable Front End"

#define itpm_cpld_regfile_enable_adc_M	0x00000001
#define itpm_cpld_regfile_enable_adc_B	0
#define itpm_cpld_regfile_enable_adc_R	0x00000000
#define itpm_cpld_regfile_enable_adc_D	"Enable ADC"

#define itpm_cpld_regfile_enable_shadow  	0x30000084	//RW
#define itpm_cpld_regfile_enable_shadow_M	0xFFFFFFFF
#define itpm_cpld_regfile_enable_shadow_R	0x0
#define itpm_cpld_regfile_enable_shadow_D	"Missing description"

#define itpm_cpld_regfile_enable_shadow_fpga_M	0x00000004
#define itpm_cpld_regfile_enable_shadow_fpga_B	2
#define itpm_cpld_regfile_enable_shadow_fpga_R	0x00000000
#define itpm_cpld_regfile_enable_shadow_fpga_D	"Enable Shadow FPGA"

#define itpm_cpld_regfile_enable_shadow_sysr_M	0x00000008
#define itpm_cpld_regfile_enable_shadow_sysr_B	3
#define itpm_cpld_regfile_enable_shadow_sysr_R	0x00000000
#define itpm_cpld_regfile_enable_shadow_sysr_D	"Enable Shadow SYSR"

#define itpm_cpld_regfile_enable_shadow_adc_M	0x00000001
#define itpm_cpld_regfile_enable_shadow_adc_B	0
#define itpm_cpld_regfile_enable_shadow_adc_R	0x00000000
#define itpm_cpld_regfile_enable_shadow_adc_D	"Enable Shadow ADC"

#define itpm_cpld_regfile_enable_shadow_vga_M	0x00000010
#define itpm_cpld_regfile_enable_shadow_vga_B	4
#define itpm_cpld_regfile_enable_shadow_vga_R	0x00000000
#define itpm_cpld_regfile_enable_shadow_vga_D	"Enable Shadow VGA"

#define itpm_cpld_regfile_enable_shadow_fe_M	0x00000002
#define itpm_cpld_regfile_enable_shadow_fe_B	1
#define itpm_cpld_regfile_enable_shadow_fe_R	0x00000000
#define itpm_cpld_regfile_enable_shadow_fe_D	"Enable Shadow Front End"

#define itpm_cpld_regfile_ad_pwdn  	0x30000090	//RW
#define itpm_cpld_regfile_ad_pwdn_M	0x00000001
#define itpm_cpld_regfile_ad_pwdn_R	0x00000001
#define itpm_cpld_regfile_ad_pwdn_D	"AD Power Down"

#define itpm_cpld_regfile_axis_disable  	0x300000B4	//RW
#define itpm_cpld_regfile_axis_disable_M	0x00000030
#define itpm_cpld_regfile_axis_disable_R	0x00000000
#define itpm_cpld_regfile_axis_disable_D	"Disable Axis port"

#define itpm_cpld_regfile_frameidclr  	0x300000B8	//RW
#define itpm_cpld_regfile_frameidclr_M	0x00000100
#define itpm_cpld_regfile_frameidclr_R	0x00000000
#define itpm_cpld_regfile_frameidclr_D	"Frame ID Clear"

#define itpm_cpld_regfile_ena_header  	0x300000BC	//RW
#define itpm_cpld_regfile_ena_header_M	0x00001000
#define itpm_cpld_regfile_ena_header_R	0x00000000
#define itpm_cpld_regfile_ena_header_D	"Enable C2C Header"

#define itpm_cpld_regfile_header_ins  	0x300000C0	//RW
#define itpm_cpld_regfile_header_ins_M	0x00002000
#define itpm_cpld_regfile_header_ins_R	0x00000000
#define itpm_cpld_regfile_header_ins_D	"Header Insert"

#define itpm_cpld_regfile_ethernet_pause  	0x300000C4	//RW
#define itpm_cpld_regfile_ethernet_pause_M	0x0000ffff
#define itpm_cpld_regfile_ethernet_pause_R	0x00000000
#define itpm_cpld_regfile_ethernet_pause_D	"UDP ETH Pause"

#define itpm_cpld_regfile_stream_dst_ip0  	0x300000D0	//RW
#define itpm_cpld_regfile_stream_dst_ip0_M	0xffffffff
#define itpm_cpld_regfile_stream_dst_ip0_R	0x0A000A01
#define itpm_cpld_regfile_stream_dst_ip0_D	"Stream Destination IP 0"

#define itpm_cpld_regfile_stream_src_port0  	0x300000D4	//RW
#define itpm_cpld_regfile_stream_src_port0_M	0x0000ffff
#define itpm_cpld_regfile_stream_src_port0_R	0x00001234
#define itpm_cpld_regfile_stream_src_port0_D	"Stream Source Port 0"

#define itpm_cpld_regfile_stream_dst_port0  	0x300000D8	//RW
#define itpm_cpld_regfile_stream_dst_port0_M	0x0000ffff
#define itpm_cpld_regfile_stream_dst_port0_R	0x00001234
#define itpm_cpld_regfile_stream_dst_port0_D	"Stream Destination Port 0"

#define itpm_cpld_regfile_stream_dst_ip1  	0x300000DC	//RW
#define itpm_cpld_regfile_stream_dst_ip1_M	0xffffffff
#define itpm_cpld_regfile_stream_dst_ip1_R	0x00000000
#define itpm_cpld_regfile_stream_dst_ip1_D	"Stream Destination IP 1"

#define itpm_cpld_regfile_stream_src_port1  	0x300000E0	//RW
#define itpm_cpld_regfile_stream_src_port1_M	0x0000ffff
#define itpm_cpld_regfile_stream_src_port1_R	0x00000000
#define itpm_cpld_regfile_stream_src_port1_D	"Stream Source Port 1"

#define itpm_cpld_regfile_stream_dst_port1  	0x300000E4	//RW
#define itpm_cpld_regfile_stream_dst_port1_M	0x0000ffff
#define itpm_cpld_regfile_stream_dst_port1_R	0x00000000
#define itpm_cpld_regfile_stream_dst_port1_D	"Stream Destination Port 1"

#define itpm_cpld_regfile_sam_nrst  	0x30000120	//RW
#define itpm_cpld_regfile_sam_nrst_M	0x00000001
#define itpm_cpld_regfile_sam_nrst_R	0x00000001
#define itpm_cpld_regfile_sam_nrst_D	"CPU Reset"

#define itpm_cpld_regfile_bkplane_gpio  	0x30000130	//RW
#define itpm_cpld_regfile_bkplane_gpio_M	0x00000001
#define itpm_cpld_regfile_bkplane_gpio_R	0x00000000
#define itpm_cpld_regfile_bkplane_gpio_D	"Backplane GPIO"

#define itpm_cpld_regfile_spi_cs  	0x30000200	//RW
#define itpm_cpld_regfile_spi_cs_M	0xFFFFFFFF
#define itpm_cpld_regfile_spi_cs_R	0x0
#define itpm_cpld_regfile_spi_cs_D	"Missing description"

#define itpm_cpld_regfile_spi_cs_ow_M	0x00000001
#define itpm_cpld_regfile_spi_cs_ow_B	0
#define itpm_cpld_regfile_spi_cs_ow_R	0x00000001
#define itpm_cpld_regfile_spi_cs_ow_D	"SPI CS OW"

#define itpm_cpld_regfile_spi_cs_cs0_M	0x00010000
#define itpm_cpld_regfile_spi_cs_cs0_B	16
#define itpm_cpld_regfile_spi_cs_cs0_R	0x00000000
#define itpm_cpld_regfile_spi_cs_cs0_D	"SPI CS0"

#define itpm_cpld_regfile_spi_tx_byte  	0x30000204	//RW
#define itpm_cpld_regfile_spi_tx_byte_M	0x01ffffff
#define itpm_cpld_regfile_spi_tx_byte_R	0x00000000
#define itpm_cpld_regfile_spi_tx_byte_D	"SPI TX Byte"

#define itpm_cpld_regfile_spi_rx_byte  	0x30000208	//R
#define itpm_cpld_regfile_spi_rx_byte_M	0x01ffffff
#define itpm_cpld_regfile_spi_rx_byte_R	0x00000000
#define itpm_cpld_regfile_spi_rx_byte_D	"SPI RX Byte"

#define itpm_cpld_regfile_spi_tx_buf_len  	0x3000020C	//R
#define itpm_cpld_regfile_spi_tx_buf_len_M	0x01ffffff
#define itpm_cpld_regfile_spi_tx_buf_len_R	0x00000000
#define itpm_cpld_regfile_spi_tx_buf_len_D	"SPI TX Buffer length"

#define itpm_cpld_regfile_spi_rx_buf_len  	0x30000210	//R
#define itpm_cpld_regfile_spi_rx_buf_len_M	0x01ffffff
#define itpm_cpld_regfile_spi_rx_buf_len_R	0x00000000
#define itpm_cpld_regfile_spi_rx_buf_len_D	"SPI RX Buffer length"

#define itpm_cpld_regfile_spi_fifo_addr  	0x30000214	//RW
#define itpm_cpld_regfile_spi_fifo_addr_M	0x01ffffff
#define itpm_cpld_regfile_spi_fifo_addr_R	0x00000000
#define itpm_cpld_regfile_spi_fifo_addr_D	"SPI Fifo Address"

#define itpm_cpld_regfile_spi_mux  	0x30000218	//RW
#define itpm_cpld_regfile_spi_mux_M	0x00000003
#define itpm_cpld_regfile_spi_mux_R	0x00000000
#define itpm_cpld_regfile_spi_mux_D	"SPI Mux"

#define itpm_cpld_regfile_spi_route  	0x3000021C	//RW
#define itpm_cpld_regfile_spi_route_M	0x00000001
#define itpm_cpld_regfile_spi_route_R	0x00000000
#define itpm_cpld_regfile_spi_route_D	"SPI Routing to Slave Serial"

#define itpm_cpld_regfile_eth_mac_l  	0x30000300	//R
#define itpm_cpld_regfile_eth_mac_l_M	0xffffffff
#define itpm_cpld_regfile_eth_mac_l_R	0xbbccdd01
#define itpm_cpld_regfile_eth_mac_l_D	"Current MAC b31-b0"

#define itpm_cpld_regfile_eth_mac_h  	0x30000304	//R
#define itpm_cpld_regfile_eth_mac_h_M	0x0000ffff
#define itpm_cpld_regfile_eth_mac_h_R	0x000002aa
#define itpm_cpld_regfile_eth_mac_h_D	"Current MAC b47-b32"

#define itpm_cpld_regfile_eth_ip  	0x30000308	//R
#define itpm_cpld_regfile_eth_ip_M	0xffffffff
#define itpm_cpld_regfile_eth_ip_R	0x0a000a02
#define itpm_cpld_regfile_eth_ip_D	"Current IP"

#define itpm_cpld_regfile_eth_mask  	0x3000030C	//R
#define itpm_cpld_regfile_eth_mask_M	0xffffffff
#define itpm_cpld_regfile_eth_mask_R	0xffffff00
#define itpm_cpld_regfile_eth_mask_D	"Current Mask"

#define itpm_cpld_regfile_eth_gway  	0x30000310	//R
#define itpm_cpld_regfile_eth_gway_M	0xffffffff
#define itpm_cpld_regfile_eth_gway_R	0x0a000a01
#define itpm_cpld_regfile_eth_gway_D	"Current Gateway"

#define itpm_cpld_regfile_eth_key  	0x30000314	//R
#define itpm_cpld_regfile_eth_key_M	0x00ffffff
#define itpm_cpld_regfile_eth_key_R	0x00000000
#define itpm_cpld_regfile_eth_key_D	"Current Key"

#define itpm_cpld_regfile_key_ok  	0x30000318	//R
#define itpm_cpld_regfile_key_ok_M	0x00000001
#define itpm_cpld_regfile_key_ok_R	0x00000000
#define itpm_cpld_regfile_key_ok_D	"Key OK Status"

#define itpm_cpld_regfile_factory_rst  	0x3000031C	//R
#define itpm_cpld_regfile_factory_rst_M	0x00000001
#define itpm_cpld_regfile_factory_rst_R	0x00000000
#define itpm_cpld_regfile_factory_rst_D	"Factory Reset Jumper Status"

#define itpm_cpld_regfile_jtag0_length  	0x30000320	//RW
#define itpm_cpld_regfile_jtag0_length_M	0xffffffff
#define itpm_cpld_regfile_jtag0_length_R	0x00000000
#define itpm_cpld_regfile_jtag0_length_D	"JTAG Command length in bits"

#define itpm_cpld_regfile_jtag0_TMS  	0x30000324	//RW
#define itpm_cpld_regfile_jtag0_TMS_M	0xffffffff
#define itpm_cpld_regfile_jtag0_TMS_R	0x00000000
#define itpm_cpld_regfile_jtag0_TMS_D	"JTAG TMS vector"

#define itpm_cpld_regfile_jtag0_TDI  	0x30000328	//RW
#define itpm_cpld_regfile_jtag0_TDI_M	0xffffffff
#define itpm_cpld_regfile_jtag0_TDI_R	0x00000000
#define itpm_cpld_regfile_jtag0_TDI_D	"JTAG TDI vector"

#define itpm_cpld_regfile_jtag0_TDO  	0x3000032C	//R
#define itpm_cpld_regfile_jtag0_TDO_M	0xffffffff
#define itpm_cpld_regfile_jtag0_TDO_R	0x00000000
#define itpm_cpld_regfile_jtag0_TDO_D	"JTAG TDO vector (valid when CTRL is low)"

#define itpm_cpld_regfile_jtag0_CTRL  	0x30000330	//RW
#define itpm_cpld_regfile_jtag0_CTRL_M	0x00000001
#define itpm_cpld_regfile_jtag0_CTRL_R	0x00000000
#define itpm_cpld_regfile_jtag0_CTRL_D	"JTAG CTRL (write '1' to start JTAG command, self resets when finished)"

#define itpm_cpld_regfile_jtag1_length  	0x30000340	//RW
#define itpm_cpld_regfile_jtag1_length_M	0xffffffff
#define itpm_cpld_regfile_jtag1_length_R	0x00000000
#define itpm_cpld_regfile_jtag1_length_D	"JTAG Command length in bits"

#define itpm_cpld_regfile_jtag1_TMS  	0x30000344	//RW
#define itpm_cpld_regfile_jtag1_TMS_M	0xffffffff
#define itpm_cpld_regfile_jtag1_TMS_R	0x00000000
#define itpm_cpld_regfile_jtag1_TMS_D	"JTAG TMS vector"

#define itpm_cpld_regfile_jtag1_TDI  	0x30000348	//RW
#define itpm_cpld_regfile_jtag1_TDI_M	0xffffffff
#define itpm_cpld_regfile_jtag1_TDI_R	0x00000000
#define itpm_cpld_regfile_jtag1_TDI_D	"JTAG TDI vector"

#define itpm_cpld_regfile_jtag1_TDO  	0x3000034C	//R
#define itpm_cpld_regfile_jtag1_TDO_M	0xffffffff
#define itpm_cpld_regfile_jtag1_TDO_R	0x00000000
#define itpm_cpld_regfile_jtag1_TDO_D	"JTAG TDO vector (valid when CTRL is low)"

#define itpm_cpld_regfile_jtag1_CTRL  	0x30000350	//RW
#define itpm_cpld_regfile_jtag1_CTRL_M	0x00000001
#define itpm_cpld_regfile_jtag1_CTRL_R	0x00000000
#define itpm_cpld_regfile_jtag1_CTRL_D	"JTAG CTRL (write '1' to start JTAG command, self resets when finished)"

#define itpm_cpld_regfile_wdt_sem  	0x30000400	//RW
#define itpm_cpld_regfile_wdt_sem_M	0xFFFFFFFF
#define itpm_cpld_regfile_wdt_sem_R	0x0
#define itpm_cpld_regfile_wdt_sem_D	"Missing description"

#define itpm_cpld_regfile_wdt_sem_timer_M	0xffff0000
#define itpm_cpld_regfile_wdt_sem_timer_B	16
#define itpm_cpld_regfile_wdt_sem_timer_R	0x00000000
#define itpm_cpld_regfile_wdt_sem_timer_D	"SEM Watchdog Timer Value"

#define itpm_cpld_regfile_wdt_sem_status_M	0x00000030
#define itpm_cpld_regfile_wdt_sem_status_B	4
#define itpm_cpld_regfile_wdt_sem_status_R	0x00000000
#define itpm_cpld_regfile_wdt_sem_status_D	"SEM Watchdog Status"

#define itpm_cpld_regfile_wdt_sem_enable_M	0x00000003
#define itpm_cpld_regfile_wdt_sem_enable_B	0
#define itpm_cpld_regfile_wdt_sem_enable_R	0x00000000
#define itpm_cpld_regfile_wdt_sem_enable_D	"SEM Watchdog Enable"

#define itpm_cpld_regfile_wdt_mcu  	0x30000404	//RW
#define itpm_cpld_regfile_wdt_mcu_M	0xFFFFFFFF
#define itpm_cpld_regfile_wdt_mcu_R	0x0
#define itpm_cpld_regfile_wdt_mcu_D	"Missing description"

#define itpm_cpld_regfile_wdt_mcu_timer_M	0xffff0000
#define itpm_cpld_regfile_wdt_mcu_timer_B	16
#define itpm_cpld_regfile_wdt_mcu_timer_R	0x00000000
#define itpm_cpld_regfile_wdt_mcu_timer_D	"MCU Watchdog Timer Value"

#define itpm_cpld_regfile_wdt_mcu_status_M	0x00000010
#define itpm_cpld_regfile_wdt_mcu_status_B	4
#define itpm_cpld_regfile_wdt_mcu_status_R	0x00000000
#define itpm_cpld_regfile_wdt_mcu_status_D	"MCU Watchdog Status"

#define itpm_cpld_regfile_wdt_mcu_enable_M	0x00000001
#define itpm_cpld_regfile_wdt_mcu_enable_B	0
#define itpm_cpld_regfile_wdt_mcu_enable_R	0x00000000
#define itpm_cpld_regfile_wdt_mcu_enable_D	"MCU Watchdog Enable"

#define itpm_cpld_regfile_global_status  	0x30000500	//RW
#define itpm_cpld_regfile_global_status_M	0xFFFFFFFF
#define itpm_cpld_regfile_global_status_R	0x0
#define itpm_cpld_regfile_global_status_D	"Missing description"

#define itpm_cpld_regfile_global_status_SEM_M	0x00003000
#define itpm_cpld_regfile_global_status_SEM_B	12
#define itpm_cpld_regfile_global_status_SEM_R	0x00000000
#define itpm_cpld_regfile_global_status_SEM_D	"SEM Watchdog Status: 0x00=ok, 0x01=warning, 0x10=alarm"

#define itpm_cpld_regfile_global_status_voltage_M	0x00000c00
#define itpm_cpld_regfile_global_status_voltage_B	10
#define itpm_cpld_regfile_global_status_voltage_R	0x00000000
#define itpm_cpld_regfile_global_status_voltage_D	"Voltage Status: 0x00=ok, 0x01=warning, 0x10=alarm"

#define itpm_cpld_regfile_global_status_temperature_M	0x00000300
#define itpm_cpld_regfile_global_status_temperature_B	8
#define itpm_cpld_regfile_global_status_temperature_R	0x00000000
#define itpm_cpld_regfile_global_status_temperature_D	"Temperature Status: 0x00=ok, 0x01=warning, 0x10=alarm"

#define itpm_cpld_regfile_global_status_powerclass_M	0x00000003
#define itpm_cpld_regfile_global_status_powerclass_B	0
#define itpm_cpld_regfile_global_status_powerclass_R	0x00000000
#define itpm_cpld_regfile_global_status_powerclass_D	"Powerclass Status"

#define itpm_cpld_regfile_global_status_MCU_M	0x0000c000
#define itpm_cpld_regfile_global_status_MCU_B	14
#define itpm_cpld_regfile_global_status_MCU_R	0x00000000
#define itpm_cpld_regfile_global_status_MCU_D	"MCU Watchdog Status: 0x00=ok, 0x01=warning, 0x10=alarm"

#define itpm_cpld_regfile_global_status_ack  	0x30000504	//RW
#define itpm_cpld_regfile_global_status_ack_M	0xFFFFFFFF
#define itpm_cpld_regfile_global_status_ack_R	0x0
#define itpm_cpld_regfile_global_status_ack_D	"Missing description"

#define itpm_cpld_regfile_global_status_ack_powerclass_M	0x00000003
#define itpm_cpld_regfile_global_status_ack_powerclass_B	0
#define itpm_cpld_regfile_global_status_ack_powerclass_R	0x00000000
#define itpm_cpld_regfile_global_status_ack_powerclass_D	"Powerclass Status Acknowledge"

#define itpm_cpld_regfile_global_status_ack_temperature_M	0x00000300
#define itpm_cpld_regfile_global_status_ack_temperature_B	8
#define itpm_cpld_regfile_global_status_ack_temperature_R	0x00000000
#define itpm_cpld_regfile_global_status_ack_temperature_D	"Temperature Status Acknowledge"

#define itpm_cpld_regfile_global_status_ack_voltage_M	0x00000c00
#define itpm_cpld_regfile_global_status_ack_voltage_B	10
#define itpm_cpld_regfile_global_status_ack_voltage_R	0x00000000
#define itpm_cpld_regfile_global_status_ack_voltage_D	"Voltage Status Acknowledge"

#define itpm_cpld_regfile_global_status_ack_SEM_M	0x00003000
#define itpm_cpld_regfile_global_status_ack_SEM_B	12
#define itpm_cpld_regfile_global_status_ack_SEM_R	0x00000000
#define itpm_cpld_regfile_global_status_ack_SEM_D	"SEM Watchdog Status Acknowledge"

#define itpm_cpld_regfile_global_status_ack_MCU_M	0x0000c000
#define itpm_cpld_regfile_global_status_ack_MCU_B	14
#define itpm_cpld_regfile_global_status_ack_MCU_R	0x00000000
#define itpm_cpld_regfile_global_status_ack_MCU_D	"MCU Watchdog Status Acknowledge"

#define itpm_cpld_regfile_standalone  	0x30000508	//RW
#define itpm_cpld_regfile_standalone_M	0x00000001
#define itpm_cpld_regfile_standalone_R	0x00000001
#define itpm_cpld_regfile_standalone_D	"Standalone = 1 or Subrack = 0"

#define itpm_cpld_regfile_peripheralbusy  	0x3000050C	//RW
#define itpm_cpld_regfile_peripheralbusy_M	0x00000001
#define itpm_cpld_regfile_peripheralbusy_R	0x00000001
#define itpm_cpld_regfile_peripheralbusy_D	"FPGA1/FPGA2/ADC Busy Status"

#define itpm_cpld_regfile_peripheralbusy_ack  	0x30000510	//RW
#define itpm_cpld_regfile_peripheralbusy_ack_M	0x00000001
#define itpm_cpld_regfile_peripheralbusy_ack_R	0x00000000
#define itpm_cpld_regfile_peripheralbusy_ack_D	"FPGA1/FPGA2/ADC Busy Acknowledge"

#define itpm_cpld_regfile_sw_bram_update  	0x30000514	//RW
#define itpm_cpld_regfile_sw_bram_update_M	0x00000001
#define itpm_cpld_regfile_sw_bram_update_R	0x00000000
#define itpm_cpld_regfile_sw_bram_update_D	"BRAM Threshold updated"

#define itpm_cpld_regfile_safety_override  	0x30000518	//RW
#define itpm_cpld_regfile_safety_override_M	0x00000001
#define itpm_cpld_regfile_safety_override_R	0x00000001
#define itpm_cpld_regfile_safety_override_D	"Safety Override"

#define itpm_cpld_regfile_traceid_l  	0x30000600	//R
#define itpm_cpld_regfile_traceid_l_M	0xffffffff
#define itpm_cpld_regfile_traceid_l_R	0x00000000
#define itpm_cpld_regfile_traceid_l_D	"Trace ID Low Part"

#define itpm_cpld_regfile_traceid_h  	0x30000604	//R
#define itpm_cpld_regfile_traceid_h_M	0xffffffff
#define itpm_cpld_regfile_traceid_h_R	0x00000000
#define itpm_cpld_regfile_traceid_h_D	"Trace ID High Part"

#define itpm_cpld_i2c_command  	0x40000000	//RW
#define itpm_cpld_i2c_command_M	0xFFFFFFFF
#define itpm_cpld_i2c_command_R	0x0
#define itpm_cpld_i2c_command_D	"Missing description"

#define itpm_cpld_i2c_command_wrbyte_M	0x00000f00
#define itpm_cpld_i2c_command_wrbyte_B	8
#define itpm_cpld_i2c_command_wrbyte_R	0x0
#define itpm_cpld_i2c_command_wrbyte_D	"Number of byte to write"

#define itpm_cpld_i2c_command_phyadd_M	0x0000007f
#define itpm_cpld_i2c_command_phyadd_B	0
#define itpm_cpld_i2c_command_phyadd_R	0x0
#define itpm_cpld_i2c_command_phyadd_D	"Physical Address"

#define itpm_cpld_i2c_command_rdbyte_M	0x0000f000
#define itpm_cpld_i2c_command_rdbyte_B	12
#define itpm_cpld_i2c_command_rdbyte_R	0x0
#define itpm_cpld_i2c_command_rdbyte_D	"Number of byte to read"

#define itpm_cpld_i2c_command_mux_M	0x00030000
#define itpm_cpld_i2c_command_mux_B	16
#define itpm_cpld_i2c_command_mux_R	0x0
#define itpm_cpld_i2c_command_mux_D	"Output mux control"

#define itpm_cpld_i2c_transmit  	0x40000004	//RW
#define itpm_cpld_i2c_transmit_M	0xFFFFFFFF
#define itpm_cpld_i2c_transmit_R	0x0
#define itpm_cpld_i2c_transmit_D	"Data to be transmitted"

#define itpm_cpld_i2c_receive  	0x40000008	//R
#define itpm_cpld_i2c_receive_M	0xFFFFFFFF
#define itpm_cpld_i2c_receive_R	0x0
#define itpm_cpld_i2c_receive_D	"Data received"

#define itpm_cpld_i2c_status  	0x4000000C	//RW
#define itpm_cpld_i2c_status_M	0xFFFFFFFF
#define itpm_cpld_i2c_status_R	0x0
#define itpm_cpld_i2c_status_D	"Missing description"

#define itpm_cpld_i2c_status_ack_error_M	0x00000002
#define itpm_cpld_i2c_status_ack_error_B	1
#define itpm_cpld_i2c_status_ack_error_R	0x0
#define itpm_cpld_i2c_status_ack_error_D	"Acknowledge error"

#define itpm_cpld_i2c_status_busy_M	0x00000001
#define itpm_cpld_i2c_status_busy_B	0
#define itpm_cpld_i2c_status_busy_R	0x0
#define itpm_cpld_i2c_status_busy_D	"Busy"

#define itpm_cpld_i2c_irq  	0x40000010	//RW
#define itpm_cpld_i2c_irq_M	0xFFFFFFFF
#define itpm_cpld_i2c_irq_R	0x0
#define itpm_cpld_i2c_irq_D	"Missing description"

#define itpm_cpld_i2c_irq_ack_error_M	0x00000002
#define itpm_cpld_i2c_irq_ack_error_B	1
#define itpm_cpld_i2c_irq_ack_error_R	0x0
#define itpm_cpld_i2c_irq_ack_error_D	"IRQ Acknowledge error"

#define itpm_cpld_i2c_irq_done_M	0x00000001
#define itpm_cpld_i2c_irq_done_B	0
#define itpm_cpld_i2c_irq_done_R	0x0
#define itpm_cpld_i2c_irq_done_D	"IRQ Done"

#define itpm_cpld_i2c_irq_en  	0x40000014	//RW
#define itpm_cpld_i2c_irq_en_M	0x00000003
#define itpm_cpld_i2c_irq_en_R	0x0
#define itpm_cpld_i2c_irq_en_D	"IRQ Enables"

#define itpm_cpld_i2c_update_network_config  	0x40000018	//RW
#define itpm_cpld_i2c_update_network_config_M	0x00000001
#define itpm_cpld_i2c_update_network_config_R	0x0
#define itpm_cpld_i2c_update_network_config_D	"Update network config"

#define itpm_cpld_i2c_mac_hi  	0x40000020	//RW
#define itpm_cpld_i2c_mac_hi_M	0x0000ffff
#define itpm_cpld_i2c_mac_hi_R	0x0
#define itpm_cpld_i2c_mac_hi_D	"MAC high part"

#define itpm_cpld_i2c_mac_lo  	0x40000024	//RW
#define itpm_cpld_i2c_mac_lo_M	0xffffffff
#define itpm_cpld_i2c_mac_lo_R	0x0
#define itpm_cpld_i2c_mac_lo_D	"MAC low part"

#define itpm_cpld_i2c_ip  	0x40000028	//RW
#define itpm_cpld_i2c_ip_M	0xffffffff
#define itpm_cpld_i2c_ip_R	0x0
#define itpm_cpld_i2c_ip_D	"IP"

#define itpm_cpld_i2c_netmask  	0x4000002C	//RW
#define itpm_cpld_i2c_netmask_M	0xffffffff
#define itpm_cpld_i2c_netmask_R	0x0
#define itpm_cpld_i2c_netmask_D	"Netmask"

#define itpm_cpld_i2c_gateway  	0x40000030	//RW
#define itpm_cpld_i2c_gateway_M	0xffffffff
#define itpm_cpld_i2c_gateway_R	0x0
#define itpm_cpld_i2c_gateway_D	"Gateway"

#define itpm_cpld_i2c_key  	0x40000034	//R
#define itpm_cpld_i2c_key_M	0x00ffffff
#define itpm_cpld_i2c_key_R	0x0
#define itpm_cpld_i2c_key_D	"Key"

#define itpm_cpld_i2c_password_lo  	0x40000038	//RW
#define itpm_cpld_i2c_password_lo_M	0xffffffff
#define itpm_cpld_i2c_password_lo_R	0x0
#define itpm_cpld_i2c_password_lo_D	"Password low part"

#define itpm_cpld_i2c_password  	0x4000003C	//RW
#define itpm_cpld_i2c_password_M	0xFFFFFFFF
#define itpm_cpld_i2c_password_R	0x0
#define itpm_cpld_i2c_password_D	"Missing description"

#define itpm_cpld_i2c_password_ok_M	0x00010000
#define itpm_cpld_i2c_password_ok_B	16
#define itpm_cpld_i2c_password_ok_R	0x0
#define itpm_cpld_i2c_password_ok_D	"Password OK"

#define itpm_cpld_i2c_password_hi_M	0x0000ffff
#define itpm_cpld_i2c_password_hi_B	0
#define itpm_cpld_i2c_password_hi_R	0x0
#define itpm_cpld_i2c_password_hi_D	"Password high part"

#define itpm_cpld_smap_global  	0x50000000	//RW
#define itpm_cpld_smap_global_M	0xFFFFFFFF
#define itpm_cpld_smap_global_R	0x0
#define itpm_cpld_smap_global_D	"Missing Description"

#define itpm_cpld_smap_global_rdfifo_empty_M	0x00000020
#define itpm_cpld_smap_global_rdfifo_empty_B	5
#define itpm_cpld_smap_global_rdfifo_empty_R	0x0
#define itpm_cpld_smap_global_rdfifo_empty_D	"rdfifo empty flag (unused)"

#define itpm_cpld_smap_global_program_M	0x00000002
#define itpm_cpld_smap_global_program_B	1
#define itpm_cpld_smap_global_program_R	0x0
#define itpm_cpld_smap_global_program_D	"both FPGA program pin"

#define itpm_cpld_smap_global_wrfifo_empty_M	0x00000010
#define itpm_cpld_smap_global_wrfifo_empty_B	4
#define itpm_cpld_smap_global_wrfifo_empty_R	0x0
#define itpm_cpld_smap_global_wrfifo_empty_D	"wrfifo empty flag"

#define itpm_cpld_smap_global_cs_M	0x00000001
#define itpm_cpld_smap_global_cs_B	0
#define itpm_cpld_smap_global_cs_R	0x0
#define itpm_cpld_smap_global_cs_D	"chip select (unused)"

#define itpm_cpld_smap_xil_0  	0x50000004	//RW
#define itpm_cpld_smap_xil_0_M	0xFFFFFFFF
#define itpm_cpld_smap_xil_0_R	0x0
#define itpm_cpld_smap_xil_0_D	"Missing Description"

#define itpm_cpld_smap_xil_0_done_M	0x00000002
#define itpm_cpld_smap_xil_0_done_B	1
#define itpm_cpld_smap_xil_0_done_R	0x0
#define itpm_cpld_smap_xil_0_done_D	"done pin FPGA1"

#define itpm_cpld_smap_xil_0_init_M	0x00000001
#define itpm_cpld_smap_xil_0_init_B	0
#define itpm_cpld_smap_xil_0_init_R	0x0
#define itpm_cpld_smap_xil_0_init_D	"init pin FPGA1"

#define itpm_cpld_smap_xil_0_sel_M	0x00000010
#define itpm_cpld_smap_xil_0_sel_B	4
#define itpm_cpld_smap_xil_0_sel_R	0x0
#define itpm_cpld_smap_xil_0_sel_D	"select pin FPGA1"

#define itpm_cpld_smap_xil_1  	0x50000008	//RW
#define itpm_cpld_smap_xil_1_M	0xFFFFFFFF
#define itpm_cpld_smap_xil_1_R	0x0
#define itpm_cpld_smap_xil_1_D	"Missing Description"

#define itpm_cpld_smap_xil_1_sel_M	0x00000010
#define itpm_cpld_smap_xil_1_sel_B	4
#define itpm_cpld_smap_xil_1_sel_R	0x0
#define itpm_cpld_smap_xil_1_sel_D	"select pin FPGA2"

#define itpm_cpld_smap_xil_1_init_M	0x00000001
#define itpm_cpld_smap_xil_1_init_B	0
#define itpm_cpld_smap_xil_1_init_R	0x0
#define itpm_cpld_smap_xil_1_init_D	"init pin FPGA2"

#define itpm_cpld_smap_xil_1_done_M	0x00000002
#define itpm_cpld_smap_xil_1_done_B	1
#define itpm_cpld_smap_xil_1_done_R	0x0
#define itpm_cpld_smap_xil_1_done_D	"done pin FPGA2"

#define itpm_cpld_smap_wr_fifo  	0x50001000	//RW
#define itpm_cpld_smap_wr_fifo_M	0xFFFFFFFF
#define itpm_cpld_smap_wr_fifo_R	0x0
#define itpm_cpld_smap_wr_fifo_D	"data to be written in Slave Serial IF"


#define itpm_cpld_lock_queue_number  	0x60000000	//R
#define itpm_cpld_lock_queue_number_M	0x0000FFFF
#define itpm_cpld_lock_queue_number_R	0x0
#define itpm_cpld_lock_queue_number_D	"Queue number"

#define itpm_cpld_lock_lock_anaspi  	0x60000004	//RW
#define itpm_cpld_lock_lock_anaspi_M	0x0000FFFF
#define itpm_cpld_lock_lock_anaspi_R	0x0
#define itpm_cpld_lock_lock_anaspi_D	"Lock Analog SPI"

#define itpm_cpld_lock_lock_i2c  	0x60000008	//RW
#define itpm_cpld_lock_lock_i2c_M	0x0000FFFF
#define itpm_cpld_lock_lock_i2c_R	0x0
#define itpm_cpld_lock_lock_i2c_D	"Lock I2C"

#define itpm_cpld_lock_lock_smap  	0x6000000C	//RW
#define itpm_cpld_lock_lock_smap_M	0x0000FFFF
#define itpm_cpld_lock_lock_smap_R	0x0
#define itpm_cpld_lock_lock_smap_D	"Lock Select MAP"

#define itpm_cpld_lock_lock_fespi  	0x60000010	//RW
#define itpm_cpld_lock_lock_fespi_M	0x0000FFFF
#define itpm_cpld_lock_lock_fespi_R	0x0
#define itpm_cpld_lock_lock_fespi_D	"Lock Front End SPI"

#define itpm_cpld_lock_lock_confspi  	0x60000014	//RW
#define itpm_cpld_lock_lock_confspi_M	0x0000FFFF
#define itpm_cpld_lock_lock_confspi_R	0x0
#define itpm_cpld_lock_lock_confspi_D	"Lock Config SPI"

/*
#define itpm_cpld_lock_mlock0  	0x60000000	//RW
#define itpm_cpld_lock_mlock0_M	0xffffffff
#define itpm_cpld_lock_mlock0_R	0xffffffff
#define itpm_cpld_lock_mlock0_D	"MCU slave lock 0"

#define itpm_cpld_lock_mlock1  	0x60000004	//RW
#define itpm_cpld_lock_mlock1_M	0xffffffff
#define itpm_cpld_lock_mlock1_R	0xffffffff
#define itpm_cpld_lock_mlock1_D	"UCP slave lock 1"

#define itpm_cpld_lock_mlock2  	0x60000008	//RW
#define itpm_cpld_lock_mlock2_M	0xffffffff
#define itpm_cpld_lock_mlock2_R	0xffffffff
#define itpm_cpld_lock_mlock2_D	"SWLINK slave lock 2"

*/

#define itpm_cpld_fespi_serio_reg0  	0x70000000	//R
#define itpm_cpld_fespi_serio_reg0_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg0_R	0x0
#define itpm_cpld_fespi_serio_reg0_D	"Serial Data bits 31-0"

#define itpm_cpld_fespi_serio_reg1  	0x70000004	//R
#define itpm_cpld_fespi_serio_reg1_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg1_R	0x0
#define itpm_cpld_fespi_serio_reg1_D	"Serial Data bits 63-32"

#define itpm_cpld_fespi_serio_reg2  	0x70000008	//R
#define itpm_cpld_fespi_serio_reg2_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg2_R	0x0
#define itpm_cpld_fespi_serio_reg2_D	"Serial Data bits 95-64"

#define itpm_cpld_fespi_serio_reg3  	0x7000000C	//R
#define itpm_cpld_fespi_serio_reg3_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg3_R	0x0
#define itpm_cpld_fespi_serio_reg3_D	"Serial Data bits 127-96"

#define itpm_cpld_fespi_command  	0x70000010	//RW
#define itpm_cpld_fespi_command_M	0xFFFFFFFF
#define itpm_cpld_fespi_command_R	0x0
#define itpm_cpld_fespi_command_D	"Serial Command and Mux"

#define itpm_cpld_fespi_command_rw_M	0x00000003
#define itpm_cpld_fespi_command_rw_B	0
#define itpm_cpld_fespi_command_rw_R	0x0
#define itpm_cpld_fespi_command_rw_D	"0x1 = Read - 0x3 = Write"

#define itpm_cpld_fespi_command_mux_M	0x00000004
#define itpm_cpld_fespi_command_mux_B	2
#define itpm_cpld_fespi_command_mux_R	0x0
#define itpm_cpld_fespi_command_mux_D	"0x1 = Send to FE1- 0x0 = Send To FE0"

#define itpm_cpld_info_cpld_magic  	0x80000000	//R
#define itpm_cpld_info_cpld_magic_M	0xFFFFFFFF
#define itpm_cpld_info_cpld_magic_R	0x0
#define itpm_cpld_info_cpld_magic_D	"Magic Number 0xF1233215"

#define itpm_cpld_info_cpld_xml_offset  	0x80000004	//R
#define itpm_cpld_info_cpld_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_cpld_xml_offset_R	0x0
#define itpm_cpld_info_cpld_xml_offset_D	"BRAM XML file offset"

#define itpm_cpld_info_spi_xml_offset  	0x80000008	//R
#define itpm_cpld_info_spi_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_spi_xml_offset_R	0x0
#define itpm_cpld_info_spi_xml_offset_D	"BRAM SPI file offset"

#define itpm_cpld_info_fpga1_base_add  	0x8000000C	//R
#define itpm_cpld_info_fpga1_base_add_M	0xFFFFFFFF
#define itpm_cpld_info_fpga1_base_add_R	0x0
#define itpm_cpld_info_fpga1_base_add_D	"FPGA1 base address"

#define itpm_cpld_info_fpga2_base_add  	0x80000010	//R
#define itpm_cpld_info_fpga2_base_add_M	0xFFFFFFFF
#define itpm_cpld_info_fpga2_base_add_R	0x0
#define itpm_cpld_info_fpga2_base_add_D	"FPGA2 base address"

#define itpm_cpld_info_fw1_xml_offset  	0x80000014	//R
#define itpm_cpld_info_fw1_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_xml_offset_R	0x0
#define itpm_cpld_info_fw1_xml_offset_D	"Offset of Firmware1 register map xml file"

#define itpm_cpld_info_fw2_xml_offset  	0x80000018	//R
#define itpm_cpld_info_fw2_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_xml_offset_R	0x0
#define itpm_cpld_info_fw2_xml_offset_D	"Offset of Firmware2 register map xml file (unused)"

#define itpm_cpld_info_fw3_xml_offset  	0x8000001C	//R
#define itpm_cpld_info_fw3_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_xml_offset_R	0x0
#define itpm_cpld_info_fw3_xml_offset_D	"Offset of Firmware3 register map xml file (unused)"

#define itpm_cpld_info_fw1_extended_info_offset  	0x80000020	//R
#define itpm_cpld_info_fw1_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_extended_info_offset_R	0x0
#define itpm_cpld_info_fw1_extended_info_offset_D	"FW1 extended information offset"

#define itpm_cpld_info_fw2_extended_info_offset  	0x80000024	//R
#define itpm_cpld_info_fw2_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_extended_info_offset_R	0x0
#define itpm_cpld_info_fw2_extended_info_offset_D	"FW2 extended information offset(unused)"

#define itpm_cpld_info_fw3_extended_info_offset  	0x80000028	//R
#define itpm_cpld_info_fw3_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_extended_info_offset_R	0x0
#define itpm_cpld_info_fw3_extended_info_offset_D	"FW3 extended information offset(unused)"

#define itpm_cpld_info_fw1_offset  	0x8000002C	//R
#define itpm_cpld_info_fw1_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_offset_R	0x0
#define itpm_cpld_info_fw1_offset_D	"FW1 bitstream offset"

#define itpm_cpld_info_fw2_offset  	0x80000030	//R
#define itpm_cpld_info_fw2_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_offset_R	0x0
#define itpm_cpld_info_fw2_offset_D	"FW2 bitstream offset(unused)"

#define itpm_cpld_info_fw3_offset  	0x80000034	//R
#define itpm_cpld_info_fw3_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_offset_R	0x0
#define itpm_cpld_info_fw3_offset_D	"FW3 bitstream offset(unused)"

#define itpm_cpld_bram_cpu  	0x90000000	//RW
#define itpm_cpld_bram_cpu_M	0xFFFFFFFF
#define itpm_cpld_bram_cpu_R	0x0
#define itpm_cpld_bram_cpu_D	"CPU BRAM"

#define itpm_cpld_confspi_rxtx_buffer  	0xA0000000	//RW
#define itpm_cpld_confspi_rxtx_buffer_M	0xFFFFFFFF
#define itpm_cpld_confspi_rxtx_buffer_R	0x0
#define itpm_cpld_confspi_rxtx_buffer_D	"TX-RX Data Buffer"

#define itpm_cpld_uart_rnw  	0xB0000000	//RW
#define itpm_cpld_uart_rnw_M	0x00000001
#define itpm_cpld_uart_rnw_R	0x0
#define itpm_cpld_uart_rnw_D	"Read Not Write Command Execute"

#define itpm_cpld_uart_txdata  	0xB0000004	//RW
#define itpm_cpld_uart_txdata_M	0x000000ff
#define itpm_cpld_uart_txdata_R	0x0
#define itpm_cpld_uart_txdata_D	"TX Data"

#define itpm_cpld_uart_rxdata  	0xB0000008	//R
#define itpm_cpld_uart_rxdata_M	0x000000ff
#define itpm_cpld_uart_rxdata_R	0x0
#define itpm_cpld_uart_rxdata_D	"RX Data"

#define itpm_cpld_uart_status  	0xB000000C	//R
#define itpm_cpld_uart_status_M	0x00000003
#define itpm_cpld_uart_status_R	0x0
#define itpm_cpld_uart_status_D	"Status"

#define itpm_cpld_intc_status  	0xC0000000	//R
#define itpm_cpld_intc_status_M	0x00000fff
#define itpm_cpld_intc_status_R	0x00000000
#define itpm_cpld_intc_status_D	"Interrupt Status"

#define itpm_cpld_intc_mask  	0xC0000004	//RW
#define itpm_cpld_intc_mask_M	0x00000fff
#define itpm_cpld_intc_mask_R	0x00000fff
#define itpm_cpld_intc_mask_D	"Interrupt Mask"

#define itpm_cpld_intc_ack  	0xC0000008	//RW
#define itpm_cpld_intc_ack_M	0x00000fff
#define itpm_cpld_intc_ack_R	0x00000000
#define itpm_cpld_intc_ack_D	"Interrupt Acknowledge"

#define itpm_cpld_bram_xml  	0xD0000000	//RW
#define itpm_cpld_bram_xml_M	0xFFFFFFFF
#define itpm_cpld_bram_xml_R	0x0
#define itpm_cpld_bram_xml_D	"CPU XML File"

#define itpm_cpld_bram_spi  	0xE0000000	//RW
#define itpm_cpld_bram_spi_M	0xFFFFFFFF
#define itpm_cpld_bram_spi_R	0x0
#define itpm_cpld_bram_spi_D	"CPU SPI File"
