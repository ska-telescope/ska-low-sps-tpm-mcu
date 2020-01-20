#define itpm_cpld_spi_address  	0x20000000	//RW
#define itpm_cpld_spi_address_M	0x0000FFFF
#define itpm_cpld_spi_address_R	0x0
#define itpm_cpld_spi_address_D	"Missing Description"

#define itpm_cpld_spi_write_data  	0x20000004	//RW
#define itpm_cpld_spi_write_data_M	0x0000FFFF
#define itpm_cpld_spi_write_data_R	0x0
#define itpm_cpld_spi_write_data_D	"Missing Description"

#define itpm_cpld_spi_read_data  	0x20000008	//RW
#define itpm_cpld_spi_read_data_M	0x0000FFFF
#define itpm_cpld_spi_read_data_R	0x0
#define itpm_cpld_spi_read_data_D	"Missing Description"

#define itpm_cpld_spi_chip_select  	0x2000000C	//RW
#define itpm_cpld_spi_chip_select_M	0x0000FFFF
#define itpm_cpld_spi_chip_select_R	0x0
#define itpm_cpld_spi_chip_select_D	"Missing Description"

#define itpm_cpld_spi_sclk  	0x20000010	//RW
#define itpm_cpld_spi_sclk_M	0x0000FFFF
#define itpm_cpld_spi_sclk_R	0x0
#define itpm_cpld_spi_sclk_D	"Missing Description"

#define itpm_cpld_spi_cmd  	0x20000014	//RW
#define itpm_cpld_spi_cmd_M	0xFFFFFFFF
#define itpm_cpld_spi_cmd_R	0x0
#define itpm_cpld_spi_cmd_D	"Missing Description"

#define itpm_cpld_spi_cmd_start  	0x20000014	//RW
#define itpm_cpld_spi_cmd_start_M	0x00000001
#define itpm_cpld_spi_cmd_start_R	0x0
#define itpm_cpld_spi_cmd_start_D	"Missing Description"

#define itpm_cpld_spi_cmd_rnw  	0x20000014	//RW
#define itpm_cpld_spi_cmd_rnw_M	0x00000002
#define itpm_cpld_spi_cmd_rnw_R	0x0
#define itpm_cpld_spi_cmd_rnw_D	"Missing Description"

#define itpm_cpld_regfile_date_code  	0x30000000	//R
#define itpm_cpld_regfile_date_code_M	0xFFFFFFFF
#define itpm_cpld_regfile_date_code_R	0x00000000
#define itpm_cpld_regfile_date_code_D	"Compile date"

#define itpm_cpld_regfile_master_rst  	0x30000008	//RW
#define itpm_cpld_regfile_master_rst_M	0x00008000
#define itpm_cpld_regfile_master_rst_R	0x00000000
#define itpm_cpld_regfile_master_rst_D	"Global XO3 Reset"

#define itpm_cpld_regfile_ena_stream  	0x30000018	//RW
#define itpm_cpld_regfile_ena_stream_M	0x00000001
#define itpm_cpld_regfile_ena_stream_R	0x00000000
#define itpm_cpld_regfile_ena_stream_D	"Enable C2C Stream"

#define itpm_cpld_regfile_pll  	0x30000020	//RW
#define itpm_cpld_regfile_pll_M	0xFFFFFFFF
#define itpm_cpld_regfile_pll_R	0x0
#define itpm_cpld_regfile_pll_D	"Missing description"

#define itpm_cpld_regfile_pll_resetn  	0x30000020	//RW
#define itpm_cpld_regfile_pll_resetn_M	0x00000001
#define itpm_cpld_regfile_pll_resetn_R	0x00000000
#define itpm_cpld_regfile_pll_resetn_D	"PLL AD9528 and AD9550 Reset"

#define itpm_cpld_regfile_pll_status  	0x30000020	//RW
#define itpm_cpld_regfile_pll_status_M	0x00000030
#define itpm_cpld_regfile_pll_status_R	0x00000000
#define itpm_cpld_regfile_pll_status_D	"PLL AD9528 Status"

#define itpm_cpld_regfile_c2c_pll  	0x30000028	//RW
#define itpm_cpld_regfile_c2c_pll_M	0xFFFFFFFF
#define itpm_cpld_regfile_c2c_pll_R	0x0
#define itpm_cpld_regfile_c2c_pll_D	"Missing description"

#define itpm_cpld_regfile_c2c_pll_phasedir  	0x30000028	//RW
#define itpm_cpld_regfile_c2c_pll_phasedir_M	0x00000010
#define itpm_cpld_regfile_c2c_pll_phasedir_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasedir_D	"c2c PLL Phase Direction"

#define itpm_cpld_regfile_c2c_pll_phasesel  	0x30000028	//RW
#define itpm_cpld_regfile_c2c_pll_phasesel_M	0x00000300
#define itpm_cpld_regfile_c2c_pll_phasesel_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasesel_D	"c2c PLL Phase Select"

#define itpm_cpld_regfile_c2c_pll_phasestep  	0x30000028	//RW
#define itpm_cpld_regfile_c2c_pll_phasestep_M	0x00000001
#define itpm_cpld_regfile_c2c_pll_phasestep_R	0x00000000
#define itpm_cpld_regfile_c2c_pll_phasestep_D	"c2c PLL Phase Step"

#define itpm_cpld_regfile_c2c_ctrl  	0x3000002C	//RW
#define itpm_cpld_regfile_c2c_ctrl_M	0xFFFFFFFF
#define itpm_cpld_regfile_c2c_ctrl_R	0x0
#define itpm_cpld_regfile_c2c_ctrl_D	"Missing description"

#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable  	0x3000002C	//RW
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_M	0x00000002
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_R	0x00000000
#define itpm_cpld_regfile_c2c_ctrl_mm_burst_enable_D	"Enable C2C Burst"

#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream  	0x3000002C	//RW
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_M	0x00000001
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_R	0x00000000
#define itpm_cpld_regfile_c2c_ctrl_mm_read_stream_D	"Enable C2C Management"

#define itpm_cpld_regfile_eth10ge  	0x30000030	//RW
#define itpm_cpld_regfile_eth10ge_M	0xFFFFFFFF
#define itpm_cpld_regfile_eth10ge_R	0x0
#define itpm_cpld_regfile_eth10ge_D	"Missing description"

#define itpm_cpld_regfile_eth10ge_psnt  	0x30000030	//RW
#define itpm_cpld_regfile_eth10ge_psnt_M	0x00000003
#define itpm_cpld_regfile_eth10ge_psnt_R	0x00000000
#define itpm_cpld_regfile_eth10ge_psnt_D	"10GE QSFP Present"

#define itpm_cpld_regfile_eth10ge_lock  	0x30000030	//RW
#define itpm_cpld_regfile_eth10ge_lock_M	0x00000030
#define itpm_cpld_regfile_eth10ge_lock_R	0x00000000
#define itpm_cpld_regfile_eth10ge_lock_D	"PLL AD9550 Lock"

#define itpm_cpld_regfile_test_error  	0x30000040	//RW
#define itpm_cpld_regfile_test_error_M	0x00000030
#define itpm_cpld_regfile_test_error_R	0x00000000
#define itpm_cpld_regfile_test_error_D	"C2C Test Error"

#define itpm_cpld_regfile_ucplastpsn  	0x30000044	//RW
#define itpm_cpld_regfile_ucplastpsn_M	0xffffffff
#define itpm_cpld_regfile_ucplastpsn_R	0x00000000
#define itpm_cpld_regfile_ucplastpsn_D	"UCP Last PSN"

#define itpm_cpld_regfile_ethled  	0x30000050	//RW
#define itpm_cpld_regfile_ethled_M	0x00000007
#define itpm_cpld_regfile_ethled_R	0x00000000
#define itpm_cpld_regfile_ethled_D	"Etherned LED status"

#define itpm_cpld_regfile_xilinx  	0x30000060	//RW
#define itpm_cpld_regfile_xilinx_M	0xFFFFFFFF
#define itpm_cpld_regfile_xilinx_R	0x0
#define itpm_cpld_regfile_xilinx_D	"Missing description"

#define itpm_cpld_regfile_xilinx_init  	0x30000060	//R
#define itpm_cpld_regfile_xilinx_init_M	0x00000300
#define itpm_cpld_regfile_xilinx_init_R	0x00000000
#define itpm_cpld_regfile_xilinx_init_D	"Xilinx Init"

#define itpm_cpld_regfile_xilinx_program  	0x30000060	//RW
#define itpm_cpld_regfile_xilinx_program_M	0x00000030
#define itpm_cpld_regfile_xilinx_program_R	0x00000003
#define itpm_cpld_regfile_xilinx_program_D	"Xilinx Program"

#define itpm_cpld_regfile_xilinx_reset  	0x30000060	//RW
#define itpm_cpld_regfile_xilinx_reset_M	0x00000001
#define itpm_cpld_regfile_xilinx_reset_R	0x00000000
#define itpm_cpld_regfile_xilinx_reset_D	"Xilinx Reset"

#define itpm_cpld_regfile_xilinx_done  	0x30000060	//R
#define itpm_cpld_regfile_xilinx_done_M	0x00003000
#define itpm_cpld_regfile_xilinx_done_R	0x00000000
#define itpm_cpld_regfile_xilinx_done_D	"Xilinx Done"

#define itpm_cpld_regfile_amp  	0x30000070	//RW
#define itpm_cpld_regfile_amp_M	0xFFFFFFFF
#define itpm_cpld_regfile_amp_R	0x0
#define itpm_cpld_regfile_amp_D	"Missing description"

#define itpm_cpld_regfile_amp_fa  	0x30000070	//RW
#define itpm_cpld_regfile_amp_fa_M	0x00000001
#define itpm_cpld_regfile_amp_fa_R	0x00000000
#define itpm_cpld_regfile_amp_fa_D	"AMP Fast Attack"

#define itpm_cpld_regfile_amp_pmode  	0x30000070	//RW
#define itpm_cpld_regfile_amp_pmode_M	0x00000002
#define itpm_cpld_regfile_amp_pmode_R	0x00000000
#define itpm_cpld_regfile_amp_pmode_D	"AMP Power Mode"

#define itpm_cpld_regfile_enable  	0x30000080	//RW
#define itpm_cpld_regfile_enable_M	0xFFFFFFFF
#define itpm_cpld_regfile_enable_R	0x0
#define itpm_cpld_regfile_enable_D	"Missing description"

#define itpm_cpld_regfile_enable_fpga  	0x30000080	//RW
#define itpm_cpld_regfile_enable_fpga_M	0x00000004
#define itpm_cpld_regfile_enable_fpga_R	0x00000000
#define itpm_cpld_regfile_enable_fpga_D	"Enable FPGA"

#define itpm_cpld_regfile_enable_adc  	0x30000080	//RW
#define itpm_cpld_regfile_enable_adc_M	0x00000001
#define itpm_cpld_regfile_enable_adc_R	0x00000000
#define itpm_cpld_regfile_enable_adc_D	"Enable ADC"

#define itpm_cpld_regfile_enable_fe  	0x30000080	//RW
#define itpm_cpld_regfile_enable_fe_M	0x00000002
#define itpm_cpld_regfile_enable_fe_R	0x00000000
#define itpm_cpld_regfile_enable_fe_D	"Enable Front End"

#define itpm_cpld_regfile_enable_vga  	0x30000080	//RW
#define itpm_cpld_regfile_enable_vga_M	0x00000010
#define itpm_cpld_regfile_enable_vga_R	0x00000000
#define itpm_cpld_regfile_enable_vga_D	"Enable VGA"

#define itpm_cpld_regfile_enable_sysr  	0x30000080	//RW
#define itpm_cpld_regfile_enable_sysr_M	0x00000008
#define itpm_cpld_regfile_enable_sysr_R	0x00000000
#define itpm_cpld_regfile_enable_sysr_D	"Enable SYSR"

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

#define itpm_cpld_regfile_tmp_present  	0x30000140	//RW
#define itpm_cpld_regfile_tmp_present_M	0x00000001
#define itpm_cpld_regfile_tmp_present_R	0x00000000
#define itpm_cpld_regfile_tmp_present_D	"TMP Present"

#define itpm_cpld_regfile_spi_cs_ow  	0x30000200	//RW
#define itpm_cpld_regfile_spi_cs_ow_M	0x00000001
#define itpm_cpld_regfile_spi_cs_ow_R	0x00000000
#define itpm_cpld_regfile_spi_cs_ow_D	"SPI CS OW"

#define itpm_cpld_regfile_spi_cs0  	0x30000204	//RW
#define itpm_cpld_regfile_spi_cs0_M	0x00000001
#define itpm_cpld_regfile_spi_cs0_R	0x00000000
#define itpm_cpld_regfile_spi_cs0_D	"SPI CS"

#define itpm_cpld_regfile_spi_tx_byte  	0x30000208	//RW
#define itpm_cpld_regfile_spi_tx_byte_M	0x000007ff
#define itpm_cpld_regfile_spi_tx_byte_R	0x00000000
#define itpm_cpld_regfile_spi_tx_byte_D	"SPI TX Byte"

#define itpm_cpld_regfile_spi_rx_byte  	0x3000020C	//R
#define itpm_cpld_regfile_spi_rx_byte_M	0x000007ff
#define itpm_cpld_regfile_spi_rx_byte_R	0x00000000
#define itpm_cpld_regfile_spi_rx_byte_D	"SPI RX Byte"

#define itpm_cpld_regfile_spi_tx_buf_len  	0x30000210	//R
#define itpm_cpld_regfile_spi_tx_buf_len_M	0x0000ffff
#define itpm_cpld_regfile_spi_tx_buf_len_R	0x00000000
#define itpm_cpld_regfile_spi_tx_buf_len_D	"SPI TX Buffer length"

#define itpm_cpld_regfile_spi_rx_buf_len  	0x30000214	//R
#define itpm_cpld_regfile_spi_rx_buf_len_M	0x0000ffff
#define itpm_cpld_regfile_spi_rx_buf_len_R	0x00000000
#define itpm_cpld_regfile_spi_rx_buf_len_D	"SPI RX Buffer length"

#define itpm_cpld_regfile_spi_fifo_addr  	0x30000218	//RW
#define itpm_cpld_regfile_spi_fifo_addr_M	0x000007ff
#define itpm_cpld_regfile_spi_fifo_addr_R	0x00000000
#define itpm_cpld_regfile_spi_fifo_addr_D	"SPI Fifo Address"

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

#define itpm_cpld_i2c_command  	0x40000000	//RW
#define itpm_cpld_i2c_command_M	0xFFFFFFFF
#define itpm_cpld_i2c_command_R	0x0
#define itpm_cpld_i2c_command_D	"Missing Description"

#define itpm_cpld_i2c_transmit  	0x40000004	//RW
#define itpm_cpld_i2c_transmit_M	0xFFFFFFFF
#define itpm_cpld_i2c_transmit_R	0x0
#define itpm_cpld_i2c_transmit_D	"Missing Description"

#define itpm_cpld_i2c_receive  	0x40000008	//RW
#define itpm_cpld_i2c_receive_M	0xFFFFFFFF
#define itpm_cpld_i2c_receive_R	0x0
#define itpm_cpld_i2c_receive_D	"Missing Description"

#define itpm_cpld_i2c_status  	0x4000000C	//RW
#define itpm_cpld_i2c_status_M	0x00000002
#define itpm_cpld_i2c_status_R	0x0
#define itpm_cpld_i2c_status_D	"Missing Description"

#define itpm_cpld_i2c_update_network_config  	0x40000018	//RW
#define itpm_cpld_i2c_update_network_config_M	0x00000001
#define itpm_cpld_i2c_update_network_config_R	0x0
#define itpm_cpld_i2c_update_network_config_D	"Missing Description"

#define itpm_cpld_smap_global  	0x50000000	//RW
#define itpm_cpld_smap_global_M	0xFFFFFFFF
#define itpm_cpld_smap_global_R	0x0
#define itpm_cpld_smap_global_D	"Missing Description"

#define itpm_cpld_smap_xil_0  	0x50000004	//RW
#define itpm_cpld_smap_xil_0_M	0xFFFFFFFF
#define itpm_cpld_smap_xil_0_R	0x0
#define itpm_cpld_smap_xil_0_D	"Missing Description"

#define itpm_cpld_smap_xil_1  	0x50000008	//RW
#define itpm_cpld_smap_xil_1_M	0xFFFFFFFF
#define itpm_cpld_smap_xil_1_R	0x0
#define itpm_cpld_smap_xil_1_D	"Missing Description"

#define itpm_cpld_smap_wr_fifo  	0x50001000	//RW
#define itpm_cpld_smap_wr_fifo_M	0xFFFFFFFF
#define itpm_cpld_smap_wr_fifo_R	0x0
#define itpm_cpld_smap_wr_fifo_D	"Missing Description"

#define itpm_cpld_flash_cmd  	0x60000000	//RW
#define itpm_cpld_flash_cmd_M	0xFFFFFFFF
#define itpm_cpld_flash_cmd_R	0x0
#define itpm_cpld_flash_cmd_D	"Missing Description"

#define itpm_cpld_flash_add  	0x60000004	//RW
#define itpm_cpld_flash_add_M	0xFFFFFFFF
#define itpm_cpld_flash_add_R	0x0
#define itpm_cpld_flash_add_D	"Missing Description"

#define itpm_cpld_flash_num  	0x60000008	//RW
#define itpm_cpld_flash_num_M	0xFFFFFFFF
#define itpm_cpld_flash_num_R	0x0
#define itpm_cpld_flash_num_D	"Missing Description"

#define itpm_cpld_flash_bank  	0x6000000C	//RW
#define itpm_cpld_flash_bank_M	0xFFFFFFFF
#define itpm_cpld_flash_bank_R	0x0
#define itpm_cpld_flash_bank_D	"Missing Description"

#define itpm_cpld_flash_busy  	0x60000010	//RW
#define itpm_cpld_flash_busy_M	0xFFFFFFFF
#define itpm_cpld_flash_busy_R	0x0
#define itpm_cpld_flash_busy_D	"Missing Description"

#define itpm_cpld_flash_wr_data  	0x60010000	//RW
#define itpm_cpld_flash_wr_data_M	0xFFFFFFFF
#define itpm_cpld_flash_wr_data_R	0x0
#define itpm_cpld_flash_wr_data_D	"Missing Description"

#define itpm_cpld_flash_rd_data  	0x60020000	//RW
#define itpm_cpld_flash_rd_data_M	0xFFFFFFFF
#define itpm_cpld_flash_rd_data_R	0x0
#define itpm_cpld_flash_rd_data_D	"Missing Description"

#define itpm_cpld_uart_ctrl_rnw  	0x70000000	//RW
#define itpm_cpld_uart_ctrl_rnw_M	0x00000001
#define itpm_cpld_uart_ctrl_rnw_R	0x0
#define itpm_cpld_uart_ctrl_rnw_D	"Read Not Write Command Execute"

#define itpm_cpld_uart_ctrl_txdata  	0x70000004	//RW
#define itpm_cpld_uart_ctrl_txdata_M	0x000000ff
#define itpm_cpld_uart_ctrl_txdata_R	0x0
#define itpm_cpld_uart_ctrl_txdata_D	"TX Data"

#define itpm_cpld_uart_ctrl_rxdata  	0x70000008	//R
#define itpm_cpld_uart_ctrl_rxdata_M	0x000000ff
#define itpm_cpld_uart_ctrl_rxdata_R	0x0
#define itpm_cpld_uart_ctrl_rxdata_D	"RX Data"

#define itpm_cpld_uart_ctrl_status  	0x7000000C	//R
#define itpm_cpld_uart_ctrl_status_M	0x00000003
#define itpm_cpld_uart_ctrl_status_R	0x0
#define itpm_cpld_uart_ctrl_status_D	"Status"

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
#define itpm_cpld_info_fpga1_base_add_D	"Missing Description"

#define itpm_cpld_info_fpga2_base_add  	0x80000010	//R
#define itpm_cpld_info_fpga2_base_add_M	0xFFFFFFFF
#define itpm_cpld_info_fpga2_base_add_R	0x0
#define itpm_cpld_info_fpga2_base_add_D	"Missing Description"

#define itpm_cpld_info_fw1_xml_offset  	0x80000014	//R
#define itpm_cpld_info_fw1_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_xml_offset_R	0x0
#define itpm_cpld_info_fw1_xml_offset_D	"Missing Description"

#define itpm_cpld_info_fw2_xml_offset  	0x80000018	//R
#define itpm_cpld_info_fw2_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_xml_offset_R	0x0
#define itpm_cpld_info_fw2_xml_offset_D	"Missing Description"

#define itpm_cpld_info_fw3_xml_offset  	0x8000001C	//R
#define itpm_cpld_info_fw3_xml_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_xml_offset_R	0x0
#define itpm_cpld_info_fw3_xml_offset_D	"Missing Description"

#define itpm_cpld_info_fw1_extended_info_offset  	0x80000020	//R
#define itpm_cpld_info_fw1_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_extended_info_offset_R	0x0
#define itpm_cpld_info_fw1_extended_info_offset_D	"Missing Description"

#define itpm_cpld_info_fw2_extended_info_offset  	0x80000024	//R
#define itpm_cpld_info_fw2_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_extended_info_offset_R	0x0
#define itpm_cpld_info_fw2_extended_info_offset_D	"Missing Description"

#define itpm_cpld_info_fw3_extended_info_offset  	0x80000028	//R
#define itpm_cpld_info_fw3_extended_info_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_extended_info_offset_R	0x0
#define itpm_cpld_info_fw3_extended_info_offset_D	"Missing Description"

#define itpm_cpld_info_fw1_offset  	0x8000002C	//R
#define itpm_cpld_info_fw1_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw1_offset_R	0x0
#define itpm_cpld_info_fw1_offset_D	"Missing Description"

#define itpm_cpld_info_fw2_offset  	0x80000030	//R
#define itpm_cpld_info_fw2_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw2_offset_R	0x0
#define itpm_cpld_info_fw2_offset_D	"Missing Description"

#define itpm_cpld_info_fw3_offset  	0x80000034	//R
#define itpm_cpld_info_fw3_offset_M	0xFFFFFFFF
#define itpm_cpld_info_fw3_offset_R	0x0
#define itpm_cpld_info_fw3_offset_D	"Missing Description"

#define itpm_cpld_bram_cpu  	0x90000000	//RW
#define itpm_cpld_bram_cpu_M	0xFFFFFFFF
#define itpm_cpld_bram_cpu_R	0x0
#define itpm_cpld_bram_cpu_D	"CPU BRAM"

#define itpm_cpld_confspi  	0xA0000000	//RW
#define itpm_cpld_confspi_M	0xFFFFFFFF
#define itpm_cpld_confspi_R	0x0
#define itpm_cpld_confspi_D	"SPI Configuration Data Buffer"

#define itpm_cpld_fespi_serio_reg0  	0xB0000000	//R
#define itpm_cpld_fespi_serio_reg0_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg0_R	0x0
#define itpm_cpld_fespi_serio_reg0_D	"Serial Data bits 31-0"

#define itpm_cpld_fespi_serio_reg1  	0xB0000004	//R
#define itpm_cpld_fespi_serio_reg1_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg1_R	0x0
#define itpm_cpld_fespi_serio_reg1_D	"Serial Data bits 63-32"

#define itpm_cpld_fespi_serio_reg2  	0xB0000008	//R
#define itpm_cpld_fespi_serio_reg2_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg2_R	0x0
#define itpm_cpld_fespi_serio_reg2_D	"Serial Data bits 95-64"

#define itpm_cpld_fespi_serio_reg3  	0xB000000C	//R
#define itpm_cpld_fespi_serio_reg3_M	0xFFFFFFFF
#define itpm_cpld_fespi_serio_reg3_R	0x0
#define itpm_cpld_fespi_serio_reg3_D	"Serial Data bits 127-96"

#define itpm_cpld_fespi_command  	0xB0000010	//RW
#define itpm_cpld_fespi_command_M	0xFFFFFFFF
#define itpm_cpld_fespi_command_R	0x0
#define itpm_cpld_fespi_command_D	"Serial Command and Mux"

#define itpm_cpld_fespi_command_mux  	0xB0000010	//RW
#define itpm_cpld_fespi_command_mux_M	0x00000004
#define itpm_cpld_fespi_command_mux_R	0x0
#define itpm_cpld_fespi_command_mux_D	"0x1 = Send to FE1- 0x0 = Send To FE0"

#define itpm_cpld_fespi_command_rw  	0xB0000010	//RW
#define itpm_cpld_fespi_command_rw_M	0x00000003
#define itpm_cpld_fespi_command_rw_R	0x0
#define itpm_cpld_fespi_command_rw_D	"0x1 = Read - 0x3 = Write"

#define itpm_cpld_intc_status  	0xC0000000	//R
#define itpm_cpld_intc_status_M	0x00000fff
#define itpm_cpld_intc_status_R	0x00000000
#define itpm_cpld_intc_status_D	"Interrupt Status"

#define itpm_cpld_intc_mask  	0xC0000004	//RW
#define itpm_cpld_intc_mask_M	0x00000fff
#define itpm_cpld_intc_mask_R	0x00000fff
#define itpm_cpld_intc_mask_D	"Interrupt Value"

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

