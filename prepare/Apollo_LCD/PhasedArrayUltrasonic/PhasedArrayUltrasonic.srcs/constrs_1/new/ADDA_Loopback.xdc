###############################################################################
# Mizar Z7 system clock and reset
###############################################################################
set_property PACKAGE_PIN H16 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]
create_clock -name sys_clk -period 20.000 -waveform {0.000 10.000} [get_ports clk_50m]

set_property PACKAGE_PIN R19 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]
set_false_path -from [get_ports rst_n]

###############################################################################
# 3PA1030 dual 10-bit ADC adapter on JP2
###############################################################################
set_property PACKAGE_PIN J18 [get_ports {ad_ch1_data[0]}]
set_property PACKAGE_PIN H18 [get_ports {ad_ch1_data[1]}]
set_property PACKAGE_PIN G17 [get_ports {ad_ch1_data[2]}]
set_property PACKAGE_PIN G18 [get_ports {ad_ch1_data[3]}]
set_property PACKAGE_PIN K14 [get_ports {ad_ch1_data[4]}]
set_property PACKAGE_PIN J14 [get_ports {ad_ch1_data[5]}]
set_property PACKAGE_PIN H15 [get_ports {ad_ch1_data[6]}]
set_property PACKAGE_PIN G15 [get_ports {ad_ch1_data[7]}]
set_property PACKAGE_PIN J20 [get_ports {ad_ch1_data[8]}]
set_property PACKAGE_PIN H20 [get_ports {ad_ch1_data[9]}]
set_property PACKAGE_PIN L14 [get_ports ad_clk1]
set_property PACKAGE_PIN L15 [get_ports ad_oe1]

set_property PACKAGE_PIN K19 [get_ports {ad_ch2_data[0]}]
set_property PACKAGE_PIN J19 [get_ports {ad_ch2_data[1]}]
set_property PACKAGE_PIN K16 [get_ports {ad_ch2_data[2]}]
set_property PACKAGE_PIN J16 [get_ports {ad_ch2_data[3]}]
set_property PACKAGE_PIN L19 [get_ports {ad_ch2_data[4]}]
set_property PACKAGE_PIN L20 [get_ports {ad_ch2_data[5]}]
set_property PACKAGE_PIN L16 [get_ports {ad_ch2_data[6]}]
set_property PACKAGE_PIN L17 [get_ports {ad_ch2_data[7]}]
set_property PACKAGE_PIN M14 [get_ports {ad_ch2_data[8]}]
set_property PACKAGE_PIN M15 [get_ports {ad_ch2_data[9]}]
set_property PACKAGE_PIN N15 [get_ports ad_clk2]
set_property PACKAGE_PIN N16 [get_ports ad_oe2]

set_property IOSTANDARD LVCMOS33 [get_ports {ad_ch1_data[*] ad_ch2_data[*] ad_clk1 ad_clk2 ad_oe1 ad_oe2}]

###############################################################################
# 3PD5651E dual 10-bit DAC adapter on JP1
###############################################################################
set_property PACKAGE_PIN N18 [get_ports da_clk1]
set_property PACKAGE_PIN P19 [get_ports {da_ch1_data[9]}]
set_property PACKAGE_PIN N17 [get_ports {da_ch1_data[8]}]
set_property PACKAGE_PIN P18 [get_ports {da_ch1_data[7]}]
set_property PACKAGE_PIN N20 [get_ports {da_ch1_data[6]}]
set_property PACKAGE_PIN P20 [get_ports {da_ch1_data[5]}]
set_property PACKAGE_PIN T17 [get_ports {da_ch1_data[4]}]
set_property PACKAGE_PIN R18 [get_ports {da_ch1_data[3]}]
set_property PACKAGE_PIN T20 [get_ports {da_ch1_data[2]}]
set_property PACKAGE_PIN U20 [get_ports {da_ch1_data[1]}]
set_property PACKAGE_PIN V20 [get_ports {da_ch1_data[0]}]

set_property PACKAGE_PIN W20 [get_ports da_clk2]
set_property PACKAGE_PIN Y18 [get_ports {da_ch2_data[9]}]
set_property PACKAGE_PIN Y19 [get_ports {da_ch2_data[8]}]
set_property PACKAGE_PIN Y16 [get_ports {da_ch2_data[7]}]
set_property PACKAGE_PIN Y17 [get_ports {da_ch2_data[6]}]
set_property PACKAGE_PIN W18 [get_ports {da_ch2_data[5]}]
set_property PACKAGE_PIN W19 [get_ports {da_ch2_data[4]}]
set_property PACKAGE_PIN U18 [get_ports {da_ch2_data[3]}]
set_property PACKAGE_PIN U19 [get_ports {da_ch2_data[2]}]
set_property PACKAGE_PIN V16 [get_ports {da_ch2_data[1]}]
set_property PACKAGE_PIN W16 [get_ports {da_ch2_data[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {da_ch1_data[*] da_ch2_data[*] da_clk1 da_clk2}]
set_property DRIVE 8 [get_ports {da_ch1_data[*] da_ch2_data[*] da_clk1 da_clk2 ad_clk1 ad_clk2 ad_oe1 ad_oe2}]
set_property SLEW FAST [get_ports {da_ch1_data[*] da_ch2_data[*] da_clk1 da_clk2 ad_clk1 ad_clk2}]

###############################################################################
# UART TX to STM32, Mizar JP1 pin 25
###############################################################################
set_property PACKAGE_PIN V15 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property DRIVE 8 [get_ports uart_tx]
set_property SLEW SLOW [get_ports uart_tx]
set_property PULLUP true [get_ports uart_tx]

###############################################################################
# Active-low board LEDs
###############################################################################
set_property PACKAGE_PIN C20 [get_ports {led[0]}]
set_property PACKAGE_PIN G14 [get_ports {led[1]}]
set_property PACKAGE_PIN H17 [get_ports {led[2]}]
set_property PACKAGE_PIN B20 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

###############################################################################
# Source-synchronous timing intent
###############################################################################
create_generated_clock -name ad_clk1_fwd -source [get_ports clk_50m] -divide_by 1 [get_ports ad_clk1]
create_generated_clock -name ad_clk2_fwd -source [get_ports clk_50m] -divide_by 1 [get_ports ad_clk2]
create_generated_clock -name da_clk1_fwd -source [get_ports clk_50m] -divide_by 1 [get_ports da_clk1]
create_generated_clock -name da_clk2_fwd -source [get_ports clk_50m] -divide_by 1 [get_ports da_clk2]

# Conservative board-plus-converter delays. ADC data is captured on the falling
# system-clock edge; DAC data changes after that edge and is sampled externally
# on the next forwarded rising edge.
set_input_delay -clock ad_clk1_fwd -min 1.000 [get_ports {ad_ch1_data[*]}]
set_input_delay -clock ad_clk1_fwd -max 8.000 [get_ports {ad_ch1_data[*]}]
set_input_delay -clock ad_clk2_fwd -min 1.000 [get_ports {ad_ch2_data[*]}]
set_input_delay -clock ad_clk2_fwd -max 8.000 [get_ports {ad_ch2_data[*]}]

# The 3PA1030 output has a pipeline/data-valid delay longer than one 20 ns
# period. Each falling-edge register may therefore retain the previous word;
# the newly launched word is required at the following falling edge.
set_multicycle_path 2 -setup -from [get_clocks ad_clk1_fwd] -to [get_clocks sys_clk]
set_multicycle_path 1 -hold  -from [get_clocks ad_clk1_fwd] -to [get_clocks sys_clk]
set_multicycle_path 2 -setup -from [get_clocks ad_clk2_fwd] -to [get_clocks sys_clk]
set_multicycle_path 1 -hold  -from [get_clocks ad_clk2_fwd] -to [get_clocks sys_clk]

set_output_delay -clock da_clk1_fwd -min -1.000 [get_ports {da_ch1_data[*]}]
set_output_delay -clock da_clk1_fwd -max 3.000 [get_ports {da_ch1_data[*]}]
set_output_delay -clock da_clk2_fwd -min -1.000 [get_ports {da_ch2_data[*]}]
set_output_delay -clock da_clk2_fwd -max 3.000 [get_ports {da_ch2_data[*]}]
