######################################################################
# System clock and reset
######################################################################
set_property PACKAGE_PIN H16 [get_ports clk_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clk_50m]

set_property PACKAGE_PIN R19 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

######################################################################
# PCM1808 I2S interface
######################################################################
set_property PACKAGE_PIN U12 [get_ports bck]
set_property IOSTANDARD LVCMOS33 [get_ports bck]

set_property PACKAGE_PIN T11 [get_ports lrck]
set_property IOSTANDARD LVCMOS33 [get_ports lrck]

set_property PACKAGE_PIN T10 [get_ports dout]
set_property IOSTANDARD LVCMOS33 [get_ports dout]

######################################################################
# UART interface
######################################################################
set_property PACKAGE_PIN T20 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]

set_property PACKAGE_PIN U20 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]
set_property PULLUP true [get_ports uart_tx]

######################################################################
# 74HC595 SPI control
######################################################################
set_property PACKAGE_PIN J18 [get_ports rclk_595]
set_property PACKAGE_PIN H18 [get_ports sclk_595]
set_property PACKAGE_PIN G17 [get_ports sdi_595]
set_property IOSTANDARD LVCMOS33 [get_ports rclk_595]
set_property IOSTANDARD LVCMOS33 [get_ports sclk_595]
set_property IOSTANDARD LVCMOS33 [get_ports sdi_595]

######################################################################
# LED indicators (4 bits)
######################################################################
set_property PACKAGE_PIN C20 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN G14 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

set_property PACKAGE_PIN H17 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

set_property PACKAGE_PIN B20 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

######################################################################
# Carrier debug output
######################################################################
set_property PACKAGE_PIN G15 [get_ports carrier]
set_property IOSTANDARD LVCMOS33 [get_ports carrier]
