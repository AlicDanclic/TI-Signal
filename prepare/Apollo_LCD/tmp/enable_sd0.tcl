open_project E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.xpr
open_bd_design E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.srcs/sources_1/bd/mizar_ps/mizar_ps.bd
set ps [get_bd_cells -hierarchical -filter {VLNV =~ *processing_system7*}]
if {$ps eq ""} { error "processing_system7 cell not found" }
set_property -dict [list \
    CONFIG.PCW_SD0_PERIPHERAL_ENABLE {1} \
    CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45} \
    CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ {50} \
    CONFIG.PCW_SDIO_PERIPHERAL_VALID {1} \
] $ps
puts "SD0_ENABLE=[get_property CONFIG.PCW_SD0_PERIPHERAL_ENABLE $ps]"
puts "SD0_IO=[get_property CONFIG.PCW_SD0_SD0_IO $ps]"
puts "SDIO_FREQ=[get_property CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ $ps]"
save_bd_design
close_project
