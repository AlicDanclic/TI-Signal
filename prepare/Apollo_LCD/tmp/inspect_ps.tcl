open_project E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.xpr
open_bd_design E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.srcs/sources_1/bd/mizar_ps/mizar_ps.bd
set ps [get_bd_cells -hierarchical -filter {VLNV =~ *processing_system7*}]
puts "PS_CELL=$ps"
if {$ps ne ""} {
    foreach p [list_property $ps] {
        if {[string match *SD* $p] || [string match *MIO* $p] || [string match *DDR* $p]} {
            if {[catch {get_property $p $ps} v] == 0} {
                puts "$p=$v"
            }
        }
    }
}
close_project
