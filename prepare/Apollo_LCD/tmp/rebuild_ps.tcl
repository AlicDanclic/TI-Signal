open_project E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.xpr
open_bd_design E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.srcs/sources_1/bd/mizar_ps/mizar_ps.bd
validate_bd_design
generate_target all [get_files E:/Apollo_LCD/zynq_boot_support/zynq_boot_support.srcs/sources_1/bd/mizar_ps/mizar_ps.bd]
update_compile_order -fileset sources_1
set synth [get_runs synth_1]
set impl [get_runs impl_1]
puts "SYNTH_STATUS=[get_property STATUS $synth]"
puts "IMPL_STATUS=[get_property STATUS $impl]"
reset_run $synth
reset_run $impl
launch_runs $synth -jobs 4
wait_on_run $synth
if {[get_property STATUS $synth] ne "synth_design Complete!"} { error "synthesis failed: [get_property STATUS $synth]" }
launch_runs $impl -to_step write_bitstream -jobs 4
wait_on_run $impl
if {[get_property STATUS $impl] ne "write_bitstream Complete!"} { error "implementation failed: [get_property STATUS $impl]" }
open_run $impl
write_hw_platform -fixed -include_bit -force E:/Apollo_LCD/zynq_boot_support/mizar_ps_wrapper_sd.xsa
close_project
