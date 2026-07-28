set root_dir [file normalize [file dirname [info script]]]
set source_dir [file join $root_dir PhasedArrayUltrasonic.srcs sources_1 new]
set constraint_file [file join $root_dir PhasedArrayUltrasonic.srcs constrs_1 new ADDA_Loopback.xdc]
set report_dir [file join $root_dir reports_ad_da_scope]
set checkpoint_dir [file join $root_dir standalone_ad_da_scope]
set bit_file [file join $root_dir PhasedArrayUltrasonic.runs impl_1 top_ad_da_uart.bit]

file mkdir $report_dir
file mkdir $checkpoint_dir

create_project -in_memory -part xc7z020clg400-2
read_verilog [list \
    [file join $source_dir top.v] \
    [file join $source_dir adda_data_path.v] \
    [file join $source_dir adc_monitor_capture.v] \
    [file join $source_dir wave_uart_frame_tx.v] \
    [file join $source_dir uart_tx.v]]
read_xdc $constraint_file

synth_design -top top_ad_da_uart -part xc7z020clg400-2
write_checkpoint -force [file join $checkpoint_dir post_synth.dcp]

opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $checkpoint_dir post_route.dcp]

report_timing_summary -delay_type min_max -check_timing_verbose \
    -max_paths 10 -input_pins -file [file join $report_dir timing_summary.rpt]
report_drc -file [file join $report_dir drc.rpt]
report_utilization -file [file join $report_dir utilization.rpt]

set setup_failures [get_timing_paths -quiet -delay_type max \
    -slack_lesser_than 0.000 -max_paths 1]
set hold_failures [get_timing_paths -quiet -delay_type min \
    -slack_lesser_than 0.000 -max_paths 1]
if {[llength $setup_failures] != 0 || [llength $hold_failures] != 0} {
    error "Implemented design has negative timing slack"
}

write_bitstream -force $bit_file
if {![file exists $bit_file]} {
    error "Bitstream was not generated"
}
puts "BITSTREAM_FILE: $bit_file"
close_project
