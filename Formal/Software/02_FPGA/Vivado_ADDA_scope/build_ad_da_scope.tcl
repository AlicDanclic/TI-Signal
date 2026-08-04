set project_file [file normalize [file join [file dirname [info script]] PhasedArrayUltrasonic.xpr]]

open_project $project_file
update_compile_order -fileset sources_1

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
puts "SYNTH_STATUS: $synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "Synthesis did not complete"
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set impl_status [get_property STATUS [get_runs impl_1]]
puts "IMPL_STATUS: $impl_status"
if {![string match "*Complete*" $impl_status]} {
    error "Implementation did not complete"
}

open_run impl_1
set report_dir [file normalize [file join [file dirname [info script]] reports_ad_da_scope]]
file mkdir $report_dir
report_timing_summary -delay_type min_max -check_timing_verbose \
    -max_paths 10 -input_pins -file [file join $report_dir timing_summary.rpt]
report_drc -file [file join $report_dir drc.rpt]
report_utilization -file [file join $report_dir utilization.rpt]

set setup_failures [get_timing_paths -quiet -delay_type max -slack_lesser_than 0.000 -max_paths 1]
set hold_failures [get_timing_paths -quiet -delay_type min -slack_lesser_than 0.000 -max_paths 1]
if {[llength $setup_failures] != 0 || [llength $hold_failures] != 0} {
    error "Implemented design has negative timing slack"
}

set bit_file [file normalize [file join [get_property DIRECTORY [get_runs impl_1]] top_ad_da_uart.bit]]
if {![file exists $bit_file]} {
    error "Bitstream was not generated"
}
puts "BITSTREAM_FILE: $bit_file"
close_project
