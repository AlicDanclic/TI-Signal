transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

asim +access +r +m+mizar_ps  -L xil_defaultlib -L xilinx_vip -L axi_infrastructure_v1_1_0 -L axi_vip_v1_1_21 -L processing_system7_vip_v1_0_23 -L xilinx_vip -L unisims_ver -L unimacro_ver -L secureip -O5 xil_defaultlib.mizar_ps xil_defaultlib.glbl

do {mizar_ps.udo}

run 1000ns

endsim

quit -force
