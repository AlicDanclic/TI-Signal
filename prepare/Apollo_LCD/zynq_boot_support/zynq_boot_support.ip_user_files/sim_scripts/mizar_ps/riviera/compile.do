transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib riviera/xilinx_vip
vlib riviera/axi_infrastructure_v1_1_0
vlib riviera/axi_vip_v1_1_21
vlib riviera/processing_system7_vip_v1_0_23
vlib riviera/xil_defaultlib

vmap xilinx_vip riviera/xilinx_vip
vmap axi_infrastructure_v1_1_0 riviera/axi_infrastructure_v1_1_0
vmap axi_vip_v1_1_21 riviera/axi_vip_v1_1_21
vmap processing_system7_vip_v1_0_23 riviera/processing_system7_vip_v1_0_23
vmap xil_defaultlib riviera/xil_defaultlib

vlog -work xilinx_vip  -incr "+incdir+E:/Vitis2/2025.1/Vivado/data/xilinx_vip/include" -l xilinx_vip -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_21 -l processing_system7_vip_v1_0_23 -l xil_defaultlib \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi4stream_vip_axi4streampc.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi_vip_axi4pc.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/xil_common_vip_pkg.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi4stream_vip_pkg.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi_vip_pkg.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi4stream_vip_if.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/axi_vip_if.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/clk_vip_if.sv" \
"E:/Vitis2/2025.1/Vivado/data/xilinx_vip/hdl/rst_vip_if.sv" \

vlog -work axi_infrastructure_v1_1_0  -incr -v2k5 "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/ec67/hdl" "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/6cfa/hdl" "+incdir+../../../../../../Vitis2/2025.1/Vivado/data/rsb/busdef" "+incdir+E:/Vitis2/2025.1/Vivado/data/xilinx_vip/include" -l xilinx_vip -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_21 -l processing_system7_vip_v1_0_23 -l xil_defaultlib \
"../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/ec67/hdl/axi_infrastructure_v1_1_vl_rfs.v" \

vlog -work axi_vip_v1_1_21  -incr "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/ec67/hdl" "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/6cfa/hdl" "+incdir+../../../../../../Vitis2/2025.1/Vivado/data/rsb/busdef" "+incdir+E:/Vitis2/2025.1/Vivado/data/xilinx_vip/include" -l xilinx_vip -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_21 -l processing_system7_vip_v1_0_23 -l xil_defaultlib \
"../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/f16f/hdl/axi_vip_v1_1_vl_rfs.sv" \

vlog -work processing_system7_vip_v1_0_23  -incr "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/ec67/hdl" "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/6cfa/hdl" "+incdir+../../../../../../Vitis2/2025.1/Vivado/data/rsb/busdef" "+incdir+E:/Vitis2/2025.1/Vivado/data/xilinx_vip/include" -l xilinx_vip -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_21 -l processing_system7_vip_v1_0_23 -l xil_defaultlib \
"../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/6cfa/hdl/processing_system7_vip_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr -v2k5 "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/ec67/hdl" "+incdir+../../../../zynq_boot_support.gen/sources_1/bd/mizar_ps/ipshared/6cfa/hdl" "+incdir+../../../../../../Vitis2/2025.1/Vivado/data/rsb/busdef" "+incdir+E:/Vitis2/2025.1/Vivado/data/xilinx_vip/include" -l xilinx_vip -l axi_infrastructure_v1_1_0 -l axi_vip_v1_1_21 -l processing_system7_vip_v1_0_23 -l xil_defaultlib \
"../../../bd/mizar_ps/ip/mizar_ps_processing_system7_0_0/sim/mizar_ps_processing_system7_0_0.v" \
"../../../bd/mizar_ps/sim/mizar_ps.v" \

vlog -work xil_defaultlib \
"glbl.v"

