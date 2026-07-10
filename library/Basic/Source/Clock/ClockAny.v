`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   ClockAny.v - ClockAny
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 任意频率发生器模块。
*   Dependencies: accuml
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module ClockAny # (
        // 相位位宽。
        parameter PHASE_WIDTH = 32
    ) (
        // 时钟输入
        input                    clock,
        // 异步复位，高电平有效
        input                    reset,

        // 频率控制字。
        // (输出时钟频率 * 2^PHASE_WIDTH)/时钟频率
        input  [PHASE_WIDTH-1:0] sample_fre,
        // 输出时钟信号
        output		             sample_clk
    );

    wire [PHASE_WIDTH-1:0]	Q;

    accuml #(
        .WIDTH 		( PHASE_WIDTH ))
    u_accuml(
        // 端口
        .clock   		( clock      ),
        .reset   		( reset      ),
        .clr     		( 1'b0       ),
        .add_sub 		( 1'b0       ),
        .D       		( sample_fre ),
        .Q       		( Q          )
    );

    assign sample_clk = Q[PHASE_WIDTH-1];

endmodule
