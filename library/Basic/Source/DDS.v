`timescale 1 ns / 1 ns

/*
*   Date : 2024-07-02
*   Author : nitcloud
*   Module Name:   DDS.v - DDS
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 直接数字频率合成。
*   Dependencies: accuml + Sin
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module DDS #(
        // 输出信号位宽。
        parameter OUTPUT_WIDTH = 12,
        // 相位位宽。
        parameter PHASE_WIDTH  = 32
    ) (
        // 时钟输入
        input                       clock,
        // 异步复位，高电平有效
        input                       reset, 

        // 频率控制字。
        // (输出时钟频率 * 2^PHASE_WIDTH)/时钟频率
        input  [PHASE_WIDTH-1 : 0]  fre_word, 
        
        // 相位控制字。
        // (输出相位 * 2^PHASE_WIDTH) / (时钟频率 * 360°)
        input  [PHASE_WIDTH-1 : 0]  pha_word, 

        // 正弦波输出。
        output [OUTPUT_WIDTH-1 : 0] wave_sin,
        // 三角波输出。
        output [OUTPUT_WIDTH-1 : 0] wave_tri,
        // 锯齿波输出。
        output [OUTPUT_WIDTH-1 : 0] wave_saw
    ); 

    wire [PHASE_WIDTH-1:0] Q;

    accuml #(
        .WIDTH 		( PHASE_WIDTH 		))
    u_accuml(
        // 端口
        .clock   		( clock   		),
        .reset   		( reset   		),
        .clr     		( 1'b0     		),
        .add_sub 		( 1'b0  		),
        .D       		( fre_word      ),
        .Q       		( Q       		)
    );

    reg [PHASE_WIDTH-1:0] phase;
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            phase <= 0;
        end
        else begin
            phase <= Q + pha_word;
        end
    end

    reg [9:0] addr;
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            addr <= 0;
        end
        else begin
            addr <= phase[PHASE_WIDTH-1:PHASE_WIDTH-10];
        end
    end

    wire [15:0]	douta;
    Sin u_Sin(
        // 端口
        .clka  		( clock  		),
        .rsta  		( reset  		),
        .addra 		( addr  		),
        .douta 		( douta 		)
    );

    assign wave_sin = douta >>> (16-OUTPUT_WIDTH);
    assign wave_saw = phase[PHASE_WIDTH-1:PHASE_WIDTH-OUTPUT_WIDTH];
    assign wave_tri = wave_saw[OUTPUT_WIDTH-1] ? 
                      {{ wave_saw[OUTPUT_WIDTH-2]}, ~wave_saw[OUTPUT_WIDTH-3:0], 1'b0}: 
                      {{~wave_saw[OUTPUT_WIDTH-2]},  wave_saw[OUTPUT_WIDTH-3:0], 1'b0};

endmodule
