`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   shiftSets.v - shiftSets
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 
*                 1. 在Vivado中可直接综合为BRAM。
*   Dependencies: none(FPGA) auto for BRAM in vivado | RAM_IP with IC 
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '101010101010101010101'},
  {name: 'case1:ivalid', wave: '01...................'},
  {name: 'case2:ivalid', wave: '101010101010101010101'},
  {name: 'idata', wave: 'x4.4.4.4.4.x.........', data: ['data0','data1','data2','data3','data4']},
  {name: 'ivalid', wave: '0........1...........'},
  {name: 'odata', wave: 'x........5.5.5.5.5.x.', data: ['data0','data1','data2','data3','data4']},
]}
*/
module shiftSets #(
        // 数据位宽。
        parameter WIDTH = 16,
        // 延迟或移位长度位宽。
        parameter DEEPW = 8
    ) (
        // 时钟输入
        input                   clock,
        // 异步复位，高电平有效
        input                   reset,

        // @Flow 输入有效
        input                   ivalid, 
        // @Flow 输入移位数据
        input  [WIDTH - 1 : 0]  shiftin,
        input  [DEEPW - 1 : 0]  delay,
 
        // @Flow 输出数据 
        output [WIDTH - 1 : 0]  shiftout
    );

    reg [DEEPW:0]  count;

    SDPRAM #(
        .WIDTH 		( WIDTH 		),
        .DEPTH 		( 1<<DEEPW  	))
    u_SDPRAM(
        //端口
        .clock  	( clock  		),
        .reset   	( reset   		),

        .wen   		( ivalid   		),
        .ren        ( 1'b1          ),

        .waddr 		( count 		),
        .raddr 		( count 		),

        .din  		( shiftin  		),
        .dout 		( shiftout 		)
    );

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            count <= 0;
        end 
        else begin            
            if (ivalid) begin
                count <= count + 1;
                if (count == (delay-2)) begin
                    count <= 0;
                end
            end
        end
    end
    
endmodule

