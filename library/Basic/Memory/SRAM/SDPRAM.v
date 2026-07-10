`timescale 1ns/1ps

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   SDPRAM.v - SDPRAM
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 
*                 1. 在Vivado中可直接综合为BRAM。
*                 2. 当ren禁用时，输出最后的数据。
*                 3. 写模式下，当前数据输入优先写入，
*                    读取前一周期地址输入的数据。
*                    读模式下，直接读取当前周期地址输入的数据。
*                    写模式下写入不同地址时，
*                    直接读取当前周期地址输入对应的数据。
*                 4. 复位为同步复位高电平有效，也可作为
*                    SDPRAM的使能输入，低电平有效。
*   Dependencies: none(FPGA) auto for BRAM in vivado | RAM_IP with IC 
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '101010101010101'},
  {name: 'wen', wave: '01.........0...'},
  {name: 'ren', wave: '01.......0.1.0.'},
  {name: 'waddr', wave: 'x3...3.....3.x.', data: ['addr0','addr1','addr2']},
  {name: 'raddr', wave: 'x3...3.3.3.3.x.', data: ['addr0','addr2','addr0','addr1','addr0']},
  {name: 'din', wave: 'x4.4.4...4.4.x.', data: ['data0','data1','data2','data3','data4']},
  {name: 'dout', wave: 'x..5.5.5.....x.', data: ['data0','data2','data1']},
]}
*/
module SDPRAM #(
        // The depth parameter of RAM.
        parameter DEPTH = 1024,
        // The width parameter for reading and writing data.
        parameter WIDTH = 12,
        // do not set by user
        parameter STEP  = ($clog2(DEPTH) == 0) ? 0 : ($clog2(DEPTH) - 1)
    ) (
        // 时钟输入
        input      clock,
        // Synchronous reset with active high
        // When DEPTH == 1; asynchronous reset with active high
        input      reset,

        // Write enable input, active high.
        input      wen,  
        // Read enable input, active high.
        input      ren,

        // Write input address.
        input  [STEP:0] waddr,
        // Read input address.
        input  [STEP:0] raddr,

        // Write input data.
        input  [WIDTH-1:0] din,
        // Read output data.
        output [WIDTH-1:0] dout
    );

    reg [WIDTH-1:0] rdout;

    // define ram as array or ip
    reg [WIDTH-1:0] ram [DEPTH-1:0];
    integer i;
    initial begin
        for(i=0; i<DEPTH; i=i+1) begin
            ram[i] = 0;
        end
    end
    
    always @(posedge clock) begin
        if(reset) begin
            rdout <= 0;
        end
        else begin
            if (wen) begin
                ram[waddr] <= din;
            end
            if (ren) begin
                rdout <= ram[raddr];
            end
        end
    end

    assign dout = rdout;

endmodule //
