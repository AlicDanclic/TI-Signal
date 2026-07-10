`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   DPRAM.v - DPRAM
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 同步双端口SRAM，A、B端口可访问同一存储位置。
*                 两个端口均可独立读写存储器阵列。
*                 1. 在Vivado中可直接综合为BRAM。
*                 2. 使能时持续输出数据，非使能时输出最后数据。
*                 3. 当A、B端口同时向同一地址写数据时，
*                    B端口的写操作优先。
*                 4. 写模式下，当前数据输入优先写入，
*                    读取前一周期地址输入的数据。
*                    读模式下，直接读取当前周期地址输入的数据。
*                    写模式下写入不同地址时，
*                    直接读取当前周期地址输入对应的数据。
*   Dependencies: none(FPGA) auto for BRAM in vivado | RAM_IP with IC 
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clka/b', wave: '101010101'},
  {name: 'ena/b', wave: '01...0...'},
  {name: 'wea/b', wave: '01...0...'},
  {name: 'addra/b', wave: 'x3...3.x.', data: ['addr0','addr2']},
  {name: 'dina/b', wave: 'x4.4.x...', data: ['data0','data1']},
  {name: 'douta/b', wave: 'x..5.5.x.', data: ['data0','data2']},
]}
*/
module DPRAM #(
        // 读写数据位宽。
        parameter WIDTH = 16,
        // RAM深度。
        parameter DEPTH = 1024
    ) (
        // A端口时钟输入
        input                       clka,
        // A端口使能，高电平有效
        input                       ena,
        // A端口写使能，高电平有效
        input                       wea,
        // A端口地址输入
        input [$clog2(DEPTH)-1:0]   addra,
        // A端口数据输入
        input [WIDTH-1:0]           dina,
        // A端口数据输出
        output reg [WIDTH-1:0]      douta,

        // B端口时钟输入
        input                       clkb,
        // B端口使能，高电平有效
        input                       enb,
        // B端口写使能，高电平有效
        input                       web,
        // B端口地址输入
        input [$clog2(DEPTH)-1:0]   addrb,
        // B端口数据输入
        input [WIDTH-1:0]           dinb,
        // B端口数据输出
        output reg [WIDTH-1:0]      doutb
    );

    reg [WIDTH - 1 : 0] ram [DEPTH - 1 : 0];
    integer i;
    initial begin
        for(i=0;i<DEPTH;i=i+1) begin
            ram[i] <= 0;
        end
        douta <= 0;
        doutb <= 0;
    end

    always @(posedge clka) begin
        if (ena) begin
            if (wea) begin
                ram[addra] <= dina;
            end
            douta <= ram[addra];
        end
    end

    always @(posedge clkb) begin
        if (enb) begin
            if (web) begin
                ram[addrb] <= dinb;
            end
            doutb <= ram[addrb];
        end
    end
    
endmodule
