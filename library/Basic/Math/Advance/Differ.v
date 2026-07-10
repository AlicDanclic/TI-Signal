`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   Differ.v - Differ
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 简单的定点差分计算，
*                 计算两个有效时间点之间的差值。
*                 输入和输出均为有符号数。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '1010101010101'},
  {name: 'reset', wave: '10...........'},
  {name: 'ivalid', wave: '01.......0...'},
  {name: 'idata', wave: 'x3.3.3.3.x...', data: ['5','10','7','0']},
  {name: 'ovalid', wave: '0..1.......0.'},
  {name: 'odata', wave: 'x..5.5.5.5.x.', data: ['x','5','-3','-7']}, 
]}
*/
module Differ #(
        // 输入数据位宽。
        parameter WIDTH = 12,
        // 输出模式设置：
        // 0 : 截取高WIDTH位输出
        // 1 : 截取低WIDTH位输出。
        parameter OMODE = 1
    ) (
        // 时钟输入
        input  clock,
        // 异步复位，高电平有效
        input  reset,
        
        // @Flow 输入有效
        input                     ivalid,
        // @Flow 有符号输入端口
        input  signed [WIDTH-1:0] idata,
        
        // @Flow 输出有效
        // 与odata端口同步输出
        output                    ovalid,
        // @Flow 有符号模值输出
        output signed [WIDTH-1:0] odata
    );

    reg                    ovalid_reg;
    reg signed [WIDTH-1:0] odata_reg;
    reg signed [WIDTH:0]   odata_buf;
    always @(posedge clock) begin
        if(reset) begin
            odata_reg <= 0;
            odata_buf <= 0;
            ovalid_reg <= 0;
        end
        else begin
            if (ivalid) begin
                odata_reg <= idata;
                odata_buf <= idata - odata_reg;
            end
            ovalid_reg <= ivalid;
        end
    end

    assign odata = OMODE ? odata_buf[WIDTH-1:0] : odata_buf[WIDTH:1];
    assign ovalid = ovalid_reg;
    
endmodule  //Differ
