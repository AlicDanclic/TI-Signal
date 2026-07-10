`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   Mean.v - Mean
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 计算输入A和B的平均值。
*                 输入和输出均为有符号数。
*                 C = (A+B)/2
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '1010101010101'},
  {name: 'reset', wave: '10...........'},
  {name: 'ivalid', wave: '01...0.......'},
  {name: 'A', wave: 'x3.3.x.......', data: ['-31','11']},
  {name: 'B', wave: 'x3.3.x.......', data: ['11','21']},
  {name: 'ovalid', wave: '0....1.....0.'},
  {name: 'C', wave: 'x....5.5.x...', data: ['-10','16']}, 
]}
*/
module Mean #(
        parameter WIDTH = 16
    )(
        // 时钟输入
        input  clock,
        // 异步复位，高电平有效
        input  reset,
        // 输出符号选择。
        // 0 : 正常输出。
        // 1 : 反相输出。
        input  sign,
 
        // @Flow 输入有效
        input                ivalid,
        // @Flow 有符号A输入端口
        input  signed [WIDTH-1:0] A,
        // @Flow 有符号B输入端口
        input  signed [WIDTH-1:0] B,

        // @Flow 输出有效
        // 与C端口同步输出
        output               ovalid,
        // @Flow 有符号平均值C输出端口
        output signed [WIDTH-1:0] C
    );

    reg signed [WIDTH:0]   sum;
    reg signed [WIDTH-1:0] res;
    reg signed [WIDTH-1:0] cc;

    reg [1:0] sign_buf;
    reg [2:0] ovalid_buf;

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            sign_buf <= 0;
            ovalid_buf <= 0;
        end 
        else begin    
            sign_buf <= {sign_buf[0], sign};
            ovalid_buf <= {ovalid_buf[1:0], ivalid};
        end
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            sum <= 0;
            res <= 0;
            cc <= 0;
        end 
        else if (ivalid) begin
            sum <= A + B;
        end
        res <= sum >>> 1;
        cc <= sign_buf[1] ? ~res+1 : res;
    end

    assign ovalid = ovalid_buf[2];
    assign C = cc;

endmodule

