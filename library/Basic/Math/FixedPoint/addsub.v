`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   addsub.v - addsub
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 4级流水线加减器。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '10101010101010101'},
  {name: 'reset', wave: '10...............'},
  {name: 'clr', wave: '01.0.............'},
  {name: 'idata', wave: 'x3...............', data: ['5']},
  {name: 'odata', wave: 'x........5.5.5.5.', data: ['5','10','25','30']}, 
]}
*/
module addsub #(
        // 数据位宽。
        parameter WIDTH = 16
    ) (
        // 时钟输入
        input  clock,
        // 异步复位，高电平有效
        input  reset,

        // 加减法类型选择信号。
        // 0 : 加法
        // 1 : 减法
        input  add_sub,

        // 加减器的输入数据。
        input  [WIDTH-1:0] A,
        input  [WIDTH-1:0] B,

        // 加减器的输出。
        output [WIDTH-1:0] S
    );

    
endmodule